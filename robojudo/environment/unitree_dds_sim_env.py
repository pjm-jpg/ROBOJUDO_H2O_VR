"""Actual UnitreeEnv lowcmd/lowstate transport with local simulator lifecycle."""
import json
import time
import uuid

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

from robojudo.environment import env_registry
from robojudo.environment.unitree_env import UnitreeEnv
from robojudo.environment.utils.dds_sim import CONTROL_TOPIC, STATUS_TOPIC, TARGET_TOPIC, decode_status, initialize_sim_dds


@env_registry.register
class UnitreeDDSSimEnv(UnitreeEnv):
    def __init__(self, cfg_env, device='cpu'):
        # Revalidate even if callers mutated a previously validated config.
        from robojudo.config.h1.env.h1_dds_sim_env_cfg import H1DDSSimEnvCfg
        cfg_env = H1DDSSimEnvCfg.model_validate(cfg_env.model_dump())
        self._sim_status = None
        self._sim_status_received = 0
        self._sim_active = False
        self._seen_timeouts = 0
        self.viewer = None
        self._initialize_sim(cfg_env.unitree.domain_id)
        try:
            super().__init__(cfg_env, device)
        except Exception:
            self._status_sub.Close()
            self._control_pub.Close()
            self._target_pub.Close()
            raise

    def _initialize_sim(self, domain):
        initialize_sim_dds(domain)
        self._control_pub = ChannelPublisher(CONTROL_TOPIC, String_)
        self._control_pub.Init()
        self._target_pub = ChannelPublisher(TARGET_TOPIC, String_)
        self._target_pub.Init()
        self._status_sub = ChannelSubscriber(STATUS_TOPIC, String_)
        self._status_sub.Init(self._status_handler, 10)

    def _initialize_channel(self):
        initialize_sim_dds(self.cfg_env.unitree.domain_id)

    def _status_handler(self, msg):
        try:
            self._sim_status = decode_status(msg)
            self._sim_status_received = time.monotonic()
        except (ValueError, TypeError, KeyError):
            pass

    def _lifecycle(self, action):
        request = uuid.uuid4().hex
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            status = self._sim_status
            if status and time.monotonic() - self._sim_status_received < .5:
                if status.get('request_id') == request:
                    return
                self._control_pub.Write(String_(json.dumps(dict(
                    action=action, request_id=request, server_id=status['server_id']))))
            time.sleep(.02)
        raise TimeoutError(f'MuJoCo lifecycle acknowledgement timed out: {action}')

    def reborn(self):
        self._lifecycle('reset')
        self._sim_active = False

    def set_sim_active(self, active):
        if active != self._sim_active:
            self._lifecycle('resume' if active else 'pause')
            self._sim_active = active

    def consume_sim_fault(self):
        status = self._sim_status or {}
        count = status.get('command_timeout_count', 0)
        if count > self._seen_timeouts:
            self._seen_timeouts = count
            self._sim_active = False
            return 'DDS lowcmd timeout; E required to resume'
        return None

    def publish_targets(self, references, base_pos):
        status = self._sim_status
        if not status or time.monotonic() - self._sim_status_received > .5:
            return
        self._target_pub.Write(String_(json.dumps(dict(
            server_id=status['server_id'], order=['left', 'right', 'head'],
            targets_m=np.asarray(references['ref_body_pos_subset']).tolist(),
            raw_targets_m=(np.asarray(references['raw_ref_body_pos_subset']).tolist()
                           if 'stationary_status' in references else None),
            stationary_status=references.get('stationary_status'),
            base_pos_m=np.asarray(base_pos).tolist()), allow_nan=False)))

    def shutdown(self):
        try:
            self._lifecycle('pause')
        except (TimeoutError, OSError):
            pass
        try:
            super().shutdown()
        finally:
            self._status_sub.Close()
            self._control_pub.Close()
            self._target_pub.Close()
