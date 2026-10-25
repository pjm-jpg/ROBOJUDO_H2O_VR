"""H1 state inspection: subscriber only, never creates a command publisher."""
import json
import threading
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

from robojudo.config.h1.env.h1_real_env_cfg import H1RealEnvCfg


def valid_h1_state(message):
    if list(message.head) != [0xFE, 0xEF] or message.crc != CRC().Crc(message):
        return False
    q = np.asarray(message.imu_state.quaternion)
    values = [v for motor in message.motor_state for v in (motor.q, motor.dq)]
    return bool(np.isfinite(values).all() and np.isfinite(q).all()
                and abs(np.linalg.norm(q) - 1) < .02
                and np.isfinite(message.imu_state.gyroscope).all())


def inspect_h1_state(interface='enp3s0', duration=10, domain=0):
    ChannelFactoryInitialize(domain, interface)
    lock = threading.Lock()
    latest = None
    count = rejected = 0
    last = first = None
    gaps = []

    def receive(message):
        nonlocal latest, count, rejected, first, last
        with lock:
            if not valid_h1_state(message):
                rejected += 1
                return
            now = time.monotonic()
            if last is not None:
                gaps.append(now - last)
            else:
                first = now
            latest, last, count = message, now, count + 1

    sub = ChannelSubscriber('rt/lowstate', LowState_)
    sub.Init(receive, 10)
    start = time.monotonic()
    cfg = H1RealEnvCfg()
    try:
        while time.monotonic() - start < duration:
            time.sleep(.5)
            with lock:
                if latest is None:
                    print(f'Waiting for valid H1 lowstate; rejected={rejected}', flush=True)
                else:
                    joints = [round(latest.motor_state[i].q, 4) for i in cfg.joint2motor_idx]
                    print(json.dumps(dict(frames=count, rejected=rejected,
                                          age_s=round(time.monotonic()-last, 3), joint_q=joints,
                                          imu_wxyz=latest.imu_state.quaternion)), flush=True)
        with lock:
            age = None if last is None else time.monotonic() - last
            report = dict(frames=count, rejected=rejected, last_state_age_s=age,
                          received_hz=(count-1)/(last-first) if count > 1 else 0,
                          max_gap_s=max(gaps, default=0), command_messages_sent=0,
                          passed=count > 1 and age < .1 and rejected == 0 and max(gaps, default=0) < .1)
        print(json.dumps(report, indent=2), flush=True)
        return report
    finally:
        sub.Close()
