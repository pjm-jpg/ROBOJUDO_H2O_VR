"""Stationary task invariants, including saturated velocity and retreat recovery."""
import math
from types import SimpleNamespace
import json
import sys
import threading
import unittest
from unittest.mock import Mock, patch

import numpy as np
from pydantic import ValidationError

from robojudo.policy.utils.stationary_reference import StationaryReferenceCfg, StationaryReferenceGuard
from robojudo.environment.h1_dds_bridge import H1DDSBridge
from robojudo.pipeline.quest_h2h_deploy_pipeline import QuestH2HDeployPipeline
from scripts import run_pipeline, run_quest_h2h, run_quest_h2h_real


ANCHOR = np.array([[.25,.25,1.1], [.25,-.25,1.1], [0,0,1.6]])


def observation(yaw=0):
    return dict(base_quat=[0,0,math.sin(yaw/2),math.cos(yaw/2)], base_pos=np.array([0.,0.,1.]),
                fk_info={name: dict(pos=np.array(pos)) for name,pos in [
                    ('left_shoulder_pitch_link',[0,.2,1.4]), ('right_shoulder_pitch_link',[0,-.2,1.4]),
                    ('left_ankle_link',[0,.15,.05]), ('right_ankle_link',[0,-.15,.05])]})


class StationaryReferenceTest(unittest.TestCase):
    def setUp(self):
        self.cfg=StationaryReferenceCfg(enabled=True, feedback_warmup_seconds=0)
        self.guard=StationaryReferenceGuard(self.cfg)

    def run_to_target(self, raw, count=200, data=None):
        robot=ANCHOR.copy()
        for _ in range(count):
            target,velocity,status=self.guard.apply(raw,robot,ANCHOR,observation() if data is None else data,.02)
            np.testing.assert_allclose(velocity*.02,target-robot,atol=1e-12)
            self.assertLessEqual(np.linalg.norm(velocity,axis=1).max(),self.cfg.max_speed_m_s+1e-10)
            np.testing.assert_array_equal(target[2],ANCHOR[2])
            robot=target
        return target,velocity,status

    def test_common_translation_and_head_motion_do_not_move_body_target(self):
        target,velocity,status=self.guard.apply(ANCHOR+[2,-1,.5],ANCHOR,ANCHOR,observation(),.02)
        np.testing.assert_allclose(target,ANCHOR,atol=1e-12)
        np.testing.assert_allclose(velocity,0,atol=1e-12)
        self.assertIn('head fixed',status['reasons'])

    def test_overshoot_saturates_without_windup_and_recovers_inward_immediately(self):
        raw=ANCHOR.copy();raw[0]+=[3,-2,2]
        target,velocity,status=self.run_to_target(raw)
        self.assertTrue(status['limited'])
        self.assertGreaterEqual(target[0,1],self.cfg.hand_separation_m/2-1e-12)
        self.assertLessEqual(np.linalg.norm(target[0]-[0,.2,1.4]),self.cfg.shoulder_reach_m+1e-12)
        np.testing.assert_allclose(velocity,0,atol=1e-8)
        next_target,velocity,_=self.guard.apply(ANCHOR,target,ANCHOR,observation(),.02)
        self.assertLess(np.linalg.norm(next_target[0]-ANCHOR[0]),np.linalg.norm(target[0]-ANCHOR[0]))
        self.assertLessEqual(np.linalg.norm(velocity,axis=1).max(),self.cfg.max_speed_m_s+1e-10)

    def test_inside_workspace_remains_unchanged_at_steady_state(self):
        raw=ANCHOR.copy();raw[0]+=[.02,.01,.02]
        target,_,status=self.run_to_target(raw)
        np.testing.assert_allclose(target,raw,atol=1e-10)
        self.assertFalse(status['limited'])

    def test_raising_and_extending_fist_is_not_suppressed_as_combined_delta(self):
        raw=ANCHOR.copy();raw[1]+=[.35,0,.45]
        target,_,_=self.guard.apply(raw,ANCHOR,ANCHOR,observation(),.02,active=False)
        self.assertGreater(target[1,0]-ANCHOR[1,0],.25)
        self.assertGreater(target[1,2]-ANCHOR[1,2],.35)
        self.assertLessEqual(np.linalg.norm(target[1]-[0,-.2,1.4]),self.cfg.shoulder_reach_m+1e-12)

    def test_directional_limits_pass_forward_outward_and_upward(self):
        # Preserve points before the soft boundary, then verify each directional cap.
        self.assertEqual(self.cfg.forward_m,1.10)
        self.assertEqual(self.cfg.backward_m,.45)
        self.assertEqual(self.cfg.lateral_m,1.70)
        self.assertEqual(self.cfg.upward_m,.90)
        self.assertEqual(self.cfg.shoulder_reach_m,1.70)
        data=observation()
        for index,side in enumerate(('left','right')):
            center=data['fk_info'][side+'_shoulder_pitch_link']['pos']
            for direction,limit in (([1,0,0],self.cfg.forward_m),
                                    ([-1,0,0],self.cfg.backward_m),
                                    ([0,1 if index==0 else -1,0],self.cfg.lateral_m),
                                    ([0,0,1],self.cfg.upward_m)):
                guard=StationaryReferenceGuard(self.cfg)
                raw=ANCHOR.copy();raw[index]=center+(limit*(self.cfg.soft_boundary-.01))*np.asarray(direction)
                target,_,_=guard.apply(raw,ANCHOR,ANCHOR,data,.02,active=False)
                np.testing.assert_allclose(target[index],raw[index],atol=1e-12)
                raw[index]=center+3*np.asarray(direction)
                target,_,status=guard.apply(raw,ANCHOR,ANCHOR,data,.02,active=False)
                self.assertTrue(status['limited'])
                self.assertAlmostEqual(np.linalg.norm(target[index]-center),limit)

    def test_settled_bias_is_not_a_fault_but_new_tracking_error_still_triggers(self):
        guard=StationaryReferenceGuard(StationaryReferenceCfg(enabled=True,feedback_warmup_seconds=.1))
        measured=ANCHOR.copy();measured[:2,2]-=.15
        for _ in range(60):
            _,_,status=guard.apply(ANCHOR,measured,ANCHOR,observation(),.02)
        self.assertFalse(status['retreat_latched'])
        np.testing.assert_allclose(guard.tracking_baseline,.15)
        measured[0,2]-=.4
        for _ in range(40):
            _,_,status=guard.apply(ANCHOR,measured,ANCHOR,observation(),.02)
        self.assertTrue(status['retreat_latched'])

    def test_warmup_cannot_absorb_arbitrarily_large_tracking_bias(self):
        guard=StationaryReferenceGuard(StationaryReferenceCfg(enabled=True,feedback_warmup_seconds=.1))
        measured=ANCHOR.copy();measured[:2,2]-=.6
        for _ in range(60):
            _,_,status=guard.apply(ANCHOR,measured,ANCHOR,observation(),.02)
        self.assertTrue(status['retreat_latched'])
        self.assertLessEqual(guard.tracking_baseline.max(),guard.cfg.tracking_baseline_max_m)

    def test_retreat_latches_until_input_returns_near_neutral(self):
        raw=ANCHOR.copy();raw[0,0]+=.18
        target,_,_=self.run_to_target(raw)
        data=observation();angle=math.radians(25)
        data['base_quat']=[math.sin(angle/2),0,0,math.cos(angle/2)]
        for _ in range(150):
            target,_,status=self.guard.apply(raw,target,ANCHOR,data,.02)
        self.assertTrue(status['retreat_latched'])
        self.assertLess(np.linalg.norm(target-ANCHOR),.005)
        for _ in range(150):
            target,_,status=self.guard.apply(raw,target,ANCHOR,observation(),.02)
        self.assertTrue(status['retreat_latched'])
        for _ in range(60):
            target,_,status=self.guard.apply(ANCHOR,target,ANCHOR,observation(),.02)
        self.assertFalse(status['retreat_latched'])

    def test_foot_motion_and_persistent_tracking_error_trigger_retreat(self):
        raw=ANCHOR.copy();raw[0,0]+=.18
        target,_,_=self.run_to_target(raw)
        data=observation();data['fk_info']['left_ankle_link']['pos'][0]+=.15
        _,_,status=self.guard.apply(raw,target,ANCHOR,data,.02)
        self.assertIn('foot displacement',status['reasons'])
        self.guard.reset()
        # Deliberately retain a large measured error; a single frame cannot trip the dwell.
        measured=ANCHOR.copy();measured[0,0]-=.4
        _,_,status=self.guard.apply(raw,measured,ANCHOR,observation(),.02)
        self.assertFalse(status['retreat_latched'])
        for _ in range(40):
            _,_,status=self.guard.apply(raw,measured,ANCHOR,observation(),.02)
        self.assertTrue(status['retreat_latched'])

    def test_preview_does_not_preload_a_far_target_and_anchor_never_follows_lean(self):
        raw=ANCHOR.copy();raw[0,0]+=1
        preview,_,_=self.guard.apply(raw,ANCHOR,ANCHOR,observation(),.02,active=False)
        self.assertGreater(np.linalg.norm(preview-ANCHOR),.01)
        np.testing.assert_array_equal(self.guard.target,ANCHOR)
        data=observation();data['fk_info']['left_shoulder_pitch_link']['pos'][0]+=.3
        target,_,_=self.guard.apply(raw,ANCHOR,ANCHOR,data,.02)
        self.assertLessEqual(np.linalg.norm(target-ANCHOR),self.cfg.max_speed_m_s*.02+1e-12)
        np.testing.assert_allclose(self.guard.shoulders[0],[0,.2,1.4])

    def test_fixed_heading_rotates_the_workspace(self):
        raw=ANCHOR.copy();raw[0]+=[.04,.02,.03]
        normal,_,_=self.guard.apply(raw,ANCHOR,ANCHOR,observation(),.02,active=False)
        rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        other=StationaryReferenceGuard(self.cfg)
        data=observation(math.pi/2)
        for item in data['fk_info'].values(): item['pos']=rotation@item['pos']
        rotated,_,_=other.apply(raw@rotation.T,ANCHOR@rotation.T,ANCHOR@rotation.T,data,.02,active=False)
        np.testing.assert_allclose(rotated,normal@rotation.T,atol=1e-12)

    def test_bad_configuration_and_nonfinite_input_fail(self):
        for values in (dict(forward_m=0),dict(drift_hard_m=.01),dict(max_speed_m_s=float('nan'))):
            with self.assertRaises(ValidationError): StationaryReferenceCfg(**values)
        raw=ANCHOR.copy();raw[0,0]=np.nan
        with self.assertRaises(ValueError):self.guard.apply(raw,ANCHOR,ANCHOR,observation(),.02)

    def test_startup_support_capture_does_not_move_workspace(self):
        guard=StationaryReferenceGuard(StationaryReferenceCfg(enabled=True,feedback_warmup_seconds=.1))
        data=observation()
        for _ in range(5):
            data['fk_info']['left_ankle_link']['pos'][0]+=.02
            _,_,status=guard.apply(ANCHOR,ANCHOR,ANCHOR,data,.02)
            self.assertFalse(status['retreat_latched'])
        np.testing.assert_array_equal(guard.anchor,ANCHOR)
        data['fk_info']['left_ankle_link']['pos'][0]+=.15
        _,_,status=guard.apply(ANCHOR,ANCHOR,ANCHOR,data,.02)
        self.assertTrue(status['retreat_latched'])


class LauncherTest(unittest.TestCase):
    def test_default_and_full_body_motion_modes(self):
        for flag in ([],['--full-body']):
            cfg=run_pipeline.make_cfg(run_pipeline.parse_args(['-c','h1_h2h','--policy-name','model_30000_jit',*flag]))
            self.assertEqual(cfg.policy.stationary_reference.enabled,not bool(flag))
            self.assertFalse(cfg.policy.use_imu_torso)
            self.assertEqual(cfg.policy.action_beta,1.)

    def test_quest_launchers_use_the_same_default_and_full_body_switch(self):
        for launcher in (run_quest_h2h,run_quest_h2h_real):
            for flags in ([],['--full-body']):
                with patch.object(sys,'argv',['launcher','--dds-sim','--no-keyboard',*flags]):
                    cfg=launcher.make_cfg(launcher.parse_args())
                self.assertEqual(cfg.policy.stationary_reference.enabled,not bool(flags))

    def test_stand_transition_discards_unprepared_wait_anchor(self):
        guard=StationaryReferenceGuard(StationaryReferenceCfg(enabled=True))
        guard.apply(ANCHOR,ANCHOR,ANCHOR,observation(),.02)
        pipeline=QuestH2HDeployPipeline.__new__(QuestH2HDeployPipeline)
        pipeline.phase='READY'
        prepared=ANCHOR+[0,0,.1]
        pipeline.quest=SimpleNamespace(robot_points=lambda:prepared.copy())
        pipeline.policy=Mock()
        pipeline.policy.reset_reference_guard.side_effect=guard.reset
        data=SimpleNamespace(dof_pos=np.zeros(19))
        pipeline._policy_env_data=lambda value:value
        pipeline._commands(['[H1_STAND]'],data)
        self.assertEqual(pipeline.phase,'STAND')
        self.assertIsNone(guard.anchor)
        np.testing.assert_array_equal(pipeline._stand_reference,prepared)


class DDSVisualizationTest(unittest.TestCase):
    def test_raw_and_accepted_markers_and_legacy_packet_without_network(self):
        bridge=H1DDSBridge.__new__(H1DDSBridge)
        bridge.lock=threading.Lock();bridge.server_id='local-test'
        bridge.viewer=Mock();bridge.data=SimpleNamespace(qpos=np.array([0.,0.,1.]))
        packet=dict(server_id='local-test',order=['left','right','head'],
                    targets_m=ANCHOR.tolist(),base_pos_m=[0,0,1],raw_targets_m=(ANCHOR+[.4,0,0]).tolist())
        bridge.on_targets(SimpleNamespace(data=json.dumps(packet)))
        bridge.draw_targets()
        markers={call.kwargs['id']:call.kwargs for call in bridge.viewer.add_marker.call_args_list}
        np.testing.assert_allclose(markers[1000]['pos'],ANCHOR[0])
        np.testing.assert_allclose(markers[2000]['pos'],ANCHOR[0]+[.4,0,0])
        del packet['raw_targets_m']
        bridge.on_targets(SimpleNamespace(data=json.dumps(packet)))
        self.assertIsNone(bridge.raw_targets)
        bridge.viewer.add_marker.reset_mock();bridge.draw_targets()
        markers={call.kwargs['id']:call.kwargs for call in bridge.viewer.add_marker.call_args_list}
        self.assertEqual(markers[2000]['size'],0)


if __name__=='__main__': unittest.main()
