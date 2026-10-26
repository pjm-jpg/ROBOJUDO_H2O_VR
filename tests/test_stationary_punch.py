"""A real punch must retain useful travel while remaining bounded in MuJoCo."""
import json
import math
from pathlib import Path
import unittest

import numpy as np

from robojudo.config.config_manager import ConfigManager
from robojudo.pipeline.rl_pipeline import RlPipeline
from robojudo.policy.utils.h2h_model import select_h2h_model


class StationaryPunchTest(unittest.TestCase):
    def test_complete_punch_has_measurable_hand_travel_at_both_speeds(self):
        reports=[]
        for speed in (.1,1.):
            cfg=ConfigManager('h1_h2h').get_cfg()
            cfg.policy=select_h2h_model(cfg.policy,'model_30000_jit')
            cfg.env.headless=True
            cfg.ctrl=[c for c in cfg.ctrl if c.ctrl_type=='MotionH2HCtrl']
            for ctrl in cfg.ctrl:ctrl.motion_ctrl_gui=False
            pipeline=RlPipeline(cfg)
            motion=pipeline.ctrl_manager.controllers['MotionH2HCtrl'].inst
            targets=[];actual=[];body=[];scales=[]
            original=pipeline.policy.get_observation
            def capture(data,control):
                obs=original(data,control)
                ref=control['MotionH2HCtrl']
                p=np.asarray(ref['ref_body_pos_subset'])
                targets.append(p.copy())
                scales.append(ref['stationary_status']['arm_scale'])
                return obs
            pipeline.policy.get_observation=capture
            try:
                for _ in range(150):pipeline.step()
                targets.clear();scales.clear()
                for _ in range(round((float(motion.motion_length)/speed+1)/pipeline.dt)):
                    motion.play_speed_ratio=speed
                    pipeline.step()
                    p=motion.get_robot_state()[1]
                    actual.append(p[:2]-p[2])
                    root=pipeline.env.data.qpos[:7]
                    tilt=math.acos(float(np.clip(1-2*(root[4]**2+root[5]**2),-1,1)))
                    body.append([root[0],root[1],root[2],tilt])
                    self.assertGreater(root[2],.65)
                    self.assertLess(tilt,.5)
                targets=np.asarray(targets);actual=np.asarray(actual);body=np.asarray(body)
                target_relative=targets[:,:2]-targets[:,2:3]
                target_span=np.ptp(target_relative,axis=0)[1]
                actual_span=np.ptp(actual,axis=0)[1]
                self.assertGreater(target_span[0],.50,'Forward punch was suppressed by workspace constraints')
                self.assertGreater(target_span[2],.50,'Lifting the fist was suppressed')
                self.assertGreater(actual_span[0],.36,'Robot did not execute a useful forward punch')
                self.assertGreater(actual_span[2],.50)
                self.assertGreater(min(scales),.9,'Normal punch falsely triggered retreat')
                self.assertLess(np.linalg.norm(body[:,:2],axis=1).max(),.15)
                np.testing.assert_allclose(targets[:,2],np.broadcast_to(targets[0,2],targets[:,2].shape),atol=1e-12)
                maximum_step=np.linalg.norm(np.diff(targets,axis=0),axis=2).max()
                self.assertLessEqual(maximum_step,cfg.policy.stationary_reference.max_speed_m_s*pipeline.dt+1e-9)
                reports.append(dict(speed=speed,reference_right_span_m=target_span.tolist(),
                    actual_right_span_m=actual_span.tolist(),max_body_drift_m=float(np.linalg.norm(body[:,:2],axis=1).max()),
                    max_tilt_degrees=float(np.degrees(body[:,3].max())),minimum_arm_scale=float(min(scales))))
            finally:pipeline.env.shutdown()
        Path('/tmp/h2h-stationary-punch-regression.json').write_text(json.dumps(reports,indent=2))


if __name__=='__main__':unittest.main()
