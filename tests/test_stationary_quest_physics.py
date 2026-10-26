"""Bounded synthetic Quest motion with actual H1 Actor and MuJoCo physics."""
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from robojudo.config.config_manager import ConfigManager
from robojudo.pipeline.quest_h2h_pipeline import QuestH2HPipeline
from robojudo.policy.utils.h2h_model import select_h2h_model


HUMAN=np.array([[.3,.3,1.2],[.3,-.3,1.2],[0,0,1.7]])


def frame(sequence, positions=HUMAN):
    return dict(session_id=1,space_epoch=0,sequence=sequence,sample_time_ns=1_000_000_000+sequence*250_000_000,
                reference_space='stage',focused=True,points={name:dict(valid=True,tracked=True,
                position_m=pos.tolist(),orientation_valid=True,orientation_tracked=True,
                orientation_xyzw=[0,0,0,1]) for name,pos in zip(('left','right','head'),positions)})


class Reader:
    def __init__(self,**kwargs):self.frame=None
    def start(self):pass
    def close(self):pass
    def get_latest(self,**kwargs):return self.frame


class StationaryQuestPhysicsTest(unittest.TestCase):
    def test_small_motion_overshoot_translation_and_recovery_for_both_models(self):
        reports=[]
        for model in ('h2h_release','model_30000_jit'):
            cfg=ConfigManager('h1_h2h_quest').get_cfg()
            cfg.policy=select_h2h_model(cfg.policy,model)
            cfg.env.headless=True
            cfg.ctrl=[c for c in cfg.ctrl if c.ctrl_type=='QuestH2HCtrl']
            with patch('robojudo.controller.quest_h2h_ctrl.load_reader_class',return_value=Reader):
                pipeline=QuestH2HPipeline(cfg)
            try:
                quest=pipeline.quest
                quest.reader.frame=frame(0);quest.get_data();quest.command('[QUEST_CALIBRATE]')
                for i in range(1,6):quest.reader.frame=frame(i);quest.get_data()
                self.assertIsNotNone(quest.alignment)
                anchor=quest.alignment.robot.copy()
                pipeline._commands(['[QUEST_ENABLE]'])
                initial=pipeline.env.data.qpos[:3].copy()
                min_height=float('inf');max_tilt=max_drift=0.
                normal_travel=0.;limited=0;last=None;head_error=0.
                for tick in range(1000):
                    t=tick*pipeline.dt;positions=HUMAN.copy()
                    if 2<=t<6:
                        positions[0,0]+=.04*math.sin((t-2)*math.pi)
                        positions[1,2]+=.04*math.sin((t-2)*math.pi)
                    if 6<=t<16:
                        amount=min((t-6)/2,1.) if t<12 else max(0.,1-(t-12)/4)
                        positions[0]+=[amount, -.5*amount, .5*amount]
                        positions[1]+=[amount, .5*amount, .5*amount]
                        # Whole-body operator translation must not move head reference.
                        positions+=[.6*amount,0,-.3*amount]
                    quest.reader.frame=frame(tick+10,positions)
                    pipeline.step()
                    guard=pipeline.policy.reference_guard
                    target=guard.target.copy()
                    if last is not None:
                        self.assertLessEqual(np.linalg.norm(target-last,axis=1).max(),cfg.policy.stationary_reference.max_speed_m_s*pipeline.dt+1e-9)
                    last=target
                    head_error=max(head_error,float(np.linalg.norm(target[2]-anchor[2])))
                    self.assertTrue(quest.enabled)
                    if 2<=t<6:normal_travel=max(normal_travel,float(np.linalg.norm(target[:2]-anchor[:2],axis=1).max()))
                    if t>=6 and np.linalg.norm(target[:2]-positions[:2])>.1:limited+=1
                    root=pipeline.env.data.qpos[:7]
                    tilt=math.acos(float(np.clip(1-2*(root[4]**2+root[5]**2),-1,1)))
                    min_height=min(min_height,float(root[2]));max_tilt=max(max_tilt,tilt)
                    max_drift=max(max_drift,float(np.linalg.norm(root[:2]-initial[:2])))
                    self.assertGreater(root[2],.65,'Robot fell')
                    self.assertLess(tilt,.5,'Large body tilt')
                self.assertGreater(normal_travel,.01,'Guard prevented all normal arm motion')
                self.assertLess(head_error,1e-12)
                self.assertGreater(limited,0)
                self.assertLess(max_drift,.15)
                self.assertLess(np.linalg.norm(guard.target[:2]-anchor[:2],axis=1).max(),.03)
                reports.append(dict(model=model,simulation_seconds=20,normal_target_travel_m=normal_travel,
                                    max_base_drift_m=max_drift,max_tilt_degrees=math.degrees(max_tilt),
                                    min_root_height_m=min_height,head_reference_error_m=head_error,
                                    final_retreat_scale=guard.scale))
            finally:pipeline.close()
        Path('/tmp/h2h-stationary-physics.json').write_text(json.dumps(reports,indent=2))


if __name__=='__main__':unittest.main()
