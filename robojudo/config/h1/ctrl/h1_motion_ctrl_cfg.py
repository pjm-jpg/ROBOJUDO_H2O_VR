from robojudo.controller.ctrl_cfgs import MotionH2HCtrlCfg

#也是继承了MotionH2HCtrlCfg类，定义了一个名为H1MotionH2HCtrlCfg的配置类，
# 用于配置H1机器人在H2H任务中的动作控制器。
class H1MotionH2HCtrlCfg(MotionH2HCtrlCfg):
    # ==== policy specific configs ====
    #跟踪点的名称列表，用于指定机器人在H2H任务中需要跟踪的关键点。
    track_keypoints_names: list[str] = []
    phc: MotionH2HCtrlCfg.PhcCfg = MotionH2HCtrlCfg.PhcCfg(
        robot_config_file="robot/unitree_h1.yaml",
    )

    # ==== motion config ====
    robot: str = "h1"
    # PHC retargeted motion
    # motion_name: str = "dance_sample_h1" # Warning: special heading
    motion_name: str = "singles/0-KIT_572_punch_right01_poses"
