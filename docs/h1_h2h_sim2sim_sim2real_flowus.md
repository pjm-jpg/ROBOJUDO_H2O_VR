# RoboJuDo H1 H2H 的 sim2sim 与 sim2real 学习笔记

这份笔记以 RoboJuDo 本地项目的 H1 H2H 部署路径为主线，解释程序怎样加载策略、组织观测、处理动作，并在 MuJoCo 或真实机器人中执行控制。阅读目标是能够沿着代码追踪一轮控制，而不只是记住启动命令。

> 版本说明：依据 2026 年 10 月 8 日学习过程中检查的本地代码整理。本地含有定制修改，不应将所有细节视为上游仓库的通用行为。本文没有进行真机验证，示例数值均为教学假设。

> 名称说明：讨论中提到的 H2O，在本仓库中使用的具体入口是 `h1_h2h`，策略类是 `H2HStudentPolicy`，项目文档将其归于 Human2Humanoid。本文围绕这条实现展开，不把 H2O、OmniH2O 和其他论文实现视为完全相同。

## 阅读顺序

1. 项目定位与目录地图
2. 启动与配置
3. 一轮控制的数据流
4. H2H 观测构造
5. 动作处理与关节目标
6. 所有关节顺序与电机编号映射
7. MuJoCo 控制时序
8. 真机通信与状态估计
9. sim2sim 和 sim2real 的迁移排查
10. 左膝端到端练习
11. 已知代码限制与源码阅读索引

## 1 项目定位与目录地图

### 1.1 训练与部署

策略可以理解为一个已经训练好的函数：根据观测生成动作。RoboJuDo 的核心部署流程负责加载模型并运行闭环，不在每轮控制时重新训练模型。

```text
训练体系：仿真环境 + 奖励 + 学习算法
                  |
                  v
              导出策略模型
                  |
          +-------+--------+
          v                v
另一仿真中的闭环验证       真实机器人部署
      sim2sim                 sim2real
```

sim2sim 关注在其他仿真训练体系中得到的策略能否在目标仿真中正常闭环运行。仅显示机器人模型、直接播放关节动画，不等于完成策略迁移验证。

sim2real 还涉及传感器、状态估计、通信、执行器与物理环境。统一软件接口可以复用代码，但不能自动消除动力学和时延差异。

### 1.2 四个核心角色

| 模块 | 职责 | H1 H2H 中的实例 |
| --- | --- | --- |
| Controller | 提供用户操作或参考任务信息 | KeyboardCtrl、MotionH2HCtrl |
| Policy | 整理观测、调用模型、处理输出 | H2HStudentPolicy |
| Environment | 获取机器人状态、执行关节目标 | MujocoEnv、UnitreeEnv |
| Pipeline | 按顺序连接以上模块 | RlPipeline |

这里的 Controller 主要是任务输入来源，不等同于电机底层的 PD 控制器。

### 1.3 目录地图

```text
scripts/
  run_pipeline.py                 通用启动入口
  run_tracker_pipeline.py         其他跟踪策略的专用入口
  view_log_msgpack.py             日志读取与可视化示例
robojudo/
  config/                         参数、模块选择与配置注册
    h1/h1_cfg.py                  h1、h1_real、h1_h2h 等配置
    h1/env/                       H1 仿真与真机参数
    h1/policy/                    H1 各策略参数与关节约定
    h1/ctrl/                      H1 参考动作配置
  pipeline/                       主循环、准备动作、多策略切换
  controller/                     键盘、手柄、动作参考输入
  policy/                         策略观测、模型推理、动作处理
  environment/                    MuJoCo 与硬件接口
  tools/                          关节适配、运动学、日志等
  utils/                          旋转、注册表及通用工具
assets/
  robots/h1/h1.xml                H1 MuJoCo 模型
  models/h1/h2h/h2h_release.pt     H2H 策略模型
  motions/h1/phc/                 PHC 格式参考动作
third_party/phc/                  动作处理与机器人配置等依赖
packages/unitree_cpp/             另一条 C++ 硬件集成路径
docs/                             环境、策略、控制器与 SDK 文档
```

项目还有 G1、多策略切换和其他跟踪策略。它们复用部分基础设施，但不能套用本文 H1 H2H 的输入维度和关节参数。

## 2 启动与配置

### 2.1 入口命令

```bash
python scripts/run_pipeline.py --config h1_h2h
```

这条命令选择名为 `h1_h2h` 的 Python 配置类，不是自动读取 `h1_h2h.yaml`。运行前需要完成项目与相应依赖安装；本文没有实际执行该命令。

### 2.2 配置查找过程

```text
run_pipeline.py 的 main()
  -> parse_args() 取得字符串 h1_h2h
  -> ConfigManager(config_name="h1_h2h")
  -> cfg_registry.get("h1_h2h") 找到类
  -> cfg_class() 创建配置对象
  -> get_cfg() 返回 cfg
  -> 根据 cfg.pipeline_type 找到 RlPipeline
  -> RlPipeline(cfg=cfg) 创建控制系统
```

`@cfg_registry.register` 在模块导入时登记配置类。`getattr(module, name)` 根据字符串取得类。类型标注如 `cfg: RlPipelineCfg` 用于描述类型，不负责创建对象。

### 2.3 H1 H2H 的具体选择

以下为保留有效选择的简化配置：

```python
class h1_h2h(RlPipelineCfg):
    robot = "h1"
    env = H1MujocoEnvCfg()
    ctrl = [KeyboardCtrlCfg(), H1MotionH2HCtrlCfg()]
    policy = H1H2HPolicyCfg()
```

`G1...Cfg()` 或 `H1...Cfg()` 这类对象保存参数，不等于创建实际环境。真正的环境在流水线调用 `env_class(...)` 时创建。

本地 H1 H2H 配置使用：

| 资源 | 路径或选择 | 用途 |
| --- | --- | --- |
| 策略类 | H2HStudentPolicy | 组织观测并调用模型 |
| 模型 | assets/models/h1/h2h/h2h_release.pt | 已训练策略 |
| 动作 | assets/motions/h1/phc/singles/0-KIT_572_punch_right01_poses.pkl | 参考运动 |
| PHC 配置 | third_party/phc/phc/data/cfg/robot/unitree_h1.yaml | 机器人与扩展点定义 |

动作文件告诉系统“要跟踪什么”，模型负责根据当前反馈生成控制动作。该动作文件名指向右拳动作，本文未播放确认内容。

### 2.4 流水线初始化

`RlPipeline.__init__()` 的主要工作依次为：

1. 保存配置，选择计算设备，按需创建日志器。
2. 创建环境对象。
3. 创建 CtrlManager 及其输入控制器。
4. 创建 PolicyWrapper，内部创建 H2HStudentPolicy 并加载模型。
5. 建立观测与动作关节映射。
6. 用策略动作关节配置更新环境对应参数。
7. 设置策略频率和周期，重置并自检。

自检中的 `step(dry_run=True)` 会执行观测和推理，但跳过正常的 `env.step(...)` 调用。它仍可能改变历史缓存等内部状态，不是完全无副作用的数学检查。

> `h1_real` 继承的是普通 `h1` 配置，默认选择 Unitree 策略。它不是 `h1_h2h` 的现成真机版本。

## 3 一轮控制的数据流

主线位于 `robojudo/pipeline/rl_pipeline.py`。以下省略模式切换、淡入淡出和日志分支：

```python
self.env.update()
env_data = self.env.get_data()
ctrl_data = self.ctrl_manager.get_ctrl_data(env_data)
obs, extras = self.policy.get_observation(env_data, ctrl_data)
pd_target = self.policy.get_pd_target(obs)
self.env.step(pd_target, extras.get("hand_pose", None))
```

| 变量 | 含义 |
| --- | --- |
| env_data | 环境提供的状态，不一定全部送入网络 |
| ctrl_data | 用户操作、参考运动及相关任务数据 |
| obs | 按训练约定拼接的网络输入 |
| extras | 可视化、附加控制或回调所需数据 |
| pd_target | 环境关节顺序下的目标角度 |

`PolicyWrapper` 先把环境关节状态调整到策略顺序，再调用策略观测构造；输出则反向适配回环境顺序。

## 4 H2H 观测构造

### 4.1 当前机器人状态

入口是 `H2HStudentPolicy.get_observation(env_data, ctrl_data)`。

H1 H2H 使用 19 个关节。当前 `use_dof_pos_offset=False`，因此输入关节角度本身，而不是相对默认姿态的角度偏差。关节速度和角速度的当前缩放系数均为 1.0。

`use_imu_torso=True` 使该策略使用躯干角速度和躯干姿态生成相应观测。变量名 `base_ang_vel` 在这条分支中实际保存躯干角速度。

重力观测由世界中的单位方向 `[0, 0, -1]` 经过躯干姿态的逆旋转得到，表达“向下”在躯干坐标系中的方向。它不是大小为 9.81 的加速度，也无法单独确定水平朝向。

### 4.2 三个身体跟踪点

当前 H1 配置的普通跟踪点列表为空，但控制器仍加入 PHC `extend_config` 中的三个扩展点：

1. left_hand_link：相对左肘连杆定义。
2. right_hand_link：相对右肘连杆定义。
3. head_link：相对 pelvis 定义。

它们是模型中的任务点，不代表三个实际传感器。机器人当前点的位置通过运动学计算得到。每组位置或速度包含三个点，每点三个坐标，形状为 `(3, 3)`。

MotionH2HCtrl 提供参考点位置、参考点速度和当前机器人对应点位置。参考速度还乘以播放倍率。

### 4.3 27 维任务观测

`robojudo/policy/utils/utils_h2h.py` 中的函数生成三组量：

| 数据 | 计算方式 | 维度 |
| --- | --- | --- |
| 点位误差 | 参考身体点位置 - 当前身体点位置 | 9 |
| 相对参考位置 | 参考身体点位置 - 当前根部位置 | 9 |
| 参考点速度 | 参考运动中的点速度 | 9 |

以上向量都旋转到去除根部水平朝向的坐标表达中。这里去除的是 heading，不是完整躯干姿态；不要与重力观测的逆旋转混淆。

构造任务观测前，若参考头部与根部的水平距离超过 1 米，代码会调整参考头部的水平位置，将距离限制到 1 米。因此送入任务计算的参考头部可能不同于原始动作点。

### 4.4 历史与上一步动作

每条历史包含关节角度 19 维、关节速度 19 维、角速度 3 维、重力方向 3 维和上一步动作 19 维，共 63 维。25 条历史共 1575 维，不包含 task_obs。

先读取旧历史构造当前观测，再将本次记录通过 `appendleft` 放到最前面，因此历史由近到远排列。正常 50 Hz 控制时大致覆盖之前半秒；自检等额外调用不应直接解释为真实经过了半秒。

历史初始填零。当前 H2H 的 `reset()` 是空实现，不会自动清空历史或 last_action。

### 4.5 完整输入布局

| 数据段 | 维度 | 下标范围 |
| --- | --- | --- |
| 关节角度 | 19 | 0 至 18 |
| 关节速度 | 19 | 19 至 37 |
| 躯干角速度 | 3 | 38 至 40 |
| 重力方向 | 3 | 41 至 43 |
| 任务观测 | 27 | 44 至 70 |
| 上一步动作 | 19 | 71 至 89 |
| 历史 | 1575 | 90 至 1664 |
| 总计 | 1665 | 0 至 1664 |

1665 是按当前配置和拼接代码推导的结果，不是模型运行验证结果。维度一致只是必要条件；每个位置的含义、单位、顺序和坐标约定也必须一致。

## 5 动作处理与关节目标

### 5.1 推理与后处理

`Policy.get_action()` 将 `(1665,)` 输入转为 `(1, 1665)` 张量，调用模型，然后将输出转回数组。这里的批次大小为 1。

当前 H2H 动作处理可表达为：

```text
a_smooth[t] = 0.2 * a_smooth[t-1] + 0.8 * a_raw[t]
last_action = a_smooth[t]
delta_q = 0.25 * clip(a_smooth[t], -10, 10)
q_target = q_default + delta_q
```

`last_action` 保存平滑后、裁剪和缩放前的动作。动作裁剪不等于逐关节机械角度限位。

### 5.2 默认角度的意义

目标是在固定默认姿态上叠加偏移，不是在当前实际角度上逐轮累加，也不是直接在参考动作关节角度上叠加。

左膝示例：上一轮平滑动作 0.2，本轮原始动作 1.0，得到平滑动作 0.84；缩放得到 0.21 rad；加默认角度 0.8 rad，目标为 1.01 rad。

### 5.3 参数与目标一起遵循关节顺序

default_pos、stiffness、damping 等数组都需要与 joint_names 对应。流水线通过 `update_dof_cfg` 和 `merge_dof_cfgs` 将策略中非空的关节参数按名字覆盖到环境配置，不能只看环境初始增益。

H1 H2H 的左膝最终配置增益为 Kp=200、Kd=4。环境初始配置的左膝增益不同。

## 6 所有关节顺序与电机编号映射

### 6.1 三个索引空间

同一个关节名字在不同数组中可以对应不同位置。索引从 0 开始。

```text
读取：电机消息顺序 -> 环境关节顺序 -> 策略观测关节顺序
发送：策略动作关节顺序 -> 环境关节顺序 -> 电机消息顺序
```

DoFAdapter 通过关节名字匹配策略与环境。joint2motor_idx 是人工配置的环境关节到消息槽位对应表，不会自动向硬件查询关节名称。

名字也必须约定一致；代码不会自动将 left_knee 和 left_knee_joint 当作同一个名字。映射只改变数据位置，不改变单位、旋转正方向或零点。

### 6.2 H1 H2H 完整对应表

| 关节名称 | 策略下标 | 环境下标 | 电机消息编号 |
| --- | --- | --- | --- |
| left_hip_yaw_joint | 0 | 0 | 7 |
| left_hip_roll_joint | 1 | 1 | 3 |
| left_hip_pitch_joint | 2 | 2 | 4 |
| left_knee_joint | 3 | 3 | 5 |
| left_ankle_joint | 4 | 4 | 10 |
| right_hip_yaw_joint | 5 | 5 | 8 |
| right_hip_roll_joint | 6 | 6 | 0 |
| right_hip_pitch_joint | 7 | 7 | 1 |
| right_knee_joint | 8 | 8 | 2 |
| right_ankle_joint | 9 | 9 | 11 |
| torso_joint | 10 | 10 | 6 |
| left_shoulder_pitch_joint | 11 | 11 | 16 |
| left_shoulder_roll_joint | 12 | 12 | 17 |
| left_shoulder_yaw_joint | 13 | 13 | 18 |
| left_elbow_joint | 14 | 14 | 19 |
| right_shoulder_pitch_joint | 15 | 15 | 12 |
| right_shoulder_roll_joint | 16 | 16 | 13 |
| right_shoulder_yaw_joint | 17 | 17 | 14 |
| right_elbow_joint | 18 | 18 | 15 |

这张表依据当前真机配置，不代替硬件型号核验。19 个关节使用 0 至 19 中的 19 个槽位，跳过 9；仅凭此表不能确定槽位 9 的用途。

### 6.3 如何使用映射表

```python
joint2motor_idx = [7, 3, 4, 5, 10, 8, 0, 1, 2, 11, 6, 16, 17, 18, 19, 12, 13, 14, 15]
# 下标是环境关节位置，值是电机消息编号。
# 以下为解释读写方向的伪代码。
env_q[j] = motor_q[joint2motor_idx[j]]
motor_target[joint2motor_idx[j]] = env_target[j]
```

左膝读取：motor_state[5].q 放到 env_data.dof_pos[3]，再进入策略左膝位置 3。

左膝发送：策略左膝目标 3 放到环境目标 3，再写入 motor_cmd[5].q。读是按地址取数，写是按地址放数，因此可以共用同一张表。

### 6.4 为什么不全部统一

本项目已统一内部环境接口。但训练好的网络记住了输入输出位置的语义，硬件协议也固定了消息槽位的含义。只修改配置中的名字列表，不能让网络或硬件自动改变约定。

当前 H1 H2H 的策略顺序与环境顺序已经一致，因此 DoFAdapter 在这条路径上不改变排列。保留适配层是为了兼容其他策略，而不是重复重排。

### 6.5 MuJoCo 与身体点索引

MuJoCo 后端不使用真机 joint2motor_idx。当前环境从 qpos、qvel 末尾取出 19 个关节数据，并按环境顺序写入 data.ctrl。这依赖模型关节和执行器排列与环境约定一致。

qpos 前部含浮动根部的位置和四元数，不能把 qpos 原始下标直接当成环境关节下标。

MotionH2HCtrl 还按身体名称查找运动学身体点，并附加扩展点。身体点索引与关节索引不同：19 个受控关节不意味着仅有 19 个身体点，头部扩展点也不对应一个必须独立控制的头部电机。

## 7 MuJoCo 控制时序

### 7.1 两层循环

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| policy.freq | 50 Hz | 策略期望更新频率 |
| sim_dt | 0.001 秒 | 单次物理步推进的仿真时间 |
| sim_decimation | 20 | 一次环境 step 中的物理步数 |
| control_dt | 0.02 秒 | sim_dt 乘 sim_decimation |

```text
策略推理一次，得到目标角度
  -> 用最新关节状态计算 PD 力矩
  -> 力矩裁剪
  -> mj_step 推进 1 毫秒仿真
  -> 更新关节状态
  -> 重复，共 20 个小步
更新完整状态，进入下一轮策略推理
```

20 个物理小步中保持不变的是目标角度，力矩会根据新反馈反复计算。

### 7.2 PD 力矩

```text
torque = Kp * (q_target - q_actual) - Kd * dq_actual
```

假设左膝目标 1.01 rad、当前角度 0.90 rad、速度 0.50 rad/s，则力矩为 `200 * 0.11 - 4 * 0.50 = 20 N·m`，随后还要经过力矩限制。

目标角度不会让关节瞬间到位；物理引擎根据力矩、惯量、重力和接触等推进状态。

### 7.3 仿真时间与现实时间

一次 mj_step 推进 1 毫秒仿真，不代表电脑耗时 1 毫秒。run_pipeline.py 测量一轮实际耗时，默认尝试补足到 20 毫秒。

若计算用了 8 毫秒，则尝试等待 12 毫秒；若用了 30 毫秒，则不再等待，仿真落后于现实时间。sleep 不会推进这段同步仿真。

run_fullspeed=True 跳过启动脚本的限速等待，但仍受推理、物理计算、可视化等开销影响。

### 7.4 参考动作时间

MotionCtrl 每次回调默认执行：

```text
motion_time += 0.02 * play_speed_ratio
```

倍率为 0 时参考时间停住，策略仍可继续控制机器人跟踪静止目标。本地 MotionH2HCtrl 设置初始播放倍率 0，并设置 speed_target=0.1；实际播放还受 GUI 和命令处理影响，不能直接假设按原速播放。

更改策略频率时，必须同时核对物理步长与小步数、参考时间推进、历史长度对应的实际时间以及动作平滑效果。

## 8 真机通信与状态估计

### 8.1 H1 当前硬件路径

H1RealEnvCfg 选择 UnitreeEnv，使用 unitree_sdk2py 相关通信接口。H1 配置的消息类型为 go；不要把其他型号的消息结构直接套用到此处。

```text
真实机器人
  -> LowState 状态消息
  -> 回调保存 self.low_state
  -> update 提取状态、重排、姿态处理与运动学计算
  -> env_data -> H2H 观测与策略 -> pd_target
  -> 电机映射、填写 LowCmd、CRC 校验、发布命令
  -> 真实机器人
```

默认状态话题为 rt/lowstate，命令话题为 rt/lowcmd。网卡名称由配置提供，当前 H1 示例为 eno2，需对应部署机器的实际接口。

状态接收与策略推理不同步；每轮读取的是缓存状态，策略频率不能证明数据每轮都新鲜。

### 8.2 位置命令字段

| 字段 | 当前位置控制分支写入 | 含义 |
| --- | --- | --- |
| q | pd_target 对应值 | 目标角度 |
| qd | 0 | 目标速度 |
| kp | 对应关节 stiffness | 位置增益 |
| kd | 对应关节 damping | 速度阻尼 |
| tau | 0 | 不额外指定前馈力矩 |

以上字段名称按当前代码描述；与所安装 SDK 的结构兼容性仍需验证。tau=0 不表示电机不出力，位置目标与增益仍参与底层控制。

MuJoCo 路径在 Python 中计算力矩；当前真机位置路径发送目标与增益。不能从这段代码推断硬件内层频率等于仿真中的 1000 Hz。

### 8.3 H1 姿态来源

H1 分支将 IMU 数据作为躯干姿态与角速度。消息四元数先调整到项目内部使用的排列；代码还结合腰部角度与速度，将躯干 IMU 数据转换到根部表达。

H2H 自身状态观测使用躯干信息，但任务观测仍使用根部位置和朝向，所以二者都需要正确。

### 8.4 DUMMY 位置的限制

当前 H1 真机配置默认 odometry_type=DUMMY，对应固定填入：

```python
base_pos = [0.0, 0.0, 0.8]
base_lin_vel = [0.0, 0.0, 0.0]
```

这是占位假设，不是真实位置测量。机器人移动后它不会自动反映位移。由于 H2H 任务观测使用根部位置及身体点，不能因为没有直接输入根部线速度，就认为策略不需要位置估计。

UNITREE 和 ZED 分支提供其他位置来源，但是否可用、如何对齐和延迟多大，需要针对实际部署核实。

### 8.5 准备过程与实时性

真机启动路径会调用 prepare，默认包含约 3 秒姿态过渡和约 5 秒策略目标混合。H2H 没有某些跟踪策略的 set_default_pose_mode，不能把通用日志中的“保持默认姿态”直接当成该策略一定具备的模式。

真实机器人不会等待 Python：推理或通信变慢时，物理世界继续运动。这与仅在 mj_step 时推进的同步仿真有本质区别。

## 9 迁移问题定位

### 9.1 查找最早出现异常的位置

```text
原始状态与参考数据
  -> env_data 和 ctrl_data
  -> obs
  -> 网络原始动作
  -> 平滑与缩放
  -> pd_target
  -> 实际关节运动
```

机器人摔倒是最终结果，不直接证明模型不行。先确认数据含义，再讨论增益、延迟和动力学。

### 9.2 分层核对

| 层次 | 主要核对内容 |
| --- | --- |
| 状态 | 关节对应、单位、正方向、姿态来源、数据新鲜程度 |
| 任务 | 跟踪点顺序、参考与机器人坐标一致、播放倍率 |
| 观测 | 1665 维各段语义、历史顺序、重置、非有限值 |
| 动作 | 原始输出、平滑参数、裁剪、缩放、默认角度 |
| 映射 | 策略到环境、环境到电机两个方向都正确 |
| 控制 | 增益、目标与实际响应、力矩限制、周期 |
| 动力学 | 质量惯量、接触、摩擦、执行器与延迟差异 |

相同维度不代表相同观测。可以用相同输入和相同内部状态比较两条部署路径的原始模型输出；有历史或状态的模型不能只对齐当前一帧。

### 9.3 现象与排查方向

| 现象 | 优先检查 |
| --- | --- |
| 目标先突然跳变 | 参考数据、观测、输出处理、切换分支 |
| 目标平滑但实际滞后 | 延迟、增益、力矩限制、负载和接触 |
| 目标稳定但关节振荡 | 增益、延迟、速度反馈、机械因素 |
| 多关节同时异常 | 公共坐标、映射、状态源或时序 |
| 重置后第一次行为不同 | 历史与 last_action、策略内部状态 |

这些是线索，不是仅凭现象就能确定的诊断结论。

### 9.4 项目日志

debug.log_obs 默认关闭。启用后，DebugLogger 保存运行配置，以及 env_data、ctrl_data、extras、pd_target、timestep 和记录时间。

当前日志没有直接保存最终 obs、网络原始输出、每个物理小步的力矩，也没有保证记录时间就是传感器采样时间。深入定位需要补充对应记录。

本轮 env_data 取自执行前，本轮 pd_target 是随后生成的目标。应结合下一轮及之后的状态分析响应，不能要求同一条记录中的实际角度立即等于目标。

scripts/view_log_msgpack.py 使用 G1 配置和固定索引，不能未经调整就当作 H1 通用分析工具。日志写入有额外开销，不能忽略其对时序测量的影响。

## 10 左膝端到端练习

以下假设已进入正常控制，不包含启动插值或淡出分支。

| 阶段 | 示例值 | 解释 |
| --- | --- | --- |
| 消息角度 | motor_state[5].q = 0.90 | 实际左膝角度 |
| 消息速度 | motor_state[5].dq = 0.50 | 实际左膝角速度 |
| 环境角度 | dof_pos[3] = 0.90 | 硬件映射后 |
| 观测角度 | obs[3] = 0.90 | 当前关节角度段 |
| 观测速度 | obs[22] = 0.50 | 19 + 3 |
| 上轮动作 | obs[74] = 0.20 | 71 + 3 |
| 本轮原始动作 | a_raw[3] = 1.00 | 假设模型输出 |
| 平滑动作 | 0.84 | 0.2 * 0.20 + 0.8 * 1.00 |
| 角度偏移 | 0.21 rad | 裁剪后乘 0.25 |
| 目标角度 | pd_target[3] = 1.01 | 加默认角度 0.80 |
| 真机目标槽位 | motor_cmd[5].q = 1.01 | 写回左膝电机 |
| 仿真首步力矩 | 20 N·m | 按当前示例状态计算 |

本轮观测读取旧 last_action=0.20；本轮推理后才保存新值 0.84。下一轮状态应反映实际响应，不要求瞬间达到 1.01。

自测问题：

1. 如果目标写到 motor_cmd[3]，是哪一层错误？
2. 如果目标变成 0.21，可能漏了什么操作？
3. 如果目标每轮都在实际角度上继续累加，是否符合当前策略接口？
4. 如果角度目标正确但实际明显滞后，能否立即断定需要增大 Kp？
5. 为什么 obs 维度正确，仍可能左右腿行为颠倒？

参考答案：第一题是硬件编号映射；第二题可能漏加默认姿态；第三题不符合，此处偏移相对固定默认姿态；第四题不能，还需查延迟、负载、限矩等；第五题是维度不包含每个位置的语义约束。

## 11 已知代码限制

| 位置 | 当前情况 | 学习与部署含义 |
| --- | --- | --- |
| h1_cfg.py | h1_real 默认是 Unitree 策略 | 不能直接当成 H2H 真机配置 |
| h2h_student_policy.py | reset 为 pass | 不会自动清空历史与动作记忆 |
| h1_real_env_cfg.py | 默认 DUMMY 位置 | 不提供真实根部位移 |
| unitree_env.py 的 H1 IMU 分支 | 使用 motor_state.q[...] 和 motor_state.dq[...] | 与其他位置的电机状态序列访问方式不一致，按序列结构可能报错，需结合 SDK 验证并修正 |
| unitree_env.py 的 step | 目标位置限位代码被注释 | 不能假定逐关节角度限位已经执行 |
| motion_ctrl.py | 参考时间步默认 0.02 | 改策略频率需要同步核对 |
| motion_h2h_ctrl.py | 本地播放速度有定制值 | 不应将原速作为默认假设 |
| debug_log.py | 未直接保存 obs 与原始动作 | 日志不足以覆盖每个检查点 |
| view_log_msgpack.py | 使用 G1 配置 | H1 分析需要适配 |

这些是当前代码阅读得到的限制或待验证项，不是完整代码审计报告，也不表示已经修复。硬件 SDK 字段、实际型号和运行状态必须匹配。

## 12 源码阅读索引

路径均相对项目根目录。导入 FlowUs 后可据此在本地编辑器中搜索，不依赖本机绝对路径链接。

| 路径 | 重点函数或类 | 阅读目的 |
| --- | --- | --- |
| scripts/run_pipeline.py | main、parse_args | 启动与实际耗时控制 |
| robojudo/config/config_manager.py | parse_config | 配置名称到对象 |
| robojudo/utils/module_registry.py | Registry | 名称注册与查找 |
| robojudo/config/h1/h1_cfg.py | h1_h2h、h1_real | 任务组合与区别 |
| robojudo/config/h1/policy/h1_h2h_policy_cfg.py | H1H2HDoF、H1H2HPolicyCfg | 关节顺序、默认姿态与增益 |
| robojudo/config/h1/ctrl/h1_motion_ctrl_cfg.py | H1MotionH2HCtrlCfg | 动作资源与 PHC 配置 |
| robojudo/config/h1/env/h1_env_cfg.py | H1DoF、H1EnvCfg | 环境关节与运动学 |
| robojudo/config/h1/env/h1_real_env_cfg.py | H1RealEnvCfg | 电机编号与位置来源 |
| robojudo/pipeline/rl_pipeline.py | PolicyWrapper、RlPipeline | 适配、循环与过渡 |
| robojudo/controller/ctrl_manager.py | CtrlManager | 输入组合与回调 |
| robojudo/controller/motion_ctrl.py | get_motion、post_step_callback | 动作读取与时间推进 |
| robojudo/controller/motion_h2h_ctrl.py | get_robot_state、get_data | 扩展身体点与任务数据 |
| robojudo/policy/h2h_student_policy.py | get_observation | 1665 维观测构造 |
| robojudo/policy/utils/utils_h2h.py | compute_imitation_observations_teleop_max_np | 27 维任务观测 |
| robojudo/policy/base_policy.py | get_action | 推理、平滑、裁剪、缩放 |
| robojudo/policy/policy_cfgs.py | H2HPolicyCfg | 模型路径与处理参数 |
| robojudo/tools/dof.py | DoFAdapter、merge_dof_cfgs | 映射与参数覆盖 |
| robojudo/environment/mujoco_env.py | update、step | 仿真状态与物理小步 |
| robojudo/environment/unitree_env.py | update、control_joints、set_cmd_i | 真机状态与命令 |
| robojudo/tools/debug_log.py | DebugLogger | 当前日志范围 |
| assets/robots/h1/h1.xml | 关节与执行器定义 | 仿真模型的顺序和物理参数 |
| third_party/phc/phc/data/cfg/robot/unitree_h1.yaml | extend_config | 手和头部扩展点 |

## 13 核心概念速查

| 概念 | 含义 |
| --- | --- |
| 策略 | 根据观测生成动作的已训练模型及其部署处理 |
| 参考动作 | 任务目标数据，不是必须直接发送的关节命令 |
| 观测 | 按训练约定整理的输入向量 |
| 原始动作 | 网络输出，尚未经过部署后处理 |
| 目标角度 | 动作处理后交给底层控制的关节目标 |
| 实际角度 | 仿真或硬件反馈的当前关节状态 |
| PD | 根据位置误差和速度计算控制作用 |
| FK | 从关节与根部状态计算身体位置和姿态 |
| 里程计或位置估计 | 提供机器人根部运动信息 |
| 关节映射 | 保持关节语义，改变数组中的存放位置 |
| 物理步长 | 单次仿真推进的时间 |
| 控制周期 | 两次策略更新之间期望的时间间隔 |

## 14 FlowUs 导入说明

本文使用 UTF-8 Markdown，包含分级标题、普通表格、引用、列表和代码块。流程采用纯文本，公式采用代码形式，不依赖 Mermaid、HTML 或特殊数学渲染扩展。

FlowUs 官网介绍支持 Markdown 本地文件导入。使用工作区中的导入功能选择本文件即可；具体入口名称以当前客户端为准。本文未在 FlowUs 账号内执行导入或验证导入后的页面外观。

文件内源码路径是可检索文本，不会在 FlowUs 中直接打开本机代码。表格列较多时可在导入后按阅读宽度调整布局。

格式依据：[FlowUs 官方网站](https://www.flowus.cn/)。
