# H2H Actor 导出与模型选择

`model_30000.pt` 是训练检查点，包含 Actor、Critic、优化器等。当前部署通过 `torch.jit.load` 加载 Actor，不能直接选择训练检查点。原训练文件保留，导出为独立的 `model_30000_jit.pt`。

## 已完成的导出

- Actor：`1665 → 512 → 256 → 128 → 19`，隐藏层激活为 ELU。
- 根据检查点自带的 `training_config.policy` 重建 Actor，只加载 `actor.*` 参数。
- 导出确定性的 Actor，不包含 Critic、优化器或动作采样噪声。
- 零输入与 32 组固定随机输入验证：导出前后最大输出误差为 0。
- TorchScript 内附 `h2h_export_metadata.json`，记录源文件 SHA256、迭代号、网络维度等。

重新导出或导出其他匹配的检查点：

```bash
cd /home/pjm/Desktop/RoboJuDo
conda activate robojudo
python scripts/export_h2h_actor.py \
  --checkpoint assets/models/h1/h2h/model_30000.pt \
  --output assets/models/h1/h2h/model_30000_jit.pt
```

目标文件已存在时会拒绝覆盖。需要重新导出时选择新的输出名；不会改写源检查点或旧部署模型。

## 两个入口统一选择

`scripts/run_quest_h2h.py` 与 `scripts/run_quest_h2h_real.py` 都支持：

```text
--policy-name model_30000_jit
```

也接受 `model_30000_jit.pt`，参数只接受模型名，不接受目录路径。模型位于 `assets/models/h1/h2h/`；不传参数时仍使用 `h2h_release`。若误选 `model_30000`，在创建控制环境前提示先导出 Actor。只读状态检查不加载策略，也不受模型选择影响。

原 C/E 仿真流程：

```bash
python -u scripts/run_quest_h2h.py --dds-sim \
  --policy-name model_30000_jit --ip 192.168.50.210
```

P/L/C/E 部署流程的 DDS 演练：终端 1 启动模拟端：

```bash
python -u scripts/run_h1_dds_sim.py
```

终端 2 启动控制端：

```bash
python -u scripts/run_quest_h2h_real.py --dds-sim --actuate \
  --policy-name model_30000_jit \
  --ip 192.168.50.210 \
  --adb /home/pjm/Desktop/humanoid-projects/H2O-VR/.build-tools/android-sdk/platform-tools/adb
```

IP 使用 Quest 当前地址。切回旧模型只需改为 `--policy-name h2h_release`，或省略该参数。实机入口也接受相同参数；实机操作条件仍见 [实机部署说明](h1_quest_real.md)。

## 不用 VR：原有动作回放

原入口 `scripts/run_pipeline.py` 也支持 `--policy-name`，只对 H2H 策略生效。`h1_h2h` 使用本机 MuJoCo 和 `MotionH2HCtrl`，不连接 Quest、ADB 或 DDS，不向真实机器人发送指令。

```bash
cd /home/pjm/Desktop/RoboJuDo
conda activate robojudo
python -u scripts/run_pipeline.py --config h1_h2h --policy-name model_30000_jit
```

会打开 MuJoCo 和 `Motion Control Panel`。点击 **Fade In** 开始推进参考动作，**Fade Out** 暂停参考播放并继续运行 H2H 平衡策略，**Reset Progress** 把动作进度置零。键盘 `<`（英文输入，Shift+逗号）也可开始播放。启动时参考动作进度暂停，但策略与物理仿真已经运行。退出使用终端 Ctrl+C。

当前默认素材是 `assets/motions/h1/phc/singles/0-KIT_572_punch_right01_poses.pkl`，为约 5.23 秒的右手出拳动作。这里读取已经处理好的动作数据，不直接解析 MP4 视频。当前 `MotionH2HCtrl.speed_target` 保留项目已有的 `0.1`，点击 Fade In 后按 0.1 倍速播放，整段约需 52 秒。

对比旧策略：停止程序后，将命令末尾改为 `--policy-name h2h_release`，使用同一动作和控制参数比较站立、跟踪、抖动与跌倒情况。此入口不使用 VR 的 P/L/C/E 按键流程。运动测试结果见下方验证范围。

## model_30000 专用观测与动作处理

已找到该检查点对应的 `/home/pjm/Desktop/pjm_study_date/studentmodel/env_cfg.json` 和 `train_cfg.json`。原目录与部署目录的 `model_30000.pt` SHA256 均为 `c18868aab143f15e29d07fa887da99620e74b3feaafc3c768000eaa253359519`。

这份学生模型训练时使用骨盆坐标下的角速度与重力方向，且没有部署端的 EMA 动作滤波。沿用旧策略的躯干观测和 `action_beta=0.8` 会在静止参考下产生持续振荡。共享模型选择函数现在读取导出文件内的来源哈希，只对该检查点自动应用：

- `use_imu_torso=False`：当前帧和历史观测均使用骨盆角速度与重力方向。
- `action_beta=1.0`：使用确定性的 Actor 输出，取消额外的 EMA 滤波及其反馈延迟。

原动作回放、Quest 仿真与部署入口共用该选择函数。重命名导出文件仍可匹配来源；未知模型与旧 `h2h_release` 不套用该配置。模型权重、PD、动作缩放、动作限幅与三个关键点的几何定义均未改动。训练 PD 与关键点定义还存在其他差异，不能把这两项修正称为完整训练配置复刻。

回放控制同时修正了两个问题：H2H 的 Fade Out 只停止参考动作，持续运行平衡策略；Confirm 当前已加载的动作不再触发 Fade Out。Confirm 用于选择动作，启动播放仍使用 Fade In。其他类型策略保留原有 Fade Out 行为。

## 验证范围

导出前后 Actor 输出一致。此次修正额外对齐了该模型的骨盆观测来源与无 EMA 动作处理，保留观测排列、历史顺序、PD、缩放与限幅；没有更改实机 P/L/C/E 状态机与保护阈值。

新 Actor 与最终专用配置已用原 `h1_h2h` 动作控制器完成 CPU 无界面 MuJoCo 验证：45 秒静止、56 秒 0.1 倍速播放与 10 秒原速播放，均无数值异常，未触发测试的跌倒条件（根部高度低于 0.5 米或倾角超过 1 弧度）。三次最大倾角分别约 10.8°、7.1°、6.9°。另验证 28.06 秒的播放、重复 Confirm、暂停、恢复流程，平衡策略持续运行。测试关闭键盘与动作窗口，直接调用实际按钮回调；未验证 GUI 窗口操作、定量跟踪误差、其他动作、VR 跟随或真实机器人表现。

2026-10-26 减抖验证：固定参考、相同 MuJoCo 与初态、仿真 15 秒且排除前 2 秒的对比中，原配置关节目标逐帧变化 RMS 约 1.530°；骨盆观测且无 EMA 后约 0.011°。只修正骨盆而保留 EMA 虽能减少静止振荡，但慢速回放会跌倒，因此没有发布该中间配置。

最终配置保留原动作限幅，另完成 45 秒静止、56 秒 0.1 倍速播放、10 秒原速播放，三次均未达到测试跌倒条件；播放测试通过实际 Fade In 回调的队列与定时器启动。按钮测试直接调用回调，未打开 Tk 窗口。关节目标变化 RMS 是抖动诊断量，不是跟踪精度或实机安全证明。

原地站立优先的双手参考约束已默认启用；行为、范围、回退与 `--full-body` 对比选项见 [原地三点参考说明](stationary_h2h.md)。
