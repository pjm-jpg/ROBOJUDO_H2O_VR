# 19 自由度 H1：本机运行策略，命令行选择网卡

使用 `h2h_release.pt`，机器人 DDS 网卡通过 `--interface enp3s0` 指定，不需要机器人 IP；`--ip` 只表示 Quest 的无线 ADB 地址。接线后仍需正确的网络配置，以只读 DDS 检查能收到状态为准。程序不修改网卡配置，也不自动释放宇树运动服务。

**已完成本机 DDS 自动测试，尚未连接真实 H1 验证。** 首次测试应在吊装/支撑条件下逐阶段核对，另有现场人员负责遥控器和停止操作。VR 失联后的固定目标策略不保证真实机器人一定站稳。当前沿用 DUMMY 里程计，没有真实全局位置反馈。

## 1. 只读检查

Quest 通过 Wi-Fi 或 USB 接电脑，H1 用网线接电脑的 `enp3s0`。停止其他低层控制程序，按实际 H1 固件的流程进入调试状态。

```bash
cd /home/pjm/Desktop/RoboJuDo
conda activate robojudo
python -u scripts/run_quest_h2h_real.py --interface enp3s0 --duration 10
```

默认只读模式不创建电机命令发布器，不启动 Quest，不改变运动服务。输出按项目顺序的 19 个关节角、IMU wxyz、有效/拒绝帧数、频率和最大间隔，汇总显示 `command_messages_sent: 0`。检查失败时不要进入动作阶段。

在支撑状态下人工小幅移动单个关节，核对编号和方向。映射沿用 H1RealEnvCfg。部署限位按 XML 中的关节名称查询；旧 H1DoF 表的两侧肩 roll 限位与 XML 对调，因此没有直接使用旧表。真实零位、方向、限位和增益还需现场核对。

## 2. 带动作权限启动

确认 Quest ADB 为 device，将下方 IP 替换成头显当前地址：

```bash
cd /home/pjm/Desktop/RoboJuDo
conda activate robojudo
python -u scripts/run_quest_h2h_real.py \
  --interface enp3s0 --actuate \
  --ip 192.168.50.210 \
  --adb /home/pjm/Desktop/humanoid-projects/H2O-VR/.build-tools/android-sdk/platform-tools/adb
```

Quest USB 使用 `--serial 230YC01DB80182` 替换 `--ip ...`。实机时不运行 MuJoCo 模拟端，不加 `--dds-sim`。

启动先进入 WAIT：硬件命令发送被门控。键盘切英文输入法，按键松开时触发。

| 按键/阶段 | 行为 |
| --- | --- |
| WAIT | 接收反馈、Quest 和策略自检，不发送电机命令 |
| P | 检查 MotionSwitcher 没有活动模式后解锁发送；约 5 秒过渡到初始姿态并渐增增益。必须保持支撑 |
| READY | 维持初始关节姿态，尚未启用平衡策略，继续保持支撑 |
| L | 从 READY 启动固定三点目标策略，默认一个策略周期内交给原策略输出，不额外削弱平衡动作。现场分阶段检查站立 |
| C | STAND/VR 下校准，操作员静止约 2 秒，不自动接管 |
| E | 校准有效、追踪正常、STAND 下接入 VR；从单手小幅动作开始 |
| 空格/L（策略运行中） | 停止 VR 接管，约 2 秒返回原固定三点任务，策略继续运行，不冻结机器人 |
| 空格（准备中/READY） | 中断准备，保持当时关节姿态；仍需支撑，P 可重新准备 |
| X / Esc / 机器人 Select | 阻尼并退出。阻尼不能保持站立，可能倒下，需现场支撑 |
| Ctrl+C / 异常退出 | 清理时尝试阻尼；强杀进程或断网时不能保证命令送达 |

不提供 R 复位，真实机器人没有 reborn。运动服务仍活动或 CheckMode 失败时 P 拒绝执行；程序不调用 ReleaseMode。应由现场人员按本机 H1 的官方流程处理，再重启。

## 3. 中断行为

- 头显摘戴、失焦或追踪失效：退回固定目标策略，继续推理，平滑改变目标。恢复数据后不会自行接管，按提示 C、E。
- 状态超过 100 ms 未更新，或主循环两步间超过 150 ms：停止并在清理时尝试阻尼。发送线程目标超时也改发阻尼，避免无限重发旧位置命令。
- Select 在状态回调中锁存，发送线程独立于主策略循环处理。它依赖 DDS，不是独立硬件急停。
- 非有限状态/动作、倾角大于 0.7 rad 等触发退出；关节目标按 XML 限位。6 rad/s 目标变化约束仅用于准备/关节保持，不施加于平衡策略输出；VR 三点目标仍由原控制器平滑和限速。
- 命令线程间隔 2.5 ms，策略频率沿用 H2H 配置；Python/Linux 不保证硬实时。
- 低层状态检查消息头、CRC、数值和 IMU 四元数；无效包不刷新有效状态时间。

没有自动原厂平衡控制接管，也未验证硬件侧断网处理。真实驱动器响应、接触、限幅、固件和无线可靠性还需实机验收。

## 4. 先在 DDS MuJoCo 演练新的部署流程

终端 1：

```bash
cd /home/pjm/Desktop/RoboJuDo
conda activate robojudo
python -u scripts/run_h1_dds_sim.py
```

终端 2：

```bash
cd /home/pjm/Desktop/RoboJuDo
conda activate robojudo
python -u scripts/run_quest_h2h_real.py --dds-sim --actuate \
  --ip 192.168.50.210 \
  --adb /home/pjm/Desktop/humanoid-projects/H2O-VR/.build-tools/android-sdk/platform-tools/adb
```

P → 等待 READY → L → C → E。绿色球显示实际策略三点目标。仿真准备阶段物理暂停，近似外部支撑；L 后物理持续运行，VR 断流不会冻结物理。此简化不能验证真实吊装时的关节与增益过渡。

旧的 `run_quest_h2h.py` 仍是仿真专用 C/E 流程。新的 `run_quest_h2h_real.py` 使用 P/L/C/E 阶段。

## 自动测试

```bash
python -m unittest discover -s tests -p test_quest_h2h_deploy.py
```

只绑定回环 DDS 域 44，验证只读零命令、解锁门控、MotionSwitcher 拒绝、Select 锁存、阶段切换、VR 失联继续推理和不自动接管。VR 数据由测试生成，不涉及真实网卡或电机。

新增持续站立回归：使用默认 5 秒准备过程，L 后维持固定目标至少 30 秒，再校准、接入小幅 VR 动作并断流回退，持续检查根高度和倾角；报告写入 `/tmp/quest-h1-balance-validation.json`。该测试覆盖落地后的物理平衡，不仅检查消息或有限数值。

2026-10-22 的启动问题：初版把策略输出在 1 秒内逐渐混入，并按 6 rad/s 限制其 PD 位置目标，复现中会落地摔倒。PD 位置目标不是电机实际速度；这两层处理改变了学习策略的闭环控制。修正后 P 仍进行缓慢姿态/增益准备，L 默认在一个策略周期内启用策略，策略输出不再经过准备阶段的速率限制。DDS 演练和实机入口共用这一实现；实机物理验证仍未完成。
