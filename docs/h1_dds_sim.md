# H1 实机 DDS 接口在 MuJoCo 中验证

新增的分阶段实机入口及其 DDS 演练步骤见 [H1 Quest 实机部署](h1_quest_real.md)。本文以下仍描述原 C/E 仿真流程。

## 实现范围

原有 `run_quest_h2h.py` 默认仍直接调用 MuJoCo。添加 `--dds-sim` 后，控制端复用真实的 `UnitreeEnv`、Unitree SDK、`go` 型 `LowCmd_` / `LowState_` 和 CRC，MuJoCo 在另一个进程模拟机器人端。

```text
Quest → QuestH2HCtrl → h2h_release.pt → UnitreeEnv
                                       │ rt/lowcmd
                                       ↓
                               独立 MuJoCo DDS 端
                                       │ rt/lowstate
                                       └────→ UnitreeEnv → 下一轮观测
```

默认 DDS 域 42，绑定 `lo` / `127.0.0.1`。仿真入口拒绝域 0 和硬件网卡；Quest 的无线 ADB 连接不受此限制。控制端与模拟端必须使用同一 DDS 域。一个域同时只启动一组模拟端和控制端，避免多个消息源互相干扰。

这能验证本机跨进程 DDS 发现、消息序列化、CRC、关节映射、状态观测、策略推理和命令执行。不能替代真实 H1 的网线、网卡、固件、驱动器或电机响应验证。

## 启动：两个终端

先按既有步骤将 Quest 连上热点并启用无线 ADB。头显 IP 会变化，下面的 `192.168.50.210` 只是已验证过的地址；应使用 `wlan0 ... src` 查询到的当前地址。

终端 1，启动模拟机器人：

```bash
cd /home/pjm/Desktop/RoboJuDo
conda activate robojudo
python -u scripts/run_h1_dds_sim.py
```

出现 `H1 DDS MuJoCo ready: lo, domain=42` 后，终端 2 启动控制程序：

```bash
cd /home/pjm/Desktop/RoboJuDo
conda activate robojudo
python -u scripts/run_quest_h2h.py \
  --dds-sim \
  --ip 192.168.50.210 \
  --adb /home/pjm/Desktop/humanoid-projects/H2O-VR/.build-tools/android-sdk/platform-tools/adb
```

使用 USB 的替代命令：

```bash
python -u scripts/run_quest_h2h.py --dds-sim --serial 230YC01DB80182 \
  --adb /home/pjm/Desktop/humanoid-projects/H2O-VR/.build-tools/android-sdk/platform-tools/adb
```

使用电脑键盘的 C 校准（静止约 2 秒）、E 启用、空格暂停、R 重置、Esc 退出控制端。终端 1 的模拟端仍会保留，按 Ctrl+C 关闭。先退出控制端，再关闭模拟端。需图形桌面，操作键盘前切换英文输入法。

DDS 模式的 MuJoCo 窗口显示三个绿色球，标记 LEFT、RIGHT、HEAD，代表策略使用的左手、右手和头部目标。校准后移动双手或头部即可观察，暂停时仍更新目标球。球仅用于显示，不参与碰撞或施力；显示数据通过独立 `robojudo/sim/targets` 话题传送，不改变实机命令。当前 DUMMY 里程计的根位置固定，因此显示位置按“策略目标 − 控制端根位置 + MuJoCo 根位置”投影，表示策略在当前机器人根部期望的三点位置。显示数据超过 0.5 秒未更新时隐藏球。`--preview-only` 可以禁止启用策略。头显内是否显示坐标面板不影响电脑端目标球；仍须确保采集应用在前台且持续发送有效数据。

不需要窗口时，在模拟端加 `--headless`；控制端加 `--no-keyboard` 只做被动接收，不可通过键盘校准/启用。控制端的 `--headless` 仅用于原直接仿真，DDS 模式窗口由模拟端负责。

如要改变测试域，两端一起修改：

```bash
python -u scripts/run_h1_dds_sim.py --domain 43
python -u scripts/run_quest_h2h.py --dds-sim --dds-domain 43 --ip 192.168.50.210
```

## 消息和物理执行

- 19 个关节按 `H1RealEnvCfg.joint2motor_idx` 映射到 20 槽的 Unitree 电机消息。状态反向映射，未使用槽保留默认值。
- 接收 `q`、`dq`、`kp`、`kd`、`tau` 和电机模式，验证头部、CRC、数值有限性及非负增益。按官方停止哨兵值禁用相应 PD 项；禁用的电机模式不产生命令力矩。
- 每个 1 ms 物理步计算 `tau = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff`，按 H1 配置的力矩上限裁剪。XML 的 position 执行器在模拟进程中转换为单位增益、零位置偏置的力矩执行器，避免重复计算位置力。
- 模拟端名义上每 2 ms 发布关节状态和 CRC，IMU 来自 XML 的躯干 site：四元数为 SDK 的 wxyz，角速度和加速度来自躯干传感器。这里的频率是调度目标，不是实时性保证。
- 控制端沿用当前实机配置的 `DUMMY` 里程计（根位置固定为 `[0,0,0.8]`、线速度为零）。没有偷偷使用 MuJoCo 根部真值；真实里程计接入仍是后续工作。
- 策略仍是原来的 `h2h_release.pt`，输入 1665、输出 19，未重新训练。

`robojudo/sim/control` 和 `robojudo/sim/status` 是额外的本机仿真管理通道，用于暂停物理、重置和确认执行结果，不属于真实 H1 协议。控制命令包含本次模拟端的 server_id 和请求 ID，防止旧请求重放；不用于替代关节 DDS 命令。

## 中断行为和边界

- 模拟端 0.25 秒没有有效 lowcmd：暂停物理，要求操作端明确恢复；新数据到达不会自行启动。
- 控制端 0.5 秒没有低层状态：报超时并退出控制；发送线程在状态或目标超时时发送阻尼命令，而非无限重发旧位置目标。
- Quest 断流、失焦、追踪失效等仍由原 VR 控制器处理；按其状态提示重新校准或恢复。
- “冻结物理”只是一种仿真行为，真实机器人不能这样暂停。当前 Quest 入口继续拒绝真实 Unitree 环境，实机部署仍需单独完成保护和启停验证。
- CRC 失败、非法消息不会刷新模拟端的有效命令时间。

## 依赖与自动验证

这台电脑已在 `robojudo` 环境补齐 `cyclonedds==0.10.2`、本地官方 SDK `unitree_sdk2py==1.0.1` 和 `opencv-python==4.11.0.86`。SDK 源码路径为 `/home/pjm/Desktop/rl_projects/unitree_sdk2_python`，提交 `ab0d8ae2fee8e3ec337ba941d5e7fd49de5fd30a`；未更改 SDK 源码。没有运行 sudo。

Cyclone Python 绑定使用本机已有 Unitree C++ SDK 的头文件和库编译，用户目录前缀 `/home/pjm/.local/opt/robojudo-cyclonedds` 指向 `/home/pjm/Desktop/humanoid-projects/RobotBridge/unitree_sdk2/thirdparty`。这些路径移动或删除后可能需要重装绑定。换电脑应先按 [官方 SDK 安装说明](https://github.com/unitreerobotics/unitree_sdk2_python) 安装匹配的 Cyclone DDS C 库和 Python 绑定，然后安装 SDK；这些可选依赖没有加入所有用户都必须安装的主 requirements。

```bash
conda activate robojudo
cd /home/pjm/Desktop/RoboJuDo
python -m unittest discover -s tests -p test_h1_dds_sim.py
python -m unittest discover -s tests -p test_quest_h2h.py
```

DDS 测试启动独立无窗口 MuJoCo 进程，经过实际 SDK 消息收发，检查 CRC、关节响应、IMU 变换、暂停/重置、状态中断、命令中断和策略闭环。VR 输入由测试生成，不要求头显在线，也不构成对本次真实无线链路的验收。

模拟端支持定时运行并保存汇总：

```bash
python -u scripts/run_h1_dds_sim.py --headless --duration 30 --report /tmp/h1-dds-report.json
```

汇总包含有效/拒绝命令数、状态发送数、命令超时次数、最大命令到达间隔和模拟时间；这些数据不能直接当作端到端延迟。
