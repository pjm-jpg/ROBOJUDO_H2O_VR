# Quest Pro → H1 H2H 仿真控制

本入口使用 `assets/models/h1/h2h/h2h_release.pt`。Quest 提供三点目标，策略结合 MuJoCo 中的机器人状态输出 19 个关节目标。该入口只允许 MuJoCo，不会启动 Unitree 实机通信。

## 1. 环境和连接

这台电脑已有 `robojudo` Conda 环境，开发验证使用该环境中的 Python 3.11、PyTorch、MuJoCo 和项目自带的 `mujoco_viewer`。无需重新安装环境或转换策略。本机头显已更新为 **1.5-session-order / versionCode 6** APK，修复 Quest Pro 的 OpenXR READY/STOPPING 与 Android resume/pause 事件先后顺序引起的调试断言崩溃；其他设备建议使用该版本。接收协议最低兼容 versionCode 5。

```bash
conda activate robojudo
cd /home/pjm/Desktop/RoboJuDo
export PATH="/home/pjm/Desktop/humanoid-projects/H2O-VR/.build-tools/android-sdk/platform-tools:$PATH"
hash -r
```

Wi-Fi 已经启用且头显地址仍为 `192.168.1.117` 时：

```bash
adb connect 192.168.1.117:5555
adb devices -l
adb -s 192.168.1.117:5555 shell getprop ro.product.model
```

无线设备必须显示 `device`，最后一条应输出 `Quest Pro`。IP 是之前使用的地址；换 Wi-Fi 后需重新查询。

如果无线 ADB 尚未启用，先接 USB，在头显内允许调试：

```bash
adb devices -l
adb -s 230YC01DB80182 shell ip route
adb -s 230YC01DB80182 tcpip 5555
```

从 `wlan0 ... src` 读取头显 IP，替换下文地址，再执行 `adb connect IP:5555`。确认无线设备状态为 `device` 后再拔 USB。重启设备后可能需要重新启用无线 ADB。

## 2. 首次只检查目标球

```bash
python scripts/run_quest_h2h.py --ip 192.168.1.117 --preview-only
```

USB 替代命令：

```bash
python scripts/run_quest_h2h.py --serial 230YC01DB80182 --preview-only
```

当前优先使用上述 USB 路径排查与控制。头显改接主板直连口后，已通过 USB 连续检查 35 秒：接收 2510 帧，格式错误为 0，除启动等待外未出现持续断流。先退出原来的 Wi-Fi 入口，再启动 USB 入口；确认目标球预览后，去掉 `--preview-only`，重新 C 校准、E 启用即可在仿真中跟随。无线持续稳定性仍未通过验收。

程序自动启动头显的坐标应用并订阅三点数据，不需要另开旧的 `python -m oculus_reader.reader`。电脑会打开 MuJoCo 窗口；头显继续显示之前的坐标面板，机器人画面目前只显示在电脑上。

仿真启动后冻结在 XML 中的 H1 初始姿势。红球表示机器人当前左手、右手、头部的计算位置；绿球表示转换后的 VR 目标位置。校准前两者重合。

1. 戴好头显，完成地面/边界设置，拿起并唤醒两只手柄。VR 面板正常追踪时仅显示名称与 XYZ，不会额外写出 `TRACKED`；检查三点坐标持续更新，且没有 `[INFERRED]`、`NO TRACKING` 或 `PAUSED` 提示。终端单独运行三点接收器时才会明确打印 `TRACKED`。校准仍会严格检查协议中的追踪标志。
2. 面朝你打算作为“前方”的方向，平视，站稳。双手在身体两侧略向前自然持握，保持容易重复的中立姿势；不用强行模仿机器人关节角度。
3. 请同伴在电脑上按下并松开 **C**，然后保持三点和头部朝向稳定至少一秒。键盘按键在松开时生效。校准允许位置漂移最多 4 cm、朝向变化最多 10°，超出会重新计时。
4. 终端出现 `calibrated / preview` 表示完成。此刻绿球和机器人三点重合。
5. 分别小幅移动左手、右手，确认只改变对应绿球；整体前移、抬高或下蹲，确认三点变化方向正确。转头本身不应使目标围绕头部旋转。
6. `--preview-only` 下 E 不会启用物理仿真。用 Esc 或 Ctrl+C 退出。

按 C 时现在立即打印 `C received / calibration requested`。若显示 `cannot calibrate: no fresh data / disconnected`，先恢复头显坐标应用和数据流，再按 C；不能只靠反复按 C 完成校准。只有出现 `calibrated / preview` 后，绿色目标球才会跟随。此后若再次断流，校准会失效，需要重做。

键盘监听使用现有 pynput 控制器，需要图形桌面环境；应避免在其他输入框打字时操作。没有桌面或仅需检查接收链路时，可运行下面的被动模式；此模式没有校准/启用按键，也不会启动物理控制：

```bash
python scripts/run_quest_h2h.py --ip 192.168.1.117 --headless --no-keyboard --steps 250
```

## 3. 启用策略跟随

确认预览方向和幅度正确后，退出并去掉 `--preview-only` 重新启动：

```bash
python scripts/run_quest_h2h.py --ip 192.168.1.117
```

重新按 **C** 完成校准，保持中立姿势，再按 **E**。程序先重置策略历史，以当前关节角为起点，用约一秒时间过渡到策略输出。先验证单手小幅抬起、放下，再尝试另一只手和小幅下蹲，观察机器人实际动作。

| 按键 | 行为 |
| --- | --- |
| C | 暂停并重新采集初始姿势；完成后仍需 E |
| E | 已校准且数据有效时开始/恢复策略控制 |
| 空格 | 暂停物理仿真，保留校准，继续显示目标球 |
| R | 重置机器人为初始姿势、清除校准；需要重新 C、E |
| Esc / Ctrl+C | 退出并关闭接收器及仿真窗口 |

暂停会冻结 MuJoCo 的物理时间，机器人不会因重力继续倒下。这是仿真暂停功能，不能作为实机的失联处理方案。暂停期间请保持在原来的活动范围，恢复前回到合适姿势；若机器人已经倒地，使用 R 后重新校准。

## 4. 坐标如何对齐

接收协议保持 `STAGE` 地面空间和米制 FLU 坐标：X 前、Y 左、Z 上。输入顺序转换为策略要求的 **左手、右手、头部**。

校准同时记录人的三点、头部水平朝向和机器人当前三点、底座水平朝向，得到固定的绕 Z 轴旋转。人的初始三点分别对应机器人的初始三点，之后映射的是相对于校准姿势的变化。因此身高 1.7 m 的人不必迫使机器人把头移动到 1.7 m，也不必把手摆到机器人初始手位。

更精确地说，令 `R` 为校准时两者朝向之差，`h` 为头部位置，`p` 为某只手的位置，下标 `0` 表示校准值：

```text
目标头 = 机器人初始头 + head_scale × R × (人的当前头 - 人的初始头)
目标手 = 目标头 + (机器人初始手 - 机器人初始头)
       + hand_scale × R × [(人的当前手 - 人的当前头) - (人的初始手 - 人的初始头)]
```

默认两个比例均为 1。整体位移由头部传递给三点，手相对头的额外运动再单独映射。旋转 R 在校准后保持固定，后续转头不会重新定义前方。头部朝向只用于初始校准；此策略接口控制三点位置和速度，不直接控制头部朝向、手腕旋转或手指。

例如，保持头和右手不动，将左手向前移动 10 cm，左手目标就沿机器人校准前方移动 10 cm。跟随精度取决于策略能力、平衡和关节约束，不保证末端完全到达目标。

## 5. 数据保护和调节

- 收到有效但 `INFERRED` 的位置时，默认允许运行，以兼容已有手柄采集。校准时三点位置以及头部朝向都必须 `TRACKED`。运行时也要严格追踪可添加 `--require-tracked`。
- 任意一点无有效位置会暂停；有效性恢复后需手动 E，不会自动恢复。
- 超过 0.5 秒没有新接收帧、ADB 断开、应用失去焦点、参考空间或会话变化时，暂停并清除校准，恢复后重新 C、E。中途摘戴头显后建议主动重新 C。
- 任一点相对于机器人校准目标的偏移超过 1 m，清除校准并暂停。默认适合原地、小范围动作，长距离走动需要后续扩展。
- 启用控制时目标有 0.08 秒时间常数的平滑，每点最大目标速度为 1 m/s；送入策略的参考速度根据实际平滑位移计算。预览直接显示映射目标，便于核对方向。
- 数据新鲜度按电脑收到数据的时间判断，它不等于端到端延迟；当前没有测量真实 Wi-Fi 控制延迟。

如果 ADB 断线导致日志接收进程退出，需先重新 `adb connect`，再退出并重新启动仿真入口、重新 C 和 E；当前接收器不自动重连。只是短暂没有有效追踪、日志流仍保持连接时，可按上述按键恢复。

若一直显示 `no fresh data / disconnected`，不要反复按 C 或 E。`adb devices` 有时仍保留失效的 `device` 状态，要实际执行 `adb -s IP:5555 shell getprop ro.product.model` 检查响应；长时间不返回、超时或 `No route to host` 表示无线链路尚未恢复。确认两端 Wi-Fi，接回 USB 查看头显当前 `ip route`，重新启用 TCP ADB 后再连接。也可以先用 `--serial 230YC01DB80182 --preview-only` 验证 USB 下的完整校准流程，再切回无线；切换连接后需重启入口并重新校准。

本机还出现过电脑 Wi-Fi 显示已连接、却连路由器和头显都无法正常通信的情况；重新连接电脑的 Wi-Fi 后恢复。USB 与无线调试授权可能分别弹出，需要在头显内允许。恢复无线后曾连续 30 秒收到 2147 帧，格式错误为 0；通信恢复不等于三点始终 `TRACKED`，校准时仍须拿稳两只手柄并检查追踪状态。

因为重连后又复发，本机 `403_4G 1` 连接已关闭 Wi-Fi 省电：`802-11-wireless.powersave` 从 0（默认）改为 2（禁用），重新连接后 `iw ... get power_save` 已确认 `off`。这是稳定性排查措施，尚不能单凭它确认网卡或驱动故障根因。数值定义参见 [NetworkManager 官方说明](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/settings-802-11-wireless.html)。如需恢复默认配置，可执行 `nmcli connection modify '403_4G 1' 802-11-wireless.powersave 0` 并重新连接。

后续 90 秒测试中，约第 26 秒再次断流，最终有效帧数停在 1854，所以省电关闭并未解决持续稳定性问题。USB 拓扑显示网卡位于 `1-7.1`，此前头显位于同一 Hub 的 `1-7.2`；内核日志记录了该头显端口反复断连、枚举失败及 `error -71`。下一步应换到主板直连 USB 口，并视情况更换头显数据线，再进行持续测试。这些证据尚不能确定是 Hub、线缆、网卡驱动还是其他具体硬件故障，不能将短时收到数据作为验收通过。

当接收持续超时，控制器每 5 秒打印 `Quest receiver` 诊断，按 C 失败时也会立即打印。`frames` 为累计有效协议帧，`last_frame_age` 为最后接收距今时间，`logcat_running` 仅表示本地日志进程还存在，不证明网络畅通。连接恢复后，退出旧入口再重新启动，重新校准。

手部动作偏大时，可以重新启动并减小比例，例如：

```bash
python scripts/run_quest_h2h.py --ip 192.168.1.117 --hand-scale 0.7 --head-scale 0.8
```

录制原始三点及电脑接收时间：

```bash
python scripts/run_quest_h2h.py --ip 192.168.1.117 --record logs/quest/session.jsonl
```

录制文件采用追加写入，包含未校准帧；它不是机器人关节轨迹或已经转换的控制目标。程序复用 `../humanoid-projects/H2O-VR/oculus_reader-main/oculus_reader/three_point_reader.py`，若移动目录请用 `--reader-root /新路径/oculus_reader-main` 指定。接收器默认使用 H2O-VR 中打包的新版 ADB，避免 Conda 内旧版 ADB 冲突；也可以用 `--adb /绝对路径/adb` 显式指定。

## 6. 已完成的验证与边界

使用现有 `robojudo` 环境运行：

```bash
python -m unittest discover -s tests -p test_quest_h2h.py
```

测试覆盖坐标旋转与比例、稳定姿势校准、重复帧、丢追踪、断流、焦点/参考空间变化、命令优先级，以及真实 `h2h_release.pt` 的 1665 维输入、19 维输出和 MuJoCo 物理步进。另用合成三点完成 10 秒仿真，包括静止和小幅左手抬升，机器人未倒地、状态保持有限。

已用隐藏的 OpenGL 窗口验证该入口的目标球绘制、鼠标旋转、平移和滚轮缩放。自带查看器的 `third_party/mujoco_viewer/mujoco_viewer/callbacks.py` 已兼容 MuJoCo 3.11 起移除相机移动函数 scene 参数的变化，并保留旧版调用方式；这是本机查看器子模块中的修改。回归测试为：

```bash
python -m unittest discover -s tests -p test_mujoco_viewer_callbacks.py
```

新版 APK 已通过 Wi-Fi 安装并实测接收：6 秒收到 381 帧，格式错误为 0，采样期间头部和双手均有效且被追踪。尚未完成真人校准后驱动目标球及机器人策略跟随的完整交互验收，也尚未测量真实无线延迟或验证 sim2real。请先完成上面的目标球预览，再启用仿真跟随。
