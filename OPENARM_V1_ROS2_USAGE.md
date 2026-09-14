# OpenArm v1 双臂 Quest 遥操作

本文件描述独立项目的日常使用。首次安装和构建先阅读 [DEPLOYMENT.md](DEPLOYMENT.md)。
所有宿主命令都从本仓库根目录执行，容器挂载点为 `/workspace`。
真机只能通过 `bash ./run_openarm_v1.sh` 启动。

## 控制链与运行模式

```text
Quest / 固定 OpenXR APK / USB ADB
  → pub_pose_openarm_v1.py
  → 左右 pub_delta_pose_openarm_v1.py：原子 intent / token
  → openarm_bimanual_ik_node.py：七轴双臂受约束 IK
  → openarm_command_guard_node.py：机械臂与夹爪命令安全门
  → 四个 forward-position controller + joint_state_broadcaster
  → GenericSystem（fake）或带补丁的 OpenArm 硬件插件（real）
```

每侧 ID1–ID7 是机械臂，ID8 是夹爪。Grip 只门控对应机械臂；Index Trigger 独立控制对应夹爪。
真机 HOME 强制禁用。启动 FAULT 健康解除后，夹爪先限速张开到运行开位 `0.040 m`，
再允许新的 Trigger 闭合输入。官方硬范围是 `0..0.044 m`，`0.044 m` 不是日常张开目标。

默认 `OPENARM_MODE=fake`。先运行不需要头显的合成输入 smoke：

```bash
OPENARM_MODE=fake OPENARM_NO_TTY=1 bash ./run_openarm_v1.sh \
  bash /workspace/scripts/smoke_openarm_v1_fake_in_container.sh
```

fake 检查 ROS 控制链、IK、安全门与 GenericSystem 反馈，不访问真实 CAN，也不证明真实动作已经验收。
本次项目分离不代表进行了新的真机验证。

## 固定 CAN 身份

| 侧别 | 接口 | `bus-info` | `dev_id` | 驱动 | 电机 |
|---|---|---|---|---|---|
| 左侧 | `can0` | `3-1:1.0` | `0x0` | `peak_usb` | ID1–ID7 + ID8 |
| 右侧 | `can1` | `3-1:1.0` | `0x1` | `peak_usb` | ID1–ID7 + ID8 |

两个接口对应同一只 PEAK PCAN-USB Pro FD 的两个通道，因此 `bus-info` 相同。
runner 固定这一物理身份、左右映射及 `1 Mbit/s / 5 Mbit/s` CAN-FD 仲裁/数据速率，
不接受用环境变量改写通道。换机、改 USB 端口或硬件型号后必须重新核对身份；
出现 mismatch 应停止并确认接线和审核配置，不能跳过检查。

## Quest 安装与启动

Quest 开启开发者模式，USB 连接宿主，在头显中允许 ADB 调试。
宿主检查应只有一条 USB 设备记录，状态为 `device`：

```bash
adb devices -l
sha256sum src/oculus_reader/APK/teleop-debug.apk
```

固定 APK 来自 `jborbik/oculus_reader` 的 Quest 3/OpenXR 维护版，commit
`9689484d319c4798e54d59509b192436647b7427`，SHA-256 为：

```text
6ddd90d8bced3a9533ae36099c950238fdb3ffd5feb5a6daa0afd63ac484cdb0
```

初次安装或相同签名升级：

```bash
adb install -r src/oculus_reader/APK/teleop-debug.apk
```

仅从旧 VrApi/不同签名 APK 迁移且安装提示签名冲突时，先卸载旧包再安装：

```bash
adb uninstall com.rail.oculus.teleop
adb install src/oculus_reader/APK/teleop-debug.apk
```

每次遥操作前将 App 拉到前台：

```bash
adb shell am start -W -n com.rail.oculus.teleop/.MainActivity
adb shell pidof com.rail.oculus.teleop
```

第二条应返回进程号。戴上头显，关闭 Universal Menu，唤醒并活动左右手柄。
APK 安装校验不保证 App 正在输出跟踪；头显休眠、App 后台或菜单占据前台会导致 source stale。
real runner 使用宿主 ADB server，并校验设备数量、授权状态和头显内 `base.apk` 的 hash。
第二台设备、`unauthorized` 或 `offline` 记录也会在进入 CAN 阶段前被拒绝。

## 真机启动顺序

### 1. 现场检查

- 双臂有可靠支撑；完整运动范围内无人、无障碍，急停可立即触及。
- 两侧夹爪清空，手指远离；启动解除 FAULT 后夹爪会自动张开。
- 左臂 `can0`、右臂 `can1`，电源和急停状态正确。
- 两侧 Grip 和 Index Trigger 全部松开。
- 旧 real/fake 容器、其他 ROS domain 91 控制节点和 `candump` 等 CAN receiver 已正常停止。

### 2. 可单独执行整机无运动预检

```bash
OPENARM_MODE=real OPENARM_REAL_PREFLIGHT_ONLY=1 bash ./run_openarm_v1.sh
```

逐项核对 runner 展示的现场条件，确认后直接按 Enter。非空输入或关闭输入取消启动。
预检不会启动 ROS，不发送 enable、set-zero、位置、速度或力矩命令。
它会配置 CAN-FD，对两侧 ID1–ID8 发送 disable，并要求两轮完整的新鲜 `status=0` 反馈。
disable 会释放电机扭矩，因此“无运动预检”仍要求支撑机械臂并清空夹爪。

成功日志包括：

```text
PASS OpenArm whole-robot no-motion preflight: can0/can1 ID1-8 disabled and two complete fresh feedback rounds received.
OpenArm real preflight-only mode passed; ROS was not started and no gate was created.
```

### 3. 启动遥操作

```bash
OPENARM_MODE=real bash ./run_openarm_v1.sh
```

runner 在接触 CAN 前完成源码、镜像/补丁证明、real overlay 指纹和 Quest/APK 检查。
现场确认后才进行固定 CAN 身份、整机失能反馈预检以及一次性 launch 门禁。
即使刚刚单独做过预检，日常 real 入口仍会再做同等级预检。
real 模式拒绝自定义附加命令；不要在容器内手动执行 real launch。

real 固定 `ROS_DOMAIN_ID=91`、`ROS_LOCALHOST_ONLY=1`。
镜像为 `questarm-openarm-v1:humble-standalone-1.0`，safety schema 为 v7；
源码配套 overlay 为本仓库下 `.openarm_v1_real_overlay_v4`。

### 4. 启动后验证

确认 `joint_state_broadcaster`、左右机械臂 position controller 和左右夹爪 position controller 均已激活。
两侧 Grip 和 Index Trigger 全部松开至少 `0.5 s`，等待锁存 FAULT 健康解除并进入 HOLD。
等待日志出现对应侧 `startup/rearm gripper opening complete` 后再按 Trigger。

先只接管一侧机械臂，另一侧 Grip 保持松开，依次用约 `2–3 mm` 小位移确认前后、左右、上下，
再小角度逐轴确认旋转。头显保持稳定，注意下文 HMD-relative 输入限制。
夹爪在完全清空时不按 Grip，分别小幅扣动左右 Index Trigger，确认比例闭合和松开后的张开。
方向、姿态或跟随异常时松开相应控制，必要时使用硬件急停。

### 5. 正常停止

先松开两侧 Grip 和 Index Trigger，在宿主 runner 终端按 `Ctrl+C`，等待最终整机失能：

```text
Running final whole-robot ID1-8 torque-disable on can0/can1 ...
PASS OpenArm whole-robot shutdown: can0/can1 ID1-8 disabled with fresh status=0 confirmation.
```

日常停止不要使用 `kill -9` 或直接关闭 Docker daemon。
软件退出钩子无法覆盖 SIGKILL、宿主掉电或内核崩溃，紧急情况依靠硬件急停。

## 手柄与参考空间

| 输入 | 行为 |
|---|---|
| 左/右连续 Grip | 按住接管对应机械臂；松开回 HOLD，重新按下独立重锚 |
| 左/右 Index Trigger | 不依赖 Grip，独立比例控制对应夹爪 |
| HOME | 真机禁用 |

Trigger 行程 `t∈[0,1]` 对应目标 `q_des=0.040*(1-t) m`：
`0/25/50/75/100%` 分别对应 `0.040/0.030/0.020/0.010/0 m`。
这是目标位置映射，最终命令仍受限速、跟踪反馈和故障门控制。

位姿换轴和相对公式参考 `qrafty-ai/teleop_xr v1.3.5`
（`7025347b520615d8483b75b4672560e719b938f0`）：

```text
A = [[0,0,-1],[-1,0,0],[0,1,0]]
p_H = A * p_APK                # (x,y,z) → (-z,-x,y)
R_H = A * R_APK * A^T
p_target = p_E0 + (p_H - p_H0)
R_target = (R_H * R_H0^T) * R_E0
```

每侧 Grip 接管时分别记录规范化手柄 `(p_H0,R_H0)` 与当前 TCP `(p_E0,R_E0)`。
重锚帧目标等于当前 TCP，不跳变；末端仍为 `openarm_left_hand_tcp` / `openarm_right_hand_tcp`。
只做一次换轴，不在下游重复变换。

固定 APK 上报的是 **HMD-relative** 手柄位姿，缺少 **LocalFloor** 世界稳定锚点。
矩阵换轴不能恢复缺失的跟踪空间，头显移动仍可能进入手柄增量。
因此当前实现不能宣称与 LocalFloor 端到端等价；逐轴观察应保持头显稳定，并区分头部移动和手柄移动。

## 夹爪运行合同

- 官方硬范围 `0..0.044 m`；日常运行开位与 rearm 命令目标都是 `0.040 m`。
- 张开、闭合命令速度上限均为 `0.5000 m/s`，100 Hz 对应每周期最多 `0.005 m`。
  `0.040 m` 全行程无反压名义时间是 `0.08 s`，不构成实际完成时延保证。
- 命令/反馈 tracking 硬门为 `0.00504 m`。完整 5 mm 步只剩 40 µm 余量；
  自适应 reserve 只在反馈已跟上时允许下一完整步，落后时 HOLD/backpressure。
- rearm 完成要求命令保持 `0.040 m`，左反馈至少 `0.0375 m`、右至少 `0.0355 m`，
  对应 Trigger 连续松开 `0.25 s`。左右容差分别为 `0.0025/0.0045 m`；
  右侧相对 tracking 门仍有 `0.00054 m` 余量。开始 rearm 后 `5 s` 未完成会锁存 FAULT。
- 方向反转先停止向新方向迈步，将领先于反馈的命令按现有限速回退到最新反馈完成刹车。
  此类对齐不改变已记录的用户运动方向。随后需连续三条新的 100 Hz 反馈确认
  command-feedback 进入 40 µm，且单步和累计旧向漂移均不超过 20 µm，才放行反向步。
  30 ms 观察窗长于底层 20 ms stale 门，旧缓存不能满足释放条件。
- 启动一直扣住 Trigger 不会让第一个动作直接闭合；必须完成松开、健康解除 FAULT 和 rearm。

## 机械臂、IK 与硬件保护

IK、command guard 和 ros2_control 主循环为 100 Hz。
机械臂命令速度上限为官方 IK cap 的 90%：
`[1.413, 1.413, 2.826, 2.826, 11.34, 11.34, 11.34] rad/s`。
J1–J4 tracking 门 `0.12 rad`，J5–J7 `0.24 rad`，底层位置令牌最多累积 20 ms。
反馈落后时反压会收紧实际推进速度。

IK 软工作区按官方每个硬限位端点相对零位精确乘 0.95；软饱和不锁存 FAULT，
已在软范围外时禁止继续向外，不能瞬间拉回。
command guard 和硬件插件仍用官方 100% 范围检查最终命令和反馈。
固定 `openarm-control==0.2.0` 已在 Docker 构建中补丁为只允许受约束求解，失败返回无解。

每次接管由 guard-anchor 对齐 IK 与 controller 保留的七轴命令，不是向反馈瞬时跳转。
IK 只从 guard 已实际发出的 ACK 命令继续，未确认时重发同一步，不暗中累计目标步进。
原子目标 intent 同时携带位姿、Quest 源时间戳和 token；
`/openarm/target_pose/left|right` 仅供调试，控制使用 `/openarm/target_intent/left|right`。
每侧目标最多保留 32 条、接收年龄不超过 0.30 s，双臂配对源时间差不超过 30 ms。
任一请求接管侧未就绪时，两臂 cohort 保持零机械臂命令的 HOLD，旧 token 组合的结果整体失效。

首条有效 IK 必须在 Grip 按下后 0.75 s 内通过 guard 实际提交。
接管中的短暂双臂同步等待也只能累计 0.75 s；超时锁存 FAULT。
正常 TELEOP 中、没有显式同步等待时，IK 断流按 0.30 s 门限失效。
故障后松开两侧 Grip 和 Index Trigger 至少 0.5 s，待健康恢复 HOLD 后再接管。

硬件激活先 disable 并读取全套反馈，以实测位置建立命令，再 enable 原位保持，不自动 HOME。
ID8 必须确认 MIT 模式回执。位置、命令步进、跟踪、状态、反馈年龄、温度和 NaN 等门失败时 fail-close。
机械臂 ID1–ID7 反馈力矩 gross 门为 `[40,40,27,27,7,7,7] N·m`，ID8 为 `7 N·m`；
新鲜样本严格超过该峰值即 trip。这些是异常反馈门，不是人体碰撞阈值。

反馈 gross overspeed 门与命令速度 cap 分开：

| 电机 ID | 反馈绝对速度 gross 门（rad/s） |
|---|---:|
| ID1–ID2 | 15.079644737 |
| ID3–ID4 | 4.900884539 |
| ID5–ID8 | 18.849555921 |

这些值沿用源实现按对应 BOM 电机名义 24 V 空载转速的 90% 计算的策略，
在激活、使能/失能反馈事务和运行期按严格单样本判定，没有速度 debounce 宽限。
空载转速不证明带载安全速度、碰撞安全或制动距离。
预检不读取电机真实型号、内部 VMAX 或证明解码量程匹配；preflight PASS 不能当作这些标定的实机验证。

## 排障与重建

| 现象 | 处理 |
|---|---|
| 镜像缺失、schema/patch SHA/静态证明失败 | 执行 `OPENARM_BUILD_IMAGE=1 OPENARM_BUILD_ONLY=1 bash ./run_openarm_v1.sh`；不要跳过证明 |
| real overlay 过期或非审核构建 | 正常停止容器，保留旧 overlay 为备份，按部署文档重建 |
| ADB 数量/授权失败 | 只保留一台状态为 `device` 的 Quest；处理额外 offline/unauthorized 记录 |
| incompatible APK | 校验本地 hash，按迁移步骤安装固定 APK |
| source stale、logcat 只有 `wE9ryARX: &` | 戴上头显、关闭菜单、唤醒两只手柄，确认 App 前台运行 |
| physical identity mismatch | 检查固定 USB/PEAK 通道身份和左右接线 |
| existing CAN receiver registered | 正常停止旧控制程序或 candump |
| missing fresh disabled motor feedback | 按报错侧别/ID 检查电源、急停、CAN-H/L、终端电阻及 ID8 支路 |
| Grip 按下后保持 HOLD | 首次 IK 与双臂同步准备期间允许 HOLD；超时后松开并重新接管，检查 source/anchor/ACK |
| `non-monotonic` / `unsafe guard acknowledgement` | 检查 IK 和 guard 是否来自同一最新 overlay，不能直接放大超时 |
| 夹爪暂时不响应 | 不要求 Grip；检查 rearm 完成日志、Trigger 连续值、ID8 反馈和反转 interlock |
| rearm 5 s 超时 | 查看 feedback、保留命令和反转等待状态；不要要求反馈精确到达 0.040 或 0.044 m |
| `outside the ... safety limit` | 官方硬限位已越界；保持失能和支撑，检查零位与机械姿态 |

完整依赖和构建路径见 [DEPLOYMENT.md](DEPLOYMENT.md)。
