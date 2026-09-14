# QuestArmTeleop OpenArm

基于 Meta Quest 和 ROS 2 Humble 的 OpenArm v1 双臂 VR 遥操作项目。通过 Quest 手柄控制机械臂末端的位置、姿态和夹爪，实现左右臂独立接管与双臂协同操作。

## 功能

- **双臂遥操作**：支持 OpenArm v1 左右两条 7 自由度机械臂。
- **相对位姿控制**：按住 Grip 接管对应机械臂，松开后保持位置，重新按下时重新建立控制基准。
- **夹爪比例控制**：左右 Index Trigger 分别控制对应夹爪，无需同时按住 Grip。
- **运动保护**：关节限位、命令限速、输入和反馈超时检测、故障保持及退出失能。
- **Docker 部署**：提供 ROS 2、IK 和 OpenArm 驱动的环境构建与启动脚本。
- **Fake hardware**：支持使用合成手柄输入检查完整控制链，无需连接机器人。

## 硬件与环境

| 项目 | 要求 |
|---|---|
| 机械臂 | OpenArm v1 双臂，每侧 7 个关节电机和 1 个夹爪电机 |
| VR 设备 | Meta Quest 头显及左右手柄，开启开发者模式和 USB 调试 |
| CAN 设备 | PEAK PCAN-USB Pro FD 双通道适配器 |
| 宿主系统 | Ubuntu 22.04、Docker Engine、ADB |
| 运行环境 | ROS 2 Humble、Python 3.10，由 Docker 镜像提供 |

默认硬件映射为左臂 `can0`、右臂 `can1`。启动脚本会检查适配器的 USB 身份和通道信息，具体接线要求见 [OpenArm 操作指南](OPENARM_V1_ROS2_USAGE.md#固定-can-身份)。

## 安装

### 1. 获取代码

```bash
git clone https://github.com/MADAO-00/QuestArmTeleop-openarm.git
cd QuestArmTeleop-openarm
```

### 2. 构建运行环境

确认 Docker 可用后，在项目根目录执行：

```bash
OPENARM_BUILD_IMAGE=1 OPENARM_BUILD_ONLY=1 bash ./run_openarm_v1.sh
```

构建脚本安装 ROS 2 和 Python 依赖，获取固定版本的 OpenArm 驱动并应用配套补丁。首次使用需要构建；镜像和环境未变化时，日常运行可直接进入启动步骤。

宿主工具安装、依赖说明和重建方法见 [环境部署指南](DEPLOYMENT.md)。

### 3. 安装 Quest 应用

通过 USB 连接 Quest，在头显中允许 USB 调试，然后执行：

```bash
adb devices -l
adb install -r src/oculus_reader/APK/teleop-debug.apk
```

设备列表中应只有一台 Quest，状态为 `device`。应用安装和签名冲突的处理方法见 [Quest 安装与启动](OPENARM_V1_ROS2_USAGE.md#quest-安装与启动)。

## 启动遥操作

以下命令在宿主机的项目根目录执行。运行环境已经准备好时，每次使用只需启动 Quest 应用和遥操作程序。

### 1. 启动 Quest 应用

```bash
adb shell am start -W -n com.rail.oculus.teleop/.MainActivity
```

戴上头显，关闭系统菜单，唤醒左右手柄并保持应用在前台。

### 2. 启动机械臂遥操作

确认双臂有可靠支撑、工作区域清空、急停可用，并松开两侧 Grip 和 Index Trigger：

```bash
OPENARM_MODE=real bash ./run_openarm_v1.sh
```

按终端提示核对现场条件后按 Enter。脚本会完成设备检查、CAN 配置和整机失能反馈预检，再启动双臂控制程序。

控制器启动后，保持两侧 Grip 和 Index Trigger 松开至少 0.5 秒。夹爪会自动张开到运行开位，等待终端提示对应侧 `startup/rearm gripper opening complete` 后开始操作。首次连接或调整坐标映射后，先小幅逐轴确认运动方向。

### 3. 手柄操作

| 操作 | 效果 |
|---|---|
| 按住左 / 右 Grip 并移动手柄 | 控制对应机械臂的末端位置与姿态 |
| 松开 Grip | 对应机械臂保持当前位置 |
| 重新按住 Grip | 从当前机械臂位置重新接管 |
| 扣动左 / 右 Index Trigger | 按行程比例闭合对应夹爪 |
| 松开 Index Trigger | 对应夹爪张开到 `0.040 m` 运行开位 |

夹爪可独立于 Grip 操作。当前 Quest 应用提供相对头显的手柄位姿，操作时应保持头显基准稳定；参考空间和坐标映射说明见 [手柄与参考空间](OPENARM_V1_ROS2_USAGE.md#手柄与参考空间)。

### 4. 停止

松开两侧 Grip 和 Index Trigger，在启动终端按 `Ctrl+C`，等待脚本完成整机失能后退出。异常情况使用硬件急停。

完整操作流程及故障排查见 [OPENARM_V1_ROS2_USAGE.md](OPENARM_V1_ROS2_USAGE.md)。

## 无硬件联调

环境构建完成后，可运行 fake-hardware 测试，检查手柄输入、IK、机械臂和夹爪控制：

```bash
OPENARM_MODE=fake OPENARM_NO_TTY=1 bash ./run_openarm_v1.sh \
  bash /workspace/scripts/smoke_openarm_v1_fake_in_container.sh
```

此测试使用合成输入，不需要 Quest 或真实 CAN。成功时输出 `OPENARM_FAKE_TELEOP_OK`。

## 项目结构

```text
.
├── src/oculus_reader/             # Quest 输入、OpenArm IK、命令保护、launch 和配置
├── assets/openarm_mujoco/         # OpenArm v1 模型与网格
├── scripts/                      # 环境加载、构建、CAN 预检与测试工具
├── patches/                      # OpenArm 驱动与 IK 补丁
├── tests/                        # 启动流程、预检与 vCAN 回归测试
├── run_openarm_v1.sh              # 启动入口，默认 fake 模式
├── Dockerfile.openarm-base        # ROS 2 与 Python 基础环境
├── Dockerfile.openarm-v1          # OpenArm 驱动与 IK 环境
├── openarm_v1.repos               # OpenArm 上游仓库与版本
└── requirements*.txt             # Python 依赖
```

控制流程：Quest 手柄 → 相对位姿目标 → 双臂 IK → 命令保护 → ros2_control → OpenArm CAN 驱动。ROS 包名为 `oculus_reader`。

## 文档

- [环境部署指南](DEPLOYMENT.md)
- [OpenArm 操作指南与故障排查](OPENARM_V1_ROS2_USAGE.md)
- [测试与环境验证记录](VALIDATION.md)
- [第三方来源与许可](THIRD_PARTY_NOTICES.md)
