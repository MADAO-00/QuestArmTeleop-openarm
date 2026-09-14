# QuestArmTeleop OpenArm v1

使用 Meta Quest 手柄遥操作 OpenArm v1 双臂与夹爪的独立 ROS 2 Humble 项目。
仓库包含 Quest 输入、双臂 IK、命令安全门、机器人模型、Docker 构建文件、CAN 预检与退出失能脚本。
全部项目路径都从本仓库根目录解析，可以独立 clone、构建和运行。

```text
Quest / OpenXR APK / USB ADB
  → 左右手柄位姿、Grip、Index Trigger
  → 原子目标 intent + 七轴双臂 IK
  → OpenArm command guard
  → ros2_control 机械臂/夹爪 position controllers
  → OpenArm 硬件插件 → can0 / can1
```

每侧为 7 轴机械臂（ID1–ID7）和 1 个夹爪电机（ID8）。Grip 控制对应机械臂的接管；Index Trigger 独立比例控制对应夹爪。默认运行 fake hardware，真机必须通过仓库根目录的 runner 启动。

## 开始使用

宿主环境与完整安装步骤见 [DEPLOYMENT.md](DEPLOYMENT.md)。在本仓库根目录执行：

```bash
# 构建独立基础镜像和带固定安全补丁的 OpenArm 镜像；不启动机器人。
OPENARM_BUILD_IMAGE=1 OPENARM_BUILD_ONLY=1 bash ./run_openarm_v1.sh

# 在 fake hardware 中验证完整 ROS 控制链。
OPENARM_MODE=fake OPENARM_NO_TTY=1 bash ./run_openarm_v1.sh \
  bash /workspace/scripts/smoke_openarm_v1_fake_in_container.sh
```

fake smoke 使用合成手柄输入，不需要 Quest 或 CAN 设备。成功时末尾输出
`OPENARM_FAKE_TELEOP_OK` 和 `OpenArm v1 fake-hardware ROS smoke test passed.`。
这不代表真实硬件已经通过验收。

真机接线、Quest APK 安装、无运动预检和日常操作见
[OPENARM_V1_ROS2_USAGE.md](OPENARM_V1_ROS2_USAGE.md)。
操作前阅读其中的固定 CAN 身份和启动夹爪自动张开说明。

## 项目文件

| 路径 | 用途 |
|---|---|
| `run_openarm_v1.sh` | 宿主机唯一真机入口；默认 fake |
| `Dockerfile.openarm-base` | 独立 ROS 2 Humble / Python 3.10 基础环境 |
| `Dockerfile.openarm-v1`、`openarm_v1.repos` | 固定上游版本、应用安全补丁、构建 OpenArm 驱动 |
| `requirements*.txt` | 基础、OpenArm IK 和开发测试 Python 依赖 |
| `src/oculus_reader/` | Quest reader、相对目标、IK、command guard 与 ROS 2 测试 |
| `assets/openarm_mujoco/v1/` | OpenArm v1 双臂 MuJoCo 模型与网格 |
| `patches/` | OpenArm CAN、硬件、描述模型、受约束 IK 的补丁及验证器 |
| `scripts/` | 容器环境、overlay、fake smoke、CAN 预检与失能流程 |

ROS 包名保留为 `oculus_reader`，因此 launch 命令仍使用这个包名。
模型随项目提供，运行不需要另一个工作区或额外的数据流项目。

## 当前边界

- Python requirements 只描述 Python 依赖；完整环境还包括 ROS 2 消息、ros2_control、OpenArm C++ 驱动与必需的受约束 IK 补丁。安装 requirements 后不能直接启动真机。
- 固定 APK 的手柄位姿是 HMD-relative。换轴与相对位姿算法不会把它变成 LocalFloor 世界稳定跟踪；头显移动仍可能影响手柄输入。
- 分离整理不会自动形成新的真机或全新机器安装验收记录。离线测试、fake ROS 测试和真机动作验证应分别记录。

来源与保留的许可说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
本次拆分的检查结果见 [VALIDATION.md](VALIDATION.md)。
