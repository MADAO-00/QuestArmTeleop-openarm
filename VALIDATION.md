# 独立 OpenArm 项目验证记录

原工程中的 OpenArm 遥操链路已在实际设备上正常使用。本页记录将其整理为独立项目后进行的构建和回归检查，以及新部署环境的验证进度。

日期：2026-09-14。分离来源为同机的 `QuestArmTeleop-ros2` 及工作区根目录的 OpenArm 部署材料。
源目录保持不变。新项目只包含 OpenArm ROS 2 链路、Quest 公共辅助代码、固定 APK、v1 模型和部署工具。

## 已完成

| 检查 | 结果 |
|---|---|
| OpenArm 核心逻辑 | 145 passed |
| ROS launch、输入、IK 与 command guard 合同 | 71 passed |
| 宿主 CAN 身份检查与整机无运动预检单元测试 | 25 passed，使用模拟数据 |
| 宿主 runner 行为、安全证明、拒绝篡改与退出流程 | 25 passed，使用模拟 Docker/ADB |
| Python 环境隔离与 colcon setup chain | 4 passed |
| 上述整套离线回归 | 270 passed，3 条上游 Python 弃用警告 |
| 新项目单独 colcon 构建 | 通过，只构建 `src/oculus_reader` |
| 安装后的 APK 和模型 | APK 存在；MuJoCo `scene.xml` 加载成功，`nq=18` |
| 完整 fake-hardware 遥操 | `OPENARM_FAKE_TELEOP_OK`，命令对应反馈，夹爪独立比例控制和 Grip 重复接管通过 |
| fake SIGINT 退出 | 四个 Python 进程干净退出，launch 返回 0 |
| 两份 Dockerfile 静态检查 | BuildKit 检查通过；使用本机镜像解析构建元数据，没有据此认定完整构建成功 |

集成构建与 fake 回归在本机现有 `questarm-openarm-v1:humble-real-0.6` 环境内运行，
仅挂载新项目至 `/workspace`，使用 `--network none`、只读源码和 `/tmp` 独立构建目录。
未挂载源项目、旧 overlay、USB 或宿主 CAN。现有镜像只用于此次验证，独立部署的默认镜像是
`questarm-openarm-v1:humble-standalone-1.0`。

## 尚未完成

- 全新基础镜像构建：已经尝试，从 Docker Hub 获取 `ros:humble-ros-base-jammy` manifest 时连接超时，未进入安装层。网络恢复后需按 [DEPLOYMENT.md](DEPLOYMENT.md) 完整构建，并在新镜像上再次运行 fake smoke。
- 物理 Quest、CAN 和机械臂动作：本次没有执行真实硬件预检或动作，没有新的真机验收结论。

独立基础镜像已显式包含预检需要的 `ethtool`；运行环境不再依赖原先的 Piper 镜像。
真机仍会校验 v7 镜像标记、基础镜像身份、脚本校验和、源码指纹和安装后模型内容。
移动文件或重新构建不会跳过这些检查。
