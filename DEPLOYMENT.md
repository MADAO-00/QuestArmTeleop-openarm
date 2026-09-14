# OpenArm v1 独立环境部署

本仓库面向 OpenArm v1 双臂、两侧 ID8 夹爪和 Quest USB ADB 输入，使用 ROS 2 Humble。
下列命令都在本仓库根目录执行；容器内同一目录挂载为 `/workspace`。

## 1. 宿主环境

目标环境为 Ubuntu 22.04 Linux、可用的 Docker Engine，以及能运行 Linux 容器的权限。
真机使用 Linux SocketCAN；宿主机必须能识别固定的 PEAK PCAN-USB Pro FD 双通道设备。
Docker 镜像包含 ROS 2 与编译工具，宿主机无需先安装另一个 ROS 工作区或 Conda。

安装宿主工具并检查 Docker：

```bash
sudo apt-get update
sudo apt-get install -y git adb can-utils iproute2 ethtool util-linux
docker version
```

`docker version` 应同时显示客户端和服务端。首次构建需要访问容器镜像源、Ubuntu/ROS 软件源、
PyPI 和 `openarm_v1.repos` 中的 GitHub 上游。网络不可达会导致构建失败。

如果已经有项目目录，直接进入该目录。需要获取远程副本时：

```bash
git clone git@github.com:MADAO-00/QuestArmTeleop-openarm.git
cd QuestArmTeleop-openarm
```

仓库根目录应包含 `run_openarm_v1.sh`、两份 Dockerfile 和 `assets/`，对应本文的独立 OpenArm 部署结构。

## 2. 构建完整环境

```bash
OPENARM_BUILD_IMAGE=1 OPENARM_BUILD_ONLY=1 bash ./run_openarm_v1.sh
```

runner 构建并使用两个本项目镜像：

| 文件 | 默认镜像 | 作用 |
|---|---|---|
| `Dockerfile.openarm-base` | `questarm-openarm-base:humble` | 从 ROS 2 Humble / Jammy 建立编译、ROS control、ADB 和 Python 环境 |
| `Dockerfile.openarm-v1` | `questarm-openarm-v1:humble-standalone-1.0` | 拉取固定 OpenArm 上游、验证并应用补丁、编译 C++ 驱动和安装受约束 IK |

独立基础镜像中的 Python 3.10 环境是 `venv --system-site-packages`，
路径保留为 `/root/miniforge3/envs/vt` 以兼容现有脚本；它不要求安装 Miniforge，也不使用 `conda activate`。
容器环境脚本会加载 ROS/OpenArm underlay 并将该 Python 加入 `PATH`。

最终镜像使用 safety schema `questarm-openarm-v1-real-safety-v7`。
这个 schema 用于区分独立部署的镜像证明，不能用旧工作区的镜像或 marker 冒充。
上游版本记录在 [openarm_v1.repos](openarm_v1.repos)，Python 版本记录在：

- [requirements.txt](requirements.txt)：NumPy、SciPy、PyYAML 和 ADB Python 客户端；
- [requirements-openarm-humble.txt](requirements-openarm-humble.txt)：固定 OpenArm / MuJoCo / Mink 求解依赖；
- [requirements-dev.txt](requirements-dev.txt)：离线开发测试依赖。

`rclpy`、ROS 消息、`ros2_control` 和 OpenArm C++ 驱动不由 pip requirements 安装。
另外，`openarm-control==0.2.0` 必须应用本仓库的
`patches/openarm_control-constrained-ik.patch` 并运行验证器，保证受约束求解失败时返回无解。
完整 Docker 构建会执行该步骤；单独 `pip install` 得到的上游包不具备这项保证。

## 3. 首先运行 fake smoke

```bash
OPENARM_MODE=fake OPENARM_NO_TTY=1 bash ./run_openarm_v1.sh \
  bash /workspace/scripts/smoke_openarm_v1_fake_in_container.sh
```

fake smoke 会构建当前 ROS 包，启动 GenericSystem、合成 Quest 输入、IK 与 command guard，
验证启动 FAULT、Trigger 松开后的夹爪 rearm、左右夹爪独立比例控制、Grip 二次接管，以及机械臂和夹爪的命令/反馈对应关系。
不需要 Quest 或物理 CAN 设备；fake 容器使用独立网络并移除 CAN 网络配置权限，不暴露宿主机 CAN 接口。

成功日志应包括：

```text
OPENARM_FAKE_TELEOP_OK ...
OpenArm v1 fake-hardware ROS smoke test passed.
```

交互式联调入口：

```bash
OPENARM_MODE=fake bash ./run_openarm_v1.sh
```

只有完成 overlay 构建并加载它后，才可在 fake 容器中启动：

```bash
ros2 launch oculus_reader teleop_double_openarm_v1.launch.py
```

没有真实 Quest 数据时，正常输入链会保持 FAULT；自动 smoke 自带合成输入。
ROS 包名继续使用 `oculus_reader`。MuJoCo 模型来自仓库内的
`assets/openarm_mujoco/v1/scene.xml`，没有跨仓库路径依赖。

## 4. Quest 与真机

fake 检查通过后，再按 [OPENARM_V1_ROS2_USAGE.md](OPENARM_V1_ROS2_USAGE.md)
完成 APK 安装、手柄和参考空间检查、机械支撑、CAN 身份检查及失能反馈预检。

真机预检命令：

```bash
OPENARM_MODE=real OPENARM_REAL_PREFLIGHT_ONLY=1 bash ./run_openarm_v1.sh
```

真机操作命令：

```bash
OPENARM_MODE=real bash ./run_openarm_v1.sh
```

交互确认后才会进入 CAN 阶段。真机不接受自定义容器命令，不能绕过 runner 手动执行 real launch。
预检会发送整机 disable，支撑不足时仍可能释放承重扭矩；它不等于完全无硬件影响。

## 5. overlay 与重建

生成物都在本仓库内并被 Git 忽略：

| 目录 | 用途 |
|---|---|
| `.openarm_v1_overlay/` | fake 开发构建 |
| `.openarm_v1_real_overlay_v4/` | 真机使用的独立、非 symlink 审核构建 |
| `build/`、`install/`、`log/` | 手工 colcon 构建可能产生的本地输出 |

真实 overlay 在无网络、无 CAN 权限的容器中生成，真机 runner 校验其源码指纹。
改动 launch、控制代码、配置或模型后，fake 模式会重新生成匹配的 overlay；
real 模式还要求维护者完成回归并更新 runner 内的审核发布指纹，单独重建不会接受未审核源码。
改动镜像、安全补丁或镜像内审核脚本后，先重新执行第 2 节构建命令。
不要把另一个目录的 install 或 overlay 复制进来继续启动。

若 runner 报告已有 real overlay 过期，先正常停止所有本项目容器，
将 `.openarm_v1_real_overlay_v4` 改名为一个未使用的备份目录，再重跑预检入口。
不要手动修改指纹或删除审核检查。

## 6. 离线开发检查

只运行不连接 ROS/CAN 的核心逻辑测试时，可在宿主的 Python 3.10 虚拟环境中执行：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m pytest -q src/oculus_reader/test/test_openarm_teleop_core.py
```

这些依赖足以支持核心 Python 检查，不替代 Docker 内的 ROS launch、模型、驱动、补丁和 fake smoke 验证。
检查所有测试时应使用完整容器环境，并区分缺依赖导致的未执行和真正通过。

## 验证状态如何理解

上述步骤是独立项目的部署流程。源项目以往的实机或 fake 记录不自动成为独立镜像的验收结果。
本次文件拆分、语法检查或单元测试通过，也不能证明已在全新机器成功安装或已完成真机动作复验。
实际部署时应分别记录镜像构建结果、fake smoke 结果和现场动作结果。

本次拆分的具体检查结果及尚未完成的检查见 [VALIDATION.md](VALIDATION.md)。
