# 第三方来源与许可

本项目从原有 Quest/OpenArm 集成工作区中分离 OpenArm ROS 2 控制链。
下列来源说明用于保留原作者归属；各文件已有的版权头和许可文本继续适用。
本文件不为没有附带明确许可声明的内容新增授权。

| 保留内容 | 来源 / 归属 | 许可记录 |
|---|---|---|
| ROS 2 Quest reader 框架和公共输入工具 | 原集成工作区；其 README 指向 `agilexrobotics/QuestArmTeleop` | ROS 包 `package.xml` 声明 Apache-2；保留各源文件原有声明 |
| `src/oculus_reader/scripts/transformations.py` | Christoph Gohlke；The Regents of the University of California | 文件头保留完整 BSD 风格三条款许可和免责声明 |
| `src/oculus_reader/APK/teleop-debug.apk` | `jborbik/oculus_reader` 的 Quest 3/OpenXR 维护版，commit `9689484d319c4798e54d59509b192436647b7427` | 该 APK 沿用原集成工作区固定二进制；其来源与 hash 见操作指引，不将其他文件的许可推定为二进制全部组成部分的许可 |
| `assets/openarm_mujoco/v1/` | Enactic OpenArm MuJoCo，经原集成工作区保留的 OpenArm v1 模型 | [assets/openarm_mujoco/LICENSE](assets/openarm_mujoco/LICENSE)，Apache License 2.0 |
| 构建时获取的 OpenArm ROS 2、description、CAN | Enactic，上游地址和完整 commit 见 `openarm_v1.repos` | 上游检出中的许可及版权声明随构建保留 |
| 构建时安装的 `openarm-control` / `openarm-mujoco` 等 Python 包 | 各包发布者，版本见 requirements | 以安装包包含的许可声明为准 |

模型包含本控制链使用的 `openarm_left_hand_tcp` 和 `openarm_right_hand_tcp`。
软件参考系约定参考 `qrafty-ai/teleop_xr` 的 `v1.3.5`
（`7025347b520615d8483b75b4672560e719b938f0`）；本项目仍使用固定 APK 的 HMD-relative 输入。

`patches/` 保存针对固定上游的本地安全修改，Docker 构建会验证上游版本和补丁。
对上游重新分发时应保留其许可、版权声明以及已修改的说明。
