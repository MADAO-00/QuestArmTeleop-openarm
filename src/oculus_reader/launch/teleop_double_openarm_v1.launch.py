"""Quest dual-hand teleoperation for an OpenArm v1 bimanual robot.

The control graph remains Quest -> official OpenArm IK -> command guard ->
ros2_control.  Hardware bringup is local so the reviewed command guard remains
the sole publisher to the two arm and two ID8 gripper controllers.
"""

from __future__ import annotations

import os
from pathlib import Path
import re
import stat

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


LEFT_CAN_INTERFACE = "can0"
RIGHT_CAN_INTERFACE = "can1"
REAL_PREFLIGHT_GATE = "/run/openarm-real/preflight.gate"
REAL_PREFLIGHT_TOKEN_RE = re.compile(r"[0-9a-f]{64}")
SAFETY_CRITICAL_RUNTIME_NODES = (
    "pub_pose",
    "left_delta",
    "right_delta",
    "bimanual_ik",
    "command_guard",
)


def _as_bool(raw_value: str, label: str) -> bool:
    value = raw_value.strip().lower()
    if value in {"true", "1", "yes", "on"}:
        return True
    if value in {"false", "0", "no", "off"}:
        return False
    raise RuntimeError(f"{label} must be true or false, got {raw_value!r}.")


def _require_real_runner_gate(environ=None) -> None:
    """Validate the short-lived, read-only token created by the host runner."""
    env = os.environ if environ is None else environ
    if env.get("OPENARM_HARDWARE_MODE") != "real":
        raise RuntimeError(
            "Real OpenArm launch requires OPENARM_HARDWARE_MODE=real from the runner."
        )
    if env.get("OPENARM_REAL_PREFLIGHT_PASSED") != "1":
        raise RuntimeError(
            "Real OpenArm launch requires OPENARM_REAL_PREFLIGHT_PASSED=1 from "
            "a successful host preflight."
        )
    if env.get("OPENARM_REAL_PREFLIGHT_GATE") != REAL_PREFLIGHT_GATE:
        raise RuntimeError(
            "Real OpenArm launch requires the fixed read-only preflight gate at "
            f"{REAL_PREFLIGHT_GATE}."
        )

    token = env.get("OPENARM_REAL_PREFLIGHT_TOKEN", "")
    if REAL_PREFLIGHT_TOKEN_RE.fullmatch(token) is None:
        raise RuntimeError(
            "OPENARM_REAL_PREFLIGHT_TOKEN must be a 64-character lowercase hex nonce."
        )

    gate_path = Path(REAL_PREFLIGHT_GATE)
    try:
        gate_stat = gate_path.lstat()
    except OSError as exc:
        raise RuntimeError("The real-hardware preflight gate is unavailable.") from exc
    if gate_path.is_symlink() or not stat.S_ISREG(gate_stat.st_mode):
        raise RuntimeError("The real-hardware preflight gate must be a regular file.")
    if stat.S_IMODE(gate_stat.st_mode) != 0o444:
        raise RuntimeError("The real-hardware preflight gate must have mode 0444.")

    try:
        gate_content = gate_path.read_bytes()
    except OSError as exc:
        raise RuntimeError("The real-hardware preflight gate cannot be read.") from exc
    if gate_content != token.encode("ascii") + b"\n":
        raise RuntimeError("The real-hardware preflight token does not match its gate.")


def _require_safe_configuration(context):
    """Fail closed before xacro expansion or ros2_control process creation."""
    left_can_interface = LaunchConfiguration("left_can_interface").perform(context).strip()
    right_can_interface = LaunchConfiguration("right_can_interface").perform(context).strip()
    if (left_can_interface, right_can_interface) != (
        LEFT_CAN_INTERFACE,
        RIGHT_CAN_INTERFACE,
    ):
        raise RuntimeError(
            "OpenArm v1 CAN mapping is fixed to left=can0 and right=can1; "
            f"got left={left_can_interface!r}, right={right_can_interface!r}."
        )

    use_fake_hardware = _as_bool(
        LaunchConfiguration("use_fake_hardware").perform(context),
        "use_fake_hardware",
    )
    enable_home = _as_bool(
        LaunchConfiguration("enable_home").perform(context), "enable_home"
    )
    start_quest_publisher = _as_bool(
        LaunchConfiguration("start_quest_publisher").perform(context),
        "start_quest_publisher",
    )
    if not use_fake_hardware:
        _require_real_runner_gate()
        if not start_quest_publisher:
            raise RuntimeError(
                "The real OpenArm v1 must start its reviewed Quest publisher."
            )
        if enable_home:
            raise RuntimeError(
                "HOME is not commissioned for the real OpenArm v1 and must remain disabled."
            )
    return []


def _shutdown_when_process_exits(label: str):
    """Treat loss of the shared controller manager as a whole-cell stop."""

    def _handler(event, _context):
        return [
            EmitEvent(
                event=Shutdown(
                    reason=f"Safety-critical process {label} exited "
                    f"with code {event.returncode}."
                )
            )
        ]

    return _handler


def _shutdown_when_process_fails(label: str):
    """Shut down the full graph if a one-shot controller spawner fails."""

    def _handler(event, _context):
        if event.returncode == 0:
            return []
        reason = f"Safety-critical process {label} failed with code {event.returncode}."
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]

    return _handler


def _generate_robot_description(
    context, use_fake_hardware, left_can_interface, right_can_interface
) -> str:
    description_share = Path(get_package_share_directory("openarm_description"))
    xacro_path = (
        description_share
        / "assets"
        / "robot"
        / "openarm_v1.0"
        / "urdf"
        / "openarm_v10.urdf.xacro"
    )
    return xacro.process_file(
        str(xacro_path),
        mappings={
            "arm_type": "openarm_v1.0",
            "bimanual": "true",
            "ros2_control": "true",
            "use_fake_hardware": use_fake_hardware.perform(context).strip().lower(),
            "left_can_interface": left_can_interface.perform(context).strip(),
            "right_can_interface": right_can_interface.perform(context).strip(),
            "can_fd": "true",
            # Use the official OpenArm v1 DM4310 / ID8 hardware path.  The local
            # command guard applies the reviewed position, rate and freshness
            # gates before either gripper controller receives a command.
            "hand": "true",
        },
    ).toprettyxml(indent="  ")


def _spawn_safe_robot_nodes(
    context,
    use_fake_hardware,
    left_can_interface,
    right_can_interface,
    controllers_file,
):
    robot_description = _generate_robot_description(
        context, use_fake_hardware, left_can_interface, right_can_interface
    )
    # Keep the original multi-node YAML intact. Passing an unresolved
    # PathJoinSubstitution makes launch_ros materialize only the controller
    # manager section, so dynamically loaded ForwardCommandController nodes
    # lose their top-level ``joints`` parameters.
    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {
        "robot_description": ParameterValue(robot_description, value_type=str)
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )
    # Register before process creation so even an immediate plugin/configuration
    # failure tears down the complete bimanual graph.
    controller_manager_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager,
            on_exit=_shutdown_when_process_exits("controller_manager"),
        )
    )
    return [controller_manager_exit, robot_state_publisher, controller_manager]


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    left_can_interface = LaunchConfiguration("left_can_interface")
    right_can_interface = LaunchConfiguration("right_can_interface")
    enable_home = LaunchConfiguration("enable_home")
    launch_rviz = LaunchConfiguration("launch_rviz")
    start_quest_publisher = LaunchConfiguration("start_quest_publisher")

    package_share = FindPackageShare("oculus_reader")
    parameter_file = PathJoinSubstitution(
        [package_share, "config", "openarm_v1_teleop.yaml"]
    )
    controllers_file = PathJoinSubstitution(
        [
            package_share,
            "config",
            "openarm_v1_bimanual_controllers.yaml",
        ]
    )
    model_xml = PathJoinSubstitution(
        [package_share, "assets", "openarm_mujoco", "v1", "scene.xml"]
    )

    safe_robot_nodes = OpaqueFunction(
        function=_spawn_safe_robot_nodes,
        args=[
            use_fake_hardware,
            left_can_interface,
            right_can_interface,
            controllers_file,
        ],
    )

    pub_pose = Node(
        package="oculus_reader",
        executable="pub_pose_openarm_v1.py",
        name="pub_pose_openarm_v1_node",
        output="screen",
        parameters=[parameter_file],
        condition=IfCondition(start_quest_publisher),
    )

    home_override = {
        "enable_home": ParameterValue(enable_home, value_type=bool),
    }
    left_delta = Node(
        package="oculus_reader",
        executable="pub_delta_pose_openarm_v1.py",
        name="left_pub_delta_pose_openarm_v1",
        output="screen",
        parameters=[parameter_file, home_override],
    )
    right_delta = Node(
        package="oculus_reader",
        executable="pub_delta_pose_openarm_v1.py",
        name="right_pub_delta_pose_openarm_v1",
        output="screen",
        parameters=[parameter_file, home_override],
    )

    bimanual_ik = Node(
        package="oculus_reader",
        executable="openarm_bimanual_ik_node.py",
        name="openarm_bimanual_ik",
        output="screen",
        parameters=[parameter_file, {"model_xml": model_xml}],
    )

    command_guard = Node(
        package="oculus_reader",
        executable="openarm_command_guard_node.py",
        name="openarm_command_guard",
        output="screen",
        parameters=[parameter_file],
    )

    arm_origin_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_openarm_origin",
        output="screen",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.698",
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "world",
            "--child-frame-id", "arm_origin",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("openarm_description"), "rviz", "bimanual.rviz"]
            ),
        ],
        condition=IfCondition(launch_rviz),
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_forward_position_controller",
            "right_forward_position_controller",
            "left_gripper_forward_position_controller",
            "right_gripper_forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    controller_spawners = TimerAction(
        period=1.0,
        actions=[joint_state_spawner, motion_controller_spawner],
    )
    spawner_failure_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=_shutdown_when_process_fails("joint_state_broadcaster spawner"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=motion_controller_spawner,
                on_exit=_shutdown_when_process_fails(
                    "dual-arm/dual-gripper controller spawner"
                ),
            )
        ),
    ]
    safety_critical_runtime_nodes = {
        "pub_pose": pub_pose,
        "left_delta": left_delta,
        "right_delta": right_delta,
        "bimanual_ik": bimanual_ik,
        "command_guard": command_guard,
    }
    if tuple(safety_critical_runtime_nodes) != SAFETY_CRITICAL_RUNTIME_NODES:
        raise RuntimeError("The OpenArm safety-critical runtime node set is incomplete.")
    runtime_exit_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=_shutdown_when_process_exits(label),
            )
        )
        for label, node in safety_critical_runtime_nodes.items()
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Use mock hardware. false is accepted only through the gated "
                    "OPENARM_MODE=real runner path."
                ),
            ),
            DeclareLaunchArgument(
                "left_can_interface",
                default_value=LEFT_CAN_INTERFACE,
                description="Fixed left-arm SocketCAN interface.",
            ),
            DeclareLaunchArgument(
                "right_can_interface",
                default_value=RIGHT_CAN_INTERFACE,
                description="Fixed right-arm SocketCAN interface.",
            ),
            DeclareLaunchArgument(
                "enable_home",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Enable the existing zero-pose HOME action in simulation only. "
                    "Real mode always rejects true."
                ),
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                choices=["true", "false"],
                description="Start RViz in addition to the control graph.",
            ),
            DeclareLaunchArgument(
                "start_quest_publisher",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Start the physical Quest/ADB publisher. It may be disabled only "
                    "for fake-hardware tests that inject a synthetic Quest stream."
                ),
            ),
            OpaqueFunction(function=_require_safe_configuration),
            safe_robot_nodes,
            rviz,
            *spawner_failure_handlers,
            *runtime_exit_handlers,
            controller_spawners,
            pub_pose,
            left_delta,
            right_delta,
            bimanual_ik,
            command_guard,
            arm_origin_tf,
        ]
    )
