#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 3 ]]; then
  echo "usage: $0 OPENARM_CAN_DIR OPENARM_ROS2_DIR OPENARM_DESCRIPTION_DIR" >&2
  exit 2
fi

openarm_can_dir="$1"
openarm_ros2_dir="$2"
openarm_description_dir="$3"

test "$(git -C "${openarm_can_dir}" rev-parse HEAD)" = \
  "c32ecd31da267967f0c913c2118c843177d88b91"
test "$(git -C "${openarm_ros2_dir}" rev-parse HEAD)" = \
  "4e837e1d0dae692ff67b560b69d8d281d7a8d4ed"
test "$(git -C "${openarm_description_dir}" rev-parse HEAD)" = \
  "6c7b720f1ba48e8bafa3a3dc752c45f397b42221"

git -C "${openarm_can_dir}" diff --check
git -C "${openarm_ros2_dir}" diff --check
git -C "${openarm_description_dir}" diff --check

test "$(git -C "${openarm_can_dir}" diff --name-only | sort)" = \
  "$(printf '%s\n' \
    include/openarm/damiao_motor/dm_motor.hpp \
    include/openarm/damiao_motor/dm_motor_control.hpp \
    include/openarm/damiao_motor/dm_motor_device_collection.hpp \
    python/src/openarm_can.cpp \
    src/openarm/can/socket/openarm.cpp \
    src/openarm/damiao_motor/dm_motor.cpp \
    src/openarm/damiao_motor/dm_motor_control.cpp \
    src/openarm/damiao_motor/dm_motor_device.cpp \
    src/openarm/damiao_motor/dm_motor_device_collection.cpp | sort)"
test "$(git -C "${openarm_ros2_dir}" diff --name-only | sort)" = \
  "$(printf '%s\n' \
    openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp \
    openarm_hardware/src/openarm_simple_hardware.cpp | sort)"
test "$(git -C "${openarm_description_dir}" diff --name-only | sort)" = \
  "$(printf '%s\n' \
    assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro \
    assets/robot/openarm_v1.0/urdf/robot/openarm_robot.xacro | sort)"

python3 - "${openarm_can_dir}" "${openarm_ros2_dir}" \
  "${openarm_description_dir}" <<'PY'
from pathlib import Path
import re
import sys

can_dir, ros2_dir, description_dir = map(Path, sys.argv[1:])


def read(root: Path, relative: str) -> str:
    return (root / relative).read_text(encoding="utf-8")


motor_hpp = read(can_dir, "include/openarm/damiao_motor/dm_motor.hpp")
motor_cpp = read(can_dir, "src/openarm/damiao_motor/dm_motor.cpp")
decoder_hpp = read(can_dir, "include/openarm/damiao_motor/dm_motor_control.hpp")
decoder_cpp = read(can_dir, "src/openarm/damiao_motor/dm_motor_control.cpp")
device_cpp = read(can_dir, "src/openarm/damiao_motor/dm_motor_device.cpp")
collection_hpp = read(
    can_dir, "include/openarm/damiao_motor/dm_motor_device_collection.hpp"
)
collection_cpp = read(
    can_dir, "src/openarm/damiao_motor/dm_motor_device_collection.cpp"
)
openarm_cpp = read(can_dir, "src/openarm/can/socket/openarm.cpp")
python_binding = read(can_dir, "python/src/openarm_can.cpp")

assert "uint64_t state_update_count() const" in motor_hpp
assert "uint64_t state_update_count_;" in motor_hpp
assert "int get_status() const { return status_; }" in motor_hpp
assert "bool is_enabled() const { return status_ == 1; }" in motor_hpp
assert "void set_status(int status);" in motor_hpp
assert "int status_;" in motor_hpp
assert "enabled_" not in motor_hpp
assert "state_update_count_(0)" in motor_cpp
assert "status_(-1)" in motor_cpp
assert "void Motor::set_status(int status)" in motor_cpp
assert "set_enabled" not in motor_cpp
assert "++state_update_count_;" in motor_cpp
assert "int status;" in decoder_hpp
assert "data[0] >> 4" in decoder_cpp
assert "response_motor_id = data[0] & 0x0F" in decoder_cpp
assert "response_motor_id != expected_motor_id" in decoder_cpp
assert device_cpp.count("motor_.set_status(result.status);") == 2
assert "set_enabled" not in device_cpp
assert '.def("get_status", &Motor::get_status)' in python_binding

assert "bool send_command_to_device" in collection_hpp
assert "bool DMDeviceCollection::send_command_to_device" in collection_cpp
for operation in ("enable", "disable", "refresh"):
    assert (
        f"Failed to send {operation} command to one or more motors"
        in collection_cpp
    )
assert "MIT parameter count must match motor count" in collection_cpp
assert "MIT control rejected; motor not in MIT mode" in collection_cpp
assert "Failed to send MIT control command to one or more motors" in collection_cpp
assert "apply_to_all_collections" in openarm_cpp
for method in ("enable_all", "disable_all", "refresh_all"):
    assert f"device_collection.{method}();" in openarm_cpp

hardware_cpp = read(
    ros2_dir, "openarm_hardware/src/openarm_simple_hardware.cpp"
)
hardware_hpp = read(
    ros2_dir,
    "openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp",
)
assert "bool runtime_feedback_bootstrap_pending_ = false;" in hardware_hpp
assert "bool bootstrap_runtime_feedback();" in hardware_hpp
assert "double last_accepted_gripper_position_ = 0.0;" in hardware_hpp
assert "double gripper_motion_time_credit_ = 0.0;" in hardware_hpp
assert "gripper_open_position_tokens_" not in hardware_hpp
assert "gripper_close_position_tokens_" not in hardware_hpp
assert "gripper_motion_direction_" not in hardware_hpp
assert "struct RuntimeTorqueExcursionState" in hardware_hpp
assert "bool over_limit_run = false;" in hardware_hpp
assert "std::array<RuntimeTorqueExcursionState, ARM_DOF>" in hardware_hpp
assert "void reset_runtime_torque_excursions() noexcept;" in hardware_hpp
assert "struct RuntimeGripperVelocityExcursionState" in hardware_hpp
assert "double peak_abs_velocity = 0.0;" in hardware_hpp
assert "runtime_gripper_velocity_excursion_{};" in hardware_hpp
assert "void reset_runtime_gripper_velocity_excursion() noexcept;" in hardware_hpp
assert "bool verify_gripper_mit_mode_ack();" in hardware_hpp

assert "constexpr double GRIPPER_LOWER_POSITION_LIMIT = 0.0;" in hardware_cpp
assert "constexpr double GRIPPER_UPPER_POSITION_LIMIT = 0.044;" in hardware_cpp
assert (
    "constexpr double GRIPPER_FEEDBACK_POSITION_TOLERANCE = 0.00002;"
    in hardware_cpp
)
assert "constexpr double MAX_GRIPPER_TRACKING_ERROR = 0.00504;" in hardware_cpp
assert (
    "constexpr double MAX_GRIPPER_OPEN_COMMAND_VELOCITY = 0.5000;"
    in hardware_cpp
)
assert (
    "constexpr double MAX_GRIPPER_CLOSE_COMMAND_VELOCITY = 0.5000;"
    in hardware_cpp
)
assert "MAX_GRIPPER_COMMAND_VELOCITY" not in hardware_cpp
assert (
    "GRIPPER_MODE_ACK_TIMEOUT = std::chrono::milliseconds(100)"
    in hardware_cpp
)
assert 'ee_type_ != "parallel_link"' in hardware_cpp
assert "Rejected non-finite or negative gripper gains" in hardware_cpp

on_init_start = hardware_cpp.index("OpenArmHW::on_init(")
configure_start = hardware_cpp.index("OpenArmHW::on_configure(", on_init_start)
on_init = hardware_cpp[on_init_start:configure_start]
init_gripper_sequence = [
    "init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE",
    "DEFAULT_GRIPPER_SEND_CAN_ID",
    "DEFAULT_GRIPPER_RECV_CAN_ID",
    "openarm::damiao_motor::ControlMode::MIT",
    "verify_gripper_mit_mode_ack()",
    "best_effort_disable();",
    "return CallbackReturn::ERROR;",
]
cursor = -1
for token in init_gripper_sequence:
    next_cursor = on_init.find(token, cursor + 1)
    assert next_cursor > cursor, f"gripper initialization missing/out of order: {token}"
    cursor = next_cursor

mode_ack_start = hardware_cpp.index("OpenArmHW::verify_gripper_mit_mode_ack()")
seed_commands_start_for_ack = hardware_cpp.index(
    "OpenArmHW::seed_commands_from_feedback(", mode_ack_start
)
mode_ack = hardware_cpp[mode_ack_start:seed_commands_start_for_ack]
mode_ack_sequence = [
    "RID::CTRL_MODE",
    "get_param(control_mode_rid) != -1.0",
    "GRIPPER_MODE_ACK_TIMEOUT",
    "openarm_->recv_all(FEEDBACK_RECV_SLICE_US)",
    "after.front().get_param(control_mode_rid)",
    "acknowledged_mode != expected_mode",
    "Verified gripper CTRL_MODE=MIT acknowledgement",
]
cursor = -1
for token in mode_ack_sequence:
    next_cursor = mode_ack.find(token, cursor + 1)
    assert next_cursor > cursor, f"gripper mode ACK missing/out of order: {token}"
    cursor = next_cursor

activate_start = hardware_cpp.index("OpenArmHW::on_activate(")
activate_end = hardware_cpp.index("OpenArmHW::on_deactivate(", activate_start)
activate = hardware_cpp[activate_start:activate_end]

assert "return_to_zero" not in activate
sequence = [
    "PREACTIVATION_DISABLE_ATTEMPTS",
    '0, "pre-activation disable", ACTIVATION_FEEDBACK_TIMEOUT',
    "[this]() { openarm_->disable_all(); }, true",
    "if (!disable_confirmed)",
    "seed_commands_from_feedback()",
    "preload_params.push_back({0.0, 0.0, pos_commands_[i], 0.0, 0.0})",
    '0, "disabled command preload", ACTIVATION_FEEDBACK_TIMEOUT',
    "openarm_->get_arm().mit_control_all(preload_params)",
    '1, "enable confirmation", ENABLE_FEEDBACK_TIMEOUT',
    "[this]() { openarm_->enable_all(); }",
    "seed_commands_from_feedback()",
    "{kp_[i], kd_[i], pos_commands_[i], 0.0, 0.0}",
    '1, "initial current-position hold", ACTIVATION_FEEDBACK_TIMEOUT',
    "openarm_->get_arm().mit_control_all(hold_params)",
    "runtime_feedback_watchdog_seeded_ = false",
    "runtime_feedback_bootstrap_pending_ = true",
]
cursor = -1
for token in sequence:
    next_cursor = activate.find(token, cursor + 1)
    assert next_cursor > cursor, f"activation sequence missing/out of order: {token}"
    cursor = next_cursor
assert activate.count("seed_commands_from_feedback()") == 2
assert activate.count("rclcpp::shutdown();") == 3
assert activate.count("best_effort_disable();") >= 3
assert activate.count("openarm_->enable_all();") == 1
assert activate.count("openarm_->get_gripper().mit_control_all(") == 2
assert "joint_to_motor_radians(pos_commands_[ARM_DOF])" in activate
assert "seed_runtime_feedback_watchdog()" not in activate
assert "catch (const std::exception& exception)" in activate
assert "catch (...)" in activate

assert "wait_for_fresh_feedback" not in hardware_cpp
assert "constexpr int FEEDBACK_DRAIN_MAX_PASSES = 5;" in hardware_cpp
assert "constexpr int FEEDBACK_DRAIN_SLICE_US = 1000;" in hardware_cpp
assert "constexpr int FEEDBACK_RECV_SLICE_US = 1000;" in hardware_cpp
assert "ACTIVATION_FEEDBACK_TIMEOUT = std::chrono::milliseconds(20)" in hardware_cpp
assert "ENABLE_FEEDBACK_TIMEOUT = std::chrono::milliseconds(100)" in hardware_cpp
assert "RUNTIME_FEEDBACK_MAX_AGE = std::chrono::milliseconds(20)" in hardware_cpp
assert "OpenArmHW::drain_feedback_until_quiet" in hardware_cpp
assert "openarm_->recv_all(FEEDBACK_DRAIN_SLICE_US);" in hardware_cpp
assert "OpenArmHW::run_feedback_transaction" in hardware_cpp
assert "bool send_disable_if_busy" in hardware_cpp
assert "send_once();\n    return false;" in hardware_cpp
assert hardware_cpp.count(
    "[this]() { openarm_->disable_all(); }, true"
) == 2
assert "send_once();\n  const auto deadline =" in hardware_cpp
assert "after.state_update_count() != before.state_update_count()" in hardware_cpp
assert "after.get_status() == expected_status" in hardware_cpp
assert "std::isfinite(after.get_position())" in hardware_cpp
assert "hand_ = false;" in hardware_cpp
assert "constexpr std::array<double, 7> MAX_ABS_FEEDBACK_VELOCITY" in hardware_cpp
assert (
    "15.079644737, 15.079644737, 4.900884539, 4.900884539,\n"
    "    18.849555921, 18.849555921, 18.849555921" in hardware_cpp
)
assert "1.57, 1.57, 3.14, 3.14, 12.6, 12.6, 12.6" not in hardware_cpp
assert (
    "constexpr double MAX_ABS_GRIPPER_FEEDBACK_VELOCITY = 18.849555921;"
    in hardware_cpp
)
assert (
    "constexpr double RUNTIME_GRIPPER_VELOCITY_HARD_LIMIT = 18.849555921;"
    in hardware_cpp
)
assert "MAX_ABS_GRIPPER_FEEDBACK_VELOCITY = 1.05" not in hardware_cpp
assert "RUNTIME_GRIPPER_VELOCITY_HARD_LIMIT = 1.20" not in hardware_cpp
assert (
    "constexpr uint64_t RUNTIME_GRIPPER_VELOCITY_MIN_FRESH_SAMPLES = 3;"
    in hardware_cpp
)
assert (
    "RUNTIME_GRIPPER_VELOCITY_MIN_DURATION =\n"
    "    std::chrono::milliseconds(20)" in hardware_cpp
)
assert "constexpr int MAX_FEEDBACK_TEMPERATURE = 70;" in hardware_cpp
assert "40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0" in hardware_cpp
assert "constexpr double MAX_ABS_GRIPPER_FEEDBACK_TORQUE = 7.0;" in hardware_cpp
assert "std::abs(after.get_torque()) <= max_abs_torque" in hardware_cpp
assert "constexpr double RUNTIME_TORQUE_CLEAR_FACTOR = 0.98;" in hardware_cpp
assert "constexpr double RUNTIME_TORQUE_HARD_FACTOR = 1.0;" in hardware_cpp
assert "constexpr uint64_t RUNTIME_TORQUE_MIN_FRESH_SAMPLES = 3;" in hardware_cpp
assert (
    "RUNTIME_TORQUE_MIN_DURATION = std::chrono::milliseconds(20)"
    in hardware_cpp
)
assert "RUNTIME_TORQUE_MAX_PENDING_DURATION" not in hardware_cpp
assert "pending_deadline_reached" not in hardware_cpp
assert "persistent_deadline" not in hardware_cpp
assert 'arm_prefix_ != "left_" && arm_prefix_ != "right_"' in hardware_cpp
official_hard_position_limits = {
    "LEFT_HARD_LOWER_POSITION_LIMITS": (
        -3.490659,
        -3.3161253267948965,
        -1.570796,
        0.0,
        -1.570796,
        -0.785398,
        -1.570796,
    ),
    "LEFT_HARD_UPPER_POSITION_LIMITS": (
        1.3962629999999998,
        0.17453267320510335,
        1.570796,
        2.443461,
        1.570796,
        0.785398,
        1.570796,
    ),
    "RIGHT_HARD_LOWER_POSITION_LIMITS": (
        -1.396263,
        -0.17453267320510335,
        -1.570796,
        0.0,
        -1.570796,
        -0.785398,
        -1.570796,
    ),
    "RIGHT_HARD_UPPER_POSITION_LIMITS": (
        3.490659,
        3.3161253267948965,
        1.570796,
        2.443461,
        1.570796,
        0.785398,
        1.570796,
    ),
}
for name, expected in official_hard_position_limits.items():
    match = re.search(
        rf"constexpr std::array<double, 7> {name} = \{{([^}}]+)\}};",
        hardware_cpp,
        re.DOTALL,
    )
    assert match is not None, f"missing official hard position limits: {name}"
    actual = tuple(float(value.strip()) for value in match.group(1).split(","))
    assert actual == expected, f"incorrect official hard position limits: {name}"
    assert hardware_cpp.count(name) == 3, f"hard limit is not used by both gates: {name}"
for legacy_name in (
    "LEFT_LOWER_POSITION_LIMITS",
    "LEFT_UPPER_POSITION_LIMITS",
    "RIGHT_LOWER_POSITION_LIMITS",
    "RIGHT_UPPER_POSITION_LIMITS",
):
    assert legacy_name not in hardware_cpp, f"legacy limit symbol remains: {legacy_name}"
assert "1.413, 1.413, 2.826, 2.826, 11.34, 11.34, 11.34" in hardware_cpp
assert "constexpr double POSITION_TOKEN_BURST_SECONDS = 0.020;" in hardware_cpp
assert "constexpr std::array<double, 7> MAX_TRACKING_ERROR" in hardware_cpp
assert "0.12, 0.12, 0.12, 0.12, 0.24, 0.24, 0.24" in hardware_cpp
assert "last_accepted_positions_[i] = position;" in hardware_cpp
assert "last_accepted_gripper_position_ = joint_position;" in hardware_cpp
assert "gripper_motion_time_credit_ = POSITION_TOKEN_BURST_SECONDS;" in hardware_cpp
assert "raw_joint_position, GRIPPER_LOWER_POSITION_LIMIT" in hardware_cpp
assert "command_gate_seeded_ = true;" in hardware_cpp
assert "bool OpenArmHW::best_effort_disable() noexcept" in hardware_cpp
assert '0, "fail-safe disable", ACTIVATION_FEEDBACK_TIMEOUT' in hardware_cpp
assert "Fail-safe disable was not confirmed" in hardware_cpp
assert "runtime_feedback_bootstrap_pending_ = false;" in hardware_cpp

transaction_start = hardware_cpp.index("OpenArmHW::run_feedback_transaction(")
seed_watchdog_start = hardware_cpp.index(
    "OpenArmHW::seed_runtime_feedback_watchdog(", transaction_start
)
feedback_transaction = hardware_cpp[transaction_start:seed_watchdog_start]
assert feedback_transaction.count(
    "std::abs(after.get_velocity()) <= max_abs_velocity"
) == 2
assert feedback_transaction.count(
    "std::abs(after.get_torque()) <= max_abs_torque"
) == 2
assert feedback_transaction.count("MAX_ABS_FEEDBACK_VELOCITY[i],") == 2
assert feedback_transaction.count(
    "MAX_ABS_GRIPPER_FEEDBACK_VELOCITY,"
) == 2
assert "velocity_limit=%.6f" in feedback_transaction

deactivate_start = hardware_cpp.index("OpenArmHW::on_deactivate(")
read_start = hardware_cpp.index("OpenArmHW::read(", deactivate_start)
deactivate = hardware_cpp[deactivate_start:read_start]
assert "if (!best_effort_disable())" in deactivate
assert "could not confirm status 0" in deactivate
assert "sleep_for" not in deactivate

write_start = hardware_cpp.index("OpenArmHW::write(", read_start)
read_method = hardware_cpp[read_start:write_start]
read_sequence = [
    "runtime_feedback_bootstrap_pending_",
    "bootstrap_runtime_feedback()",
    "collect_runtime_feedback()",
    "if (!feedback_valid)",
    "best_effort_disable();",
]
cursor = -1
for token in read_sequence:
    next_cursor = read_method.find(token, cursor + 1)
    assert next_cursor > cursor, f"runtime read sequence missing/out of order: {token}"
    cursor = next_cursor
assert "collect_runtime_feedback()" in read_method
assert "best_effort_disable();" in read_method
assert read_method.count("rclcpp::shutdown();") == 5
assert "OpenArm runtime read exception" in read_method
assert "OpenArm runtime read failed with an unknown exception" in read_method
assert "return hardware_interface::return_type::ERROR;" in read_method
assert "openarm_->refresh_all();" not in read_method
assert "run_feedback_transaction" not in read_method
assert "gripper_motors.size() != 1" in read_method
assert "std::isfinite(raw_joint_position)" in read_method
assert "GRIPPER_FEEDBACK_POSITION_TOLERANCE" in read_method
assert "pos_states_[ARM_DOF] = joint_position;" in read_method

bootstrap_start = hardware_cpp.index("OpenArmHW::bootstrap_runtime_feedback()")
runtime_start = hardware_cpp.index("OpenArmHW::collect_runtime_feedback(")
runtime_bootstrap = hardware_cpp[bootstrap_start:runtime_start]
bootstrap_sequence = [
    '1, "runtime feedback bootstrap", ACTIVATION_FEEDBACK_TIMEOUT',
    "[this]() { openarm_->refresh_all(); }",
    "seed_commands_from_feedback()",
    "seed_runtime_feedback_watchdog()",
    "runtime_feedback_bootstrap_pending_ = false",
]
cursor = -1
for token in bootstrap_sequence:
    next_cursor = runtime_bootstrap.find(token, cursor + 1)
    assert next_cursor > cursor, f"runtime bootstrap missing/out of order: {token}"
    cursor = next_cursor
assert runtime_bootstrap.count("openarm_->refresh_all();") == 1
assert "mit_control" not in runtime_bootstrap

seed_commands_start = hardware_cpp.index(
    "OpenArmHW::seed_commands_from_feedback(", runtime_start
)
runtime_feedback = hardware_cpp[runtime_start:seed_commands_start]
assert "openarm_->recv_all(0);" in runtime_feedback
assert "motor.get_status() == 1" in runtime_feedback
assert "now - next_times[i] > RUNTIME_FEEDBACK_MAX_AGE" in runtime_feedback
assert "openarm_->refresh_all();" not in runtime_feedback
assert runtime_feedback.count(
    "std::abs(motor.get_velocity()) <= max_abs_velocity"
) == 2
assert runtime_feedback.count("MAX_ABS_FEEDBACK_VELOCITY[i]") == 3
assert "feedback_values_except_velocity_valid" in runtime_feedback
assert "velocity_limit=%.6f" in runtime_feedback
assert "velocity_limit_ok=%s" in runtime_feedback
assert "velocity_gate_ok=%s" in runtime_feedback
assert "velocity_pending=%s" in runtime_feedback
assert "inspect per-axis" in runtime_feedback
assert "validation failed; max age is" not in runtime_feedback
assert "feedback_non_torque_values_valid" in runtime_feedback
assert (
    "std::abs(gripper.get_torque()) > MAX_ABS_GRIPPER_FEEDBACK_TORQUE"
    in runtime_feedback
)
assert "torque > torque_limit" in runtime_feedback
runtime_torque_sequence = [
    "feedback_stale",
    "feedback_non_torque_values_valid(",
    "if (torque > torque_hard)",
    "if (!fresh[i])",
    "if (!excursion.pending)",
    "torque > torque_limit",
    "if (torque <= torque_clear)",
    "if (torque <= torque_limit)",
    "excursion.over_limit_run = false",
    "if (!excursion.over_limit_run)",
    "excursion.started_at = now",
    "++excursion.fresh_samples",
    "excursion.fresh_samples >= RUNTIME_TORQUE_MIN_FRESH_SAMPLES",
    "now - excursion.started_at >= RUNTIME_TORQUE_MIN_DURATION",
]
cursor = -1
for token in runtime_torque_sequence:
    next_cursor = runtime_feedback.find(token, cursor + 1)
    assert next_cursor > cursor, f"runtime torque gate missing/out of order: {token}"
    cursor = next_cursor
for diagnostic in (
    "Runtime torque excursion pending side=%s can=%s",
    "Runtime torque excursion recovered side=%s can=%s",
    "Runtime torque trip reason=instant_hard side=%s can=%s",
    "Runtime torque trip reason=persistent side=%s can=%s",
):
    assert diagnostic in runtime_feedback
assert hardware_cpp.count("reset_runtime_torque_excursions();") >= 5

gripper_runtime_start = runtime_feedback.index("bool gripper_fresh = false;")
gripper_runtime_end = runtime_feedback.index("if (valid)", gripper_runtime_start)
gripper_runtime = runtime_feedback[gripper_runtime_start:gripper_runtime_end]
gripper_velocity_sequence = [
    "count != runtime_gripper_feedback_count_",
    "gripper_fresh = true",
    "feedback_stale",
    "!feedback_values_except_velocity_valid(gripper)",
    "MAX_ABS_GRIPPER_FEEDBACK_TORQUE",
    "if (velocity > RUNTIME_GRIPPER_VELOCITY_HARD_LIMIT)",
    "gripper_velocity_policy_ok = false",
    "valid = false",
    "else if (gripper_fresh)",
    "velocity <= MAX_ABS_GRIPPER_FEEDBACK_VELOCITY",
    "excursion = RuntimeGripperVelocityExcursionState{}",
    "else if (!excursion.over_limit_run)",
    "excursion.fresh_samples = 1",
    "++excursion.fresh_samples",
    "RUNTIME_GRIPPER_VELOCITY_MIN_FRESH_SAMPLES",
    "RUNTIME_GRIPPER_VELOCITY_MIN_DURATION",
    "valid = false",
]
cursor = -1
for token in gripper_velocity_sequence:
    next_cursor = gripper_runtime.find(token, cursor + 1)
    assert next_cursor > cursor, (
        f"runtime gripper velocity gate missing/out of order: {token}"
    )
    cursor = next_cursor
assert "RUNTIME_GRIPPER_VELOCITY_MAX_PENDING_DURATION" not in hardware_cpp
for diagnostic in (
    "Runtime gripper velocity excursion pending side=%s can=%s",
    "Runtime gripper velocity excursion recovered side=%s",
    "Runtime gripper velocity trip reason=instant_hard side=%s",
    "Runtime gripper velocity trip reason=persistent side=%s",
):
    assert diagnostic in gripper_runtime
assert hardware_cpp.count(
    "reset_runtime_gripper_velocity_excursion();"
) >= 5
assert runtime_feedback.count("gripper_velocity_policy_ok = false;") == 2
assert "runtime_gripper_velocity_excursion_.over_limit_run" in runtime_feedback

return_zero_start = hardware_cpp.index("OpenArmHW::return_to_zero(", write_start)
write = hardware_cpp[write_start:return_zero_start]
assert "!all_finite(pos_commands_)" in write
assert "best_effort_disable();" in write
assert "rclcpp::shutdown();" in write
assert write.count("return runtime_fault();") >= 9
assert "OpenArm runtime write exception" in write
assert "OpenArm runtime write failed with an unknown exception" in write
write_gate_sequence = [
    "!command_gate_seeded_",
    "period.seconds()",
    "pos_commands_[i] < lower_limits[i]",
    "vel_commands_[i] != 0.0 || tau_commands_[i] != 0.0",
    "std::abs(pos_commands_[i] - pos_states_[i]) > MAX_TRACKING_ERROR[i]",
    "next_tokens[i] + MAX_COMMAND_VELOCITY[i] * bounded_period",
    "std::abs(pos_commands_[i] - last_accepted_positions_[i])",
    "requested_step > next_tokens[i] + 1e-12",
    "position_tokens_ = next_tokens;",
    "last_accepted_positions_[i] = pos_commands_[i];",
    "mit_control_all(arm_params)",
]
cursor = -1
for token in write_gate_sequence:
    next_cursor = write.find(token, cursor + 1)
    assert next_cursor > cursor, f"write safety gate missing/out of order: {token}"
    cursor = next_cursor

gripper_write_gate_sequence = [
    "if (hand_)",
    "joint_names_.size() != ARM_DOF + 1",
    "pos_commands_[ARM_DOF] < GRIPPER_LOWER_POSITION_LIMIT",
    "pos_commands_[ARM_DOF] > GRIPPER_UPPER_POSITION_LIMIT",
    "vel_commands_[ARM_DOF] != 0.0 || tau_commands_[ARM_DOF] != 0.0",
    "std::abs(pos_commands_[ARM_DOF] - pos_states_[ARM_DOF])",
    "MAX_GRIPPER_TRACKING_ERROR",
    "next_gripper_motion_time_credit + bounded_period",
    "pos_commands_[ARM_DOF] - last_accepted_gripper_position_",
    "requested_delta > 0.0 ? MAX_GRIPPER_OPEN_COMMAND_VELOCITY",
    "requested_step / active_velocity",
    "requested_time > next_gripper_motion_time_credit + 1e-12",
    "next_gripper_motion_time_credit = std::max(",
    "next_gripper_motion_time_credit - requested_time",
    "gripper_motion_time_credit_ = next_gripper_motion_time_credit",
    "last_accepted_gripper_position_ = pos_commands_[ARM_DOF]",
    "openarm_->get_gripper().mit_control_all",
]
cursor = -1
for token in gripper_write_gate_sequence:
    next_cursor = write.find(token, cursor + 1)
    assert next_cursor > cursor, f"gripper write gate missing/out of order: {token}"
    cursor = next_cursor
assert "Discard both banks" not in write
assert "requested_direction" not in write

top_xacro = read(
    description_dir,
    "assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro",
)
robot_xacro = read(
    description_dir,
    "assets/robot/openarm_v1.0/urdf/robot/openarm_robot.xacro",
)
assert '<xacro:arg name="hand" default="true" />' in top_xacro
assert 'hand="$(arg hand)"' in top_xacro
assert "hand:=true" in robot_xacro
assert robot_xacro.count('hand="${hand}"') == 2
assert 'hand="true"' not in robot_xacro

print("OpenArm v1 safety patch static verification passed")
PY
