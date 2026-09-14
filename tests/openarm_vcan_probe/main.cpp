#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "openarm_hardware/openarm_simple_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace {

constexpr char kOnlyPermittedCanInterface[] = "vcan42";
constexpr int kGripperJitterPreloadCycles = 1;
constexpr int kGripperJitterReversalCycles = 12;
constexpr int kGripperJitterEndCycle =
    kGripperJitterPreloadCycles + kGripperJitterReversalCycles;
constexpr int kGripperTimeCreditRejectCycle = 2;
constexpr int kGripperTrackingFirstOverCycle = 1;
constexpr double kGripperCommandVelocity = 0.5000;
constexpr double kGripperTrackingSafeStep =
    kGripperCommandVelocity * 0.010;
constexpr double kGripperJitterReversalStep =
    kGripperCommandVelocity * 0.009;
constexpr double kGripperTrackingFirstOverPosition = 0.005041;

using CallbackReturn = hardware_interface::CallbackReturn;

bool parse_bounded_long_environment(const char* name, long default_value,
                                    long minimum, long maximum, long* value) {
  const char* raw_value = std::getenv(name);
  if (raw_value == nullptr) {
    *value = default_value;
    return true;
  }
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(raw_value, &end, 10);
  if (errno != 0 || end == raw_value || *end != '\0' || parsed < minimum ||
      parsed > maximum) {
    return false;
  }
  *value = parsed;
  return true;
}

void shutdown_ros() {
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

bool require_production_shutdown(const char* phase, int cycle) {
  // OpenArmHW's fail-closed read/write paths must request process-wide ROS
  // shutdown themselves.  Observe that state before this probe invokes its own
  // idempotent shutdown helper, otherwise the regression could accidentally
  // credit the harness for a production safety action that never happened.
  if (rclcpp::ok()) {
    std::cerr << "OPENARM_VCAN_RCLCPP_SHUTDOWN=ERROR phase=" << phase
              << " cycle=" << cycle << " production_did_not_shutdown\n";
    return false;
  }
  std::cout << "OPENARM_VCAN_RCLCPP_SHUTDOWN=YES phase=" << phase
            << " cycle=" << cycle << '\n';
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  // This helper is deliberately not a general CAN utility.  Keeping the test
  // interface hard-coded prevents an accidental invocation against can0/can1.
  if (argc != 1) {
    std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR unexpected arguments; this "
                 "probe only permits "
              << kOnlyPermittedCanInterface << '\n';
    return 64;
  }

  try {
    const char* expect_runtime_error_value =
        std::getenv("OPENARM_VCAN_EXPECT_RUNTIME_ERROR");
    const bool expect_runtime_error =
        expect_runtime_error_value != nullptr &&
        std::string(expect_runtime_error_value) == "1";
    const char* expect_bootstrap_error_value =
        std::getenv("OPENARM_VCAN_EXPECT_BOOTSTRAP_ERROR");
    const bool expect_bootstrap_error =
        expect_bootstrap_error_value != nullptr &&
        std::string(expect_bootstrap_error_value) == "1";
    if (expect_runtime_error && expect_bootstrap_error) {
      std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR conflicting error "
                   "expectations\n";
      return 65;
    }

    long post_activate_delay_ms = 0;
    long runtime_error_min_cycle = 1;
    long runtime_error_max_cycle = 2;
    if (!parse_bounded_long_environment(
            "OPENARM_VCAN_POST_ACTIVATE_DELAY_MS", 0, 0, 10000,
            &post_activate_delay_ms) ||
        !parse_bounded_long_environment(
            "OPENARM_VCAN_RUNTIME_ERROR_MIN_CYCLE", 1, 0, 29,
            &runtime_error_min_cycle) ||
        !parse_bounded_long_environment(
            "OPENARM_VCAN_RUNTIME_ERROR_MAX_CYCLE", 2, 0, 29,
            &runtime_error_max_cycle) ||
        runtime_error_min_cycle > runtime_error_max_cycle) {
      std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR invalid bounded test "
                   "environment\n";
      return 66;
    }
    std::string gripper_gate_mode;
    if (const char* mode_value =
            std::getenv("OPENARM_VCAN_GRIPPER_GATE_MODE")) {
      gripper_gate_mode = mode_value;
      if (gripper_gate_mode != "jitter_reversal" &&
          gripper_gate_mode != "time_credit_exhaustion_reject" &&
          gripper_gate_mode != "tracking_first_over_reject") {
        std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR invalid "
                     "OPENARM_VCAN_GRIPPER_GATE_MODE\n";
        return 67;
      }
    }
    rclcpp::init(argc, argv);

    hardware_interface::HardwareInfo info;
    info.name = "openarm_vcan_activation_probe";
    info.type = "system";
    info.hardware_class_type = "openarm_hardware/OpenArmHW";
    info.hardware_parameters = {
        {"can_interface", kOnlyPermittedCanInterface},
        {"arm_prefix", "left_"},
        {"hand", "true"},
        {"can_fd", "true"},
    };

    openarm_hardware::OpenArmHW hardware;
    if (hardware.on_init(info) != CallbackReturn::SUCCESS) {
      std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR on_init rejected\n";
      shutdown_ros();
      return 20;
    }

    auto command_interfaces = hardware.export_command_interfaces();
    hardware_interface::CommandInterface* gripper_position_command = nullptr;
    for (auto& command_interface : command_interfaces) {
      if (command_interface.get_name() ==
          "openarm_left_finger_joint1/position") {
        gripper_position_command = &command_interface;
        break;
      }
    }
    if (!gripper_gate_mode.empty() && gripper_position_command == nullptr) {
      std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR gripper position command "
                   "interface missing\n";
      shutdown_ros();
      return 68;
    }

    const rclcpp_lifecycle::State previous_state;
    if (hardware.on_configure(previous_state) != CallbackReturn::SUCCESS) {
      std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR on_configure rejected\n";
      shutdown_ros();
      return 21;
    }

    if (hardware.on_activate(previous_state) != CallbackReturn::SUCCESS) {
      std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR on_activate rejected\n";
      shutdown_ros();
      return 22;
    }

    std::cout << "OPENARM_VCAN_ACTIVATION=SUCCESS interface="
              << kOnlyPermittedCanInterface << '\n';
    if (post_activate_delay_ms > 0) {
      std::cout << "OPENARM_VCAN_POST_ACTIVATE_DELAY_MS="
                << post_activate_delay_ms << '\n';
      std::this_thread::sleep_for(
          std::chrono::milliseconds(post_activate_delay_ms));
    }

    // Exercise the production 100 Hz read/write cadence after activation.
    // Feedback is deliberately staggered across all eight motors, including
    // the gripper, so success proves the runtime path advances every per-axis
    // counter without weakening the 20 ms age watchdog.
    rclcpp::Clock clock;
    int runtime_writes = 0;
    for (int cycle = 0; cycle < 30; ++cycle) {
      double period_seconds = 0.010;
      if (gripper_gate_mode == "jitter_reversal") {
        if (cycle < kGripperJitterPreloadCycles) {
          period_seconds = 0.020;
        } else if (cycle < kGripperJitterEndCycle) {
          period_seconds =
              (cycle - kGripperJitterPreloadCycles) % 2 == 0 ? 0.009
                                                              : 0.011;
        }
      } else if (gripper_gate_mode == "time_credit_exhaustion_reject") {
        period_seconds = cycle <= kGripperTimeCreditRejectCycle ? 0.001
                                                                : 0.010;
      } else if (gripper_gate_mode == "tracking_first_over_reject") {
        period_seconds = 0.010;
      }
      const auto period = rclcpp::Duration::from_seconds(period_seconds);
      if (hardware.read(clock.now(), period) !=
          hardware_interface::return_type::OK) {
        if (expect_bootstrap_error) {
          if (!require_production_shutdown("bootstrap_read", cycle)) {
            (void)hardware.on_deactivate(previous_state);
            shutdown_ros();
            return 34;
          }
          if (cycle != 0 || runtime_writes != 0) {
            std::cerr << "OPENARM_VCAN_BOOTSTRAP=ERROR unexpected_phase cycle="
                      << cycle << " runtime_writes=" << runtime_writes << '\n';
            (void)hardware.on_deactivate(previous_state);
            shutdown_ros();
            return 29;
          }
          std::cout << "OPENARM_VCAN_BOOTSTRAP=EXPECTED_ERROR cycle=0 "
                       "runtime_writes=0\n";
          // read() has already issued fail-safe disable attempts.  Let any
          // scheduled replies settle before lifecycle deactivation confirms
          // the disabled state.
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
          if (hardware.on_deactivate(previous_state) !=
              CallbackReturn::SUCCESS) {
            std::cerr
                << "OPENARM_VCAN_DEACTIVATION=ERROR after_bootstrap_error\n";
            shutdown_ros();
            return 30;
          }
          std::cout << "OPENARM_VCAN_DEACTIVATION=SUCCESS\n";
          shutdown_ros();
          return EXIT_SUCCESS;
        }
        if (expect_runtime_error) {
          if (!require_production_shutdown("runtime_read", cycle)) {
            (void)hardware.on_deactivate(previous_state);
            shutdown_ros();
            return 35;
          }
          if (cycle < runtime_error_min_cycle || runtime_writes == 0) {
            std::cerr << "OPENARM_VCAN_RUNTIME=ERROR failure_before_runtime "
                         "cycle="
                      << cycle << " runtime_writes=" << runtime_writes << '\n';
            (void)hardware.on_deactivate(previous_state);
            shutdown_ros();
            return 31;
          }
          // At 100 Hz, a motor that stops replying immediately after the
          // bootstrap may survive one 10 ms read but must fail closed on the
          // next read once its age crosses the strict 20 ms watchdog.  No
          // third runtime command batch may be emitted.
          if (cycle > runtime_error_max_cycle ||
              runtime_writes > runtime_error_max_cycle) {
            std::cerr << "OPENARM_VCAN_RUNTIME=ERROR watchdog_late cycle="
                      << cycle << " runtime_writes=" << runtime_writes << '\n';
            (void)hardware.on_deactivate(previous_state);
            shutdown_ros();
            return 32;
          }
          std::cout << "OPENARM_VCAN_RUNTIME=EXPECTED_ERROR cycle=" << cycle
                    << " runtime_writes=" << runtime_writes << '\n';
          // read() has already issued its fail-safe disable attempts.  Allow
          // the deliberately staggered, already-scheduled feedback to settle
          // before asking lifecycle deactivation for an exact status-0
          // confirmation transaction.
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
          if (hardware.on_deactivate(previous_state) !=
              CallbackReturn::SUCCESS) {
            std::cerr << "OPENARM_VCAN_DEACTIVATION=ERROR after_runtime_error\n";
            shutdown_ros();
            return 28;
          }
          std::cout << "OPENARM_VCAN_DEACTIVATION=SUCCESS\n";
          shutdown_ros();
          return EXIT_SUCCESS;
        }
        std::cerr << "OPENARM_VCAN_RUNTIME=ERROR read cycle=" << cycle << '\n';
        shutdown_ros();
        return 24;
      }
      if (gripper_position_command != nullptr &&
          gripper_gate_mode == "jitter_reversal") {
        if (cycle < kGripperJitterPreloadCycles) {
          // A full 10 ms step at the reviewed 0.5000 m/s gate is still below
          // the unchanged 0.00504 m command-to-feedback tracking boundary.
          gripper_position_command->set_value(
              gripper_position_command->get_value() +
              kGripperTrackingSafeStep);
        } else if (cycle < kGripperJitterEndCycle) {
          // Alternate one 0.0045 m step across 9/11 ms controller periods.
          // Each request is at or below the 0.5000 m/s directional gate and
          // the command remains in [0.0005, 0.0050] m, strictly inside both the
          // 0..0.044 m hard range and the unchanged 0.00504 m tracking gate
          // even though the isolated fake motor reports a fixed position.
          const double directional_step =
              (cycle - kGripperJitterPreloadCycles) % 2 == 0
                  ? -kGripperJitterReversalStep
                  : kGripperJitterReversalStep;
          gripper_position_command->set_value(
              gripper_position_command->get_value() + directional_step);
        }
      } else if (gripper_position_command != nullptr &&
                 gripper_gate_mode == "time_credit_exhaustion_reject") {
        // The gate starts with 20 ms of credit.  Three alternating 5 mm
        // requests at 1 ms periods consume 10 ms each: the first leaves 10 ms,
        // the second leaves 1 ms, and the third sees only 2 ms and must fail.
        // Every candidate remains at 0 or 5 mm, so neither tracking nor hard
        // position limits can mask the time-credit-only rejection.
        if (cycle <= kGripperTimeCreditRejectCycle) {
          gripper_position_command->set_value(
              cycle % 2 == 0 ? kGripperTrackingSafeStep : 0.0);
        }
      } else if (gripper_position_command != nullptr &&
                 gripper_gate_mode == "tracking_first_over_reject") {
        // First prove that an exact 5 mm step is accepted, then request 5.041
        // mm from the fixed zero feedback.  The second delta is only 41 um, so
        // its rejection can only come from the strict 5.04 mm tracking gate.
        if (cycle == 0) {
          gripper_position_command->set_value(kGripperTrackingSafeStep);
        } else if (cycle == kGripperTrackingFirstOverCycle) {
          gripper_position_command->set_value(
              kGripperTrackingFirstOverPosition);
        }
      }
      if (hardware.write(clock.now(), period) !=
          hardware_interface::return_type::OK) {
        const bool expected_time_credit_error =
            expect_runtime_error &&
            gripper_gate_mode == "time_credit_exhaustion_reject" &&
            cycle == kGripperTimeCreditRejectCycle &&
            runtime_writes == kGripperTimeCreditRejectCycle;
        const bool expected_tracking_error =
            expect_runtime_error &&
            gripper_gate_mode == "tracking_first_over_reject" &&
            cycle == kGripperTrackingFirstOverCycle &&
            runtime_writes == kGripperTrackingFirstOverCycle;
        if (expected_time_credit_error || expected_tracking_error) {
          if (!require_production_shutdown("runtime_write", cycle)) {
            (void)hardware.on_deactivate(previous_state);
            shutdown_ros();
            return 36;
          }
          std::cout << "OPENARM_VCAN_RUNTIME=EXPECTED_ERROR cycle=" << cycle
                    << " runtime_writes=" << runtime_writes << " source="
                    << (expected_time_credit_error
                            ? "gripper_time_credit_exhaustion"
                            : "gripper_tracking_first_over")
                    << '\n';
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
          if (hardware.on_deactivate(previous_state) !=
              CallbackReturn::SUCCESS) {
            std::cerr << "OPENARM_VCAN_DEACTIVATION=ERROR "
                         "after_gripper_gate_error\n";
            shutdown_ros();
            return 33;
          }
          std::cout << "OPENARM_VCAN_DEACTIVATION=SUCCESS\n";
          shutdown_ros();
          return EXIT_SUCCESS;
        }
        std::cerr << "OPENARM_VCAN_RUNTIME=ERROR write cycle=" << cycle << '\n';
        shutdown_ros();
        return 25;
      }
      ++runtime_writes;
      std::this_thread::sleep_for(std::chrono::duration<double>(period_seconds));
    }
    if (expect_runtime_error || expect_bootstrap_error) {
      std::cerr << "OPENARM_VCAN_RUNTIME=ERROR watchdog_did_not_trip\n";
      (void)hardware.on_deactivate(previous_state);
      shutdown_ros();
      return 27;
    }
    if (gripper_gate_mode == "jitter_reversal") {
      std::cout << "OPENARM_VCAN_GRIPPER_GATE=SUCCESS "
                   "mode=jitter_reversal cycles=30 preload=1 reversals=12 "
                   "hold=17 periods_ms=20,9,11,10 "
                   "gate_m_s=+0.5000,-0.5000 preload_step_m=0.005000000 "
                   "reversal_step_m=0.004500000\n";
    }
    const auto period = rclcpp::Duration::from_seconds(1.0 / 100.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (hardware.read(clock.now(), period) !=
        hardware_interface::return_type::OK) {
      std::cerr << "OPENARM_VCAN_RUNTIME=ERROR final drain\n";
      shutdown_ros();
      return 26;
    }
    std::cout << "OPENARM_VCAN_RUNTIME=SUCCESS cycles=30\n";

    if (hardware.on_deactivate(previous_state) != CallbackReturn::SUCCESS) {
      std::cerr << "OPENARM_VCAN_DEACTIVATION=ERROR\n";
      shutdown_ros();
      return 23;
    }
    std::cout << "OPENARM_VCAN_DEACTIVATION=SUCCESS\n";
    shutdown_ros();
    return EXIT_SUCCESS;
  } catch (const std::exception& exception) {
    std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR exception=" << exception.what()
              << '\n';
    shutdown_ros();
    return 70;
  } catch (...) {
    std::cerr << "OPENARM_VCAN_ACTIVATION=ERROR unknown exception\n";
    shutdown_ros();
    return 71;
  }
}
