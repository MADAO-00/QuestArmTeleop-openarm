cmake_minimum_required(VERSION 3.16)
project(openarm_vcan_activation_probe LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
# The upstream CMake project installs OpenArmCANConfig.cmake, while the ROS
# package export names the dependency "openarm_can".  Load the real package
# first and mark that already-resolved dependency for the ament export shim.
find_package(OpenArmCAN REQUIRED CONFIG)
set(openarm_can_FOUND TRUE)
find_package(openarm_hardware REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(openarm_vcan_activation_probe main.cpp)
target_compile_features(openarm_vcan_activation_probe PRIVATE cxx_std_17)
target_compile_options(openarm_vcan_activation_probe PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  openarm_vcan_activation_probe
  hardware_interface
  openarm_hardware
  rclcpp
  rclcpp_lifecycle
)
