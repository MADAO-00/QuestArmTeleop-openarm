cmake_minimum_required(VERSION 3.8)
project(oculus_reader)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Keep the established ROS package name and OpenArm executable names.
install(PROGRAMS
  scripts/install.py
  scripts/pub_pose_openarm_v1.py
  scripts/pub_delta_pose_openarm_v1.py
  scripts/openarm_bimanual_ik_node.py
  scripts/openarm_command_guard_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Quest input and safety helpers are imported by the executables above.
install(FILES
  scripts/buttons_parser.py
  scripts/FPS_counter.py
  scripts/oculus_reader.py
  scripts/transformations.py
  scripts/openarm_teleop_core.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config APK
  DESTINATION share/${PROJECT_NAME}
)

# Ship the pinned official IK model with the ROS package. No sibling project
# or fixed workspace location is needed at runtime.
install(DIRECTORY ../../assets/openarm_mujoco
  DESTINATION share/${PROJECT_NAME}/assets
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(
    test_openarm_teleop_core
    test/test_openarm_teleop_core.py
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_openarm_launch_contract
    test/test_openarm_launch_contract.py
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
