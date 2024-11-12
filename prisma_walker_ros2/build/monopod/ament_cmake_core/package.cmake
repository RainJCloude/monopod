set(_AMENT_PACKAGE_NAME "monopod")
set(monopod_VERSION "0.0.0")
set(monopod_MAINTAINER "claudio <claudio.chiariello@unina.it>")
set(monopod_BUILD_DEPENDS "hardware_interface" "pluginlib" "controller_manager" "gazebo_ros2_control" "rclcpp" "rclpy")
set(monopod_BUILDTOOL_DEPENDS "ament_cmake" "ament_cmake_python")
set(monopod_BUILD_EXPORT_DEPENDS "hardware_interface" "pluginlib" "controller_manager" "gazebo_ros2_control" "rclcpp" "rclpy")
set(monopod_BUILDTOOL_EXPORT_DEPENDS )
set(monopod_EXEC_DEPENDS "trajectory_msgs" "joint_state_publisher_gui" "launch" "launch_ros" "robot_state_publisher" "rviz2" "urdf" "gazebo_ros" "joint_trajectory_controller" "joint_state_broadcaster" "xacro" "hardware_interface" "pluginlib" "controller_manager" "gazebo_ros2_control" "rclcpp" "rclpy")
set(monopod_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(monopod_GROUP_DEPENDS )
set(monopod_MEMBER_OF_GROUPS )
set(monopod_DEPRECATED "")
set(monopod_EXPORT_TAGS)
list(APPEND monopod_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND monopod_EXPORT_TAGS "<gazebo_ros gazebo_model_path=\"..\"/>")