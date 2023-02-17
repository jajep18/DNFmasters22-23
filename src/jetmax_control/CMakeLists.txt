cmake_minimum_required(VERSION 3.5)
project(jetmax_control)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
## Depencies referenced in https://control.ros.org/foxy/doc/ros2_controllers/doc/writing_new_controller.html
find_package(controller_interface)
find_package(hardware_interface)
find_package(pluginlib)
find_package(rclcpp_lifecycle)
find_package(controller_manager)
find_package(gazebo_ros)
find_package(gazebo_ros2_control)

# find_package(gazebo_ros2_control/gazebo_hardware_plugins REQUIRED)

##pluginlib_export_plugin_description_file()

## Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FK.srv"
  "srv/IK.srv"
)

## Install Python scripts
install(
  PROGRAMS
    scripts/jetmax_fk.py
    scripts/jetmax_ik.py
    scripts/fk_client.py
    scripts/ik_client.py
    scripts/jetmax_kinematics.py
  DESTINATION lib/${PROJECT_NAME})

## Install Assets (launch files, models, ...)
install(
  DIRECTORY
    config
    launch
    scripts
#    srv
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()