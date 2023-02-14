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
## Depencies referenced in https://control.ros.org/foxy/doc/ros2_controllers/doc/writing_new_controller.html
find_package(controller_interface)
find_package(hardware_interface)
find_package(pluginlib)
find_package(rclcpp_lifecycle)
find_package(controller_manager)
find_package(gazebo_ros)



## Service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FK.srv"
  "srv/IK.srv"
  "srv/FKRequest.srv"
)

## Install Python scripts
install(
  PROGRAMS 
    scripts/go_home.py
    scripts/jetmax_fk.py
    scripts/jetmax_ik.py
    scripts/jetmax_kinematics.py
    scripts/moving.py
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