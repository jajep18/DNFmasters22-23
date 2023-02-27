cmake_minimum_required(VERSION 3.5)
project(main_package)

#####################################################################################################
# ----------------------------------------- Configuration ----------------------------------------- #
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#####################################################################################################
# --------------------------------------- Append Prefix Path -------------------------------------- #

# Add torch library to CMAKE prefix path //Needed in clean builds //Change to local libtorch installation
#list(APPEND CMAKE_PREFIX_PATH "") 

## Add gazebo to CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/gazebo-11")

#####################################################################################################
# --------------------------------------- Find Dependencies --------------------------------------- #
find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
#find_package(dnf_package)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)

## Environment hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

#####################################################################################################
# ------------------------------------------- Libraries ------------------------------------------- #
# https://answers.ros.org/question/357633/how-to-create-a-ros2-library/?answer=385279#post-id-385279
# https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library

#####################################################################################################
# ---------------------------------------- Add Executables ---------------------------------------- #

## Camera publisher/subscriber node
add_executable(cam_pubsub nodes/camera_pubsub_function.cpp)
ament_target_dependencies(cam_pubsub custom_msgs rclcpp std_msgs OpenCV image_transport cv_bridge)

## Circle information subscriber node
add_executable(circle_listener nodes/circle_subscriber_function.cpp)
ament_target_dependencies(circle_listener custom_msgs rclcpp std_msgs OpenCV sensor_msgs)

#####################################################################################################
# ------------------------------ Install Target & Ament dependencies ------------------------------ #

## Install Header files

## Install Libraries

## Install Python scripts
install(
  PROGRAMS
    nodes/transform_pubsub.py
    nodes/movement_command_center.py
  DESTINATION lib/${PROJECT_NAME}
)

## Install Nodes
install(TARGETS
  cam_pubsub
  circle_listener
  DESTINATION lib/${PROJECT_NAME}
)

## Install Assets (launch files, models, ...)
install(
  DIRECTORY
    launch
    models
    worlds
    nodes
  DESTINATION share/${PROJECT_NAME}/
)

#####################################################################################################
# -------------------------------------------- Testing -------------------------------------------- #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
