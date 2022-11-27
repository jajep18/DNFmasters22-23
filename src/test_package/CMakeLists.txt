
cmake_minimum_required(VERSION 3.5)
project(test_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add torch library to CMAKE prefix path //Needed in clean builds
list(APPEND CMAKE_PREFIX_PATH "/home/student/Documents/libtorch/share/cmake/Torch")
#set(CMAKE_PREFIX_PATH "~/Documents/libtorch/share/cmake/Torch")

# find dependencies
find_package(gazebo_dev REQUIRED)
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
find_package(Torch REQUIRED)

# Torch setup
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

## Assets (launch files, models, ...)
install(
  DIRECTORY
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}/
)

## Environment hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

# Include headers (.hpp in includes) as library

# Add executables
# add_executable(cam_funcs src/camera_functions.cpp)
# ament_target_dependencies(cam_funcs OpenCV)
add_executable(my_talker nodes/publisher_member_function.cpp)
ament_target_dependencies(my_talker rclcpp std_msgs)
add_executable(my_listener nodes/subscriber_member_function.cpp)
ament_target_dependencies(my_listener rclcpp std_msgs)
add_executable(cam_pubsub nodes/camera_pubsub_function.cpp)
ament_target_dependencies(cam_pubsub custom_msgs rclcpp std_msgs OpenCV image_transport cv_bridge)
add_executable(circle_listener nodes/circle_subscriber_function.cpp)
ament_target_dependencies(circle_listener custom_msgs rclcpp std_msgs)
add_executable(dnf_pubsub nodes/dnf_pubsub_function.cpp)
#ament_target_dependencies(dnf_pubsub rclcpp std_msgs custom_msgs OpenCV)
ament_target_dependencies(dnf_pubsub rclcpp std_msgs custom_msgs OpenCV Torch)

# Setup link libraries - Note: Remember to link torch library to all executables (nodes) that need torch
if(NOT Torch_FOUND)
  message(FATAL_ERROR "Torch not found!")
endif(NOT Torch_FOUND)
message(STATUS "Pytorch status:")
message(STATUS "    libraries: ${TORCH_LIBRARIES}")
target_link_libraries(dnf_pubsub ${TORCH_LIBRARIES})


install(TARGETS
  # cam_funcs
  my_talker
  my_listener
  cam_pubsub
  circle_listener
  dnf_pubsub
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
