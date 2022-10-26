
cmake_minimum_required(VERSION 3.5)
project(test_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
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

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

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

ament_package()

# Include headers (.hpp in includes) as library

# Add executables
# add_executable(cam_funcs src/camera_functions.cpp)
# ament_target_dependencies(cam_funcs OpenCV)
add_executable(my_talker nodes/publisher_member_function.cpp)
ament_target_dependencies(my_talker rclcpp std_msgs)
add_executable(my_listener nodes/subscriber_member_function.cpp)
ament_target_dependencies(my_listener rclcpp std_msgs)
add_executable(cam_subscriber nodes/camera_subscriber_function.cpp)
ament_target_dependencies(cam_subscriber rclcpp std_msgs OpenCV image_transport cv_bridge)
#target_include_directories(test_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

install(TARGETS
  # cam_funcs
  my_talker
  my_listener
  cam_subscriber
  DESTINATION lib/${PROJECT_NAME})


