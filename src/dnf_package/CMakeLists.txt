cmake_minimum_required(VERSION 3.5)
project(dnf_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#####################################################################################################
# ------------------------------------- Append Prefix Path ---------------------------------------- #

# Add torch library to CMAKE prefix path //Needed in clean builds //Change to local libtorch installation
#list(APPEND CMAKE_PREFIX_PATH "/home/student/Documents/libtorch/share/cmake/Torch") #Erik
list(APPEND CMAKE_PREFIX_PATH "/home/jacob/libtorch/share/cmake/Torch") #Floe

#####################################################################################################
# ------------------------------------- Find Dependencies ----------------------------------------- #
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
  DESTINATION share/${PROJECT_NAME}/
)

## Include headers (.hpp in includes) as library
#include_directories(include)

#####################################################################################################
# ---------------------------------------- Add Executables ---------------------------------------- #

## DNF publisher/subscriber
add_executable(dnf_pubsub nodes/dnf_pubsub_function.cpp)
ament_target_dependencies(dnf_pubsub rclcpp std_msgs custom_msgs OpenCV Torch)

# Setup link libraries - Note: Remember to link torch library to all executables (nodes) that need torch
if(NOT Torch_FOUND)
  message(FATAL_ERROR "Torch not found!")
endif(NOT Torch_FOUND)
message(STATUS "Pytorch status:")
message(STATUS "    libraries: ${TORCH_LIBRARIES}")
target_link_libraries(dnf_pubsub ${TORCH_LIBRARIES})

#####################################################################################################
# ------------------------------- Install Target, Directory, Ament -------------------------------- #
install(TARGETS
  dnf_pubsub
  DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY include/
#  DESTINATION include)
#ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
