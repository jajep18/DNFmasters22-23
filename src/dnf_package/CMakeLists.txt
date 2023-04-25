cmake_minimum_required(VERSION 3.5)
project(dnf_package)

#####################################################################################################
# ----------------------------------------- Configuration ----------------------------------------- #
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#####################################################################################################
# --------------------------------------- Append Prefix Path -------------------------------------- #

# Add torch library to CMAKE prefix path //Needed in clean builds //Change to local libtorch installation
list(APPEND CMAKE_PREFIX_PATH "/home/student/Documents/libtorch/share/cmake/Torch") #Erik
list(APPEND CMAKE_PREFIX_PATH "/home/jacob/libtorch/share/cmake/Torch") #Floe

#####################################################################################################
# --------------------------------------- Find Dependencies --------------------------------------- #
find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED) #For services
find_package(rosidl_default_generators REQUIRED) #For services
find_package(rosidl_generator_cpp REQUIRED) #For services
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Torch REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Torch setup flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

#####################################################################################################
# ------------------------------------------- Libraries ------------------------------------------- #
# https://answers.ros.org/question/357633/how-to-create-a-ros2-library/?answer=385279#post-id-385279
# https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library

## DNF_1D (class) library
add_library(dnf_1d src/dnf_1d.cpp)
target_include_directories(dnf_1d PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(dnf_1d ${TORCH_LIBRARIES} rclcpp::rclcpp)
#ament_export_targets(dnf_1d HAS_LIBRARY_TARGET)

## DNF_2D (class) library
add_library(dnf_2d src/dnf_2d.cpp)
target_include_directories(dnf_2d PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(dnf_2d ${TORCH_LIBRARIES} rclcpp::rclcpp)
# ament_export_targets(dnf_2d HAS_LIBRARY_TARGET)

## Learning Trial (class) library
add_library(learning_trial src/learning_trial.cpp)
target_include_directories(learning_trial PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(learning_trial ${TORCH_LIBRARIES} rclcpp::rclcpp)

ament_export_dependencies(rclcpp)

#####################################################################################################
# ------------------------------------------ Interfaces ------------------------------------------- #

## Service files #Note: These were exported to the custom_msgs package
# rosidl_generate_interfaces(${PROJECT_NAME}
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/Decision.srv"
#   DEPENDENCIES std_msgs geometry_msgs
# )

#####################################################################################################
# ---------------------------------------- Add Executables ---------------------------------------- #

## DNF publisher/subscriber node
add_executable(dnf_pubsub nodes/dnf_pubsub_function.cpp)
ament_target_dependencies(dnf_pubsub rclcpp std_msgs custom_msgs OpenCV Torch ament_index_cpp)
target_link_libraries(dnf_pubsub ${TORCH_LIBRARIES} dnf_1d dnf_2d learning_trial)

# Torch Debug - Setup link libraries - Note: Remember to link torch library to all executables (nodes) that need torch
if(NOT Torch_FOUND)
  message(FATAL_ERROR "Torch not found!")
endif(NOT Torch_FOUND)
message(STATUS "Pytorch status:")
message(STATUS "    libraries: ${TORCH_LIBRARIES}")
#target_link_libraries(dnf_pubsub ${TORCH_LIBRARIES})

#####################################################################################################
# --------------------------------------- Installs / Exports -------------------------------------- #

## Install Header files
install(DIRECTORY 
  include/
  DESTINATION include)
include_directories(include)
ament_export_include_directories(include)

# ## Install Libraries
# install(
#   TARGETS dnf_1d
#   EXPORT dnf_1d
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

## Install Nodes
install(TARGETS
  dnf_pubsub
  DESTINATION lib/${PROJECT_NAME})

## Install Assets (launch files, models, ...)
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}/
)

#####################################################################################################
# -------------------------------------------- Testing -------------------------------------------- #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
