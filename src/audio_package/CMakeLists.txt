cmake_minimum_required(VERSION 3.5)
project(audio_package)

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

#####################################################################################################
# --------------------------------------- Find Dependencies --------------------------------------- #
find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pyaudio REQUIRED)
#find_package(custom_msgs REQUIRED) #reduce compile time, commint in later!!!!!!!


#####################################################################################################
# ------------------------------------------- Libraries ------------------------------------------- #
# https://answers.ros.org/question/357633/how-to-create-a-ros2-library/?answer=385279#post-id-385279
# https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library

#####################################################################################################
# ---------------------------------------- Add Executables ---------------------------------------- #

## Talker node
add_executable(my_talker nodes/publisher_member_function.cpp)
ament_target_dependencies(my_talker rclcpp std_msgs)

## Listener node
add_executable(my_listener nodes/subscriber_member_function.cpp)
ament_target_dependencies(my_listener rclcpp std_msgs)

## PocketSphinx node
# add_executable(pocketsphinx_node nodes/pocketsphinx_pubsub.py)
# ament_target_dependencies(pocketsphinx_node rclpy std_msgs)

#####################################################################################################
# ------------------------------ Install Target & Ament dependencies ------------------------------ #

## Install Header files

## Install Libraries

## Install programs (Python scripts, etc.)
# ament_python_install_package(${PROJECT_NAME})
# install(PROGRAMS  
#   nodes/pocketsphinx_pubsub.py
#   DESTINATION lib/${PROJECT_NAME} 
# )

## Install Nodes
install(TARGETS
  my_talker
  my_listener
  #pocketsphinx_node
  DESTINATION lib/${PROJECT_NAME}
)

## Install Assets (launch files, models, ...)
install(
  DIRECTORY
    launch
    audio_files
    #models
    #worlds
  DESTINATION share/${PROJECT_NAME}/
)

#####################################################################################################
# -------------------------------------------- Testing -------------------------------------------- #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
