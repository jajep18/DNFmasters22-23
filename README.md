# DNFmasters22-23
DNF master project between Erik Lindby and Jacob Fløe Jeppesen

# Launch Files
ros2 launch main_package main.launch.py
This launch files sets up a world with a table, camera and 3 balls.

# Quickguide to ros2: Terminal 1:
sfoxy
sgz
colcon build
sbuild
ros2 launch main_package main.launch.py
# This should open up gazebo. In terminal 2:
sfoxy
rviz2

# Requirements:
Install controller manager packages for FOXY: https://control.ros.org/foxy/doc/getting_started/getting_started.html
Torch
Ros2
Gazebo + ROS2 packages
Ros Foxy gazebo_ros2_control; Install with: sudo apt-get install ros-foxy-gazebo-ros2-control

# Notes on libtorch
When clean building the cmakelist needs to be given your path to the pre-built libtorch library. Change it in the CMakeLists.txt in dnf_package.
At runtime the OS also needs to know where the dynamic library is, so you need to append the path to LD_LIBRARY_PATH (set it up in .bashrc)
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/Qt/5.15.2/gcc_64/lib:/Yourpath/libtorch/lib


# Notes on compiling ROS2 in general
Can run single nodes: ros2 run main_package dnf_pubsub
pkg select (sic?) to build specific packages only
Can compile specific packages instead of all packages: colcon build --packages-select <pkg_name> --symlink-install
colcon build --packages-select main_package

# Notes. on rename
delete build, install and log on renaming, and re-run 'colcon build'


# Topics, services actions
ros2 topic list
ros2 node list
ros2 topic echo /rosout            - make subscriber to topic in terminal (Read data)
ros2 topic hz /rosout              - info on speed of data


colcon build
sbuild
efter hver build, source workspace (alle noder/libraries/function til terminalen), '. install/setup.bash', eller alias sbuild

#ros2 launch <package_name> <launch_file> verbose:=false world:=new_world.world 
ros2 launch main_package main.launch.py
ros2 run main_package dnf_pubsub
