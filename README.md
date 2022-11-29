# DNFmasters22-23
DNF master project between Erik Lindby and Jacob Fløe Jeppesen

#Quickguide: Terminal 1:
sfoxy
sgz
colcon build
sbuild
ros2 launch main_package main.launch.py
# This should open up gazebo. In terminal 2:
sfoxy
rviz2

# Notes on libtorch
When clean building the cmakelist needs to be given the path to the pre-built libtorch library
At runtime the OS also needs to know where the dynamic library is, so we need to give the path to LD_LIBRARY_PATH

# Notes on compiling ROS2 in general
Can run single nodes: ros2 run main_package dnf_pubsub
Can compile specific packages instead of all packages: colcon build --packages-select <pkg_name> --symlink-install
colcon build --packages-select main_package

# Notes. on rename
delete build, install and log on renaming, and re-run 'colcon build'
pkg select (sic?) to build specific packages only

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
