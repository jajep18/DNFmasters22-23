# DNFmasters22-23
DNF master project between Erik Lindby and Jacob Fløe Jeppesen

#Quickguide: Terminal 1:
sfoxy
sgz
colcon build
sbuild
ros2 launch test_package main_test.launch.py
# This should open up gazebo. In terminal 2:
sfoxy
rviz2

# Notes. on rename
delete build, install and log on renaming, and re-run 'colcon build'
pkg select (sic?) to build specific packages only

Topics, services actions

ros2 topic list
ros2 node list
ros2 topic echo /rosout            - make subscriber to topic in terminal (Read data)
ros2 topic hz /rosout              - info on speed of data


colcon build
efter hver build, source workspace (alle noder/libraries/function til terminalen), '. install/setup.bash', eller alias sbuild

#ros2 launch <package_name> <launch_file> verbose:=false world:=new_world.world 
ros2 launch test_package main_test.launch.py
