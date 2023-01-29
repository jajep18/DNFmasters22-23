#!/bin/bash

echo 'colcon build --symlink-install --packages-select custom_msgs main_package'
colcon build --symlink-install --packages-select custom_msgs main_package

echo './install/local_setup.bash aka sbuild'
source ./install/local_setup.bash