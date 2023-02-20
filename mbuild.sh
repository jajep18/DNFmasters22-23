#!/bin/bash
# Builds jetmax packages only
echo 'colcon build --symlink-install --packages-select jetmax_description jetmax_control'
colcon build --symlink-install --packages-select jetmax_description jetmax_control

echo './install/local_setup.bash aka sbuild'
source ./install/local_setup.bash