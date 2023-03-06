#!/bin/bash
# Builds jetmax packages only
echo 'colcon build --symlink-install --packages-select jetmax_description jetmax_control main_package custom_msgs audio_package'
colcon build --symlink-install --packages-select jetmax_description jetmax_control main_package custom_msgs audio_package

echo './install/local_setup.bash aka sbuild'
source ./install/local_setup.bash