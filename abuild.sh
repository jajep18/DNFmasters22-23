#!/bin/bash
# Builds jetmax packages only
echo 'colcon build --symlink-install --packages-select custom_msgs audio_package dnf_package'
colcon build --symlink-install --packages-select custom_msgs audio_package dnf_package

echo './install/local_setup.bash aka sbuild'
source ./install/local_setup.bash