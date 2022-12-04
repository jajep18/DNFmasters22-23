#!/bin/bash

echo 'colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Debug'
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Debug

echo './install/local_setup.bash'
source ./install/local_setup.bash