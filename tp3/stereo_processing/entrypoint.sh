#!/bin/bash

source /opt/ros/humble/setup.bash

./compvision "$1" &

sleep 2

rviz2 -d /usr/src/app/default.rviz
