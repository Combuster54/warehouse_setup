#!/bin/bash
source /opt/ros/humble/setup.bash

ros2 launch path_planner_server real_navigation.launch.py

exec "$@"