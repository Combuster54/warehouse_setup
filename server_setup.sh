#!/bin/bash
source /opt/ros/humble/setup.bash
ros2 launch nav2_apps start_service_servers.launch.py

exec "$@"