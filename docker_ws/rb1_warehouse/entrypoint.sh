#!/bin/bash
source /opt/ros/humble/setup.bash
source /webpage_ws/install/setup.bash
cd /webpage_ws/src/webpage
# Ejecutar servidor web
python3 -m http.server 7000 && ros2 launch rosbridge_server rossbridge_websocket_launch.xml

# Mantener el contenedor activo
exec "$@"
