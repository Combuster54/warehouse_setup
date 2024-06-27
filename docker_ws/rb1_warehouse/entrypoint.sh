#!/bin/bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws/src/warehouse_setup/docker_ws/rb1_warehouse/webpage

# Función para manejar la señal de interrupción
cleanup() {
    echo "Cleaning up..."
    kill $(jobs -p)
    wait
}

# Atrapar la señal de interrupción (Ctrl+C)
trap cleanup SIGINT

# Verificar si el puerto 7000 está en uso y liberar si es necesario
PORT=7000
if lsof -i:$PORT -t >/dev/null; then
    echo "Port $PORT is in use. Releasing it..."
    fuser -k $PORT/tcp
fi

# Ejecutar el servidor web en segundo plano
python3 -m http.server 7000 &

sleep 1

# Verificar si el puerto del rosbridge está en uso y liberar si es necesario
PORT_ROSBRIDGE=9090 # Cambia esto al puerto correcto si es diferente
if lsof -i:$PORT_ROSBRIDGE -t >/dev/null; then
    echo "Port $PORT_ROSBRIDGE is in use. Releasing it..."
    fuser -k $PORT_ROSBRIDGE/tcp
fi

# Ejecutar lanzamiento de ROS 2 en segundo plano
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Esperar a que todos los procesos en segundo plano terminen
wait

# Mantener el contenedor activo
exec "$@"
