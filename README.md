# Automated Shelf Repositioning in Warehouse Enviroments


### The project is developed with the purpose of repositioning the shelf in the correct location. This is achieved through the use of multiple ROS2 nodes running in parallel, as well as utilizing the Nav2 framework for the Humble version.

### There are 3 servers

### - Patrol Behavior Server: Follow a set of waypoints to find the shelf
### - Approach Server: Get underneath the shelf that has already been located
### - Shelf Position Server: Reallocate the shelf to the correct position

## Generate setup (ROS2 Humble)
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-rosbridge-server
```

## Create your workspace

```
mkdir -p /ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Combuster54/warehouse_setup
```

##  Start RVIZ2 && nav2 config

```
cd ~/ros2_ws/src/warehouse_setup
./nav2_setup.sh
```

## Start webpage && rosbridge
```
#Ports
#http.server -> 7000 && rosbridge -> 9090
cd ~/ros2_ws/src/warehouse_setup
./webpage_setup.sh
```

## Start webpage && rosbridge

```
cd ~/ros2_ws/src/warehouse_setup
./server_setup.sh
```

## Status

| Service | Humble | Iron  | Jazzy | Main  |
| :---:   | :---:  | :---: | :---: | :---: |
| Tested  | ![Success](https://img.shields.io/badge/Success-green) <!-- [![Build Status](https://build.ros2.org/job/Hdev__navigation2__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__navigation2__ubuntu_jammy_amd64/) --> | Not tested <!-- [![Build Status](https://build.ros2.org/job/Idev__navigation2__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__navigation2__ubuntu_jammy_amd64/) --> | Not tested <!-- [![Build Status](https://build.ros2.org/job/Jdev__navigation2__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__navigation2__ubuntu_noble_amd64/) --> | N/A |
