# tomato2023

## Projects
- tomato_pubsub
- tomato_joycon
- tomato_kobuki
- tomato_dynamixel

## how to build the projects

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone 
$ cd tomato2023
$ catkin build
```

## Updates
- removed y axis movement
- added restriction to x axis movement
- Realsense and yolov8 for tomato positioning
- Monocular camera also supported
- Added "start" button functionality for toggle between autonomous mode and controlled mode
- Added "back" button functionality for resetting position of arm

## How to launch
### Base file
```bash
$ roslaunch dynamixelcontrol param.launch
```
### Camera-based tomato positioning 
```bash
$ roslaunch dynamixelcontrol follow.launch
```
### Realsense camera-based tomato positioning 
```bash
$ roslaunch dynamixelcontrol real.launch
```
