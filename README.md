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

## Base file

$ roslaunch dynamixelcontrol param.launch

## Camera-based tomato positioning 

$ roslaunch dynamixelcontrol follow.launch

## Realsense camera-based tomato positioning 

$ roslaunch dynamixelcontrol real.launch
