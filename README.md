# tomato2025

## Projects
- dynamixelcontrol
- tomato_dynamixel
- tomato_pubsub
- tomato_joycon
- tomato_kobuki
- tomato_dynamixel
- yolo_detection

## how to build the projects

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone 
$ cd tomato2023
$ catkin build
```

## Updates
### 5.1.1 
- MASSIVE... Autonomous kobuki movement
- Autonomous MX movement
- Cleaned up phases using enum (custom functions for readability)
- added phases (init, phase1, phase2, goback, kobuki)

### 5.0.1
- Better restrictions
- Reduced speed (positional -> incremental)
- Added control phases
- Phase #1 Fast movement
- Phase #2 Contact with tomato
- Phase #3 Pushing motion
- Phase #4 Catch and pull

### 4.1.1
- rviz visualization
- adjusted for extended position control (mx)
- added mx position based on mx angle(cm)
- fixed tomato x,y,z geometry

### 4.0.1
- Realsense and yolov8 for tomato positioning (param2.cpp, real.launch)
- Monocular camera also supported (follow.launch)
- Added "start" button functionality for toggle between autonomous mode and controlled mode
- Added "back" button functionality for resetting position of arm


## Basic Arm geometry
### x, y coordinates
![equation](https://latex.codecogs.com/svg.image?&space;x=l(sin(\theta_1)&plus;sin(\theta_2)))


![equation](https://latex.codecogs.com/svg.image?&space;y=l(cos(\theta_1)-cos(\theta_2)))

### inverse kinematics solution 
![equation](https://latex.codecogs.com/svg.image?\theta_1=sin^{-1}(\frac{\sqrt{x^2&plus;y^2}}{2l})&plus;tan^{-1}(\frac{y}{x}))


![equation](https://latex.codecogs.com/svg.image?\theta_1=sin^{-1}(\frac{\sqrt{x^2&plus;y^2}}{2l})-tan^{-1}(\frac{y}{x}))

## Usage (code)
```cpp
bool paused; // for pausing
bool backed; // for reset position
bool autonomous; // auto mode

enum Phases{
  ... //add phases here
}
//and later in main
int main(...){
  ...
  switch(current_phase){
    //add phases here
  }
}

void go_to(double _x, double _y, _double _z, ...){
...
}

// arm goes to (_x, _y, _z)

void make_move(...){
...
}

// make updates to robot movement (target_val is radians)
```

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
### Kobuki, Camera, arm launched all at once + param3.cpp*
```bash
$ roslaunch dynamixelcontrol realauto.launch
```
