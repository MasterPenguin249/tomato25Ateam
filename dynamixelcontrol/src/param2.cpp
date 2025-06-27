#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include "DynamixelControl/DynamixelControl.h"
#include <yolo_detection/BoundingBoxArray.h>
#include <yolo_detection/BoundingBox.h>
#include <geometry_msgs/PointStamped.h>

#include <cmath>
#include <memory>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdexcept>
#include <cstdlib>

#define NODE_FREQUENCY        200

#define f1(x, y) -(cos(x) / cos(y))
#define f2(x, y) (sin(x) / sin(y))
#define rad(x) (x * M_PI/180)

// tomato states
double tomato_x = 9;
double tomato_y = 9;
double tomato_z = 0;
double tomato_size = 2.8; //cm
double t = 0;
bool use_realsense = true;  // Toggle between RealSense and monocular
geometry_msgs::PointStamped latest_tomato_point;
bool has_realsense_data = false;
// bool paused_collection = false;


//moving states
double speed_1 = 0.1;
double speed_2 = 0.02;
double speed_3 = 0.08;
bool phase1 = false;
bool phase2 = false;
bool phase3 = false;

// arm states
float vel_ax_x = 0.0;
float vel_ax_y = 0.0;
bool arrived = false;
bool paused = false;
bool backed = false;
bool fixed_y = true;
bool goback = false;
bool wait = false;
double closed = rad(-17.81);
double half_closed = rad(0.5);
double opened = rad(18.75);
double l = 9; // cm
bool phase_goto = false;

//joy states
bool opening = true;
bool autonomous = false;
int prev_start_button = 0;
int prev_back_button = 0;
float lt = 0;

// camera states
double cam_angle = rad(-2);
double cam_angle_real = rad(-12.49);
double armdim_x = 9;
double armdim_y = 9;

// velocity variables
int16_t vel_mx_write= 0; // -285 ~ 285
float scale_ax = 0.05;
float vel_ax_1 = 0.0;
float vel_ax_2 = 0.0;
double speed_x = 0.02;
double speed_y = 0.02;
double go_back_speed = 0.08;
float scale_mx = 30.0; // 3/s? that's 30/ds

// mx position (cm)
float mx_pos = 20; // cm
double worm_pitch = 1.5 / (2* M_PI);
double mx_limit_upper = 29; //cm
double mx_limit_lower = 17;
bool phase_mxdown = false;
bool phase_mxup = false;


// example range
// y(250, 220) minus 150-> (100, 70)?? (because the range is 0,300 not 0,360)
// x(90, 200) -> (-60, 50)??
// (130, 150 (rest)) -> (-20, 0) try 180

// arrived (x, y)
bool is_arrived(std::vector<double> &cur_val, std::vector<double> &target_val){
  for(int i = 0; i <5; i++){
    if(i == 2 || i== 3) continue;
    if(abs(cur_val[i] - target_val[i]) < 0.1) continue;
    else return false;
  }
  return true;
}


// reset arm position
void go_back(std::vector<double> &cur_val, std::vector<double> &target_val){
  wait = true;
  target_val[0] = rad(-75);
  target_val[1] = rad(105);
  target_val[4] = rad(75);
  std::vector<double> init_state = {rad(-75), rad(105), opened, 0, rad(75)};
  backed = is_arrived(cur_val, init_state);
  if(backed){
    ROS_ERROR("YESSSSSSSSSS");
    arrived = false;
    paused = false;
    opening = true;
    goback = false;
    wait = false;
    // paused_collection = false;
    t = 0;
  } 
}

void joyCallback(const sensor_msgs::Joy& msg)
{
  // cross 
  if(msg.buttons[1]==1)
    vel_ax_1 = - msg.buttons[1]*scale_ax;
  else if(msg.buttons[2]==1)
    vel_ax_1 = msg.buttons[2]*scale_ax;
  else
    vel_ax_1 = 0;

  // arm x, y
  vel_ax_x = msg.axes[4];
  vel_ax_y = msg.axes[7];
  opening = msg.buttons[5] == 0;

  // mx
  if(msg.buttons[3]==1)
    vel_mx_write= scale_mx;    
  else if(msg.buttons[0]==1)
    vel_mx_write= -scale_mx;
  else
    vel_mx_write=0;

  // fixed y or not
  fixed_y = (msg.axes[2]< -0.8)?  false: true;

  int current_start_button = msg.buttons[7];
  // Detect button pressed (start)
  if (current_start_button == 1 && prev_start_button == 0){
     autonomous = !autonomous;
     t = 0;

  }

  int current_back_button = msg.buttons[6];
  // Detect button pressed (back)
  if (current_back_button == 1 && prev_back_button == 0) goback = true;
  
  // Update previous button state
  prev_start_button = current_start_button;
  prev_back_button = current_back_button;

}

void realsensePointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if(paused) return;
    
    latest_tomato_point = *msg;
    has_realsense_data = true;
    
    // tomato_x = msg->point.x * 100.0;
    // tomato_y = msg->point.y * 100.0;
    // tomato_z = msg->point.z * 100.0;

    //Convert from meters to centimeters 
    double distance = msg->point.x * 100.0 + tomato_size/2; 
    double center_y = msg->point.y * -100.0;
    double center_z = msg->point.z * -100.0;
    
    //tomato_x within (7, 21)
    tomato_x = distance * cos(cam_angle_real) - center_y * sin(cam_angle_real) - 3; 
    // tomato_y has to be within (0, 14)
    tomato_y = center_y * cos(cam_angle_real) + distance * sin(cam_angle_real) + 14.8; // calibration needed
    //tomato_z within (-5.5, 5.5)
    tomato_z = center_z + 4.5;
}

void bboxCallback(const yolo_detection::BoundingBoxArray::ConstPtr& msg)
{
  if(paused) return;
  double min_distance = 10000;

  if(use_realsense && has_realsense_data) {
    return;  // RealSense data is handled in realsensePointCallback
  }

  // Process each bounding box
  for (size_t i = 0; i < msg->bounding_boxes.size(); ++i)
  {
    const yolo_detection::BoundingBox& bbox = msg->bounding_boxes[i];
    double distance = tomato_size /((bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min))/2.0 *3111;
    if(bbox.class_name != "tomato" || distance > min_distance){
      continue;
    }else if(abs(bbox.x_max - bbox.x_min - (bbox.y_max-bbox.y_min)) > 0.1*std::max(bbox.x_max - bbox.x_min, bbox.y_max-bbox.y_min)){
      continue;
    }
    double center_x = (bbox.x_max + bbox.x_min)/2.0;
    double center_y = (bbox.y_max + bbox.y_min)/2.0;
    //tomato_x within (7, 21)
    tomato_x = distance * cos(cam_angle) - (240 - center_y )/480 * 11.25 / 17 * distance * sin(cam_angle)- 3; 
    // tomato_y has to be within (0, 14)
    tomato_y = (240 - center_y )/480 * 11.25 / 17 * distance * cos(cam_angle)\
    + distance * sin(cam_angle) + (41.5 - mx_pos); // calibration needed
    //tomato_z within (-5.5, 5.5)
    tomato_z = (320 - center_x )/640 * 15 / 17 * distance +1 ;
  }
}

// square
std::vector<std::pair<double, double>> square{
  {5, 5}, {5, 9}, {9, 9}, {9, 5} 
};
std::vector<std::pair<double, double>> farstraight{
  {9, -10}, {9, 10}// {9, -10 fails because of the arm geomtry}
};
int point = 0;

void limitcheck(std::vector<double> &target_val){
  // upper and lower rotational limits
  std::vector<std::pair<double, double>> limits = {
    {rad(-70), rad(80)},
    {rad(-target_val[0]+10), rad(-target_val[0]+140)},
    {rad(-150), rad(150) },
    {(mx_limit_lower- 19.63)/worm_pitch, (mx_limit_upper- 19.63)/worm_pitch},
    {rad(20), rad(120)},
    {rad(0), rad(-45)}
  };
  // gatekeep
  for(int i = 0; i < target_val.size(); i++){
    if(target_val[i] < limits[i].first){
       target_val[i] = limits[i].first;
    }else if (target_val[i] > limits[i].second){
      target_val[i] = limits[i].second;
    }
  }
}

bool is_ok(double theta1, double theta2){
  double _x = l*(sin(theta1) + sin(theta2));
  double _y = l*(cos(theta1) - cos(theta2));
  if(sqrt(_x*_x+_y*_y) > 2*l || _x > 18 || _y < -12 || pow(_x + 2, 2)/10 + pow(_y,2)/6 < 1) {
    ROS_ERROR("invalid square values");
    return false;
  }
  return true;
}

// insert coordinates, arm goes there
void go_to(double _x, double _y, double _z, std::vector<double> &target_val, double speed){
  std::vector<double> goal(3);
  //convert to angle (I got them flipped the entire time)
  goal[1] = asin(sqrt((_x*_x+_y*_y))/2/l) + atan(_y/_x);
  goal[0] = asin(sqrt((_x*_x+_y*_y))/2/l) - atan(_y/_x);
  goal[2] = _z/2.5;
  if(abs(target_val[0] - goal[0]) < speed) {
    target_val[0] = goal[0];
  }else{
    if(target_val[0] < goal[0]) target_val[0] += speed;
    else if(target_val[0] > goal[0]) target_val[0] -= speed;
  }

  if(abs(target_val[1] - goal[1]) < speed){
    target_val[1] = goal[1];
  }else{
    if(target_val[1] < goal[1]) target_val[1] += speed;
    else if(target_val[1] > goal[1]) target_val[1] -= speed;
  }

  if(abs(target_val[4] - goal[2]) < speed){
    target_val[4] = goal[2];
  }else{
    if(target_val[4] < goal[2]) target_val[4] += speed;
    else if(target_val[4] > goal[2]) target_val[4] -= speed;
  }
}

// void mx_move(double cur_height, double target_height){
//   double target_val3 = (target_height - 19.63)/worm_pitch;
//     if(abs(target_val[3] - target_val3) < rad(5)){
//     target_val[3] = target_val3;
//   }else{
//     if(target_val[3] < target_val3) target_val[3] += rad(vel_mx_write);
//     else if(target_val[3] > target_val3) target_val[3] -= rad(vel_mx_write);
//   }
//   if(abs(cur_height - target_height) < 0.5){
//     phase_mxdown = false;
//     phase_mxup = false;
//   }
// }

void choose_phase(){
  if(!paused || arrived ){
    phase1= false;
    phase2 = false;
    phase3= false;
  }
  // if(t > 1 && t < 1.8 ){
  //   ROS_INFO("phase: MX_DOWN......");
  //   phase_mxdown = true;
  //   phase_goto = false;
  //   phase_mxup = false;
  // }
  // else if(t > 3 && t < 6 ){
  //   ROS_INFO("phase: MOVING ARM.....");
  //   phase_mxdown = false;
  //   phase_goto = true;
  //   phase_mxup = false;
  // }
  // else if(t > 6.2 && t < 7 ){
  //   ROS_INFO("phase: MX_UP.........");
  //   phase_mxdown = false;
  //   phase_goto = false;
  //   phase_mxup = true;
  // }
  // else if(t > 7.5 && t < 8){
  //   if(arrived){
  //     opening = false; // open arm
  //     t = 0;
  //   }
  // }
}

void make_move(std::vector<double> &target_val, std::vector<double> &theta){
  double signx = vel_ax_x >= 0? 1:-1;
  double signy = vel_ax_y >= 0? 1:-1;
  double x = l*(sin(theta[0]) + sin(theta[1]));
  double y = l*(cos(theta[0]) - cos(theta[1]));
  double z = (theta[4] - rad(75)) * 2.5;
  
  double dtheta1 = signx * speed_x;
  double dtheta2 = signy * speed_y;

  // joy stick control
  if(abs(vel_ax_x) > 0.8 && is_ok(target_val[0] + dtheta1, target_val[1] + dtheta1 * sin(theta[0])/sin(theta[1]))){
    target_val[0] += dtheta1 ; //radians
    target_val[1] += dtheta1 * sin(theta[0])/sin(theta[1]);
    t = 0;
  }
  else if(abs(vel_ax_y) > 0.8 && !fixed_y && is_ok(target_val[0] - dtheta2 * cos(theta[1])/cos(theta[0]), target_val[1] + dtheta2)){ 
    target_val[1] += dtheta2;
    target_val[0] += -dtheta2 * cos(theta[1])/cos(theta[0]);
    t = 0;
  }
  // autonomous
  else if(autonomous && !arrived){
    // paused_collection = true;
    double _x = tomato_x - armdim_x;
    double _y =  tomato_y;
    double _z = tomato_z + rad(75)* 2.5 ;//ARM ERROR
    std::vector<double> pos1 = {_x - tomato_size/2, _y, _z, half_closed, speed_1};
    std::vector<double> pos2 = {_x, _y, _z, half_closed, speed_2};
    std::vector<double> pos3 = {_x + tomato_size*3/2, _y + tomato_size, _z, half_closed, speed_2};
    std::vector<double> pos4 = {_x , _y, _z, closed, speed_3};

    std::vector<double> final_val = { 
      asin(sqrt((_x*_x+_y*_y))/2/l) - atan(_y/_x),
      asin(sqrt((_x*_x+_y*_y))/2/l) + atan(_y/_x), 
      opened,
      0.0,
      _z/2.5
    };

    if(!is_ok(_x - 2, _y)) paused = false;
    else paused = true;

    if(t > 1 && t < 3){
      go_to(pos1[0], pos1[1], pos1[2], target_val, pos1[4]);
      target_val[2] = pos1[3];

    }else if(t > 3.5 && t < 4.5){
      go_to(pos2[0], pos2[1], pos2[2], target_val, pos2[4]);
      target_val[2] = pos2[3];
    }else if(t > 5&& t < 6.5){
      go_to(pos3[0], pos3[1], pos3[2], target_val, pos3[4]);
      target_val[2] = pos3[3];
    }else if(t > 7 && t < 13){
      go_to(pos4[0], pos4[1], pos4[2], target_val, pos4[4]);
      target_val[2] = pos4[3];
      arrived = is_arrived(theta, pos4);
    }


    if(t > 14) go_back(theta, target_val); //reset
  }

  // states for going back 
  if((arrived && t > 1.5) || goback){
    go_back(theta, target_val);
  }

  if(!autonomous) target_val[2] = opening ? opened: closed;
  mx_pos = theta[3] * worm_pitch + 19.63; //calibrated
  target_val[3] += rad(vel_mx_write);
  target_val[4] += vel_ax_1; 
  
  if(abs(t - round(t)) < 0.03){
    ROS_INFO("tomato_xyz: [%f, %f, %f]", tomato_x, tomato_y, tomato_z);
    ROS_INFO("COORDS: [%f, %f, %f]", x, y, z);
    ROS_INFO("t: %f", t);
    ROS_INFO("%f", mx_pos);
    ROS_INFO("goal: %f, %f, %f," , tomato_x - armdim_x, tomato_y, tomato_z + rad(75)* 2.5 + z);
    ROS_ERROR("paused = %d",  paused);
    ROS_ERROR("wait = %d", wait);

    if(tomato_x > 18) ROS_ERROR("TOO FAR !! move closer");
  }

  // time update
  t += 0.04;

  limitcheck(target_val);
}



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "param");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(500);
  ros::Subscriber subscriber = nh.subscribe("joy", 1, joyCallback);
  ros::Subscriber sub = nh.subscribe<yolo_detection::BoundingBoxArray>("tomato_detections", 1, bboxCallback);
  ros::Subscriber realsense_sub = nh.subscribe<geometry_msgs::PointStamped>("/realsense/tomato_3d_point", 1, realsensePointCallback);

  pnh.param<bool>("use_realsense", use_realsense, true);

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");
  DynamixelControl dynamixelcontrol(dev_name);

  // add motors
  dynamixelcontrol.addMotor("AX", 5);
  dynamixelcontrol.addMotor("AX", 4);
  dynamixelcontrol.addMotor("AX", 3);
  dynamixelcontrol.addMotor("MX", 10, "extended position control");
  dynamixelcontrol.addMotor("AX", 1);   //stage R/L
  
  dynamixelcontrol.torque_on();

  std::vector<double> target_positions{rad(-75), rad(105), opened, rad(1388), rad(75)};
  std::vector<double> current_values(5);

  while(ros::ok())
  {
    ros::spinOnce();

    make_move(target_positions, current_values);
    dynamixelcontrol.setTarget(target_positions); 
    dynamixelcontrol.write();
    dynamixelcontrol.read();  
    dynamixelcontrol.getCurrentvalues(current_values);
    
    cycle_rate.sleep();
  }
  
  dynamixelcontrol.torque_off();

  return 0;
}