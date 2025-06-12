#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <memory>
#include <string>
#include <sensor_msgs/Joy.h>
#include "DynamixelControl/DynamixelControl.h"
#include <yolo_detection/BoundingBoxArray.h>
#include <yolo_detection/BoundingBox.h>

#define NODE_FREQUENCY        200

#define f1(x, y) -(cos(x) / cos(y))
#define f2(x, y) (sin(x) / sin(y))
#define rad(x) (x * M_PI/180)

float _gain_ax = 4.0;
float vel_ax_x = 0.0;
float vel_ax_y = 0.0;
double tomato_x = 9;
double tomato_y = 9;
double speed_x = 0.02;
double speed_y = 0.02;
double mx_speed = 0.0;
bool opening = true;
double closed = rad(-50);
double opened = rad(-10);
double l = 9; // cm


uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint16_t position_ax_read_1 = 0;//stage R/L
// uint16_t position_ax_read_2 = 0;//stage F/B
uint16_t position_ax_write_1 = 512; // 0~1023
// uint16_t position_ax_write_2 = 512; // 0~1023

int16_t vel_mx_read = 0;
int16_t vel_mx_write= 0; // -285 ~ 285

float scale_ax = 0.08;//4.0
float vel_ax_1 = 0.0;
float vel_ax_2 = 0.0;

float scale_mx = 30.0;
float vel_mx = 0.0;

// example range
// y(250, 220) minus 150-> (100, 70)?? (because the range is 0,300 not 0,360)
// x(90, 200) -> (-60, 50)??
// (130, 150 (rest)) -> (-20, 0) try 180

void joyCallback(const sensor_msgs::Joy& msg)
{
  if(msg.buttons[1]==1){
    vel_ax_1 = - msg.buttons[1]*scale_ax;
  }
  else if(msg.buttons[2]==1){
    vel_ax_1 = msg.buttons[2]*scale_ax;
  }
  else{
    vel_ax_1 = 0;
  }

  vel_ax_x = msg.axes[4]*_gain_ax;
  vel_ax_y = msg.axes[7]*_gain_ax;
  opening = msg.buttons[5] == 0;

  if(msg.buttons[3]==1){
    vel_mx_write= msg.buttons[3]*scale_mx;    
  }
  else if(msg.buttons[0]==1){
    vel_mx_write= -msg.buttons[0]*scale_mx;
  }
  else{
    vel_mx_write=0;
  }

  ROS_INFO("%d", msg.buttons[0]);
}

// void bboxCallback(const yolo_detection::BoundingBoxArray::ConstPtr& msg)
// {
//   // Process each bounding box
//   for (size_t i = 0; i < msg->bounding_boxes.size(); ++i)
//   {
//     const yolo_detection::BoundingBox& bbox = msg->bounding_boxes[i];
//     double distance = 2.8/((bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min))/2.0 *3111;
//     if(bbox.class_name != "tomato"){
//       break;
//     }
//     tomato_x = distance - 7;
//     tomato_y = ((bbox.x_min + bbox.x_max) / 2.0 - 300)/600 * 15 / 17 * distance + 9;
//     ROS_INFO("TOMATOOOOOO: [%f, %f]", tomato_x, tomato_y);
//   }

// }

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
    {rad(-target_val[0]+10), rad(-target_val[0]+130)},
    {rad(-150), rad(150)},
    {-100, 100},
    {rad(30), rad(120)}
  };
  // gatekeep
  for(int i = 0; i < target_val.size(); i++){
    if(target_val[i] < limits[i].first) target_val[i] = limits[i].first;
    else if (target_val[i] > limits[i].second) target_val[i] = limits[i].second;
  }

}

//Runge-Kutta
double solveRK4(double &x, double &y, double &h, std::string mode ){
  if(mode == "dtheta1"){
    double k1 = h * f1(x, y);
    double k2 = h * f1(x + 0.5*h, y + 0.5*k1);
    double k3 = h * f1(x + 0.5*h, y + 0.5*k2);
    double k4 = h * f1(x + h, y + k3);
    return 1/6*(k1 + 2*k2 + 2*k3 + k4);
  }
  if(mode == "dtheta2"){
    double k1 = h * f2(x, y);
    double k2 = h * f2(x + 0.5*h, y + 0.5*k1);
    double k3 = h * f2(x + 0.5*h, y + 0.5*k2);
    double k4 = h * f2(x + h, y + k3);
    return 1/6*(k1 + 2*k2 + 2*k3 + k4);
  }
}

// insert coordinates, arm goes there
void go_to(double _x, double _y, std::vector<double> &target_val){
  std::vector<double> goal(2);
  if(sqrt(_x*_x+_y*_y) > 2*l) ROS_ERROR("invalid square values");
  //convert to angle (I got them flipped the entire time)
  goal[1] = asin(sqrt((_x*_x+_y*_y))/2/l) + atan(_y/_x);
  goal[0] = asin(sqrt((_x*_x+_y*_y))/2/l) - atan(_y/_x);
  ROS_INFO("goal: [%f, %f]", _x, _y);

  ROS_INFO("goal(theta): [%f, %f]", goal[0]*180/M_PI, goal[1]*180/M_PI);
  if(target_val[0] < goal[0]) target_val[0] += speed_x;
  else if(target_val[0] > goal[0]) target_val[0] -= speed_x;
  if(target_val[1] < goal[1]) target_val[1] += speed_y;
  else if(target_val[1] > goal[1]) target_val[1] -= speed_y;
}

void make_move(std::vector<double> &target_val, std::vector<double> &theta){
  ROS_INFO("current value: [%f], [%f]", theta[0], theta[1]);
  static double t = 0;
  double ratiox = abs(vel_ax_x)/_gain_ax;
  double ratioy = abs(vel_ax_y)/_gain_ax;
  double signx = vel_ax_x != 0? vel_ax_x/abs(vel_ax_x): 1;
  double signy = vel_ax_y != 0? vel_ax_y/abs(vel_ax_y): 1;
  double x = l*(sin(theta[0]) + sin(theta[1]));
  double y = l*(cos(theta[0]) - cos(theta[1]));
  ROS_INFO("COORDS: [%f, %f]", x, y);
  double dtheta1 = signx * speed_x;//x_dot/l*(cos(theta[0])- sin(theta[0])/tan(theta[1]));
  double dtheta2 = signy * speed_y;

  // joy stick control
  // TODO use runge-kutta. (p.n never mind runge-kutta is actually worse)
  if(ratiox > 0.8){
    target_val[0] += dtheta1 ; //radians, most likely
    target_val[1] += dtheta1 * sin(theta[0])/sin(theta[1]);
    //solveRK4(theta[0], theta[1], dtheta1, "dtheta2");
    t = 0;
  }
  else if(ratioy > 0.8){ 
    target_val[1] += dtheta2;
    target_val[0] += -dtheta2 * cos(theta[1])/cos(theta[0]);
    // solveRK4(theta[1], theta[0], dtheta2, "dtheta1");
    t = 0;
  }
  // self 
  else if(ratiox < 0.8 && ratioy < 0.8 && t > 10000){
    //double _x = farstraight[point].first;
    //double _y = farstraight[point].second;
    go_to(tomato_x, tomato_y, target_val);
    // reinitialize
    // if(abs(x - _x) < 0.3 && abs(y - _y) < 0.3){ 
    //   point = ++point % farstraight.size();
    // }
  }

  target_val[2] = opening ? opened: closed;
  target_val[3] = vel_mx_write;
  target_val[4] += vel_ax_1; 
  

  ROS_INFO("target values: [%f, %f]", target_val[0], target_val[1]);
  ROS_INFO("t: %f", t);
  t += 0.02;
  limitcheck(target_val);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "param");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(500);
  ros::Subscriber subscriber = nh.subscribe("joy", 1, joyCallback);
  //ros::Subscriber sub = nh.subscribe<yolo_detection::BoundingBoxArray>("tomato_detections", 1, joyCallback);

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");
  DynamixelControl dynamixelcontrol(dev_name);

  // add motors
  dynamixelcontrol.addMotor("AX", 5);
  dynamixelcontrol.addMotor("AX", 4);
  dynamixelcontrol.addMotor("AX", 3);
  dynamixelcontrol.addMotor("MX", 10);
  dynamixelcontrol.addMotor("AX", 1);   //stage R/L
  

  dynamixelcontrol.torque_on();

  std::vector<double> target_positions{rad(-5), rad(85), opened, 0, rad(75)};
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
