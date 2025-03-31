#include <cmath> 
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "DynamixelControl/MXMotor.h"

using namespace dynamixel;

// Protocol version
#define PROTOCOL_VERSION2      2.0

// Default setting
#define BAUDRATE              1000000

#define NODE_FREQUENCY        200

PortHandler * portHandler;
PacketHandler * packetHandler2;

int dxl_comm_result = COMM_TX_FAIL;

int16_t vel_mx_write = 0; // -285 ~ 285

float scale_mx = 300.0;

void joyCallback(const sensor_msgs::Joy& msg)
{
  vel_mx_write = msg.axes[1]*scale_mx;      

  if (vel_mx_write > 250){
    vel_mx_write = 250;
  }else if (vel_mx_write  < -250) {
    vel_mx_write = -250;
  }
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "mx_class_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(NODE_FREQUENCY);
  ros::Subscriber subscriber = nh.subscribe("joy", 1, joyCallback);

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");

  portHandler = PortHandler::getPortHandler(dev_name.c_str());
  packetHandler2 = PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  GroupBulkWrite groupBulkWrite(portHandler, packetHandler2);

  GroupBulkRead groupBulkRead(portHandler, packetHandler2);

  // get MX Motor Object
  MXMotor mxmotor_10(10,portHandler, packetHandler2);

  // set velocity control mode
  bool modeset = mxmotor_10.modeset("velocity control");
  if(!modeset) return -1;

  // torque on
  bool torqueon =  mxmotor_10.torque_on();
  if( !torqueon ) return -1;

  // Add Read Group
  bool dxl_addparam_result = false;
  dxl_addparam_result = mxmotor_10.read_addparam_vel(&groupBulkRead);


  while(ros::ok())
  {
    ros::spinOnce();
    dxl_comm_result = COMM_TX_FAIL;

    // MX velocity mode
    //Goal set
    mxmotor_10.goalset(vel_mx_write, &groupBulkWrite);
   
    // BulkWrite
    dxl_comm_result = groupBulkWrite.txPacket();
    if ( dxl_comm_result != COMM_SUCCESS )packetHandler2 -> getTxRxResult(dxl_comm_result);
    groupBulkWrite.clearParam();

    // BulkRead
    dxl_comm_result = groupBulkRead.txRxPacket();
    if ( dxl_comm_result != COMM_SUCCESS )packetHandler2 -> getTxRxResult(dxl_comm_result);

    // get data
    mxmotor_10.readvelocity(&groupBulkRead);
    
    cycle_rate.sleep();
  }

  // Torque Off
  bool torqueoff =  mxmotor_10.torque_off();
  if( !torqueoff ) return -1;

  portHandler->closePort();

  return 0;
}
