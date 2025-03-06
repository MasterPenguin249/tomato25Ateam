#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include<sensor_msgs/Joy.h>

using namespace dynamixel;

/// Control table address for protocol 2 (MX, XC)
#define ADDR_OPERATING_MODE_P2   11
#define ADDR_TORQUE_ENABLE_P2    64
#define ADDR_GOAL_CURRENT_P2     102
#define ADDR_GOAL_VELOCITY_P2    104
#define ADDR_GOAL_POSITION_P2    116
#define ADDR_PRESENT_CURRENT_P2  126
#define ADDR_PRESENT_VELOCITY_P2 128
#define ADDR_PRESENT_POSITION_P2 132

// Control table address for protocol 1 (AX)
#define ADDR_TORQUE_ENABLE_P1    24
#define ADDR_GOAL_POSITION_P1    30
#define ADDR_PRESENT_POSITION_P1 36

// Protocol version
#define PROTOCOL_VERSION1      1.0
#define PROTOCOL_VERSION2      2.0

// Default setting
#define DXL_AX1_ID            1
#define DXL_AX2_ID            2
#define DXL_AX3_ID            3
#define DXL_AX4_ID            4
#define DXL_AX5_ID            5
#define DXL_MX_ID             10
#define DXL_XC_ID             20
#define BAUDRATE              1000000

#define CURRENT_MODE          0
#define VELOCITY_MODE         1
#define POSITION_MODE         3

#define NODE_FREQUENCY        200

PortHandler * portHandler;
PacketHandler * packetHandler1;
PacketHandler * packetHandler2;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint16_t position_mx_read = 0;
uint16_t position_mx_write = 2048; // 0~4095
int16_t dir = 5;

int main(int argc, char ** argv)
{
  
  ros::init(argc, argv, "mx_pos");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(NODE_FREQUENCY);

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");

  portHandler = PortHandler::getPortHandler(dev_name.c_str());
  packetHandler1 = PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
  packetHandler2 = PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  //// change operating mode. current/velocity/position mode
  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, CURRENT_MODE, &dxl_error);
  //dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, VELOCITY_MODE, &dxl_error);
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_OPERATING_MODE_P2, POSITION_MODE, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("Success to change mode for Dynamixel ID %d", DXL_MX_ID); 
  }else{
    ROS_ERROR("Failed to change mode for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }

  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_TORQUE_ENABLE_P2, 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_MX_ID); 
  }else{
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }

  while(ros::ok())
  {
    ros::spinOnce();

    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    
    // position mode
    dxl_comm_result = packetHandler2->write4ByteTxRx(portHandler, DXL_MX_ID, ADDR_GOAL_POSITION_P2, position_mx_write, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_MX_ID, position_mx_write);
    } else {
      ROS_INFO("Failed to set position! Result: %d", dxl_comm_result);
    }
   
    dxl_comm_result = packetHandler2->read2ByteTxRx(portHandler, DXL_MX_ID, ADDR_PRESENT_POSITION_P2, (uint16_t *)&position_mx_read, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_MX_ID, position_mx_read);
    } else {
      ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    }
    
    position_mx_write = position_mx_write + dir;
    if ((position_mx_write > 3000)|(position_mx_write  < 1000)) dir = -dir;
        
    cycle_rate.sleep(); 
  }
  
  dxl_comm_result = packetHandler2->write1ByteTxRx(portHandler, DXL_MX_ID, ADDR_TORQUE_ENABLE_P2, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL_MX_ID);
    return -1;
  }

  portHandler->closePort();

  return 0;
}
