#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include<sensor_msgs/Joy.h>

using namespace dynamixel;

// Control table address for protocol 2 (MX, XC)
#define ADDR_OPERATING_MODE_P2   11
#define ADDR_TORQUE_ENABLE_P2    64
#define ADDR_GOAL_CURRENT_P2     102
#define ADDR_GOAL_VELOCITY_P2    104
#define ADDR_GOAL_POSITION_P2    116
#define ADDR_PRESENT_CURRENT_P2 126
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
uint16_t position_ax_read[2];
uint16_t position_ax_write[2] = {512, 512}; // 0~1023
int16_t dir[2] = {5, 5};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ax_double_pos");
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
 
  // id1
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX1_ID, ADDR_TORQUE_ENABLE_P1, 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    //ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_AX1_ID); 
  }else{
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_AX1_ID);
    return -1;
  }

  // id2
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX2_ID, ADDR_TORQUE_ENABLE_P1, 1, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    //ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_AX2_ID); 
  }else{
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_AX2_ID);
    return -1;
  }

  while(ros::ok())
  {
    ros::spinOnce();

    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    
    // position mode
    // id1
    dxl_comm_result = packetHandler1->write2ByteTxRx(portHandler, DXL_AX1_ID, ADDR_GOAL_POSITION_P1, position_ax_write[0], &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_AX1_ID, position_ax_write[0]);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
    // id2
    dxl_comm_result = packetHandler1->write2ByteTxRx(portHandler, DXL_AX2_ID, ADDR_GOAL_POSITION_P1, position_ax_write[1], &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_AX2_ID, position_ax_write[1]);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
   
    // id1
    dxl_comm_result = packetHandler1->read2ByteTxRx(portHandler, DXL_AX1_ID, ADDR_PRESENT_POSITION_P1, (uint16_t *)&position_ax_read[0], &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      //ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_AX1_ID, position_ax_read[0]);
    } else {
      ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    }
    // id2
    dxl_comm_result = packetHandler1->read2ByteTxRx(portHandler, DXL_AX2_ID, ADDR_PRESENT_POSITION_P1, (uint16_t *)&position_ax_read[1], &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      //ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL_AX2_ID, position_ax_read[1]);
    } else {
      ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
    }
    
    position_ax_write[0] = position_ax_write[0] + dir[0];
    if ((position_ax_write[0] > 700)|(position_ax_write[0]  < 300)) dir[0] = -dir[0];

    position_ax_write[1] = position_ax_write[1] + dir[1];
    if ((position_ax_write[1] > 700)|(position_ax_write[1]  < 300)) dir[1] = -dir[1];

    cycle_rate.sleep();
  }
  
  // id1
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX1_ID, ADDR_TORQUE_ENABLE_P1, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL_AX1_ID);
    return -1;
  }
  // id2
  dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_AX2_ID, ADDR_TORQUE_ENABLE_P1, 0, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to disable torque for Dynamixel ID %d", DXL_AX2_ID);
    return -1;
  }

  portHandler->closePort();

  return 0;
}
