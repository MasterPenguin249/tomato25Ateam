#include <cmath>
#include <memory>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "DynamixelControl/AXMotor.h"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler1;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
// uint16_t position_ax_read = 0;
// uint16_t position_ax_write = 512; // 0~1023
// int16_t dir = 5;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ax_class_test");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(200);

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");

  portHandler = dynamixel::PortHandler::getPortHandler(dev_name.c_str());
  packetHandler1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
 
  // add SyncWrite group
  std::shared_ptr<dynamixel::GroupSyncWrite> groupSyncWrite = std::make_shared<dynamixel::GroupSyncWrite>(portHandler, packetHandler1, ADDR_GOAL_POSITION_P1, 2/* Byte*/);

  // get AX Motor Object
  AXMotor axmotor_5(5,90, portHandler, packetHandler1, groupSyncWrite);
  // AXMotor axmotor_4(4,10, portHandler, packetHandler1, groupSyncWrite);

  // Torque On
  bool torque_result = axmotor_5.torque_on();
  if(!torque_result) return -1;
  
  // torque_result = axmotor_4.torque_on();
  // if(!torque_result) return -1;

  double position_ax_write =  (0 * M_PI/180);
  double position_ax_read = 0;
  double t =0;

  while(ros::ok())
  {
    ros::spinOnce();
    // Read 
    axmotor_5.read();
    // axmotor_4.read();

    position_ax_read = axmotor_5.get_current_position();
    std::cout << position_ax_read  << std::endl;

    // position_ax_write = position_ax_read + 0.5;
    // reflesh target
    position_ax_write = (45.0 * M_PI /180.0)*sin(2.0*t);


    std::cout << position_ax_write  << std::endl;

    // SyncWrite
    axmotor_5.goalset(position_ax_write);
    // axmotor_4.goalset(position_ax_write);

    dxl_comm_result = groupSyncWrite->txPacket();
    if(dxl_comm_result != COMM_SUCCESS){
        packetHandler1 -> getTxRxResult(dxl_comm_result);
        ROS_ERROR("Failed to write goal ");
    }
    else if (dxl_error != 0)
    {
        packetHandler1->getRxPacketError(dxl_error);
    }

    groupSyncWrite->clearParam();

    
    cycle_rate.sleep();
    t += 1.0/200;
  }
  
  // torque_result = axmotor_4.torque_off();
  torque_result = axmotor_5.torque_off();
  if(!torque_result)return -1;

  portHandler->closePort();

  return 0;
}
