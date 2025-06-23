#include <cmath> 
#include <cmath> 
#include <memory>
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

double pos_mx_write = 0;
// double pos_mx_write = 45*(M_PI/180);
// float scale_mx = 6.0;
double current_position = 0;
double delta = 0;

void joyCallback(const sensor_msgs::Joy& msg)
{
  // pos_mx_write = msg.axes[1]*scale_mx;      
  delta = (5.0 * M_PI /180)*msg.axes[1];
  // ROS_INFO("delta = %f", delta);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "mx_class_ex_position_test");
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

  std::shared_ptr<GroupBulkWrite> groupBulkWrite = std::make_shared<GroupBulkWrite>(portHandler, packetHandler2);
  std::shared_ptr<GroupBulkRead> groupBulkRead = std::make_shared<GroupBulkRead>(portHandler, packetHandler2);

  // get MX Motor Object
  MXMotor mxmotor_10(10,portHandler, packetHandler2, groupBulkRead, groupBulkWrite, "extended position control");

  // torque on
  bool torqueon =  mxmotor_10.torque_on();
  if( !torqueon ) return -1;

    // BulkRead
  dxl_comm_result = groupBulkRead->txRxPacket();
  if ( dxl_comm_result != COMM_SUCCESS )packetHandler2 -> getTxRxResult(dxl_comm_result);

  // get data
  mxmotor_10.read();
  current_position = mxmotor_10.get_current_position();

  while(ros::ok())
  {
    ros::spinOnce();
    dxl_comm_result = COMM_TX_FAIL;

    // MX extended position control mode
    // BulkRead
    dxl_comm_result = groupBulkRead->txRxPacket();
    if ( dxl_comm_result != COMM_SUCCESS )packetHandler2 -> getTxRxResult(dxl_comm_result);

    // get data
    mxmotor_10.read();
    current_position = mxmotor_10.get_current_position();
    pos_mx_write = current_position + delta;

    //Goal set
    mxmotor_10.goalset(pos_mx_write);
   
    // BulkWrite
    dxl_comm_result = groupBulkWrite->txPacket();
    if ( dxl_comm_result != COMM_SUCCESS )packetHandler2 -> getTxRxResult(dxl_comm_result);
    groupBulkWrite->clearParam();


    ROS_INFO("[MXMotor ID:%d] Current Position: %f [deg]", mxmotor_10.id, (mxmotor_10.get_current_position()*180/M_PI ) );
    ROS_INFO("[MXMotor ID:%d] Goal Position: %f [deg]", mxmotor_10.id, (pos_mx_write*180/M_PI ) );
    
    cycle_rate.sleep();
  }

  // Torque Off
  bool torqueoff =  mxmotor_10.torque_off();
  if( !torqueoff ) return -1;

  portHandler->closePort();

  return 0;
}
