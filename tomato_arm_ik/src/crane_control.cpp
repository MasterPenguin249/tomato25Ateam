#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

using namespace dynamixel;

// Control table address for protocol 2 (MX, XC)
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
#define ADDR_MOVING_SPEED_P1     32

// Protocol version
#define PROTOCOL_VERSION1       1.0
#define PROTOCOL_VERSION2       2.0

// Default setting
#define DXL_AX1_ID              1
#define DXL_AX2_ID              2
#define DXL_AX3_ID              3
#define DXL_AX4_ID              4
#define DXL_AX5_ID              5
#define DXL_MX_ID               10
#define DXL_XC_ID               20
#define BAUDRATE                1000000

//MODE
#define CURRENT_MODE            0
#define VELOCITY_MODE           1
#define POSITION_MODE           3

#define NODE_FREQUENCY          200

#define AX_POS_UPPER_LIMIT      1023
#define AX_POS_LOWER_LIMIT      0

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;

PortHandler * portHandler;
PacketHandler * packetHandler1;
PacketHandler * packetHandler2;

uint16_t DXL_IDS[5] = {1, 2, 3, 4, 5};

// torque function for AX series to change Torque Enable
// OnOff = 1 (Torque ON), 0 (Torque OFF)
void torque(uint8_t DXL_ID, uint8_t OnOff){
    dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE_P1, OnOff, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        // ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_ID); 
    }else{
        ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL_ID);
    }
}

// write function for AX series to write goal position
void write_pos(uint8_t DXL_ID, uint16_t write_pos){
    dxl_comm_result = packetHandler1->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION_P1, write_pos, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        //ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", DXL_AX1_ID, position_write);
    } else {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
}

// callback function
void JointStateCallback(sensor_msgs::JointState output){
    for (int i=0; i<5; i++){
        int target_position = output.position[i] /  ( 2 *M_PI ) * ( AX_POS_UPPER_LIMIT + 1 ) + 512;
        write_pos(DXL_IDS[i], target_position);
    }
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "crane_control");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Rate cycle_rate(NODE_FREQUENCY);
    ros::Subscriber joint_state_sub = nh.subscribe("joint_states", 10, JointStateCallback);

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

    // Torque On
    for (int i = 0; i++; i < sizeof(DXL_IDS) / sizeof(int)){
        torque(DXL_IDS[i], 1);   
    }

    while(ros::ok()){
        ros::spinOnce();
        dxl_error = 0;
        dxl_comm_result = COMM_TX_FAIL;
        cycle_rate.sleep();
    }

    // Torque Off
    for (int i = 0; i++; i < sizeof(DXL_IDS) / sizeof(int)){
        torque(DXL_IDS[i], 0);
    }

    portHandler->closePort();

    return 0;
}