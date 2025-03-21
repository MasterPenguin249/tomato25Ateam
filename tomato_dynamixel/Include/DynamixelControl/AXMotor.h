#include <string>
#include <cmath>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

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
#define ADDR_TORQUE_LIMIT_P1     34
#define ADDR_PRESENT_POSITION_P1 36

// Protocol version
#define PROTOCOL_VERSION1      1.0
#define PROTOCOL_VERSION2      2.0

// Default setting
#define BAUDRATE              1000000

#define CURRENT_MODE          0
#define VELOCITY_MODE         1
#define POSITION_MODE         3



class AXMotor
{
private:
    const float protocol_version = 1.0;
    double current_position;    // [rad]
    double goal_value;          // [rad] (or [rad/s] ?)
    unsigned int torque_limit_per;

    bool protocol_version_check(dynamixel::PacketHandler* packethandler);

public:
    AXMotor(int id);
    AXMotor(int id, unsigned int torque_limit_percent);
    ~AXMotor();

    const int id;

    double get_current_position();
    double get_goal_value();
    
    bool torque_on(dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler);
    bool torque_off(dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler);
    
    bool goalset(double goal /*[rad]*/, dynamixel::GroupSyncWrite* groupsyncwrite);
    bool readposition(dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler);
};

// 基本は位置制御モード。CW/CCW AngleLimit の値を0にすることで無限回転モードにも可能。
// ただし、300~30の間は不定値となるため注意が必要