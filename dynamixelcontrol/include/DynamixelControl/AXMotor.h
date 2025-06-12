#ifndef AXMOTOR_H
#define AXMOTOR_H

#include <string>
#include <cmath>
#include <memory>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "/home/ting/catkin_ws/src/tomato/dynamixelcontrol/include/DynamixelControl/Motor.h"

// Control table address for protocol 1 (AX)
#define ADDR_TORQUE_ENABLE_P1    24
#define ADDR_GOAL_POSITION_P1    30
#define ADDR_TORQUE_LIMIT_P1     34
#define ADDR_PRESENT_POSITION_P1 36

#define CURRENT_MODE          0
#define VELOCITY_MODE         1
#define POSITION_MODE         3

#define PROTOCOL_VERSION1      1.0


class AXMotor : public Motor
{
private:
    double current_position;    // [rad]
    unsigned int torque_limit_per;

    std::weak_ptr<dynamixel::GroupSyncWrite> groupsyncwrite;

public:
    AXMotor(int id, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler, std::shared_ptr<dynamixel::GroupSyncWrite> groupsyncwrite);
    AXMotor(int id, unsigned int torque_limit_percent, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler, std::shared_ptr<dynamixel::GroupSyncWrite> groupsyncwrite);
    ~AXMotor();

    // const float protocol_version = 1.0;
    
    bool torque_on () override;
    bool torque_off() override;
    
    bool goalset(double goal /*[rad]*/) override;
    bool read() override;

    double get_current_value() override;
    double get_current_position();
};

// 基本は位置制御モード。CW/CCW AngleLimit の値を0にすることで無限回転モードにも可能。
// ただし、300~30の間は不定値となるため注意が必要

#endif