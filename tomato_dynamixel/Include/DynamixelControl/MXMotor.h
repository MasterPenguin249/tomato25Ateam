#include <string>
#include <cmath>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "DynamixelControl/Motor.h"

/// Control table address for protocol 2 (MX, XC)
#define ADDR_OPERATING_MODE_P2   11
#define ADDR_TORQUE_ENABLE_P2    64
#define ADDR_GOAL_CURRENT_P2     102
#define ADDR_GOAL_VELOCITY_P2    104
#define ADDR_GOAL_POSITION_P2    116
#define ADDR_PRESENT_CURRENT_P2  126
#define ADDR_PRESENT_VELOCITY_P2 128
#define ADDR_PRESENT_POSITION_P2 132

#define CURRENT_MODE          0
#define VELOCITY_MODE         1
#define POSITION_MODE         3

// MXモーターはプロトコル1.0, 2.0の両方で通信可能
// 今回は他の拡張用モーターとの兼ね合いや、機器がすでにプロトコル2.0で設定されているため
// プロトコル2.0で通信を行う


class MXMotor : public Motor
{
private:
    double current_velocity;    //[rad/s]
    std::string mode;           //"velocity control", "current base position control"

    dynamixel::GroupBulkRead* groupbulkread;
    dynamixel::GroupBulkWrite* groupbulkwrite;
    
public:
    MXMotor(int id, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler, dynamixel::GroupBulkRead* groupbulkread, dynamixel::GroupBulkWrite* groupbulkwrite);
    ~MXMotor();

    // const float protocol_version = 2.0;

    double get_current_velocity();
    double get_goal_value();

    bool torque_on () override;
    bool torque_off() override;

    bool goalset(double goal) override;
    bool read() override;

    bool modeset(std::string mode_in);
};