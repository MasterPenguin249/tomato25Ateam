#include <string>
#include <cmath>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

/// Control table address for protocol 2 (MX, XC)
#define ADDR_OPERATING_MODE_P2   11
#define ADDR_TORQUE_ENABLE_P2    64
#define ADDR_GOAL_CURRENT_P2     102
#define ADDR_GOAL_VELOCITY_P2    104
#define ADDR_GOAL_POSITION_P2    116
#define ADDR_PRESENT_CURRENT_P2  126
#define ADDR_PRESENT_VELOCITY_P2 128
#define ADDR_PRESENT_POSITION_P2 132

// Protocol version
#define PROTOCOL_VERSION1      1.0
#define PROTOCOL_VERSION2      2.0

// Default setting
#define BAUDRATE              1000000

#define CURRENT_MODE          0
#define VELOCITY_MODE         1
#define POSITION_MODE         3

// MXモーターはプロトコル1.0, 2.0の両方で通信可能
// 今回は他の拡張用モーターとの兼ね合いや、機器がすでにプロトコル2.0で設定されているため
// プロトコル2.0で通信を行う


class MXMotor
{
private:
    double current_position;    //[rad]
    double current_velocity;    //[rad/s]
    double goal_value;          //[rad] or [rad/s] or [mA]?
    std::string mode;           //"velocity control", "current base position control"

    bool protocol_version_check(dynamixel::PacketHandler* packethandler);
    
public:
    MXMotor(int id);
    // MXMotor(int id, unsigned int limit_per);     // Impose torque limit
    ~MXMotor();

    const float protocol_version = 2.0;
    const int id;

    double get_current_position();
    double get_goal_value();

    bool modeset(std::string mode_in, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler);
    // bool read_addparam_pos(dynamixel::GroupBulkRead* groupbulkread);
    bool read_addparam_vel(dynamixel::GroupBulkRead* groupbulkread);

    bool torque_on (dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler);
    bool torque_off(dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler);
    
    bool goalset(double goal, dynamixel::GroupBulkWrite* groupbulkwrite);
    // bool readposition(dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler);
    bool readvelocity(dynamixel::GroupBulkRead* groupbulkread);

    // 多態性をもたせるにはporthandler,packethandler, groupbulkread, groupbulkwriteとかはメンバ変数としてポインタを持ったほうがいいかも
    // 同じスコープで呼び出せなくなる気がする
};