#ifndef DYNAMIXELCONTROL_H
#define DYNAMIXELCONTROL_H

#include <iostream>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "DynamixelControl/AXMotor.h"
#include "DynamixelControl/MXMotor.h"

#define AX_TORQUE_LIMIT_DEFAULT 30

class DynamixelControl
{
private:
    bool ready_to_use;
    std::string state;
    void disp_trouble();

    std::vector<std::unique_ptr< Motor> > motorlist;
    
//Dynamixel
    dynamixel::PortHandler* portHandler;    // staticで所有権はない（と思われる）

    // Protocol 1
    dynamixel::PacketHandler* packetHandler_p1; // staticで所有権はない（と思われる）
    std::shared_ptr<dynamixel::GroupSyncWrite> groupsyncwrite_p1;   

    // Protocol 2
    dynamixel::PacketHandler* packetHandler_p2; // staticで所有権はない（と思われる）
    std::shared_ptr<dynamixel::GroupBulkRead> groupbulkread_p2; 
    std::shared_ptr<dynamixel::GroupBulkWrite> groupbulkwrite_p2;


public:
    DynamixelControl(std::string dev_name);
    ~DynamixelControl();

    bool addMotor(std::string type, int id);
    bool addMotor(std::string type, int id, std::string mode);  // 制御モードを選ぶ場合
  
    bool torque_on();
    bool torque_off();

    bool setTarget(const std::vector<double>& target_values);  // 使いづらそう
    bool write();
    bool read();
  
    bool getCurrentvalues(std::vector<double> &current_values);
};

#endif