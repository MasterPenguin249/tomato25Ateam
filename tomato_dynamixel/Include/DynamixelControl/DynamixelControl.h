#include <iostream>
#include <vector>
#include <memory>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "DynamixelControl/Motor.h"
#include "DynamixelControl/AXMotor.h"
#include "DynamixelControl/MXMotor.h"

struct DynamixelControl::CurrentValue
{
    int id;
    double current_value;
};


class DynamixelControl
{
private:
    bool ready_to_use;
    std::string state;
    void disp_trouble();

    std::vector<std::unique_ptr< Motor> > motorlist;
    
//Dynamixel
    dynamixel::PortHandler* porthandler;

    // Protocol 1
    dynamixel::PacketHandler* packetHandler_p1;
    std::unique_ptr<dynamixel::GroupSyncWrite> groupsyncwrite_p1;   // もしかしてsharedじゃないとモータークラスからアクセスできない？

    // Protocol 2
    dynamixel::PacketHandler* packetHandler_p2;
    std::unique_ptr<dynamixel::GroupBulkRead> groupbulkread_p2;
    std::unique_ptr<dynamixel::GroupBulkWrite> groupbulkwrite_p2;


public:
    DynamixelControl(std::string dev_name);
    ~DynamixelControl();

    bool addMotor(std::string type, int id/*, std::string mode */);
  
    bool torque_on();
    bool torque_off();

    bool setTarget(std::vector<double> target_values);  // 使いづらそう
    bool write();
    bool read();
  
    bool getCurrentvalues(std::vector<DynamixelControl::CurrentValue> &current_values);
};