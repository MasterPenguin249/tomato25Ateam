#include "DynamixelControl/DynamixelControl.h"

DynamixelControl::DynamixelControl(std::string dev_name):
    ready_to_use(false),
    state(" ")
{
    portHandler      = dynamixel::PortHandler::getPortHandler(dev_name.c_str());
    packetHandler_p1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
    packetHandler_p2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);

    if( portHandler->openPort() )
    {
        ready_to_use = true;
    }
    else
    {
        state = "Constructor: failed to open the port!"
        ready_to_use = false;
        disp_trouble();
        return;
    }

    if( portHandler->setBaudRate(BAUDRATE) )
    {
        ready_to_use = true;
    }
    else
    {
        state = "Constructor: failed to set the baudrate!"
        ready_to_use = false;
        disp_trouble();
        return;
    }

    groupsyncwrite_p1 = std::make_unique<dynamixel::GroupSyncWrite>(portHandler, packetHandler_p1, ADDR_GOAL_POSITION_P1, 2/*Byte*/);
    groupbulkread_p2  = std::make_unique<dynamixel::GroupBulkRead> (portHandler, packetHandler_p2);
    groupbulkwrite_p2 = std::make_unique<dynamixel::GroupBulkWrite>(portHandler, packetHandler_p2);
};

DynamixelControl::~DynamixelControl()
{

};

bool DynamixelControl::addMotor(std::string type, int id)
{
    if( !ready_to_use ) return false;

    if(type == "AX")
    {
        motorlist.pushback( std::make_unique< AXMotor >(id, portHandler, packetHandler_p1, groupsyncwrite_p1.get()) );
        return true;
    }
    else if (type == "MX")
    {
        motorlist.pushback( std::make_unique< MXMotor >(id, porthandler, packetHandler_p2, groupbulkread_p2.get(), groupbulkwrite_p2.get()) );
        return true;
    }
    
    state = "addMotor: Wrong motor type is assigned.";
    disp_trouble();
    return false;
}

bool DynamixelControl::torque_on()
{
    if( !ready_to_use ) return false;

    bool result = false;
    for( auto& motor : motorlist)   // 範囲for文
    {
        result = motor -> torque_on();
        if(result == false) break;
    }

    state = "torque_on: failed to turn on the torque";
    disp_trouble();
    return result;
}

bool DynamixelControl::torque_off()
{
    if( !ready_to_use ) return false;

    bool result = false;
    for( auto& motor : motorlist)   // 範囲for文
    {
        result = motor -> torque_off();
        if(result == false) break;
    }

    state = "torque_off: failed to turn off the torque";
    disp_trouble();
    return result;
}

bool DynamixelControl::setTarget(std::vector<double> target_values)
{
    if( !ready_to_use ) return false;

    //この書き方ではVectorの順番を意識して合わせなければならない.
    //それは使いづらくないか？

    bool result = false;
    if( target_values.size() = motorlist.size() )
    {
        for(int i=0; i<motorlist.size(); i++)
        {
            result =  motorlist[i] -> goalset(target_values[i]);
            if( result == false ) break;
        }
    }
    state = "setTarget: failed to set the targtet value";
    disp_trouble();
    return result;
}

bool DynamixelControl::write()
{

}

bool DynamixelControl::read()
{
    if( !ready_to_use ) return false;

    bool result = false;
    for( auto& motor : motorlist)   // 範囲for文
    {
        result = motor -> read();
        if(result == false) break;
    }

    state = "read: failed to read values";
    disp_trouble();
    return result;
}

bool DynamixelControl::getCurrentvalues(std::vector<DynamixelControl::CurrentValue> &current_values)
{

}