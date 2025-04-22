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
        state = "Constructor: failed to open the port!";
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
        state = "Constructor: failed to set the baudrate!";
        ready_to_use = false;
        disp_trouble();
        return;
    }

    groupsyncwrite_p1 = std::make_shared<dynamixel::GroupSyncWrite>(portHandler, packetHandler_p1, ADDR_GOAL_POSITION_P1, 2/*Byte*/);
    groupbulkread_p2  = std::make_shared<dynamixel::GroupBulkRead> (portHandler, packetHandler_p2);
    groupbulkwrite_p2 = std::make_shared<dynamixel::GroupBulkWrite>(portHandler, packetHandler_p2);
}

DynamixelControl::~DynamixelControl()
{
    portHandler->closePort();
}

bool DynamixelControl::addMotor(std::string type, int id)
{
    if( !ready_to_use ) return false;

    if(type == "AX")
    {
        motorlist.push_back( std::make_unique< AXMotor >(id, AX_TORQUE_LIMIT_DEFAULT, portHandler, packetHandler_p1, groupsyncwrite_p1 ));
        return true;
    }
    else if (type == "MX")
    {
        motorlist.push_back( std::make_unique< MXMotor >(id, portHandler, packetHandler_p2, groupbulkread_p2, groupbulkwrite_p2) );
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

bool DynamixelControl::setTarget(const std::vector<double> &target_values)
{
    if( !ready_to_use ) return false;

    //この書き方ではVectorの順番を意識して合わせなければならない.

    bool result = false;
    if( target_values.size() == motorlist.size() )
    {
        for(int i=0; i<motorlist.size(); i++)
        {
            result =  motorlist[i] -> goalset(target_values[i]);
            if( result == false ) break;
        }
    }
    else
    {
        state = "setTarget: size of target is different from motor list's one.";
        disp_trouble();
        return false;
    }
    state = "setTarget: failed to set the targtet value";
    disp_trouble();
    return result;
}

bool DynamixelControl::write()
{
    if( !ready_to_use ) return false;

    int dxl_comm_result = COMM_TX_FAIL;

    // Protocol 1
    dxl_comm_result = groupsyncwrite_p1 -> txPacket();
    if( dxl_comm_result != COMM_SUCCESS ){
        packetHandler_p1 -> getTxRxResult(dxl_comm_result);
        state = "write: Failed to syncwrite";
        disp_trouble();
    }
    // else if (dxl_error != 0)
    // {
    //     packetHandler1->getRxPacketError(dxl_error);
    // }
    groupsyncwrite_p1 -> clearParam();

    // Protocol 2
    dxl_comm_result = groupbulkwrite_p2 -> txPacket();
    if ( dxl_comm_result != COMM_SUCCESS )
    {
        packetHandler_p2 -> getTxRxResult(dxl_comm_result);
        state = "write: Failed to syncwrite";
        disp_trouble();
    }
    groupbulkwrite_p2 -> clearParam();
    return true;
}

bool DynamixelControl::read()
{
    if( !ready_to_use ) return false;

    // Protocol 2.0
    int comm_result = groupbulkread_p2 -> txRxPacket();
    if (comm_result != COMM_SUCCESS )packetHandler_p2 -> getTxRxResult(comm_result);

    // Protocol 1.0 & 2.0
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

bool DynamixelControl::getCurrentvalues(std::vector<double> &current_values)
{
    if(motorlist.size() > current_values.size()) 
    {
        state = "getCurrentvalues: given vector is shorter than motors.";
        disp_trouble();
        return false;
    }

    for(int i=0; i<motorlist.size(); i++)
    {
        current_values[i] = motorlist[i]->get_current_value();
    }
    return true;
}

void DynamixelControl::disp_trouble()
{
    return;
}