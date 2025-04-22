#include "DynamixelControl/Motor.h"

Motor::Motor(int id, float version, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler):
    id(id),
    porthandler(porthandler), 
    packethandler(packethandler),
    protocol_version(version)
{};

Motor::~Motor(){};

bool Motor::protocol_version_check()
{
    float ph_protocol_ver = packethandler -> getProtocolVersion();
    if( ph_protocol_ver == protocol_version ) return true;
    else return false;
}

double Motor::get_goal_value()
{
    return goal_value;
}