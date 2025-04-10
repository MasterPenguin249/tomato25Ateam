#include "DynamixelControl/MXMotor.h"

MXMotor::MXMotor(int id, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler, dynamixel::GroupBulkRead* groupbulkread, dynamixel::GroupBulkWrite* groupbulkwrite):
    Motor(id, /*version =*/2.0, porthandler, packethandler),
    groupbulkread(groupbulkread), 
    groupbulkwrite(groupbulkwrite)
{
    bool dxl_addparam_result = false;
    // velocity
    dxl_addparam_result = groupbulkread->addParam(id, ADDR_PRESENT_VELOCITY_P2,4/*byte*/);
    if( !dxl_addparam_result ){
        ROS_ERROR("[id: %d]: groupBulkread addparam failed.", id);
    }

    // position
//     dxl_addparam_result = groupbulkread->addParam(id, ADDR_PRESENT_POSITION_P2,4/*byte*/);
//     if( !dxl_addparam_result ){
//         ROS_ERROR("[id: %d]: groupBulkread addparam failed.", id);
//     }
};

MXMotor::~MXMotor(){};

double MXMotor::get_current_velocity()
{
    return current_velocity;
}

bool MXMotor::modeset(std::string mode_in)
{
    // Protocol Version Check
    if( protocol_version_check() )
    {
        // Protocol Check is ok!
    }
    else
    {
        ROS_ERROR("Wrong Packet handler is used. MXMotor using Protocol 2.0.");
        return false;
    }

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    if(mode_in == "velocity control")
    {
        mode = mode_in;
        dxl_comm_result = packethandler->write1ByteTxRx(porthandler, id, ADDR_OPERATING_MODE_P2, VELOCITY_MODE, &dxl_error);
    }
    else
    {
        // 未実装 or 間違った指定
        ROS_ERROR("Wrong mode is selected.");
        return false;
    }

    if (dxl_comm_result == COMM_SUCCESS) {
        // ROS_INFO("Success to change mode for Dynamixel ID %d", id); 
        return true;
    }else{
        ROS_ERROR("Failed to change mode for Dynamixel ID %d", id);
        return false;
    }
}

bool MXMotor::torque_on()
{
    // Protocol Version Check
    if( protocol_version_check() )
    {
        // Protocol Check is ok!
    }
    else
    {
        ROS_ERROR("Wrong Packet handler is used. MXMotor using Protocol 2.0.");
        return false;
    }

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    
    dxl_comm_result = packethandler->write1ByteTxRx(porthandler, id, ADDR_TORQUE_ENABLE_P2, 1, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        // ROS_INFO("Success to enable torque for Dynamixel ID %d", id); 
        return true;
    }else{
        ROS_ERROR("Failed to enable torque for Dynamixel ID %d", id);
        return false;
    }
}

bool MXMotor::torque_off()
{
    // Protocol Version Check
    if( protocol_version_check() )
    {
        // Protocol Check is ok!
    }
    else
    {
        ROS_ERROR("Wrong Packet handler is used. MXMotor using Protocol 2.0.");
        return false;
    }

    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    
    dxl_comm_result = packethandler->write1ByteTxRx(porthandler, id, ADDR_TORQUE_ENABLE_P2, 0, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
        // ROS_INFO("Success to disable torque for Dynamixel ID %d", id); 
        return true;
    }else{
        ROS_ERROR("Failed to disable torque for Dynamixel ID %d", id);
        return false;
    }
}

bool MXMotor::goalset(double goal)
{
    // Add Write Group
    int16_t vel_write_data = (int)goal; // ひとまず整数値そのまま
    uint8_t param_goal_vel[4];
    param_goal_vel[0] = DXL_LOBYTE( DXL_LOWORD( vel_write_data ) );
    param_goal_vel[1] = DXL_HIBYTE( DXL_LOWORD( vel_write_data ) );
    param_goal_vel[2] = DXL_LOBYTE( DXL_HIWORD( vel_write_data ) );
    param_goal_vel[3] = DXL_HIBYTE( DXL_HIWORD( vel_write_data ) );
    bool dxl_addparam_result = false;
    if(mode == "velocity control") 
    {
        dxl_addparam_result = groupbulkwrite->addParam(id, ADDR_GOAL_VELOCITY_P2, 4/*byte*/, param_goal_vel);
    }
    else
    {
        return false;
    }

    if( !dxl_addparam_result){
        ROS_ERROR("[id:%d]: groupBulkWrite addparam failed.",id);
        return false;
    }
    return true;
}

bool MXMotor::read()
{
    bool dxl_getdata_result = false;
    dxl_getdata_result = groupbulkread->isAvailable(id, ADDR_PRESENT_VELOCITY_P2, 4/*Byte*/);
    if ( dxl_getdata_result != true )
    {
        ROS_ERROR("[id:%d]: groupBulkRead getdata failed", id);
        return false;
    }
    int16_t vel_mx_read = 0;
    vel_mx_read = groupbulkread->getData(id, ADDR_PRESENT_VELOCITY_P2, 4/*byte*/);
    
    current_velocity = (vel_mx_read * 0.229) /60 * 2* M_PI;
    ROS_INFO("get velocity : [ID:%d] -> [VELOCITY:%f]", id, current_velocity);
    return true;
}