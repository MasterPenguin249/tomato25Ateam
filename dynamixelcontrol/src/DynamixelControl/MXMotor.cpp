#include "DynamixelControl/MXMotor.h"

MXMotor::MXMotor(int id, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler, std::shared_ptr<dynamixel::GroupBulkRead> groupbulkread, std::shared_ptr<dynamixel::GroupBulkWrite> groupbulkwrite, std::string control_mode):
    Motor(id, /*version =*/2.0, porthandler, packethandler)
{
    bool dxl_addparam_result = false;

    this->groupbulkread = groupbulkread;
    this->groupbulkwrite = groupbulkwrite;

    this -> modeset(control_mode);
 
    // mode = "velocity control";

    // velocity
    if(mode == "velocity control")
    {
        dxl_addparam_result = groupbulkread->addParam(id, ADDR_PRESENT_VELOCITY_P2,4/*byte*/);
        if( !dxl_addparam_result ){
            ROS_ERROR("[id: %d]: groupBulkread addparam failed.", id);
        }
    }
    else if(/*mode == "position_control" || */ mode == "extended position control")
    {
        // position
        dxl_addparam_result = groupbulkread->addParam(id, ADDR_PRESENT_POSITION_P2,4/*byte*/);
        if( !dxl_addparam_result ){
        ROS_ERROR("[id: %d]: groupBulkread addparam failed.", id);
    
        }
    }

};

MXMotor::~MXMotor(){};

double MXMotor::get_current_velocity()
{
    return current_velocity;
}

double MXMotor::get_current_position()
{
    return current_position;
}

double MXMotor::get_current_value()
{
    if(mode == "velocity control")
    {
        return get_current_velocity();
    }
    else if (mode == "extended position control")
    {
        return get_current_position();
    }
    
    ROS_ERROR("[Motor ID: %d] get_current_value : unknown mode", id);
    return 0;
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
    if( mode_in == "extended position control")
    {
        mode = mode_in;
        dxl_comm_result = packethandler->write1ByteTxRx(porthandler, id, ADDR_OPERATING_MODE_P2, EXTENDED_POSITION_MODE, &dxl_error);
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
        ROS_INFO("Success to enable torque for Dynamixel ID %d", id); 
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

    int write_data;
    // velocity control
    if(mode == "velocity control")
    {
        // rad/s で受け取る
        // vel[rpm] = val * 0.229[rpm]
        write_data = round(goal * (60.0 / (2.0*M_PI*0.229 )) );

        // 一応ソフト的に制限しておく　限界は1024. モーター側の設定でさらに制限される(確認時は400)
        if(write_data > MX_VELOCITY_LIMIT)
        {
            ROS_WARN("MXMotor[ID:%d]: goal is out of range!",id);
            write_data = MX_VELOCITY_LIMIT;
        }
        else if(write_data < -MX_VELOCITY_LIMIT)
        {
            ROS_WARN("MXMotor[ID:%d]: goal is out of range!",id);
            write_data = -MX_VELOCITY_LIMIT;
        }
    }
    else if(mode == "extended position control")
    {
        //  rad で受け取る
        //  [deg] = value * 360/4096 
        //  -> value = [rad] * pi/2048
        write_data = round( goal * (M_PI / 2048));

        // プログラム側からも角度制限
        // ハード的な可動範囲は -1048575 ~ + 1048575        source:https://www.besttechnology.co.jp/modules/knowledge/?MX%20Series%20Control%20table%282.0%29#a86abf50
        if(write_data > MX_EXPOSITON_LIMIT)
        {
            ROS_WARN("MXMotor[ID:%d]: goal is out of range!",id);
            write_data = MX_EXPOSITON_LIMIT;
        }
        else if(write_data < -MX_EXPOSITON_LIMIT)
        {
            ROS_WARN("MXMotor[ID:%d]: goal is out of range!",id);
            write_data = -MX_EXPOSITON_LIMIT;
        }
    }
    

    uint8_t param_goal_vel[4];
    param_goal_vel[0] = DXL_LOBYTE( DXL_LOWORD( write_data ) );
    param_goal_vel[1] = DXL_HIBYTE( DXL_LOWORD( write_data ) );
    param_goal_vel[2] = DXL_LOBYTE( DXL_HIWORD( write_data ) );
    param_goal_vel[3] = DXL_HIBYTE( DXL_HIWORD( write_data ) );
    bool dxl_addparam_result = false;

    if(mode == "velocity control" && !groupbulkwrite.expired()) 
    {
        std::shared_ptr<dynamixel::GroupBulkWrite> gbw =  groupbulkwrite.lock();
        dxl_addparam_result = gbw->addParam(id, ADDR_GOAL_VELOCITY_P2, 4/*byte*/, param_goal_vel);
    }
    else if(mode == "extended position control" && !groupbulkwrite.expired()) 
    {
        std::shared_ptr<dynamixel::GroupBulkWrite> gbw =  groupbulkwrite.lock();
        dxl_addparam_result = gbw->addParam(id, ADDR_GOAL_POSITION_P2, 4/*byte*/, param_goal_vel);
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
    if( groupbulkread.expired() ) return false;

    std::shared_ptr<dynamixel::GroupBulkRead> gbr =  groupbulkread.lock();
    int32_t mx_read ;   // = NULL のほうが適当か？

    // velocity mode
    if(mode == "velocity control")
    {
        dxl_getdata_result = gbr->isAvailable(id, ADDR_PRESENT_VELOCITY_P2, 4/*Byte*/);
        if ( dxl_getdata_result != true )
        {
            ROS_ERROR("[id:%d]: groupBulkRead getdata failed", id);
            return false;
        }
        mx_read = gbr->getData(id, ADDR_PRESENT_VELOCITY_P2, 4/*byte*/);
        current_velocity = (mx_read * 0.229) /60.0 * 2.0* M_PI;
    // ROS_INFO("get velocity : [ID:%d] -> [VELOCITY:%f]", id, current_velocity);

    }
    else if(mode == "extended position control")
    {
        dxl_getdata_result = gbr->isAvailable(id, ADDR_PRESENT_POSITION_P2, 4/*Byte*/);
        if ( dxl_getdata_result != true )
        {
            ROS_ERROR("[id:%d]: groupBulkRead getdata failed", id);
            return false;
        }
        mx_read = gbr->getData(id, ADDR_PRESENT_POSITION_P2, 4/*byte*/);
        current_position = (mx_read * (M_PI/2048));
    // ROS_INFO("get position : [ID:%d] -> [POSITION:%f]", id, current_position);
    }
    
    
    return true;
}