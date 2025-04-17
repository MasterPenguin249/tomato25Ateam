#include "DynamixelControl/AXMotor.h"


AXMotor::AXMotor(int id, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler, std::shared_ptr<dynamixel::GroupSyncWrite> groupsyncwrite):
    Motor(id, /*version=*/1.0, porthandler, packethandler), 
    torque_limit_per(5)
{
    this->groupsyncwrite = groupsyncwrite;
};

AXMotor::AXMotor(int id, unsigned int torque_limit_percent, dynamixel::PortHandler* porthandler, dynamixel::PacketHandler* packethandler, std::shared_ptr<dynamixel::GroupSyncWrite> groupsyncwrite ):
    Motor(id, /*version=*/1.0, porthandler, packethandler)
{
    this->groupsyncwrite = groupsyncwrite;
    if( torque_limit_percent > 90){
        torque_limit_per = 90;
    }else if (torque_limit_percent < 0)
    {
        torque_limit_per  = 0;
    }else{
        torque_limit_per = torque_limit_percent;
    }
    
}

AXMotor::~AXMotor(){};

double AXMotor::get_current_position()
{
    return current_position;
}

double AXMotor::get_current_value()
{
    return get_current_position();
}


bool AXMotor::torque_on()
{
    // Protocol Version Check
    if( protocol_version_check() )
    {
        // Protocol check is ok!
    }
    else
    {
        ROS_ERROR("Wrong Packet handler is used. AXMotor using Protocol 1.");
        return false;
    }

    uint8_t dxl_error = 0;

    uint16_t limit_data = (int)(1024 * torque_limit_per/100); 
    ROS_INFO("[AX Motor ID: %d]: Torque Limit is %d %% (%d)",id,torque_limit_per,limit_data);

    // Impose Torque Limit
    int result = packethandler -> write2ByteTxRx(porthandler,id, ADDR_TORQUE_LIMIT_P1, limit_data, &dxl_error);
    if (result == COMM_SUCCESS) {
        ROS_INFO("Success to impose torque limit for Dynamixel ID %d", id); 
    }else{
        ROS_ERROR("Failed to impose torque limit for Dynamixel ID %d", id);
        return false;
    }

    // Torque on 
    result = packethandler->write1ByteTxRx(porthandler, id, ADDR_TORQUE_ENABLE_P1, 1, &dxl_error);
    if (result == COMM_SUCCESS) {
        ROS_INFO("Success to enable torque for Dynamixel ID %d", id); 
        return true;
    }else{
        ROS_ERROR("Failed to enable torque for Dynamixel ID %d", id);
        return false;
    }
}

bool AXMotor::torque_off()
{
    // Protocol Version Check
    if( protocol_version_check() )
    {
        // Protocol check is ok!
    }
    else
    {
        ROS_ERROR("Wrong Packet handler is used. AXMotor using Protocol 1.");
        return false;
    }

    uint8_t dxl_error = 0;
    int result = packethandler->write1ByteTxRx(porthandler, id, ADDR_TORQUE_ENABLE_P1, 0, &dxl_error);
    if (result == COMM_SUCCESS) {
        //ROS_INFO("Success to enable torque for Dynamixel ID %d", DXL_AX1_ID); 
        return true;
    }else{
        ROS_ERROR("Failed to disable torque for Dynamixel ID %d",id);
        return false;
    }
}

bool  AXMotor::goalset(double goal) // WARNIG: this GroupSyncWrite Pointer should be Protocol 1.0. Be careful!!
{
    // groupsyncwriteのオブジェクトを作成する段階でpackethandlerを指定するため、
    // ここでprotocolのバージョンを確認できない。かなり用心して使用すべし。

    goal_value = goal;

    // ラジアンで受け取る
    int goal_data = round(goal * 3.0 /(5.0*M_PI) * 1024) + 512;
    if (goal_data > 1023){
        ROS_WARN("AXMotor[ID:%d]: goal is out of range!",id);
        goal_data = 1023;
    }
    else if (goal_data < 0){
        ROS_WARN("AXMotor[ID:%d]: goal is out of range!",id);
        goal_data = 0;
    }

    // ビット値のままで受け取る場合
    // int goal_data = goal;
    // double goal_rad = goal * (300 * M_PI / 180)/1024;
    // ROS_INFO("goal_pos : %f", goal_rad);

    // 配列に格納し直してポインタを合わせる
    uint8_t param_goal_position[2];
    param_goal_position[0] = DXL_LOBYTE( goal_data );
    param_goal_position[1] = DXL_HIBYTE( goal_data );

    bool dxl_addparam_result = false;
    if(!groupsyncwrite.expired()){
        std::shared_ptr<dynamixel::GroupSyncWrite> gsw = groupsyncwrite.lock();
        dxl_addparam_result = gsw->addParam(id, param_goal_position);
    }

    if (dxl_addparam_result != true)
    {
        ROS_ERROR("Failed to add parameter : id %d", id);
        return false;
    }

    return true;
}

// AXモーターはSyncReadができないのでそのまま読み取る

bool AXMotor::read()
{
    // Protocol Version Check
    if( protocol_version_check() )
    {
        // Protocol check is ok!
    }
    else
    {
        ROS_ERROR("Wrong Packet handler is used. AXMotor using Protocol 1.");
        return false;
    }

    uint16_t data_current_pos;
    uint8_t dxl_error;
    bool dxl_comm_result = packethandler->read2ByteTxRx(porthandler, id, ADDR_PRESENT_POSITION_P1, &data_current_pos, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      current_position = ( (double)data_current_pos - 512 )/1024.0 * 5.0 * M_PI / 3.0;
    //   ROS_INFO("getPosition : [ID:%d] -> [POSITION:%f[deg]]", id , current_position);
    } else {
      ROS_ERROR("Failed to get position! Result: %d", dxl_comm_result);
      return false;
    }

    return true;
}