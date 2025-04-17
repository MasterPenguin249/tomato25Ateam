#include <ros/ros.h>
#include <cmath>
#include <memory>
#include "DynamixelControl/DynamixelControl.h"

void make_move(std::vector<double> &target_val)
{
    static double move_count_ax = 0;
    static double move_count_mx = 0;
    
    int size = target_val.size();
    if(size != 6) return;

    for(int i = 0; i<5; i++)
    {
        if(i%2 == 0)
        {
            target_val[i] = (45.0*M_PI/180.0)*sin(move_count_ax);
        }else{
            target_val[i] = -(45.0*M_PI/180.0)*sin(move_count_ax);
        }
    }
    if(move_count_mx < 1.0)
    {
        target_val[5] = 3.0;
    }
    else
    {
        target_val[5]= -3.0;    
        if(move_count_mx > 2.0)
        {
            move_count_mx = 0;
        }
    }
    move_count_ax += 0.03;
    move_count_mx += 0.03;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ax_class_test");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Rate cycle_rate(500);

    std::string dev_name;
    pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");

    DynamixelControl dynamixelcontrol(dev_name);

    dynamixelcontrol.addMotor("AX", 1);
    dynamixelcontrol.addMotor("AX", 2);
    dynamixelcontrol.addMotor("AX", 3);
    dynamixelcontrol.addMotor("AX", 4);
    dynamixelcontrol.addMotor("AX", 5);
    dynamixelcontrol.addMotor("MX",10);

    dynamixelcontrol.torque_on();

    std::vector<double> target_positions(6, 0.0);
    std::vector<double> current_values(6);

    while (ros::ok())
    {
        ros::spinOnce();

        make_move(target_positions);
        dynamixelcontrol.setTarget(target_positions);   // この処理まとめたほうが良さそう
        dynamixelcontrol.write();

        dynamixelcontrol.read();        // ここも同様
        dynamixelcontrol.getCurrentvalues(current_values);

        cycle_rate.sleep();
    }

    dynamixelcontrol.torque_off();

    return 0;
}