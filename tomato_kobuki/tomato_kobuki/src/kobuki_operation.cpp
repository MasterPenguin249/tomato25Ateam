#include "../include/kobuki_operation/kobuki_operation.hpp"
#include<sensor_msgs/Joy.h>

KobukiOperation::KobukiOperation(double freq) :
    _nh(),
    _freq(freq),
    _update_rate(freq),
    _control{0},
    _speed_acc(0.1),
    _turn_acc(0.6)

{
    _joy_sub = _nh.subscribe("joy",10,&KobukiOperation::joy_callback,this);
    _kobuki_pub = _nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);   
}

void KobukiOperation::joy_callback(const sensor_msgs::Joy &joy_msg)
{
    double _g_speed = 0.2;
    double motion_v_tmp = joy_msg.axes[7] * _g_speed;

    // double _g_turn = 1;
    // double rotation_v_tmp = _____;

    if( abs(joy_msg.axes[7])<=0.1 ){
        std::cout << "Stop" << std::endl;
        kobukiStop();
    }else{
        std::cout << "Move" << std::endl;
        kobukiMove(motion_v_tmp);
    }
}


void KobukiOperation::spin()
{
    ///< awaken timer
    ros::Duration(0.1).sleep();

    ///< change terminal setting
    struct termios old_terminal, new_terminal;
    int old_fcntl;
    tcgetattr(STDIN_FILENO, &old_terminal);
    new_terminal = old_terminal;
    new_terminal.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal);
    old_fcntl = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, old_fcntl | O_NONBLOCK);

    while (ros::ok())
    {
        ros::spinOnce();
        normalOperation();
    }
    kobukiStop();

    ///< restore terminal setting
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
    fcntl(STDIN_FILENO, F_SETFL, old_fcntl);
}


void KobukiOperation::kobukiMove(double speed)
{
    _control.target_speed = speed;
    kobukiInterpolate();
}

void KobukiOperation::kobukiStop()
{
    _control.target_speed = 0.0;
    kobukiInterpolate();
}

void KobukiOperation::normalOperation()
{
    kobukiInterpolate();
    _update_rate.sleep();
  
}

void KobukiOperation::kobukiInterpolate()
{
    _control.control_speed = _control.target_speed;

    geometry_msgs::Twist command;
    command.linear.x = _control.control_speed;
    command.angular.z = 0.0;
    _kobuki_pub.publish(command);
}