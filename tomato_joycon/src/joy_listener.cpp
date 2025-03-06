#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>


void joyCallback(const sensor_msgs::Joy& msg)
{
    ROS_INFO("axes[0] value: %f", msg.axes[0]);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "joy_listener");
    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("joy", 1, joyCallback);

    ros::spin();
    return 0;
}
