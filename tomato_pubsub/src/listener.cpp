#include <ros/ros.h>
#include <std_msgs/String.h>

void chatterCallback(const std_msgs::String& msg){
        ROS_INFO("this node subscribed: %s", msg.data.c_str());
}
  
int main(int argc, char** argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("chatter", 10, chatterCallback);

    ros::spin();
    return 0;
}
