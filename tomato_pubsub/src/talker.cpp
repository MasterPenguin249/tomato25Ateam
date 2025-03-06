#include <ros/ros.h>
#include <std_msgs/String.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    ros::Publisher chatter_publisher = nh.advertise<std_msgs::String>("chatter", 10);
    ros::Rate loop_rate(10);

    while (ros::ok()){
        std_msgs::String msg;
        msg.data = "Hello world";
        ROS_INFO("this node published: %s", msg.data.c_str());
        chatter_publisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
