#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "arm_ik.h"
#include<sensor_msgs/Joy.h>
#include <math.h>

float cmd_x;
float cmd_y;
float cmd_z;
float cmd_rot;

void joyCallback(const sensor_msgs::Joy& msg){
    float _gain_x = 0.001;
    float _gain_y = 0.001;
    float _gain_z = 0.001;
    float _gain_rot = 0.01;
    cmd_x = msg.axes[1] * _gain_x;
    cmd_y = msg.axes[0] * (-_gain_y);
    cmd_z = msg.axes[3] * _gain_z;
    cmd_rot = msg.axes[4] * _gain_rot;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "arm_ik_joy");
    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("target_point", 1);
    ros::Subscriber joy_sub = nh.subscribe("joy", 1, joyCallback);

    ArmMock arm_mock(0.05, 0.05, 0.15, 0.15, 0.1);
    ArmSolver arm_solver(0.05, 0.05, 0.15, 0.15, 0.1);
    ArmSmooth arm_smooth;
    
    geometry_msgs::PointStamped init_target_point;
    init_target_point.point.x = 0.2;
    init_target_point.point.y = 0.0;
    init_target_point.point.z = 0.3;
    float init_target_angle = 1.507;

    geometry_msgs::PointStamped target_point;
    target_point = init_target_point;
    float target_angle;
    target_angle = init_target_angle;

    ros::Rate loop_rate(100);
    while(ros::ok()){
        target_point.header.stamp = ros::Time::now();
        target_point.header.frame_id = "base_link";
        
        Angle4D angles;

        target_point.point.x += cmd_x;
        target_point.point.y += cmd_y;
        target_point.point.z += cmd_z;
        target_angle += cmd_rot;
        
        float limit = M_PI * 105 / 180;
        float detJ = arm_solver.jacobiDet(angles);
        arm_solver.solve(target_point.point, target_angle, angles);

        if (detJ < j_det_min){
            ROS_INFO("Too close to the singular posture");
            target_point.point.x -= cmd_x;
            target_point.point.y -= cmd_y;
            target_point.point.z -= cmd_z;
            target_angle -= cmd_rot;
        }else if (fabs(angles.angle1) > limit || fabs(angles.angle2) > limit || fabs(angles.angle3) > limit || fabs(angles.angle4) > limit || fabs(angles.angle5) > limit ){
            ROS_INFO("Too close to the limit of the joint angles");
            target_point.point.x -= cmd_x;
            target_point.point.y -= cmd_y;
            target_point.point.z -= cmd_z;
            target_angle -= cmd_rot;
        }

        arm_smooth.setTargetAngles(angles);
        for(int i = 0; i<20; i++){
            arm_mock.setAngle(arm_smooth.output((float)i/10));
            joint_state_pub.publish(arm_mock.getJointState());
            ros::spinOnce();
        }
        
        loop_rate.sleep();
    }
    return 0;
}