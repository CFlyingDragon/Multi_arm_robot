#include <ros/ros.h>
//#include "servocontrol.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "string.h"
using namespace std;

std_msgs::Float64 joint1,joint2,joint3,joint4,joint5,joint6,joint7;
int resetFlag =0;
void jointCommandCallback(const sensor_msgs::JointStateConstPtr& jointCommandMsg)
{
    joint1.data = jointCommandMsg->position[0];
    joint2.data = jointCommandMsg->position[1];
    joint3.data = jointCommandMsg->position[2];
    joint4.data = jointCommandMsg->position[3];
    joint5.data = jointCommandMsg->position[4];
    joint6.data = jointCommandMsg->position[5];
    joint7.data = jointCommandMsg->position[6];
    ROS_INFO("Recieved a New Joint Command j1 = %f j2 = %f j3 = %f j4 = %f j5 = %f j6 = %f j7 = %f ",
             joint1.data,joint2.data,joint3.data,joint4.data,joint5.data,joint6.data,joint7.data);
}
void resetCommandCallback(const std_msgs::BoolConstPtr& resetCommandMsg)
{
    ROS_INFO("got a  Reset");
    if(resetCommandMsg->data)
        resetFlag =1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_command_interface");
    ros::NodeHandle nh;

    ros::Subscriber jointCommandSub = nh.subscribe("/joint_command", 1, jointCommandCallback);
    ros::Subscriber resetCommandSub = nh.subscribe("/reset_flag",1,resetCommandCallback);
    ros::Publisher joint1ControllerCommandPub =
            nh.advertise<std_msgs::Float64>("joint1_position_controller/command",1);
    ros::Publisher joint2ControllerCommandPub =
            nh.advertise<std_msgs::Float64>("joint2_position_controller/command",1);
    ros::Publisher joint3ControllerCommandPub =
            nh.advertise<std_msgs::Float64>("joint3_position_controller/command",1);
    ros::Publisher joint4ControllerCommandPub =
            nh.advertise<std_msgs::Float64>("joint4_position_controller/command",1);
    ros::Publisher joint5ControllerCommandPub =
            nh.advertise<std_msgs::Float64>("joint5_position_controller/command",1);
    ros::Publisher joint6ControllerCommandPub =
            nh.advertise<std_msgs::Float64>("joint6_position_controller/command",1);
    ros::Publisher joint7ControllerCommandPub =
            nh.advertise<std_msgs::Float64>("joint7_position_controller/command",1);
    ros::Rate rate(50);
    ROS_INFO("get Ready");
    while(ros::ok())
    {
        // if(!resetFlag){
        //     ROS_INFO("Wait For Reset");
        // }
        // else{
            ROS_INFO("send joint command once");
            joint1ControllerCommandPub.publish(joint1);
            joint2ControllerCommandPub.publish(joint2);
            joint3ControllerCommandPub.publish(joint3);
            joint4ControllerCommandPub.publish(joint4);
            joint5ControllerCommandPub.publish(joint5);
            joint6ControllerCommandPub.publish(joint6);
            joint7ControllerCommandPub.publish(joint7);
        // }

        ros::spinOnce();
        rate.sleep();

    }
}
