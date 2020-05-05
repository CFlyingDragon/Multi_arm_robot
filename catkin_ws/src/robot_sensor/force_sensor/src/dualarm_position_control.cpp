//双臂位置控制轨迹
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <sstream>
#include <stdio.h>
#include "force_sensor/global.h"
#include <math.h>
int main(int argc, char **argv) {
    ros::init(argc, argv, "dualarm_position_control");
    ros::NodeHandle nh;
    ros::Publisher pos_pub = nh.advertise<sensor_msgs::JointState>("/joint_command",1000);
    // ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1); //rviz
    ros::Rate loop_rate(50);
    float theta00[7] = {140,240,180,260,180,220,50};
    float theta01[7] = {220,120,180,100,180,140,40};
    float thetat0[7] = {0};
    float thetat1[7] = {0};
    float TT0[4][4] = {{0,1,0,0.5},{1,0,0,-0.3},{0,0,-1,0.04},{0,0,0,1}};
    float TT1[4][4] = {{-1,0,0,-0.5},{0,1,0,-0.3},{0,0,-1,0.04},{0,0,0,1}};
    float Tn[4][4] = {0};
    float Toa_ea[4][4], Tob_eb[4][4];
    float tt = 0;
    int time = 1;
    usleep(2000000);
    sensor_msgs::JointState jointCommand;
    // sensor_msgs::JointState joint_state; //rviz
    for(int i = 0; i < 14; i++)
        jointCommand.position.push_back(0);
    while(ros::ok && tt <= 35){
    // while(ros::ok && tt <= 8){
        PathPlanning(tt,Toa_ea,Tob_eb); //规划末端位姿
        My_ikine_IteraFcn(Toa_ea,theta00,thetat0);  //id1-7关节角度
        My_ikine_IteraFcn(Tob_eb,theta01,thetat1);  //id8-14关节角度
        if(tt == 0){
            ROS_INFO("\n const double theta1 = %.4f;\n const double theta2 = %.4f;\n const double theta3 = %.4f;\n const double theta4 = %.4f;\n const double theta5 = %.4f;\n const double theta6 = %.4f;\n const double theta7 = %.4f;",
            thetat0[0],thetat0[1],thetat0[2],thetat0[3],thetat0[4],thetat0[5],thetat0[6]);
            ROS_INFO("\n const double theta1 = %.4f;\n const double theta2 = %.4f;\n const double theta3 = %.4f;\n const double theta4 = %.4f;\n const double theta5 = %.4f;\n const double theta6 = %.4f;\n const double theta7 = %.4f;",
            thetat1[0],thetat1[1],thetat1[2],thetat1[3],thetat1[4],thetat1[5],thetat1[6]);
        } 
        tt = tt + 0.02;

        //ID 1-7
        jointCommand.position[0] = thetat0[0] + 2;  //ID1
        jointCommand.position[1] = thetat0[1] + 1;  //ID2
        jointCommand.position[2] = thetat0[2];  //ID3
        jointCommand.position[3] = thetat0[3] + 2;  //ID4
        jointCommand.position[4] = thetat0[4];  //ID5
        jointCommand.position[5] = thetat0[5] + 1;  //ID6
        jointCommand.position[6] = thetat0[6];  //ID7
        //ID 8-14
        jointCommand.position[7] = thetat1[0] + 2;  //ID8
        jointCommand.position[8] = thetat1[1];  //ID9
        jointCommand.position[9] = thetat1[2] + 2;  //ID10
        jointCommand.position[10] = thetat1[3] - 2;  //ID11
        jointCommand.position[11] = thetat1[4];  //ID12
        jointCommand.position[12] = thetat1[5];  //ID13
        jointCommand.position[13] = thetat1[6];  //ID14

        if(Judge_joint_outof_angle(thetat0,thetat1) == 1)   //判断逆解是否超过关节极限
            break;
        ROS_INFO("%d",time++);
        pos_pub.publish(jointCommand);

        //rviz
        // {
        //     double degree = 3.1415926/180;
        //     joint_state.header.stamp = ros::Time::now();
        //     joint_state.name.resize(14);
        //     joint_state.position.resize(14);
        //     joint_state.name[0] = "joint1";
        //     joint_state.name[1] = "joint2";
        //     joint_state.name[2] = "joint3";
        //     joint_state.name[3] = "joint4";
        //     joint_state.name[4] = "joint5";
        //     joint_state.name[5] = "joint6";
        //     joint_state.name[6] = "joint7";
        //     joint_state.name[7] = "joint8";
        //     joint_state.name[8] = "joint9";
        //     joint_state.name[9] = "joint10";
        //     joint_state.name[10] = "joint11";
        //     joint_state.name[11] = "joint12";
        //     joint_state.name[12] = "joint13";
        //     joint_state.name[13] = "joint14";
        //     joint_state.position[0] = thetat0[0]*degree;   
        //     joint_state.position[1] = -(thetat0[1]-180)*degree;
        //     joint_state.position[2] = -(thetat0[2]-180)*degree;
        //     joint_state.position[3] = -(thetat0[3]-180)*degree;
        //     joint_state.position[4] = -(thetat0[4]-180)*degree;
        //     joint_state.position[5] = (thetat0[5]-180)*degree;
        //     joint_state.position[6] = thetat0[6]*degree;
        //     joint_state.position[7] = thetat1[0]*degree;   
        //     joint_state.position[8] = -(thetat1[1]-180)*degree;
        //     joint_state.position[9] = -(thetat1[2]-180)*degree;
        //     joint_state.position[10] = -(thetat1[3]-180)*degree;
        //     joint_state.position[11] = -(thetat1[4]-180)*degree;
        //     joint_state.position[12] = (thetat1[5]-180)*degree;
        //     joint_state.position[13] = thetat1[6]*degree;
        //     joint_pub.publish(joint_state);
        // }
        
        loop_rate.sleep();
    }
    return 0;
}
