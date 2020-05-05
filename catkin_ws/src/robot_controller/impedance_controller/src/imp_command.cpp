/*********************************************************************
 * 作用：对基于力控的阻抗控制器中转节点
 * 作者：陈永厅
 * 时间：2019/12/23
 * 版权：哈尔滨工业大学（深圳）
 *********************************************************************/
#include "impedance_controller/imp_controller.hpp"
#include <iostream>

#include <ros/ros.h>
//#include "servocontrol.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/Bool.h"
#include "string.h"

//定义关节状态为全局变量
std_msgs::Float64MultiArray joints;
//关节状态订阅
void jointStateCallback(const sensor_msgs::JointStateConstPtr& jointCommandMsg)
{
    joints.data[0] = jointCommandMsg->position[0];
    joints.data[1] = jointCommandMsg->position[1];
    joints.data[2] = jointCommandMsg->position[2];
    joints.data[3] = jointCommandMsg->position[3];
    joints.data[4] = jointCommandMsg->position[4];
    joints.data[5] = jointCommandMsg->position[5];
    joints.data[6] = jointCommandMsg->position[6];
    ROS_INFO("Recieved a New Joint Command j1 = %f j2 = %f j3 = %f j4 = %f j5 = %f j6 = %f j7 = %f ",
             joints.data[0],joints.data[1],joints.data[2],joints.data[3],joints.data[4],joints.data[5],joints.data[7]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_command_interface");
    ros::NodeHandle nh;

    ros::Publisher CommandPub =nh.advertise<std_msgs::Float64MultiArray>("/armc/effort_joint_controller/command",1);
    ros::Subscriber jointStateSub = nh.subscribe("/armc/joint_states", 1, jointStateCallback);

    //阻抗模型建立
    std::string path = "/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf";
    std::string base_link = "base_link";
    std::string top_link = "Link7";

    impedance_controller::ImpedanceController armc_impc;

    //设置机器人模型
    if(!armc_impc.set_robot_model(path,base_link,top_link))
    {
        std::cout << "Creating robot model faled!" << std::endl;
    }

     std::cout << "Creating robot model succeed!" << std::endl;

    //设置基座标系的重力
    std::vector<double> g{0,0,9.8};
    if(!armc_impc.set_base_gravity(g))
    {
        std::cout << "Set base gravity faled!" << std::endl;
    }
    std::cout << "Set base gravity succeed!" << std::endl;

    //设置阻抗参数
    std::vector<double> md{0.5,0.2,0.3,0.4,0.6,0.5};
    std::vector<double> bd{10,10,10,10,10,10};
    std::vector<double> kd{1000,1000,1000,1000,1000,1000};
    if(armc_impc.set_imp_param(md,bd,kd)){
         std::cout << "Set imp_param succeed!" << std::endl;
    }

    //设置末端期望位置
    std::vector<double> xd{0.5,0.2,0.3,0.4,0.6,0.5};
    std::vector<double> xvd{0,0,0,0,0,0};
    std::vector<double> xad{0,0,0,0,0,0};
    if(armc_impc.set_expect_pos(xd,xvd,xad)){
         std::cout << "Set expect_pos succeed!" << std::endl;
    }

    //设置机器人的状态
    std::vector<double> qq{0.5,0.2,0.3,0.4,0.6,0.5,1};
    if(armc_impc.set_robot_state(qq))
    {
         std::cout << "Set qq succeed!" << std::endl;
    }

    //设置机器人的状态
    std::vector<double> fd{0.5,0.2,0.3,0.4,0.6,0.5};
    if(armc_impc.set_expect_force(fd))
    {
         std::cout << "Set expect_force succeed!" << std::endl;
    }

    //求取关节力矩
    std::vector<double> tau;
    armc_impc.get_joint_torque(tau);
    for(int i = 0;i<tau.size();i++){
        std::cout << "tau[" << i <<"]: " << tau[i] <<std::endl;
    }

    //建立循环
    ros::Rate rate(100);
    ROS_INFO("imp_controller run!");
    while(ros::ok())
    {
        ROS_INFO("send joint command once");
        std_msgs::Float64MultiArray joint_tau;
        joint_tau.data = tau;
        CommandPub.publish(joint_tau);

        //ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
