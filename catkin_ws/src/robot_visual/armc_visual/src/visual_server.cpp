#include <iostream>

#include "ros/ros.h"
#include <std_msgs/Float32.h>

#include "armc_visual/capture.h"
#include "armc_visual/VisualVar.h"
 
// service回调函数，输入参数req，输出参数res
bool visualInspection(armc_visual::VisualVar::Request  &req,
         armc_visual::VisualVar::Response &res)
{
    int a=req.a;
    float tag_T_cam[4][4]={0.0f};
    float twist_1[6]={0.0f};
    float twist_2[6]={0.0f};
    int flag=0;
    if (a == 0)
    {
        std::cout<<"a0= "<<a<< std::endl;
        flag=lock_capture_pose(tag_T_cam);
        std::cout<<"a1= "<<a<< std::endl;
        for(int i=0;i<12;i++)
        {
            res.T[i] = (double)tag_T_cam[i/4][i%4];
        }
        std::cout << "T=:\n" << std::endl;
        std::cout << "[";
        for(int i=0;i<4;i++)
        {
             std::cout << "[";
            for(int j=0;j<4;j++)
            {
               std::cout << tag_T_cam[i][j] << ", ";
            }
            std::cout << "]\n";
        }
        std::cout << "]"<< std::endl;
    }
    else
    {
        flag=handle_capture_pose(twist_1,twist_2);
        std::cout<<"a2= "<<a<<std::endl;
        for(int i=0;i<6;i++)
        {
           res.T[i] = (double)twist_1[i];
           res.T[6+i] = (double)twist_2[i];
        }
    }

    //输出

    res.flag = flag;
  return true;
}
 
int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "visual_server_node");
  
  // 创建节点句柄
  ros::NodeHandle n;
 
  // 创建一个名为add_two_ints的server，注册回调函数add()
  ros::ServiceServer service = n.advertiseService("visual_inspection", visualInspection);
  
  // 循环等待回调函数
  ROS_INFO("Ready to visual inspection.");
  ros::spin();
 
  return 0;
}
