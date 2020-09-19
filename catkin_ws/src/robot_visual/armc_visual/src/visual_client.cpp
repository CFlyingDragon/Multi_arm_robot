#include <cstdlib>
#include <iostream>
#include "ros/ros.h"
#include "armc_visual/VisualVar.h"
 
int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "Visual_client");
  
  // 从终端命令行获取检测种类
  if (argc != 2)
  {
    ROS_INFO("usage: visual_client_node a（0 or 1)");
    return -1;
  }
 
  // 创建节点句柄
  ros::NodeHandle n;
  
  // 创建一个client，请求visual_inspection service，service消息类型是armc_visual::VisualVar
  ros::ServiceClient client = n.serviceClient<armc_visual::VisualVar>("visual_inspection");
  
  // 创建armc_visual::VisualVar类型的service消息
  armc_visual::VisualVar srv;
  srv.request.a = atoll(argv[1]);
  
  // 发布service请求，等待加法运算的应答结果
  if (client.call(srv))
  {
    std::cout << "检测结果：" << std::endl;
    std::cout << "检测是否成功：" << srv.response.flag << std::endl;
    std::cout << "检测结果：" << srv.response.flag << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service visual_inspection");
    return 1;
  }
 
  return 0;
}
