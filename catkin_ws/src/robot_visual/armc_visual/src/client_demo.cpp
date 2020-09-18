#include <cstdlib>
#include "ros/ros.h"
#include "armc_visual/AddTwoInts.h"
 
int main(int argc, char **argv)
{
  // ROS节点初始化
  ros::init(argc, argv, "add_two_ints_client");
  
  // 从终端命令行获取两个加数
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }
 
  // 创建节点句柄
  ros::NodeHandle n;
  
  // 创建一个client，请求add_two_int service，service消息类型是armc_visual::AddTwoInts
  ros::ServiceClient client = n.serviceClient<armc_visual::AddTwoInts>("add_two_ints");
  
  // 创建armc_visual::AddTwoInts类型的service消息
  armc_visual::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  
  // 发布service请求，等待加法运算的应答结果
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
 
  return 0;
}
