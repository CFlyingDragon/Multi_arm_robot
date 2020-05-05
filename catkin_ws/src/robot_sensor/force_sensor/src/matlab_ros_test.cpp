//发送六维力传感器信息，matlab接收测试
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "matlab_ros_pubtest");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/six_axis_force_1", 1000);

  ros::Rate loop_rate(50);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float64MultiArray Force;
    std::vector<double> testArray = {0,0,0,0,0,0};
    testArray[0] = 0.01*count;testArray[1] = 0.02*count;testArray[2] = 0.03*count;
    testArray[3] = 0.04*count;testArray[4] = 0.05*count;testArray[5] = 0.06*count;
    // Force.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // Force.layout.dim[0].stride = 1;
    // Force.layout.dim[0].size = 6;
    Force.data = testArray;


    ROS_INFO("%d times,F1 is %f",count,Force.data[0]);
    chatter_pub.publish(Force);

    loop_rate.sleep();
    ++count;
  }


  return 0;
}