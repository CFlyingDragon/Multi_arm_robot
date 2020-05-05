//测试用ROS的串口能达到的发送频率
//速度和用asio差不多，无法达到100HZ，但是单片机首发可以达到？？
//19.1.3

#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
serial::Serial ros_ser;
unsigned char buf[19]={0};

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"my_serial_node");
    ros::NodeHandle n;
    ros::Publisher force_pub = n.advertise<std_msgs::Float64MultiArray>("force_1",1000);
    unsigned char open[8]={0x41,0x54,0x2B,0x47,0x4F,0x44,0x0D,0x0A};
    static int p;
    try{
         //设置串口属性，并打开串口
         ros_ser.setPort("/dev/ttyUSB0");
         ros_ser.setBaudrate(115200);
         serial::Timeout to = serial::Timeout::simpleTimeout(1000);
         ros_ser.setTimeout(to);
         ros_ser.open();
    }
     catch (serial::IOException& e){
         ROS_ERROR_STREAM("Unable to open port ");
         return -1;
    }
 
     //检测串口是否已经打开，并给出提示信息 
     if(ros_ser.isOpen()){
         ROS_INFO_STREAM("Serial Port opened");
    }else{
         return -1;
    }
    ros::Rate loop_rate(100);
    while (ros::ok()){
        ros_ser.write(open,8);
        ROS_INFO("serial sends:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",
                open[0],open[1],open[2],open[3],open[4],open[5],open[6],open[7]);
        std_msgs::UInt8MultiArray serial_data;
        // p = ros_ser.available();
        ros_ser.read(serial_data.data,19);
        for(int i = 0; i < 19; i++)
            buf[i] = serial_data.data[i];
        ROS_INFO("I heard:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\
        0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x ",buf[0],buf[1],buf[2],buf[3],
        buf[4],buf[5],buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],
        buf[14],buf[15],buf[16],buf[17],buf[18]);
        loop_rate.sleep();
    }
    // ros::Rate loop_rate(100);
    // while (ros::ok()){
    //     ROS_INFO("hello");
    //     loop_rate.sleep();
    // }
    return 0;
}

