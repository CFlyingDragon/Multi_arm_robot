/*************************************************************************
	> File Name: serial_odompub_driver.cpp
	> Author: Wangshunyao
	> Mail: wangshunyao@cqu.edu.cn
	> Created Time: 2017年12月07日 星期四
 ************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <tf/transform_broadcaster.h>
#define pi 3.1415926
#define g 9.8
#define r 0.5
#define a 0.5
#define b 0.5
//#define gear 7.5

using namespace std;
using namespace boost::asio;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "serial_odompub");
  ros::NodeHandle n;
  ros::Publisher odompub = n.advertise<nav_msgs::Odometry>("odom_encoder", 1000);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw",50);
  //ros::Publisher wheel_velocitypub = n.advertise<geometry_msgs::Vector3Stamped>("wheel_velocity", 1000);
  ros::Rate loop_rate(40);
  tf::TransformBroadcaster br;
  tf::Transform transform_rb2nav;
  tf::Transform transform_imu2rb;  
  nav_msgs::Odometry odom_encoder;
  sensor_msgs::Imu mpu6050;
  //geometry_msgs::Vector3Stamped wheel_velocity;

  odom_encoder.header.stamp=ros::Time::now();
  mpu6050.header.stamp = ros::Time::now();
  odom_encoder.pose.pose.position.x=0;
  odom_encoder.pose.pose.position.y=0;
  mpu6050.header.frame_id = "imu_frame";
  odom_encoder.header.frame_id="navgation_frame";
	odom_encoder.child_frame_id="robotbase_frame";

//串口1，编码器反馈
  io_service iosev1;
        //节点文件
  serial_port sp1(iosev1, "/dev/ttyUSB0");
        // 设置参数
  sp1.set_option(serial_port::baud_rate(115200));
  sp1.set_option(serial_port::flow_control(serial_port::flow_control::none));
  sp1.set_option(serial_port::parity(serial_port::parity::none));
  sp1.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  sp1.set_option(serial_port::character_size(8));
  // //串口2，MPU6050数据
  // io_service iosev2;
  //       //节点文件
  // serial_port sp2(iosev2, "/dev/ttyUSB1");
  //       // 设置参数
  // sp2.set_option(serial_port::baud_rate(9600));
  // sp2.set_option(serial_port::flow_control(serial_port::flow_control::none));
  // sp2.set_option(serial_port::parity(serial_port::parity::none));
  // sp2.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
  // sp2.set_option(serial_port::character_size(8));
        // 向串口写数据
        //write(sp, buffer("?BS\r", 4));
		//write(sp, buffer("?BS\n", 4));
  //串口数据格式，帧头0xfffe，16位2字节，float32 位数，vx,vy,px,py,一共18字节
        // 向串口读数据
  int w1=1,w2=1,w3=1,w4=1,count=0,flag=3;
  char result_w[19];
  char bufhead_w1[1];
  char data[2]={0x00,0x00};
  float acc[3],gryo[3],rpy[3],vx, vy, px, py, w,w11=0,w22=0,w33=0,w44=0;
  char result_mpu[9];
  char bufhead_mpu[2];
  size_t n1,n2;
  int readflag=1;

  while (ros::ok())
  {
    //串口1数据格式，帧头0xfffe，16位2字节，float32 位数，w1,w2,w3,w4,方向位一位，帧尾0xfefe，一共18字节
    //if (flag==3)
    //{
    while (readflag)
    {
      read(sp1, buffer(bufhead_w1));
      //read(sp1, buffer(bufhead_w2));
      //data[0]=bufhead_w1[0];
      data[1]=bufhead_w1[0];
      if ((data[0]=='\377')&&(data[1]=='\376'))
      {
        readflag=0;
        break;
      }
      data[0]=data[1];
    }
    if (readflag==0)
    {
      n1 = read(sp1,buffer(result_w));
      //cout<<n1<<endl;
      //w1=(result_w[3]<<24)+(result_w[2]<<16)+(result_w[1]<<8)+result_w[0];
      w1=result_w[3]<<24|result_w[2]<<16|result_w[1]<<8|result_w[0];
      w2=result_w[7]<<24|result_w[6]<<16|result_w[5]<<8|result_w[4];
      w3=result_w[11]<<24|result_w[10]<<16|result_w[9]<<8|result_w[8];
      w4=result_w[15]<<24|result_w[14]<<16|result_w[13]<<8|result_w[12];
      if (result_w[16]|0x08==0) w1=w1;else w1=-w1;
      if (result_w[16]|0x04==0) w2=w2;else w2=-w2;
      if (result_w[16]|0x02==0) w3=w3;else w3=-w3;
      if (result_w[16]|0x01==0) w4=w4;else w4=-w4;
      readflag=1;
    }
    //flag=0;
    //}
    //串口2数据格式，帧头0x55，标识0x51，16位short数，d1,d2,d3,d4,四个数8字节，末位校验和，一共11字节
    // read(sp2, buffer(bufhead_mpu));
    // if (bufhead_mpu[0] == 0x55)
    // {
    //     n2=read(sp2,buffer(result_mpu));
    //      count++;
    //      //cout<<n2<<endl;
    //   switch (bufhead_mpu[1])
    //   {
    //     case 0x51:
    //       acc[0]=((result_mpu[1]<<8)+result_mpu[0])/32768.0*16.0*g;
    //       acc[1]=((result_mpu[3]<<8)+result_mpu[2])/32768.0*16.0*g;
    //       //acc[2]=((result_mpu[5]<<8)|(short)result_mpu[4])/32768.0*16.0*g;
    //       acc[2]=((result_mpu[5]<<8)+result_mpu[4])/32768.0*16.0*g;
    //       flag++;
    //       //cout<<acc[2]<<endl<<count<<endl;
    //       break;
    //     case 0x52:
    //       gryo[0]=((result_mpu[1]<<8)+result_mpu[0])/32768.0*2000;
    //       gryo[1]=((result_mpu[3]<<8)+result_mpu[2])/32768.0*2000;
    //       gryo[2]=((result_mpu[5]<<8)+result_mpu[4])/32768.0*2000;
    //       flag++;
    //       break;
    //     case 0x53:
    //       rpy[0]=((result_mpu[1]<<8)+result_mpu[0])/32768.0*180;
    //       rpy[1]=((result_mpu[3]<<8)+result_mpu[2])/32768.0*180;
    //       rpy[2]=((result_mpu[5]<<8)+result_mpu[4])/32768.0*180;
    //       flag++;
    //       break;
    //   }

    // }
    //速度解算,正前方为X方向
    w11=w1*pi/30;
    w22=w2*pi/30;
    w33=w3*pi/30;
    w44=w4*pi/30;
    vx = r*(w11+w22+w33+w44)/4;
    vy = r*(-w11+w22-w33+w44)/4;
    w = r*(-w11+w22+w33-w44)/(a+b)/4;
//弧度转换，imu的Y轴向前，与车身X轴共线
      float roll = rpy[0]*pi/180;
      float pitch = rpy[1]*pi/180;
      float yaw = rpy[2]*pi/180;
//cout<<result_mpu[4]<<result_mpu[5]<<endl;
//cout<<roll<<endl<<pitch<<endl<<yaw<<endl<<acc[2]<<endl;
//cout<<w2<<endl<<rpy[2]<<endl;
//cout<<(int)result_w[3]<<endl;
cout<<w11<<' '<<w22<<' '<<w33<<' '<<w44<<endl;

      tf::Quaternion q;
      q.setRPY(0, 0, yaw+pi/2);
      //q.setRPY(roll, pitch, yaw);
      transform_rb2nav.setRotation(q);//只考虑二维，只有航向角
        //东北天坐标
      float dx=-vx*sin(yaw)+vy*cos(yaw);
      float dy=vx*cos(yaw)+vy*sin(yaw);
      float dt=(ros::Time::now()-odom_encoder.header.stamp).toSec();

      odom_encoder.pose.pose.position.x=odom_encoder.pose.pose.position.x+dx*dt;
  	  odom_encoder.pose.pose.position.y=odom_encoder.pose.pose.position.y+dy*dt;
    	odom_encoder.twist.twist.linear.x=vx;
    	odom_encoder.twist.twist.linear.y=vy;
      odom_encoder.twist.twist.angular.z=w;
      odom_encoder.pose.pose.orientation.x=transform_rb2nav.getRotation().x();
      odom_encoder.pose.pose.orientation.y=transform_rb2nav.getRotation().y();
      odom_encoder.pose.pose.orientation.z=transform_rb2nav.getRotation().z();
      odom_encoder.pose.pose.orientation.w=transform_rb2nav.getRotation().w();
    	odom_encoder.header.stamp=ros::Time::now();

      q.setRPY(roll, pitch, yaw);
      
      transform_imu2rb.setRotation(q);

      mpu6050.linear_acceleration.x = acc[0];
    	mpu6050.linear_acceleration.y = acc[1];
      mpu6050.linear_acceleration.z = acc[2];
    	mpu6050.angular_velocity.x = gryo[0];
    	mpu6050.angular_velocity.y = gryo[1];
    	mpu6050.angular_velocity.z = gryo[2];
      mpu6050.orientation.x = transform_imu2rb.getRotation().x();
      mpu6050.orientation.y = transform_imu2rb.getRotation().y();
      mpu6050.orientation.z = transform_imu2rb.getRotation().z();
      mpu6050.orientation.w = transform_imu2rb.getRotation().w();

      transform_rb2nav.setOrigin( tf::Vector3(odom_encoder.pose.pose.position.x, odom_encoder.pose.pose.position.y, 0.0) );
      transform_imu2rb.setOrigin( tf::Vector3(0, 0, 0.0) );
     
      //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "navgation_frame", "robotbase_frame"));
      if (flag==3)
      {
      br.sendTransform(tf::StampedTransform(transform_rb2nav, ros::Time::now(), "navgation_frame", "robotbase_frame"));        
      br.sendTransform(tf::StampedTransform(transform_imu2rb, ros::Time::now(), "robotbase_frame", "imu_frame"));        
      
      imu_pub.publish(mpu6050);
      odompub.publish(odom_encoder);
      }


    //ros::spinOnce();
    loop_rate.sleep();
    //count++;
  }
  return 0;
}
