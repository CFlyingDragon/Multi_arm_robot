#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "iostream"
#include "fstream"
#define postoangle 1440000.0/360 
#define big 2498560.0/360
#define small 1440000.0/360
#define degreetoradius 180.0/3.1415926
#define radiustodegree 3.1415926/180.0

using namespace std;


testcan::IpPos ip_pos;
int sendangle(int id, double angle){
    ip_pos.id = id;
    ip_pos.pos = angle;
    
    return 0;
}

int main(int argc, char *argv[])
{
    printf("hello\n");
    ros::init(argc, argv, "arm_test_node");
    ros::NodeHandle nh_;
	printf("start\n");
    ros::Publisher pos_pub = nh_.advertise<testcan::IpPos>("/canopenexample/ip_pos",8);
    
    usleep(500000);
    ifstream infile("catkin_ws/src/CANopen_v1-master/testcan/data/testqq.txt");
    double j1,j2,j3,j4,j5,j6,j7;
    // infile>>j1>>j2>>j3>>j4>>j5>>j6>>j7;
    // cout<<"j1"<<j1<<"j2"<<j2<<"j3"<<j3<<"j4"<<j4<<"j5"<<j5<<"j6"<<j6<<"j7"<<j7;
    ros::Rate loop_rate(50);
    // i= 0;
    while (!infile.eof()&&ros::ok())  
    {
        infile>>j1>>j2>>j3>>j4>>j5>>j6>>j7;
        cout<<"j1="<<j1<<"j2="<<j2<<"j3="<<j3<<"j4="<<j4<<"j5="<<j5<<"j6="<<j6<<"j7="<<j7<<endl;
        sendangle(1, j1);
        pos_pub.publish(ip_pos);
        sendangle(2, j2);   
        pos_pub.publish(ip_pos);
        sendangle(3, j3);
        pos_pub.publish(ip_pos);
        sendangle(4, j4);
        pos_pub.publish(ip_pos);
        sendangle(5, j5);
        pos_pub.publish(ip_pos);
        sendangle(6, j6);
        pos_pub.publish(ip_pos);   
        sendangle(7, j7);
        pos_pub.publish(ip_pos);

        // double end = ros::Time::now().toNSec();
        // ROS_INFO("end time is : %f",end);
        // ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}