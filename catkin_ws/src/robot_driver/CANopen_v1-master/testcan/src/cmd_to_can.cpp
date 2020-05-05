#include "testcan/Frame.h"
#include "testcan/canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/IpPos.h"
// #include "boost.h"

// int argc_ = 0;
// char *argv_[1] = {0};
// ros::init(argc_, argv_, "cmd_to_can");

CANopenVCIROS CANopenVCIROS1;
char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
void PosCmdCallback(const testcan::IpPos::ConstPtr &cmd){
    int data = cmd->pos;
    int dev_id = cmd->id;
    ip_pos[0] = (data<<24)>>24;
    ip_pos[1] = (data<<16)>>24;
    ip_pos[2] = (data<<8)>>24;
    ip_pos[3] = data>>24;
    
    CANopenVCIROS1.send(0x07,CAN1,RPDO3,ip_pos,1);
    printf("send once");
}

int main(int argc, char *argv[])
{
    /* code for main function */
    ros::init(argc, argv, "cmd_to_can");
    ros::NodeHandle nh_;
    CANopenVCIROS1.PDO_init(0x07);
    // CANopenVCIROS CANopenVCIROS1;
    // int m_run0=1;
	// pthread_t threadid;
	// int ret;
	// ret=pthread_create(&threadid,NULL,&CANopenVCIROS1.receive_func,&m_run0);
    ros::Subscriber ip_pos_sub = nh_.subscribe("ip_pos",1,PosCmdCallback);
    // CANopenVCIROS CANopenVCIROS1;
    // int dev_id = 0x07;
    // CANopenVCIROS1.PDO_init(0x07);
    // char ip_dis[2][8] = {{0x00,0xF9,0x15,0x00,0x00,0x00,0x00,0x00},
    //                 {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};
    // CANopenVCIROS1.send(dev_id,CAN1,RPDO3,ip_dis[0]);
    // usleep(1000000);
    // CANopenVCIROS1.send(dev_id,CAN1,RPDO3,ip_dis[1]);
    ros::spin();
    printf("once");
    return 0;
}