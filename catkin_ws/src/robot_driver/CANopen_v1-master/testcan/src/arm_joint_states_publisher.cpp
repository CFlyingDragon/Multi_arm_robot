#include "testcan/Frame.h"
#include "testcan/canopen_vci_ros.h"
#include "ros/ros.h"
#include "testcan/IpPos.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "iostream"
#include "fstream"
#include "string.h"

using namespace std;

class JointState{
public:
    JointState();
    void registerNodeHandle(ros::NodeHandle& _nh);
    void registerPubSub();
    void init();
    void PosFeedCallback(const testcan::Frame::ConstPtr &can_msg);
private:
int init_structure_motor_pos[7] = {                                                            //电机初始化
   27611,//38534,//61995,//37542,
   258387,//259133,//258396
   216098,//216283,//216265,
   126361,//126354,//147464,//127610,
   119294,//31699,//85491,
   251734,//251963,//261785,//175998,
   24472,//10170,//21034,
            };
    int last_motor_pos, last_out_pos, motor_pos, out_pos;
    testcan::IpPos cur_pos_msg, absolute_pos_msg, vel_msg,Ic_msg, ip_pos_msg;
    int pos_dev_id = 1;
    int cv_dev_id = 1;
    int flag[7] = {0,0,0,0,0,0,0};
    double angleToZeroPosition[7];
    double motorPositionToEnd[7];
    unsigned char tpdo2_data[8],tpdo3_data[8];
    sensor_msgs::JointState jointFeedbackMsg;
    ros::Publisher jointFeedbackPub;
    ros::Subscriber ip_pos_sub;
    ros::NodeHandle nh;
};

int main(int argc, char *argv[])
{
    printf("hello\n");
    ros::init(argc, argv, "arm_joint_states_publisher");
    ros::NodeHandle nh;
     
    JointState JointState1;

    JointState1.registerNodeHandle(nh);
    JointState1.registerPubSub();
    JointState1.init();
    
    ros::spin();                                                                                          //一直回调

    return 0;
}

JointState::JointState(){};

void JointState::registerNodeHandle(ros::NodeHandle& _nh){                                                //传入参数到类
    nh = _nh;
};

void JointState::registerPubSub(){
    jointFeedbackPub = nh.advertise<sensor_msgs::JointState>("/joint_states",8);                          //发送关节状态
    ip_pos_sub = nh.subscribe("/canopenexample/can_recieve",40,&JointState::PosFeedCallback,this);        //订阅来之底层的can信息
}

void JointState::init(){                                                                                  //关节位置及关节名存入到反馈变量
    for(int i =0;i<7;i++)
        {
            jointFeedbackMsg.position.push_back(0);
            jointFeedbackMsg.name.push_back("joint"+std::to_string(i+1));
        }   
}

void JointState::PosFeedCallback(const testcan::Frame::ConstPtr &can_msg){                                 //接受到的数据帧
    int Iom;
    Iom = 0;
    int vel;
    int date_len = can_msg->dlc;                                                                           //数据长度
    static int feedback_time = 0;
    // cout<<"recieve CAN ID is : "<<can_msg->id<<endl;
    if(TPDO2<can_msg->id&&can_msg->id<TPDO3){                                                              //id为16进制，需要解码
        for(int i = 0; i < date_len; i++)
        {
            tpdo2_data[i] = can_msg->data[i];                                                              //复制数据
            pos_dev_id = can_msg->id - TPDO2;                                                              //解析设备id
        } 
    }
    //  else if(TPDO3<can_msg->id&&can_msg->id<TPDO4){
    //     for(int i = 0; i < date_len; i++)
	//     {
	// 	    tpdo3_data[i] = can_msg->data[i];
    //         cv_dev_id = can_msg->id - TPDO3;
    //         // cout<<"CV ID is : "<<cv_dev_id<<endl;
	//     } 
    // }
    int motor_pos = (tpdo2_data[3]<<24)|(tpdo2_data[2]<<16)|(tpdo2_data[1]<<8)|tpdo2_data[0];              //共8个字节，前四个为电机位置
    int absolute_pos = (tpdo2_data[7]<<24)|(tpdo2_data[6]<<16)|(tpdo2_data[5]<<8)|tpdo2_data[4];           //后四个为绝对编码器位置
    // int joint_velocity = (tpdo3_data[3]<<24)|(tpdo3_data[2]<<16)|(tpdo3_data[1]<<8)|tpdo3_data[0]; 
    // int joint_current = (tpdo3_data[5]<<8)|tpdo3_data[4];
    if(!flag[pos_dev_id-1])                                                                                //是否有初始位置
    {
        int dp = 0;
        // if(dp<0)
        //     dp = 262144+dp;
        int baisToZero = 262144-init_structure_motor_pos[pos_dev_id-1];                                     //相对于0位的位置
        cout<<"bais "<<pos_dev_id<<" is "<<baisToZero<<endl;
        int correctAbPos = absolute_pos +baisToZero;                                                        //加上偏置
        cout<<"correctAbPos "<<pos_dev_id<<" is "<<correctAbPos<<endl;
        if(correctAbPos > 262144)                                                                           //
            correctAbPos = correctAbPos - 262144;
        if(correctAbPos<=131072)
            dp = correctAbPos;
        if(correctAbPos>131072)                                                                             //改到正负范围
            dp = correctAbPos - 262144;
        cout<<"dp "<<pos_dev_id<<" is "<<dp<<endl;
        angleToZeroPosition[pos_dev_id-1] = 2*3.1415927*(dp)/262144.0;                                      //转换为相对位置角度
        flag[pos_dev_id-1]=1;                                                                               //标志以获得
        ROS_INFO("init Zero position bais of joint %d is %f",pos_dev_id,angleToZeroPosition[pos_dev_id-1]);
    
    }
    switch (pos_dev_id)
        {
            case 1:
            case 2:
            case 3:
            case 4:
                /* code for condition */
                if(motor_pos>=0)
                    motorPositionToEnd[pos_dev_id-1] =2*3.1415927*(motor_pos%2498560)/2498560.0 + angleToZeroPosition[pos_dev_id-1];
                if(motor_pos<0)
                    motorPositionToEnd[pos_dev_id-1] =-2*3.1415927*(abs(motor_pos)%2498560)/2498560.0 + angleToZeroPosition[pos_dev_id-1];
                break;
            case 5:
                // ROS_INFO("motor ID : %d ,pos is %d absolute pos is %d",pos_dev_id, motor_pos, absolute_pos);
                // ROS_INFO("current CV ID ; %d joint_velocity is %d joint_current pos is %d", cv_dev_id, joint_velocity, joint_current);
            case 6:
                // ROS_INFO("motor ID : %d ,pos is %d absolute pos is %d",pos_dev_id, motor_pos, absolute_pos);
            case 7:
                // ROS_INFO("motor ID : %d ,pos is %d absolute pos is %d",pos_dev_id, motor_pos, absolute_pos);
                if(motor_pos>=0)
                    motorPositionToEnd[pos_dev_id-1] = 2*3.1415927*(motor_pos%1440000)/1440000.0 + angleToZeroPosition[pos_dev_id-1];
                if(motor_pos<0)
                    motorPositionToEnd[pos_dev_id-1] = -2*3.1415927*(abs(motor_pos)%1440000)/1440000.0 + angleToZeroPosition[pos_dev_id-1];

                break;
        }
    jointFeedbackMsg.position[pos_dev_id-1] = motorPositionToEnd[pos_dev_id-1];                            //电机位置
    // jointFeedbackMsg.velocity[cv_dev_id-1] = joint_velocity;
    // jointFeedbackMsg.effort[cv_dev_id-1] = joint_current;
    ROS_INFO("current position of joint %d is %f",pos_dev_id,motorPositionToEnd[pos_dev_id-1]);
    feedback_time++;                                                                                       //关节反馈时间
    if(feedback_time == 7) {
        jointFeedbackMsg.header.stamp = ros::Time::now();
        jointFeedbackPub.publish(jointFeedbackMsg);
        feedback_time = 0;
        ROS_INFO("publish once!"); 
    }

};

/**********************************************/
// int init_structure_motor_pos[7] = {
//         27603,//70764,//37542,
//         258408,//259133,//258396,//39888,//191867,
//         216098,//216283,//216265,
//         126354,//147464,//127610,
//         119294,//31699,//85491,
//         252011,//251963,//261785,//175998,
//         -244491,//10170,//21034,
//         };
// int init_structure_motor_pos_oppsite[7]= {
//     158675,//193067,//201836,
//     128061,//127324,
//     90026,//85211,
//     257426,//16392,
//     250366,//162771,
//     120891,//130713,
//     142201,
// };
// // char ip_pos[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
// testcan::IpPos cur_pos_msg, absolute_pos_msg, vel_msg,Ic_msg, ip_pos_msg;
// sensor_msgs::JointState jointFeedbackMsg;
// int last_motor_pos, last_out_pos, motor_pos, out_pos;
// int pos_dev_id = 1;
// int cv_dev_id = 1;
// int flag[7] = {0,0,0,0,0,0,0};
// double angleToZeroPosition[7];
// double motorPositionToEnd[7];
// unsigned char tpdo2_data[8],tpdo3_data[8];

// void PosFeedbackCallback(const testcan::Frame::ConstPtr &can_msg){
//     // unsigned char tpdo2_data[8],tpdo3_data[8];
//     // int pos_dev_id = 1;
//     // int cv_dev_id = 1;
//     int Iom;
//     Iom = 0;
//     int vel;
//     int date_len = can_msg->dlc;
//     // cout<<"recieve CAN ID is : "<<can_msg->id<<endl;
//     if(TPDO2<can_msg->id&&can_msg->id<TPDO3){
//         for(int i = 0; i < date_len; i++)
//         {
//             tpdo2_data[i] = can_msg->data[i];
//             pos_dev_id = can_msg->id - TPDO2;
//             // cout<<"Pos ID is : "<<pos_dev_id<<endl;
//         } 
//     } else if(TPDO3<can_msg->id&&can_msg->id<TPDO4){
//         for(int i = 0; i < date_len; i++)
// 	    {
// 		    tpdo3_data[i] = can_msg->data[i];
//             cv_dev_id = can_msg->id - TPDO3;
//             // cout<<"CV ID is : "<<cv_dev_id<<endl;
// 	    } 
//     }
//     int motor_pos = (tpdo2_data[3]<<24)|(tpdo2_data[2]<<16)|(tpdo2_data[1]<<8)|tpdo2_data[0]; 
//     int absolute_pos = (tpdo2_data[7]<<24)|(tpdo2_data[6]<<16)|(tpdo2_data[5]<<8)|tpdo2_data[4]; 
//     int joint_velocity = (tpdo3_data[3]<<24)|(tpdo3_data[2]<<16)|(tpdo3_data[1]<<8)|tpdo3_data[0]; 
//     int joint_current = (tpdo3_data[5]<<8)|tpdo3_data[4];
//     if(!flag[pos_dev_id-1])
//     {
//         int dp = 0;
//         // if(dp<0)
//         //     dp = 262144+dp;
//         int baisToZero = 262144-init_structure_motor_pos[pos_dev_id-1];
//         cout<<"bais "<<pos_dev_id<<" is "<<baisToZero<<endl;
//         int correctAbPos = absolute_pos +baisToZero;
//         cout<<"correctAbPos "<<pos_dev_id<<" is "<<correctAbPos<<endl;
//         if(correctAbPos > 262144)
//             correctAbPos = correctAbPos - 262144;
//         if(correctAbPos<=131072)
//             dp = correctAbPos;
//         if(correctAbPos>131072)
//             dp = correctAbPos - 262144;
//         cout<<"dp "<<pos_dev_id<<" is "<<dp<<endl;
//         angleToZeroPosition[pos_dev_id-1] = 2*3.1415927*(dp)/262144.0;
//         flag[pos_dev_id-1]=1;
//         ROS_INFO("init Zero position bais of joint %d is %f",pos_dev_id,angleToZeroPosition[pos_dev_id-1]);
    
//     }
//     switch (pos_dev_id)
//         {
//             case 1:
//             case 2:
//             case 3:
//             case 4:
//                 /* code for condition */
//                 if(motor_pos>=0)
//                     motorPositionToEnd[pos_dev_id-1] =2*3.1415927*(motor_pos%2498560)/2498560.0 + angleToZeroPosition[pos_dev_id-1];
//                 if(motor_pos<0)
//                     motorPositionToEnd[pos_dev_id-1] =-2*3.1415927*(abs(motor_pos)%2498560)/2498560.0 + angleToZeroPosition[pos_dev_id-1];
//                 break;
//             case 5:
//                 // ROS_INFO("motor ID : %d ,pos is %d absolute pos is %d",pos_dev_id, motor_pos, absolute_pos);
//                 // ROS_INFO("current CV ID ; %d joint_velocity is %d joint_current pos is %d", cv_dev_id, joint_velocity, joint_current);
//             case 6:
//                 // ROS_INFO("motor ID : %d ,pos is %d absolute pos is %d",pos_dev_id, motor_pos, absolute_pos);
//             case 7:
//                 // ROS_INFO("motor ID : %d ,pos is %d absolute pos is %d",pos_dev_id, motor_pos, absolute_pos);
//                 if(motor_pos>=0)
//                     motorPositionToEnd[pos_dev_id-1] = 2*3.1415927*(motor_pos%1440000)/1440000.0 + angleToZeroPosition[pos_dev_id-1];
//                 if(motor_pos<0)
//                     motorPositionToEnd[pos_dev_id-1] = -2*3.1415927*(abs(motor_pos)%1440000)/1440000.0 + angleToZeroPosition[pos_dev_id-1];

//                 break;
//         }
//     jointFeedbackMsg.position[pos_dev_id-1] = motorPositionToEnd[pos_dev_id-1];
//     // cout<<"position of joint 2 is "<<motorPositionToEnd[1]<<endl;
//     jointFeedbackMsg.velocity[cv_dev_id-1] = joint_velocity;
//     jointFeedbackMsg.effort[cv_dev_id-1] = joint_current;
//     ROS_INFO("current position of joint %d is %f",pos_dev_id,motorPositionToEnd[pos_dev_id-1]);
// }


// int main(int argc, char *argv[])
// {
//     /* code for main function */
//     ros::init(argc, argv, "arm_joint_states_publisher");
//     ros::NodeHandle nh_;
//     boost::recursive_mutex rec_mutex; 
     

//     for(int i =0;i<7;i++)
//     {
//         jointFeedbackMsg.position.push_back(0);
//         jointFeedbackMsg.velocity.push_back(0);
//         jointFeedbackMsg.effort.push_back(0);
//         jointFeedbackMsg.name.push_back("joint"+ std::to_string(i+1));
//     }           
    
//     ros::Subscriber ip_pos_sub = nh_.subscribe("/canopenexample/can_recieve",400,PosFeedbackCallback);
//     ros::Publisher jointFeedbackPub = nh_.advertise<sensor_msgs::JointState>("/joint_states",1);

//     ros::Rate loop_rate(50);
//     while (ros::ok())
//     {
//         /* code for loop body */
//         jointFeedbackMsg.header.stamp = ros::Time::now();
//         boost::recursive_mutex::scoped_lock lock(rec_mutex);  
//         jointFeedbackPub.publish(jointFeedbackMsg);
//         lock.unlock();
//         // cout<<jointFeedbackMsg.position[0]<<"\t"<<jointFeedbackMsg.position[1]<<"\t"<<
//         //     jointFeedbackMsg.position[2]<<"\t"<<jointFeedbackMsg.position[3]<<"\t"<<
//         //     jointFeedbackMsg.position[4]<<"\t"<<jointFeedbackMsg.position[5]<<"\t"<<
//         //     jointFeedbackMsg.position[6]<<endl;
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
    
//     // canrecieve_pub = nh_.advertise<testcan::Frame>("can_recieve", 1);
//     // CANopenVCIROS CANopenVCIROS1;
//     // int dev_id = 0x07;
//     // CANopenVCIROS1.PDO_init(0x07);
//     // char ip_dis[2][8] = {{0x00,0xF9,0x15,0x00,0x00,0x00,0x00,0x00},
//     //                 {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}};
//     // CANopenVCIROS1.send(dev_id,CAN1,RPDO3,ip_dis[0]);
//     // usleep(1000000);
//     // CANopenVCIROS1.send(dev_id,CAN1,RPDO3,ip_dis[1]);
//     // ros::spin();
//     // printf("once");
//     return 0;
// }
