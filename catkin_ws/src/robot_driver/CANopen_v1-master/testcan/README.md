# CANopen_v1
CANopen ROS interface

#文件功能介绍
(1)canopen_vci_ros.cpp 
接收来自机械臂的can消息，解码后将关节角转发ros话题供上位机使用，接收上位机的关节命令，编码为对应信息下发给机械臂
接口话题：
    ros::NodeHandle nh_("canopenexample");
    canrecieve_pub = nh_.advertise<testcan::Frame>("can_recieve", 16);
    canrecieve2_pub = nh_.advertise<testcan::Frame>("can_recieve2", 16);
    cansend_sub = nh_.subscribe("can_send", 1, &CANopenVCIROS::cansendCallback);
    ip_pos_sub = nh_.subscribe("ip_pos",16,&CANopenVCIROS::PosCmdCallback);
    cmd_vel_sub = nh_.subscribe("cmd_vel",1,&CANopenVCIROS::veltowheelCallback);
(2) joint_states_publisher.cpp和arm_joint_states_publisher.cpp功能相同，后者是在前者基础上进行修改，后者加上命名空间，适合多机器人
话题接口：
jointFeedbackPub = nh.advertise<sensor_msgs::JointState>("/armt/joint_states",8);                          //发送关节状态
ip_pos_sub = nh.subscribe("/canopenexample/can_recieve",40,&JointState::PosFeedCallback,this);        //订阅来之底层的can信息

