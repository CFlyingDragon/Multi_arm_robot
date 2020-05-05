#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "sensor_msgs/JointState.h"
#include "force_sensor/global.h"
#include "std_msgs/Bool.h"

#define FORCE_SENSITIVE 0.00008
#define MOMENT_SENSITIVE 0.004
#define FORCE_ZEROERR 3
#define MOMENT_ZEROERR 0.05

class Teaching{
public:
    Teaching();
    void registerNodeHandle(ros::NodeHandle& _nh);
    void registerPubSub();
    void init();
    void forceCallback(const std_msgs::Float64MultiArray::ConstPtr& ForceMsg);
    void joint_stateCallback(const sensor_msgs::JointState::ConstPtr& jointFeedbackMsg);
private:
    int count_force = 0;
    int count = 0;
    float ForceBuffer[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float CurrentJntAngleBuffer[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float fPosedeltaX[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float relative_angle[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float fJntAngleDesired[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float force_modify[6];
    std_msgs::Bool ResetFlag;
    sensor_msgs::JointState jointCommand;
    ros::Publisher pos_pub;
    ros::Publisher resetCommandPub;
    ros::Subscriber force_sub;
    ros::Subscriber joint_states_sub;
    ros::NodeHandle nh;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "testforce_subscribe");
    ros::NodeHandle nh;
    Teaching teaching1;
    teaching1.registerNodeHandle(nh);
    teaching1.registerPubSub();
    teaching1.init();
    ros::spin();
}

Teaching::Teaching(){};

void Teaching::registerNodeHandle(ros::NodeHandle& _nh){
    nh = _nh;
};

void Teaching::registerPubSub(){
    pos_pub = nh.advertise<sensor_msgs::JointState>("/joint_command",1000);
    resetCommandPub = nh.advertise<std_msgs::Bool>("/reset_flag",10);
    force_sub = nh.subscribe("six_axis_force_1",12,&Teaching::forceCallback,this);
    joint_states_sub = nh.subscribe("/joint_states_7",20,&Teaching::joint_stateCallback,this);
}

void Teaching::init(){
    ResetFlag.data = true;
    for(int i = 0; i < 7; i++)
        jointCommand.position.push_back(0);
}

void Teaching::forceCallback(const std_msgs::Float64MultiArray::ConstPtr& ForceMsg){
    count_force++;
    ForceBuffer[0] = static_cast<float>(ForceMsg -> data[0]);
    ForceBuffer[1] = static_cast<float>(ForceMsg -> data[1]);
    ForceBuffer[2] = static_cast<float>(ForceMsg -> data[2]);
    ForceBuffer[3] = static_cast<float>(ForceMsg -> data[3]);
    ForceBuffer[4] = static_cast<float>(ForceMsg -> data[4]);
    ForceBuffer[5] = static_cast<float>(ForceMsg -> data[5]);
    if(count_force == 50){
        ROS_INFO("I heard:%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f",
            ForceMsg->data[0],ForceMsg->data[1],ForceMsg->data[2],
            ForceMsg->data[3],ForceMsg->data[4],ForceMsg->data[5]);
        count_force = 0;
    }
}

void Teaching::joint_stateCallback(const sensor_msgs::JointState::ConstPtr& jointFeedbackMsg){
    CurrentJntAngleBuffer[0] = static_cast<float>(jointFeedbackMsg->position[0]);
    CurrentJntAngleBuffer[1] = static_cast<float>(jointFeedbackMsg->position[1]);
    CurrentJntAngleBuffer[2] = static_cast<float>(jointFeedbackMsg->position[2]);
    CurrentJntAngleBuffer[3] = static_cast<float>(jointFeedbackMsg->position[3]);
    CurrentJntAngleBuffer[4] = static_cast<float>(jointFeedbackMsg->position[4]);
    CurrentJntAngleBuffer[5] = static_cast<float>(jointFeedbackMsg->position[5]);
    CurrentJntAngleBuffer[6] = static_cast<float>(jointFeedbackMsg->position[6]);
    resetCommandPub.publish(ResetFlag);
    for (int i = 0; i < 3; i++){ //力修正
        if (ForceBuffer[i] > FORCE_ZEROERR)
            force_modify[i] = ForceBuffer[i] - FORCE_ZEROERR;
        else if (ForceBuffer[i] < -FORCE_ZEROERR)
            force_modify[i] = ForceBuffer[i] + FORCE_ZEROERR;
        else
            force_modify[i] = 0;
    }
    for (int i = 3; i < 6; i++){ //力矩修正
        if (ForceBuffer[i] > MOMENT_ZEROERR)
            force_modify[i] = ForceBuffer[i] - MOMENT_ZEROERR;
        else if (ForceBuffer[i] < -MOMENT_ZEROERR)
            force_modify[i] = ForceBuffer[i] + MOMENT_ZEROERR;
        else
            force_modify[i] = 0;
    }
    for (int i = 0; i < 3; i++)     //位移变化量
        fPosedeltaX[i] = FORCE_SENSITIVE * force_modify[i];
    for (int i = 3; i < 6; i++)
        fPosedeltaX[8-i] = MOMENT_SENSITIVE * force_modify[i];
    relative_angle[0] = CurrentJntAngleBuffer[0]/180.0*PI - D_H[0][0];
    relative_angle[6] = CurrentJntAngleBuffer[6]/180.0*PI - D_H[6][0];
    for (int i = 1; i < 6; i++)
        relative_angle[i] = CurrentJntAngleBuffer[i]/180.0*PI - D_H[i][0];
    if (nfremotecontrol(fPosedeltaX, relative_angle,fJntAngleDesired)){     //返回绝对值，弧度制，角度范围（-pi,pi） 
        for(int i = 0; i < 7; i++)
            jointCommand.position[i] = fJntAngleDesired[i]/PI*180;
        count++;
        if(count == 50){
            ROS_INFO("%f %f %f %f %f %f %f\n",fJntAngleDesired[0]*180.0/PI,fJntAngleDesired[1]*180.0/PI,
            fJntAngleDesired[2]*180.0/PI,fJntAngleDesired[3]*180.0/PI,fJntAngleDesired[4]*180.0/PI,
            fJntAngleDesired[5]*180.0/PI,fJntAngleDesired[6]*180.0/PI);
            count = 0;
        }      
        pos_pub.publish(jointCommand);
    }
    else
        printf("out of range!\n");
}
