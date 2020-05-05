#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    //tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;
    int IncFlag = 1;
    // robot state
    double joint1= 0, joint2= 0, joint3= 0, joint4= 0, joint5= 0, joint6= 0, joint7= 0;
    double joint1_inc = 0.005, joint2_inc = 0.005, joint3_inc = 0.005, joint4_inc = 0.005, joint5_inc = 0.005, joint6_inc = 0.005, joint7_inc = 0.005;
    // message declarations
    //geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    //odom_trans.header.frame_id = "odom";
    //odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(7);
        joint_state.position.resize(7);
        joint_state.name[0] ="Joint1";
        joint_state.position[0] = joint1;
        joint_state.name[1] ="Joint2";
        joint_state.position[1] = joint2;
        joint_state.name[2] ="Joint3";
        joint_state.position[2] = joint3;
	joint_state.name[3] ="Joint4";
        joint_state.position[3] = joint4;
	joint_state.name[4] ="Joint5";
        joint_state.position[4] = joint5;
	joint_state.name[5] ="Joint6";
        joint_state.position[5] = joint6;
	joint_state.name[6] ="Joint7";
        joint_state.position[6] = joint7;

        // update transform
        // (moving in a circle with radius)
        //odom_trans.header.stamp = ros::Time::now();
        //odom_trans.transform.translation.x = cos(angle);
        //odom_trans.transform.translation.y = sin(angle);
        //odom_trans.transform.translation.z = 0.0;
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        ROS_INFO("joint_state publish");
        //broadcaster.sendTransform(odom_trans);


	// Create new robot state
        if (joint1 > 1.56)  IncFlag = 0;   
        if (joint1 < -1.56)  IncFlag = 1;
        if (IncFlag)
        {
                joint1 += joint1_inc;
                joint2 += joint2_inc;
                joint3 += joint3_inc;
                joint4 += joint4_inc;
                joint5 += joint5_inc;
                joint6 += joint6_inc;
                joint7 += joint7_inc;
        }
        else
        {
                joint1 -= joint1_inc;
                joint2 -= joint2_inc;
                joint3 -= joint3_inc;
                joint4 -= joint4_inc;
                joint5 -= joint5_inc;
                joint6 -= joint6_inc;
                joint7 -= joint7_inc;
        }
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
