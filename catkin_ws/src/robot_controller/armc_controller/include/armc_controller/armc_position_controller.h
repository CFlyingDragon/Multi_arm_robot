#ifndef ARMC_POSITION_CONTROLLER_H
#define ARMC_POSITION_CONTROLLER_H

#include <hardware_interface/joint_command_interface.h>                              //命令接口
#include <controller_interface/controller.h>                                         //控制器
#include <std_msgs/Float64MultiArray.h>                                              //64位数组
#include <realtime_tools/realtime_buffer.h>                                          //实时缓存
#include "ros_ethercat_driver/hardware_interface/robot_state_interface.hpp"

namespace armc_controller                                                 //命名空间
{

/**
 * \brief Forward command controller for a set of joints.
 *
 * This class forwards the command signal down to a set of joints.
 * Command signal and joint hardware interface are of the same type, e.g. effort commands for an effort-controlled
 * joint.
 *
 * \tparam T Type implementing the JointCommandInterface.
 *
 * \section ROS interface
 *
 * \param type hardware interface type.
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint commands to apply.
 */
class ArmcPositionController: public controller_interface::Controller<hardware_interface::PositionJointInterface>     //更改为自定义接口
{
public:
  ArmcPositionController(); // {}
  ~ArmcPositionController(); // {sub_command_.shutdown();}                                                         //关闭接收节点

  bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
  unsigned int n_joints_;
  std::vector<std::string> control_methods;

private:
  ros::Subscriber sub_command_;
   void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
};
}

#endif
