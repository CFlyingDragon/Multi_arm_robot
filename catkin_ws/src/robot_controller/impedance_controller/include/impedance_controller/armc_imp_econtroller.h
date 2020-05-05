/*********************************************************************
 * 作用：基于力的阻抗控制
 * 作者：陈永厅
 * 时间：2019/12/12
 * 版权：基于 effort_controllers::ArmEffortPositionController改
 *      哈尔滨工业大学（深圳）
 *********************************************************************/

#ifndef ARMC_IMP_CONTROLLER_H
#define ARMC_IMP_CONTROLLER_H

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

#include "impedance_controller/imp_controller.hpp"

namespace impendance_controller                                                                                              //命名空间区分力控还是位控
{

/**
 * \brief Forward command controller for a set of effort controlled joints (torque or force).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupEffortController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint efforts to apply
 */
class ImpEffortController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  ImpEffortController();
  ~ImpEffortController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);                //
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_pos_buffer_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_vel_buffer_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_acc_buffer_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_force_buffer_;
  unsigned int n_joints_;
  unsigned int m_dof_ = 6;

private:
  std::shared_ptr<impedance_controller::ImpedanceController> imp_control_;

  ros::Subscriber sub_command_pos_;
  ros::Subscriber sub_command_vel_;
  ros::Subscriber sub_command_acc_;
  ros::Subscriber sub_command_force_;

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;                                         //urdf中的关节

  bool set_imp_controller(ros::NodeHandle &n);

  void commandPosCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  void commandVelCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  void commandAccCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  void commandForceCB(const std_msgs::Float64MultiArrayConstPtr& msg);

  void enforceJointLimits(double &command, unsigned int index);                                //力的范围限制                                                                 //增加重力补偿
}; // class

} // namespace

#endif
