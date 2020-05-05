/*********************************************************************
 * 作用：基于位置PID的力控
 * 作者：陈永厅
 * 时间：2019/12/12
 * 版权：基于 effort_controllers::ArmEffortPositionController改
 *      哈尔滨工业大学（深圳）
 *********************************************************************/

#ifndef ARMC_EFFORT_POSITION_CONTROLLER_H
#define ARMC_EFFORT_POSITION_CONTROLLER_H

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

namespace armc_controller                                                                                              //命名空间区分力控还是位控
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
class ArmcEffortPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{  
public:
  ArmcEffortPositionController();
  ~ArmcEffortPositionController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);                //
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
  unsigned int n_joints_;

private:
  ros::Subscriber sub_command_;

  std::vector<control_toolbox::Pid> pid_controllers_;       /**< Internal PID controllers. */  //内环PID控制器

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;                                         //urdf中的关节

  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  void enforceJointLimits(double &command, unsigned int index);                                //力的范围限制
  void gravityCompensation();                                                                  //增加重力补偿
}; // class

} // namespace

#endif
