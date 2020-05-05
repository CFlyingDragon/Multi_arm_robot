/*********************************************************************
 *Desc: Effort(force)-based position controller using basic PID loop
 /*********************************************************************
 * 作用：基于位置PID的力控
 * 作者：陈永厅
 * 时间：2019/12/14
 * 版权：基于effort_controllers::JointPositionController改
 *      哈尔滨工业大学（深圳）
 *********************************************************************/ 

#ifndef JOINT_EFFORT_POSITION_CONTROLLER_H
#define JOINT_EFFORT_POSITION_CONTROLLER_H

/**
   @class effort_controllers::JointPositionController
   @brief Joint Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @param type Must be "effort_controllers::JointPositionController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The joint position to achieve.

   Publishes:

   - @b state (control_msgs::JointControllerState) :
     Current state of the controller, including pid error and gains.

*/

#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>

namespace effort_controllers
{

class JointPositionController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:

  /**
   * \brief Store position and velocity command in struct to allow easier realtime buffer usage
   */
  struct Commands
  {
    double position_; // Last commanded position
    double velocity_; // Last commanded velocity
    bool has_velocity_; // false if no velocity command has been specified
  };

  JointPositionController();
  ~JointPositionController();

  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param robot The specific hardware interface used by this controller.
   *
   * \param n A NodeHandle in the namespace from which the controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */  
  bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *
   * \param command
   */
  void setCommand(double pos_target);

  /*!
   * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
   *        Also supports a target velocity
   *
   * \param pos_target - position setpoint
   * \param vel_target - velocity setpoint
   */
  void setCommand(double pos_target, double vel_target);

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time);
  
  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Get the PID parameters
   */
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  /**
   * \brief Get the PID parameters
   */
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);

  /**
   * \brief Print debug info to console
   */
  void printDebug();

  /**
   * \brief Set the PID parameters
   */
  void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);

  /**
   * \brief Get the name of the joint this controller uses
   */
  std::string getJointName();

  /**
   * \brief Get the current position of the joint
   * \return current position
   */
  double getPosition();

  hardware_interface::JointHandle joint_;
  urdf::JointConstSharedPtr joint_urdf_;
  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer

private:
  int loop_count_;
  control_toolbox::Pid pid_controller_;       /**< Internal PID controller. */

  std::unique_ptr<
    realtime_tools::RealtimePublisher<
      control_msgs::JointControllerState> > controller_state_publisher_ ;

  ros::Subscriber sub_command_;

  /**
   * \brief Callback from /command subscriber for setpoint
   */
  void setCommandCB(const std_msgs::Float64ConstPtr& msg);

  /**
   * \brief Check that the command is within the hard limits of the joint. Checks for joint
   *        type first. Sets command to limit if out of bounds.
   * \param command - the input to test
   */
  void enforceJointLimits(double &command);

};

} // namespace

#endif
