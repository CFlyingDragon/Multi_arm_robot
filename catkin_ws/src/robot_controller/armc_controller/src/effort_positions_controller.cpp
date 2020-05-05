/*********************************************************************
 * 作用：基于位置PID的力控,考虑重力补偿
 * 作者：陈永厅
 * 时间：2019/12/21
 * 版权：基于 effort_controllers::JointGroupEffortController改
 *      哈尔滨工业大学（深圳）
 *********************************************************************/

#include "armc_controller/effort_positions_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

namespace armc_controller
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
 //只要填充init、stop、updata、begin
  EffortPositionsController::EffortPositionsController() {}
  EffortPositionsController::~EffortPositionsController() {sub_command_.shutdown();}

  bool EffortPositionsController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))                                                                   //获取关节
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    // Get URDF
    urdf::Model urdf;                                                                                            //ros_control控制URDF必不可少
    if (!urdf.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    pid_controllers_.resize(n_joints_);                                                                           //pid控制个数，与关节控制器相同

    for(unsigned int i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];                                                                    //auto,复制数据，自动分配类型

      try
      {
        joints_.push_back(hw->getHandle(joint_name));                                                               //从robotHW获取关节句柄
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);                                               //获得对应关节的urdf
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);                                                                              //利用vector写入关节urdf

      // Load PID Controller using gains set on parameter server
      if (!pid_controllers_[i].init(ros::NodeHandle(n, joint_name + "/pid")))                                          //初始化PID控制器，使用参数服务器中的参数
      {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_name + "/pid");
        return false;
      }
    }

    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));                                              //命令缓存初始化为零

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &EffortPositionsController::commandCB, this);
    return true;
  }

  //在update中写如控制指令，
  void EffortPositionsController::update(const ros::Time& time, const ros::Duration& period)
  {
    std::vector<double> & commands = *commands_buffer_.readFromRT();                                //从缓存中读取命令
    for(unsigned int i=0; i<n_joints_; i++)                                                         //写入每个关节                
    {
        double command_position = commands[i];                                                      //期望位置，同时进行初始化操作

        double error; //, vel_error;                                                                //误差
        double commanded_effort;                                                                    //命令力

        double current_position = joints_[i].getPosition();                                         //从关节中获取当前时刻的位置，RobotHW中获得

        // Make sure joint is within limits if applicable
        enforceJointLimits(command_position, i);                                                    //判断关节极限

        // Compute position error                                                                   //计算误差
        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)                                         //关节类型
        {
         angles::shortest_angular_distance_with_limits(                                             //求取关节角误差，存入error中
            current_position,
            command_position,
            joint_urdfs_[i]->limits->lower,
            joint_urdfs_[i]->limits->upper,
            error);
        }
        else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
        {
          error = angles::shortest_angular_distance(current_position, command_position);
        }
        else //prismatic
        {
          error = command_position - current_position;
        }

        // Set the PID error and compute the PID command with nonuniform
        // time step size.
        commanded_effort = pid_controllers_[i].computeCommand(error, period);                       //用关节误差计算关节力，采用PID方式计算，可改为阻抗方式计算

        joints_[i].setCommand(commanded_effort);                                                    //将命令写入关节
    }
  }

  void EffortPositionsController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)      //回调函数
  {
    if(msg->data.size()!=n_joints_)                                                                 //是否获得全部数据
    { 
      ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return; 
    }
    commands_buffer_.writeFromNonRT(msg->data);                                                     //写入到实时缓存中
  }

  void EffortPositionsController::enforceJointLimits(double &command, unsigned int index)        //关节限制
  {
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
      if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
      {
        command = joint_urdfs_[index]->limits->upper;
      }
      else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
      {
        command = joint_urdfs_[index]->limits->lower;
      }
    }
  }

  void EffortPositionsController::gravityCompensation(){

  }

} // namespace

PLUGINLIB_EXPORT_CLASS(armc_controller::EffortPositionsController, controller_interface::ControllerBase)
