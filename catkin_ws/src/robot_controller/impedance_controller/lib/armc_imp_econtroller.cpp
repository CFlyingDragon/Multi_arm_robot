/*********************************************************************
 * 作用：基于位置PID的力控
 * 作者：陈永厅
 * 时间：2019/12/12
 * 版权：基于 effort_controllers::JointGroupEffortController改
 *      哈尔滨工业大学（深圳）
 *********************************************************************/
#include "impedance_controller/armc_imp_econtroller.h"
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

namespace impendance_controller
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
  ImpEffortController::ImpEffortController() {}
  ImpEffortController::~ImpEffortController() {
      sub_command_pos_.shutdown();
      sub_command_vel_.shutdown();
      sub_command_acc_.shutdown();
      sub_command_force_.shutdown();
  }

  bool ImpEffortController::set_imp_controller(ros::NodeHandle &n){
     //get impendance params
    std::vector<double> imp_m;
    if(!n.getParam("imp_param/imp_m", imp_m))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "imp_m" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    std::vector<double> imp_b;
    if(!n.getParam("imp_param/imp_b", imp_b))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "imp_k" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    std::vector<double> imp_k;
    if(!n.getParam("imp_param/imp_k", imp_k))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "imp_k" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
      //get rabot models
    std::string path;
    if(!n.getParam("robot_model/path", path))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "uedf_path" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    std::string base_link;
    if(!n.getParam("robot_model/base_link", base_link))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "base_link" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    std::string top_link;
    if(!n.getParam("robot_model/top_link", top_link))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << "top_link" << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    imp_control_.reset(new impedance_controller::ImpedanceController());
    imp_control_->set_robot_model(path,base_link,top_link);
    imp_control_->set_imp_param(imp_m,imp_b,imp_k);
    return true;
  }

  bool ImpEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
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
    urdf::Model urdf;                                                                                                  //ros_control控制URDF必不可少
    if (!urdf.initParam("robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

   set_imp_controller(n);                                                                                              //建立阻抗控制器

    for(unsigned int i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];                                                                         //auto,复制数据，自动分配类型

      try
      {
        joints_.push_back(hw->getHandle(joint_name));                                                                   //从robotHW获取关节句柄
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);                                                 //获得对应关节的urdf
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);                                                                               //利用vector写入关节urdf
    }

    //commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));                                                //用于存储机械臂的力矩
    commands_pos_buffer_.writeFromNonRT(std::vector<double>(m_dof_, 0.0));                                               //笛卡尔空间位置
    commands_vel_buffer_.writeFromNonRT(std::vector<double>(m_dof_, 0.0));                                               //笛卡尔空间速度
    commands_acc_buffer_.writeFromNonRT(std::vector<double>(m_dof_, 0.0));                                               //笛卡尔空间加速度
    commands_force_buffer_.writeFromNonRT(std::vector<double>(m_dof_, 0.0));                                             //笛卡尔空间力

    sub_command_pos_ = n.subscribe<std_msgs::Float64MultiArray>("command_pos", 1, &ImpEffortController::commandPosCB, this);
    sub_command_vel_ = n.subscribe<std_msgs::Float64MultiArray>("command_vel", 1, &ImpEffortController::commandVelCB, this);
    sub_command_acc_ = n.subscribe<std_msgs::Float64MultiArray>("command_acc", 1, &ImpEffortController::commandAccCB, this);
    sub_command_force_ = n.subscribe<std_msgs::Float64MultiArray>("command_force", 1, &ImpEffortController::commandForceCB, this);

    return true;
 }

  //在update中写如控制指令，
  void ImpEffortController::update(const ros::Time& time, const ros::Duration& period)
  {
    //获取期望位置并设置到阻抗控制器
    std::vector<double> & commands_pos = *commands_pos_buffer_.readFromRT();
    std::vector<double> & commands_vel = *commands_vel_buffer_.readFromRT();
    std::vector<double> & commands_acc = *commands_acc_buffer_.readFromRT();
    std::vector<double> & commands_force = *commands_force_buffer_.readFromRT();
    imp_control_->set_expect_pos(commands_pos,commands_vel,commands_acc); //必须比关节角先设置
    imp_control_->set_expect_force(commands_force);

    //获取关节实时状态并设置到阻抗控制器
    std::vector<double> current_joints_pos;
    for(unsigned int i=0; i<n_joints_; i++){
        double current_position = joints_[i].getPosition();
        current_joints_pos.push_back(current_position);
    }
    imp_control_->set_robot_state(current_joints_pos);

    //求取阻抗控制器计算出的期望关节力矩
    std::vector<double> tau;
    imp_control_->get_joint_torque(tau);

    for(unsigned int i=0; i<n_joints_; i++)
    {
        // Make sure joint is within limits if applicable
       // enforceJointLimits(command_position, i);                                                    //判断关节极限，此处可以调用正动力学预估关节位置

        joints_[i].setCommand(tau[i]);                                                    //将命令写入关节
    }
  }

  void ImpEffortController::commandPosCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if(msg->data.size()!=m_dof_)
    {
      ROS_ERROR_STREAM("Dimension of command_pos (" << msg->data.size() << ") does not match number of cartsion (" << m_dof_ << ")! Not executing!");
      return;
    }
    commands_pos_buffer_.writeFromNonRT(msg->data);
  }

  void ImpEffortController::commandVelCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if(msg->data.size()!=m_dof_)
    {
      ROS_ERROR_STREAM("Dimension of command_vel (" << msg->data.size() << ") does not match number of cartsion (" << m_dof_ << ")! Not executing!");
      return;
    }
    commands_vel_buffer_.writeFromNonRT(msg->data);
  }

  void ImpEffortController::commandAccCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if(msg->data.size()!= m_dof_)
    {
      ROS_ERROR_STREAM("Dimension of command_acc (" << msg->data.size() << ") does not match number ofcartsion (" << m_dof_ << ")! Not executing!");
      return;
    }
    commands_acc_buffer_.writeFromNonRT(msg->data);
  }

  void ImpEffortController::commandForceCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if(msg->data.size()!= m_dof_)
    {
      ROS_ERROR_STREAM("Dimension of command_force (" << msg->data.size() << ") does not match number ofcartsion (" << m_dof_ << ")! Not executing!");
      return;
    }
    commands_acc_buffer_.writeFromNonRT(msg->data);
  }

  void ImpEffortController::enforceJointLimits(double &command, unsigned int index)        //关节限制
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

} // namespace

PLUGINLIB_EXPORT_CLASS(impendance_controller::ImpEffortController,controller_interface::ControllerBase)
