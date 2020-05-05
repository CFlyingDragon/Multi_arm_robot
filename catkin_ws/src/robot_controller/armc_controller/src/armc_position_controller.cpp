#include <armc_controller/armc_position_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace armc_controller                                                 //命名空间
{
ArmcPositionController::ArmcPositionController() {}
ArmcPositionController::~ArmcPositionController() {sub_command_.shutdown();}
bool ArmcPositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
{
   //获取控制方法
//   std::string param_name1 = "control_methods";
//      if(!n.getParam(param_name1, control_methods))
//        {
//          ROS_ERROR_STREAM("Failed to getParam '" << param_name1 << "' (namespace: " << n.getNamespace() << ").");
//          return false;
//        }
  // List of controlled joints
  std::string param_name2 = "joints";
  if(!n.getParam(param_name2, joint_names_))                                       //获得关节名
  {
    ROS_ERROR_STREAM("Failed to getParam '" << param_name2 << "' (namespace: " << n.getNamespace() << ").");
    return false;
  }
  n_joints_ = joint_names_.size();

  if(n_joints_ == 0){
    ROS_ERROR_STREAM("List of joint names is empty.");

    return false;
  }
  for(unsigned int i=0; i<n_joints_; i++)
  {
    try
    {
      joints_.push_back(hw->getHandle(joint_names_[i]));                          //通过关节名获得句柄
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
  }

  commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));          //将命令初值赋值为0

  sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &ArmcPositionController::commandCB, this);
  return true;
}

void ArmcPositionController::starting(const ros::Time& time)                      //开始控制器
{
  // Start controller with current joint positions
  std::vector<double> & commands = *commands_buffer_.readFromRT();
  for(unsigned int i=0; i<joints_.size(); i++)
  {
    commands[i]=joints_[i].getPosition();                                         //从底层获取关节当前位置为命令的初始值
  }
}

void ArmcPositionController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  std::vector<double> & commands = *commands_buffer_.readFromRT();
  for(unsigned int i=0; i<n_joints_; i++)
  {  joints_[i].setCommand(commands[i]);  }                                       //设置关节命令
}

void ArmcPositionController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)     //命令回调函数
{
  if(msg->data.size()!=n_joints_)
  {
    ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
    return;
  }
  commands_buffer_.writeFromNonRT(msg->data);
}

} //namespace end
PLUGINLIB_EXPORT_CLASS(armc_controller::ArmcPositionController,controller_interface::ControllerBase)
