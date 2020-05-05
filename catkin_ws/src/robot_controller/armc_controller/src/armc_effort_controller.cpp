
#include <armc_controller/joint_group_effort_controller.h>
#include <pluginlib/class_list_macros.hpp>

template <class T>
void forward_command_controller::ForwardJointGroupCommandController<T>::starting(const ros::Time& time)
{
  // Start controller with 0.0 efforts
  commands_buffer_.readFromRT()->assign(n_joints_, 0.0);
}


PLUGINLIB_EXPORT_CLASS(effort_controllers::JointGroupEffortController,controller_interface::ControllerBase)
