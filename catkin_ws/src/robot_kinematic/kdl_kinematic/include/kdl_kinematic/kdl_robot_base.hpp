/*kdl_base.hpp
 *作用：建立基本的KDL库运用，减少KDL库的使用成本
 * 可通过继承，直接运用于特定机械臂，也可以直接使用
 * 本函数仅涉及KDL的热门功能，KDL的较冷门的功能可以直接调用KDL库
 * 直接使用
 *作者：陈永厅
 *版权：哈尔滨工业大学（深圳）
 *时间：2019/12/21
*/
//导出插件
#include <pluginlib/class_list_macros.h>

//kdl
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include "kdl_kinematic/chainfdsolver_recursive_newton_euler.hpp"

#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "kdl_kinematic/chainjnttojacdotsolver.hpp"

#include "eigen3/Eigen/Eigen"
#include "Eigen/Eigen"
#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/Jacobi"

#include "unordered_map"

#include <fstream>
#include <iostream>
#include <stdio.h>

#include<algorithm>
#include <memory>

namespace KDL {

class KdlBase
{

public:
  KdlBase();
  ~KdlBase();

  //建立机器人模型,输入URDF地址和运动链
  bool set_robot_model(std::string path,std::string base_link,std::string top_link);
  //设置重力
  bool set_base_gravity(const std::vector<double>& g);

  //获取关节状态
  bool set_robot_state(std::vector<double> qq);
  bool set_robot_state(std::vector<double> qq,std::vector<double> qv);
  bool set_robot_state(std::vector<double> qq,std::vector<double> qv,std::vector<double> qa);

  //输入末端六维力，输出关节力矩
  bool force_to_joint_torque(const std::vector<double>& qq,const std::vector<double>& f,std::vector<double>& tau);

  //建立机器人正运动学
  bool get_fkine(const std::vector<double>& qq,std::vector<double>& x_vec);

  bool get_ikine(const std::vector<double>& qq_init,const std::vector<double>& x_vec,std::vector<double>& qq);

  //建立机器人逆动力学
  bool get_idyna(const std::vector<double>& qq,const std::vector<double>& qv,const std::vector<double>& qa,std::vector<double>& tau);

  //建立机器人正动力学
  bool get_fdyna(const std::vector<double>& qq,const std::vector<double>& qv,const std::vector<double>& f_ext, const std::vector<double>& tau, std::vector<double>& qa);


  //求取雅克比矩阵
  bool get_jacob(const std::vector<double>& qq,Eigen::MatrixXd& jac);

  //求取雅克比的微分与关节微分的乘积
  bool get_jacobXq_dot(const std::vector<double>& qq,const std::vector<double>& qv,std::vector<double>& jqd_vec);

protected:
  //kdl相关参数
  KDL::Chain chain_;

  //求解器相关
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jacosolver_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> iksolver_;
  std::shared_ptr<KDL::ChainJntToJacDotSolver> jacodotsolver_;
  std::shared_ptr<KDL::ChainIdSolver_RNE> idsolver_;
  std::shared_ptr<KDL::ChainFdSolver_RNE> fdsolver_;

  //关节空间、笛卡尔空间自由度数
  unsigned int n_dof_;
  unsigned int m_dof_ = 6;

  //关节空间变量
  KDL::JntArray qq_;
  KDL::JntArray qv_;
  KDL::JntArray qa_;
  KDL::JntArray tau_;
  KDL::Wrenches fc_;

  Eigen::VectorXd jacdot_qdot_;

  //环境参数
  KDL::Vector g_;

private:
  //广义逆
  void pinv(const Eigen::MatrixXd& a_in,Eigen::MatrixXd& a_out){
    /*建立一个简单的求伪逆的函数*/
      Eigen::MatrixXd a_T = a_in.transpose();
      Eigen::MatrixXd axa_T = a_in*a_T;
      a_out = a_T*axa_T.inverse();
  }
  //内部雅克比求取
  void jacob();

  //建立高增益评估器，用于估计关节速度，加速度
  void high_gain_evaluator();
  void init_param();

};
}//namespace end
