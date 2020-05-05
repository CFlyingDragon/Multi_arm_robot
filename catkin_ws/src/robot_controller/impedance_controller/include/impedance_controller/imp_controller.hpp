/*imp_controller.hpp
 *作用：建立基于力控的阻抗控制
 *作者：陈永厅
 *版权：哈尔滨工业大学（深圳）
 *时间：2019/12/17
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

#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_kinematic/chainjnttojacdotsolver.hpp>

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
namespace impedance_controller {

class ImpedanceController 
{

public:
    ImpedanceController();
    ~ImpedanceController();
  //建立机器人模型
  bool set_robot_model(std::string path,std::string base_link,std::string top_link);

  //获取关节状态
  bool set_robot_state(const std::vector<double>& qq);
  bool set_robot_state(const std::vector<double>& qq,const std::vector<double>& qv);
  bool set_robot_state(const std::vector<double>& qq,const std::vector<double>& qv,const std::vector<double>& qa);

  //重力参数设置
  bool set_base_gravity(const std::vector<double>& g);

  //获取期望位置
  bool set_expect_pos(const std::vector<double>& xd,const std::vector<double>& xdv,const std::vector<double>& xda);

  //获取期望力
  bool set_expect_force(const std::vector<double>& fd);

  //输出关节力矩
  void get_joint_torque(std::vector<double>& tau);

  //建立机器人正运动学
  void fkine(Eigen::VectorXd& x_vec);

  //建立机器人动力学
  void idyna(Eigen::VectorXd& tau_vec);

  //设置阻抗参数
  bool set_imp_param(const std::vector<double>& md,const std::vector<double>& bd,const std::vector<double>& kd);

  //阻抗方程求解
  void imp_function(Eigen::VectorXd& f_err);

  //建立高增益评估器，用于估计关节速度，加速度
  void high_gain_evaluator();

protected:
  //kdl相关参数
  KDL::Tree my_tree_;
  KDL::Chain chain_;

  //求解器相关
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jacosolver_;
  std::shared_ptr<KDL::ChainJntToJacDotSolver> jacodotsolver_;
  std::shared_ptr<KDL::ChainIdSolver_RNE> idsolver_;

  //雅克比相关变量
  Eigen::MatrixXd jac_;

  //关节空间、笛卡尔空间自由度数
  unsigned int n_dof_;
  unsigned int m_dof_;

  //关节空间变量
  KDL::JntArray qq_;
  KDL::JntArray qv_;
  KDL::JntArray qa_;
  KDL::JntArray tau_;
  KDL::Wrenches fc_;

  //笛卡尔空间变量
  Eigen::VectorXd xd_;
  Eigen::VectorXd xdv_;
  Eigen::VectorXd xda_;
  Eigen::VectorXd fd_;
  Eigen::VectorXd fe_;
  Eigen::VectorXd fd_last_;
  Eigen::VectorXd fe_last_;

  Eigen::VectorXd jacdot_qdot_;

  //环境参数
  KDL::Vector g_;

  //阻抗参数
  std::vector<double> imp_md_;
  std::vector<double> imp_bd_;
  std::vector<double> imp_kd_;

  //自适应补偿项
  Eigen::VectorXd pos_omega_last_;
  Eigen::VectorXd vel_omega_last_;

private:
  void pinv(const Eigen::MatrixXd& a_in,Eigen::MatrixXd& a_out){
    /*建立一个简单的求伪逆的函数*/
      Eigen::MatrixXd a_T = a_in.transpose();
      Eigen::MatrixXd axa_T = a_in*a_T;
      a_out = a_T*axa_T.inverse();
  }

  //求取雅克比矩阵
  void jacobi();

  //求取雅克比的微分与关节微分的乘积
  void jacobi_dot();

  //基于位置补偿项
  bool position_compensation(const std::vector<double>& eta,Eigen::VectorXd& omegas);

  //基于速度补偿项
  bool velocity_compensation(const std::vector<double>& eta,Eigen::VectorXd& omegas);

  void init_param();

};

}  //namespace end

