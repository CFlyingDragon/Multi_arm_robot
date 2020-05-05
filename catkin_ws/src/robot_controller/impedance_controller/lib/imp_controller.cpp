/*imp_controller.hpp
 * 基于KDL库搭建
 *作用：建立基于力控的阻抗控制
 *作者：陈永厅
 *版权：哈尔滨工业大学（深圳）
 *时间：2019/12/17
*/

#include "impedance_controller/imp_controller.hpp"

namespace impedance_controller {

ImpedanceController::ImpedanceController(){
    std::cout << "Impedanc controller starting!" << std::endl;
}

ImpedanceController::~ImpedanceController(){
    std::cout << "Impedanc controller stoping!" << std::endl;
}

void ImpedanceController::init_param(){
    //运动链中获得关节数
    n_dof_ = chain_.getNrOfJoints();
    m_dof_ = 6;//笛卡尔空间默认为6维

    //建立关节数组变量
    qq_ = KDL::JntArray(n_dof_);
    qv_ = KDL::JntArray(n_dof_);
    qa_ = KDL::JntArray(n_dof_);
    fc_ = KDL::Wrenches(n_dof_);
    tau_ = KDL::JntArray(n_dof_);

    //建立求解器
    //建立运动学求解器
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    //建立雅克比求解器
    jacosolver_.reset(new KDL::ChainJntToJacSolver(chain_));
    //建立雅克比矩阵的微分
    jacodotsolver_.reset(new KDL::ChainJntToJacDotSolver(chain_));
    //建立动力学模型
    g_ = KDL::Vector(0.0,0.0,-9.8);
    idsolver_.reset(new KDL::ChainIdSolver_RNE(chain_,g_));

    //关节力，末端六维力
    fd_.resize(m_dof_);
    fe_.resize(m_dof_);
    fd_last_.setZero(m_dof_);

    //阻抗参数
    imp_md_.resize(m_dof_);
    imp_bd_.resize(m_dof_);
    imp_kd_.resize(m_dof_);

    //末端位置
    xd_.resize(m_dof_);
    xdv_.resize(m_dof_);
    xda_.resize(m_dof_);


    //雅克比微分与关节角微分的积，6维
    jacdot_qdot_.resize(m_dof_);


    //自适应补偿项
    pos_omega_last_.setZero(m_dof_);
    vel_omega_last_.setZero(m_dof_);
}

bool ImpedanceController::set_robot_model(std::string path,std::string base_link,std::string top_link){
    /*建立机器人模型，通过URDF*/
    //获得机器人树，可以通过多种方法获得，URDF
    kdl_parser::treeFromFile(path,my_tree_);
    //获得运动链，基座和末端
    bool exit_value;
    //exit_value = my_tree_.getChain("base_link","Link7",chain_);
    exit_value = my_tree_.getChain(base_link,top_link,chain_);
    if(!exit_value){
        printf("%s \n","Can not get chain! ");
        return false;
    }
    //初始化参数
    init_param();
    return true;
}

void ImpedanceController::fkine(Eigen::VectorXd& x_vec){
    //求解正运动学
    KDL::Frame cartpos;
    bool kinematics_status;
    kinematics_status = fksolver_->JntToCart(qq_,cartpos);
    if(kinematics_status<0){
         printf("%s \n","Error:could not calculate forward kinematics : ");
    }
    KDL::Vector xe = cartpos.p;
    KDL::Rotation Re = cartpos.M;
    double alpha,beta,gamma;
    Re.GetEulerZYX(alpha,beta,gamma);
    //将求解值传到全局变量
    x_vec.resize(6);
    x_vec(0) = xe(0);
    x_vec(1) = xe(1);
    x_vec(2) = xe(2);
    x_vec(3) = alpha;
    x_vec(4) = beta;
    x_vec(5) = gamma;
}

void ImpedanceController::jacobi(){
    /*建立雅克比求取*/
    KDL::Jacobian jac = KDL::Jacobian(n_dof_);
    jacosolver_->JntToJac(qq_,jac);
    jac_ = jac.data;
}

void ImpedanceController::jacobi_dot(){
    /*求取雅克比的导数与关节的导数的乘积*/
    KDL::JntArrayVel qqv_in = KDL::JntArrayVel(n_dof_);
    KDL::Twist jacdot_qdot;
    qqv_in.q = qq_;
    qqv_in.qdot = qv_;
    //求取关节速度
    jacodotsolver_->JntToJacDot(qqv_in,jacdot_qdot,m_dof_);
    for(int i=0;i<m_dof_;i++){
        jacdot_qdot_(i) = jacdot_qdot(i);
    }

}

void ImpedanceController::idyna(Eigen::VectorXd& tau_vec){
    /*求取逆动力学*/
    KDL::JntArray tau = KDL::JntArray(n_dof_);
    //求取逆动力学,输入输入关节位置，速度，加速度，末端力，获得关节力矩tau
    idsolver_->CartToJnt(qq_,qv_,qa_,fc_,tau);
    tau_vec = tau.data;
}

bool ImpedanceController::set_imp_param(const std::vector<double>& md,const std::vector<double>& bd,const std::vector<double>& kd){
    /*默认取六维，期望位置通过外围获得*/
    int lm = md.size();
    int lb = bd.size();
    int lk = kd.size();
    if(!(lm==m_dof_ && lb==m_dof_ && lk==m_dof_)){
        printf("%s \n","Error: get expect impendance parameter failed!");
        return false;
    }
    for (int i = 0;i<m_dof_; i++){
        imp_md_[i] = md[i];
        imp_bd_[i] = bd[i];
        imp_kd_[i] = kd[i];
    }
    return true;
}

bool ImpedanceController::set_robot_state(const std::vector<double>& qq){
    /*获取关节的状态，仅获取关节位置,通过期望值求取关节速度、加速度，同时求取雅克比、雅克比微分*/
    int lq = qq.size();
    if(!(lq==n_dof_)){
        printf("%s \n","Error: get robot joints position failed!");
        return false;
    }
    for (int i = 0;i<n_dof_; i++){
        qq_(i) = qq[i];
    }
    //评估器获取关节角速度和加速度
     jacobi();            //求取雅克比
    Eigen::MatrixXd jac_pinv;
    pinv(jac_,jac_pinv);   //自定义函数求取广义逆

    qv_.data = jac_pinv*xd_;   //Jntarray a;a.data 为Eigen::vectorXt
    //获取加速度
    jacobi_dot();
    qa_.data = jac_pinv*(xda_ - jacdot_qdot_);
    return true;
}

bool ImpedanceController::set_robot_state(const std::vector<double>& qq,const std::vector<double>& qv){
    /*获取关节的状态，获取关节位置,关节角速度*/
    int lq = qq.size();
    int lv = qv.size();
    if(!(lq==n_dof_ && lv==n_dof_)){
        printf("%s \n","Error: get robot joints position failed!");
        return false;
    }
    for (int i = 0;i<n_dof_; i++){
        qq_(i) = qq[i];
        qv_(i) = qv[i];
    }

    //评估器获取关节加速度
     jacobi();            //求取雅克比
    Eigen::MatrixXd jac_pinv;
    pinv(jac_,jac_pinv);   //自定义函数求取广义逆

    //获取加速度
    jacobi_dot();
    qa_.data = jac_pinv*(xda_ - jacdot_qdot_);


    return true;
}

bool ImpedanceController::set_robot_state(const std::vector<double>& qq,const std::vector<double>& qv,const std::vector<double>& qa){
    /*获取关节的状态，获取关节位置,关节速度，关节加速度*/
    int lq = qq.size();
    int lv = qv.size();
    int la = qa.size();
    if(!(lq==n_dof_ && lv==n_dof_ && la==n_dof_)){
        printf("%s \n","Error: get robot joints position failed!");
        return false;
    }
    for (int i = 0;i<n_dof_; i++){
        qq_(i) = qq[i];
        qv_(i) = qv[i];
        qa_(i) = qa[i];
    }
    //求取雅克比
    jacobi();
    jacobi_dot();

    return true;
}

bool ImpedanceController::set_expect_pos(const std::vector<double>& xd,const std::vector<double>& xdv,const std::vector<double>& xda){
    //默认取六维，期望位置通过外围获得
    int l = xd.size();
    int lv = xdv.size();
    int la = xda.size();
    if(!(l==m_dof_ && lv==m_dof_ && la==m_dof_)){
        printf("%s \n","Error: get expect cartesian pos failed!");
        return false;
    }

    //后面可能会用到两种不同格式的数据，所以两种格式度做全局变量
    for (int i = 0;i<m_dof_; i++){
        xd_(i) = xd[i];      //Eigen::VectorXd
        xdv_(i) = xdv[i];
        xda_(i) = xda[i];
    }
    return true;
}

bool ImpedanceController::set_expect_force(const std::vector<double>& fd){
    /*设置期望末端六维力*/
    int lf = fd.size();
    if(!(lf==m_dof_)){
        printf("%s \n","Error: get expect force failed!");
        return false;
    }
    for (int i = 0;i<m_dof_; i++){
        fd_(i) = fd[i];
    }
    return true;
}

void ImpedanceController::imp_function(Eigen::VectorXd& f_err){
    /*阻抗控制表达方程建立，不考虑自适应项*/
    //末端实时位置
    Eigen::VectorXd x_vec;
    Eigen::VectorXd xv_vec;
    Eigen::VectorXd xa_vec;
    Eigen::VectorXd f_imp;
    f_imp.resize(m_dof_);

    fkine(x_vec);
    //求取末端实际速度
    xv_vec = jac_*qv_.data;
    //求取末端实际加速度
    xa_vec = jacdot_qdot_ + jac_*qa_.data;
    //阻抗表达
    for(int i=0;i<m_dof_;i++)
    {
        f_imp(i) = imp_md_[i]*(xda_(i) - xa_vec(i)) + imp_bd_[i]*(xdv_(i) - xv_vec(i)) +imp_kd_[i]*(xd_(i) - x_vec(i));
    }

    f_err = f_imp - fd_ ;
}

bool ImpedanceController::position_compensation(const std::vector<double>& eta,Eigen::VectorXd& omegas){
    /*建立位置误差项的自适应调整项*/
    int l = eta.size();
    if(!(l==m_dof_)){
        printf("%s \n","Error: get param eta failed!");
        return false;
    }
    omegas.resize(l);

    for(int i ;i < m_dof_;i++){
        if(std::abs(imp_kd_[i]) < std::pow(10,-3)){
            omegas(i) =pos_omega_last_(i) ;
        }
        else{
            omegas(i) = pos_omega_last_(i) + eta[i]*(fd_last_(i) - fe_last_(i))/imp_kd_[i];
        }
    }
    return true;
}

bool ImpedanceController::velocity_compensation(const std::vector<double>& eta,Eigen::VectorXd& omegas){
    /*建立位置误差项的自适应调整项*/
    int l = eta.size();
    if(!(l==m_dof_)){
        printf("%s \n","Error: get param eta failed!");
        return false;
    }
    omegas.resize(l);

    for(int i ;i < m_dof_;i++){
        if(std::abs(imp_kd_[i]) < std::pow(10,-3)){
            omegas(i) = vel_omega_last_(i);
        }
        else{
            omegas(i) = vel_omega_last_(i) + eta[i]*(fd_last_(i) - fe_last_(i))/imp_bd_[i];
        }
    }
    return true;
}

void ImpedanceController::get_joint_torque(std::vector<double>& tau){
    /*计算后输出动力学产生的力矩，阻抗计算出的力矩,期望力所产生的力矩*/
    //计算动力学产生的关节力矩
    tau.resize(n_dof_);
    Eigen::VectorXd tau_dyn;
    idyna(tau_dyn);

    //计算力误差产生的力矩
    Eigen::VectorXd f_err;
    Eigen::VectorXd tau_imp;
    imp_function(f_err);
    tau_imp = jac_.transpose()*f_err;

    //输出和力矩
    Eigen::VectorXd tau_vec;
    tau_vec = tau_dyn - tau_imp;
    for (int i = 0;i<n_dof_;i++){
        tau[i] = tau_vec(i);
    }
}

bool ImpedanceController::set_base_gravity(const std::vector<double>& g){
    //默认取六维，期望位置通过外围获得
    int lg = g.size();
    if(!(lg==3)){
        printf("%s \n","Error: get base gravity parameter failed!");
        return false;
    }
    for (int i = 0;i<3; i++){
        g_(i) = g[i];
    }
    return true;
}
} //namespace end



