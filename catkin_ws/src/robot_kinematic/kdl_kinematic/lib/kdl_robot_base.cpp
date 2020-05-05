/* 基于KDL库搭建
 *作用：建立基于力控的阻抗控制
 *作者：陈永厅
 *版权：哈尔滨工业大学（深圳）
 *时间：2019/12/21
*/

#include "kdl_kinematic/kdl_robot_base.hpp"

namespace KDL {
KdlBase::KdlBase(){
    std::cout << "starting!" << std::endl;
}
KdlBase::~KdlBase(){
    std::cout << "stoping!" << std::endl;
}

bool KdlBase::set_robot_model(std::string path,std::string base_link,std::string top_link){

    //获得机器人树，可以通过多种方法获得，URDF
    KDL::Tree my_tree;
    kdl_parser::treeFromFile(path,my_tree);

    //获得运动链，基座和末端
    bool exit_value;
    //exit_value = my_tree_.getChain("base_link","Link7",chain_);
    exit_value = my_tree.getChain(base_link,top_link,chain_);
    if(!exit_value){
        printf("%s \n","Can not get chain! ");
        return false;
    }
    //初始化变量
    init_param();
    return true;
}

void KdlBase::init_param(){
    //运动链中获得关节数
    n_dof_ = chain_.getNrOfJoints();

    //建立运动学求解器
    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    //建立雅克比求解器
    jacosolver_.reset(new KDL::ChainJntToJacSolver(chain_));
    //建立逆运动学模型
    double eps=1E-8;
    int maxiter=500;
    double eps_joints=1E-15;
    iksolver_.reset(new KDL::ChainIkSolverPos_LMA(chain_,eps,maxiter,eps_joints));
    //建立雅克比矩阵的微分
    jacodotsolver_.reset(new KDL::ChainJntToJacDotSolver(chain_));
    //建立逆动力学模型
    g_ = KDL::Vector(0.0,0.0,-9.8);
    idsolver_.reset(new KDL::ChainIdSolver_RNE(chain_,g_));
    //建立正动力学模型
    //fdsolver_.reset(new KDL::ChainFdSolver_RNE(chain_,g_));

    //建立关节数组变量
    qq_ = KDL::JntArray(n_dof_);
    qv_ = KDL::JntArray(n_dof_);
    qa_ = KDL::JntArray(n_dof_);
    fc_ = KDL::Wrenches(n_dof_);
    tau_ = KDL::JntArray(n_dof_);
}

bool KdlBase::set_base_gravity(const std::vector<double>& g){
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

bool KdlBase::get_fkine(const std::vector<double>& qq,std::vector<double>& x_vec){
    //求解正运动学
    x_vec.reserve(m_dof_);
    int l = qq.size();
    if(!(l==n_dof_)){
        printf("%s \n","Error: get joints qq parameter failed!");
        return false;
    }
    for(int i = 0;i < n_dof_;i++){
        qq_(i) = qq[i];
    }

    KDL::Frame cartpos;
    bool kinematics_status;
    kinematics_status = fksolver_->JntToCart(qq_,cartpos);
    if(kinematics_status<0){
         printf("%s \n","Error:could not calculate forward kinematics : ");
         return false;
    }

    KDL::Vector xe = cartpos.p;
    KDL::Rotation Re = cartpos.M;
    double alpha,beta,gamma;
    Re.GetEulerZYX(alpha,beta,gamma);
    //将求解值传到全局变量
    x_vec.resize(6);
    x_vec[0] = xe(0);
    x_vec[1] = xe(1);
    x_vec[2] = xe(2);
    x_vec[3] = alpha;
    x_vec[4] = beta;
    x_vec[5] = gamma;
    return true;
}

bool KdlBase::get_ikine(const std::vector<double>& qq_init,const std::vector<double>& x_vec,std::vector<double>& qq){
    /*求取逆运动学*/
    int lq = qq_init.size();
    int lx = x_vec.size();
    if(!(lq==n_dof_ && lx==m_dof_)){
        printf("%s \n","Error: get qq_init or X parameter failed!");
        return false;
    }
    qq.resize(n_dof_);
    //转换为旋转矩阵，输入xyz轴
    KDL::Vector vector = KDL::Vector(x_vec[0], x_vec[1], x_vec[2]);

    KDL::Rotation rot;
    rot.EulerZYX(x_vec[3], x_vec[4], x_vec[5]);

    //合成末端坐标，等价于末端齐次矩阵
    KDL::Frame cartpos = KDL::Frame(rot,vector);

    //建立关节变量接收关节逆解
    KDL::JntArray jointpositions = KDL::JntArray(n_dof_);
    KDL::JntArray jointGuesspositions = KDL::JntArray(n_dof_);

    //运动学逆解求取
    bool kinematics_status;
    kinematics_status = iksolver_->CartToJnt(jointGuesspositions,cartpos,jointpositions);
    if(kinematics_status < 0){
         printf("%s \n","Error:could not calculate backword kinematics : ");
         return false;
    }

    //转变输出格式
    for (int i =0;i<n_dof_;i++){
        qq[i] = jointpositions(i);
    }
    return true;
}

bool KdlBase::get_jacob(const std::vector<double>& qq,Eigen::MatrixXd &jac){
    /*获取雅克比*/
    int lq = qq.size();
    if(!(lq==n_dof_ )){
        printf("%s \n","Error: get qq parameter failed!");
        return false;
    }
    KDL::Jacobian jacb;
    for(int i = 0;i< n_dof_;i++ ){
        qq_(i) = qq[i];
    }
    jacosolver_->JntToJac(qq_,jacb);
    jac = jacb.data;
    return true;
}

bool KdlBase::get_jacobXq_dot(const std::vector<double>& qq,const std::vector<double>& qv,std::vector<double>& jqd_vec){
    /*建立雅克比的微分与q微分的乘积*/
    int lq = qq.size();
    int lv = qv.size();
    if(!(lq==n_dof_ && lv==n_dof_)){
        printf("%s \n","Error: get qq or qv parameter failed!");
        return false;
    }
    jqd_vec.resize(m_dof_);
    for(int i = 0;i< n_dof_;i++ ){
        qq_(i) = qq[i];
        qv_(i) = qv[i];
    }
    //求取雅克比的导数与关节的导数的乘积
    KDL::JntArrayVel qqv_in = KDL::JntArrayVel(n_dof_);
    KDL::Twist jacdot_qdot;
    qqv_in.q = qq_;
    qqv_in.qdot = qv_;
    //求取关节速度
    jacodotsolver_->JntToJacDot(qqv_in,jacdot_qdot,n_dof_);
    for(int i=0;i<m_dof_;i++){
        jqd_vec[i] = jacdot_qdot(i);
    }
    return true;
}

bool KdlBase::get_idyna(const std::vector<double>& qq,const std::vector<double>& qv,const std::vector<double>& qa,std::vector<double>& tau){
    /*获取逆动力学，不考虑末端和关节摩擦力*/
    int lq = qq.size();
    int lv = qv.size();
    int la = qa.size();
    if(!(lq==n_dof_ && lv==n_dof_ && la==n_dof_)){
        printf("%s \n","Error: get robot joints position failed!");
        return false;
    }
    tau.resize(n_dof_);

    for(int i = 0;i<n_dof_;i++){
        qq_(i) = qq[i];
        qv_(i) = qv[i];
        qa_(i) = qa[i];
    }

    //求取逆动力学,输入输入关节位置，速度，加速度，末端力，获得关节力矩tau
    idsolver_->CartToJnt(qq_,qv_,qa_,fc_,tau_);
    for(int i = 0;i<n_dof_;i++){
        tau[i] = tau_(i);
    }
}

//bool KdlBase::get_fdyna(const std::vector<double>& qq,const std::vector<double>& qv,const std::vector<double>& f_ext,
//                        const std::vector<double>& tau, std::vector<double>& qa){
//    /*获取逆动力学，不考虑末端和关节摩擦力*/
//    int lq = qq.size();
//    int lv = qv.size();
//    int lt = tau.size();
//    int lf = f_ext.size();
//    if(!(lq==n_dof_ && lv==n_dof_ && lt==n_dof_ && lf == m_dof_)){
//        printf("%s \n","Error: get robot joints position failed!");
//        return false;
//    }
//    qa.resize(n_dof_);

//    for(int i = 0;i<n_dof_;i++){
//        qq_(i) = qq[i];
//        qv_(i) = qv[i];
//        tau_(i) = tau[i];
//    }

//    //求取逆动力学,输入输入关节位置，速度，加速度，末端力，获得关节力矩tau
//    fdsolver_->CartToJnt(qq_,qv_,tau_,fc_,qa_);
//    for(int i = 0;i<n_dof_;i++){
//          qa[i] = qa_(i);
//    }
//}

bool KdlBase::force_to_joint_torque(const std::vector<double>& qq,const std::vector<double>& f,std::vector<double>& tau){
    /*末端力映射到关节力*/
    int lf = f.size();
    int lq = qq.size();

    if(!(lf==m_dof_ && lq==n_dof_)){
        printf("%s \n","Error: get cartsion force or joint position failed!");
        return false;
    }
    tau.resize(m_dof_);
    Eigen::VectorXd f_vec;
    f_vec.resize(m_dof_);

    for(int i = 0;i< m_dof_;i++ ){
        f_vec(i) = f[i];
        qq_(i) = qq[i];
    }

    KDL::Jacobian jacb;
    Eigen::MatrixXd jacb_matr;
    jacosolver_->JntToJac(qq_,jacb);
    jacb_matr = jacb.data;

    Eigen::VectorXd tau_vec;
    tau_vec = jacb_matr.transpose()*f_vec;

    for(int i= 0;i<n_dof_;i++){
        tau[i] = tau_vec(i);
    }
    return true;
}
} //namespace end
