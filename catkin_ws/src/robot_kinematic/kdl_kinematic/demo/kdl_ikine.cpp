#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <stdio.h>
#include <iostream>

using namespace KDL;
using namespace std;
int main(int argc,char** argv){

    //获得运动树
    Tree my_tree;
    kdl_parser::treeFromFile("/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf",my_tree);

    bool exit_value;
    Chain chain;
    exit_value = my_tree.getChain("base_link","Link7",chain);

    //求解误差
    double eps=1E-5;
    //最大迭代次数
    int maxiter=500;
    //关节误差
    double eps_joints=1E-15;

    //建立逆运动学求解器
    ChainIkSolverPos_LMA iksolver = ChainIkSolverPos_LMA(chain,eps,maxiter,eps_joints);
    ChainIkSolverVel_pinv vel_iksolver = ChainIkSolverVel_pinv(chain);

    //建立速度级逆运动学求解


    //建立迭代初始值
    unsigned int nj = chain.getNrOfJoints();
    JntArray jointGuesspositions = JntArray(nj);

    for(unsigned int i=0;i<nj;i++){
//        float myinput;
//        printf("Enter the initial guess position of joint %i: ",i);
//        scanf("%e",&myinput);
        jointGuesspositions(i)= 0;//(double)myinput;
}

    //输入末端位置
    double x,y,z;
    printf("Enter the x: ");
    scanf("%lf",&x);
    printf("Enter the y: ");
    scanf("%lf",&y);
    printf("Enter the z: ");
    scanf("%lf",&z);
    Vector vector = Vector(x,y,z);

//    //输入末端姿态角，采用RPY角，对应欧拉ZYX角
//    float roll,pitch,yaw;
//    printf("Enter the roll: ");
//    scanf("%e",&roll);
//    printf("Enter the pitch: ");
//    scanf("%e",&pitch);
//    printf("Enter the yaw: ");
//    scanf("%e",&yaw);
//    float cy = cos(yaw);
//    float sy = sin(yaw);
//    float cp = cos(pitch);
//    float sp = sin(pitch);
//    float cr = cos(roll);
//    float sr = sin(roll);
//    double rot0 = cy*cp;
//    double rot1 = cy*sp*sr - sy*cr;
//    double rot2 = cy*sp*cr + sy*sr;
//    double rot3 = sy*cp;
//    double rot4 = sy*sp*sr + cy*cr;
//    double rot5 = sy*sp*cr - cy*sr;
//    double rot6 = -sp;
//    double rot7 = cp*sr;
//    double rot8 = cp*cr;

    //转换为旋转矩阵，输入xyz轴
    Rotation rot;
    rot.EulerZYX(1,1,1); //(rot0,rot1,rot2,rot3,rot4,rot5,rot6,rot7,rot8);

    //合成末端坐标，等价于末端齐次矩阵
    Frame cartpos = Frame(rot,vector);

    //建立关节变量接收关节逆解
    JntArray jointpositions = JntArray(nj);

    //运动学逆解求取
    bool kinematics_status;
    kinematics_status = iksolver.CartToJnt(jointGuesspositions,cartpos,jointpositions);
    if(kinematics_status>=0){
        for(int i=0;i<nj;i++){
            std::cout << jointpositions(i) << std::endl;
        }
        printf("%s \n","Success, thanks KDL!");
    }
    else{
        printf("%s \n","Error:could not calculate backword kinematics : ");
    }

    //速度级逆运动学求解
    JntArray qv_out;
    Twist twist_e;
    twist_e.vel = Vector(2,1,2);
    twist_e.rot = Vector(1,2,3);
    cout << "坐标速度" << twist_e.vel <<endl;

    vel_iksolver.CartToJnt(jointpositions,twist_e,qv_out);
    cout << "关节速度" << qv_out.data<< endl;
}
