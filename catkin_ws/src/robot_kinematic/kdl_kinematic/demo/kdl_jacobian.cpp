#include <kdl/jacobian.hpp>
#include "kdl_kinematic/chainjnttojacdotsolver.hpp"   //应导入不成功，拷贝了源文件
//#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>

using namespace KDL;
using namespace std;
int main(int argc,char** argv){

    //获得机器人树，可以通过多种方法获得，URDF
    Tree my_tree;
    kdl_parser::treeFromFile("/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf",my_tree);

    //获得运动链，基座和末端
    bool exit_value;
    Chain chain;
    exit_value = my_tree.getChain("base_link","Link7",chain);

    //雅克比大求解器
    ChainJntToJacSolver jacosolver = ChainJntToJacSolver(chain);

    //建立雅克比导数求解器
    ChainJntToJacDotSolver jacodotsolver = ChainJntToJacDotSolver(chain);

    //运动链中获得关节数
    unsigned int nj = chain.getNrOfJoints();

    //建立关节数组变量
    JntArray q_in = JntArray(nj);
    JntArray qv_in = JntArray(nj);

    //输入关节变量
    for(unsigned int i=0;i<nj;i++){

        printf("Enter the position of joint %i: ",i);
        //scanf("%e",&myinput_qq);
        q_in(i)= 1;//(double)myinput_qq;
        printf("Enter the velocity of joint %i: ",i);
        //scanf("%e",&myinput_qv);
        qv_in(i) = 1;//(double)myinput_qv;
    }

    //求取雅克比
    Jacobian jac = Jacobian(nj);
    unsigned int seg_nr = chain.getNrOfSegments();
    printf("Get cartsion dof number of chain is:%i",seg_nr);
    jacosolver.JntToJac(q_in,jac, seg_nr);
    cout << "get jacobian is:\n" << jac.data << endl;
    cout << "The jacobian columns are:" << jac.columns() << endl;


    //求取雅克比导数与关节角加速度的乘积
    Twist jac_dot_q_dot;
    JntArrayVel qqv_in = JntArrayVel(nj);
    qqv_in.q = q_in;
    qqv_in.qdot = qv_in;
    //求取关节速度
    jacodotsolver.JntToJacDot(qqv_in,jac_dot_q_dot,seg_nr);
    cout << "get the jacodot_q_dot: \n" << jac_dot_q_dot << endl;


    //求取雅克比导数
    Jacobian jac_dot = Jacobian(nj);

    //求取关节速度
    jacodotsolver.JntToJacDot(qqv_in,jac_dot,seg_nr);
    cout << "get the jac_dot: \n" << jac_dot.data << endl;

}
