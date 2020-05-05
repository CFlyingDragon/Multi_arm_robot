#include <kdl/kdl.hpp>
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

    //建立正向运动学求解器
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    //运动链中获得关节数
    unsigned int nj = chain.getNrOfJoints();

    //建立关节数组变量
    JntArray jointpositions = JntArray(nj);

    //输入关节变量
    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf("Enter the position of joint %i: ",i);
        scanf("%e",&myinput);
        jointpositions(i)=(double)myinput;
}

    //建立坐标系，可以通过M\P获得旋转矩阵和坐标原点
    Frame cartpos;

    //求解正运动学
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos << std::endl;
        printf("%s \n","Success, thanks KDL!");
    }
    else{
        printf("%s \n","Error:could not calculate forward kinematics : ");
    }
}
