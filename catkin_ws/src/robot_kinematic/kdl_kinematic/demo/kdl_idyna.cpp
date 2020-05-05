#include <kdl/chain.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

using namespace KDL;
using namespace std;

int main(int argc , char** argv){
    
    //获得运动树
    Tree my_tree;
    kdl_parser::treeFromFile("/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf",my_tree);

    //获取运动链
    bool exit_value;
    Chain armc_chain;
    exit_value = my_tree.getChain("base_link","Link7",armc_chain);
    
    //建立关节位置、速度、加速度变量，关节力，末端六维力
    JntArray q(armc_chain.getNrOfJoints());
    JntArray qdot(armc_chain.getNrOfJoints());
    JntArray qdotdot(armc_chain.getNrOfJoints());
    JntArray tau(armc_chain.getNrOfJoints());
    std::cout << "getNrOfSegment:" <<armc_chain.getNrOfSegments()<< "\n" << std::endl;
    Wrenches f(armc_chain.getNrOfSegments());       //六维力个数等于基本构件数,建立关节六维力默认为0

    //输入关节状态
    for(unsigned int i=0;i<armc_chain.getNrOfJoints();i++){
      q(i)=0.0;
      qdot(i)=0.0;
      qdotdot(i)=0.0;
      
      //if(i<2)
      //{
	std::cout << "give q(" << i+1 << ")\n" << std::endl;        //输入关节位置，速度，加速度
    //std::cin >> q(i);
    q(i) = 1;
	std::cout << "give qdot(" << i+1 << ")\n" << std::endl;
    //std::cin >> qdot(i);
    qdot(i) = 1;
	std::cout << "give qdotdot(" << i << ")\n" << std::endl;
    //std::cin >> qdotdot(i);
    qdotdot(i) = 1;
      //}
        
    }
    
    //建立正向运动学求解器
    ChainFkSolverPos_recursive fksolver(armc_chain);

    //建立坐标系
    Frame T;

    //牛顿欧拉法建立逆动力学求解器
    ChainIdSolver_RNE idsolver(armc_chain,Vector(0.0,0.0,-9.81));
    
    //求取正向运动学
    fksolver.JntToCart(q,T);

    //求取逆动力学
    idsolver.CartToJnt(q,qdot,qdotdot,f,tau);
    std::cout<<"f:" << f[0] <<std::endl;
    std::cout<<T<<std::endl;
    std::cout<<"tau: "<<tau<<std::endl;


}
    
