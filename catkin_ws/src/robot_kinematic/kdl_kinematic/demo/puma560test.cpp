#include <kdl/chain.hpp>
#include "models.hpp"
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

using namespace KDL;
namespace KDL{
    Chain Puma560(){
        Chain puma560;
        puma560.addSegment(Segment());
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.4318,0.0,0.0,0.0),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        puma560.addSegment(Segment());
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0203,-M_PI_2,0.15005,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.4318,0.0),
                                   RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        puma560.addSegment(Segment());
        puma560.addSegment(Segment());
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.0,0.0),
                                   RigidBodyInertia(0.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
        puma560.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0,0.0),
                                   RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        puma560.addSegment(Segment());
        return puma560;
    }
    
}

int main(int argc , char** argv){
    
    Chain p560=Puma560();
    //Chain p560;
//    p560.addSegment(Segment(Joint(Joint::RotX),Frame::Identity(),RigidBodyInertia(1.0,Vector(0.0,1.0,.0),RotationalInertia(1.0,2.0,3.0))));
//    p560.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation::Identity(),Vector(0,2,0)),RigidBodyInertia(1.0,Vector(1.0,0.0,.0),RotationalInertia(1.0,2.0,3,4,5,6))));
//    p560.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::Identity(),Vector(2,0,0)),RigidBodyInertia(1.0,Vector(0.0,0.0,1),RotationalInertia(1.0,2.0,3,4,5,6))));
    
    JntArray q(p560.getNrOfJoints());
    JntArray qdot(p560.getNrOfJoints());
    JntArray qdotdot(p560.getNrOfJoints());
    JntArray tau(p560.getNrOfJoints());
    Wrenches f(p560.getNrOfSegments());

    for(unsigned int i=0;i<p560.getNrOfJoints();i++){
      q(i)=0.0;
      qdot(i)=0.0;
      qdotdot(i)=0.0;
      
      //if(i<2)
      //{
	std::cout << "give q(" << i+1 << ")\n" << std::endl;        //输入关节位置，速度，加速度
	std::cin >> q(i);
	std::cout << "give qdot(" << i+1 << ")\n" << std::endl;
	std::cin >> qdot(i);
	std::cout << "give qdotdot(" << i << ")\n" << std::endl;
	std::cin >> qdotdot(i);
      //}
        
    }
    
    ChainFkSolverPos_recursive fksolver(p560);
    Frame T;
    ChainIdSolver_RNE idsolver(p560,Vector(0.0,0.0,-9.81));
    
    //#include <time.h>
    //time_t before,after;
    //time(&before);
    //unsigned int k=0;
    //for(k=0;k<1e7;k++)
        fksolver.JntToCart(q,T);
    //time(&after);
    //std::cout<<"elapsed time for FK: "<<difftime(after,before)<<" seconds for "<<k<<" iterations"<<std::endl;
    //std::cout<<"time per iteration for FK: "<<difftime(after,before)/k<<" seconds."<<std::endl;
    //time(&before);
    //for(k=0;k<1e7;k++)
        idsolver.CartToJnt(q,qdot,qdotdot,f,tau);
        //time(&after);
        //std::cout<<"elapsed time for ID: "<<difftime(after,before)<<" seconds for "<<k<<" iterations"<<std::endl;
        //std::cout<<"time per iteration for ID: "<<difftime(after,before)/k<<" seconds."<<std::endl;

    std::cout<<T<<std::endl;
    std::cout<<"tau: "<<tau<<std::endl;


}
    
