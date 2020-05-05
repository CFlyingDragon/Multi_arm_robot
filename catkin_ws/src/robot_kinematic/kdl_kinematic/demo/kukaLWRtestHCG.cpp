#include <kdl/chain.hpp>
#include "models.hpp"
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp> //know  how to print different types on screen

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>   //牛顿欧拉法求解动力学
#include <kdl/jntspaceinertiamatrix.hpp>                  //关节空间惯性张量
#include <kdl/chaindynparam.hpp>                          //动力学参数

using namespace KDL;
using namespace std;

void outputLine( double, double, double, double, double, double, double);
int getInputs(JntArray&, JntArray&, JntArray&, int&);

int main(int argc , char** argv){
    
    Chain kLWR=KukaLWR_DHnew();
  
    JntArray q(kLWR.getNrOfJoints());                                         //建立变量，kLWR.getNrOfJoints()返回关节个数
    JntArray qdot(kLWR.getNrOfJoints());
    JntArray qdotdot(kLWR.getNrOfJoints());
    JntArray tau(kLWR.getNrOfJoints());
    JntArray tauHCGa(kLWR.getNrOfJoints());
    JntArray tauHCG(kLWR.getNrOfJoints());
    JntArray C(kLWR.getNrOfJoints()); //coriolis matrix
    JntArray G(kLWR.getNrOfJoints()); //gravity matrix
    Wrenches f(kLWR.getNrOfSegments());
    Vector grav(0.0,0.0,-9.81);
    JntSpaceInertiaMatrix H(kLWR.getNrOfJoints()); //inertiamatrix H=square matrix of size= number of joints
    ChainDynParam chaindynparams(kLWR,grav);                                   //动力学参数
    
    int linenum; //number of experiment= number of line 
    //read out inputs from files
    getInputs(q, qdot,qdotdot,linenum);                                        //从文件中读取输入
    
    //calculation of torques with kukaLWRDH_new.cpp (dynamic model)            //已建立好动力学模型
    ChainFkSolverPos_recursive fksolver(kLWR);                                 //正向运动学求解器
    Frame T;
    ChainIdSolver_RNE idsolver(kLWR,grav);                                     //逆向动力学

    fksolver.JntToCart(q,T);                                                   //输入关节，获得坐标
    idsolver.CartToJnt(q,qdot,qdotdot,f,tau);                                  //输入关节位置、速度、加速度、末端力、得到关节力

    std::cout<<"pose (with dynamic model): \n"<<T<<std::endl;
    std::cout<<"tau (with dynamic model): \n"<<tau<<std::endl;
    
    //calculation of the HCG matrices
    chaindynparams.JntToMass(q,H);                                              //从动力学模型中获取相关参数
    chaindynparams.JntToCoriolis(q,qdot,C);
    chaindynparams.JntToGravity(q,G);
    
    //calculation of the torques with the HCG matrices                          //计算关节力矩，拉格朗日形式，HCG模式，最后一个参数为接收参数
    Multiply(H, qdotdot, tauHCG); //H*qdotdot
    Add(tauHCG,C,tauHCGa); //tauHCGa=H*qdotdot+C
    Add(tauHCGa,G,tauHCG); //tauHCG=H*qdotdot+C+G
        
    std::cout<<"H= \n"<<H<<"\n C = \n "<<C<<"\n G= \n"<<G<<" \n tau (with HCG)= \n"<< tauHCG  <<std::endl;
    
    //write file: code based on example 14.4, c++ how to program, Deitel and Deitel, book p 708
    ofstream outPoseFile("poseResultaat.dat",ios::app);
    if(!outPoseFile)
    {
    cerr << "File poseResultaat could not be opened" <<endl;
    exit(1);
    }
    outPoseFile << "linenumber=experimentnr= "<< linenum << "\n";
    outPoseFile << T << "\n \n";
    outPoseFile.close();

    ofstream outTauFile("tauResultaat.dat",ios::app);
    if(!outTauFile)
    {
    cerr << "File tauResultaat could not be opened" <<endl;
    exit(1);
    }
    outTauFile << setiosflags( ios::left) << setw(10)  << linenum;
    outTauFile << tau << "\n";
    outTauFile.close();
}
    


int getInputs(JntArray &_q, JntArray &_qdot, JntArray &_qdotdot, int &linenr)
{
  //cout << " q" << _q<< "\n";
  
  //declaration
  //int linenr; //line =experiment number
  int counter;
  
  //initialisation
  counter=0;
  
  //ask which experiment number= line number in files
  cout << "Give experiment number= line number in files \n ?";
  cin >> linenr;
    
  //read files: code based on example 14.8, c++ how to program, Deitel and Deitel, book p 712
  
  /*
   *READING Q = joint positions
   */
  
  ifstream inQfile("interpreteerbaar/q", ios::in);

  if (!inQfile)
  {
    cerr << "File q could not be opened \n";
    exit(1);
  }
  
  //print headers
  cout << setiosflags( ios::left) << setw(15) << "_q(0)" << setw(15) << "_q(1)" << setw(15) << "_q(2)" << setw(15) << "_q(3)" << setw(15) << "_q(4)" << setw(15) << "_q(5)" << setw(15) << "_q(6)"   << " \n" ;
  
  while(!inQfile.eof())
  {
    //read out a line of the file
    inQfile >> _q(0) >> _q(1) >> _q(2) >> _q(3) >> _q(4) >> _q(5) >> _q(6); 
    counter++;
    if(counter==linenr)
    {
      outputLine( _q(0), _q(1), _q(2), _q(3), _q(4), _q(5), _q(6));
      break;
    }
    
  }
  inQfile.close();
  
  /*
   *READING Qdot = joint velocities
   */
  counter=0;//reset counter
  ifstream inQdotfile("interpreteerbaar/qdot", ios::in);

  if (!inQdotfile)
  {
    cerr << "File qdot could not be opened \n";
    exit(1);
  }
  
  //print headers
  cout << setiosflags( ios::left) << setw(15) << "_qdot(0)" << setw(15) << "_qdot(1)" << setw(15) << "_qdot(2)" << setw(15) << "_qdot(3)" << setw(15) << "_qdot(4)" << setw(15) << "_qdot(5)" << setw(15) << "_qdot(6)"   << " \n" ;
  
  while(!inQdotfile.eof())
  {
    //read out a line of the file
    inQdotfile >> _qdot(0) >> _qdot(1) >> _qdot(2) >> _qdot(3) >> _qdot(4) >> _qdot(5) >> _qdot(6) ; 
    counter++;
    if(counter==linenr)
    {
      outputLine( _qdot(0), _qdot(1), _qdot(2), _qdot(3), _qdot(4), _qdot(5), _qdot(6));
      break;
    }
    
  }
  inQdotfile.close(); 
  
  /*
   *READING Qdotdot = joint accelerations
   */
  counter=0;//reset counter
  ifstream inQdotdotfile("interpreteerbaar/qddot", ios::in);

  if (!inQdotdotfile)
  {
    cerr << "File qdotdot could not be opened \n";
    exit(1);
  }
  
  //print headers
  cout << setiosflags( ios::left) << setw(15) << "_qdotdot(0)" << setw(15) << "_qdotdot(1)" << setw(15) << "_qdotdot(2)" << setw(15) << "_qdotdot(3)" << setw(15) << "_qdotdot(4)" << setw(15) << "_qdotdot(5)" << setw(15) << "_qdotdot(6)"  << " \n" ;
  
  while(!inQdotdotfile.eof())
  {
    //read out a line of the file
    inQdotdotfile >> _qdotdot(0) >> _qdotdot(1) >> _qdotdot(2) >> _qdotdot(3) >> _qdotdot(4) >> _qdotdot(5) >> _qdotdot(6); 
    counter++;
    if(counter==linenr)
    {
      outputLine(_qdotdot(0), _qdotdot(1), _qdotdot(2), _qdotdot(3), _qdotdot(4), _qdotdot(5), _qdotdot(6) );
      break;
    }
    
  }
  inQdotdotfile.close();

  
  return 0;
}

void outputLine( double x1, double x2, double x3, double x4, double x5, double x6, double x7)
{
  cout << setiosflags(ios::left) << setiosflags(ios::fixed | ios::showpoint) <<setw(15) 
  << x1 << setw(15) << x2 <<setw(15) <<setw(15) << x3 <<setw(15) << x4 <<setw(15) << x5 <<setw(15) << x6 <<setw(15) << x7 <<"\n";
}
