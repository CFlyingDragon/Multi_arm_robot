/* 基于KDL库测试代码
 *作用：建立对封装后的KDL库的测试
 *作者：陈永厅
 *版权：哈尔滨工业大学（深圳）
 *时间：2019/12/23
*/
#include "kdl_kinematic/kdl_robot_base.hpp"
#include <iostream>

int main(){
    std::string path = "/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf";
    std::string base_link = "base_link";
    std::string top_link = "Link7";

    KDL::KdlBase armc;

    //设置机器人模型
    if(!armc.set_robot_model(path,base_link,top_link))
    {
        std::cout << "Creating robot model faled!" << std::endl;
    }

    //设置基座标系的重力
    std::vector<double> g{0,0,9.8};
    if(!armc.set_base_gravity(g))
    {
        std::cout << "Set base gravity faled!" << std::endl;
    }

    //求取正运动学
    std::vector<double> qq{0.5,0.2,0.3,0.4,0.6,0.5,1};
    std::vector<double> Xe;
    if(armc.get_fkine(qq,Xe))
    {
        for(int i=0;i<Xe.size();i++){
            std::cout << "The Xe[" << i << "]: " << Xe[i] << std::endl;
        }
    }

    //求取逆运动学
    std::vector<double> q_init(7,0.0);
    std::vector<double> qq_i;
    if(armc.get_ikine(q_init,Xe,qq_i)){
        for(int i=0;i<qq_i.size();i++){
            std::cout << "The qq_i[" << i << "]: " << qq_i[i] << std::endl;
        }
    }

    //求取逆动力学
    std::vector<double> qv{5,2,3,4,6,5,1};
    std::vector<double> qa{2,3,3,7,6,5,1};
    std::vector<double> tau;
    if(armc.get_idyna(qq,qv,qa,tau)){
        for(int i=0;i<tau.size();i++){
            std::cout << "The tau[" << i << "]: " << tau[i] << std::endl;
        }
    }
    return 0;
}



