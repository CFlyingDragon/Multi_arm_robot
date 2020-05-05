/*********************************************************************
 * 作用：对基于力控的阻抗控制器进行测试
 * 测试结论：（1）建立好机器人模型后，应先设置期望位置，在设置机械臂状态，因为关节角的速度和加速度
 *         采用期望位置来估计，所以得到期望位置后才能的到关节状态
 *          (2)编译的时候，应该把该ImpedanceController_lib加到目标连接上
 *          target_link_libraries(imp_controller_test ${catkin_LIBRARIES} ImpedanceController_lib)
 * 作者：陈永厅
 * 时间：2019/12/23
 * 版权：哈尔滨工业大学（深圳）
 *********************************************************************/
#include "impedance_controller/imp_controller.hpp"
#include <iostream>

int main()
{
    std::string path = "/home/d/catkin_ws/src/robot_description/armc_description/urdf/armc_description.urdf";
    std::string base_link = "base_link";
    std::string top_link = "Link7";

    impedance_controller::ImpedanceController armc_impc;

    //设置机器人模型
    if(!armc_impc.set_robot_model(path,base_link,top_link))
    {
        std::cout << "Creating robot model faled!" << std::endl;
    }

     std::cout << "Creating robot model succeed!" << std::endl;

    //设置基座标系的重力
    std::vector<double> g{0,0,9.8};
    if(!armc_impc.set_base_gravity(g))
    {
        std::cout << "Set base gravity faled!" << std::endl;
    }
    std::cout << "Set base gravity succeed!" << std::endl;

    //设置阻抗参数
    std::vector<double> md{0.5,0.2,0.3,0.4,0.6,0.5};
    std::vector<double> bd{0.5,0.2,0.3,0.4,0.6,0.5};
    std::vector<double> kd{0.5,0.2,0.3,0.4,0.6,0.5};
    if(armc_impc.set_imp_param(md,bd,kd)){
         std::cout << "Set imp_param succeed!" << std::endl;
    }

    //设置末端期望位置
    std::vector<double> xd{0.5,0.2,0.3,0.4,0.6,0.5};
    std::vector<double> xvd{0.5,0.2,0.3,0.4,0.6,0.5};
    std::vector<double> xad{0.5,0.2,0.3,0.4,0.6,0.5};
    if(armc_impc.set_expect_pos(xd,xvd,xad)){
         std::cout << "Set expect_pos succeed!" << std::endl;
    }

    //设置机器人的状态
    std::vector<double> qq{0.5,0.2,0.3,0.4,0.6,0.5,1};
    if(armc_impc.set_robot_state(qq))
    {
         std::cout << "Set qq succeed!" << std::endl;
    }

    //设置机器人的状态
    std::vector<double> fd{0.5,0.2,0.3,0.4,0.6,0.5};
    if(armc_impc.set_expect_force(fd))
    {
         std::cout << "Set expect_force succeed!" << std::endl;
    }

    //求取关节力矩
    std::vector<double> tau;
    armc_impc.get_joint_torque(tau);
    for(int i = 0;i<tau.size();i++){
        std::cout << "tau[" << i <<"]: " << tau[i] <<std::endl;
    }
    return 1;
}
