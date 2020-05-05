#include <iostream>
#include <kdl/frames.hpp>

//using namespace KDL;
using namespace std;
int main(int argc, char** argv)
{
//    Vector v = Vector(1,2,3);
    KDL::Vector v1(1,2,3); //隐式创建对象
    for(int i=0; i<3; ++i){
        cout<<v1[i]<<";";
    }

    cout<<"\n";
    KDL::Vector v0 = KDL::Vector::Zero(); //显式创建对象
    KDL::Vector vec; //隐式创建对象，调用默认构造函数，成员数据被初始化为0
    cout<<vec.x()<<";"<<vec.y()<<";"<<vec.z()<<"\n";
//    std::cout<<v.data[0]<<std::endl; //获取数组的第一个元素
    for(int i=0; i<3; ++i){
        cout<<v0(i)<<" "; //循环输出数组的元素
    }
    cout<<"\n";
    cout<<v0.x()<<" "; //获取数组的第一个元素
    cout<<v0[0]; //获取数组的第一个元素
    return 0;
}
