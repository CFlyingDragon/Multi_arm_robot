#include "eigen3/Eigen/Eigen"
#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/Jacobi"
#include"Eigen/Eigen"

#include <iostream>

using namespace Eigen;
using namespace std;
int main(){
    Matrix<double,6,7> a;
    a.setRandom(6,7);
    cout <<"a:\n" << a << endl;
    MatrixXd b = a.transpose();
    cout << "b:\n" <<b << endl;

    MatrixXd c = a*b;
    cout << "c:\n" << c << endl;

    MatrixXd c_inv = c.inverse();
    cout << "c_inv:\n" << c_inv << endl;

    MatrixXd a_pinv = b*c_inv;
    cout << "a_pinv:\n" << a_pinv << endl;


    return 0;
}
