#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
//typedef Matrix<double, 7, 1> Vector7d;
typedef Array<double, 6, 1> Array6d;

int main(void)
{
    MatrixXd axis_value(8, 6);
    axis_value = MatrixXd::Constant(8,6,0.0);
    std::cout << "axis_value = \n" << axis_value << std::endl;
    Array<Matrix4d, 7, 1> T;
    for (int i = 0; i < 7; i++)
    {
        T(i) = Matrix4d::Constant(0.0);         //Constant(4, 4, 0.0);
    }
    //std::cout << "T = \n" << T << std::endl;  // not valid
    for (int i = 0; i < 7; i++)
    {
        std::cout << "T[" << i << "] = \n" << T(i) << std::endl;
    }
    Array6d alpha;
    alpha << 1,3,5,7,9,6;
    alpha = Array6d::Constant(0.0);
    alpha[0] = 9.0;
    alpha[1] = 8.0;
    alpha[2] = 5.0;
    std::cout << "alpha = " << alpha << std::endl;
    return 0;
}
