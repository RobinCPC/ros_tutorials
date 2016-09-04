// A simple first program of Eigen3
// source:
// http://eigen.tuxfamily.org/dox/GettingStarted.html#title0
// Compile: g++ -I /usr/include/eigen3/ eigen_ex1.cpp -o eigen_ex1
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;

int main()
{
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << '\n';
    return 0;
}
