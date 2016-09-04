// Example 2: Matrices and vectors
// source:
// http://eigen.tuxfamily.org/dox/GettingStarted.html#title0
// Compile: g++ -I /usr/include/eigen3/ eigen_ex2.cpp -o eigen_ex2
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main(void)
{
    MatrixXd m = MatrixXd::Random(3,3);
    m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
    cout << "m = " << '\n' << m << '\n';
    VectorXd v(3);
    v << 1, 2, 3;
    cout << "m * v = " << '\n' << m * v << '\n';
    return 0;
}
