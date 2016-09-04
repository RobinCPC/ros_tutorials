// souece: http://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
// Compile: g++ -I /usr/include/eigen3/ eigen_ex1.cpp -o eigen_ex1
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;

int main(void)
{
    Eigen::Matrix3f m3;
    // Comma-initialization
    m3 << 1, 2, 3,
          4, 5, 6,
          7, 8, 9;
    std::cout << m3 << std::endl;
    
    // Resize
    MatrixXd m(2,5);
    m.resize(4,3);
    std::cout << "The matrix m is of size " 
              << m.rows() << "X" << m.cols() << std::endl;
    std::cout << "It has " << m.size() << " coefficients"
        << std::endl;
    Eigen::VectorXd v(2);
    v.resize(5);
    std::cout << "The vector v is of size " << v.size()
        << std::endl;
    std::cout << "As a matrix, v is of size "
              << v.rows() << "x" << v.cols() << std::endl;

    // Assignment (operator=) and resizing
    Eigen::MatrixXf a(2,2);
    std::cout << "a is of size " << a.rows() << "x" << a.cols() 
        << std::endl;
    Eigen::MatrixXf b(3,3);
    a = b;  // using operator=, will resize the matrix on the left-hand.
    std::cout << "a is now of size " << a.rows() << "x" << a.cols() 
        << std::endl;
    return 0;
}
