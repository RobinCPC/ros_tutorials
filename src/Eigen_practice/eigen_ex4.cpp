// Eigen lib Tutorial
// Source: http://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html
// Compile: g++ -I /usr/include/eigen3/ eigen_ex4.cpp -o eigen_ex4

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main(void)
{
    // addition and subtraction
    Matrix2d a;
    a << 1, 2,
         3, 4;
    MatrixXd b(2,2);
    b << 2, 3,
         1, 4;
    std::cout << "a + b =\n" << a + b << std::endl;
    std::cout << "a - b =\n" << a - b << std::endl;
    std::cout << "Doing a += b;" << std::endl;
    a += b;
    std::cout << "Now a = \n" << a << std::endl;
    Vector3d v(1,2,3);
    Vector3d w(1,0,0);
    std::cout << "-v + w - v =\n" << -v + w - v << std::endl;

    // Scalar multiplication and division
    a -= b;
    std::cout << "a * 2,5 =\n" << a * 2.5  << std::endl;
    std::cout << "0.1 * v =\n" << 0.1 * v  << std::endl;
    std::cout << "Doing v *= 2;" << std::endl;
    v *= 2;
    std::cout << "Now v =\n" << v << std::endl;

    // Transposition and conjugation
    MatrixXcf c = MatrixXcf::Random(2,2);
    std::cout << "Here is the matrix c\n" << c  << std::endl;
    std::cout << "Here is the matrix c^T\n" << c.transpose() << std::endl;
    std::cout << "Here is the conjugate of c\n" <<
        c.conjugate() << std::endl;
    std::cout << "Here is the matrix c^*\n" << c.adjoint() << std::endl;

    // Matrix-matrix and matrix-vector multiplication
    Matrix2d mat;
    mat << 1, 2,
           3, 4;
    Vector2d u(-1,1), x(2,0);
    std::cout << "Here is mat*mat:\n" << mat*mat  << std::endl;
    std::cout << "Here is mat*u:\n" << mat*u << std::endl;
    std::cout << "Here is u^T*mat:\n" << u.transpose()*mat << std::endl;
    std::cout << "Here is u^T*x\n" << u.transpose()*x << std::endl;
    std::cout << "Here is u*x^T\n" << u*x.transpose() << std::endl;
    std::cout << "let's multiply mat by itself" << std::endl;
    mat = mat*mat;
    std::cout << "Now mat is mat:\n" << mat << std::endl;

    // Dot product and cross product
    Vector3d e(1,2,3);
    Vector3d f(0,1,2);

    std::cout << "Dot product: " << e.dot(f) << std::endl;
    double dp = e.adjoint()*f;  // automatic converiosn of the inner product to a scalar
    std::cout << "Dot product via a matrix product:" << dp << std::endl;
    std::cout << "Cross product:\n" << e.cross(f) << std::endl;

    // Basic arithmetic reduction operations
    Matrix2d mat2;
    mat2 << 1, 2,
            3, 4;
    std::cout << "Here is mat2.sum():       " << mat2.sum() << std::endl;
    std::cout << "Here is mat2.prod():      " << mat2.prod() << std::endl;
    std::cout << "Here is mat2.mean():      " << mat2.mean() << std::endl;
    std::cout << "Here is mat2.minCoeff():  " << mat2.minCoeff() << std::endl;
    std::cout << "Here is mat2.maxCoeff():  " << mat2.maxCoeff() << std::endl;
    std::cout << "Here is mat2.trace():     " << mat2.trace() << std::endl;

    return 0;
}
