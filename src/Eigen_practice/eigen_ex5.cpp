// Eigen lib tutorial -- Array class and coefficient-wise operations
// Source:
// http://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main(void)
{
    // Accessing values inside an Array
    ArrayXXf m(2,2);

    // assign some value coefficient by coefficient
    m(0,0) = 1.0; m(0,1) = 2.0;
    m(1,0) = 3.0; m(1,1) = m(0,1) + m(1,0);

    // print values to standard output
    std::cout << m << '\n' << std::endl;

    // using the comma-initializer is also allowed
    m << 1.0, 3.0,
         5.0, 7.0;
    std::cout << m  << std::endl;

    // Array addition and subtraction
    ArrayXXf a(3,3);
    ArrayXXf b(3,3);
    a << 1,2,3,
         4,5,6,
         7,8,9;
    b << 1,2,3,
         1,2,3,
         1,2,3;

    // Adding two arrays
    std::cout << "a + b = " << '\n' << a + b << std::endl;

    // Subtracting a scalar from an array
    std::cout << "a - 2 = " << '\n' << a -2 << std::endl;


    // Array multiplication
    ArrayXXf c(2,2);
    ArrayXXf d(2,2);
    c << 1,2,
         3,4;
    d << 5,6,
         7,8;
    std::cout << "c * d = " << '\n' << c * d << std::endl;

    // Converting b/w array and matrix expressions
    MatrixXf u(2,2);
    MatrixXf v(2,2);
    MatrixXf result(2,2);

    u << 1,2,
         3,4;
    v << 5,6,
         7,8;

    result = u * v;
    std::cout << "-- Matrix u * v: --\n" << result << '\n' << std::endl;

    result = u.array() * v.array();
    std::cout << "-- Array u * v: --\n" << result << '\n' << std::endl;

    result = u.cwiseProduct(v);
    std::cout << "-- With cwiseProduct: --\n" << result << '\n' << std::endl;

    result = u.array() + 4;
    std::cout << "-- Array u + 4: --\n" << result << '\n' << std::endl;

    result = (u.array() + 4).matrix() * u;
    std::cout << "-- Combination 1: --" << std::endl << result << std::endl << std::endl;
    result = (u.array() * v.array()).matrix() * u;
    std::cout << "-- Combination 2: --" << std::endl << result << std::endl << std::endl;

    return 0;
}
