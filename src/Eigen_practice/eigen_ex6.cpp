// Eigen tutorial -- Block operations
// SOurce:
// http://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main(void)
{
    MatrixXf m(4,4);
    m << 1, 2, 3, 4,
         5, 6, 7, 8,
         9,10,11,12,
        13,14,15,16;
    std::cout << "Block in the middle" << std::endl;
    std::cout << m.block<2,2>(1,1) << '\n' << std::endl;
    for (int i = 0; i <= 3; i++) {
        std::cout << "Block of size " << i << "x" << i << std::endl;
        std::cout << m.block(0,0,i,i) << '\n' << std::endl;
    }

    // block can also be used as lvalues, meaning that you can assign to a block
    Array22f r;
    r << 1,2,
         3,4;
    Array44f a = Array44f::Constant(0.6);
    std::cout << "Here is the array a:\n" << a << '\n' << std::endl;
    a.block<2,2>(1,1) = r;
    std::cout << "Here is now a with m copied into its central 2x2 block:\n"
        << a << '\n' << std::endl;
    a.block(0,0,2,3) = a.block(2,1,2,3);
    std::cout << "Here is now a with bottom-right 2x3 copied into top-left 2x2 block:\n"
        << a << '\n' << std::endl;

    // Columns and rows
    MatrixXf u(3,3);
    u << 1,2,3,
         4,5,6,
         7,8,9;
    std::cout << "Here is thematrix u:\n" << u << std::endl;
    std::cout << "2nd Row: " << u.row(1) << std::endl;
    u.col(2) += 3 * u.col(0);
    std::cout << "After adding 3 times the first column into the third column,"
        << "the matrix u is:\n" << u << std::endl;

    // Corner-related operations
    MatrixXf w(4,4);
    w << 1, 2, 3, 4,
         5, 6, 7, 8,
         9,10,11,12,
        13,14,15,16;
    std::cout << "w.leftCols(2) =\n" << m.leftCols(2) << '\n' << std::endl;
    std::cout << "w.bottomRows<2>() =\n" <<  w.bottomRows<2>() << '\n' << std::endl;
    w.topLeftCorner(1,3) = w.bottomRightCorner(3,1).transpose();
    std::cout << "After assignment, w =\n" << w << std::endl;

    // Block operations for vectors
    ArrayXf y(6);
    y << 1,2,3,4,5,6;
    std::cout << "y.head(3) = \n" << y.head(3) << '\n' << std::endl;
    std::cout << "y.tail<3>() =\n" << y.tail<3>() << "\n\n";
    y.segment(1,4) *= 2;
    std::cout << "after 'y.segment(1,4) *= 2', y =\n" << y << std::endl;

    return 0;
}
