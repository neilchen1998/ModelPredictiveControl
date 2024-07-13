#include <controllers/linear_quadratic_regulator.hpp>

#include <Eigen/Dense>

#include <iostream>

namespace linear_quadratic_regulator
{

LinearQuadraticRegulator::LinearQuadraticRegulator(const Eigen::MatrixXf& inputA, const Eigen::MatrixXf& inputB, const Eigen::MatrixXf& inputQ, const Eigen::MatrixXf& inputR)
{
    A = inputA;
    B = inputB;
    Q = inputQ;
    R = inputR;
    N = A.rows();
    M = B.cols();

    maxIteration = 50000000;
    tolerance = 0.0001;

    AT = A.transpose();
    BT = B.transpose();
    r = R(0, 0);

    #if DEBUG
        std::cout << "Constructor func was called!" << std::endl;
    #endif
}

// LinearQuadraticRegulator::LinearQuadraticRegulator(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, float q, float r)
// {

// }

void LinearQuadraticRegulator::SolveARE()
{
    Eigen::MatrixXf P = Eigen::MatrixXf::Random(N, N);
    Eigen::MatrixXf nextP;

    for (unsigned int i = 0; i < maxIteration; ++i)
    {
        float d = r + (BT * P * B)(0, 0);

        #if DEBUG
            if (i % 100 == 0)
            {
                std::cout << "P:\n" << P << std::endl;
                std::cout << "nextP:\n" << nextP << std::endl;
                std::cout << "AT * P * B * (1 / d) * BT * P * A:\n" << AT * P * B * (1 / d) * BT * P * A << std::endl;
            }
        #endif
        
        nextP = Q + AT * P * A - AT * P * B * (1 / d) * BT * P * A;
        auto err = fabs((nextP - P).maxCoeff());
        P = nextP;
        if (err < tolerance)    break;
    }
}

Eigen::MatrixXf LinearQuadraticRegulator::Solve()
{
    SolveARE();

    #if DEBUG
        std::cout << "SolveARE completed!" << std::endl;
        std::cout << "P:\n" << P << std::endl;
    #endif

    float Term = r + (BT * P * B)(0, 0);

    // return -(R + BT * P * B).inverse() * P * A;
    return -1 * BT * P * A;
}

LinearQuadraticRegulator::~LinearQuadraticRegulator() = default;

}   // linear_quadratic_regulator