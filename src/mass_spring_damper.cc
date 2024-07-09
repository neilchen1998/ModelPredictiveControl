#include <models/mass_spring_damper.hpp>

#include <Eigen/Dense>

#if DEBUG
    #include <iostream>
    #include <fmt/format.h>
#endif

namespace models
{

models::MassSpringDamper::MassSpringDamper()
{
}

models::MassSpringDamper::MassSpringDamper(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D, Eigen::MatrixXf x0) :
    A(A),
    B(B),
    C(C),
    D(D),
    x(x0),
    t(0)
{
    N = A.rows();
    assert(A.cols() == N);
    M = B.rows();
    assert(B.cols() == N);
    P = C.rows();
    assert(C.cols() == P);
}

Eigen::MatrixXf MassSpringDamper::Step()
{
    // increases time stamp
    ++t;

    // updates the state
    x = A * x;

    #if DEBUG
        std::cout << "X:\n" << X << std::endl;
    #endif

    // updates the output
    Eigen::MatrixXf y = C * x;

    return y;
}

Eigen::MatrixXf MassSpringDamper::Step(Eigen::MatrixXf u)
{
    // increases time stamp
    ++t;

    // updates the state
    x = A * x + B * u;

    #if DEBUG
        std::cout << "X:\n" << X << std::endl;
    #endif

    // updates the output
    Eigen::MatrixXf y = C * x + D * u;

    return y;
}

} // models
