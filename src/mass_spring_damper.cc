#include <models/mass_spring_damper.hpp>

#include <Eigen/Dense>

#if DEBUG
    #include <iostream>
    #include <fmt/format.h>
#endif

namespace models
{

models::MassSpringDamper::MassSpringDamper() :
    t(0)
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
    assert(B.cols() == 1);
    P = C.rows();
    assert(C.cols() == N);

    assert(x.rows() == N);
    assert(x.cols() == 1);
}

void MassSpringDamper::Step()
{
    // increases time stamp
    ++t;

    // updates the state
    x = A * x;

    // updates the output
    y = C * x;
}

void MassSpringDamper::Step(float u)
{
    // increases time stamp
    ++t;

    // updates the state
    x = A * x + B * u;

    // updates the output
    y = C * x + D * u;
}

Eigen::MatrixXf MassSpringDamper::Output() const
{
    return y;
}
} // models
