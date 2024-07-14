#include <models/mass_spring_damper.hpp>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

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

models::MassSpringDamper::MassSpringDamper(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D, Eigen::MatrixXd x0, float T) :
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

    #if DEBUG
        std::cout << "Matrix A:\n" << A << std::endl;
        std::cout << "Matrix B:\n" << B << std::endl;
    #endif

    // transforms from continuous to discrete
    // Ad = phi(T)
    Ad = (A * T).exp();

    // Bd = (Ad - I) * A^-1 * B
    // Bd = A^-1 * (Ad - I) * B
    Bd = (Ad - Eigen::MatrixXd::Identity(N, N)) * A.inverse() * B;
    // Bd = A.inverse() * (Ad - Eigen::MatrixXd::Identity(N, N))* B;

    #if DEBUG
        std::cout << "Matrix Ad:\n" << Ad << std::endl;
        std::cout << "Matrix Bd:\n" << Bd << std::endl;
    #endif
}

void MassSpringDamper::Step()
{
    // increases time stamp
    ++t;

    // updates the state
    x = Ad * x;

    // updates the output
    y = C * x;
}

void MassSpringDamper::Step(float u)
{
    // increases time stamp
    ++t;

    // updates the state
    x = Ad * x + Bd * u;

    // updates the output
    y = C * x + D * u;
}

Eigen::MatrixXd MassSpringDamper::Output() const
{
    return y;
}
} // models
