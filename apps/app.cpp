#include <controllers/model_predictive_control.hpp>
#include <models/mass_spring_damper.hpp>

#include <Eigen/Dense>

#include <iostream>
#include <cstdlib>  // EXIT_SUCCESS, EXIT_FAILURE

int main() 
{
    const int n = 4;
    const int m = 1;
    const int p = 1;
    const int K = 5;
    const float k1 = 100;
    const float k2 = 200;
    const float d1 = 1;
    const float d2 = 5;
    const float m1 = 2;
    const float m2 = 2;

    Eigen::MatrixXf A(n, n);
    A << 0, 1, 0, 0, -(k1 + k2) / m1, -(d1 + d2) / m1, k2/m1, d2/m1, 0, 0, 0, 1, k2/m2, d2/m2, -k2/m2, -d2/m2;
    Eigen::MatrixXf B(n, m);
    B << 0, 0, 0, 1/m2;
    Eigen::MatrixXf C(1, n);

    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(n, n) * 0.001;
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(n, n) * 10;
    Eigen::MatrixXf R = Eigen::MatrixXf::Constant(m, m, 10);
    Eigen::MatrixXf x0 = Eigen::MatrixXf::Random(n, 1);

    auto s = model_predictive_control::ModelPredictiveControl(A, B, P, Q, R);
    auto msd = models::MassSpringDamper(A, B, C, Eigen::MatrixXf::Zero(m, m), Eigen::MatrixXf::Zero(n, n));

    std::cout << "Optimal control:\n" << s.FindOptimalControl(x0) << std::endl;

    return EXIT_SUCCESS;
}
