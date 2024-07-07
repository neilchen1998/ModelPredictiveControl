#include <models/model_predictive_control.hpp>

#include <Eigen/Dense>

#include <iostream>
#include <cstdlib>  // EXIT_SUCCESS, EXIT_FAILURE

int main() 
{
    const int N = 4;
    const int M = 1;
    const int K = 5;
    const float k1 = 100;
    const float k2 = 200;
    const float d1 = 1;
    const float d2 = 5;
    const float m1 = 2;
    const float m2 = 2;

    Eigen::MatrixXf A(N, N);
    A << 0, 1, 0, 0, -(k1 + k2) / m1, -(d1 + d2) / m1, k2/m1, d2/m1, 0, 0, 0, 1, k2/m2, d2/m2, -k2/m2, -d2/m2;
    Eigen::MatrixXf B(N, M);
    B << 0, 0, 0, 1/m2;

    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(N, N) * 0.001;
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(N, N) * 10;
    Eigen::MatrixXf R = Eigen::MatrixXf::Constant(M, M, 10);
    Eigen::MatrixXf x0 = Eigen::MatrixXf::Random(N, 1);

    auto s = model_predictive_control::ModelPredictiveControl(A, B, P, Q, R, x0);

    std::cout << "Optimal control:\n" << s.FindOptimalControl() << std::endl;

    return EXIT_SUCCESS;
}
