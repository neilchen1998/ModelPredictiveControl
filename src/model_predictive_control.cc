#include <models/model_predictive_control.hpp>

#include <Eigen/Dense>

#if DEBUG
    #include <iostream>
    #include <fmt/format.h>
#endif

namespace model_predictive_control
{

ModelPredictiveControl::ModelPredictiveControl()
{

}

ModelPredictiveControl::ModelPredictiveControl(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf R, Eigen::VectorXf x0) : 
    A(A),
    B(B),
    P(P),
    Q(Q),
    R(R),
    x0(x0)
{
    // A: N * N
    assert(A.cols() == A.rows());
    N = A.cols();

    // B: N * M
    assert(B.rows() == N);
    M = B.cols();

    #if DEBUG
        fmt::print("N: {}, M: {}\n",  N, M);
    #endif

    // R: M * M
    assert(R.rows() == M);
    assert(R.cols() == M);

    K = 5;

    CalculateMatrixSBar();
    CalculateMatrixTBar();
    CalculateMatrixQBar();
    CalculateMatrixRBar();
    CalculateMatrixH();
    CalculateMatrixF();
    CalculateMatrixY();
}

ModelPredictiveControl::~ModelPredictiveControl()
{

}

Eigen::MatrixXf ModelPredictiveControl::FindOptimalControl()
{
    return - H.inverse() * F.transpose() * x0;
}

void ModelPredictiveControl::CalculateMatrixQBar()
{
    QBar = Eigen::MatrixXf::Zero(K * N, K * N);
    
    for (int i = 0; i < K - 1; i++)
    {
        QBar.block(i*N, i*N, N, N) = Q;
    }
    
    QBar.block((K - 1) * N, (K - 1) * N, N, N) = P;

    #if DEBUG
        std::cout << "Matrix QBar:\n" << QBar << std::endl;
        assert(QBar.rows() == K*N);
        assert(QBar.cols() == K*N);
    #endif
}

void ModelPredictiveControl::CalculateMatrixRBar()
{
    RBar = Eigen::MatrixXf::Zero(K * M, K * M);
    
    for (int i = 0; i < K; i++)
    {
        RBar.block(i*M, i*M, M, M) = R;
    }

    #if DEBUG
        std::cout << "Matrix RBar:\n" << RBar << std::endl;
        assert(RBar.rows() == K*M);
        assert(RBar.cols() == K*M);
    #endif
}

void ModelPredictiveControl::CalculateMatrixSBar()
{
    SBar = Eigen::MatrixXf::Zero(K * N, K);

    // calculates the last row blocks
    Eigen::MatrixXf product = B;
    for (int k = K-1; k >= 0; --k, product = A * product)
    {
        SBar.block((K-1) * N, k, N, 1) = product;
    }
    
    // copies from the last row blocks and applies them to the rest
    for (int k = K - 2; k >= 0; --k)
    {
        SBar.block(k * N, 0, N, k+1) = SBar.block((k + 1) * N, 1, N, k+1);
    }

    #if DEBUG
        std::cout << "Matrix SBar:\n" << SBar << std::endl;
        assert(SBar.rows() == K*N);
        assert(SBar.cols() == K*M);
    #endif
}

void ModelPredictiveControl::CalculateMatrixH()
{
    #if DEBUG
        assert(SBar.rows() == QBar.rows());
        assert(QBar.cols() == SBar.rows());
        assert(RBar.rows() == SBar.cols());
        assert(RBar.cols() == SBar.cols());
    #endif
    H = 2 * (RBar + SBar.transpose() * QBar * SBar);

    #if DEBUG
        std::cout << "Matrix H:\n" << H << std::endl;
        assert(H.rows() == K*M);
        assert(H.cols() == K*M);
    #endif
}
void ModelPredictiveControl::CalculateMatrixTBar()
{
    TBar = Eigen::MatrixXf::Zero(K * N, N);
    Eigen::MatrixXf product = A;
    for (int k = 0; k < K; ++k, product *= A)
    {
        TBar.block(k*N, 0, N, N) = product;
    }

    #if DEBUG
        std::cout << "Matrix TBar:\n" << TBar << std::endl;
        assert(TBar.rows() == K*N);
        assert(TBar.cols() == N);
    #endif
}

void model_predictive_control::ModelPredictiveControl::CalculateMatrixF()
{
    #if DEBUG
        assert(TBar.rows() == QBar.rows());
        assert(QBar.cols() == SBar.rows());
    #endif

    F = 2 * TBar.transpose() * QBar * SBar;

    #if DEBUG
        std::cout << "Matrix F:\n" << F << std::endl;
    #endif
}

void model_predictive_control::ModelPredictiveControl::CalculateMatrixY()
{
    Y = 2 * (Q + TBar.transpose() * QBar * TBar);

    #if DEBUG
        std::cout << "Matrix Y:\n" << Y << std::endl;
        assert(Y.rows() == N);
        assert(Y.cols() == N);
    #endif
}
} // model_predictive_control


