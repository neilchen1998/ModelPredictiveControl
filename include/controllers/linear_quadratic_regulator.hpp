#ifndef LINEAR_QUADRATIC_REQULATOR_H_
#define LINEAR_QUADRATIC_REQULATOR_H_

#include <Eigen/Dense>

namespace linear_quadratic_regulator
{

class LinearQuadraticRegulator
{
public:
    explicit LinearQuadraticRegulator(const Eigen::MatrixXf& inputA, const Eigen::MatrixXf& inputB, const Eigen::MatrixXf& inputQ, const Eigen::MatrixXf& inputR);
    // explicit LinearQuadraticRegulator(const Eigen::MatrixXf& A, const Eigen::MatrixXf& B, float q, float r);
    Eigen::MatrixXf Solve();
    ~LinearQuadraticRegulator();

private:
    void SolveARE();

private:
    unsigned short N;
    unsigned short M;

    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;
    float r;
    Eigen::MatrixXf P;
    Eigen::MatrixXf K;

    Eigen::MatrixXf AT;
    Eigen::MatrixXf BT;

    unsigned int maxIteration;
    double tolerance;
};

}   // linear_quadratic_regulator

#endif  // LINEAR_QUADRATIC_REQULATOR_H_