#ifndef MODELS_MASS_SPRING_DAMPER_H_
#define MODELS_MASS_SPRING_DAMPER_H_

#include <Eigen/Dense>

namespace models
{

class MassSpringDamper
{
public:
    MassSpringDamper();
    MassSpringDamper(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf C, Eigen::MatrixXf D, Eigen::MatrixXf x0);
    ~MassSpringDamper() = default;

    Eigen::MatrixXf Step(Eigen::MatrixXf u);

private:

    int N;
    int M;
    int P;

    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf D;

    Eigen::MatrixXf x;

    /// @brief the current time
    unsigned int t;
};

}   // models

#endif  // MODELS_MASS_SPRING_DAMPER_H_