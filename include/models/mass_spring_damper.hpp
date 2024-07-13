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

    /// @brief Step once without any input
    /// @return The output of the system
    void Step();

    /// @brief Step once with a given input
    /// @param u The given input
    /// @return The output of the system
    void Step(float u);

    Eigen::MatrixXf Output() const;

// variables need to be protected so the tests can access them
protected:

    int N;
    int M;
    int P;

    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf C;
    Eigen::MatrixXf D;

    Eigen::MatrixXf x;
    Eigen::MatrixXf y;

    /// @brief the current time
    unsigned int t;
};

}   // models

#endif  // MODELS_MASS_SPRING_DAMPER_H_