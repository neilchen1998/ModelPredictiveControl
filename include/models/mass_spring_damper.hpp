#ifndef MODELS_MASS_SPRING_DAMPER_H_
#define MODELS_MASS_SPRING_DAMPER_H_

#include <Eigen/Dense>

namespace models
{

class MassSpringDamper
{
public:
    MassSpringDamper();
    MassSpringDamper(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd C, Eigen::MatrixXd D, Eigen::MatrixXd x0, float T);
    ~MassSpringDamper() = default;

    /// @brief Step once without any input
    /// @return The output of the system
    void Step();

    /// @brief Step once with a given input
    /// @param u The given input
    /// @return The output of the system
    void Step(float u);

    Eigen::MatrixXd Output() const;

// variables need to be protected so the tests can access them
protected:

    int N;
    int M;
    int P;

    // the discrete matrices of A & B are different from their continous counterparts
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;

    // the discrete matrices of C & D are identical to their continous counterparts
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;

    Eigen::MatrixXd x;
    Eigen::MatrixXd y;

    /// @brief the current time
    unsigned int t;
};

}   // models

#endif  // MODELS_MASS_SPRING_DAMPER_H_