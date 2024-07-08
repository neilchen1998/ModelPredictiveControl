#ifndef MODELS_MODEL_PREDICTIVE_CONTROL_H_
#define MODELS_MODEL_PREDICTIVE_CONTROL_H_

#include <Eigen/Dense>

namespace model_predictive_control
{

class ModelPredictiveControl
{
public:
    ModelPredictiveControl();
    ModelPredictiveControl(Eigen::MatrixXf A, Eigen::MatrixXf B, Eigen::MatrixXf P, Eigen::MatrixXf Q, Eigen::MatrixXf R);
    ~ModelPredictiveControl();
    Eigen::MatrixXf FindOptimalControl(Eigen::MatrixXf x0);

private:
    void CalculateMatrixSBar();
    void CalculateMatrixQBar();
    void CalculateMatrixRBar();
    void CalculateMatrixTBar();
    void CalculateMatrixH();
    void CalculateMatrixF();
    void CalculateMatrixY();

private:
    int i;
    int N;
    int M;

    /// @brief horizen
    int K;

    // input matrices
    Eigen::MatrixXf A;
    Eigen::MatrixXf B;
    Eigen::MatrixXf P;
    Eigen::MatrixXf Q;
    Eigen::MatrixXf R;

    // essential matrices
    Eigen::MatrixXf QBar;
    Eigen::MatrixXf RBar;
    Eigen::MatrixXf SBar;
    Eigen::MatrixXf TBar;
    Eigen::MatrixXf H;
    Eigen::MatrixXf F;
    Eigen::MatrixXf Y;
};

}   // model_predictive_control

#endif  // MODELS_MODEL_PREDICTIVE_CONTROL_H_