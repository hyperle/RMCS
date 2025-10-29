// KalmanFilter.h
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

/**
 * @brief 卡尔曼滤波器类
 * 
 * 实现线性系统的卡尔曼滤波算法，用于状态估计
 * 系统模型: 
 *   x_k = A * x_{k-1} + B * u_{k-1} + w_k
 *   z_k = H * x_k + v_k
 * 其中 w_k ~ N(0, Q), v_k ~ N(0, R)
 */
class KalmanFilter {
private:
    // 系统矩阵
    Eigen::MatrixXd A;  // 状态转移矩阵
    Eigen::MatrixXd B;  // 控制输入矩阵  
    Eigen::MatrixXd H;  // 观测矩阵
    Eigen::MatrixXd Q;  // 过程噪声协方差
    Eigen::MatrixXd R;  // 观测噪声协方差
    Eigen::MatrixXd P;  // 估计误差协方差
    Eigen::MatrixXd K;  // 卡尔曼增益
    
    // 系统状态
    Eigen::VectorXd x;  // 状态向量
    
    // 系统维度
    int state_dim;      // 状态维度
    int meas_dim;       // 观测维度
    int ctrl_dim;       // 控制维度
    
    bool initialized;   // 初始化标志

public:
    /**
     * @brief 构造函数
     * @param state_dim 状态维度
     * @param meas_dim 观测维度  
     * @param ctrl_dim 控制维度
     */
    KalmanFilter(int state_dim, int meas_dim, int ctrl_dim = 0);
    
    /**
     * @brief 初始化滤波器
     * @param A 状态转移矩阵
     * @param B 控制输入矩阵
     * @param H 观测矩阵
     * @param Q 过程噪声协方差
     * @param R 观测噪声协方差
     * @param P0 初始误差协方差
     * @param x0 初始状态
     */
    void init(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
              const Eigen::MatrixXd& H, const Eigen::MatrixXd& Q,
              const Eigen::MatrixXd& R, const Eigen::MatrixXd& P0,
              const Eigen::VectorXd& x0);
    
    /**
     * @brief 预测步骤（时间更新）
     * @param u 控制输入向量
     */
    void predict(const Eigen::VectorXd& u = Eigen::VectorXd());
    
    /**
     * @brief 更新步骤（测量更新）
     * @param z 观测向量
     */
    void update(const Eigen::VectorXd& z);
    
    /**
     * @brief 完整滤波步骤（预测+更新）
     * @param z 观测向量
     * @param u 控制输入向量
     * @return 更新后的状态估计
     */
    Eigen::VectorXd filter(const Eigen::VectorXd& z, 
                          const Eigen::VectorXd& u = Eigen::VectorXd());
    
    // 获取状态估计
    Eigen::VectorXd getState() const { return x; }
    
    // 获取卡尔曼增益
    Eigen::MatrixXd getGain() const { return K; }
    
    // 获取误差协方差
    Eigen::MatrixXd getCovariance() const { return P; }
};

#endif