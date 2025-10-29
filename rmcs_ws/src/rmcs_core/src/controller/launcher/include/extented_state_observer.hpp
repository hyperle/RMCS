// ExtendedStateObserver.h
#ifndef ESO_H
#define ESO_H

#include <Eigen/Dense>

/**
 * @brief 扩展状态观测器类
 * 
 * 用于估计系统状态和未知扰动
 * 扩展系统: 
 *   x_{k+1} = A * x_k + B * u_k + w_k
 *   d_{k+1} = d_k + v_k (扰动模型)
 *   z_k = H * [x_k; d_k] + n_k
 */
class ExtendedStateObserver {
private:
    Eigen::MatrixXd A_aug;  // 增广状态矩阵
    Eigen::MatrixXd B_aug;  // 增广输入矩阵
    Eigen::MatrixXd H_aug;  // 增广观测矩阵
    Eigen::MatrixXd L;      // 观测器增益
    
    Eigen::VectorXd x_hat;  // 状态估计
    int state_dim;          // 原状态维度
    int dist_dim;           // 扰动维度
    int aug_dim;            // 增广状态维度

public:
    /**
     * @brief 构造函数
     * @param A 原系统状态矩阵
     * @param B 原系统输入矩阵  
     * @param H 原系统观测矩阵
     * @param dist_dim 扰动维度
     */
    ExtendedStateObserver(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                        const Eigen::MatrixXd& H, int dist_dim = 1);
    
    /**
     * @brief 设计观测器增益
     * @param poles 期望极点
     */
    void designObserver(const Eigen::VectorXd& poles);
    
    /**
     * @brief 更新观测器
     * @param u 控制输入
     * @param y 系统输出
     */
    void update(const Eigen::VectorXd& u, const Eigen::VectorXd& y);
    
    // 获取状态估计
    Eigen::VectorXd getStateEstimate() const { return x_hat.head(state_dim); }
    
    // 获取扰动估计
    Eigen::VectorXd getDisturbanceEstimate() const { return x_hat.tail(dist_dim); }
    
    // 获取完整增广状态估计
    Eigen::VectorXd getAugmentedState() const { return x_hat; }
};

#endif