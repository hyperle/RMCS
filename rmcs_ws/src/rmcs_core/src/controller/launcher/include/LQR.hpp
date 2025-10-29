// LinearQuadraticRegulator.h
#ifndef LQR_H
#define LQR_H

#include <Eigen/Dense>

/**
 * @brief 线性二次调节器类
 * 
 * 实现离散时间线性系统的LQR控制
 * 系统模型: x_{k+1} = A * x_k + B * u_k
 * 代价函数: J = Σ (x_k^T Q x_k + u_k^T R u_k)
 */
class LinearQuadraticRegulator {
private:
    Eigen::MatrixXd A;      // 状态矩阵
    Eigen::MatrixXd B;      // 输入矩阵
    Eigen::MatrixXd Q;      // 状态权重
    Eigen::MatrixXd R;      // 输入权重
    Eigen::MatrixXd K;      // 反馈增益矩阵
    Eigen::MatrixXd P;      // Riccati方程解
    
    int state_dim;          // 状态维度
    int input_dim;          // 输入维度
    
    bool solved;            // 求解标志

public:
    /**
     * @brief 构造函数
     * @param A 状态矩阵
     * @param B 输入矩阵
     * @param Q 状态权重
     * @param R 输入权重
     */
    LinearQuadraticRegulator(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    
    /**
     * @brief 求解Riccati方程
     * @param max_iter 最大迭代次数
     * @param tol 收敛容差
     * @return 是否成功求解
     */
    bool solveRiccati(int max_iter = 1000, double tol = 1e-6);
    
    /**
     * @brief 计算控制输入
     * @param x 当前状态
     * @param x_ref 参考状态（可选，默认为零）
     * @return 控制输入 u = -K*(x - x_ref)
     */
    Eigen::VectorXd computeControl(const Eigen::VectorXd& x, 
                                 const Eigen::VectorXd& x_ref = Eigen::VectorXd());
    
    /**
     * @brief 轨迹跟踪控制
     * @param x 当前状态
     * @param x_ref 参考状态
     * @param u_ref 参考输入（可选）
     * @return 控制输入 u = u_ref - K*(x - x_ref)
     */
    Eigen::VectorXd trackTrajectory(const Eigen::VectorXd& x, 
                                  const Eigen::VectorXd& x_ref,
                                  const Eigen::VectorXd& u_ref = Eigen::VectorXd());
    
    // 获取反馈增益矩阵
    Eigen::MatrixXd getGainMatrix() const { return K; }
    
    // 获取Riccati方程解
    Eigen::MatrixXd getRiccatiSolution() const { return P; }
    
    /**
     * @brief 检查系统可控性
     * @return 系统是否可控
     */
    bool checkControllability() const;
    
private:
    /**
     * @brief 构建可控性矩阵
     * @return 可控性矩阵
     */
    Eigen::MatrixXd buildControllabilityMatrix() const;
};

#endif