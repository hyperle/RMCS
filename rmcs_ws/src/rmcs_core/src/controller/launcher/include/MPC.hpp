// ModelPredictiveController.h
#ifndef MPC_H
#define MPC_H

#include <Eigen/Dense>
#include <vector>

/**
 * @brief 模型预测控制器类
 * 
 * 实现线性系统的模型预测控制
 * 系统模型: x_{k+1} = A * x_k + B * u_k
 * 代价函数: J = Σ (x_k^T Q x_k + u_k^T R u_k) + x_N^T P x_N
 */
class ModelPredictiveController {
private:
    // 系统参数
    Eigen::MatrixXd A;  // 状态矩阵
    Eigen::MatrixXd B;  // 输入矩阵
    
    // 权重矩阵
    Eigen::MatrixXd Q;  // 状态权重
    Eigen::MatrixXd R;  // 输入权重
    Eigen::MatrixXd P;  // 终端权重
    
    // 控制参数
    int prediction_horizon;  // 预测步长
    int state_dim;          // 状态维度
    int input_dim;          // 输入维度
    
    // 约束参数
    Eigen::VectorXd u_min;  // 输入下限
    Eigen::VectorXd u_max;  // 输入上限
    Eigen::VectorXd x_min;  // 状态下限  
    Eigen::VectorXd x_max;  // 状态上限
    
    bool constraints_enabled; // 约束使能标志

public:
    /**
     * @brief 构造函数
     * @param A 状态矩阵
     * @param B 输入矩阵
     * @param Q 状态权重
     * @param R 输入权重
     * @param P 终端权重
     * @param horizon 预测步长
     */
    ModelPredictiveController(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                            const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
                            const Eigen::MatrixXd& P, int horizon);
    
    /**
     * @brief 设置输入约束
     * @param min 输入最小值
     * @param max 输入最大值
     */
    void setInputConstraints(const Eigen::VectorXd& min, const Eigen::VectorXd& max);
    
    /**
     * @brief 设置状态约束
     * @param min 状态最小值
     * @param max 状态最大值
     */
    void setStateConstraints(const Eigen::VectorXd& min, const Eigen::VectorXd& max);
    
    /**
     * @brief 计算控制输入
     * @param x0 初始状态
     * @param x_ref 参考轨迹（可选）
     * @return 最优控制输入序列
     */
    std::vector<Eigen::VectorXd> computeControl(const Eigen::VectorXd& x0,
                                              const std::vector<Eigen::VectorXd>& x_ref = {});
    
    /**
     * @brief 计算单步控制输入（简化版本）
     * @param x0 当前状态
     * @param x_target 目标状态
     * @return 最优控制输入
     */
    Eigen::VectorXd computeStep(const Eigen::VectorXd& x0, const Eigen::VectorXd& x_target);
    
private:
    /**
     * @brief 构建QP问题
     * @param x0 初始状态
     * @param x_ref 参考轨迹
     * @return 求解得到的控制序列
     */
    std::vector<Eigen::VectorXd> solveQP(const Eigen::VectorXd& x0,
                                       const std::vector<Eigen::VectorXd>& x_ref);
    
    /**
     * @brief 无约束MPC求解（解析解）
     * @param x0 初始状态
     * @param x_ref 参考轨迹
     * @return 控制序列
     */
    std::vector<Eigen::VectorXd> solveUnconstrained(const Eigen::VectorXd& x0,
                                                  const std::vector<Eigen::VectorXd>& x_ref);
};

#endif