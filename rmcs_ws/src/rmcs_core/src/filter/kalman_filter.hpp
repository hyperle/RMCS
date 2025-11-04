#pragma once

#include <Eigen/Dense>

namespace rmcs_core::filter{

class KalmanFilter {
public:
    explicit KalmanFilter(double dt = 0.001, double process_noise = 1e-5, double measurement_noise = 1e-3) 
        : dt_(dt), process_noise_(process_noise), measurement_noise_(measurement_noise) {
        x_ = Eigen::Vector2d::Zero();
        F_ = Eigen::Matrix2d::Identity();
        F_(0, 1) = dt_;
        H_ = Eigen::Matrix<double, 1, 2>::Zero();
        H_(0, 0) = 1.0;
        Q_ = Eigen::Matrix2d::Identity() * process_noise;
        R_ = Eigen::Matrix<double, 1, 1>::Identity() * measurement_noise;
        P_ = Eigen::Matrix2d::Identity();
    }
    
    void init(double angle, double angular_velocity = 0.0) {
        x_ << angle, angular_velocity;
        P_ = Eigen::Matrix2d::Identity();
    }
    
    double update(double measurement) {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
        
        Eigen::Matrix<double, 1, 1> y = Eigen::Matrix<double, 1, 1>(measurement - H_ * x_);
        Eigen::Matrix<double, 1, 1> S = H_ * P_ * H_.transpose() + R_;
        Eigen::Matrix<double, 2, 1> K = P_ * H_.transpose() * S.inverse();
        
        x_ = x_ + K * y;
        P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
        
        return x_(0); // 滤波后的角度
    }
    
    double getAngle() const { return x_(0); }
    double getAngularVelocity() const { return x_(1); }

private:
    double dt_;
    double process_noise_;
    double measurement_noise_;
    
    Eigen::Vector2d x_; // 状态向量
    Eigen::Matrix2d F_; // 状态转移矩阵
    Eigen::Matrix<double, 1, 2> H_; // 观测矩阵
    Eigen::Matrix2d P_; // 误差协方差矩阵
    Eigen::Matrix2d Q_; // 过程噪声协方差
    Eigen::Matrix<double, 1, 1> R_; // 观测噪声协方差
};

}
