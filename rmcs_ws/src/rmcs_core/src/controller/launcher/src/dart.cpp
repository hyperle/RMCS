#include "../include/dart_state.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::dart {

DartModel::DartModel()
    : attitude_(Eigen::Quaterniond::Identity())
    , angular_velocity_(Eigen::Vector3d::Zero())
    , velocity_(Eigen::Vector3d::Zero())
    , position_(Eigen::Vector3d::Zero())
    , last_update_time_(0)
    , data_fresh_(false) {
    // 初始化为单位四元数
    attitude_.normalize();
}

void DartModel::setAttitude(const Eigen::Quaterniond& quaternion) {
    attitude_ = quaternion;
    validateQuaternion(attitude_);
    data_fresh_ = true;
}

void DartModel::setAttitude(double w, double x, double y, double z) {
    attitude_ = Eigen::Quaterniond(w, x, y, z);
    validateQuaternion(attitude_);
    data_fresh_ = true;
}

Eigen::Quaterniond DartModel::getAttitude() const {
    return attitude_;
}

void DartModel::setAngularVelocity(const Eigen::Vector3d& angular_vel) {
    angular_velocity_ = angular_vel;
    data_fresh_ = true;
}

Eigen::Vector3d DartModel::getAngularVelocity() const {
    return angular_velocity_;
}

void DartModel::setVelocity(const Eigen::Vector3d& velocity) {
    velocity_ = velocity;
    data_fresh_ = true;
}

Eigen::Vector3d DartModel::getVelocity() const {
    return velocity_;
}

void DartModel::setPosition(const Eigen::Vector3d& position) {
    position_ = position;
    data_fresh_ = true;
}

Eigen::Vector3d DartModel::getPosition() const {
    return position_;
}

Eigen::Vector3d DartModel::cameraToWorld(const Eigen::Vector3d& camera_point) const {
    // 相机坐标系 -> 弹体坐标系 -> 世界坐标系
    Eigen::Matrix3d camera_to_body = getCameraToBodyMatrix();
    Eigen::Matrix3d body_to_world = getBodyToWorldMatrix();
    
    Eigen::Vector3d body_point = CoordinateTransformer::transformPoint(
        camera_point, camera_to_body, 
        Eigen::Vector3d(camera_offset_x_, camera_offset_y_, camera_offset_z_));
    
    Eigen::Vector3d world_point = CoordinateTransformer::transformPoint(
        body_point, body_to_world, position_);
    
    return world_point;
}

Eigen::Vector3d DartModel::worldToCamera(const Eigen::Vector3d& world_point) const {
    // 世界坐标系 -> 弹体坐标系 -> 相机坐标系
    Eigen::Matrix3d world_to_body = getBodyToWorldMatrix().transpose();
    Eigen::Matrix3d body_to_camera = getCameraToBodyMatrix().transpose();
    
    Eigen::Vector3d body_point = CoordinateTransformer::transformPoint(
        world_point, world_to_body, -position_);
    
    // 减去相机偏移
    body_point -= Eigen::Vector3d(camera_offset_x_, camera_offset_y_, camera_offset_z_);
    
    Eigen::Vector3d camera_point = CoordinateTransformer::transformPoint(
        body_point, body_to_camera, Eigen::Vector3d::Zero());
    
    return camera_point;
}

Eigen::Vector3d DartModel::cameraToTrajectory(const Eigen::Vector3d& camera_point) const {
    // 相机坐标系 -> 弹体坐标系 -> 弹道坐标系
    Eigen::Matrix3d camera_to_body = getCameraToBodyMatrix();
    Eigen::Matrix3d body_to_trajectory = getTrajectoryToBodyMatrix().transpose();
    
    Eigen::Vector3d body_point = CoordinateTransformer::transformPoint(
        camera_point, camera_to_body, 
        Eigen::Vector3d(camera_offset_x_, camera_offset_y_, camera_offset_z_));
    
    Eigen::Vector3d trajectory_point = CoordinateTransformer::transformVector(
        body_point, body_to_trajectory);
    
    return trajectory_point;
}

Eigen::Vector3d DartModel::trajectoryToCamera(const Eigen::Vector3d& trajectory_point) const {
    // 弹道坐标系 -> 弹体坐标系 -> 相机坐标系
    Eigen::Matrix3d trajectory_to_body = getTrajectoryToBodyMatrix();
    Eigen::Matrix3d body_to_camera = getCameraToBodyMatrix().transpose();
    
    Eigen::Vector3d body_point = CoordinateTransformer::transformVector(
        trajectory_point, trajectory_to_body);
    
    // 减去相机偏移
    body_point -= Eigen::Vector3d(camera_offset_x_, camera_offset_y_, camera_offset_z_);
    
    Eigen::Vector3d camera_point = CoordinateTransformer::transformPoint(
        body_point, body_to_camera, Eigen::Vector3d::Zero());
    
    return camera_point;
}

Eigen::Vector3d DartModel::velocityToWorld(const Eigen::Vector3d& velocity_point) const {
    // 速度坐标系 -> 弹体坐标系 -> 世界坐标系
    Eigen::Matrix3d velocity_to_body = getVelocityToBodyMatrix();
    Eigen::Matrix3d body_to_world = getBodyToWorldMatrix();
    
    Eigen::Vector3d body_point = CoordinateTransformer::transformVector(
        velocity_point, velocity_to_body);
    
    Eigen::Vector3d world_point = CoordinateTransformer::transformVector(
        body_point, body_to_world);
    
    return world_point;
}

Eigen::Vector3d DartModel::worldToVelocity(const Eigen::Vector3d& world_point) const {
    // 世界坐标系 -> 弹体坐标系 -> 速度坐标系
    Eigen::Matrix3d world_to_body = getBodyToWorldMatrix().transpose();
    Eigen::Matrix3d body_to_velocity = getVelocityToBodyMatrix().transpose();
    
    Eigen::Vector3d body_point = CoordinateTransformer::transformVector(
        world_point, world_to_body);
    
    Eigen::Vector3d velocity_point = CoordinateTransformer::transformVector(
        body_point, body_to_velocity);
    
    return velocity_point;
}

void DartModel::updateAttitude(double delta_time) {
    if (delta_time <= 0.0) {
        return;
    }
    
    // 使用角速度积分更新四元数
    Eigen::Vector3d omega = angular_velocity_;
    double omega_norm = omega.norm();
    
    if (omega_norm > 1e-6) {
        Eigen::Quaterniond delta_q;
        double half_theta = 0.5 * omega_norm * delta_time;
        double sin_half_theta = std::sin(half_theta);
        double cos_half_theta = std::cos(half_theta);
        
        delta_q.w() = cos_half_theta;
        delta_q.vec() = (sin_half_theta / omega_norm) * omega;
        
        attitude_ = attitude_ * delta_q;
        validateQuaternion(attitude_);
    }
    
    data_fresh_ = true;
}

Eigen::Vector3d DartModel::getEulerAngles() const {
    // 将四元数转换为欧拉角 (roll, pitch, yaw)
    Eigen::Matrix3d rotation_matrix = attitude_.toRotationMatrix();
    
    double roll = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
    double pitch = std::asin(-rotation_matrix(2, 0));
    double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    
    return {roll, pitch, yaw};
}

bool DartModel::isAttitudeValid() const {
    return isQuaternionValid(attitude_);
}

bool DartModel::isDataFresh() const {
    return data_fresh_;
}

void DartModel::markDataStale() {
    data_fresh_ = false;
}

// 私有方法实现
Eigen::Matrix3d DartModel::getCameraToBodyMatrix() const {
    // 相机坐标系到弹体坐标系的旋转矩阵
    // 假设相机与弹体坐标系对齐，可能有小的安装误差
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d DartModel::getBodyToWorldMatrix() const {
    // 弹体坐标系到世界坐标系的旋转矩阵
    return attitude_.toRotationMatrix();
}

Eigen::Matrix3d DartModel::getVelocityToBodyMatrix() const {
    // 速度坐标系到弹体坐标系的旋转矩阵
    // 速度坐标系X轴沿速度方向，需要根据速度矢量计算
    if (velocity_.norm() < 1e-6) {
        return Eigen::Matrix3d::Identity();
    }
    
    Eigen::Vector3d x_axis = velocity_.normalized(); // 速度方向
    Eigen::Vector3d z_axis = x_axis.cross(Eigen::Vector3d::UnitY());
    if (z_axis.norm() < 1e-6) {
        z_axis = x_axis.cross(Eigen::Vector3d::UnitZ());
    }
    z_axis.normalize();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();
    
    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    
    return rotation;
}

Eigen::Matrix3d DartModel::getTrajectoryToBodyMatrix() const {
    // 弹道坐标系到弹体坐标系的旋转矩阵
    // 弹道坐标系通常与速度坐标系相同或相关
    // 这里简化处理，与速度坐标系相同
    return getVelocityToBodyMatrix();
}

void DartModel::validateQuaternion(Eigen::Quaterniond& quat) const {
    // 确保四元数归一化
    if (std::abs(quat.norm() - 1.0) > 1e-6) {
        quat.normalize();
    }
}

bool DartModel::isQuaternionValid(const Eigen::Quaterniond& quat) const {
    // 检查四元数有效性
    if (std::isnan(quat.w()) || std::isnan(quat.x()) || 
        std::isnan(quat.y()) || std::isnan(quat.z())) {
        return false;
    }
    
    double norm = quat.norm();
    return (std::abs(norm - 1.0) < 1e-6);
}

// CoordinateTransformer 实现
Eigen::Vector3d CoordinateTransformer::transformPoint(
    const Eigen::Vector3d& point,
    const Eigen::Matrix3d& rotation_matrix,
    const Eigen::Vector3d& translation) {
    
    return rotation_matrix * point + translation;
}

Eigen::Vector3d CoordinateTransformer::transformVector(
    const Eigen::Vector3d& vector,
    const Eigen::Matrix3d& rotation_matrix) {
    
    return rotation_matrix * vector;
}

Eigen::Matrix3d CoordinateTransformer::eulerToMatrix(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    
    Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
    return q.toRotationMatrix();
}

Eigen::Quaterniond CoordinateTransformer::matrixToQuaternion(const Eigen::Matrix3d& matrix) {
    return Eigen::Quaterniond(matrix);
}

bool CoordinateTransformer::isRotationMatrixValid(const Eigen::Matrix3d& matrix) {
    // 检查矩阵是否为旋转矩阵（正交且行列式为1）
    Eigen::Matrix3d should_be_identity = matrix * matrix.transpose();
    bool orthogonal = should_be_identity.isApprox(Eigen::Matrix3d::Identity(), 1e-6);
    bool proper_rotation = std::abs(matrix.determinant() - 1.0) < 1e-6;
    
    return orthogonal && proper_rotation;
}

} // namespace rmcs_core::dart