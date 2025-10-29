#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <array>
#include <cmath>

namespace rmcs_core::dart {

class DartModel {
public:
    DartModel();
    ~DartModel() = default;

    DartModel(const DartModel&) = delete;
    DartModel& operator=(const DartModel&) = delete;

    // 四元数操作接口
    void setAttitude(const Eigen::Quaterniond& quaternion);
    void setAttitude(double w, double x, double y, double z);
    Eigen::Quaterniond getAttitude() const;
    
    // 角速度操作接口
    void setAngularVelocity(const Eigen::Vector3d& angular_vel);
    Eigen::Vector3d getAngularVelocity() const;
    
    // 速度操作接口
    void setVelocity(const Eigen::Vector3d& velocity);
    Eigen::Vector3d getVelocity() const;
    
    // 位置操作接口（相对发射点）
    void setPosition(const Eigen::Vector3d& position);
    Eigen::Vector3d getPosition() const;

    Eigen::Vector3d cameraToWorld(const Eigen::Vector3d& camera_point) const;
    Eigen::Vector3d worldToCamera(const Eigen::Vector3d& world_point) const;
    Eigen::Vector3d cameraToTrajectory(const Eigen::Vector3d& camera_point) const;
    Eigen::Vector3d trajectoryToCamera(const Eigen::Vector3d& trajectory_point) const;
    Eigen::Vector3d velocityToWorld(const Eigen::Vector3d& velocity_point) const;
    Eigen::Vector3d worldToVelocity(const Eigen::Vector3d& world_point) const;
    
    void updateAttitude(double delta_time);
    
    Eigen::Vector3d getEulerAngles() const;
    
    bool isAttitudeValid() const;
    bool isDataFresh() const;
    void markDataStale();

private:
    Eigen::Matrix3d getCameraToBodyMatrix() const;
    Eigen::Matrix3d getBodyToWorldMatrix() const;
    Eigen::Matrix3d getVelocityToBodyMatrix() const;
    Eigen::Matrix3d getTrajectoryToBodyMatrix() const;
    
    void validateQuaternion(Eigen::Quaterniond& quat) const;
    bool isQuaternionValid(const Eigen::Quaterniond& quat) const;

    Eigen::Quaterniond attitude_;          // 姿态四元数 (世界坐标系)
    Eigen::Vector3d angular_velocity_;     // 角速度 (弹体坐标系)
    Eigen::Vector3d velocity_;             // 速度矢量 (世界坐标系)
    Eigen::Vector3d position_;             // 位置 (世界坐标系，相对发射点)
    
    uint64_t last_update_time_;
    bool data_fresh_;
    
    static constexpr double camera_offset_x_ = 0.02;    // 相机在弹体坐标系中的X偏移
    static constexpr double camera_offset_y_ = 0.0;     // 相机在弹体坐标系中的Y偏移  
    static constexpr double camera_offset_z_ = 0.0;     // 相机在弹体坐标系中的Z偏移
    
    static constexpr size_t COORDINATE_SYSTEM_COUNT = 4;
    enum CoordinateSystem {
        WORLD = 0,      // 世界坐标系：东北天(ENU)或北东地(NED)
        BODY = 1,       // 弹体坐标系：前右下(Forward-Right-Down)
        CAMERA = 2,     // 相机坐标系：前右下，原点在相机光心
        TRAJECTORY = 3  // 弹道坐标系：速度方向为X轴
    };
};

// 坐标转换工具类
class CoordinateTransformer {
public:
    CoordinateTransformer() = delete; // 纯静态工具类
    
    static Eigen::Vector3d transformPoint(
        const Eigen::Vector3d& point,
        const Eigen::Matrix3d& rotation_matrix,
        const Eigen::Vector3d& translation);
        
    static Eigen::Vector3d transformVector(
        const Eigen::Vector3d& vector,
        const Eigen::Matrix3d& rotation_matrix);
        
    static Eigen::Matrix3d eulerToMatrix(double roll, double pitch, double yaw);
    static Eigen::Quaterniond matrixToQuaternion(const Eigen::Matrix3d& matrix);
    
    static bool isRotationMatrixValid(const Eigen::Matrix3d& matrix);
};

} // namespace rmcs_core::dart