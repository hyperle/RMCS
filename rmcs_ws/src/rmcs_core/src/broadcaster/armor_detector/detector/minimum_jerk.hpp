#ifndef MINIMUM_JERK_TRAJECTORY_HPP
#define MINIMUM_JERK_TRAJECTORY_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>

namespace rmcs_core::controller {

struct TrajectoryPoint {
    double x, y;
    double vx, vy;
    double ax, ay;
    double time;
    
    TrajectoryPoint(double x = 0, double y = 0, double vx = 0, double vy = 0, 
                   double ax = 0, double ay = 0, double time = 0)
        : x(x), y(y), vx(vx), vy(vy), ax(ax), ay(ay), time(time) {}
};

struct Polynomial {
    double a, b, c, d, e; // 系数: a*t⁴ + b*t³ + c*t² + d*t + e
    double duration;
    
    Polynomial() : a(0), b(0), c(0), d(0), e(0), duration(0) {}
    
    double position(double t) const {
        return a * t*t*t*t + b * t*t*t + c * t*t + d * t + e;
    }
    
    double velocity(double t) const {
        return 4*a * t*t*t + 3*b * t*t + 2*c * t + d;
    }
    
    double acceleration(double t) const {
        return 12*a * t*t + 6*b * t + 2*c;
    }
    
    double jerk(double t) const {
        return 24*a * t + 6*b;
    }
};

class MinimumJerkTrajectory {
private:
    std::vector<std::vector<double>> waypoints_;
    std::vector<Polynomial> x_trajectories_;
    std::vector<Polynomial> y_trajectories_;
    double max_velocity_, max_acceleration_;
    bool trajectory_planned_;
    double total_duration_;

public:
    MinimumJerkTrajectory(double max_vel = 1.0, double max_acc = 1.0)
        : max_velocity_(max_vel), max_acceleration_(max_acc), 
          trajectory_planned_(false), total_duration_(0) {}

    void setWaypoints(const std::vector<std::vector<double>>& points) {
        waypoints_ = points;
        trajectory_planned_ = false;
    }

    void setConstraints(double max_vel, double max_acc) {
        max_velocity_ = max_vel;
        max_acceleration_ = max_acc;
        trajectory_planned_ = false;
    }

    Polynomial calculateSegment(double start_pos, double end_pos, 
                               double start_vel, double end_vel,
                               double start_acc, double end_acc, double T) {
        Polynomial poly;
        poly.duration = T;
        
        if (T <= 0) return poly;
        
        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;
        
        double pos_diff = end_pos - start_pos;
        
        poly.a = (6*pos_diff - 3*(start_vel + end_vel)*T + 0.5*(start_acc - end_acc)*T2) / T5;
        poly.b = (-15*pos_diff + (7*end_vel + 8*start_vel)*T + (1.5*end_acc - 3*start_acc)*T2) / T4;
        poly.c = (10*pos_diff - (4*end_vel + 6*start_vel)*T + (1.5*start_acc - 0.5*end_acc)*T2) / T3;
        poly.d = start_vel;
        poly.e = start_pos;
        
        return poly;
    }

    bool checkConstraints(const Polynomial& poly_x, const Polynomial& poly_y, 
                         int samples = 100) {
        double dt = poly_x.duration / samples;
        
        for (int i = 0; i <= samples; ++i) {
            double t = i * dt;
            
            double vx = poly_x.velocity(t);
            double vy = poly_y.velocity(t);
            double velocity = std::sqrt(vx*vx + vy*vy);
            
            double ax = poly_x.acceleration(t);
            double ay = poly_y.acceleration(t);
            double acceleration = std::sqrt(ax*ax + ay*ay);
            
            if (velocity > max_velocity_ || acceleration > max_acceleration_) {
                return false;
            }
        }
        return true;
    }

    double optimizeSegmentTime(double start_x, double start_y, double end_x, double end_y,
                             double start_vel_x, double start_vel_y,
                             double end_vel_x, double end_vel_y,
                             double start_acc_x, double start_acc_y,
                             double end_acc_x, double end_acc_y) {
        
        double min_time = 0.1;
        double max_time = 10.0;
        double best_time = max_time;
        
        for (int iter = 0; iter < 20; ++iter) {
            double mid_time = (min_time + max_time) / 2.0;
            
            Polynomial poly_x = calculateSegment(start_x, end_x, start_vel_x, end_vel_x, 
                                               start_acc_x, end_acc_x, mid_time);
            Polynomial poly_y = calculateSegment(start_y, end_y, start_vel_y, end_vel_y, 
                                               start_acc_y, end_acc_y, mid_time);
            
            if (checkConstraints(poly_x, poly_y)) {
                best_time = mid_time;
                max_time = mid_time;
            } else {
                min_time = mid_time;
            }
        }
        
        return best_time;
    }

    bool planTrajectory() {
        if (waypoints_.size() < 2) {
            trajectory_planned_ = false;
            return false;
        }
        
        x_trajectories_.clear();
        y_trajectories_.clear();
        total_duration_ = 0;
        
        double start_vel_x = 0, start_vel_y = 0;
        double start_acc_x = 0, start_acc_y = 0;
        
        for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
            const auto& start = waypoints_[i];
            const auto& end = waypoints_[i + 1];
            
            double end_vel_x = 0, end_vel_y = 0;
            double end_acc_x = 0, end_acc_y = 0;
            
            if (i < waypoints_.size() - 2) {
                const auto& next = waypoints_[i + 2];
                double dx = next[0] - end[0];
                double dy = next[1] - end[1];
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist > 0) {
                    end_vel_x = dx / dist * 0.3;
                    end_vel_y = dy / dist * 0.3;
                }
            }
            
            double optimal_time = optimizeSegmentTime(start[0], start[1], end[0], end[1],
                                                    start_vel_x, start_vel_y,
                                                    end_vel_x, end_vel_y,
                                                    start_acc_x, start_acc_y,
                                                    end_acc_x, end_acc_y);
            
            Polynomial poly_x = calculateSegment(start[0], end[0], start_vel_x, end_vel_x,
                                               start_acc_x, end_acc_x, optimal_time);
            Polynomial poly_y = calculateSegment(start[1], end[1], start_vel_y, end_vel_y,
                                               start_acc_y, end_acc_y, optimal_time);
            
            x_trajectories_.push_back(poly_x);
            y_trajectories_.push_back(poly_y);
            total_duration_ += optimal_time;
            
            start_vel_x = poly_x.velocity(optimal_time);
            start_vel_y = poly_y.velocity(optimal_time);
            start_acc_x = poly_x.acceleration(optimal_time);
            start_acc_y = poly_y.acceleration(optimal_time);
        }
        
        trajectory_planned_ = true;
        return true;
    }

    TrajectoryPoint getTrajectoryPoint(double global_time) {
        if (!trajectory_planned_ || x_trajectories_.empty()) {
            return {};
        }
        
        double accumulated_time = 0;
        
        for (size_t i = 0; i < x_trajectories_.size(); ++i) {
            double segment_time = x_trajectories_[i].duration;
            
            if (global_time <= accumulated_time + segment_time) {
                double local_time = global_time - accumulated_time;
                
                double x = x_trajectories_[i].position(local_time);
                double y = y_trajectories_[i].position(local_time);
                double vx = x_trajectories_[i].velocity(local_time);
                double vy = y_trajectories_[i].velocity(local_time);
                double ax = x_trajectories_[i].acceleration(local_time);
                double ay = y_trajectories_[i].acceleration(local_time);
                
                return {x, y, vx, vy, ax, ay, global_time};
            }
            accumulated_time += segment_time;
        }
        
        // 返回最后一个点
        const auto& last_wp = waypoints_.back();
        return {last_wp[0], last_wp[1], 0, 0, 0, 0, global_time};
    }

    // 获取采样轨迹点用于可视化
    std::vector<TrajectoryPoint> getSampledTrajectory(int samples_per_segment = 50) {
        std::vector<TrajectoryPoint> sampled_points;
        
        if (!trajectory_planned_) return sampled_points;
        
        for (size_t i = 0; i < x_trajectories_.size(); ++i) {
            double segment_duration = x_trajectories_[i].duration;
            double dt = segment_duration / samples_per_segment;
            
            double segment_start_time = 0;
            for (size_t j = 0; j < i; ++j) {
                segment_start_time += x_trajectories_[j].duration;
            }
            
            for (int k = 0; k <= samples_per_segment; ++k) {
                double local_time = k * dt;
                double global_time = segment_start_time + local_time;
                
                double x = x_trajectories_[i].position(local_time);
                double y = y_trajectories_[i].position(local_time);
                double vx = x_trajectories_[i].velocity(local_time);
                double vy = y_trajectories_[i].velocity(local_time);
                double ax = x_trajectories_[i].acceleration(local_time);
                double ay = y_trajectories_[i].acceleration(local_time);
                
                sampled_points.emplace_back(x, y, vx, vy, ax, ay, global_time);
            }
        }
        
        return sampled_points;
    }

    bool isTrajectoryPlanned() const { return trajectory_planned_; }
    double getTotalDuration() const { return total_duration_; }
    const std::vector<std::vector<double>>& getWaypoints() const { return waypoints_; }
};

} // namespace rmcs_core::controller

#endif // MINIMUM_JERK_TRAJECTORY_HPP