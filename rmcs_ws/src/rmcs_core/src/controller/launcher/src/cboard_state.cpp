/*
#include "hardware/device/bmi088.hpp"
#include "rclcpp/rclcpp.hpp"
#include "librmcs/device/bmi088.hpp"
#include <deque>
#include <numeric>

class AdvancedBmi088Node : public rclcpp::Node
{
public:
    AdvancedBmi088Node() : Node("advanced_bmi088_node")
    {
        // 参数配置
        sample_freq_ = this->declare_parameter<double>("sample_frequency", 100.0);
        bool enable_raw_stats = this->declare_parameter<bool>("enable_raw_stats", true);
        int stats_window = this->declare_parameter<int>("stats_window", 50); // 统计窗口大小
        
        // 初始化传感器
        bmi088_ = std::make_unique<librmcs::device::Bmi088>(
            sample_freq_, 
            this->declare_parameter<double>("kp", 0.5),
            this->declare_parameter<double>("ki", 0.0)
        );
        
        // 主更新定时器
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(static_cast<int>(1000000.0 / sample_freq_)),
            [this] { updateAndAnalyzeData(); });
        
        // 数据统计定时器
        if (enable_raw_stats) {
            stats_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),  // 10Hz统计
                [this, stats_window]() { this->printAccelerationStats(stats_window); });
        }
        
        RCLCPP_INFO(this->get_logger(), "高级BMI088节点已启动");
    }

private:
    void updateAndAnalyzeData()
    {
        simulateSensorData();
        bmi088_->update_status();
        storeAccelerationHistory();
    }

    void storeAccelerationHistory()
    {
        // 存储加速度历史数据用于统计
        AccelerationData data;
        data.raw = accelerometer_data_snapshot_;
        data.processed = {bmi088_->ax(), bmi088_->ay(), bmi088_->az()};
        data.timestamp = this->now();
        
        acceleration_history_.push_back(data);
        
        // 保持历史数据在窗口大小内
        if (acceleration_history_.size() > max_history_size_) {
            acceleration_history_.pop_front();
        }
    }

    void printAccelerationStats(int window_size)
    {
        if (acceleration_history_.size() < 2) return;
        
        // 计算最近window_size个数据的统计信息
        int actual_window = std::min(static_cast<int>(acceleration_history_.size()), window_size);
        auto recent_data = std::vector<AccelerationData>(
            acceleration_history_.end() - actual_window, 
            acceleration_history_.end()
        );
        
        // 计算原始数据的统计
        auto raw_stats = calculateAccelerationStats(recent_data, true);
        auto processed_stats = calculateAccelerationStats(recent_data, false);
        
        // 计算数据变化率
        double time_span = (recent_data.back().timestamp - recent_data.front().timestamp).seconds();
        double data_rate = actual_window / time_span;
        
        RCLCPP_INFO(this->get_logger(),
            "加速度统计 (最近%d样本, %.1fHz) | "
            "原始数据 - 均值: [%.3f, %.3f, %.3f]g, 标准差: [%.3f, %.3f, %.3f] | "
            "处理数据 - 均值: [%.3f, %.3f, %.3f]g, 标准差: [%.3f, %.3f, %.3f]",
            actual_window, data_rate,
            raw_stats.mean_x, raw_stats.mean_y, raw_stats.mean_z,
            raw_stats.std_x, raw_stats.std_y, raw_stats.std_z,
            processed_stats.mean_x, processed_stats.mean_y, processed_stats.mean_z,
            processed_stats.std_x, processed_stats.std_y, processed_stats.std_z);
    }

    struct AccelerationStats {
        double mean_x, mean_y, mean_z;
        double std_x, std_y, std_z;
    };

    AccelerationStats calculateAccelerationStats(const std::vector<AccelerationData>& data, bool use_raw)
    {
        AccelerationStats stats = {};
        int count = data.size();
        if (count == 0) return stats;
        
        // 计算均值
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& d : data) {
            if (use_raw) {
                sum_x += d.raw.x / 32767.0 * 6.0;
                sum_y += d.raw.y / 32767.0 * 6.0;
                sum_z += d.raw.z / 32767.0 * 6.0;
            } else {
                sum_x += d.processed.x;
                sum_y += d.processed.y;
                sum_z += d.processed.z;
            }
        }
        stats.mean_x = sum_x / count;
        stats.mean_y = sum_y / count;
        stats.mean_z = sum_z / count;
        
        // 计算标准差
        double sum_var_x = 0, sum_var_y = 0, sum_var_z = 0;
        for (const auto& d : data) {
            double dx, dy, dz;
            if (use_raw) {
                dx = d.raw.x / 32767.0 * 6.0 - stats.mean_x;
                dy = d.raw.y / 32767.0 * 6.0 - stats.mean_y;
                dz = d.raw.z / 32767.0 * 6.0 - stats.mean_z;
            } else {
                dx = d.processed.x - stats.mean_x;
                dy = d.processed.y - stats.mean_y;
                dz = d.processed.z - stats.mean_z;
            }
            sum_var_x += dx * dx;
            sum_var_y += dy * dy;
            sum_var_z += dz * dz;
        }
        stats.std_x = std::sqrt(sum_var_x / count);
        stats.std_y = std::sqrt(sum_var_y / count);
        stats.std_z = std::sqrt(sum_var_z / count);
        
        return stats;
    }

    void simulateSensorData()
    {
        // 同前的模拟数据生成代码
        static double time = 0.0;
        
        int16_t ax_raw = 8192 + static_cast<int16_t>(1000 * std::sin(time * 2 * M_PI * 2.0));
        int16_t ay_raw = 1024 + static_cast<int16_t>(800 * std::sin(time * 2 * M_PI * 1.5));  
        int16_t az_raw = 24576 + static_cast<int16_t>(1200 * std::sin(time * 2 * M_PI * 3.0));
        
        int16_t gx_raw = 100 + static_cast<int16_t>(50 * std::sin(time * 2 * M_PI * 0.5));
        int16_t gy_raw = -200 + static_cast<int16_t>(80 * std::sin(time * 2 * M_PI * 1.2));
        int16_t gz_raw = 150 + static_cast<int16_t>(60 * std::sin(time * 2 * M_PI * 0.8));
        
        bmi088_->store_accelerometer_status(ax_raw, ay_raw, az_raw);
        bmi088_->store_gyroscope_status(gx_raw, gy_raw, gz_raw);
        accelerometer_data_snapshot_ = {ax_raw, ay_raw, az_raw};
        
        time += 1.0 / sample_freq_;
    }

    std::unique_ptr<librmcs::device::Bmi088> bmi088_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    double sample_freq_;
    
    struct AccelerationData {
        RawAccelerationData raw;
        struct { double x, y, z; } processed;
        rclcpp::Time timestamp;
    };
    
    struct RawAccelerationData {
        int16_t x, y, z;
    };
    RawAccelerationData accelerometer_data_snapshot_;
    
    std::deque<AccelerationData> acceleration_history_;
    const size_t max_history_size_ = 1000; // 最大历史数据点数
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdvancedBmi088Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/