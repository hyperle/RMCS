/*
//#include <cv_bridge/cv_bridge.h>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>

namespace rmcs_core::armor_detector {

struct LightBar {
    cv::Point2f top_point;    
    cv::Point2f bottom_point; 
    std::string color;
    float length;
    float angle; 
    float pixel_length; 
};

struct ArmorPlate {
    cv::Point2f corners[4];
    std::string color;
    float yaw;
    float pitch;
    float roll;
    float distance;
};

struct ColorRange {
    int lower[3];
    int upper[3];
    std::string name;
};

class GetCameraFrame
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GetCameraFrame()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        camera_profile_.invert_image = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());

        image_capturer_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        // 创建图像发布器
        //image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        //    get_parameter("image_topic_name").as_string(), 10);

        // 相机内参 && 畸变系数
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            1000.0, 0.0, 320.0,
            0.0, 1000.0, 240.0,
            0.0, 0.0, 1.0);
        dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        armor_real_width_ = 0.14;
        armor_real_height_ = 0.125;
        lightbar_real_height_ = 0.06;

        loadColorRanges();

        binary_threshold_ = get_parameter("binary_threshold").as_int();
        min_contour_area_ = get_parameter("min_contour_area").as_int();
        //min_aspect_ratio_ = get_parameter("min_aspect_ratio").as_double();
        //max_aspect_ratio_ = get_parameter("max_aspect_ratio").as_double();
        //max_angle_diff_ = get_parameter("max_angle_diff").as_double();
        //min_color_saturation_ = get_parameter("min_color_saturation").as_int();
        //min_color_value_ = get_parameter("min_color_value").as_int();

        //RCLCPP_INFO(logger_, "Armor detector initialized");
        //RCLCPP_INFO(logger_, "Armor size: %.3f x %.3f m", armor_real_width_, armor_real_height_);
        RCLCPP_INFO(logger_, "Binary threshold: %d", binary_threshold_);
        RCLCPP_INFO(logger_, "min_contour_area: %d", min_contour_area_);
        //RCLCPP_INFO(logger_, "min_aspect_ratio: %f", min_aspect_ratio_);
        //RCLCPP_INFO(logger_, "max_aspect_ratio: %f", max_aspect_ratio_);
        //RCLCPP_INFO(logger_, "max_angle_diff: %f", max_angle_diff_);
        //RCLCPP_INFO(logger_, "min_color_saturation: %d", min_color_saturation_);
        //RCLCPP_INFO(logger_, "min_color_value: %d", min_color_value_);
        for (const auto& color_range : color_ranges_) {
            RCLCPP_INFO(logger_, "%s HSV: (%d,%d,%d) to (%d,%d,%d)", 
                       color_range.name.c_str(),
                       color_range.lower_h, color_range.lower_s, color_range.lower_v,
                       color_range.upper_h, color_range.upper_s, color_range.upper_v);
        }

        camera_thread_ = std::thread(&GetCameraFrame::camera_frame_update, this);
    }

    ~GetCameraFrame() {
        if (camera_thread_.joinable()) {
            camera_thread_.join();
        }
    }

    void update() override {
    }

private:
void loadColorRanges() {
        ColorRange red_range;
        red_range.name = "red";
        red_range.lower_h = get_parameter("red_lower_h").as_int();
        red_range.lower_s = get_parameter("red_lower_s").as_int();
        red_range.lower_v = get_parameter("red_lower_v").as_int();
        red_range.upper_h = get_parameter("red_upper_h").as_int();
        red_range.upper_s = get_parameter("red_upper_s").as_int();
        red_range.upper_v = get_parameter("red_upper_v").as_int();
        color_ranges_.push_back(red_range);

        ColorRange blue_range;
        blue_range.name = "blue";
        blue_range.lower_h = get_parameter("blue_lower_h").as_int();
        blue_range.lower_s = get_parameter("blue_lower_s").as_int();
        blue_range.lower_v = get_parameter("blue_lower_v").as_int();
        blue_range.upper_h = get_parameter("blue_upper_h").as_int();
        blue_range.upper_s = get_parameter("blue_upper_s").as_int();
        blue_range.upper_v = get_parameter("blue_upper_v").as_int();
        color_ranges_.push_back(blue_range);
    }

    void camera_frame_update() {
        cv::Mat display_hsv;

        while (rclcpp::ok()) {
            cv::Mat camera_frame = image_capturer_->read();
            if (camera_frame.empty()) {
                RCLCPP_WARN(logger_, "Empty frame received");
                continue;
            }

            cv::cvtColor(camera_frame, display_hsv, cv::COLOR_BGR2HSV);
            cv::Mat color_mask = createColorMask(display_hsv);

            // 装甲板检测与识别
            std::vector<ArmorPlate> armors = detectArmors(camera_frame,display_hsv);
            drawResults(camera_frame, armors);

            // 发布处理后的图像
            //publishImage(camera_frame);

            cv::imshow("HSV Image", display_hsv);
            if (cv::waitKey(1) == 27) { // ESC键退出
                break;
            }
        }
    }

    cv::Mat createColorMask(const cv::Mat& hsv_image) {
        cv::Mat mask = cv::Mat::zeros(hsv_image.size(), CV_8UC1);
        
        for (const auto& color_range : color_ranges_) {
            cv::Mat color_mask;
            cv::inRange(hsv_image, 
                       cv::Scalar(color_range.lower_h, color_range.lower_s, color_range.lower_v),
                       cv::Scalar(color_range.upper_h, color_range.upper_s, color_range.upper_v),
                       color_mask);
            
            cv::bitwise_or(mask, color_mask, mask);
        }

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        
        return mask;
    }

    std::vector<ArmorPlate> detectArmors(cv::Mat& image, cv::Mat& hsv_image) {
        std::vector<ArmorPlate> armors;
        std::vector<LightBar> light_bars = detectLightBars(image, hsv_image);
        armors = pairLightBars(light_bars);
        for (auto& armor : armors) {
            calculatePose(armor);
        }
        
        return armors;
    }

    std::vector<LightBar> detectLightBars(cv::Mat& image, cv::Mat& hsv_image) {
        std::vector<LightBar> light_bars;
        cv::Mat color_mask = createColorMask(hsv_image);
        // 转换为灰度图
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        
        // 二值化
        cv::Mat binary;
        cv::threshold(gray, binary, binary_threshold_, 255, cv::THRESH_BINARY);
        
        // 形态学操作，连接相邻区域
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
                
        // 转换为HSV颜色空间用于颜色识别
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(color_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // 处理每个轮廓
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) < min_contour_area_) continue;
            cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
            
            float width = rotated_rect.size.width;
            float height = rotated_rect.size.height;
            //float aspect_ratio = std::max(width, height) / std::min(width, height);
            //if (aspect_ratio < min_aspect_ratio_ || aspect_ratio > max_aspect_ratio_) continue;
            // 获取旋转矩形的四个顶点
            cv::Point2f vertices[4];
            rotated_rect.points(vertices);
            
            // 找到y坐标最小和最大的点作为灯条的顶部和底部
            cv::Point2f top_point = vertices[0];
            cv::Point2f bottom_point = vertices[0];
            
            for (int i = 1; i < 4; i++) {
                if (vertices[i].y < top_point.y) top_point = vertices[i];
                if (vertices[i].y > bottom_point.y) bottom_point = vertices[i];
            }
            
            // 计算灯条长度
            float length = cv::norm(top_point - bottom_point);
            float pixel_length = cv::norm(top_point - bottom_point);
            
            // 计算灯条角度（与垂直方向的夹角）
            //float dx = bottom_point.x - top_point.x;
            //float dy = bottom_point.y - top_point.y;
            //float angle = std::atan2(dx, dy) * 180 / CV_PI;
            // 过滤角度过大的灯条（非垂直方向）
            //if (std::abs(angle) > 30) continue;
            // 获取灯条中心点
            cv::Point2f center = (top_point + bottom_point) / 2;
            // 在中心点周围取一个小区域分析颜色
            int roi_size = 10;
            cv::Rect roi(center.x - roi_size/2, center.y - roi_size/2, roi_size, roi_size);
            roi &= cv::Rect(0, 0, image.cols, image.rows);
            
            if (roi.width <= 0 || roi.height <= 0) continue;
            
            // 提取ROI区域
            cv::Mat color_roi = hsv(roi);
            
            // 计算平均HSV值
            cv::Scalar mean_hsv = cv::mean(color_roi);
            
            // 提取H、S、V分量
            float H = mean_hsv[0];
            float S = mean_hsv[1];
            float V = mean_hsv[2];
            
            // 颜色识别
            std::string color;
            if ((H <= 10 || H >= 170) && S > min_color_saturation_ && V > min_color_value_) {
                color = "RED";
            } else if (H >= 100 && H <= 140 && S > min_color_saturation_ && V > min_color_value_) {
                color = "BLUE";
            } else {
                continue; // 不是红蓝灯条，跳过
            }

            std::string color = detectColorAtPoint(hsv_image, center);
            if (color.empty()) continue; 
            
            // 创建灯条对象
            LightBar light_bar;
            light_bar.top_point = top_point;
            light_bar.bottom_point = bottom_point;
            light_bar.color = color;
            light_bar.length = estimateDistanceFromLightBar(pixel_length);
            light_bar.pixel_length = pixel_length;
            //light_bar.angle = angle;
            
            light_bars.push_back(light_bar);
        }
        
        return light_bars;
    }

    float estimateDistanceFromLightBar(float pixel_length) {
        double fx = camera_matrix_.at<double>(0, 0);
        
        return (fx * lightbar_real_height_) / pixel_length;
    }

    std::string detectColorAtPoint(const cv::Mat& hsv_image, const cv::Point2f& point) {
        // 在点周围取一个小区域分析颜色
        int roi_size = 10;
        cv::Rect roi(point.x - roi_size/2, point.y - roi_size/2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, hsv_image.cols, hsv_image.rows);
        
        if (roi.width <= 0 || roi.height <= 0) return "";
        
        cv::Mat roi_hsv = hsv_image(roi);
        
        std::map<std::string, int> color_counts;
        
        for (const auto& color_range : color_ranges_) {
            cv::Mat mask;
            cv::inRange(roi_hsv, 
                       cv::Scalar(color_range.lower_h, color_range.lower_s, color_range.lower_v),
                       cv::Scalar(color_range.upper_h, color_range.upper_s, color_range.upper_v),
                       mask);
            
            color_counts[color_range.name] = cv::countNonZero(mask);
        }
        
        auto max_color = std::max_element(color_counts.begin(), color_counts.end(),
            [](const std::pair<std::string, int>& a, const std::pair<std::string, int>& b) {
                return a.second < b.second;
            });
        
        if (max_color->second < 5) return "";
        
        return max_color->first;
    }

    std::vector<ArmorPlate> pairLightBars(const std::vector<LightBar>& light_bars) {
        std::vector<ArmorPlate> armors;
        
        std::map<std::string, std::vector<LightBar>> color_groups;
        for (const auto& bar : light_bars) {
            color_groups[bar.color].push_back(bar);
        }
        
        for (const auto& group : color_groups) {
            auto group_armors = pairBarsOfSameColor(group.second);
            armors.insert(armors.end(), group_armors.begin(), group_armors.end());
        }
        
        return armors;
    }

    std::vector<ArmorPlate> pairBarsOfSameColor(const std::vector<LightBar>& bars) {
        std::vector<ArmorPlate> armors;
        
        if (bars.size() < 2) return armors;
        
        for (size_t i = 0; i < bars.size() - 1; i++) {
            for (size_t j = i + 1; j < bars.size(); j++) {
                const LightBar& bar1 = bars[i];
                const LightBar& bar2 = bars[j];
                float angle_diff = std::abs(bar1.angle - bar2.angle);
                if (angle_diff > max_angle_diff_) continue;
                
                // 2. 长度比在合理范围内
                float length_ratio = std::max(bar1.length, bar2.length) / 
                                    std::min(bar1.length, bar2.length);
                if (length_ratio < min_length_ratio_ || length_ratio > max_length_ratio_) continue;
                
                // 3. 距离与平均长度的比例在合理范围内
                cv::Point2f center1 = (bar1.top_point + bar1.bottom_point) / 2;
                cv::Point2f center2 = (bar2.top_point + bar2.bottom_point) / 2;
                float distance = cv::norm(center1 - center2);
                float avg_length = (bar1.pixel_length + bar2.pixel_length) / 2.0f;
                
                if (distance > avg_length * max_distance_ratio_) continue;
                // 确定左右灯条
                const LightBar* left_bar = &bar1;
                const LightBar* right_bar = &bar2;
                if (center1.x > center2.x) {
                    left_bar = &bar2;
                    right_bar = &bar1;
                }
                
                // 配对成功，创建装甲板
                ArmorPlate armor;
                armor.color = bar1.color;
                
                // 确定装甲板的四个角点
                armor.corners[0] = left_bar->top_point;     // 左上
                armor.corners[1] = right_bar->top_point;    // 右上
                armor.corners[2] = right_bar->bottom_point; // 右下
                armor.corners[3] = left_bar->bottom_point;  // 左下

                armor.distance = (bar1.length + bar2.length) / 2.0f;
                armors.push_back(armor);
            }
        }
        
        return armors;
    }

    void calculatePose(ArmorPlate& armor) {
        // 装甲板的3D模型点（世界坐标）
        std::vector<cv::Point3f> object_points;
        float half_width = armor_real_width_ / 2.0f;
        float half_height = armor_real_height_ / 2.0f;
        
        object_points.emplace_back(-half_width, -half_height, 0); // 左上
        object_points.emplace_back(half_width, -half_height, 0);  // 右上
        object_points.emplace_back(half_width, half_height, 0);   // 右下
        object_points.emplace_back(-half_width, half_height, 0);  // 左下
        
        // 图像点
        std::vector<cv::Point2f> image_points;
        for (int i = 0; i < 4; i++) {
            image_points.push_back(armor.corners[i]);
        }
        
        // 使用PnP求解位姿
        cv::Mat rvec, tvec;
        cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec);
        
        // 计算距离
        armor.distance = cv::norm(tvec);
        
        // 将旋转向量转换为旋转矩阵
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        float sy = std::sqrt(rotation_matrix.at<double>(0,0) * rotation_matrix.at<double>(0,0) + 
                            rotation_matrix.at<double>(1,0) * rotation_matrix.at<double>(1,0));
        
        bool singular = sy < 1e-6;
        
        if (!singular) {
            armor.yaw = std::atan2(rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));
            armor.pitch = std::atan2(-rotation_matrix.at<double>(2,0), sy);
            armor.roll = std::atan2(rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(0,0));
        } else {
            armor.yaw = std::atan2(-rotation_matrix.at<double>(1,2), rotation_matrix.at<double>(1,1));
            armor.pitch = std::atan2(-rotation_matrix.at<double>(2,0), sy);
            armor.roll = 0;
        }
        
        // 转换为角度
        armor.yaw = armor.yaw * 180 / CV_PI;
        armor.pitch = armor.pitch * 180 / CV_PI;
        armor.roll = armor.roll * 180 / CV_PI;
    }

    void drawResults(cv::Mat& image, const std::vector<ArmorPlate>& armors) {
        for (const auto& armor : armors) {
            // 绘制装甲板边界
            for (int i = 0; i < 4; i++) {
                cv::line(image, armor.corners[i], armor.corners[(i+1)%4], 
                        cv::Scalar(0, 255, 0), 2);
            }
            
            // 绘制装甲板中心
            cv::Point2f center(0, 0);
            for (auto corner : armor.corners) {
                center += corner;
            }
            center /= 4.0;
            cv::circle(image, center, 3, cv::Scalar(0, 255, 0), -1);
            
            // 显示装甲板信息
            std::string info = armor.color + " Y:" + std::to_string(int(armor.yaw)) + 
                              " P:" + std::to_string(int(armor.pitch)) + 
                              " D:" + std::to_string(int(armor.distance * 100)) + "cm";
            
            cv::putText(image, info, cv::Point(armor.corners[0].x, armor.corners[0].y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       armor.color == "BLUE" ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255), 
                       1);
        }
    }

    
    void publishImage(const cv::Mat& image) {
        // 使用cv_bridge将OpenCV图像转换为ROS图像消息
        std::unique_ptr<cv_bridge::CvImage> img_bridge;
        sensor_msgs::msg::Image msg;
        
        try {
            img_bridge = std::make_unique<cv_bridge::CvImage>();
            img_bridge->header.stamp = this->now();
            img_bridge->header.frame_id = "camera_frame";
            img_bridge->encoding = "bgr8";
            img_bridge->image = image.clone();
            img_bridge->toImageMsg(msg);
            image_publisher_->publish(msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to convert and publish image: %s", e.what());
        }
    }

    rclcpp::Logger logger_;
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capturer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::thread camera_thread_;
    
    // 固定的相机内参和畸变系数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // 固定的装甲板实际尺寸
    double armor_real_width_;
    double armor_real_height_;
    double lightbar_real_height_;

    std::vector<ColorRange> color_ranges_;
    
    // 图像处理参数（从YAML获取）
    int binary_threshold_;
    int min_contour_area_;
    //double min_aspect_ratio_;
    //double max_aspect_ratio_;
    //double max_angle_diff_;
    //int min_color_saturation_;
    //int min_color_value_;
};

} // namespace rmcs_core::armor_detector 

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::armor_detector::GetCameraFrame, rmcs_executor::Component)


#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>


class ArmorDetector {
public:
    struct Config {
        int binary_threshold = 160; // 二值化阈值
        double min_contour_area = 100.0; // 最小轮廓面积
        double min_length_width_ratio = 1.5; // 最小长宽比
        double max_length_width_ratio = 5.0; // 最大长宽比
        double max_angle_diff = 10.0; // 最大角度差
        double max_center_distance_ratio = 3.0; // 最大中心距离比
    };

    struct Armor {
        cv::Rect bounding_rect; // 装甲板边界矩形
        std::string color; // 颜色标识
        Eigen::Vector3d euler_angles; // 欧拉角 (Yaw, Pitch, Roll)
        double confidence; // 置信度
        Armor(const cv::Rect& rect, const std::string& c, const Eigen::Vector3d& euler, double conf = 1.0)
        : bounding_rect(rect), color(c), euler_angles(euler), confidence(conf) {}
    };

    explicit ArmorDetector(const Config& config = Config()) : config_(config) {

    // 初始化相机参数（实际应用中应从配置文件读取）
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
        1000, 0, 320,
        0, 1000, 240,
        0, 0, 1);
        dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
        0.1, -0.2, 0.001, 0.002, 0);

    }


// 灯条结构体

struct LightBar {
    cv::RotatedRect rect;    
    cv::Point2f top_point;    
    cv::Point2f bottom_point; 
    std::string color;        
    double angle;             

    LightBar(const cv::RotatedRect& r, const std::string& c)
        : rect(r), color(c) 
    {
        cv::Point2f vertices[4];
        rect.points(vertices);
       
        if (vertices[0].y < vertices[2].y) {
            top_point = (vertices[0] + vertices[3]) / 2.0f; 
            bottom_point = (vertices[1] + vertices[2]) / 2.0f; 
        } else {
            top_point = (vertices[1] + vertices[2]) / 2.0f;
            bottom_point = (vertices[0] + vertices[3]) / 2.0f;
        }
        
        angle = std::atan2(bottom_point.y - top_point.y, 
                          bottom_point.x - top_point.x) * 180 / CV_PI;
    }
};

// 装甲板结构体
struct Armor {
    LightBar left_bar;
    LightBar right_bar;
    cv::Rect bounding_rect;
    std::string color;
    double confidence;
    
    Armor(const LightBar& left, const LightBar& right)
        : left_bar(left), right_bar(right), color(left.color) {
        // 计算装甲板边界框
        cv::Point2f center = (left_bar.rect.center + right_bar.rect.center) / 2.0f;
        cv::Size2f size(std::abs(left_bar.rect.center.x - right_bar.rect.center.x) + 
                       (left_bar.rect.size.width + right_bar.rect.size.width) / 2,
                       std::max(left_bar.rect.size.height, right_bar.rect.size.height) * 1.2f);
        bounding_rect = cv::RotatedRect(center, size, 0).boundingRect();
    }
};

class ArmorDetector {
public:
    struct Config {
        int binary_threshold = 160;
        double min_contour_area = 100.0;
        double max_length_width_ratio = 5.0;
        double min_length_width_ratio = 1.5;
        double max_angle_diff = 10.0;
        double max_center_distance_ratio = 3.0;
        int color_roi_size = 10;
    };
    
    ArmorDetector() = default;
    explicit ArmorDetector(const Config& config) : config_(config) {}
    
    // 主检测函数
    std::vector<Armor> detect(const cv::Mat& image) {
        std::vector<Armor> armors;
        
        // 预处理图像
        cv::Mat binary = preprocess(image);
        
        // 检测灯条
        auto light_bars = detectLightBars(image, binary);
        
        // 配对灯条
        armors = pairLightBars(light_bars);
        
        return armors;
    }
    
    // 绘制检测结果
    void drawResults(cv::Mat& image, const std::vector<Armor>& armors) {
        for (const auto& armor : armors) {
            // 绘制装甲板矩形
            cv::rectangle(image, armor.bounding_rect, cv::Scalar(0, 255, 255), 2);
            
            // 绘制灯条
            drawLightBar(image, armor.left_bar);
            drawLightBar(image, armor.right_bar);
            
            // 添加颜色标签
            std::string label = armor.color + " Armor";
            cv::putText(image, label, armor.bounding_rect.tl() + cv::Point(0, -10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }
    }
    
    // 更新配置
    void updateConfig(const Config& config) {
        config_ = config;
    }
    
    const Config& getConfig() const {
        return config_;
    }

private:
    // 图像预处理
    cv::Mat preprocess(const cv::Mat& image) {
        cv::Mat gray, binary;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, binary, config_.binary_threshold, 255, cv::THRESH_BINARY);
        return binary;
    }
    
    // 检测灯条
    std::vector<LightBar> detectLightBars(const cv::Mat& image, const cv::Mat& binary) {
        std::vector<LightBar> light_bars;
        
        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        
        // 转换为HSV用于颜色识别
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        
        for (const auto& contour : contours) {
            // 过滤太小的区域
            if (cv::contourArea(contour) < config_.min_contour_area) continue;
            
            auto rotated = cv::minAreaRect(contour);
            double ratio = std::max(rotated.size.width, rotated.size.height) / 
                          std::min(rotated.size.width, rotated.size.height);
            
            // 过滤不符合长宽比的区域
            if (ratio < config_.min_length_width_ratio || ratio > config_.max_length_width_ratio) continue;
            
            // 识别颜色
            std::string color = recognizeColor(hsv, rotated.center, config_.color_roi_size);
            if (color != "RED" && color != "BLUE") continue;
            
            light_bars.emplace_back(rotated, color);
        }
        
        return light_bars;
    }
    
    // 识别颜色
    std::string recognizeColor(const cv::Mat& hsv, const cv::Point2f& center, int roi_size) {
        cv::Rect roi(center.x - roi_size/2, center.y - roi_size/2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
        
        if (roi.width <= 0 || roi.height <= 0) return "UNKNOWN";
        
        cv::Mat color_roi = hsv(roi);
        cv::Scalar mean_hsv = cv::mean(color_roi);
        
        float H = mean_hsv[0];
        float S = mean_hsv[1];
        float V = mean_hsv[2];
        
        // 红色检测 (Hue在0-10或170-180范围)
        if ((H <= 10 || H >= 170) && S > 50 && V > 50) {
            return "RED";
        }
        // 蓝色检测 (Hue在100-140范围)
        else if (H >= 100 && H <= 140 && S > 50 && V > 50) {
            return "BLUE";
        }
        
        return "UNKNOWN";
    }
    
    // 配对灯条
    std::vector<Armor> pairLightBars(const std::vector<LightBar>& light_bars) {
        std::vector<Armor> armors;
        
        for (size_t i = 0; i < light_bars.size(); ++i) {
            for (size_t j = i + 1; j < light_bars.size(); ++j) {
                const auto& bar1 = light_bars[i];
                const auto& bar2 = light_bars[j];
                
                // 只配对相同颜色的灯条
                if (bar1.color != bar2.color) continue;
                
                // 检查角度差异
                if (std::abs(bar1.angle - bar2.angle) > config_.max_angle_diff) continue;
                
                // 检查中心距离与灯条长度的比例
                double distance = cv::norm(bar1.rect.center - bar2.rect.center);
                double avg_length = (bar1.length + bar2.length) / 2.0;
                if (distance / avg_length > config_.max_center_distance_ratio) continue;
                
                // 检查Y坐标差异（确保灯条大致水平）
                if (std::abs(bar1.rect.center.y - bar2.rect.center.y) > avg_length / 2.0) continue;
                
                // 确定左右灯条
                if (bar1.rect.center.x < bar2.rect.center.x) {
                    armors.emplace_back(bar1, bar2);
                } else {
                    armors.emplace_back(bar2, bar1);
                }
            }
        }
        
        return armors;
    }
    
    // 绘制灯条
    void drawLightBar(cv::Mat& image, const LightBar& light_bar) {
        cv::Scalar color = (light_bar.color == "RED") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        
        // 绘制旋转矩形
        for (int k = 0; k < 4; k++) {
            cv::line(image, light_bar.vertices[k], light_bar.vertices[(k + 1) % 4], color, 2);
        }
        
        // 绘制中心点
        cv::circle(image, light_bar.rect.center, 3, color, -1);
    }
    
    Config config_;
};

// #include <cv_bridge/cv_bridge.h>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

namespace rmcs_core::armor_detector {

class GetCameraFrame
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GetCameraFrame()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        camera_profile_.invert_image = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());

        image_capturer_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        // image_publisher_ =
        // this->create_publisher<sensor_msgs::msg::Image>(get_parameter("image_topic_name").as_string(), 1000);

        camera_thread_ = std::thread(&GetCameraFrame::camera_frame_update, this);
    }

    void update() override {
        // 注意：相机帧率和rmcs更新频率不同，image_capturer_->read()这一句不应该写在这里，会阻塞
        // 所以自瞄有一个自己的线程
    }

private:
    void camera_frame_update() {
        while (true) {
            cv::Mat camera_frame = image_capturer_->read();

            cv::imshow("test", camera_frame);
            cv::waitKey(1);

            // 正常来说，我们不会在这里直接使用imshow()去显示图像
            // 不过调试的话，暂时没什么问题
            // 更好的做法是使用publisher发布图像信息，然后使用foxglove或者rviz显示出来，如下

            // sensor_msgs::msg::Image message;
            // //
            // image_publisher_->publish(message);
            //
            // 注：cv_bridge不知道怎么了找不到头文件，后面再修，先用imshow
        }
    }
    rclcpp::Logger logger_;
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capturer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::string image_type_;

    std::thread camera_thread_;
};

} // namespace rmcs_core::armor_detector 

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::armor_detector::GetCameraFrame, rmcs_executor::Component)
*/