#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
#include <vector>
#include <cmath>
#include <algorithm>

namespace rmcs_core::armordetector {

class SingleDetector
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SingleDetector()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        // 获取相机参数
        camera_profile_.invert_image = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());

        image_capturer_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        // 获取检测参数
        binary_threshold_ = get_parameter("binary_threshold").as_int();
        min_contour_area_ = get_parameter("min_contour_area").as_int();
        max_angle_diff_ = get_parameter("max_angle_diff").as_double();
        min_length_ratio_ = get_parameter("min_length_ratio").as_double();
        max_length_ratio_ = get_parameter("max_length_ratio").as_double();
        // 获取颜色范围参数
        auto red_lower = get_parameter("red_lower").as_integer_array();
        auto red_upper = get_parameter("red_upper").as_integer_array();
        auto blue_lower = get_parameter("blue_lower").as_integer_array();
        auto blue_upper = get_parameter("blue_upper").as_integer_array();

        red_lower_ = cv::Scalar(red_lower[0], red_lower[1], red_lower[2]);
        red_upper_ = cv::Scalar(red_upper[0], red_upper[1], red_upper[2]);
        blue_lower_ = cv::Scalar(blue_lower[0], blue_lower[1], blue_lower[2]);
        blue_upper_ = cv::Scalar(blue_upper[0], blue_upper[1], blue_upper[2]);

        camera_thread_ = std::thread(&SingleDetector::camera_frame_update, this);
    }

    void update() override {
        // 注意：相机帧率和rmcs更新频率不同，image_capturer_->read()这一句不应该写在这里，会阻塞
        // 所以自瞄有一个自己的线程
    }

private:
    void camera_frame_update() {
        while (rclcpp::ok()) {
            cv::Mat camera_frame = image_capturer_->read();
            if (camera_frame.empty()) {
                RCLCPP_WARN(logger_, "Empty frame received");
                continue;
            }

            processFrame(camera_frame);
            
            cv::imshow("Armor Detection", camera_frame);
            
            if (cv::waitKey(1) == 27) { // ESC键退出
                break;
            }
        }
    }

    void processFrame(cv::Mat& image) {
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        cv::Mat binary;
        cv::threshold(gray, binary, binary_threshold_, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        std::vector<LightBar> light_bars;

        for (const auto& contour : contours) {
            if (cv::contourArea(contour) < min_contour_area_) continue;
            
            cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
            
            float aspect_ratio = rotated_rect.size.width / rotated_rect.size.height;
            if (aspect_ratio < 1.0) aspect_ratio = 1.0 / aspect_ratio;
            
            if (aspect_ratio < 2.0 || aspect_ratio > 8.0) continue;
            
            LightBar light_bar = extractLightBar(rotated_rect, hsv, contour);
            if (!light_bar.color.empty()) {
                light_bars.push_back(light_bar);
                
                cv::Point2f vertices[4];
                rotated_rect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 0, 255), 2);
                }
            }
        }

        pairLightBars(light_bars, image);
    }

    struct LightBar {
        cv::Point2f top;
        cv::Point2f bottom;
        cv::Point2f center;
        std::string color;
        float length;
        float angle;
        std::vector<cv::Point2f> contour;
        
        void calculateDirection() {
            float dx = bottom.x - top.x;
            float dy = bottom.y - top.y;
            angle = std::atan2(dy, dx) * 180 / CV_PI;
        }
    };

    LightBar extractLightBar(const cv::RotatedRect& rect, const cv::Mat& hsv, const std::vector<cv::Point>& contour) {
        cv::Point2f vertices[4];
        rect.points(vertices);
        
        cv::Point2f top = (vertices[0] + vertices[1]) / 2.0f;
        cv::Point2f bottom = (vertices[2] + vertices[3]) / 2.0f;
        
        if (top.y > bottom.y) {
            std::swap(top, bottom);
        }
        
        cv::Point2f center = (top + bottom) / 2.0f;
        
        std::string color = detectColor(hsv, center);
        if (color.empty()) return LightBar();
        
        LightBar light_bar;
        light_bar.top = top;
        light_bar.bottom = bottom;
        light_bar.center = center;
        light_bar.color = color;
        light_bar.length = cv::norm(top - bottom);
         light_bar.contour.clear();
        for (const auto& point : contour) {
            light_bar.contour.push_back(cv::Point2f(static_cast<float>(point.x), static_cast<float>(point.y)));
        }
        
        light_bar.calculateDirection();
        
        return light_bar;
    }

    std::string detectColor(const cv::Mat& hsv, const cv::Point2f& point) {
        int roi_size = 10;
        cv::Rect roi(point.x - roi_size/2, point.y - roi_size/2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
        
        if (roi.width <= 0 || roi.height <= 0) return "";
        
        cv::Mat roi_hsv = hsv(roi);
        
        cv::Mat red_mask;
        cv::inRange(roi_hsv, red_lower_, red_upper_, red_mask);
        int red_pixels = cv::countNonZero(red_mask);
        
        cv::Mat blue_mask;
        cv::inRange(roi_hsv, blue_lower_, blue_upper_, blue_mask);
        int blue_pixels = cv::countNonZero(blue_mask);
        
        if (red_pixels > blue_pixels && red_pixels > 5) {
            return "red";
        } else if (blue_pixels > red_pixels && blue_pixels > 5) {
            return "blue";
        }
        
        return "";
    }

    void pairLightBars(const std::vector<LightBar>& light_bars, cv::Mat& image) {
        if (light_bars.size() < 2) return;
        
        std::vector<LightBar> red_bars;
        std::vector<LightBar> blue_bars;
        
        for (const auto& bar : light_bars) {
            if (bar.color == "red") {
                red_bars.push_back(bar);
            } else if (bar.color == "blue") {
                //blue_bars.push_back(bar);
            }
        }
        
        pairBarsOfSameColor(red_bars, image);
        
        pairBarsOfSameColor(blue_bars, image);
    }

    void pairBarsOfSameColor(const std::vector<LightBar>& bars, cv::Mat& image) {
    if (bars.size() < 2) return;
    
    for (size_t i = 0; i < bars.size() - 1; i++) {
        for (size_t j = i + 1; j < bars.size(); j++) {
            const LightBar& bar1 = bars[i];
            const LightBar& bar2 = bars[j];
            
            float angle_diff = std::abs(bar1.angle - bar2.angle);
            if (angle_diff > max_angle_diff_) continue;
            
            float length_ratio = std::max(bar1.length, bar2.length) / 
                                std::min(bar1.length, bar2.length);
            if (length_ratio < min_length_ratio_ || length_ratio > max_length_ratio_) continue;
            
            // 获取两个灯条的旋转矩形
            cv::RotatedRect rect1 = cv::minAreaRect(bar1.contour);
            cv::RotatedRect rect2 = cv::minAreaRect(bar2.contour);
            
            // 获取旋转矩形的四个顶点
            cv::Point2f vertices1[4], vertices2[4];
            rect1.points(vertices1);
            rect2.points(vertices2);
            
            // 确定左右灯条
            const LightBar* left = &bar1;
            const LightBar* right = &bar2;
            cv::Point2f* leftVertices = vertices1;
            cv::Point2f* rightVertices = vertices2;
            
            if (bar1.center.x > bar2.center.x) {
                left = &bar2;
                right = &bar1;
                leftVertices = vertices2;
                rightVertices = vertices1;
            }
            
            // 对左右灯条的顶点按x坐标排序
            std::sort(leftVertices, leftVertices + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.x < b.x;
            });
            
            std::sort(rightVertices, rightVertices + 4, [](const cv::Point2f& a, const cv::Point2f& b) {
                return a.x < b.x;
            });
            
            std::vector<cv::Point2f> armor_points;
            
            armor_points.push_back(leftVertices[2]);
            armor_points.push_back(leftVertices[3]);
            
            armor_points.push_back(rightVertices[0]);
            armor_points.push_back(rightVertices[1]);
            
            cv::Point2f center(0, 0);
            for (const auto& p : armor_points) {
                center += p;
            }
            center /= 4.0f;
            
            std::sort(armor_points.begin(), armor_points.end(), [center](const cv::Point2f& a, const cv::Point2f& b) {
                return std::atan2(a.y - center.y, a.x - center.x) < std::atan2(b.y - center.y, b.x - center.x);
            });
            
            for (int k = 0; k < 4; k++) {
                cv::line(image, armor_points[k], armor_points[(k+1)%4], cv::Scalar(255, 0, 0), 3);
            }
            
            cv::Point2f text_center = (armor_points[0] + armor_points[1] + armor_points[2] + armor_points[3]) / 4.0f;
            cv::putText(image, left->color, text_center, 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, 
                       left->color == "red" ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0), 
                       2);
        }
    }
}

    rclcpp::Logger logger_;
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capturer_;
    std::thread camera_thread_;
    
    int binary_threshold_;
    int min_contour_area_;
    double max_angle_diff_;
    double min_length_ratio_;
    double max_length_ratio_;
    
    cv::Scalar red_lower_;
    cv::Scalar red_upper_;
    cv::Scalar blue_lower_;
    cv::Scalar blue_upper_;
};

} // namespace rmcs_core::armordetector

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::armordetector::SingleDetector, rmcs_executor::Component)