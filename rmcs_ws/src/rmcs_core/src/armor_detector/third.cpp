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

struct LightBar {
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    std::string color;
    float length;
    float angle;
    cv::RotatedRect rect;
    
    void calculateDirection() {
        float dx = bottom.x - top.x;
        float dy = bottom.y - top.y;
        angle = std::atan2(dy, dx) * 180 / CV_PI;
    }
};

class ArmorPredictor
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmorPredictor()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        camera_profile_.invert_image = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());

        image_capturer_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        brightness_threshold_ = get_parameter("brightness_threshold").as_int();
        enermy_color_ = get_parameter("enermy_color").as_string();
        auto red_lower1 = get_parameter("red_lower1").as_integer_array();
        auto red_upper1 = get_parameter("red_upper1").as_integer_array();
        auto red_lower2 = get_parameter("red_lower2").as_integer_array();
        auto red_upper2 = get_parameter("red_upper2").as_integer_array();
        auto blue_lower = get_parameter("blue_lower").as_integer_array();
        auto blue_upper = get_parameter("blue_upper").as_integer_array();

        red_lower1_ = cv::Scalar(red_lower1[0], red_lower1[1], red_lower1[2]);
        red_upper1_ = cv::Scalar(red_upper1[0], red_upper1[1], red_upper1[2]);
        red_lower2_ = cv::Scalar(red_lower2[0], red_lower2[1], red_lower2[2]);
        red_upper2_ = cv::Scalar(red_upper2[0], red_upper2[1], red_upper2[2]);
        blue_lower_ = cv::Scalar(blue_lower[0], blue_lower[1], blue_lower[2]);
        blue_upper_ = cv::Scalar(blue_upper[0], blue_upper[1], blue_upper[2]);

        camera_thread_ = std::thread(&ArmorPredictor::camera_frame_update, this);
    }

    void update() override {
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
            cv::waitKey(1);
        }
    }

    void processFrame(cv::Mat& image) {
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv, red_lower1_, red_upper1_, red_mask1);
        cv::inRange(hsv, red_lower2_, red_upper2_, red_mask2);
        red_mask = red_mask1 | red_mask2;
        
        cv::Mat blue_mask;
        cv::inRange(hsv, blue_lower_, blue_upper_, blue_mask);
        
        cv::Mat v_channel;
        cv::extractChannel(hsv, v_channel, 2);
        cv::Mat brightness_mask;
        cv::threshold(v_channel, brightness_mask, brightness_threshold_, 255, cv::THRESH_BINARY);
        
        cv::Mat final_mask;
        if (enermy_color_ == "red") {
            final_mask = red_mask & brightness_mask;
        } else if (enermy_color_ == "blue") {
            final_mask = blue_mask & brightness_mask;
        } else {
            final_mask = (red_mask | blue_mask) & brightness_mask;
        }
        
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(final_mask, final_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(final_mask, final_mask, cv::MORPH_OPEN, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(final_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<LightBar> light_bars;

        for (const auto& contour : contours) {       
            cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
            
            float width = rotated_rect.size.width;
            float height = rotated_rect.size.height;
            float aspect_ratio = (width > height) ? width / height : height / width;
            
            if (aspect_ratio < 3.0 || aspect_ratio > 30.0) continue;
            
            cv::Point2f center = rotated_rect.center;
            std::string color = detectColor(hsv, center);
            if (color.empty()) continue;
            if (color != enermy_color_) continue;

            LightBar light_bar = extractLightBar(rotated_rect, color);
            if (!light_bar.color.empty()) {
                light_bars.push_back(light_bar);
                
                cv::Point2f vertices[4];
                rotated_rect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 0, 255), 2);
                }
            }
        }

        /*
        if (light_bars.size() == 1) {
            detectPotentialLightBars(hsv, final_mask, light_bars, image);
            findOccludedLightBar(hsv, final_mask, light_bars[0], light_bars, image);
        }
        */

        pairLightBars(light_bars, image);
    }

    void detectPotentialLightBars(const cv::Mat& hsv, const cv::Mat& mask, std::vector<LightBar>& light_bars, cv::Mat& image) {
        if (light_bars.empty()) return;
        
        for (auto& bar : light_bars) {
            cv::Point2f direction(cos(bar.angle * CV_PI / 180.0), sin(bar.angle * CV_PI / 180.0));
            
            for (int side = -1; side <= 1; side += 2) {
                float search_distance = bar.length * 4.0f; 
                cv::Point2f start_point = bar.center;
                cv::Point2f end_point = bar.center + direction * search_distance * side;
                
                cv::Rect roi(
                    std::min(start_point.x, end_point.x) - bar.length/2,
                    std::min(start_point.y, end_point.y) - bar.length/2,
                    std::abs(end_point.x - start_point.x) + bar.length,
                    std::abs(end_point.y - start_point.y) + bar.length
                );
                
                roi &= cv::Rect(0, 0, mask.cols, mask.rows);
                if (roi.width <= 0 || roi.height <= 0) continue;
                
                cv::Mat roi_mask = mask(roi);
                
                std::vector<std::vector<cv::Point>> roi_contours;
                cv::findContours(roi_mask, roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                
                for (const auto& contour : roi_contours) {
                    if (cv::contourArea(contour) < 15) continue;
                    cv::RotatedRect potential_rect = cv::minAreaRect(contour);
                    
                    float width = potential_rect.size.width;
                    float height = potential_rect.size.height;
                    float aspect_ratio = (width > height) ? width / height : height / width;
                    
                    if (aspect_ratio < 1.5 || aspect_ratio > 6.0) continue; 
                    potential_rect.center.x += roi.x;
                    potential_rect.center.y += roi.y;
                    
                    cv::Point2f center = potential_rect.center;
                    std::string color = detectColor(hsv, center);
                    if (color.empty() || color != enermy_color_) continue;
                    
                    LightBar potential_bar = extractLightBar(potential_rect, color);
                    float angle_diff = std::abs(bar.angle - potential_bar.angle);
                    if (angle_diff > 20.0 && angle_diff < 160.0) continue;
                    
                    cv::Point2f center_vec = potential_bar.center - bar.center;
                    float center_angle = std::atan2(center_vec.y, center_vec.x) * 180 / CV_PI;
                    
                    float short_angle = bar.angle + 90.0;
                    short_angle = std::fmod(short_angle + 180.0, 180.0);
                    center_angle = std::fmod(center_angle + 180.0, 180.0);
                    
                    float angle_diff2 = std::abs(center_angle - short_angle);
                    if (angle_diff2 > 20.0 && angle_diff2 < 160.0) continue;
                    
                    float distance = cv::norm(center_vec);
                    if (distance > bar.length * 4.0 || distance < bar.length * 0.3) continue;
                    
                    float height_diff = std::abs(bar.center.y - potential_bar.center.y);
                    if (height_diff > bar.length * 1.0) continue;
                    
                    light_bars.push_back(potential_bar);
                    
                    cv::Point2f vertices[4];
                    potential_rect.points(vertices);
                    for (int i = 0; i < 4; i++) {
                        cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 255), 2); // 黄色表示潜在灯条
                    }
                    
                    cv::line(image, bar.center, potential_bar.center, cv::Scalar(255, 255, 0), 1);
                    
                    break;
                }
            }
        }
    }

    void findOccludedLightBar(const cv::Mat& hsv, const cv::Mat& mask, const LightBar& bar, 
                             std::vector<LightBar>& light_bars, cv::Mat& image) {
        cv::Point2f direction(cos(bar.angle * CV_PI / 180.0), sin(bar.angle * CV_PI / 180.0));
        
        cv::Point2f normal(-direction.y, direction.x);
        
        for (int side = -1; side <= 1; side += 2) {
            float search_distance = bar.length * 2.0f;
            
            cv::Point2f search_center = bar.center + normal * search_distance * side;
            
            cv::Rect roi(
                search_center.x - bar.length,
                search_center.y - bar.length/2,
                bar.length * 2,
                bar.length
            );
            
            roi &= cv::Rect(0, 0, mask.cols, mask.rows);
            if (roi.width <= 0 || roi.height <= 0) continue;
            
            cv::Mat roi_mask = mask(roi);
            
            std::vector<std::vector<cv::Point>> roi_contours;
            cv::findContours(roi_mask, roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            for (const auto& contour : roi_contours) {
                //
                if (cv::contourArea(contour) < 30) continue; 
                cv::RotatedRect potential_rect = cv::minAreaRect(contour);
                
                float width = potential_rect.size.width;
                float height = potential_rect.size.height;
                float aspect_ratio = (width > height) ? width / height : height / width;
                
                if (aspect_ratio < 1.2 || aspect_ratio > 5.0) continue; 

                potential_rect.center.x += roi.x;
                potential_rect.center.y += roi.y;
                
                cv::Point2f center = potential_rect.center;
                std::string color = detectColor(hsv, center);
                if (color.empty() || color != enermy_color_) continue;
                
                LightBar potential_bar = extractLightBar(potential_rect, color);
                float angle_diff = std::abs(bar.angle - potential_bar.angle);
                if (angle_diff > 25.0 && angle_diff < 155.0) continue;
                
                cv::Point2f center_vec = potential_bar.center - bar.center;
                float center_angle = std::atan2(center_vec.y, center_vec.x) * 180 / CV_PI;
                
                float short_angle = bar.angle + 90.0;
                short_angle = std::fmod(short_angle + 180.0, 180.0);
                center_angle = std::fmod(center_angle + 180.0, 180.0);
                
                float angle_diff2 = std::abs(center_angle - short_angle);
                if (angle_diff2 > 25.0 && angle_diff2 < 155.0) continue;
                
                float distance = cv::norm(center_vec);
                if (distance > bar.length * 3.0 || distance < bar.length * 0.5) continue;
                
                float height_diff = std::abs(bar.center.y - potential_bar.center.y);
                if (height_diff > bar.length * 0.6) continue;
                
                light_bars.push_back(potential_bar);
                
                cv::Point2f vertices[4];
                potential_rect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    cv::line(image, vertices[i], vertices[(i+1)%4], cv::Scalar(255, 0, 255), 2);
                }
                
                cv::rectangle(image, roi, cv::Scalar(0, 255, 0), 1);
                
                break;
            }
        }
    }

    LightBar extractLightBar(const cv::RotatedRect& rect, const std::string& color) {
        cv::Point2f vertices[4];
        rect.points(vertices);
        
        cv::Point2f top = (vertices[0] + vertices[1]) / 2.0f;
        cv::Point2f bottom = (vertices[2] + vertices[3]) / 2.0f;
        
        if (top.y > bottom.y) {
            std::swap(top, bottom);
        }
        
        cv::Point2f center = (top + bottom) / 2.0f;
        
        LightBar light_bar;
        light_bar.top = top;
        light_bar.bottom = bottom;
        light_bar.center = center;
        light_bar.color = color;
        light_bar.length = cv::norm(top - bottom);
        light_bar.rect = rect;
        light_bar.calculateDirection();
        
        return light_bar;
    }

    std::string detectColor(const cv::Mat& hsv, const cv::Point2f& point) {
        int roi_size = 10;
        cv::Rect roi(point.x - roi_size/2, point.y - roi_size/2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
        
        if (roi.width <= 0 || roi.height <= 0) return "";
        
        cv::Mat roi_hsv = hsv(roi);
        
        cv::Mat red_mask1, red_mask2;
        cv::inRange(roi_hsv, red_lower1_, red_upper1_, red_mask1);
        cv::inRange(roi_hsv, red_lower2_, red_upper2_, red_mask2);
        cv::Mat red_mask = red_mask1 | red_mask2;
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
        if (light_bars.size() < 1) return;

        std::vector<LightBar> enemy_bars;
        for (const auto& bar : light_bars) {
            if (bar.color == enermy_color_) {
                enemy_bars.push_back(bar);
            }
        }
        
        pairBarsOfSameColor(enemy_bars, image);
    }

    void pairBarsOfSameColor(const std::vector<LightBar>& bars, cv::Mat& image) {
        if (bars.size() < 1) return;
        
        std::vector<LightBar> sorted_bars = bars;
        std::sort(sorted_bars.begin(), sorted_bars.end(), 
            [](const LightBar& a, const LightBar& b) {
                return a.center.x < b.center.x;
            });
        
        for (size_t i = 0; i < sorted_bars.size() - 1; i++) {
            for (size_t j = i + 1; j < sorted_bars.size(); j++) {
                const LightBar& bar1 = sorted_bars[i];
                const LightBar& bar2 = sorted_bars[j];
                
                float angle_diff = std::abs(bar1.angle - bar2.angle);
                if (angle_diff > 20.0 && angle_diff < 160.0) continue;
                
                cv::Point2f center_vec = bar2.center - bar1.center;
                float center_angle = std::atan2(center_vec.y, center_vec.x) * 180 / CV_PI;
                
                float short_angle1 = bar1.angle + 90.0;
                float short_angle2 = bar2.angle + 90.0;
                
                short_angle1 = std::fmod(short_angle1 + 180.0, 180.0);
                short_angle2 = std::fmod(short_angle2 + 180.0, 180.0);
                center_angle = std::fmod(center_angle + 180.0, 180.0);
                
                float angle_diff1 = std::abs(center_angle - short_angle1);
                float angle_diff2 = std::abs(center_angle - short_angle2);
                
                if (angle_diff1 > 20.0 && angle_diff1 < 160.0) continue;
                if (angle_diff2 > 20.0 && angle_diff2 < 160.0) continue;
                
                float distance = cv::norm(center_vec);
                float avg_length = (bar1.length + bar2.length) / 2.0;
                
                if (distance > avg_length * 4.0 || distance < avg_length * 0.3) continue;
                
                //float height_diff = std::abs(bar1.center.y - bar2.center.y);
                //if (height_diff > avg_length * 0.5) continue;
                
                float length_ratio = std::max(bar1.length, bar2.length) / std::min(bar1.length, bar2.length);
                if (length_ratio > 3.0) continue;
                
                const LightBar* left = &bar1;
                const LightBar* right = &bar2;
                if (bar1.center.x > bar2.center.x) {
                    left = &bar2;
                    right = &bar1;
                }
                
                cv::Point2f armor_corners[4] = {
                    left->top,    
                    right->top,   
                    right->bottom, 
                    left->bottom   
                };
                
                for (int k = 0; k < 4; k++) {
                    cv::line(image, armor_corners[k], armor_corners[(k+1)%4], cv::Scalar(255, 0, 0), 3);
                }
                
                cv::Point2f center = (armor_corners[0] + armor_corners[1] + armor_corners[2] + armor_corners[3]) / 4.0f;
                cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);
                
                cv::putText(image, left->color, center, 
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
    
    int brightness_threshold_;
    std::string enermy_color_;
    cv::Scalar red_lower1_;
    cv::Scalar red_upper1_;
    cv::Scalar red_lower2_;
    cv::Scalar red_upper2_;
    cv::Scalar blue_lower_;
    cv::Scalar blue_upper_;
};

} // namespace rmcs_core::armordetector

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::armordetector::ArmorPredictor, rmcs_executor::Component)