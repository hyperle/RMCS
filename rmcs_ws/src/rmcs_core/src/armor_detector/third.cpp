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
        if (angle < 0) angle += 180;
    }
};

struct Armor {
    std::vector<cv::Point2f> corners;
    std::string color;
    cv::Point2f center;
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

        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            1000.0, 0.0, 320.0,
            0.0, 1000.0, 240.0,
            0.0, 0.0, 1.0);
        dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

        armor_width_ = 0.14;
        armor_height_ = 0.125;
        LightBar light_bar;
        light_bar.length = 0.6;

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
            
            //
            if (aspect_ratio < 3.0 || aspect_ratio > 24.0) continue;
            
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
        std::vector<Armor> detected_armors = pairLightBars(light_bars, image);
        if (!detected_armors.empty()) {
            calculateAndDisplayEulerAngles(detected_armors[0], image);
        }
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

    std::vector<Armor> pairLightBars(const std::vector<LightBar>& light_bars, cv::Mat& image) {
        std::vector<Armor> armors;
        
        if (light_bars.size() < 2) return armors;
        
        std::vector<LightBar> sorted_bars = light_bars;
        std::sort(sorted_bars.begin(), sorted_bars.end(), 
            [](const LightBar& a, const LightBar& b) {
                return a.center.x < b.center.x;
            });
        
        for (size_t i = 0; i < sorted_bars.size() - 1; i++) {
            for (size_t j = i + 1; j < sorted_bars.size(); j++) {
                const LightBar& bar1 = sorted_bars[i];
                const LightBar& bar2 = sorted_bars[j];
                
                if (bar1.color != bar2.color) continue;
                
                float angle_diff = std::abs(bar1.angle - bar2.angle);
                if (angle_diff > 20.0 && angle_diff < 160.0) continue;
                
                //float distance = cv::norm(bar2.center - bar1.center);
                //float avg_length = (bar1.length + bar2.length) / 2.0;
                //if (distance > avg_length * 4.0 || distance < avg_length * 0.3) continue;
                
                float length_ratio = std::max(bar1.length, bar2.length) / std::min(bar1.length, bar2.length);
                if (length_ratio > 3.0) continue;
                
                const LightBar* left = &bar1;
                const LightBar* right = &bar2;
                if (bar1.center.x > bar2.center.x) {
                    left = &bar2;
                    right = &bar1;
                }
                
                std::vector<cv::Point2f> armor_corners = {
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
                
                Armor armor;
                armor.corners = armor_corners;
                armor.color = left->color;
                armor.center = center;
                armors.push_back(armor);
            }
        }
        
        return armors;
    }

    void calculateAndDisplayEulerAngles(const Armor& armor, cv::Mat& image) {
        std::vector<cv::Point3f> object_points;
        double half_width = armor_width_ / 2.0f;
        double half_height = armor_height_ / 2.0f;
        
        object_points.emplace_back(-half_width, -half_height, 0); // 左上
        object_points.emplace_back(half_width, -half_height, 0);  // 右上
        object_points.emplace_back(half_width, half_height, 0);   // 右下
        object_points.emplace_back(-half_width, half_height, 0);  // 左下
        
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(object_points, armor.corners, camera_matrix_, dist_coeffs_, rvec, tvec);
        
        if (success) {
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
            
            cv::Vec3d euler_angles = rotationMatrixToEulerAngles(rotation_matrix);
            
            double pitch = euler_angles[0] * 180 / CV_PI;
            double yaw = euler_angles[1] * 180 / CV_PI;
            double roll = euler_angles[2] * 180 / CV_PI;
            
            std::string pitch_text = "Pitch: " + std::to_string(pitch).substr(0, 6) + " deg";
            std::string yaw_text = "Yaw: " + std::to_string(yaw).substr(0, 6) + " deg";
            std::string roll_text = "Roll: " + std::to_string(roll).substr(0, 6) + " deg";
            
            cv::putText(image, pitch_text, cv::Point(10, 30), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            cv::putText(image, yaw_text, cv::Point(10, 60), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            cv::putText(image, roll_text, cv::Point(10, 90), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            
            double distance = cv::norm(tvec);
            std::string dist_text = "Distance: " + std::to_string(distance).substr(0, 6) + " m";
            cv::putText(image, dist_text, cv::Point(10, 120), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }
    }

    cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat& R) {
        double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
        
        bool singular = sy < 1e-6;
        
        double x, y, z;
        if (!singular) {
            x = atan2(R.at<double>(2,1), R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        } else {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }
        
        return {x, y, z};
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

    rclcpp::Logger logger_;
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capturer_;
    std::thread camera_thread_;
    
    int brightness_threshold_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double armor_height_;
    double armor_width_;
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