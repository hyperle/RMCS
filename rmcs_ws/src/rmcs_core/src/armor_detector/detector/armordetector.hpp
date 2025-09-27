#ifndef ARMOR_DETECTOR_H
#define ARMOR_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>
#include <string>
#include <memory>

namespace rmcs_core::armordetector {

struct LightBar {
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    std::string color;
    double distance;
    float length;
    float real_length;
    float angle;
    cv::RotatedRect rect;
    int id;

    void calculateDirection() {
        float dx = bottom.x - top.x;
        float dy = bottom.y - top.y;
        angle = std::atan2(dy, dx) * 180 / CV_PI;
        if (angle < 0) angle += 180;
    }

    void calculateCenter() {
        center = (top + bottom) / 2.0f;
    }

    void calculateDistance(double focal_length) {
        if (length > 0) {
            distance = (real_length * focal_length) / length;
        }
    }

};

struct Armor {
    std::vector<cv::Point2f> corners;
    std::string color;
    cv::Point2f center;
    float width;
    float height;
    float real_width;
    float real_height;
    cv::Vec3d euler_angles;
    std::pair<int, int> lightbar_ids;

    void calculateDimensions() {
        width = cv::norm(corners[1] - corners[0]);
        height = cv::norm(corners[2] - corners[1]);
    }
};

class ArmorDetector {
private:
    int next_lightbar_id_ = 0;
    std::set<int> paired_lightbar_ids_;

public:
    cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            1000.0, 0.0, 320.0,
            0.0, 1000.0, 240.0,
            0.0, 0.0, 1.0);
    cv::Mat dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    void processFrame(cv::Mat& image, 
                 const cv::Scalar& red_lower1, const cv::Scalar& red_upper1,
                 const cv::Scalar& red_lower2, const cv::Scalar& red_upper2,
                 const cv::Scalar& blue_lower, const cv::Scalar& blue_upper,
                 int brightness_threshold, double min_contour_area, const std::string& enemy_color) {

        LightBar light_bar;
        light_bar.real_length = 0.06;

        Armor armor;
        armor.real_height = 0.125;
        armor.real_width  = 0.14;
        paired_lightbar_ids_.clear();

        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(hsv, red_lower1, red_upper1, red_mask1);
        cv::inRange(hsv, red_lower2, red_upper2, red_mask2);
        red_mask = red_mask1 | red_mask2;
        
        cv::Mat blue_mask;
        cv::inRange(hsv, blue_lower, blue_upper, blue_mask);
        
        cv::Mat v_channel;
        cv::extractChannel(hsv, v_channel, 2);
        cv::Mat brightness_mask;
        cv::threshold(v_channel, brightness_mask, brightness_threshold, 255, cv::THRESH_BINARY);
        
        cv::Mat final_mask;
        if (enemy_color == "red") {
        red_mask = red_mask & brightness_mask;
        final_mask = red_mask;
        } else if (enemy_color == "blue") {
            blue_mask = blue_mask & brightness_mask;
            final_mask = blue_mask;
        }
        
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(final_mask, final_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(final_mask, final_mask, cv::MORPH_OPEN, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(final_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<LightBar> light_bars;

        for (const auto& contour : contours) {      
            double area = cv::contourArea(contour);
            if (area < min_contour_area) continue; 

            cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
            
            float width = rotated_rect.size.width;
            float height = rotated_rect.size.height;
            float aspect_ratio = height / width;
            
            //
            if (aspect_ratio < 3.0 || aspect_ratio > 12.0) continue;
            
            cv::Point2f center = rotated_rect.center;
            std::string color = detectColor(hsv, center, red_mask, blue_mask);
            if (color.empty()) continue;
            if (color != enemy_color) continue;

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
        double focal_height = camera_matrix_.at<double>(0,0);
        double focal_width  = camera_matrix_.at<double>(0,0);
        armor.calculateDistance(focal_height, focal_width);
        std::vector<Armor> detected_armors = pairLightBars(light_bars, image);
        if (!detected_armors.empty()) {
            calculateAndDisplayEulerAngles(detected_armors[0], image);
        }
    }
    
    std::string detectColor(const cv::Mat& hsv, const cv::Point2f& point, const cv::Mat& red_mask, const cv::Mat& blue_mask) {
        int roi_size = 10;
        cv::Rect roi(point.x - roi_size/2, point.y - roi_size/2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
        
        if (roi.width <= 0 || roi.height <= 0) return "";
        
        cv::Mat roi_hsv = hsv(roi);
        int red_pixels  = cv::countNonZero(red_mask);
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
                return a.top.x < b.top.x && a.bottom.x < b.bottom.x;
            });
        
        for (size_t i = 0; i < sorted_bars.size() - 1; i++) {
            for (size_t j = i + 1; j < sorted_bars.size(); j++) {
                const LightBar& bar1 = sorted_bars[i];
                const LightBar& bar2 = sorted_bars[j];
                
                if (bar1.color != bar2.color) continue;
                
                float angle_diff = std::abs(bar1.angle - bar2.angle);
                if (angle_diff > 20.0 && angle_diff < 160.0) continue;
                
                //float distance = cv::norm(bar2.center - bar1.center);
                //if (distance > avg_length * 4.0 || distance < avg_length * 0.3) continue;
                
                float length_ratio = std::max(bar1.length, bar2.length) / std::min(bar1.length, bar2.length);
                if (length_ratio > 3.0) continue;
                
                const LightBar* left = &bar1;
                const LightBar* right = &bar2;
                if (bar1.top.x > bar2.top.x && bar1.bottom.x > bar2.bottom.x) {
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
        double half_width = armor.width / 2.0f;
        double half_height = armor.height / 2.0f;
        
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
            //std::swap(top, bottom);
        }

        LightBar light_bar;
        light_bar.top = top;
        light_bar.bottom = bottom;
        light_bar.color = color;
        light_bar.length = cv::norm(top - bottom);
        light_bar.rect = rect;
        light_bar.calculateDirection();
        
        return light_bar;
    }
};

} // namespace rmcs_core::armordetector

#endif // ARMOR_DETECTOR_H