#ifndef ARMOR_DETECTOR_H
#define ARMOR_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>
#include <string>
#include <memory>

namespace rmcs_core::armordetector {

struct Vector {
    double roll;    //x
    double pitch;   //y
    double yaw;     //z
};

class Object {
public:
    int id;
    cv::RotatedRect rect;
    cv::Point2f center;
    std::string color;
    std::vector<cv::Point2f> corners;
    std::vector<Vector> normal_vector;

    void calculateFaceNormal() {
        if (corners.size() < 4) {
            cv::Point2f vertices[4];
            rect.points(vertices);
            corners = std::vector<cv::Point2f>(vertices, vertices + 4);
        }
        
        if (corners.size() >= 4) {
            cv::Point3f edge1(corners[1].x - corners[0].x, 
                            corners[1].y - corners[0].y, 0);
            cv::Point3f edge2(corners[3].x - corners[0].x, 
                            corners[3].y - corners[0].y, 0);
            
            cv::Point3f normal_3d(
                edge1.y * edge2.z - edge1.z * edge2.y,
                edge1.z * edge2.x - edge1.x * edge2.z,
                edge1.x * edge2.y - edge1.y * edge2.x
            );
            
            float norm = std::sqrt(normal_3d.x * normal_3d.x + 
                                normal_3d.y * normal_3d.y + 
                                normal_3d.z * normal_3d.z);
            if (norm > 0) {
                normal_3d.x /= norm;
                normal_3d.y /= norm;
                normal_3d.z /= norm;
            }
            
            Vector normal_vec;
            normal_vec.roll = normal_3d.x;
            normal_vec.pitch = normal_3d.y;
            normal_vec.yaw = normal_3d.z;
            
            normal_vector.clear();
            normal_vector.push_back(normal_vec);
        }
    }
};

class LightBar : public Object {
public:
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f top_mid;
    cv::Point2f bottom_mid;
    float length;
    float real_length;

    void calculateCenter() {
        center = (top + bottom) / 2.0f;
    }

    void calculateMidPoints() {
    cv::Point2f vertices[4];
    rect.points(vertices);
    
    std::vector<cv::Point2f> top_points, bottom_points;
    for (auto & vertice : vertices) {
        if (vertice.y < center.y) {
            top_points.push_back(vertice);
        } else {
            bottom_points.push_back(vertice);
        }
    }
    
    if (top_points.size() == 2) {
        top_mid = (top_points[0] + top_points[1]) / 2.0f;
    }
    if (bottom_points.size() == 2) {
        bottom_mid = (bottom_points[0] + bottom_points[1]) / 2.0f;
    }
}
};

class Armor : public Object {
public:
    float width;
    float height;
    double distance;
    std::pair<int, int> lightbar_ids;
    cv::Vec3d euler_angles;
    float pairing_confidence;
};

class ArmorDetector {
private:
    int next_light_bar_id_ = 1;
    std::set<int> paired_light_bar_ids_;

    cv::Scalar red_lower1_, red_upper1_, red_lower2_, red_upper2_, blue_lower_, blue_upper_;

public:
    cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            1000.0, 0.0, 320.0,
            0.0, 1000.0, 240.0,
            0.0, 0.0, 1.0);
    cv::Mat dist_coeffs_ = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    double armor_width_ = 0.14;
    double armor_height_ = 0.125;

    void processFrame(cv::Mat& image, 
                 const cv::Scalar& red_lower1, const cv::Scalar& red_upper1,
                 const cv::Scalar& red_lower2, const cv::Scalar& red_upper2,
                 const cv::Scalar& blue_lower, const cv::Scalar& blue_upper,
                 int brightness_threshold, double min_contour_area, const std::string& enemy_color) {

        red_lower1_ = red_lower1;
        red_upper1_ = red_upper1;
        red_lower2_ = red_lower2;
        red_upper2_ = red_upper2;
        blue_lower_ = blue_lower;
        blue_upper_ = blue_upper;

        paired_light_bar_ids_.clear();

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
        cv::threshold(v_channel, brightness_mask, brightness_threshold, 255, cv::THRESH_BINARY);
        
        cv::Mat final_mask;
        if (enemy_color == "red") {
            final_mask = red_mask & brightness_mask;
        } else if (enemy_color == "blue") {
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
            std::string color = detectColor(hsv, center, red_mask, blue_mask, enemy_color);
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
        if (light_bars.size() >= 2) {
            // 按x坐标排序
            std::sort(light_bars.begin(), light_bars.end(), 
                     [](const LightBar& a, const LightBar& b) { return a.center.x < b.center.x; });
            
            // 尝试配对相邻的灯条
            for (size_t i = 0; i < light_bars.size() - 1; i++) {
                for (size_t j = i + 1; j < light_bars.size(); j++) {
                    LightBar& left_bar = light_bars[i];
                    LightBar& right_bar = light_bars[j];
                    
                    drawArmor(image, left_bar, right_bar, left_bar.color);
                        
                }
            }
        }
    }
private:
    LightBar extractLightBar(const cv::RotatedRect& rect, const std::string& color) {
        cv::Point2f vertices[4];
        rect.points(vertices);

        cv::Point2f edge = vertices[1] - vertices[0];
        float angle = std::atan2(edge.y, edge.x);
        std::vector<cv::Point2f> points(vertices, vertices + 4);
    
        cv::Point2f top = vertices[0];
        cv::Point2f bottom = vertices[0];

        for (int i = 1; i < 4; i++) {
            if (vertices[i].y < top.y) top = vertices[i];
            if (vertices[i].y > bottom.y) bottom = vertices[i];
        }


        LightBar light_bar;
        light_bar.id = next_light_bar_id_++;
        light_bar.top = top;
        light_bar.bottom = bottom;
        light_bar.color = color;
        light_bar.rect = rect;
        light_bar.corners = std::vector<cv::Point2f>(vertices, vertices + 4);
        
        light_bar.calculateCenter();
        light_bar.calculateMidPoints();
        
        return light_bar;
    }

    void drawArmor(cv::Mat& image, const LightBar& left_bar, const LightBar& right_bar, const std::string& color) {
        cv::Scalar armor_color = (color == "red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);

        cv::Point2f top_left = left_bar.top_mid;
        cv::Point2f top_right = right_bar.top_mid;
        cv::Point2f bottom_right = right_bar.bottom_mid;
        cv::Point2f bottom_left = left_bar.bottom_mid;

        cv::line(image, top_left, top_right, armor_color, 3);
        cv::line(image, top_right, bottom_right, armor_color, 3);
        cv::line(image, bottom_right, bottom_left, armor_color, 3);
        cv::line(image, bottom_left, top_left, armor_color, 3);
    }

    std::string detectColor(const cv::Mat& hsv, const cv::Point2f& point,
                            const cv::Mat& red_mask, const cv::Mat& blue_mask,
                            const std::string& enemy_color) {
        int roi_size = 10;
        cv::Rect roi(point.x - roi_size/2, point.y - roi_size/2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
        
        if (roi.width <= 0 || roi.height <= 0) return "";

        cv::Mat roi_hsv = hsv(roi);

        cv::Mat red_mask1, red_mask2;
        cv::inRange(roi_hsv, red_lower1_, red_upper1_, red_mask1);
        cv::inRange(roi_hsv, red_lower2_, red_upper2_, red_mask2);
        cv::Mat red_mask_roi = red_mask1 | red_mask2;
        int red_pixels = cv::countNonZero(red_mask_roi);
                
        cv::Mat blue_mask_roi;
        cv::inRange(roi_hsv, blue_lower_, blue_upper_, blue_mask_roi);
        int blue_pixels = cv::countNonZero(blue_mask_roi);


        if (enemy_color == "red") {
            if (red_pixels > 10 && red_pixels > blue_pixels * 1.5) {
                return "red";
            }
        } else if (enemy_color == "blue") {
            if (blue_pixels > 10 && blue_pixels > red_pixels * 1.5) {
                return "blue";
            }
        } else {
            if (red_pixels > 10 && red_pixels > blue_pixels * 1.5) {
                return "red";
            } else if (blue_pixels > 10 && blue_pixels > red_pixels * 1.5) {
                return "blue";
            }
        }
            
        return "";
    }


    };
} // namespace rmcs_core::armordetector

#endif // ARMOR_DETECTOR_H