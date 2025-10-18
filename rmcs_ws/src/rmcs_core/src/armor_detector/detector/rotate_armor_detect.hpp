#ifndef ARMOR_DETECTOR_H
#define ARMOR_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>
#include <string>
#include <memory>

namespace rmcs_core::armordetector {

struct Vector {
    double roll;      //x
    double pitch;     //y
    double yaw;       //z
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
        if (corners.size() < 4) return;

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
};

class LightBar : public Object {
public:
    cv::Point2f top, bottom, top_mid, bottom_mid;
    float length, real_length, angle;

    void calculateCenter() {
        center = (top + bottom) / 2.0f;
    }

    void calculateAngle() {
        cv::Point2f direction = bottom - top;
        angle = std::atan2(direction.y, direction.x);
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
    float width, height, pairing_confidence;
    double distance;
    std::pair<int, int> lightbar_ids;
    cv::Vec3d euler_angles;
};

class Car : public Armor {
public:
    int id;
    std::vector<Armor> armors;
    Vector rotate_center;
    double velocity, acceleration, angular_velocity;

    void calculateRotateCenter() {

    }

    void armorPrediction() {

    }
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
        
        for (auto& light_bar : light_bars) {
            light_bar.calculateAngle();
            // 绘制灯条角度信息,调试好参数后可以删去，但是没有第二块装甲板，目前调不了参数（距离与角度）
            drawLightBarInfo(image, light_bar);
        }

        std::vector<Armor> detected_armors = pairLightBars(light_bars, image);
        
        for (auto& armor : detected_armors) {
            armor.calculateFaceNormal();
            drawArmorNormal(image, armor);
        }

        if (!detected_armors.empty()) {
            calculateAndDisplayEulerAngles(detected_armors[0], image);
        }                  
    }
    

private:
    LightBar extractLightBar(const cv::RotatedRect& rect, const std::string& color) {
        cv::Point2f vertices[4];
        rect.points(vertices);

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
        light_bar.length = cv::norm(top - bottom);
        light_bar.rect = rect;
        light_bar.corners = std::vector<cv::Point2f>(vertices, vertices + 4);
        
        light_bar.calculateCenter();
        light_bar.calculateMidPoints();
        
        return light_bar;
    }

    std::string detectColor(const cv::Mat& hsv, const cv::Point2f& point,
                            const cv::Mat& red_mask, const cv::Mat& blue_mask,
                            const std::string& enemy_color) {
        float roi_size = 10;
        cv::Rect roi(point.x - roi_size/2, point.y - roi_size/2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
        
        if (roi.width <= 0 || roi.height <= 0) return "";

        cv::Mat red_mask_roi = red_mask(roi);
        cv::Mat blue_mask_roi = blue_mask(roi);
        int red_pixels = cv::countNonZero(red_mask_roi);
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

    std::vector<Armor> pairLightBars(std::vector<LightBar>& light_bars, cv::Mat& image) {
        std::vector<Armor> armors;
        if (light_bars.size() < 2) {
            return armors;
        }

        std::set<int> paired_ids;

        for (size_t i = 0; i < light_bars.size(); ++i) {
            if (paired_ids.find(light_bars[i].id) != paired_ids.end()) {
                continue;
            }
            
            for (size_t j = i + 1; j < light_bars.size(); ++j) {
                if (paired_ids.find(light_bars[j].id) != paired_ids.end()) {
                    continue;
                }
                
                float confidence = evaluatePair(light_bars[i], light_bars[j]);
                
                if (confidence > 0.8f) {
                    Armor armor = createArmorFromPair(light_bars[i], light_bars[j], confidence);
                    armors.push_back(armor);
                    
                    paired_ids.insert(light_bars[i].id);
                    paired_ids.insert(light_bars[j].id);
                    
                    if (!image.empty()) {
                        drawArmorPair(image, armor, light_bars[i], light_bars[j]);
                    }
                    break;
                }
            }
        }
        return armors;
    }

    float evaluatePair(const LightBar& bar1, const LightBar& bar2) {
        if (bar1.color != bar2.color) {
            return 0.0f;
        }
        
        float confidence = 1.0f;
        
        float angle_diff = calculateAngleDifference(bar1, bar2);
        float angle_confidence = 1.0f - (angle_diff / (CV_PI / 2));
        confidence *= angle_confidence;

        float distance = cv::norm(bar2.center - bar1.center);
        float avg_length = (bar1.length + bar2.length) / 2.0f;
        float distance_ratio = distance / avg_length;
        if (distance_ratio < 0.1f || distance_ratio > 2.5f) {
            confidence *= 0.3f;
        }
        
        float height_diff = std::abs(bar1.center.y - bar2.center.y);
        float height_ratio = height_diff / avg_length;
        if (height_ratio > 1.0f) { 
            confidence *= 0.7f;
        }
        
        float length_ratio = std::min(bar1.length, bar2.length) / std::max(bar1.length, bar2.length);
        if (length_ratio < 0.5f) {
            confidence *= 0.5f;
        }

        float parallel_confidence = calculateParallelConfidence(bar1, bar2);
        confidence *= parallel_confidence;
        
        return confidence;
    }

    float calculateAngleDifference(const LightBar& bar1, const LightBar& bar2) const {
        float raw_diff = std::abs(bar1.angle - bar2.angle);
        float diff = std::min(raw_diff, static_cast<float>(CV_PI - raw_diff));
        
        return diff;
    }

    float calculateParallelConfidence(const LightBar& bar1, const LightBar& bar2) const {
        cv::Point2f dir1 = bar1.bottom - bar1.top;
        cv::Point2f dir2 = bar2.bottom - bar2.top;
        
        double norm1 = cv::norm(dir1);
        double norm2 = cv::norm(dir2);
        
        if (norm1 < 1e-6 || norm2 < 1e-6) return 0.5f;
        
        dir1 /= norm1;
        dir2 /= norm2;
        
        float dot = dir1.x * dir2.x + dir1.y * dir2.y;
        dot = std::max(-1.0f, std::min(1.0f, dot));
        
        return std::abs(dot);
    }

    Armor createArmorFromPair(const LightBar& bar1, const LightBar& bar2, float confidence) {
        Armor armor;
        
        const LightBar* left = &bar1;
        const LightBar* right = &bar2;
        if (bar1.center.x > bar2.center.x) {
            left = &bar2;
            right = &bar1;
        }
        
        armor.corners = {
            left->top,       // 左上
            right->top,      // 右上
            right->bottom,   // 右下
            left->bottom     // 左下
        };
        
        armor.color = left->color;
        armor.center = (left->center + right->center) / 2.0f;
        armor.lightbar_ids = std::make_pair(left->id, right->id);
        armor.pairing_confidence = confidence;
        
        std::vector<cv::Point2f> points = armor.corners;
        armor.rect = cv::minAreaRect(points);
        
        armor.width = cv::norm(right->center - left->center);
        armor.height = (left->length + right->length) / 2.0f;
        
        return armor;
    }
    
    void drawArmorPair(cv::Mat& image, const Armor& armor, const LightBar& left_bar, const LightBar& right_bar) {
        cv::Scalar armor_color = (armor.color == "red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        
        cv::line(image, left_bar.top_mid, right_bar.top_mid, armor_color, 3);
        cv::line(image, right_bar.top_mid, right_bar.bottom_mid, armor_color, 3);
        cv::line(image, right_bar.bottom_mid, left_bar.bottom_mid, armor_color, 3);
        cv::line(image, left_bar.bottom_mid, left_bar.top_mid, armor_color, 3);
        
        cv::circle(image, armor.center, 5, cv::Scalar(0, 255, 0), -1);
        
        std::string conf_text = "Conf: " + std::to_string(armor.pairing_confidence).substr(0, 4);
        cv::putText(image, conf_text, armor.center + cv::Point2f(10, -10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    }

    void drawLightBarInfo(cv::Mat& image, const LightBar& light_bar) {
        cv::circle(image, light_bar.center, 3, cv::Scalar(255, 255, 0), -1);
        
        std::string angle_text = std::to_string(light_bar.angle * 180 / CV_PI).substr(0, 4) + "°";
        cv::putText(image, angle_text, light_bar.center + cv::Point2f(5, 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        
        cv::Point2f direction(50 * std::sin(light_bar.angle), 50 * std::cos(light_bar.angle));
        cv::arrowedLine(image, light_bar.center, light_bar.center + direction, 
                       cv::Scalar(255, 255, 0), 1, cv::LINE_AA, 0, 0.3);
    }

    void drawArmorNormal(cv::Mat& image, const Armor& armor) {
        if (armor.normal_vector.empty()) return;
        
        const Vector& normal = armor.normal_vector[0];
        float normal_length = 40.0f;
        
        cv::Point2f normal_2d(normal.roll * normal_length, normal.pitch * normal_length);
        cv::Point2f normal_end = armor.center + normal_2d;
        
        cv::arrowedLine(image, armor.center, normal_end, cv::Scalar(0, 255, 255), 2, cv::LINE_AA, 0, 0.3);
        
        std::string normal_text = "N:(" + std::to_string(normal.roll).substr(0, 4) + "," +
                                 std::to_string(normal.pitch).substr(0, 4) + "," +
                                 std::to_string(normal.yaw).substr(0, 4) + ")";
        cv::putText(image, normal_text, armor.center + cv::Point2f(10, 20),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    }

    void calculateAndDisplayEulerAngles(const Armor& armor, cv::Mat& image) {
        std::vector<cv::Point3f> object_points;
        double half_width  = armor_width_  / 2.0;
        double half_height = armor_height_ / 2.0;
        
        object_points.emplace_back(-half_width, -half_height, 0);  // 左上
        object_points.emplace_back( half_width, -half_height, 0);  // 右上
        object_points.emplace_back( half_width,  half_height, 0);  // 右下
        object_points.emplace_back(-half_width,  half_height, 0);  // 左下
        
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
            
            double pnp_distance = cv::norm(tvec);
            std::string dist_text = "PnP Dist: " + std::to_string(pnp_distance).substr(0, 6) + " m";
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
};
    
} // namespace rmcs_core::armordetector

#endif // ARMOR_DETECTOR_H