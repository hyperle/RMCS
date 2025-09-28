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
    float length;
    float real_length;

    void calculateCenter() {
        center = (top + bottom) / 2.0f;
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
        std::vector<Armor> detected_armors = pairLightBars(light_bars, image);
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
        
        return light_bar;
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

    std::vector<Armor> pairLightBars(std::vector<LightBar>& light_bars, cv::Mat& image) {
        std::vector<Armor> armors;
        if (light_bars.size() < 2) {
            return armors;
        
        for (auto& bar : light_bars) {
            if (bar.id == 0) {
                bar.id = next_light_bar_id_++;
            }
            bar.calculateFaceNormal();
        }
        
        std::set<int> paired_ids;
        
        for (size_t i = 0; i < light_bars.size(); ++i) {

            if (paired_ids.find(light_bars[i].id) != paired_ids.end()) {
                continue;
            }
            
            std::vector<LightBar*> neighbors = findNeighbors(light_bars, i, 2);
            
            for (LightBar* neighbor : neighbors) {
                if (neighbor->id == light_bars[i].id || 
                    paired_ids.find(neighbor->id) != paired_ids.end()) {
                    continue;
                }
                
                float confidence = evaluatePairSimple(light_bars[i], *neighbor);
                
                if (confidence > 0.001f) {
                    Armor armor = createArmorFromPair(light_bars[i], *neighbor, confidence);
                    armors.push_back(armor);
                    
                    paired_ids.insert(light_bars[i].id);
                    paired_ids.insert(neighbor->id);
                    
                    if (!image.empty()) {
                        drawArmorPair(image, armor, light_bars[i], *neighbor);
                    }
                    break;
                }
            }
        }
        return armors;
    }
    }

    std::vector<LightBar*> findNeighbors(const std::vector<LightBar>& bars, size_t center_idx, int max_neighbors) {
        std::vector<LightBar*> neighbors;
        const LightBar& center_bar = bars[center_idx];
        
        std::vector<std::pair<float, LightBar*>> distance_pairs;
        
        for (size_t i = 0; i < bars.size(); ++i) {
            if (i == center_idx) continue;
            
            float distance = cv::norm(bars[i].center - center_bar.center);
            distance_pairs.emplace_back(distance, const_cast<LightBar*>(&bars[i]));
        }
        
        if (distance_pairs.size() > max_neighbors * 2) {
            std::partial_sort(
                distance_pairs.begin(), 
                distance_pairs.begin() + max_neighbors * 2, 
                distance_pairs.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; }
            );
            
            for (int i = 0; i < max_neighbors * 2; ++i) {
                neighbors.push_back(distance_pairs[i].second);
            }
        } else {
            for (const auto& pair : distance_pairs) {
                neighbors.push_back(pair.second);
            }
        }
        
        return neighbors;
    }

    float evaluatePairSimple(const LightBar& bar1, const LightBar& bar2) {
        if (bar1.color != bar2.color) {
            return 0.0f;
        }
        
        float confidence = 1.0f;
        
        confidence *=calculateNormalAngleDifference(bar1, bar2);
        
        float distance = cv::norm(bar2.center - bar1.center);
        float avg_length = (bar1.length + bar2.length) / 2.0f;
        
        if (distance < avg_length * 0.2 || distance > avg_length * 2.5) {
            confidence *= 0.3f; // 距离不合适时降低置信度
        }
        
        return confidence;
    }

    float calculateNormalAngleDifference(const LightBar& bar1, const LightBar& bar2) const {
        const Vector& n1 = bar1.normal_vector[0];
        const Vector& n2 = bar2.normal_vector[0];
        float cos = n1.roll * n2.roll + n1.pitch * n2.pitch + n1.yaw * n2.yaw;
        if (cos < 0) {
            cos = -cos;
        }
        return cos;
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
        
        armor.calculateFaceNormal();
        
        return armor;
    }
    
    void drawArmorPair(cv::Mat& image, const Armor& armor, const LightBar& left_bar, const LightBar& right_bar) {
        
        cv::Scalar armor_color = (armor.color == "red") ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::line(image, left_bar.top, right_bar.top, armor_color, 3);
        cv::line(image, right_bar.top, right_bar.bottom, armor_color, 3);
        cv::line(image, right_bar.bottom, left_bar.bottom, armor_color, 3);
        cv::line(image, left_bar.bottom, left_bar.top, armor_color, 3);
        cv::circle(image, armor.center, 5, cv::Scalar(0, 255, 0), -1);
        // 绘制灯条法线
        drawNormalVector(image, left_bar, cv::Scalar(255, 200, 100));
        drawNormalVector(image, right_bar, cv::Scalar(100, 200, 255));
        
        std::string conf_text = "Conf: " + std::to_string(armor.pairing_confidence).substr(0, 4);
        cv::putText(image, conf_text, armor.center + cv::Point2f(10, -10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    }
    
    void drawNormalVector(cv::Mat& image, const LightBar& bar, const cv::Scalar& color) {
        if (bar.normal_vector.empty()) return;
        
        const Vector& normal = bar.normal_vector[0];
        float normal_length = bar.length * 0.6f;
        
        cv::Point2f normal_2d(normal.roll, normal.pitch);
        float norm = cv::norm(normal_2d);
        if (norm > 0) {
            normal_2d /= norm;
        }
        
        cv::Point2f normal_end = bar.center + normal_2d * normal_length;
        cv::arrowedLine(image, bar.center, normal_end, color, 2, cv::LINE_AA, 0, 0.2);
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