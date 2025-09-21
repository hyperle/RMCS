/*#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <unordered_map>

namespace rmcs_core::armordetector {

struct LightBar {
    cv::Point2f top;
    cv::Point2f bottom;
    cv::Point2f center;
    std::string color;
    float pixel_length;
    cv::Vec2f direction;
    
    void calculateDirection() {
        float dx = bottom.x - top.x;
        float dy = bottom.y - top.y;
        direction = cv::Vec2f(dx, dy);
        cv::normalize(direction, direction);
    }
};

struct ArmorPlate {
    cv::Point2f corners[4];
    std::string color;
    Eigen::Vector3f position;
    Eigen::Vector3f euler_angles;
    float distance;
    float confidence;
    int id;
    
    cv::Point2f center() const {
        cv::Point2f c(0, 0);
        for (auto corner : corners) {
            c += corner;
        }
        return c / 4.0f;
    }
};

struct DetectionResult {
    std::vector<ArmorPlate> armors;
    cv::Mat debug_image;
    uint64_t timestamp;
};

class AutoAim
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAim()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        camera_profile_.invert_image = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());

        image_capturer_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        // image_publisher_ =
        // this->create_publisher<sensor_msgs::msg::Image>(get_parameter("image_topic_name").as_string(), 1000);

        camera_thread_ = std::thread(&AutoAim::camera_frame_update, this);
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

    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::string image_type_;

    std::thread camera_thread_;
};

class Camera {
public:
    virtual ~Camera() = default;
    virtual bool initialize() = 0;
    virtual cv::Mat captureFrame() = 0;
    virtual const cv::Mat& cameraMatrix() const = 0;
    virtual const cv::Mat& distCoeffs() const = 0;
};

class LightBarDetector {
public:
    virtual ~LightBarDetector() = default;
    virtual std::vector<LightBar> detect(const cv::Mat& frame) = 0;
};

class ArmorPairer {
public:
    virtual ~ArmorPairer() = default;
    virtual std::vector<ArmorPlate> pairLightBars(const std::vector<LightBar>& light_bars) = 0;
};

class PoseEstimator {
public:
    virtual ~PoseEstimator() = default;
    virtual bool estimatePose(ArmorPlate& armor) = 0;
};

class TargetSelector {
public:
    struct Target {
        ArmorPlate armor;
        float priority;
        int id;
    };
    
    virtual ~TargetSelector() = default;
    virtual Target selectTarget(const std::vector<ArmorPlate>& armors) = 0;
};

class MotionPredictor {
public:
    virtual ~MotionPredictor() = 0;
    virtual void update(const ArmorPlate& armor, uint64_t timestamp) = 0;
    virtual Eigen::Vector3f predictPosition(float dt) = 0;
    virtual void reset() = 0;
};

inline MotionPredictor::~MotionPredictor() = default; 


class HikCamera : public Camera {
public:
    HikCamera(rclcpp::Logger logger) : logger_(logger) {
        camera_matrix_ = (cv::Mat_<double>(3, 3) << 
            1722.231837421459, 0.0, 701.3056440882832,
            0.0, 1724.876404292754, 564.5821718351237,
            0.0, 0.0, 1.0);
            
        dist_coeffs_ = (cv::Mat_<double>(1, 5) << 
            -0.064232403853946, -0.087667493884102, 0.0, 0.0, 0.792381808294582);
    }
    
    bool initialize() override {
        hikcamera::ImageCapturer::CameraProfile profile;
        profile.invert_image = false;
        profile.exposure_time = std::chrono::microseconds(5000);
        profile.gain = 1.0;
        
        capturer_ = std::make_unique<hikcamera::ImageCapturer>(profile);
        return capturer_ != nullptr;
    }
    
    cv::Mat captureFrame() override {
        if (capturer_) {
            return capturer_->read();
        }
        return cv::Mat();
    }
    
    const cv::Mat& cameraMatrix() const override { return camera_matrix_; }
    const cv::Mat& distCoeffs() const override { return dist_coeffs_; }
    
private:
    rclcpp::Logger logger_;
    std::unique_ptr<hikcamera::ImageCapturer> capturer_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

class ContourBasedLightBarDetector : public LightBarDetector {
public:
    struct Config {
        int binary_threshold;
        int min_contour_area;
        double min_aspect_ratio;
        double max_aspect_ratio;
        std::vector<std::tuple<std::string, cv::Scalar, cv::Scalar>> color_ranges;
        
        Config() : 
            binary_threshold(160),
            min_contour_area(100),
            min_aspect_ratio(2.0),
            max_aspect_ratio(8.0),
            color_ranges({
                {"red", cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255)},
                {"blue", cv::Scalar(100, 100, 100), cv::Scalar(140, 255, 255)}
            }) {}
    };
    
    ContourBasedLightBarDetector(const Config& config = Config()) : config_(config) {}
    
    std::vector<LightBar> detect(const cv::Mat& frame) override {
        std::vector<LightBar> light_bars;
        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        cv::Mat mask = createColorMask(hsv);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) < config_.min_contour_area) continue;
            
            cv::RotatedRect rect = cv::minAreaRect(contour);
            float aspect_ratio = rect.size.width / rect.size.height;
            if (aspect_ratio < 1.0) aspect_ratio = 1.0 / aspect_ratio;
            
            if (aspect_ratio < config_.min_aspect_ratio || 
                aspect_ratio > config_.max_aspect_ratio) continue;
            
            LightBar bar = extractLightBar(rect, hsv);
            if (!bar.color.empty()) {
                light_bars.push_back(bar);
            }
        }
        
        return light_bars;
    }
    
private:
    cv::Mat createColorMask(const cv::Mat& hsv) {
        cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
        
        for (const auto& range : config_.color_ranges) {
            cv::Mat color_mask;
            cv::inRange(hsv, std::get<1>(range), std::get<2>(range), color_mask);
            cv::bitwise_or(mask, color_mask, mask);
        }
        
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        
        return mask;
    }
    
    LightBar extractLightBar(const cv::RotatedRect& rect, const cv::Mat& hsv) {
        cv::Point2f vertices[4];
        rect.points(vertices);
        
        cv::Point2f top = vertices[0];
        cv::Point2f bottom = vertices[0];
        
        for (int i = 1; i < 4; i++) {
            if (vertices[i].y < top.y) top = vertices[i];
            if (vertices[i].y > bottom.y) bottom = vertices[i];
        }
        
        cv::Point2f center = (top + bottom) / 2.0f;
        std::string color = detectColor(hsv, center);
        if (color.empty()) return LightBar();
        
        LightBar bar;
        bar.top = top;
        bar.bottom = bottom;
        bar.center = center;
        bar.color = color;
        bar.pixel_length = cv::norm(top - bottom);
        bar.calculateDirection();
        
        return bar;
    }
    
    std::string detectColor(const cv::Mat& hsv, const cv::Point2f& point) {
        int roi_size = 10;
        cv::Rect roi(point.x - roi_size/2, point.y - roi_size/2, roi_size, roi_size);
        roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
        
        if (roi.width <= 0 || roi.height <= 0) return "";
        
        cv::Mat roi_hsv = hsv(roi);
        std::map<std::string, int> color_counts;
        
        for (const auto& range : config_.color_ranges) {
            cv::Mat mask;
            cv::inRange(roi_hsv, std::get<1>(range), std::get<2>(range), mask);
            color_counts[std::get<0>(range)] = cv::countNonZero(mask);
        }
        
        auto max_color = std::max_element(color_counts.begin(), color_counts.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        
        return max_color->second > 5 ? max_color->first : "";
    }
    
    Config config_;
};

class GeometricArmorPairer : public ArmorPairer {
public:
    struct Config {
        double max_angle_diff;
        double min_length_ratio;
        double max_length_ratio;
        double max_distance_ratio;
        
        Config() : 
            max_angle_diff(15.0),
            min_length_ratio(0.5),
            max_length_ratio(2.0),
            max_distance_ratio(4.0) {}
    };
    
    GeometricArmorPairer(const Config& config = Config()) : config_(config) {}
    
    std::vector<ArmorPlate> pairLightBars(const std::vector<LightBar>& light_bars) override {
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
    
private:
    std::vector<ArmorPlate> pairBarsOfSameColor(const std::vector<LightBar>& bars) {
        std::vector<ArmorPlate> armors;
        
        if (bars.size() < 2) return armors;
        
        for (size_t i = 0; i < bars.size() - 1; i++) {
            for (size_t j = i + 1; j < bars.size(); j++) {
                const LightBar& bar1 = bars[i];
                const LightBar& bar2 = bars[j];
                
                float angle_diff = std::acos(bar1.direction.dot(bar2.direction)) * 180 / CV_PI;
                if (angle_diff > config_.max_angle_diff) continue;
                
                float length_ratio = std::max(bar1.pixel_length, bar2.pixel_length) / 
                                    std::min(bar1.pixel_length, bar2.pixel_length);
                if (length_ratio < config_.min_length_ratio || 
                    length_ratio > config_.max_length_ratio) continue;
                
                float distance = cv::norm(bar1.center - bar2.center);
                float avg_length = (bar1.pixel_length + bar2.pixel_length) / 2.0f;
                
                if (distance > avg_length * config_.max_distance_ratio) continue;
                
                const LightBar* left = &bar1;
                const LightBar* right = &bar2;
                if (bar1.center.x > bar2.center.x) {
                    left = &bar2;
                    right = &bar1;
                }
                
                ArmorPlate armor;
                armor.color = bar1.color;
                armor.corners[0] = left->top;
                armor.corners[1] = right->top;
                armor.corners[2] = right->bottom;
                armor.corners[3] = left->bottom;
                
                armor.confidence = calculatePairingConfidence(*left, *right, angle_diff, 
                                                             length_ratio, distance/avg_length);
                
                armors.push_back(armor);
            }
        }
        
        std::sort(armors.begin(), armors.end(), 
            [](const ArmorPlate& a, const ArmorPlate& b) {
                return a.confidence > b.confidence;
            });
        
        return armors;
    }
    
    float calculatePairingConfidence(const LightBar& left, const LightBar& right, 
                                    float angle_diff, float length_ratio, float distance_ratio) {
        float angle_score = 1.0 - (angle_diff / config_.max_angle_diff);
        float length_score = 1.0 - (std::abs(length_ratio - 1.0) / 
                                   (config_.max_length_ratio - 1.0));
        float distance_score = 1.0 - (distance_ratio / config_.max_distance_ratio);
        
        return (angle_score + length_score + distance_score) / 3.0;
    }
    
    Config config_;
};

class HomographyPoseEstimator : public PoseEstimator {
public:
    HomographyPoseEstimator(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs)
        : camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs) {
        armor_real_width_ = 0.14;
        armor_real_height_ = 0.125;
    }
    
    bool estimatePose(ArmorPlate& armor) override {
        std::vector<cv::Point3f> object_points;
        float half_width = static_cast<float>(armor_real_width_ / 2.0f);
        float half_height = static_cast<float>(armor_real_height_ / 2.0f);
        
        object_points.emplace_back(-half_width, -half_height, 0);
        object_points.emplace_back(half_width, -half_height, 0);
        object_points.emplace_back(half_width, half_height, 0);
        object_points.emplace_back(-half_width, half_height, 0);
        
        std::vector<cv::Point2f> image_points;
        for (int i = 0; i < 4; i++) {
            image_points.push_back(armor.corners[i]);
        }
        
        cv::Mat homography = cv::findHomography(object_points, image_points);
        if (homography.empty()) return false;
        
        return decomposeHomography(homography, armor);
    }
    
private:
    bool decomposeHomography(const cv::Mat& H, ArmorPlate& armor) {
        Eigen::Matrix3d K;
        K << camera_matrix_.at<double>(0,0), camera_matrix_.at<double>(0,1), camera_matrix_.at<double>(0,2),
             camera_matrix_.at<double>(1,0), camera_matrix_.at<double>(1,1), camera_matrix_.at<double>(1,2),
             camera_matrix_.at<double>(2,0), camera_matrix_.at<double>(2,1), camera_matrix_.at<double>(2,2);
        
        Eigen::Matrix3d H_eigen;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                H_eigen(i, j) = H.at<double>(i, j);
            }
        }
        
        Eigen::Matrix3d H_normalized = K.inverse() * H_eigen;
        
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(H_normalized, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();
        Eigen::Vector3d S = svd.singularValues();
        
        Eigen::Matrix3d R = U * V.transpose();
        
        if (R.determinant() < 0) {
            R = -R;
        }
        
        Eigen::Vector3d t = H_normalized.col(2) / S.mean();
        Eigen::Vector3d euler_d = R.eulerAngles(2, 1, 0);
        
        // 显式转换双精度值为单精度
        armor.position = Eigen::Vector3f(
            static_cast<float>(t.x()),
            static_cast<float>(t.y()),
            static_cast<float>(t.z())
        );
        
        armor.euler_angles = Eigen::Vector3f(
            static_cast<float>(euler_d.x()),
            static_cast<float>(euler_d.y()),
            static_cast<float>(euler_d.z())
        );
        
        armor.distance = static_cast<float>(t.norm());
        
        return true;
    }
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    double armor_real_width_;
    double armor_real_height_;
};

class PriorityBasedTargetSelector : public TargetSelector {
public:
    Target selectTarget(const std::vector<ArmorPlate>& armors) override {
        if (armors.empty()) return Target();
        
        std::vector<Target> targets;
        for (const auto& armor : armors) {
            Target target;
            target.armor = armor;
            target.priority = calculatePriority(armor);
            target.id = generateId(armor);
            targets.push_back(target);
        }
        
        auto best_target = std::max_element(targets.begin(), targets.end(),
            [](const Target& a, const Target& b) { return a.priority < b.priority; });
        
        return *best_target;
    }
    
private:
    float calculatePriority(const ArmorPlate& armor) {
        float distance_score = 1.0 / (1.0 + armor.distance);
        float angle_score = 1.0 / (1.0 + std::abs(armor.euler_angles[0]) + std::abs(armor.euler_angles[1]));
        
        return distance_score * 0.4 + angle_score * 0.4 + armor.confidence * 0.2;
    }
    
    int generateId(const ArmorPlate& armor) {
        cv::Point2f center = armor.center();
        return static_cast<int>(center.x * 1000 + center.y);
    }
};

class LinearMotionPredictor : public MotionPredictor {
public:
    struct TargetState {
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        uint64_t timestamp;
    };
    
    void update(const ArmorPlate& armor, uint64_t timestamp) override {
        TargetState new_state;
        new_state.position = armor.position;
        new_state.timestamp = timestamp;
        
        if (!states_.empty()) {
            const TargetState& last_state = states_.back();
            float dt = (timestamp - last_state.timestamp) / 1e9;
            
            if (dt > 0) {
                new_state.velocity = (new_state.position - last_state.position) / dt;
            }
        }
        
        states_.push_back(new_state);
        
        if (states_.size() > max_history_) {
            states_.pop_front();
        }
    }
    
    Eigen::Vector3f predictPosition(float dt) override {
        if (states_.size() < 2) {
            return states_.back().position;
        }
        
        const TargetState& current = states_.back();
        return current.position + current.velocity * dt;
    }
    
    void reset() override {
        states_.clear();
    }
    
private:
    std::deque<TargetState> states_;
    const size_t max_history_ = 10;
};

class ArmorDetectionPipeline {
public:
    struct Config {
        std::shared_ptr<Camera> camera;
        std::shared_ptr<LightBarDetector> lightbar_detector;
        std::shared_ptr<ArmorPairer> armor_pairer;
        std::shared_ptr<PoseEstimator> pose_estimator;
        std::shared_ptr<TargetSelector> target_selector;
        std::shared_ptr<MotionPredictor> motion_predictor;
    };
    
    explicit ArmorDetectionPipeline(Config  config) : config_(std::move(config)) {}
    
    bool initialize() {
        return config_.camera->initialize();
    }
    
    DetectionResult processFrame() {
        DetectionResult result;
        result.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        cv::Mat frame = config_.camera->captureFrame();
        if (frame.empty()) {
            return result;
        }
        
        std::vector<LightBar> light_bars = config_.lightbar_detector->detect(frame);
        std::vector<ArmorPlate> armors = config_.armor_pairer->pairLightBars(light_bars);
        
        for (auto& armor : armors) {
            if (config_.pose_estimator->estimatePose(armor)) {
                result.armors.push_back(armor);
            }
        }
        
        auto target = config_.target_selector->selectTarget(result.armors);
        if (target.priority > 0) {
            config_.motion_predictor->update(target.armor, result.timestamp);
        }
        
        result.debug_image = createDebugImage(frame, result.armors);
        
        return result;
    }
    
    void setCamera(std::shared_ptr<Camera> camera) { config_.camera = camera; }
    void setLightBarDetector(std::shared_ptr<LightBarDetector> detector) { config_.lightbar_detector = detector; }
    void setArmorPairer(std::shared_ptr<ArmorPairer> pairer) { config_.armor_pairer = pairer; }
    void setPoseEstimator(std::shared_ptr<PoseEstimator> estimator) { config_.pose_estimator = estimator; }
    void setTargetSelector(std::shared_ptr<TargetSelector> selector) { config_.target_selector = selector; }
    void setMotionPredictor(std::shared_ptr<MotionPredictor> predictor) { config_.motion_predictor = predictor; }
    
private:
    cv::Mat createDebugImage(const cv::Mat& frame, const std::vector<ArmorPlate>& armors) {
        cv::Mat debug_image = frame.clone();
        
        for (const auto& armor : armors) {
            for (int i = 0; i < 4; i++) {
                cv::line(debug_image, armor.corners[i], armor.corners[(i+1)%4], 
                        cv::Scalar(0, 255, 0), 2);
            }
            
            cv::Point2f center = armor.center();
            cv::circle(debug_image, center, 3, cv::Scalar(0, 255, 0), -1);
            
            std::string info = armor.color + " Y:" + std::to_string(int(armor.euler_angles[0])) + 
                              " P:" + std::to_string(int(armor.euler_angles[1])) + 
                              " D:" + std::to_string(int(armor.distance)) + "m";
            
            cv::putText(debug_image, info, cv::Point(armor.corners[0].x, armor.corners[0].y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       armor.color == "blue" ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255), 
                       1);
        }
        
        return debug_image;
    }
    
    Config config_;
};

} // namespace rmcs_core::armordetector

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::armordetector::AutoAim, rmcs_executor::Component)*/