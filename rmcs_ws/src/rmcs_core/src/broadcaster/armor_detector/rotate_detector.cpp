// #include <cv_bridge/cv_bridge.h>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include "detector/rotate_armor_detect.hpp" 

namespace rmcs_core::armor_detector {

class GetCameraFrame
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GetCameraFrame()
        : Node(get_component_name()
        , rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        camera_profile_.invert_image = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain = static_cast<float>(get_parameter("gain").as_double());

        image_capturer_ = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);

        brightness_threshold_ = get_parameter("brightness_threshold").as_int();
        min_contour_area_ = get_parameter("min_contour_area").as_double();
        enermy_color_ = get_parameter("enermy_color").as_string();
        auto toScalar = [](const std::vector<int64_t>& vec) {
                return cv::Scalar(vec[0], vec[1], vec[2]);
            };
        red_lower1_ = toScalar(get_parameter("red_lower1").as_integer_array());
        red_upper1_ = toScalar(get_parameter("red_upper1").as_integer_array());
        red_lower2_ = toScalar(get_parameter("red_lower2").as_integer_array());
        red_upper2_ = toScalar(get_parameter("red_upper2").as_integer_array());
        blue_lower_ = toScalar(get_parameter("blue_lower").as_integer_array());
        blue_upper_ = toScalar(get_parameter("blue_upper").as_integer_array());

        camera_thread_ = std::thread(&GetCameraFrame::camera_frame_update, this);
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
            armor_detector_.processFrame(camera_frame,
                red_lower1_, red_upper1_, 
                red_lower2_, red_upper2_, 
                blue_lower_, blue_upper_,
                brightness_threshold_,
                min_contour_area_,
                enermy_color_
            );
            cv::imshow("Armor Detection", camera_frame);
            cv::waitKey(1);
        }
    }

    armordetector::ArmorDetector armor_detector_;
    cv::Scalar red_lower1_, red_upper1_, red_lower2_, red_upper2_, blue_lower_, blue_upper_;
    rclcpp::Logger logger_;
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capturer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::thread camera_thread_;
    int brightness_threshold_;
    double min_contour_area_;
    std::string enermy_color_;
};

} // namespace rmcs_examples

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::armor_detector::GetCameraFrame, rmcs_executor::Component)