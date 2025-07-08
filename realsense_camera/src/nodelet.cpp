#include <realsense_camera/camera.hpp>
#include <realsense_msgs/msg/rgbd.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

namespace realsense_camera {

class RSCameraNodelet : public rclcpp::Node {
public:
    explicit RSCameraNodelet(const rclcpp::NodeOptions &options)
        : Node("realsense_camera", options), _clock(RCL_ROS_TIME) {
        initializeParameters();
        initializeCamera();
        initializePublishers();
        startFrameTimer();
    }

private:
    template <typename T>
    void declareAndLoadParameter(const std::string &param_name, T &param_value, const T &default_value) {
        this->declare_parameter<T>(param_name, default_value);
        if (!this->get_parameter(param_name, param_value)) {
            std::cout << "[RSCamera] No " << param_name << "!" << std::endl << std::flush;
        } else {
            std::cout << "[RSCamera] " << param_name << ": ";
            if constexpr (std::is_same<T, std::vector<std::string>>::value) {
                for (const auto& val : param_value) std::cout << val << " ";
                std::cout << std::endl;
            } else {
                std::cout << param_value << std::endl;
            }
        }
    }

    void initializeParameters() {
        declareAndLoadParameter("frame_id", _frame_id, std::string("camera_link"));
        declareAndLoadParameter("ws_path", _ws_path, std::string(""));
        declareAndLoadParameter("serial_number", _serial_numbers, std::vector<std::string>{});
        declareAndLoadParameter("width", _width, 0);
        declareAndLoadParameter("height", _height, 0);
        declareAndLoadParameter("fps", _fps, 0);
    }

    void initializeCamera() {
        _camera.init(_ws_path, _width, _height, _fps, _serial_numbers);
    }

    void initializePublishers() {
        for (const auto& serial : _serial_numbers) {
            _pub_color[serial] = this->create_publisher<sensor_msgs::msg::Image>("camera/serial_" + serial + "/color/image_raw", 1);
            _pub_depth[serial] = this->create_publisher<sensor_msgs::msg::Image>("camera/serial_" + serial + "/depth/image_raw", 1);
            _pub_rgbd[serial]  = this->create_publisher<realsense_msgs::msg::RGBD>("camera/serial_" + serial + "/rgbd/image_raw", 1);
        }
    }

    void startFrameTimer() {
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / _fps)),
            std::bind(&RSCameraNodelet::grabAndPublishFrames, this));
    }

    void grabAndPublishFrames() {
        for (const auto& serial : _serial_numbers) {
            cv::Mat color_image, depth_image;
            uint64_t timestamp;

            if (!_camera.grabFrames(serial, color_image, depth_image, timestamp)) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), _clock, 1000,
                    "[RSCamera] Dropped frame from camera %s", serial.c_str());
                continue;
            }

            auto color_msg = convertCvMatToRosImage(color_image, sensor_msgs::image_encodings::RGB8);
            auto depth_msg = convertCvMatToRosImage(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);

            rclcpp::Time ts(timestamp);
            setHeaderInfo(color_msg, ts, _frame_id);
            setHeaderInfo(depth_msg, ts, _frame_id);

            _pub_color[serial]->publish(*color_msg);
            _pub_depth[serial]->publish(*depth_msg);

            realsense_msgs::msg::RGBD rgbd_msg;
            rgbd_msg.header.stamp = ts;
            rgbd_msg.header.frame_id = _frame_id;
            rgbd_msg.rgb_camera_info.header.stamp = ts;
            rgbd_msg.depth_camera_info.header.stamp = ts;
            rgbd_msg.rgb = *color_msg;
            rgbd_msg.depth = *depth_msg;
            _pub_rgbd[serial]->publish(rgbd_msg);
        }
    }

    sensor_msgs::msg::Image::SharedPtr convertCvMatToRosImage(const cv::Mat &image, const std::string &encoding) {
        return cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
    }

    void setHeaderInfo(sensor_msgs::msg::Image::SharedPtr &msg, const rclcpp::Time &timestamp, const std::string &frame_id) {
        msg->header.stamp = timestamp;
        msg->header.frame_id = frame_id;
    }

    // Node parameters
    std::string _frame_id;
    std::string _ws_path;
    std::vector<std::string> _serial_numbers;
    int _width;
    int _height;
    int _fps;

    // Camera
    RSCamera _camera;

    // ROS communication
    rclcpp::Clock _clock;
    rclcpp::TimerBase::SharedPtr _timer;

    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> _pub_color;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> _pub_depth;
    std::unordered_map<std::string, rclcpp::Publisher<realsense_msgs::msg::RGBD>::SharedPtr> _pub_rgbd;
};

} // namespace realsense_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(realsense_camera::RSCameraNodelet)
