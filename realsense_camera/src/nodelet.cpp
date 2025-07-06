#include <realsense_camera/camera.hpp>
#include <realsense_msgs/msg/rgbd.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

namespace realsense_camera {

class RSCameraNodelet : public rclcpp::Node {
public:
    // Constructor for RSCameraNodelet class
    // Initializes the node, declares and retrieves parameters, sets up publishers, and starts the timer to grab frames
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
        if (!this->get_parameter(param_name, param_value))
            std::cout << "[RSCamera] No " << param_name << "!" << std::endl << std::flush;
        else
            std::cout << "[RSCamera] " << param_name << ": " << param_value << std::endl << std::flush;
    }

    // Initialize parameters from the parameter server
    void initializeParameters() {
        declareAndLoadParameter("frame_id", _frame_id, std::string(""));
        declareAndLoadParameter("ws_path", _ws_path, std::string(""));
        declareAndLoadParameter("serial_number", _serial_number, std::string(""));
        declareAndLoadParameter("width", _width, 0);
        declareAndLoadParameter("height", _height, 0);
        declareAndLoadParameter("fps", _fps, 0);
    }

    // Initialize the RealSense camera with given parameters
    void initializeCamera() {
        _camera.init(_ws_path, _width, _height, _fps, _serial_number);
    }

    // Initialize ROS 2 publishers for color, depth, and RGBD topics
    void initializePublishers() {
        _pub_color = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 1);
        _pub_depth = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", 1);
        _pub_rgbd = this->create_publisher<realsense_msgs::msg::RGBD>("camera/rgbd/image_raw", 1);
    }

    // Create a timer to grab and publish frames at the specified rate
    void startFrameTimer() {
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / _fps)),
            std::bind(&RSCameraNodelet::grabAndPublishFrame, this));
    }

    // Grab frames from the RealSense camera and publish them as ROS messages
    void grabAndPublishFrame() {
        cv::Mat color_image, depth_image;

        // Grab frames from the RealSense device
        if (!_camera.grabFrames(color_image, depth_image, _timestamp)) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), _clock, 1000,
                "[RSCamera] Dropped frame longer than RealSense API timeout");
            return;
        }

        // Convert OpenCV matrices to ROS Image messages
        auto color_msg = convertCvMatToRosImage(color_image, sensor_msgs::image_encodings::RGB8);
        auto depth_msg = convertCvMatToRosImage(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);

        // Set header information for timestamp and frame ID
        rclcpp::Time timestamp(_timestamp);
        setHeaderInfo(color_msg, timestamp, _frame_id);
        setHeaderInfo(depth_msg, timestamp, _frame_id);

        // Publish color and depth images
        _pub_color->publish(*color_msg);
        _pub_depth->publish(*depth_msg);

        // Publish RGBD message
        publishRGBDMessage(color_msg, depth_msg, timestamp);
    }

    // Convert an OpenCV Mat to a ROS Image message
    sensor_msgs::msg::Image::SharedPtr convertCvMatToRosImage(const cv::Mat &image, const std::string &encoding) {
        return cv_bridge::CvImage(std_msgs::msg::Header(), encoding, image).toImageMsg();
    }

    // Set header information for a ROS Image message
    void setHeaderInfo(sensor_msgs::msg::Image::SharedPtr &msg, const rclcpp::Time &timestamp, const std::string &frame_id) {
        msg->header.stamp = timestamp;
        msg->header.frame_id = frame_id;
    }

    // Publish an RGBD message consisting of color and depth images
    void publishRGBDMessage(const sensor_msgs::msg::Image::SharedPtr &color_msg,
                            const sensor_msgs::msg::Image::SharedPtr &depth_msg,
                            const rclcpp::Time &timestamp) {
        realsense_msgs::msg::RGBD rgbd_msg;
        rgbd_msg.header.stamp = timestamp;
        rgbd_msg.header.frame_id = _frame_id;
        rgbd_msg.rgb_camera_info.header.stamp = timestamp;
        rgbd_msg.depth_camera_info.header.stamp = timestamp;
        rgbd_msg.rgb = *color_msg;
        rgbd_msg.depth = *depth_msg;
        _pub_rgbd->publish(rgbd_msg);
    }

    // Node parameters
    std::string _frame_id;
    std::string _ws_path;
    std::string _serial_number;
    int _width{640};   // Default width
    int _height{480};  // Default height
    int _fps{30};      // Default FPS
    uint64_t _timestamp{0};

    // Camera instance
    RSCamera _camera;

    // ROS communication
    rclcpp::Clock _clock;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_depth;
    rclcpp::Publisher<realsense_msgs::msg::RGBD>::SharedPtr _pub_rgbd;
};

} // namespace realsense_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(realsense_camera::RSCameraNodelet)
