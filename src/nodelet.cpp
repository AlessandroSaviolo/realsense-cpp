#include <realsense-cpp/camera.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

namespace realsense_camera {

class RSCameraNodelet : public rclcpp::Node {
public:
    explicit RSCameraNodelet(const rclcpp::NodeOptions &options)
        : Node("realsense_camera", options), _clock(RCL_ROS_TIME) {

        // Declare and get parameters
        this->declare_parameter<std::string>("frame_id", "");
        this->declare_parameter<std::string>("ws_path", "");
        this->declare_parameter<int>("width", 0);
        this->declare_parameter<int>("height", 0);
        this->declare_parameter<int>("fps", 0);

        this->get_parameter("ws_path", _ws_path);
        this->get_parameter("frame_id", _frame_id);
        this->get_parameter("width", _width);
        this->get_parameter("height", _height);
        this->get_parameter("fps", _fps);

        _camera.init(_ws_path, _width, _height, _fps);

        // Initialize publishers
        _pub_color = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 1);
        _pub_depth = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", 1);
        _pub_rgbd = this->create_publisher<realsense2_camera_msgs::msg::RGBD>("camera/rgbd/image_raw", 1);

        // Create a timer to grab and publish frames
        _timer = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / _fps)),
            std::bind(&RSCameraNodelet::grabAndPublishFrame, this));
    }

private:
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
        auto color_msg = cv_bridge::CvImage(
                          std_msgs::msg::Header(),
                          sensor_msgs::image_encodings::RGB8,
                          color_image).toImageMsg();

        auto depth_msg = cv_bridge::CvImage(
                          std_msgs::msg::Header(),
                          sensor_msgs::image_encodings::TYPE_16UC1,
                          depth_image).toImageMsg();

        // Set header information
        rclcpp::Time timestamp(_timestamp);
        color_msg->header.stamp = timestamp;
        depth_msg->header.stamp = timestamp;
        color_msg->header.frame_id = _frame_id;
        depth_msg->header.frame_id = _frame_id;

        // Publish color and depth images
        _pub_color->publish(*color_msg);
        _pub_depth->publish(*depth_msg);

        // Publish RGBD message
        realsense2_camera_msgs::msg::RGBD rgbd_msg;
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
    int _width{0};
    int _height{0};
    int _fps{0};
    uint64_t _timestamp{0};

    // Camera
    RSCamera _camera;

    // ROS communication
    rclcpp::Clock _clock;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_depth;
    rclcpp::Publisher<realsense2_camera_msgs::msg::RGBD>::SharedPtr _pub_rgbd;
};

} // namespace realsense_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(realsense_camera::RSCameraNodelet)
