#include <realsense-cpp/camera.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>

namespace realsense_camera
{
class RSCameraNodelet : public rclcpp::Node {
public:
  RSCameraNodelet(const rclcpp::NodeOptions &options) 
  : Node("realsense_camera", options) {

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

    // Create publishers for color and depth images
    _pub_color = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 1);
    _pub_depth = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", 1);
    _pub_rgbd = this->create_publisher<realsense2_camera_msgs::msg::RGBD>("camera/rgbd/image_raw", 1);

    // Create a timer to grab and publish frames continuously
    _timer = this->create_wall_timer(
        std::chrono::milliseconds(1 / _fps * 1000),
        std::bind(&RSCameraNodelet::grabAndPublishFrame, this));
  }

private:
    void grabAndPublishFrame() {

        // Grab frames from RealSense device
        cv::Mat color_image, depth_image;

        // Check if we succesfully grabbed a frame in 10ms
        if (_camera.grabFrames(color_image, depth_image, _timestamp) == false) {
            RCLCPP_ERROR(this->get_logger(), "Dropped frame longing than RealSense API timeout");
            return;
        }

        // Convert OpenCV matrices to ROS Image messages
        sensor_msgs::msg::Image::SharedPtr color_msg = 
            cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, color_image).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr depth_msg = 
            cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, depth_image).toImageMsg();

        // Publish the depth and color frames
        rclcpp::Time t = rclcpp::Time(_timestamp);
        color_msg->header.stamp = t;
        depth_msg->header.stamp = t;
        color_msg->header.frame_id = _frame_id;
        depth_msg->header.frame_id = _frame_id;
        _pub_color->publish(*color_msg);
        _pub_depth->publish(*depth_msg);

        // Publish the RGBD message
        realsense2_camera_msgs::msg::RGBD rgbd_msg;
        rgbd_msg.header.stamp = t;
        rgbd_msg.rgb_camera_info.header.stamp = t;
        rgbd_msg.depth_camera_info.header.stamp = t;
        rgbd_msg.header.frame_id = _frame_id;
        rgbd_msg.rgb = *color_msg;
        rgbd_msg.depth = *depth_msg;
        _pub_rgbd->publish(rgbd_msg);
    }
    
    std::string _frame_id;
    int _width;
    int _height;
    int _fps;
    uint64_t _timestamp;
    std::string _ws_path;

    RSCamera _camera;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_depth;
    rclcpp::Publisher<realsense2_camera_msgs::msg::RGBD>::SharedPtr _pub_rgbd;
    rclcpp::TimerBase::SharedPtr _timer;
};

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(realsense_camera::RSCameraNodelet)
