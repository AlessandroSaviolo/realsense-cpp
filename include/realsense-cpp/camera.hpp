#ifndef CAMERA_NODELET_CAMERA_HPP
#define CAMERA_NODELET_CAMERA_HPP

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <iostream>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <atomic>
#include <csignal>
#include <fstream>

#ifdef QUADROTOR_NU
    #define SAVED_QUADROTOR_NU QUADROTOR_NU
    #undef QUADROTOR_NU
#endif
#include <opencv2/opencv.hpp>
#ifdef SAVED_QUADROTOR_NU
    #define QUADROTOR_NU SAVED_QUADROTOR_NU
    #undef SAVED_QUADROTOR_NU
#endif

namespace arpl_realsense
{
std::atomic<bool> keep_running(true); // Global atomic flag to control the loop

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "[RSCamera] Interrupt signal received. Shutting down..." << std::endl;
        keep_running = false;
    }
}

class RSCamera {
public:
    RSCamera() : _pipeline_started(false), _timeout_ms(0.1) { 
        std::signal(SIGINT, signal_handler); // Set up signal handler
    }
    ~RSCamera() { 
        if (_pipeline && _pipeline_profile) { 
            _pipeline->stop(); 
        } 
    }

    void init(std::string workspace_path, int width, int height, int fps) {
        std::cout << "[RSCamera] Initializing ... " << std::endl;
        try {
            _width = width;
            _height = height;
            _fps = fps;

            _ctx = std::make_shared<rs2::context>();

            while (keep_running) {
                _device_list = _ctx->query_devices();
                if (_device_list.size() == 0) {
                    std::cout << "[RSCamera] No RealSense devices found." << std::endl;
                } else {
                    std::cout << "[RSCamera] RealSense device(s) found." << std::endl;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::seconds(1)); // Throttle the loop to run every second
            }

            if (keep_running) {     // Only proceed if not interrupted
                _dev = _device_list.front();
                _pipeline = std::make_shared<rs2::pipeline>();
                _cfg = std::make_shared<rs2::config>();

                std::cout << "[RSCamera] Configuring streams..." << std::endl;
                _cfg->enable_stream(RS2_STREAM_DEPTH, 0, _width, _height, RS2_FORMAT_Z16, _fps);
                _cfg->enable_stream(RS2_STREAM_COLOR, 0, _width, _height, RS2_FORMAT_RGB8, _fps);

                std::cout << "[RSCamera] Loading HQ configuration saved at " << workspace_path << "/src/arpl_realsense/config/rs_d455_config.json" << std::endl;
                std::ifstream file(workspace_path + "/src/arpl_realsense/config/rs_d455_config.json");
                if (file.good()) {
                    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
                    auto prof = _cfg->resolve(*_pipeline);
                    if (auto advanced = prof.get_device().as<rs400::advanced_mode>()) {
                        advanced.load_json(str);
                    }
                    std::cout << "[RSCamera] Successfully loaded configuration file." << std::endl;
                } else {
                    std::cerr << "[RSCamera] Failed to open file." << std::endl;
                }

                std::cout << "[RSCamera] Starting pipeline..." << std::endl;
                _pipeline_profile = _pipeline->start(*_cfg);
                _pipeline_started = true;

                // Get the stream intrinsics
                auto stream = _pipeline_profile.get_stream(RS2_STREAM_COLOR);
                _intrinsics = stream.as<rs2::video_stream_profile>().get_intrinsics();
                _distortion_model = _intrinsics.model;
                std::memcpy(_distortion_coeffs, _intrinsics.coeffs, 5 * sizeof(float));

                // Output the intrinsics and distortion model
                std::cout << "[RSCamera] Camera Intrinsics:" << std::endl;
                std::cout << "[RSCamera] Width: " << _intrinsics.width << std::endl;
                std::cout << "[RSCamera] Height: " << _intrinsics.height << std::endl;
                std::cout << "[RSCamera] FX: " << _intrinsics.fx << std::endl;
                std::cout << "[RSCamera] FY: " << _intrinsics.fy << std::endl;
                std::cout << "[RSCamera] CX: " << _intrinsics.ppx << std::endl;
                std::cout << "[RSCamera] CY: " << _intrinsics.ppy << std::endl;

                std::cout << "[RSCamera] Distortion Model: " << _distortion_model << std::endl;
                std::cout << "[RSCamera] Distortion Coefficients: ";
                for (int i = 0; i < 5; ++i) {
                    std::cout << _distortion_coeffs[i] << " ";
                }
                std::cout << std::endl;

                _align_to_color = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
                std::cout << "[RSCamera] Initialization complete." << std::endl;
            }
        } 
        catch (const rs2::error& e) {
            std::cerr << "[RSCamera] RealSense error calling: " << e.get_failed_function()
                    << "(" << e.get_failed_args() << "): " << e.what() << std::endl;
        } 
        catch (const std::exception& e) {
            std::cerr << "[RSCamera] Exception: " << e.what() << std::endl;
        }
    }

    uint64_t frameSystemTimeSec(rs2::frame frame) {
        double timestamp_ms = frame.get_timestamp();
        double int_part_ms, fract_part_ms;
        fract_part_ms = modf(timestamp_ms, &int_part_ms);
        static constexpr uint64_t milli_to_nano = 1000000;
        uint64_t int_part_ns = static_cast<uint64_t>(int_part_ms) * milli_to_nano;
        uint64_t fract_part_ns = static_cast<uint64_t>(std::round(fract_part_ms * milli_to_nano));
        return int_part_ns + fract_part_ns;
    }

    // Return a cv::Mat color/depth and timestamp.
    // Timestamp is uint64_t. uint64_t to convert to timestamp is just rclcpp::Time(timestamp)
    // This is a nanosecond fixpoint of timestamp
    // Return success or failure on timeout
    bool grabFrames(cv::Mat& color_image, cv::Mat& depth_image, uint64_t& timestamp) {
        bool success = _pipeline->try_wait_for_frames(&_frames, _timeout_ms);
        if (success) {
            _frames = _align_to_color->process(_frames);
            rs2::frame color_frame = _frames.get_color_frame();
            rs2::frame depth_frame = _frames.get_depth_frame();

            // rs2::frame filtered = depth_frame;
            // filtered = _dec_filter.process(filtered);
            // filtered = _thr_filter.process(filtered);
            // filtered = _spat_filter.process(filtered);
            // filtered = _temp_filter.process(filtered);

            timestamp = frameSystemTimeSec(color_frame);
            color_image = cv::Mat(cv::Size(_width, _height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            depth_image = cv::Mat(cv::Size(_width, _height), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        }
        return success;
    }

    void setTimeOut(int n) {
        _timeout_ms = n;
    }

    // Getters for the camera intrinsics
    void getParameters(float &width, float &height, float &fx, float &fy, float &cx, float &cy) { 
        width = _intrinsics.width; 
        height = _intrinsics.height;
        fx = _intrinsics.fx; 
        fy = _intrinsics.fy;
        cx = _intrinsics.ppx;
        cy = _intrinsics.ppy;
    }
    void getNormalizedParameters(float &width, float &height, float &fx, float &fy, float &cx, float &cy) { 
        width = _intrinsics.width; 
        height = _intrinsics.height;
        fx = _intrinsics.fx / _intrinsics.width; 
        fy = _intrinsics.fy / _intrinsics.height;
        cx = _intrinsics.ppx / _intrinsics.width;
        cy = _intrinsics.ppy / _intrinsics.height;
    }

private:
    int _width;
    int _height;
    int _fps;
    int _timeout_ms;
    bool _pipeline_started;

    // rs2::decimation_filter _dec_filter;  // Decimation - reduces depth frame density
    // rs2::threshold_filter _thr_filter;   // Threshold  - removes values outside recommended range
    // rs2::spatial_filter _spat_filter;    // Spatial    - edge-preserving spatial smoothing
    // rs2::temporal_filter _temp_filter;   // Temporal   - reduces temporal noise

    std::shared_ptr<rs2::context> _ctx;
    rs2::device _dev;
    rs2::device_list _device_list;
    std::shared_ptr<rs2::config> _cfg;
    std::shared_ptr<rs2::pipeline> _pipeline;
    rs2::pipeline_profile _pipeline_profile;

    rs2::frameset _frames; 

    // Align depth to color
    // Note: viceversa would cause the color to have invalid values due to the depth artifacts
    std::shared_ptr<rs2::align> _align_to_color;

    // Camera intrinsics and distortion model
    rs2_intrinsics _intrinsics;
    rs2_distortion _distortion_model;
    float _distortion_coeffs[5];
};

}

#endif // CAMERA_NODELET_CAMERA_HPP
