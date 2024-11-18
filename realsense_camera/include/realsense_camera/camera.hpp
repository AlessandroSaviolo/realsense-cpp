#pragma once

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
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

namespace realsense_camera {

class RSCamera {
public:
    // Global atomic flag to control the loop
    inline static std::atomic<bool> keep_running = true;

    // Signal handler to handle SIGINT
    // Sets the atomic flag to false on receiving an interrupt signal
    inline static void signal_handler(int signal) {
        if (signal == SIGINT) {
            std::cerr << "[RSCamera] Interrupt signal received. Shutting down..." << std::endl;
            keep_running = false;
        }
    }

    // Constructor: Initializes the pipeline and sets the signal handler
    RSCamera() : _pipeline_started(false), _timeout_ms(0.1) {
        std::signal(SIGINT, signal_handler);
    }

    // Destructor: Stops the pipeline if it is active
    ~RSCamera() {
        _pipeline->stop();
    }

    // Initializes the RealSense camera with specified parameters
    // Throws exceptions on invalid input or initialization failure
    void init(const std::string& workspace_path, int width, int height, int fps) {
        std::cout << "[RSCamera] Initializing..." << std::endl;
        try {
            validateInput(width, height, fps);
            _width = width;
            _height = height;
            _fps = fps;
            _ctx = std::make_shared<rs2::context>();
            discoverDevices();
            configurePipeline(workspace_path);
            std::cout << "[RSCamera] Initialization complete." << std::endl;
        } catch (const rs2::error& e) {
            logError("RealSense", e.get_failed_function(), e.get_failed_args(), e.what());
        } catch (const std::exception& e) {
            logError("Exception", e.what());
        }
    }

    // Grabs frames from the RealSense camera and fills the provided color and depth images
    // Returns false if pipeline is not started or frames could not be retrieved
    bool grabFrames(cv::Mat& color_image, cv::Mat& depth_image, uint64_t& timestamp) {
        if (!_pipeline_started) {
            std::cerr << "[RSCamera] Pipeline is not started." << std::endl;
            return false;
        }

        bool success = _pipeline->try_wait_for_frames(&_frames, _timeout_ms);
        if (success) {
            try {
                _frames = _align_to_color->process(_frames);
                rs2::frame color_frame = _frames.get_color_frame();
                rs2::frame depth_frame = _frames.get_depth_frame();

                timestamp = frameSystemTimeSec(color_frame);

                color_image = cv::Mat(cv::Size(_width, _height), CV_8UC3, const_cast<void*>(color_frame.get_data()), cv::Mat::AUTO_STEP);
                depth_image = cv::Mat(cv::Size(_width, _height), CV_16U, const_cast<void*>(depth_frame.get_data()), cv::Mat::AUTO_STEP);
            } catch (const std::exception& e) {
                logError("Frame Processing", e.what());
                return false;
            }
        }
        return success;
    }

    // Sets the timeout duration for frame retrieval
    // Throws an exception if the timeout value is invalid
    void setTimeout(int timeout_ms) {
        if (timeout_ms <= 0) {
            throw std::invalid_argument("Timeout must be greater than zero.");
        }
        _timeout_ms = timeout_ms;
    }

    // Retrieves the intrinsic camera parameters
    // Outputs intrinsic parameters like width, height, and focal lengths
    void getParameters(float& width, float& height, float& fx, float& fy, float& cx, float& cy) const {
        width = _intrinsics.width;
        height = _intrinsics.height;
        fx = _intrinsics.fx;
        fy = _intrinsics.fy;
        cx = _intrinsics.ppx;
        cy = _intrinsics.ppy;
    }

    // Retrieves normalized intrinsic camera parameters
    // Outputs normalized values for focal lengths and principal point offsets
    void getNormalizedParameters(float& width, float& height, float& fx, float& fy, float& cx, float& cy) const {
        width = _intrinsics.width;
        height = _intrinsics.height;
        fx = _intrinsics.fx / _intrinsics.width;
        fy = _intrinsics.fy / _intrinsics.height;
        cx = _intrinsics.ppx / _intrinsics.width;
        cy = _intrinsics.ppy / _intrinsics.height;
    }

private:
    // Validates input parameters for camera initialization
    // Ensures width, height, and fps values are greater than zero
    void validateInput(int width, int height, int fps) const {
        if (width <= 0 || height <= 0 || fps <= 0) {
            throw std::invalid_argument("Invalid width, height, or fps parameters.");
        }
    }

    // Discovers connected RealSense devices and selects the first available device
    void discoverDevices() {
        while (keep_running) {
            _device_list = _ctx->query_devices();
            if (_device_list.size() == 0) {
                std::cerr << "[RSCamera] No RealSense devices found. Retrying..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            std::cout << "[RSCamera] RealSense device(s) found." << std::endl;
            _dev = _device_list.front();
            break;
        }
    }

    // Configures the RealSense pipeline and loads the specified configuration
    // Sets up streams for depth and color and applies advanced configuration if available
    void configurePipeline(const std::string& workspace_path) {
        if (!keep_running) {
            std::cerr << "[RSCamera] Initialization aborted by signal." << std::endl;
            return;
        }

        try {
            _pipeline = std::make_shared<rs2::pipeline>();
            _cfg = std::make_shared<rs2::config>();

            std::cout << "[RSCamera] Configuring streams..." << std::endl;
            _cfg->enable_stream(RS2_STREAM_DEPTH, 0, _width, _height, RS2_FORMAT_Z16, _fps);
            _cfg->enable_stream(RS2_STREAM_COLOR, 0, _width, _height, RS2_FORMAT_RGB8, _fps);

            loadConfiguration(workspace_path);

            std::cout << "[RSCamera] Starting pipeline..." << std::endl;
            _pipeline_profile = _pipeline->start(*_cfg);
            _pipeline_started = true;

            retrieveIntrinsics();
            _align_to_color = std::make_shared<rs2::align>(RS2_STREAM_COLOR);
        } catch (const rs2::error& e) {
            logError("Pipeline Configuration", e.get_failed_function(), e.get_failed_args(), e.what());
            throw;
        } catch (const std::exception& e) {
            logError("Pipeline Configuration", e.what());
            throw;
        }
    }

    // Loads the RealSense configuration from a specified file
    // Applies advanced mode settings if the configuration file is valid
    void loadConfiguration(const std::string& workspace_path) {
        std::string config_path = workspace_path + "/src/realsense-cpp/realsense_camera/config/rs_config.json";
        std::ifstream file(config_path);

        if (!file.good()) {
            logError("Configuration", "", "", "Failed to open configuration file: " + config_path);
            return;
        }

        try {
            std::string config_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            auto profile = _cfg->resolve(*_pipeline);
            if (auto advanced = profile.get_device().as<rs400::advanced_mode>()) {
                advanced.load_json(config_content);
                std::cout << "[RSCamera] Configuration loaded successfully." << std::endl;
            }
        } catch (const rs2::error& e) {
            logError("Configuration Loading", e.get_failed_function(), e.get_failed_args(), e.what());
        } catch (const std::exception& e) {
            logError("Configuration Loading", e.what());
        }
    }

    // Retrieves intrinsic parameters for the camera from the pipeline profile
    // Stores the retrieved intrinsics and copies distortion coefficients
    void retrieveIntrinsics() {
        try {
            auto stream = _pipeline_profile.get_stream(RS2_STREAM_COLOR);
            _intrinsics = stream.as<rs2::video_stream_profile>().get_intrinsics();
            _distortion_model = _intrinsics.model;
            std::memcpy(_distortion_coeffs, _intrinsics.coeffs, sizeof(_distortion_coeffs));
        } catch (const std::exception& e) {
            logError("Retrieve Intrinsics", e.what());
            throw;
        }
    }

    // Converts the frame timestamp from milliseconds to nanoseconds
    // Returns the timestamp in nanoseconds
    uint64_t frameSystemTimeSec(rs2::frame frame) const {
        double timestamp_ms = frame.get_timestamp();
        double int_part_ms, fract_part_ms;
        fract_part_ms = modf(timestamp_ms, &int_part_ms);
        constexpr uint64_t milli_to_nano = 1000000;
        uint64_t int_part_ns = static_cast<uint64_t>(int_part_ms) * milli_to_nano;
        uint64_t fract_part_ns = static_cast<uint64_t>(std::round(fract_part_ms * milli_to_nano));
        return int_part_ns + fract_part_ns;
    }

    // Logs error messages to the standard error output
    // Takes the error type, function name, arguments, and message as parameters
    void logError(const std::string& error_type, const std::string& function = "", const std::string& args = "", const std::string& message = "") const {
        std::cerr << "[RSCamera] " << error_type << " error";
        if (!function.empty()) {
            std::cerr << " in function: " << function << "(" << args << ")";
        }
        std::cerr << ": " << message << std::endl;
    }

    // Camera properties
    int _width{0};
    int _height{0};
    int _fps{0};
    double _timeout_ms{0.1};
    bool _pipeline_started{false};

    // RealSense components
    std::shared_ptr<rs2::context> _ctx;
    rs2::device _dev;
    rs2::device_list _device_list;
    std::shared_ptr<rs2::config> _cfg;
    std::shared_ptr<rs2::pipeline> _pipeline;
    rs2::pipeline_profile _pipeline_profile;
    rs2::frameset _frames;
    std::shared_ptr<rs2::align> _align_to_color;

    // Camera intrinsics
    rs2_intrinsics _intrinsics;
    rs2_distortion _distortion_model;
    float _distortion_coeffs[5]{};
};

} // namespace realsense_camera
