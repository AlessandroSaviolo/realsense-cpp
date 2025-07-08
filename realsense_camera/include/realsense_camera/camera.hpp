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
#include <unordered_map>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

namespace realsense_camera {

class RSCamera {
public:
    inline static std::atomic<bool> keep_running = true;

    inline static void signal_handler(int signal) {
        if (signal == SIGINT) {
            std::cerr << "[RSCamera] Interrupt signal received. Shutting down..." << std::endl;
            keep_running = false;
        }
    }

    RSCamera() : _timeout_ms(0.001) {
        std::signal(SIGINT, signal_handler);
    }

    ~RSCamera() {
        for (auto& [serial, cam] : _cameras) {
            try {
                cam.pipeline->stop();
            } catch (...) {
                std::cerr << "[RSCamera] Warning: failed to stop pipeline for " << serial << std::endl;
            }
        }
    }

    void init(const std::string& workspace_path, int width, int height, int fps, const std::vector<std::string>& serial_numbers) {
        _width = width;
        _height = height;
        _fps = fps;
        _ctx = std::make_shared<rs2::context>();

        auto devices = _ctx->query_devices();
        std::cout << "[RSCamera] Connected RealSense devices:" << std::endl;
        for (auto&& dev : devices) {
            std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
            std::cout << "  - " << name << " (Serial: " << serial << ")" << std::endl;
        }

        for (const auto& serial : serial_numbers) {
            bool found = false;
            for (auto&& dev : devices) {
                if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                throw std::runtime_error("[RSCamera] Device with serial " + serial + " not found.");
            }

            rs2::config cfg;
            cfg.enable_device(serial);
            cfg.enable_stream(RS2_STREAM_DEPTH, 0, _width, _height, RS2_FORMAT_Z16, _fps);
            cfg.enable_stream(RS2_STREAM_COLOR, 0, _width, _height, RS2_FORMAT_RGB8, _fps);

            auto pipeline = std::make_shared<rs2::pipeline>();
            auto profile = pipeline->start(cfg);
            auto align = std::make_shared<rs2::align>(RS2_STREAM_COLOR);

            rs2::video_stream_profile color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            rs2_intrinsics intrinsics = color_profile.get_intrinsics();

            CameraInstance instance;
            instance.pipeline = pipeline;
            instance.profile = profile;
            instance.align = align;
            instance.intrinsics = intrinsics;

            _cameras[serial] = std::move(instance);

            std::cout << "[RSCamera] Started camera with serial: " << serial << std::endl;
            auto streams = profile.get_streams();
            for (const auto& s : streams) {
                std::cout << "[RSCamera] Stream: " << s.stream_name() << " " << s.fps() << " Hz, "
                        << s.as<rs2::video_stream_profile>().width() << "x"
                        << s.as<rs2::video_stream_profile>().height() << std::endl;
            }
        }
    }

    bool grabFrames(const std::string& serial, cv::Mat& color_image, cv::Mat& depth_image, uint64_t& timestamp) {
        auto it = _cameras.find(serial);
        if (it == _cameras.end()) {
            std::cerr << "[RSCamera] Serial not found: " << serial << std::endl;
            return false;
        }

        CameraInstance& cam = it->second;
        bool success = cam.pipeline->try_wait_for_frames(&cam.frames, _timeout_ms);
        if (!success) return false;

        try {
            // cam.frames = cam.pipeline->wait_for_frames();
            cam.frames = cam.align->process(cam.frames);
            rs2::frame color = cam.frames.get_color_frame();
            rs2::frame depth = cam.frames.get_depth_frame();

            timestamp = frameSystemTimeSec(color);

            color_image = cv::Mat(cv::Size(_width, _height), CV_8UC3, const_cast<void*>(color.get_data()), cv::Mat::AUTO_STEP);
            depth_image = cv::Mat(cv::Size(_width, _height), CV_16U, const_cast<void*>(depth.get_data()), cv::Mat::AUTO_STEP);
        } catch (const std::exception& e) {
            logError("GrabFrames", e.what());
            return false;
        }

        return true;
    }

    void setTimeout(int timeout_ms) {
        if (timeout_ms <= 0) {
            throw std::invalid_argument("Timeout must be greater than zero.");
        }
        _timeout_ms = timeout_ms;
    }

    void getParameters(const std::string& serial, float& width, float& height, float& fx, float& fy, float& cx, float& cy) const {
        const auto& intr = _cameras.at(serial).intrinsics;
        width = intr.width;
        height = intr.height;
        fx = intr.fx;
        fy = intr.fy;
        cx = intr.ppx;
        cy = intr.ppy;
    }

    void getNormalizedParameters(const std::string& serial, float& width, float& height, float& fx, float& fy, float& cx, float& cy) const {
        const auto& intr = _cameras.at(serial).intrinsics;
        width = intr.width;
        height = intr.height;
        fx = intr.fx / width;
        fy = intr.fy / height;
        cx = intr.ppx / width;
        cy = intr.ppy / height;
    }

private:
    struct CameraInstance {
        std::shared_ptr<rs2::pipeline> pipeline;
        rs2::pipeline_profile profile;
        std::shared_ptr<rs2::align> align;
        rs2_intrinsics intrinsics;
        rs2::frameset frames;
    };

    uint64_t frameSystemTimeSec(rs2::frame frame) const {
        double timestamp_ms = frame.get_timestamp();
        double int_part_ms, fract_part_ms;
        fract_part_ms = modf(timestamp_ms, &int_part_ms);
        constexpr uint64_t milli_to_nano = 1000000;
        return static_cast<uint64_t>(int_part_ms) * milli_to_nano +
               static_cast<uint64_t>(std::round(fract_part_ms * milli_to_nano));
    }

    void logError(const std::string& context, const std::string& msg) const {
        std::cerr << "[RSCamera] " << context << " error: " << msg << std::endl;
    }

    int _width;
    int _height;
    int _fps;
    double _timeout_ms;
    std::shared_ptr<rs2::context> _ctx;
    std::unordered_map<std::string, CameraInstance> _cameras;
};

} // namespace realsense_camera
