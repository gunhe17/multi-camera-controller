// capture/camera_device.hpp

#ifndef CAMERA_DEVICE_HPP
#define CAMERA_DEVICE_HPP

#include "../include/common.hpp"
#include "timestamp_logger.hpp"

#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mfobjects.h>
#include <mferror.h>
#include <comdef.h>

#include <thread>
#include <atomic>
#include <iostream>
#include <chrono>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

class CameraDevice {
public:
    CameraDevice(const CaptureConfig& config, TimestampLogger& logger)
        : config_(config), logger_(logger), is_running_(false) {}

    bool initialize() {
        // IMFSourceReader ?λ뜃由??(?곕????닌뗭겱)
        std::cout << "[Camera " << config_.camera_id << "] Initialized (mock).\n";
        return true;
    }

    void start(std::chrono::steady_clock::time_point T0) {
        is_running_ = true;
        capture_thread_ = std::thread([this, T0]() {
            int frame_index = 0;
            while (is_running_) {
                // ?逾?MOCK: ?袁⑥쟿????륁춿 ??筌?(??쇱젫??IMFSourceReader ??GetSample)
                std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ??30fps
                auto now = std::chrono::steady_clock::now();

                // timestamp ?④쑴沅?
                uint64_t ts_ns = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()
                );
                auto rel_us = std::chrono::duration_cast<std::chrono::microseconds>(now - T0).count();

                FrameMeta meta{
                    frame_index,
                    ts_ns,
                    rel_us,
                    config_.camera_id
                };
                logger_.log(meta);

                // TODO: raw frame ?곕뗄????ffmpeg stdin??곗쨮 ?袁⑤뼎 ??됱젟
                frame_index++;
            }
        });
    }

    void stop() {
        is_running_ = false;
        if (capture_thread_.joinable())
            capture_thread_.join();
    }

private:
    CaptureConfig config_;
    TimestampLogger& logger_;
    std::thread capture_thread_;
    std::atomic<bool> is_running_;
};

#endif // CAMERA_DEVICE_HPP
