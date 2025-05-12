// capture/camera_manager.hpp

#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP

#include "../include/common.hpp"
#include "camera_device.hpp"
#include "timestamp_logger.hpp"

#include <vector>
#include <memory>
#include <chrono>

class CameraManager {
public:
    CameraManager(const std::vector<CaptureConfig>& configs)
        : configs_(configs) {}

    bool initialize() {
        for (const auto& config : configs_) {
            // 로그 파일 생성
            auto logger = std::make_unique<TimestampLogger>(config.log_filename);
            auto device = std::make_unique<CameraDevice>(config, *logger);

            if (!device->initialize()) {
                std::cerr << "Camera " << config.camera_id << " 초기화 실패\n";
                return false;
            }

            loggers_.push_back(std::move(logger));
            devices_.push_back(std::move(device));
        }
        return true;
    }

    void start_all() {
        T0_ = std::chrono::steady_clock::now();
        for (auto& device : devices_) {
            device->start(T0_);
        }
    }

    void stop_all() {
        for (auto& device : devices_) {
            device->stop();
        }
        for (auto& logger : loggers_) {
            logger->stop();
        }
    }

private:
    std::vector<CaptureConfig> configs_;
    std::vector<std::unique_ptr<TimestampLogger>> loggers_;
    std::vector<std::unique_ptr<CameraDevice>> devices_;
    std::chrono::steady_clock::time_point T0_;
};

#endif // CAMERA_MANAGER_HPP
