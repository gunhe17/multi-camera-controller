#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP

#include "../include/common.hpp"
#include "camera_device.hpp"
#include "timestamp_logger.hpp"

#include <vector>
#include <memory>
#include <chrono>
#include <mutex>
#include <condition_variable>

class CameraManager {
public:
    CameraManager(const std::vector<CaptureConfig>& configs)
        : configs_(configs),
          sync_cv_(std::make_shared<std::condition_variable>()),
          sync_mutex_(std::make_shared<std::mutex>()),
          sync_go_flag_(std::make_shared<bool>(false)) {}

    bool initialize() {
        for (const auto& config : configs_) {
            auto logger = std::make_unique<TimestampLogger>(config.log_filename);
            auto device = std::make_unique<CameraDevice>(
                config,
                *logger,
                sync_cv_,
                sync_mutex_,
                sync_go_flag_
            );

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

        {
            std::lock_guard<std::mutex> lock(*sync_mutex_);
            *sync_go_flag_ = true;
        }
        sync_cv_->notify_all();
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

    std::shared_ptr<std::condition_variable> sync_cv_;
    std::shared_ptr<std::mutex> sync_mutex_;
    std::shared_ptr<bool> sync_go_flag_;
};

#endif // CAMERA_MANAGER_HPP
