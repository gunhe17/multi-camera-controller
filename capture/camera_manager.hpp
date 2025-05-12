#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP

#include "../include/common.hpp"
#include "camera_device.hpp"
#include "timestamp_logger.hpp"

#include <vector>
#include <memory>
#include <chrono>
#include <iostream>

class CameraManager {
public:
    CameraManager(const std::vector<CaptureConfig>& configs)
        : configs_(configs) {}

    bool initialize() {
        std::cout << "[CameraManager] 초기화 시작\n";

        for (const auto& config : configs_) {
            std::cout << " - Camera " << config.camera_id << " 로그 파일 생성 중: " << config.log_filename << "\n";
            auto logger = std::make_unique<TimestampLogger>(config.log_filename);

            std::cout << " - Camera " << config.camera_id << " 장치 생성 및 초기화 중\n";
            auto device = std::make_unique<CameraDevice>(config, *logger);

            if (!device->initialize()) {
                std::cerr << "   -> Camera " << config.camera_id << " 초기화 실패\n";
                return false;
            }

            std::cout << "   -> Camera " << config.camera_id << " 초기화 완료\n";

            loggers_.push_back(std::move(logger));
            devices_.push_back(std::move(device));
        }

        std::cout << "[CameraManager] 모든 카메라 초기화 완료\n";
        return true;
    }

    void start_all() {
        std::cout << "[CameraManager] 촬영 시작\n";
        T0_ = std::chrono::steady_clock::now();
        for (auto& device : devices_) {
            device->start(T0_);
        }
    }

    void stop_all() {
        std::cout << "[CameraManager] 촬영 종료 요청\n";
        for (auto& device : devices_) {
            device->stop();
        }
        std::cout << "[CameraManager] 모든 장치 정지 완료\n";

        for (auto& logger : loggers_) {
            logger->stop();
        }
        std::cout << "[CameraManager] 로그 저장 완료\n";
    }

private:
    std::vector<CaptureConfig> configs_;
    std::vector<std::unique_ptr<TimestampLogger>> loggers_;
    std::vector<std::unique_ptr<CameraDevice>> devices_;
    std::chrono::steady_clock::time_point T0_;
};

#endif // CAMERA_MANAGER_HPP
