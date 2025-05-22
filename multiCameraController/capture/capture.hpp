#pragma once

#include <vector>
#include <thread>
#include <atomic>
#include <functional>

#include "config/config.hpp"
#include "ffmpeg/ffmpeg.hpp"
#include "capture/mediafoundation/mediafoundation.hpp"


/**
 * Class: Capture
 */

class Capture {
public:
    explicit Capture(const Config& config) : config_(config), is_running_(false) {}

    void start() {
        is_running_ = true;

        for (int camera_index : config_.camera_indexes) {
            threads_.emplace_back([this, camera_index]() {
                this->captureWorker(camera_index);
            });
        }
    }

    void stop() {
        is_running_ = false;

        for (auto& t : threads_) {
            if (t.joinable()) t.join();
        }

        std::cout << "[Capture] 모든 스레드 종료 완료\n";
        MFShutdown();
    }

private:
    Config config_;
    std::atomic<bool> is_running_;
    std::vector<std::thread> threads_;

    void captureWorker(int camera_index) {
        std::cout << "[Capture] 카메라 " << camera_index << " 캡처 시작\n";

        MediaFoundation mediafoundation;

        auto device_opt = mediafoundation.getDevice(camera_index);
        if (!device_opt.has_value()) {
            std::cout << "[Capture] 카메라 " << camera_index << " 장치 획득 실패\n";
            return;
        }
        ComPtr<IMFActivate> device = device_opt.value();    
        std::cout << "[Capture] device 생성 완료\n";

        auto reader_opt = mediafoundation.getSourceReader(device, config_);
        if (!reader_opt.has_value()) {
            std::cout << "[Capture] 카메라 " << camera_index << " 리더 생성 실패\n";
            return;
        }
        ComPtr<IMFSourceReader> source_reader = reader_opt.value();
        std::cout << "[Capture] source_reader 생성 완료\n";

        FFmpegWriter ffmpeg_writer(config_);
        ffmpeg_writer.start();
        std::cout << "[Capture] ffmpeg writer 생성 완료\n";

        uint64_t frame_index = 0;
        while (is_running_) {
            std::cout << "[Capture] 카메라 " << camera_index << " 캡처 중...\n";

            auto result = mediafoundation.record(source_reader, camera_index, frame_index);
            std::cout << "[Capture] 카메라 " << " 샘플 읽기 완료\n";
            if (!result) continue;

            const auto& r = result.value();
            ffmpeg_writer.write(r.data, r.length);
        }

        ffmpeg_writer.stop();
        std::cout << "[Capture] 카메라 " << camera_index << " 캡처 종료\n";
    }
};