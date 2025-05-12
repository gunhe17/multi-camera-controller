// main.cpp

#include "capture/camera_manager.hpp"
#include "include/common.hpp"

#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <filesystem>

int main() {
    std::filesystem::create_directories("output");

    std::cout << "멀티캠 동기 촬영 시작\n";

    // 1. 각 카메라 설정 정의
    std::vector<CaptureConfig> configs = {
        {
            0,
            1280,
            720,
            30,
            "ffmpeg",
            "output/cam_0.mp4",        // ffmpeg 연결 시 사용 예정
            "output/cam_0_log.csv"
        },
        {
            1,
            1280,
            720,
            30,
            "ffmpeg",
            "output/cam_1.mp4",
            "output/cam_1_log.csv"
        }
        // 필요 시 추가
    };

    std::cout << "설정된 카메라 목록:\n";
    for (const auto& cfg : configs) {
        std::cout << " - Camera ID: " << cfg.camera_id
                  << ", Resolution: " << cfg.width << "x" << cfg.height
                  << ", FPS: " << cfg.fps
                  << ", Output: " << cfg.output_filename
                  << ", Log: " << cfg.log_filename
                  << "\n";
    }

    // 2. 카메라 매니저 생성 및 초기화
    CameraManager manager(configs);
    if (!manager.initialize()) {
        std::cerr << "장치 초기화 실패\n";
        return 1;
    }
    std::cout << "모든 카메라 장치 초기화 완료\n";


    // 3. 모든 카메라 동시 시작
    manager.start_all();

    // 4. 일정 시간 대기 (예: 10초간 촬영)
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // 5. 촬영 종료 및 정리
    manager.stop_all();

    std::cout << "촬영 종료! 로그 파일을 확인하세요.\n";
    return 0;
}
