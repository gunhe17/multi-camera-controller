#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <functional>
#include <mfapi.h>


struct Config {
    std::vector<int> camera_indexes = {0}; // 여러 개의 인덱스를 허용하도록 변경
    int frame_width = 1280;
    int frame_height = 720;
    int frame_rate = 30;
    int warmup_time = 10;
    int duration_time = 10;

    std::string pixel_format = "YUY2";
    GUID pixel_format_guid = MFVideoFormat_YUY2;

    std::string ffmpeg_path = "ffmpeg";
    std::string output_path = "output.mp4";
};


Config parse_args(int argc, char* argv[]) {
    Config config;

    const std::unordered_map<std::string, GUID> pixel_format_map = {
        { "YUY2", MFVideoFormat_YUY2 },
        { "MJPG", MFVideoFormat_MJPG },
        { "NV12", MFVideoFormat_NV12 }
    };

    std::unordered_map<std::string, std::function<void(const std::string&)>> handlers = {
        { "--camera_indexes", [&](const std::string& v) {
            config.camera_indexes.clear();
            std::stringstream ss(v);
            std::string item;
            while (std::getline(ss, item, ',')) {
                try {
                    config.camera_indexes.push_back(std::stoi(item));
                } catch (const std::exception& e) {
                    std::cout << "[Error] camera_indexes 항목 파싱 실패: " << item << " (" << e.what() << ")\n";
                }
            }
        }},
        { "--frame_rate",   [&](const std::string& v) { config.frame_rate = std::stoi(v); } },
        { "--warmup_time",  [&](const std::string& v) { config.warmup_time = std::stoi(v); } },
        { "--duration_time", [&](const std::string& v) { config.duration_time = std::stoi(v); } },
        { "--pixel_format", [&](const std::string& v) {
            config.pixel_format = v;
            auto it = pixel_format_map.find(v);
            if (it != pixel_format_map.end()) {
                config.pixel_format_guid = it->second;
            } else {
                std::cout << "[Warning] 지원하지 않는 pixel_format: " << v << "\n";
                std::cout << "          사용 가능한 포맷: YUY2, MJPG, NV12\n";
                config.pixel_format = "YUY2";
                config.pixel_format_guid = MFVideoFormat_YUY2;
            }
        }},
        { "--resolution", [&](const std::string& v) {
            if (v == "720p") {
                config.frame_width = 1280;
                config.frame_height = 720;
            } else if (v == "1080p") {
                config.frame_width = 1920;
                config.frame_height = 1080;
            } else if (v == "4k" || v == "2160p") {
                config.frame_width = 3840;
                config.frame_height = 2160;
            } else {
                std::cout << "[Warning] 지원하지 않는 해상도: " << v << "\n";
                std::cout << "          사용 가능한 해상도: 720p, 1080p, 4k\n";
            }
        }},
        { "--ffmpeg_path", [&](const std::string& v) { config.ffmpeg_path = v; } },
        { "--output_path", [&](const std::string& v) { config.output_path = v; } }
    };

    for (int i = 1; i < argc; i += 2) {
        if (i + 1 >= argc) {
            std::cout << "[Warning] 값이 누락된 인자: " << argv[i] << "\n";
            break;
        }

        std::string key = argv[i];
        std::string value = argv[i + 1];

        auto it = handlers.find(key);
        if (it != handlers.end()) {
            try {
                it->second(value);
            } catch (const std::exception& e) {
                std::cout << "[Error] 인자 파싱 실패 (" << key << "): " << e.what() << "\n";
            }
        } else {
            std::cout << "[Warning] 알 수 없는 인자: " << key << "\n";
        }
    }

    std::cout << "\n[Config 설정 요약]\n";
    std::cout << "카메라 인덱스: ";
    for (int idx : config.camera_indexes) std::cout << idx << " ";
    std::cout << "\n해상도: " << config.frame_width << "x" << config.frame_height << "\n";
    std::cout << "프레임 레이트: " << config.frame_rate << " FPS\n";
    std::cout << "워밍업 시간: " << config.warmup_time << " 초\n";
    std::cout << "촬영 시간: " << config.duration_time << " 초\n";
    std::cout << "픽셀 포맷: " << config.pixel_format << "\n";
    std::cout << "FFmpeg 경로: " << config.ffmpeg_path << "\n";
    std::cout << "출력 경로: " << config.output_path << "\n\n";

    return config;
}
