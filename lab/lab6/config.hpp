#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <functional>
#include <mfapi.h>


struct Config {
    int camera_index = 0;
    int frame_width = 1280;
    int frame_height = 720;
    int frame_rate = 30;
    GUID pixel_format = MFVideoFormat_MJPG;
    int duration = 2;
};

Config parse_args(int argc, char* argv[]) {
    Config config;

    std::unordered_map<std::string, std::function<void(const std::string&)>> handlers = {
        { "--camera_index", [&](const std::string& v) { config.camera_index = std::stoi(v); } },
        { "--frame_rate",   [&](const std::string& v) { config.frame_rate = std::stoi(v); } },
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
        { "--pixel_format", [&](const std::string& v) { 
            if (v == "NV12") {
                config.pixel_format = MFVideoFormat_NV12;
            } else if (v == "MJPG") {
                config.pixel_format = MFVideoFormat_MJPG;
            } else {
                std::cout << "[Warning] 지원하지 않는 형식: " << v << "\n";
                std::cout << "          사용 가능한 해상도: NV12, YUY2\n";
            }
        }},
        { "--duration", [&](const std::string& v) { 
            try {
                config.duration = std::stoi(v);
                if (config.duration <= 0) {
                    throw std::invalid_argument("지속 시간은 양수여야 합니다.");
                }
            } catch (const std::exception& e) {
                std::cout << "[Error] 지속 시간 파싱 실패: " << e.what() << "\n";
            }
        }},
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

    return config;
}