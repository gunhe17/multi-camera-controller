#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <functional>
#include <mfapi.h>


struct Config {
    int camera_index = 0;
    int frame_width = 1;
    int frame_height = 1;
    int frame_rate = 1;
};

Config parse_args(int argc, char* argv[]) {
    Config config;

    std::unordered_map<std::string, std::function<void(const std::string&)>> handlers = {
        { "--camera_index", [&](const std::string& v) { config.camera_index = std::stoi(v); } },
        { "--frame_width",   [&](const std::string& v) { config.frame_rate = std::stoi(v); } },
        { "--frame_height",   [&](const std::string& v) { config.frame_rate = std::stoi(v); } },
        { "--frame_rate",   [&](const std::string& v) { config.frame_rate = std::stoi(v); } },
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