#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <sstream>
#include <filesystem>
#include <iomanip>
#include <mfapi.h>

#include "error/error.hpp"

/**
 * class: Config
 */

class Config {
public:
    // Camera settings
    std::vector<int> camera_indices = {0};
    int frame_width = 1280;
    int frame_height = 720;
    int frame_rate = 30;
    std::string pixel_format = "MJPG";
    
    // Recording settings
    int record_duration = 5;
    int consumer_sleep_microseconds = 100;
    int sync_delay_milliseconds = 100;
    
    // External tools
    std::string ffmpeg_path = "ffmpeg";
    
    // Warmup settings
    bool enable_warmup = true;
    int warmup_duration = 5;
    
    // Output settings
    enum class OutputMode { VIDEO, IMAGE };
    OutputMode output_mode = OutputMode::IMAGE;
    std::string output_filename = "output";
    std::string output_directory = ".";
    std::string video_format = "avi";
    std::string image_format = "jpg";
    bool save_timestamp_csv = true;


    // method
    bool validate() {
        // Camera validation
        if (camera_indices.empty()) {
            throw ConfigCameraError("카메라 인덱스가 비어있습니다.");
        }

        if (frame_width <= 0 || frame_height <= 0) {
            throw ConfigCameraError("프레임 크기가 잘못되었습니다.");
        }

        if (frame_rate <= 0) {
            throw ConfigCameraError("프레임 레이트가 잘못되었습니다.");
        }

        // Recording validation
        if (record_duration <= 0) {
            throw ConfigRecordingError("녹화 시간이 잘못되었습니다.");
        }

        if (consumer_sleep_microseconds <= 0) {
            throw ConfigRecordingError("Consumer sleep 시간이 잘못되었습니다.");
        }

        if (sync_delay_milliseconds < 0) {
            throw ConfigRecordingError("동기화 delay 시간이 잘못되었습니다.");
        }

        // Warmup validation
        if (warmup_duration <= 0) {
            throw ConfigWarmupError("Warmup 설정이 잘못되었습니다.");
        }

        // Output validation
        if (output_filename.empty() || output_directory.empty()) {
            throw ConfigOutputError("출력 설정이 잘못되었습니다.");
        }

        if (output_mode == OutputMode::VIDEO && video_format.empty()) {
            throw ConfigOutputError("비디오 포맷이 비어있습니다.");
        }

        if (output_mode == OutputMode::IMAGE && image_format.empty()) {
            throw ConfigOutputError("이미지 포맷이 비어있습니다.");
        }

        try {
            if (!std::filesystem::exists(output_directory)) {
                std::filesystem::create_directories(output_directory);
                std::cout << "[Config] 출력 디렉토리 생성: " << output_directory << "\n";
            }
        } catch (const std::exception& e) {
            throw ConfigExternalError("출력 디렉토리 생성 실패: " + std::string(e.what()));
        }

        return true;
    }

    void print() const {
        std::cout << "[Config] 설정 정보:\n";
        
        // Camera settings
        std::cout << "  카메라 인덱스: ";
        for (size_t i = 0; i < camera_indices.size(); ++i) {
            std::cout << camera_indices[i];
            if (i < camera_indices.size() - 1) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "  해상도: " << frame_width << "x" << frame_height << "\n";
        std::cout << "  프레임 레이트: " << frame_rate << "\n";
        std::cout << "  픽셀 포맷: " << pixel_format << "\n";
        
        // Recording settings
        std::cout << "  녹화 시간: " << record_duration << "초\n";
        std::cout << "  동기화 Delay: " << sync_delay_milliseconds << "ms\n";
        
        // Output settings
        std::cout << "  출력 모드: " << (output_mode == OutputMode::VIDEO ? "VIDEO" : "IMAGE") << "\n";
        std::cout << "  출력 디렉토리: " << output_directory << "\n";
        
        if (output_mode == OutputMode::VIDEO) {
            std::cout << "  비디오 폴더: " << output_directory << "\n";
            std::cout << "  비디오 파일: " << output_filename << "\n";
            std::cout << "  비디오 포맷: " << video_format << "\n";
        } else {
            std::cout << "  이미지 포맷: " << image_format << "\n";
        }
        
        std::cout << "  CSV 저장: " << (save_timestamp_csv ? "ON" : "OFF") << "\n";
        
        // Performance settings
        std::cout << "  FFmpeg 경로: " << ffmpeg_path << "\n";
        std::cout << "  Consumer Sleep: " << consumer_sleep_microseconds << "μs\n";
    }
};

/**
 * class: @parser
 */

class ConfigParser {
public:
    static Config parseArgs(int argc, char* argv[]) {
        Config config;
        
        std::unordered_map<std::string, std::function<void(const std::string&)>> handlers = {
            // Camera settings
            {"--camera_indices", [&](const std::string& v) { 
                config.camera_indices = _parseIntList(v); }},
            {"--frame_width", [&](const std::string& v) { 
                config.frame_width = std::stoi(v); }},
            {"--frame_height", [&](const std::string& v) { 
                config.frame_height = std::stoi(v); }},
            {"--frame_rate", [&](const std::string& v) { 
                config.frame_rate = std::stoi(v); }},
            {"--pixel_format", [&](const std::string& v) { 
                config.pixel_format = v; }},
            
            // Recording settings
            {"--record_duration", [&](const std::string& v) { 
                config.record_duration = std::stoi(v); }},
            {"--consumer_sleep", [&](const std::string& v) { 
                config.consumer_sleep_microseconds = std::stoi(v); }},
            {"--sync_delay", [&](const std::string& v) { 
                config.sync_delay_milliseconds = std::stoi(v); }},
            
            // Output settings
            {"--output_mode", [&](const std::string& v) { 
                config.output_mode = (v == "images") ? 
                    Config::OutputMode::IMAGE : Config::OutputMode::VIDEO; }},
            {"--output_filename", [&](const std::string& v) { 
                config.output_filename = v; }},
            {"--output", [&](const std::string& v) { 
                config.output_filename = v; }},  // 호환성
            {"--output_dir", [&](const std::string& v) { 
                config.output_directory = v; }},
            {"--image_format", [&](const std::string& v) { 
                config.image_format = v; }},
            {"--video_format", [&](const std::string& v) { 
                config.video_format = v; }},
            
            // External tools
            {"--ffmpeg", [&](const std::string& v) { 
                config.ffmpeg_path = v; }},
        };

        for (int i = 1; i < argc; i += 2) {
            if (i + 1 >= argc) {
                throw ConfigInputError("값이 누락된 인자: " + std::string(argv[i]));
            }

            // is valid key
            std::string key = argv[i];
            auto handler = handlers.find(key);
            if (handler == handlers.end()) {
                throw ConfigInputError("알 수 없는 인자: " + key);
            }
            
            // is valid value
            std::string value = argv[i + 1];
            try {
                handler->second(value);
            } catch (const std::exception& e) {
                throw ConfigInputError("인자 파싱 실패 (" + key + "): " + e.what());
            }
        }

        return config;
    }

private:
    static std::vector<int> _parseIntList(const std::string& str) {
        std::vector<int> result;
        std::stringstream ss(str);
        std::string item;
        
        while (std::getline(ss, item, ',')) {
            try {
                result.push_back(std::stoi(item));
            } catch (const std::exception&) {
                std::cerr << "[Warning] 잘못된 숫자 형식: " << item << "\n";
            }
        }
        
        return result;
    }
};

/**
 * class: Manager
 */

class ConfigManager {
public:
    struct Result {
        bool success;
        std::vector<Config> configs;
        
        operator bool() const { return success; }
    };
    
    Config config(int argc, char* argv[]) {
        Config config;
        
        config = ConfigParser::parseArgs(argc, argv);
            
        config.validate();    
        config.print();
            
        return config;
    }
};

/**
 * Global: gonfig
 */

extern Config gonfig;