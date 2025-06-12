#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <sstream>
#include <filesystem>
#include <mfapi.h>


struct WarmupConfig {
    bool enable_warmup = true;
    int hardware_stabilization_ms = 1500;
    double target_fps_threshold = 0.95;
    int fps_monitoring_frames = 30;
    int stable_frames_required = 10;
    int max_fps_monitoring_ms = 3000;
    int final_sync_delay_ms = 100;
};

struct CompressionConfig {
    bool use_advanced_lossless = true;
    std::string lossless_method = "h264";  // "h264", "h265", "ffv1"
    bool preserve_color_space = true;      // yuv444p 사용
    bool optimize_for_streaming = true;    // faststart 플래그
};


struct Config {
    std::vector<int> camera_indices = {0};
    int frame_width = 1280;
    int frame_height = 720;
    int frame_rate = 30;
    std::string pixel_format = "MJPG";
    int record_duration = 10;

    // path
    std::string ffmpeg = "ffmpeg";
    std::string output = "output.avi";
    
    // 새 필드들 - 동기화 및 개별화
    std::string output_dir = ".";          // CSV 파일 저장 디렉토리

    bool enable_compression = true;        // 압축 활성화
    std::string compression_preset = "medium";  // 압축 속도 (ultrafast, fast, medium)
    bool delete_original = true;          // 원본 파일 삭제 여부
    std::string compressed_suffix = "_compressed";  // 압축 파일 접미사

    // 새로 추가된 timing 설정들
    int consumer_sleep_microseconds = 100; // Consumer loop sleep 시간 (마이크로초)
    int sync_delay_milliseconds = 100;    // 동기화 시작 전 대기 시간 (밀리초)

    WarmupConfig warmup;

    bool validate() const {
        if (camera_indices.empty()) {
            std::cerr << "[Config] 카메라 인덱스가 비어있습니다.\n";
            return false;
        }
        
        if (frame_width <= 0 || frame_height <= 0) {
            std::cerr << "[Config] 프레임 크기가 잘못되었습니다.\n";
            return false;
        }
        
        if (frame_rate <= 0) {
            std::cerr << "[Config] 프레임 레이트가 잘못되었습니다.\n";
            return false;
        }
        
        if (record_duration <= 0) {
            std::cerr << "[Config] 녹화 시간이 잘못되었습니다.\n";
            return false;
        }
        
        if (consumer_sleep_microseconds <= 0) {
            std::cerr << "[Config] Consumer sleep 시간이 잘못되었습니다.\n";
            return false;
        }
        
        if (sync_delay_milliseconds < 0) {
            std::cerr << "[Config] 동기화 delay 시간이 잘못되었습니다.\n";
            return false;
        }

        if (warmup.hardware_stabilization_ms < 0 || warmup.max_fps_monitoring_ms < 0) {
            std::cerr << "[Config] Warmup 설정이 잘못되었습니다.\n";
            return false;
        }
        
        // 출력 디렉토리 생성
        try {
            std::filesystem::create_directories(output_dir);
        } catch (const std::exception& e) {
            std::cerr << "[Config] 출력 디렉토리 생성 실패: " << e.what() << "\n";
            return false;
        }
        
        return true;
    }

    void print() const {
        std::cout << "[Config] 설정 정보:\n";
        std::cout << "  카메라 인덱스: ";
        for (size_t i = 0; i < camera_indices.size(); ++i) {
            std::cout << camera_indices[i];
            if (i < camera_indices.size() - 1) std::cout << ", ";
        }
        std::cout << "\n";
        std::cout << "  해상도: " << frame_width << "x" << frame_height << "\n";
        std::cout << "  프레임 레이트: " << frame_rate << "\n";
        std::cout << "  픽셀 포맷: " << pixel_format << "\n";
        std::cout << "  녹화 시간: " << record_duration << "초\n";
        std::cout << "  출력 파일: " << output << "\n";
        std::cout << "  출력 디렉토리: " << output_dir << "\n";
        std::cout << "  FFmpeg 경로: " << ffmpeg << "\n";
        std::cout << "  압축 활성화: " << (enable_compression ? "활성화" : "비활성화") << "\n";
        std::cout << "  압축 Preset: " << compression_preset << "\n";
        std::cout << "  원본 삭제: " << (delete_original ? "활성화" : "비활성화") << "\n";
        std::cout << "  Consumer Sleep: " << consumer_sleep_microseconds << "μs\n";
        std::cout << "  동기화 Delay: " << sync_delay_milliseconds << "ms\n";
    }
    
    // Helper 메서드들
    std::string generateCameraId(int camera_index) const {
        return "cam_" + std::to_string(camera_index);
    }
    
    std::string generateCsvPath(int camera_index) const {
        return output_dir + "/timestamps_" + generateCameraId(camera_index) + ".csv";
    }
};


// helper
std::vector<int> parseIntList(const std::string& str) {
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

Config parse_args(int argc, char* argv[]) {
    Config config;

    std::unordered_map<std::string, std::function<void(const std::string&)>> handlers = {
        { "--camera_indices", [&](const std::string& v) { config.camera_indices = parseIntList(v); } },
        { "--frame_width",   [&](const std::string& v) { config.frame_width = std::stoi(v); } },
        { "--frame_height",   [&](const std::string& v) { config.frame_height = std::stoi(v); } },
        { "--frame_rate",   [&](const std::string& v) { config.frame_rate = std::stoi(v); } },
        { "--record_duration",   [&](const std::string& v) { config.record_duration = std::stoi(v); } },
        { "--pixel_format",   [&](const std::string& v) { config.pixel_format = v; } },
        { "--output",   [&](const std::string& v) { config.output = v; } },
        { "--output_dir",   [&](const std::string& v) { config.output_dir = v; } },
        { "--ffmpeg",   [&](const std::string& v) { config.ffmpeg = v; } },
        { "--enable_compression", [&](const std::string& v) { config.enable_compression = (v == "true" || v == "1"); } },
        { "--compression_preset", [&](const std::string& v) { config.compression_preset = v; } },
        { "--delete_original", [&](const std::string& v) { config.delete_original = (v == "true" || v == "1"); } },
        { "--consumer_sleep", [&](const std::string& v) { config.consumer_sleep_microseconds = std::stoi(v); } },
        { "--sync_delay", [&](const std::string& v) { config.sync_delay_milliseconds = std::stoi(v); } },
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

std::vector<Config> createMultiCameraConfigs(const Config& baseConfig) {
    std::vector<Config> configs;
    
    for (size_t i = 0; i < baseConfig.camera_indices.size(); ++i) {
        Config cameraConfig = baseConfig;
        
        // 단일 카메라 인덱스 설정 (SingleManager에서 사용하기 위해)
        cameraConfig.camera_indices = {baseConfig.camera_indices[i]};
        
        // 출력 파일명 개별화
        if (baseConfig.camera_indices.size() > 1) {
            std::string extension = ".avi";
            std::string baseName = baseConfig.output;
            
            // 확장자 제거
            size_t dotPos = baseName.find_last_of('.');
            if (dotPos != std::string::npos) {
                extension = baseName.substr(dotPos);
                baseName = baseName.substr(0, dotPos);
            }
            
            cameraConfig.output = baseName + "_camera_" + std::to_string(baseConfig.camera_indices[i]) + extension;
        }
        
        configs.push_back(cameraConfig);
    }
    
    return configs;
}