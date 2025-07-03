#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <sstream>
#include <filesystem>
#include <iomanip>
#include <algorithm>
#include <mfapi.h>

#include "error/config_error.hpp"

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
    int warmup_duration = 3;
    
    // Output settings
    enum class OutputMode { VIDEO, IMAGE };
    OutputMode output_mode = OutputMode::IMAGE;
    std::string output_filename = "output";
    std::string output_directory = ".";
    std::string video_format = "avi";
    std::string image_format = "jpg";
    bool save_timestamp_csv = true;

    // ==========================================
    // NEW: Tobii settings
    // ==========================================
    bool enable_tobii = true;
    std::string tobii_output_directory = "tobii_output";
    std::string display_area_config_path = "";
    bool save_gaze_csv = true;
    bool save_eye_images = true;
    int tobii_sample_rate = 120;

    // ==========================================
    // NEW: Tobii helper methods
    // ==========================================
    std::string getTobiiGazeCSVPath() const {
        return tobii_output_directory + "/gaze_data.csv";
    }
    
    std::string getTobiiEyeImageDir() const {
        return tobii_output_directory + "/eye_images";
    }

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

        // NEW: Tobii validation
        if (enable_tobii) {
            if (tobii_output_directory.empty()) {
                throw ConfigOutputError("Tobii 출력 디렉토리가 비어있습니다.");
            }

            if (tobii_sample_rate <= 0) {
                throw ConfigOutputError("Tobii 샘플 레이트가 잘못되었습니다.");
            }
        }

        try {
            if (!std::filesystem::exists(output_directory)) {
                std::filesystem::create_directories(output_directory);
                std::cout << "[Config] 출력 디렉토리 생성: " << output_directory << "\n";
            }

            // NEW: Tobii directory creation
            if (enable_tobii && !std::filesystem::exists(tobii_output_directory)) {
                std::filesystem::create_directories(tobii_output_directory);
                std::cout << "[Config] Tobii 출력 디렉토리 생성: " << tobii_output_directory << "\n";
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
        
        // Warmup settings
        std::cout << "  Warmup 활성화: " << (enable_warmup ? "ON" : "OFF") << "\n";
        std::cout << "  Warmup 시간: " << warmup_duration << "초\n";
        
        // Output settings
        std::cout << "  출력 모드: " << (output_mode == OutputMode::VIDEO ? "VIDEO" : "IMAGE") << "\n";
        std::cout << "  출력 디렉토리: " << output_directory << "\n";
        
        if (output_mode == OutputMode::VIDEO) {
            std::cout << "  비디오 파일: " << output_filename << "\n";
            std::cout << "  비디오 포맷: " << video_format << "\n";
        } else {
            std::cout << "  이미지 포맷: " << image_format << "\n";
        }
        
        std::cout << "  CSV 저장: " << (save_timestamp_csv ? "ON" : "OFF") << "\n";

        // NEW: Tobii settings
        std::cout << "\n  === Tobii 설정 ===\n";
        std::cout << "  Tobii 활성화: " << (enable_tobii ? "ON" : "OFF") << "\n";
        if (enable_tobii) {
            std::cout << "  Tobii 출력 디렉토리: " << tobii_output_directory << "\n";
            std::cout << "  Gaze CSV 저장: " << (save_gaze_csv ? "ON" : "OFF") << "\n";
            std::cout << "  Eye Image 저장: " << (save_eye_images ? "ON" : "OFF") << "\n";
            std::cout << "  샘플 레이트: " << tobii_sample_rate << "Hz\n";
            if (!display_area_config_path.empty()) {
                std::cout << "  Display Area Config: " << display_area_config_path << "\n";
            }
        }
    }

    // From original config file
    static Config parseArgs(int argc, char* argv[]) {
        Config conf;
        
        std::unordered_map<std::string, std::function<void(const std::string&)>> parsers = {
            // Camera settings
            {"--camera_indices", [&](const std::string& v) { 
                conf.camera_indices.clear();
                std::istringstream iss(v);
                std::string token;
                while (iss >> token) {
                    conf.camera_indices.push_back(std::stoi(token));
                }
            }},
            {"--frame_width", [&](const std::string& v) { conf.frame_width = std::stoi(v); }},
            {"--frame_height", [&](const std::string& v) { conf.frame_height = std::stoi(v); }},
            {"--frame_rate", [&](const std::string& v) { conf.frame_rate = std::stoi(v); }},
            {"--pixel_format", [&](const std::string& v) { conf.pixel_format = v; }},
            
            // Recording settings
            {"--record_duration", [&](const std::string& v) { conf.record_duration = std::stoi(v); }},
            {"--duration", [&](const std::string& v) { conf.record_duration = std::stoi(v); }},
            {"--consumer_sleep_microseconds", [&](const std::string& v) { conf.consumer_sleep_microseconds = std::stoi(v); }},
            {"--sync_delay_milliseconds", [&](const std::string& v) { conf.sync_delay_milliseconds = std::stoi(v); }},
            
            // Warmup settings
            {"--enable_warmup", [&](const std::string& v) { conf.enable_warmup = _parseBool(v); }},
            {"--warmup_duration", [&](const std::string& v) { conf.warmup_duration = std::stoi(v); }},
            
            // Output settings
            {"--output_mode", [&](const std::string& v) { 
                if (v == "video") {
                    conf.output_mode = Config::OutputMode::VIDEO;
                } else if (v == "image") {
                    conf.output_mode = Config::OutputMode::IMAGE;
                } else {
                    throw std::invalid_argument("Invalid output_mode: " + v + " (use 'video' or 'image')");
                }
            }},
            {"--output_filename", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("output_filename cannot be empty");
                conf.output_filename = v; 
            }},
            {"--output", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("output cannot be empty");
                conf.output_filename = v;
            }},
            {"--output_directory", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("output_directory cannot be empty");
                conf.output_directory = v; 
            }},
            {"--output_dir", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("output_dir cannot be empty");
                conf.output_directory = v; 
            }},
            {"--video_format", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("video_format cannot be empty");
                conf.video_format = v; 
            }},
            {"--image_format", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("image_format cannot be empty");
                conf.image_format = v; 
            }},
            {"--save_timestamp_csv", [&](const std::string& v) { 
                conf.save_timestamp_csv = _parseBool(v); 
            }},
            
            // External tools
            {"--ffmpeg_path", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("ffmpeg_path cannot be empty");
                conf.ffmpeg_path = v; 
            }},
            {"--ffmpeg", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("ffmpeg cannot be empty");
                conf.ffmpeg_path = v; 
            }},

            // ==========================================
            // NEW: Tobii settings
            // ==========================================
            {"--enable_tobii", [&](const std::string& v) { 
                conf.enable_tobii = _parseBool(v); 
            }},
            {"--tobii", [&](const std::string& v) { 
                conf.enable_tobii = _parseBool(v); 
            }},
            {"--tobii_output_directory", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("tobii_output_directory cannot be empty");
                conf.tobii_output_directory = v; 
            }},
            {"--tobii_output_dir", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("tobii_output_dir cannot be empty");
                conf.tobii_output_directory = v; 
            }},
            {"--tobii_output", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("tobii_output cannot be empty");
                conf.tobii_output_directory = v; 
            }},
            {"--display_area_config", [&](const std::string& v) { 
                conf.display_area_config_path = v; 
            }},
            {"--save_gaze_csv", [&](const std::string& v) { 
                conf.save_gaze_csv = _parseBool(v); 
            }},
            {"--save_eye_images", [&](const std::string& v) { 
                conf.save_eye_images = _parseBool(v); 
            }},
            {"--tobii_sample_rate", [&](const std::string& v) { 
                conf.tobii_sample_rate = std::stoi(v); 
            }},
        };

        _parseArguments(argc, argv, parsers);
        
        return conf;
    }

private:
    static void _parseArguments(int argc, char* argv[], 
                        std::unordered_map<std::string, std::function<void(const std::string&)>>& parsers) {
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            
            if (parsers.find(arg) != parsers.end()) {
                if (i + 1 < argc) {
                    std::string value = argv[i + 1];
                    try {
                        parsers[arg](value);
                        i++; // Skip next argument as it's the value
                    } catch (const std::exception& e) {
                        throw std::runtime_error("Failed to parse " + arg + " with value '" + value + "': " + e.what());
                    }
                } else {
                    throw std::runtime_error("Missing value for argument: " + arg);
                }
            } else {
                std::cout << "[Config Warning] Unknown argument: " << arg << "\n";
            }
        }
    }

    static bool _parseBool(const std::string& value) {
        std::string lower_value = value;
        std::transform(lower_value.begin(), lower_value.end(), lower_value.begin(), ::tolower);
        
        if (lower_value == "true" || lower_value == "1" || lower_value == "on" || lower_value == "yes") {
            return true;
        } else if (lower_value == "false" || lower_value == "0" || lower_value == "off" || lower_value == "no") {
            return false;
        } else {
            throw std::invalid_argument("Invalid boolean value: " + value);
        }
    }
};

/**
 * class: ConfigManager
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
        
        try {
            config = Config::parseArgs(argc, argv);
            config.validate();    
            config.print();
        } catch (const ConfigInputError& e) {
            std::cerr << "[Config Error] 입력 오류: " << e.what() << "\n";
            throw;
        } catch (const ConfigCameraError& e) {
            std::cerr << "[Config Error] 카메라 설정 오류: " << e.what() << "\n";
            throw;
        } catch (const ConfigRecordingError& e) {
            std::cerr << "[Config Error] 녹화 설정 오류: " << e.what() << "\n";
            throw;
        } catch (const ConfigWarmupError& e) {
            std::cerr << "[Config Error] 웜업 설정 오류: " << e.what() << "\n";
            throw;
        } catch (const ConfigOutputError& e) {
            std::cerr << "[Config Error] 출력 설정 오류: " << e.what() << "\n";
            throw;
        } catch (const ConfigExternalError& e) {
            std::cerr << "[Config Error] 외부 도구 오류: " << e.what() << "\n";
            throw;
        } catch (const std::exception& e) {
            std::cerr << "[Config Error] 예상치 못한 오류: " << e.what() << "\n";
            throw;
        }
            
        return config;
    }
};

/**
 * Global: gonfig
 */

extern Config gonfig;