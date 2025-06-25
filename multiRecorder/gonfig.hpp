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
        
        // 핸들러 맵 정의 - 모든 설정 파라미터 포함
        std::unordered_map<std::string, std::function<void(const std::string&)>> handlers = {
            // ===================
            // Camera settings
            // ===================
            {"--camera_indices", [&](const std::string& v) { 
                config.camera_indices = _parseIntList(v); 
            }},
            {"--frame_width", [&](const std::string& v) { 
                int value = std::stoi(v);
                if (value <= 0) throw std::invalid_argument("frame_width must be positive");
                config.frame_width = value; 
            }},
            {"--frame_height", [&](const std::string& v) { 
                int value = std::stoi(v);
                if (value <= 0) throw std::invalid_argument("frame_height must be positive");
                config.frame_height = value; 
            }},
            {"--frame_rate", [&](const std::string& v) { 
                int value = std::stoi(v);
                if (value <= 0) throw std::invalid_argument("frame_rate must be positive");
                config.frame_rate = value; 
            }},
            {"--pixel_format", [&](const std::string& v) { 
                // 지원되는 포맷 검증
                if (v != "MJPG" && v != "NV12" && v != "YUY2" && 
                    v != "RGB32" && v != "I420" && v != "UYVY") {
                    throw std::invalid_argument("Unsupported pixel format: " + v);
                }
                config.pixel_format = v; 
            }},
            
            // ===================
            // Recording settings
            // ===================
            {"--record_duration", [&](const std::string& v) { 
                int value = std::stoi(v);
                if (value <= 0) throw std::invalid_argument("record_duration must be positive");
                config.record_duration = value; 
            }},
            {"--consumer_sleep", [&](const std::string& v) { 
                int value = std::stoi(v);
                if (value <= 0) throw std::invalid_argument("consumer_sleep must be positive");
                config.consumer_sleep_microseconds = value; 
            }},
            {"--sync_delay", [&](const std::string& v) { 
                int value = std::stoi(v);
                if (value < 0) throw std::invalid_argument("sync_delay cannot be negative");
                config.sync_delay_milliseconds = value; 
            }},
            
            // ===================
            // Warmup settings
            // ===================
            {"--enable_warmup", [&](const std::string& v) { 
                config.enable_warmup = _parseBool(v); 
            }},
            {"--warmup_duration", [&](const std::string& v) { 
                int value = std::stoi(v);
                if (value <= 0) throw std::invalid_argument("warmup_duration must be positive");
                config.warmup_duration = value; 
            }},
            
            // ===================
            // Output settings
            // ===================
            {"--output_mode", [&](const std::string& v) { 
                std::string mode = _toLowercase(v);
                if (mode == "video" || mode == "vid" || mode == "v") {
                    config.output_mode = Config::OutputMode::VIDEO;
                } else if (mode == "image" || mode == "images" || mode == "img" || mode == "i") {
                    config.output_mode = Config::OutputMode::IMAGE;
                } else {
                    throw std::invalid_argument("Invalid output mode: " + v + " (use 'video' or 'image')");
                }
            }},
            {"--output_filename", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("output_filename cannot be empty");
                config.output_filename = v; 
            }},
            {"--output", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("output cannot be empty");
                config.output_filename = v; // 호환성
            }},
            {"--output_directory", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("output_directory cannot be empty");
                config.output_directory = v; 
            }},
            {"--output_dir", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("output_dir cannot be empty");
                config.output_directory = v; 
            }},
            {"--video_format", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("video_format cannot be empty");
                config.video_format = v; 
            }},
            {"--image_format", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("image_format cannot be empty");
                config.image_format = v; 
            }},
            {"--save_timestamp_csv", [&](const std::string& v) { 
                config.save_timestamp_csv = _parseBool(v); 
            }},
            
            // ===================
            // External tools
            // ===================
            {"--ffmpeg_path", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("ffmpeg_path cannot be empty");
                config.ffmpeg_path = v; 
            }},
            {"--ffmpeg", [&](const std::string& v) { 
                if (v.empty()) throw std::invalid_argument("ffmpeg cannot be empty");
                config.ffmpeg_path = v; 
            }},
        };

        // 인자 파싱
        for (int i = 1; i < argc; i += 2) {
            if (i + 1 >= argc) {
                throw ConfigInputError("값이 누락된 인자: " + std::string(argv[i]));
            }

            std::string key = argv[i];
            std::string value = argv[i + 1];

            // 핸들러 찾기
            auto handler_it = handlers.find(key);
            if (handler_it == handlers.end()) {
                throw ConfigInputError("알 수 없는 인자: " + key + 
                    "\n사용 가능한 인자들:\n" + _getUsageString());
            }
            
            // 값 파싱 및 설정
            try {
                handler_it->second(value);
            } catch (const std::invalid_argument& e) {
                throw ConfigInputError("인자 파싱 실패 (" + key + "): " + e.what());
            } catch (const std::out_of_range& e) {
                throw ConfigInputError("인자 범위 오류 (" + key + "): " + e.what());
            } catch (const std::exception& e) {
                throw ConfigInputError("인자 처리 실패 (" + key + "): " + e.what());
            }
        }

        return config;
    }

private:
    static std::vector<int> _parseIntList(const std::string& str) {
        if (str.empty()) {
            throw std::invalid_argument("정수 리스트가 비어있습니다");
        }
        
        std::vector<int> result;
        std::stringstream ss(str);
        std::string item;
        
        while (std::getline(ss, item, ',')) {
            // 공백 제거
            item.erase(0, item.find_first_not_of(" \t"));
            item.erase(item.find_last_not_of(" \t") + 1);
            
            if (item.empty()) continue;
            
            try {
                int value = std::stoi(item);
                if (value < 0) {
                    throw std::invalid_argument("카메라 인덱스는 음수일 수 없습니다: " + item);
                }
                result.push_back(value);
            } catch (const std::invalid_argument&) {
                throw std::invalid_argument("잘못된 숫자 형식: " + item);
            } catch (const std::out_of_range&) {
                throw std::invalid_argument("숫자 범위 초과: " + item);
            }
        }
        
        if (result.empty()) {
            throw std::invalid_argument("유효한 카메라 인덱스가 없습니다");
        }
        
        return result;
    }
    
    static bool _parseBool(const std::string& str) {
        std::string lower = _toLowercase(str);
        
        if (lower == "true" || lower == "1" || lower == "on" || 
            lower == "yes" || lower == "y" || lower == "enable") {
            return true;
        } else if (lower == "false" || lower == "0" || lower == "off" || 
                   lower == "no" || lower == "n" || lower == "disable") {
            return false;
        } else {
            throw std::invalid_argument("잘못된 불린 값: " + str + 
                " (true/false, 1/0, on/off, yes/no, enable/disable 사용)");
        }
    }
    
    static std::string _toLowercase(const std::string& str) {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(), ::tolower);
        return result;
    }
    
    static std::string _getUsageString() {
        return R"(카메라 설정:
  --camera_indices               카메라 인덱스 (예: 0,1,2)
  --frame_width                  프레임 너비 (기본값: 1280)
  --frame_height                 프레임 높이 (기본값: 720)
  --frame_rate                   프레임 레이트 (기본값: 30)
  --pixel_format                 픽셀 포맷 (MJPG, NV12, YUY2, RGB32, I420, UYVY)

녹화 설정:
  --record_duration              녹화 시간(초) (기본값: 5)
  --consumer_sleep               컨슈머 슬립(μs) (기본값: 100)
  --sync_delay                   동기화 딜레이(ms) (기본값: 100)

웜업 설정:
  --enable_warmup                웜업 활성화 (true/false, 기본값: true)
  --warmup_duration              웜업 시간(초) (기본값: 3)

출력 설정:
  --output_mode                  출력 모드 (video/image, 기본값: image)
  --output_filename, --output    출력 파일명 (기본값: output)
  --output_directory, --output_dir 출력 디렉토리 (기본값: .)
  --video_format                 비디오 포맷 (기본값: avi)
  --image_format                 이미지 포맷 (기본값: jpg)
  --save_timestamp_csv           CSV 저장 (true/false, 기본값: true)

외부 도구:
  --ffmpeg_path, --ffmpeg        FFmpeg 경로 (기본값: ffmpeg))";
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
        
        try {
            config = ConfigParser::parseArgs(argc, argv);
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