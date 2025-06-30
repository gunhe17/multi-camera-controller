#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <iomanip>
#include <filesystem>
#include <sstream>
#include <memory>
#include <string>
#include <exception>
#include <cstring>

// Tobii Research API headers
#include "tobii_research.h"
#include "tobii_research_eyetracker.h"
#include "tobii_research_streams.h"

// Windows specific
#if _WIN32 || _WIN64
#include <windows.h>
static void sleep_ms(int time) {
    Sleep(time);
}
#else
#include <unistd.h>
static void sleep_ms(int time) {
    usleep(time * 1000);
}
#endif

// ==========================================
// Forward Declarations
// ==========================================
class TobiiMaker;
class TobiiStreamer;

// ==========================================
// Error Classes
// ==========================================

class Error : public std::runtime_error {
public:
    Error(const std::string& type, const std::string& msg, int code): 
        std::runtime_error("[Error] " + type + ": " + msg + " (code: " + std::to_string(code) + ")"),
        type_(type), message_(msg), code_(code) {}

    const std::string& getType() const noexcept { return type_; }
    const std::string& getMessage() const noexcept { return message_; }
    int getCode() const noexcept { return code_; }

private:
    std::string type_;
    std::string message_;
    int code_;
};

class TobiiManagerError : public Error {
public:
    TobiiManagerError(const std::string& msg, int code = 2000)
        : Error("TobiiManagerError", msg, code) {}
};

class TobiiRecorderError : public Error {
public:
    TobiiRecorderError(const std::string& msg, int code = 2100)
        : Error("TobiiRecorderError", msg, code) {}
};

class TobiiMakerError : public Error {
public:
    TobiiMakerError(const std::string& msg, int code = 2200)
        : Error("TobiiMakerError", msg, code) {}
};

// ==========================================
// Data Structures
// ==========================================

struct TobiiGazeData {
    std::chrono::system_clock::time_point timestamp_;
    int64_t device_timestamp_;
    
    struct EyeGazePoint {
        double x, y;
        bool validity;
        double pupil_diameter;
    } left_eye_, right_eye_;
    
    struct EyePosition3D {
        double x, y, z;
        bool validity;
    } left_eye_3d_, right_eye_3d_;
};

struct TobiiEyeImageData {
    std::chrono::system_clock::time_point timestamp_;
    int64_t device_timestamp_;
    int camera_id_;
    int width_, height_;
    std::vector<uint8_t> raw_data_;
};

// ==========================================
// Config
// ==========================================

struct TobiiConfig {
    bool enable_tobii = true;
    std::string output_directory = "tobii_output";
    bool save_gaze_csv = true;
    bool save_eye_images = true;
    // int record_duration = 10;
    int warmup_duration = 2;
    
    // streaming settings
    bool enable_streaming = true;
    int stream_fps = 30;  // 출력 FPS
    bool stream_gaze = true;
    bool stream_status = true;
};

TobiiConfig tobii_gonfig;

// ==========================================
// Exception Debug Helper
// ==========================================

class ExceptionHandler {
public:
    static void setupTerminateHandler() {
        std::set_terminate([]() {
            std::cout << "[FATAL] Unhandled exception occurred! Program terminating.\n";
            try {
                auto eptr = std::current_exception();
                if (eptr) {
                    std::rethrow_exception(eptr);
                }
            } catch (const std::exception& e) {
                std::cout << "[FATAL] Exception: " << e.what() << "\n";
            } catch (...) {
                std::cout << "[FATAL] Unknown exception type\n";
            }
            std::abort();
        });
    }
};

// ==========================================
// TobiiStreamer Class
// ==========================================

class TobiiStreamer {
private:
    // identifier
    int streamer_id_;
    
    // state
    std::atomic<bool> is_streaming_;
    
    // timing control
    std::chrono::steady_clock::time_point last_gaze_output_;
    std::chrono::steady_clock::time_point last_status_output_;
    int gaze_interval_ms_;
    int status_interval_ms_;
    
    // metrics
    std::atomic<int> total_gaze_count_;

public:
    TobiiStreamer(int streamer_id) {
        streamer_id_ = streamer_id;
        is_streaming_ = false;
        gaze_interval_ms_ = 1000 / tobii_gonfig.stream_fps;  // FPS 기반 간격
        status_interval_ms_ = 1000;  // 1초마다 상태 출력
        total_gaze_count_ = 0;
    }

    ~TobiiStreamer() {
        stop();
    }

    void init() {
        // empty
    }

    void setup() {
        if (!tobii_gonfig.enable_streaming) return;
        
        std::cout << "[TobiiStreamer " << streamer_id_ << "] Setup completed\n";
    }

    void warmup() {
        if (!tobii_gonfig.enable_streaming) return;
        
        std::cout << "[TobiiStreamer " << streamer_id_ << "] warmup done\n";
    }

    void run() {
        if (!tobii_gonfig.enable_streaming) return;
        
        is_streaming_ = true;
        last_gaze_output_ = std::chrono::steady_clock::now();
        last_status_output_ = std::chrono::steady_clock::now();
        
        // 시작 신호 출력
        _outputEvent("RECORDING_STARTED", "{}");
        
        std::cout << "[TobiiStreamer " << streamer_id_ << "] streaming started\n";
    }

    void stop() {
        if (!tobii_gonfig.enable_streaming) return;
        
        is_streaming_ = false;
        
        // 종료 신호 출력
        _outputEvent("RECORDING_STOPPED", "{}");
        
        std::cout << "[TobiiStreamer " << streamer_id_ << "] stopped\n";
    }

    void updateGaze(const TobiiGazeData& data) {
        if (!is_streaming_.load() || !tobii_gonfig.stream_gaze) return;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_gaze_output_).count();
            
        if (elapsed >= gaze_interval_ms_) {
            _outputGazeData(data);
            last_gaze_output_ = now;
            total_gaze_count_.fetch_add(1);
        }
        
        // 상태 정보도 주기적으로 출력
        _checkStatusOutput(now);
    }

    void updateStatus(int gaze_count, int left_eye_count, int right_eye_count, double elapsed_seconds) {
        if (!is_streaming_.load() || !tobii_gonfig.stream_status) return;
        
        std::ostringstream status_json;
        status_json << std::fixed << std::setprecision(1);
        status_json << "{"
                   << "\"recording\":true,"
                   << "\"gaze_count\":" << gaze_count << ","
                   << "\"left_eye_count\":" << left_eye_count << ","
                   << "\"right_eye_count\":" << right_eye_count << ","
                   << "\"elapsed_seconds\":" << elapsed_seconds << ","
                   << "\"stream_count\":" << total_gaze_count_.load()
                   << "}";
        
        _outputEvent("STATUS", status_json.str());
    }

private:
    void _outputGazeData(const TobiiGazeData& data) {
        std::ostringstream gaze_json;
        gaze_json << std::fixed << std::setprecision(6);
        
        auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            data.timestamp_.time_since_epoch()).count();
            
        gaze_json << "{"
                 << "\"timestamp\":" << timestamp_ms << ","
                 << "\"device_timestamp\":" << data.device_timestamp_ << ","
                 << "\"left_eye\":{"
                 << "\"x\":" << data.left_eye_.x << ","
                 << "\"y\":" << data.left_eye_.y << ","
                 << "\"valid\":" << (data.left_eye_.validity ? "true" : "false") << ","
                 << "\"pupil\":" << data.left_eye_.pupil_diameter
                 << "},"
                 << "\"right_eye\":{"
                 << "\"x\":" << data.right_eye_.x << ","
                 << "\"y\":" << data.right_eye_.y << ","
                 << "\"valid\":" << (data.right_eye_.validity ? "true" : "false") << ","
                 << "\"pupil\":" << data.right_eye_.pupil_diameter
                 << "},"
                 << "\"average\":{"
                 << "\"x\":" << _getAverageX(data) << ","
                 << "\"y\":" << _getAverageY(data) << ","
                 << "\"valid\":" << (data.left_eye_.validity && data.right_eye_.validity ? "true" : "false")
                 << "},"
                 << "\"3d_left\":{"
                 << "\"x\":" << data.left_eye_3d_.x << ","
                 << "\"y\":" << data.left_eye_3d_.y << ","
                 << "\"z\":" << data.left_eye_3d_.z << ","
                 << "\"valid\":" << (data.left_eye_3d_.validity ? "true" : "false")
                 << "},"
                 << "\"3d_right\":{"
                 << "\"x\":" << data.right_eye_3d_.x << ","
                 << "\"y\":" << data.right_eye_3d_.y << ","
                 << "\"z\":" << data.right_eye_3d_.z << ","
                 << "\"valid\":" << (data.right_eye_3d_.validity ? "true" : "false")
                 << "}"
                 << "}";
        
        _outputEvent("GAZE_DATA", gaze_json.str());
    }
    
    void _checkStatusOutput(std::chrono::steady_clock::time_point now) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_status_output_).count();
            
        if (elapsed >= status_interval_ms_) {
            // 상태 업데이트는 TobiiMultiManager에서 호출
            last_status_output_ = now;
        }
    }
    
    void _outputEvent(const std::string& event_type, const std::string& data) {
        // STDOUT으로 구조화된 이벤트 출력
        std::cout << event_type << ":" << data << std::endl;
        std::cout.flush();  // 즉시 출력 보장
    }
    
    double _getAverageX(const TobiiGazeData& data) {
        if (data.left_eye_.validity && data.right_eye_.validity) {
            return (data.left_eye_.x + data.right_eye_.x) / 2.0;
        } else if (data.left_eye_.validity) {
            return data.left_eye_.x;
        } else if (data.right_eye_.validity) {
            return data.right_eye_.x;
        }
        return 0.0;
    }
    
    double _getAverageY(const TobiiGazeData& data) {
        if (data.left_eye_.validity && data.right_eye_.validity) {
            return (data.left_eye_.y + data.right_eye_.y) / 2.0;
        } else if (data.left_eye_.validity) {
            return data.left_eye_.y;
        } else if (data.right_eye_.validity) {
            return data.right_eye_.y;
        }
        return 0.0;
    }
};

// ==========================================
// TobiiManager Class
// ==========================================

class TobiiManager {
private:
    // state
    std::atomic<bool> enabled_;
    TobiiResearchEyeTracker* eye_tracker_;
    
    // device info
    std::string address_;
    std::string serial_number_;
    std::string device_name_;
    std::string model_;

public:
    TobiiManager() {
        enabled_ = false;
        eye_tracker_ = nullptr;
    }

    ~TobiiManager() {
        // cleanup handled by stop()
    }

    void init() {
        // empty - following your pattern
    }

    void setup() {
        std::cout << "[TobiiManager] Starting setup...\n";
        try {
            _findEyeTrackers();
            _connectEyeTracker();
            _setupDeviceInfo();
            enabled_ = true;
            std::cout << "[TobiiManager] Setup completed successfully\n";
        } catch (const std::exception& e) {
            std::cout << "[TobiiManager] Setup failed: " << e.what() << "\n";
            enabled_ = false;
            throw;
        }
    }

    void warmup() {
        if (!enabled_) {
            throw TobiiManagerError("TobiiManager not enabled", 2001);
        }
        
        std::cout << "[TobiiManager] warmup for " << tobii_gonfig.warmup_duration << " seconds\n";
        sleep_ms(tobii_gonfig.warmup_duration * 1000);
        std::cout << "[TobiiManager] warmup done\n";
    }

    void run() {
        if (!enabled_) {
            throw TobiiManagerError("TobiiManager not enabled for run", 2002);
        }
        std::cout << "[TobiiManager] run started\n";
    }

    void stop() {
        enabled_ = false;
        std::cout << "[TobiiManager] stopped\n";
    }

    // getter
    bool isEnabled() const { return enabled_.load(); }
    TobiiResearchEyeTracker* getEyeTracker() const { return eye_tracker_; }

private:
    void _findEyeTrackers() {
        TobiiResearchEyeTrackers* eyetrackers = nullptr;
        TobiiResearchStatus status = tobii_research_find_all_eyetrackers(&eyetrackers);
        
        if (status != TOBII_RESEARCH_STATUS_OK) {
            throw TobiiManagerError("Failed to find eye trackers. Status: " + std::to_string(status), 2003);
        }
        
        if (eyetrackers->count == 0) {
            tobii_research_free_eyetrackers(eyetrackers);
            throw TobiiManagerError("No eye trackers found", 2004);
        }
        
        std::cout << "[TobiiManager] Found " << eyetrackers->count << " eye tracker(s)\n";
        
        // Use first available
        eye_tracker_ = eyetrackers->eyetrackers[0];
        tobii_research_free_eyetrackers(eyetrackers);
    }

    void _connectEyeTracker() {
        if (!eye_tracker_) {
            throw TobiiManagerError("Eye tracker not found for connection", 2005);
        }
        std::cout << "[TobiiManager] Eye tracker connected\n";
    }

    void _setupDeviceInfo() {
        char* address = nullptr;
        char* serial_number = nullptr;
        char* device_name = nullptr;
        char* model = nullptr;
        
        tobii_research_get_address(eye_tracker_, &address);
        tobii_research_get_serial_number(eye_tracker_, &serial_number);
        tobii_research_get_device_name(eye_tracker_, &device_name);
        tobii_research_get_model(eye_tracker_, &model);
        
        std::cout << "[TobiiManager] Device Info:\n";
        std::cout << "  Address: " << (address ? address : "N/A") << "\n";
        std::cout << "  Model: " << (model ? model : "N/A") << "\n";
        std::cout << "  Serial: " << (serial_number ? serial_number : "N/A") << "\n";
        std::cout << "  Name: " << (device_name ? device_name : "N/A") << "\n";
        
        // Store for later use
        address_ = address ? address : "";
        serial_number_ = serial_number ? serial_number : "";
        device_name_ = device_name ? device_name : "";
        model_ = model ? model : "";
        
        // Free strings
        tobii_research_free_string(address);
        tobii_research_free_string(serial_number);
        tobii_research_free_string(device_name);
        tobii_research_free_string(model);
    }
};

// ==========================================
// TobiiRecorder Class
// ==========================================

class TobiiRecorder {
private:
    // identifier
    int recorder_id_;
    
    // state
    std::atomic<bool> is_recording_;
    
    // device
    TobiiResearchEyeTracker* eye_tracker_;

    // connections
    TobiiMaker* tobii_maker_;
    TobiiStreamer* tobii_streamer_;
    
    // counters
    std::atomic<int> gaze_count_;
    std::atomic<int> left_eye_image_count_;
    std::atomic<int> right_eye_image_count_;

public:
    TobiiRecorder(int recorder_id) {
        recorder_id_ = recorder_id;
        is_recording_ = false;
        eye_tracker_ = nullptr;
        tobii_maker_ = nullptr;
        tobii_streamer_ = nullptr;
        gaze_count_ = 0;
        left_eye_image_count_ = 0;
        right_eye_image_count_ = 0;
    }

    ~TobiiRecorder() {
        stop();
    }

    void init() {
        // empty
    }

    void setup(TobiiResearchEyeTracker* eye_tracker) {
        std::cout << "[TobiiRecorder " << recorder_id_ << "] Starting setup...\n";
        if (!eye_tracker) {
            throw TobiiRecorderError("Eye tracker is null", 2104);
        }
        
        try {
            eye_tracker_ = eye_tracker;
            _subscribeToStreams();
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Setup completed\n";
        } catch (const std::exception& e) {
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Setup failed: " << e.what() << "\n";
            throw;
        }
    }

    void warmup() {
        std::cout << "[TobiiRecorder " << recorder_id_ << "] warmup done\n";
    }

    void run() {
        is_recording_ = true;
        std::cout << "[TobiiRecorder " << recorder_id_ << "] recording started\n";
    }

    void stop() {
        is_recording_ = false;
        if (eye_tracker_) {
            _unsubscribeFromStreams();
        }
        std::cout << "[TobiiRecorder " << recorder_id_ << "] stopped\n";
    }

    // setters
    void setTobiiMaker(TobiiMaker* maker) {
        tobii_maker_ = maker;
    }
    
    void setTobiiStreamer(TobiiStreamer* streamer) {
        tobii_streamer_ = streamer;
    }

    // getters
    int getGazeCount() const { return gaze_count_.load(); }
    int getLeftEyeImageCount() const { return left_eye_image_count_.load(); }
    int getRightEyeImageCount() const { return right_eye_image_count_.load(); }

private:
    bool _subscribeToStreams() {
        std::cout << "[TobiiRecorder " << recorder_id_ << "] Subscribing to streams...\n";
        TobiiResearchStatus status;
        
        try {
            // Subscribe to notifications
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Subscribing to notifications...\n";
            status = tobii_research_subscribe_to_notifications(eye_tracker_, 
                [](TobiiResearchNotification* notification, void* user_data) {
                    auto* recorder = static_cast<TobiiRecorder*>(user_data);
                    recorder->_bufferOverflowCallback(notification);
                }, this);
            
            if (status != TOBII_RESEARCH_STATUS_OK) {
                throw TobiiRecorderError("Failed to subscribe to notifications. Status: " + std::to_string(status), 2101);
            }
            
            // Subscribe to gaze data
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Subscribing to gaze data...\n";
            status = tobii_research_subscribe_to_gaze_data(eye_tracker_, 
                [](TobiiResearchGazeData* gaze_data, void* user_data) {
                    auto* recorder = static_cast<TobiiRecorder*>(user_data);
                    recorder->_gazeDataCallback(gaze_data);
                }, this);
            
            if (status != TOBII_RESEARCH_STATUS_OK) {
                throw TobiiRecorderError("Failed to subscribe to gaze data. Status: " + std::to_string(status), 2102);
            }
            
            // Subscribe to eye images
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Subscribing to eye images...\n";
            status = tobii_research_subscribe_to_eye_image(eye_tracker_, 
                [](TobiiResearchEyeImage* eye_image, void* user_data) {
                    auto* recorder = static_cast<TobiiRecorder*>(user_data);
                    recorder->_eyeImageCallback(eye_image);
                }, this);
            
            if (status != TOBII_RESEARCH_STATUS_OK) {
                throw TobiiRecorderError("Failed to subscribe to eye images. Status: " + std::to_string(status), 2103);
            }
            
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Successfully subscribed to all streams\n";
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Subscription failed: " << e.what() << "\n";
            throw;
        }
    }

    void _unsubscribeFromStreams() {
        tobii_research_unsubscribe_from_gaze_data(eye_tracker_, 
            [](TobiiResearchGazeData* gaze_data, void* user_data) {
                auto* recorder = static_cast<TobiiRecorder*>(user_data);
                recorder->_gazeDataCallback(gaze_data);
            });
        
        tobii_research_unsubscribe_from_eye_image(eye_tracker_, 
            [](TobiiResearchEyeImage* eye_image, void* user_data) {
                auto* recorder = static_cast<TobiiRecorder*>(user_data);
                recorder->_eyeImageCallback(eye_image);
            });
        
        tobii_research_unsubscribe_from_notifications(eye_tracker_, 
            [](TobiiResearchNotification* notification, void* user_data) {
                auto* recorder = static_cast<TobiiRecorder*>(user_data);
                recorder->_bufferOverflowCallback(notification);
            });
        
        std::cout << "[TobiiRecorder " << recorder_id_ << "] Unsubscribed from all streams\n";
    }

    void _gazeDataCallback(TobiiResearchGazeData* gaze_data);
    void _eyeImageCallback(TobiiResearchEyeImage* eye_image);
    void _bufferOverflowCallback(TobiiResearchNotification* notification);
};

// ==========================================
// TobiiMaker Class
// ==========================================

class TobiiMaker {
private:
    // identifier
    int recorder_id_;
    
    // state
    std::atomic<bool> is_gate_open_;
    
    // output
    std::string output_directory_;
    std::ofstream gaze_csv_file_;
    
    // counters
    int left_eye_image_index_;
    int right_eye_image_index_;

public:
    TobiiMaker(int recorder_id) {
        recorder_id_ = recorder_id;
        is_gate_open_ = false;
        left_eye_image_index_ = 0;
        right_eye_image_index_ = 0;
        output_directory_ = tobii_gonfig.output_directory;
    }

    ~TobiiMaker() {
        stop();
    }

    void init() {
        // empty
    }

    void setup() {
        std::cout << "[TobiiMaker " << recorder_id_ << "] Starting setup...\n";
        try {
            _createOutputDirectories();
            _setupGazeCSV();
            std::cout << "[TobiiMaker " << recorder_id_ << "] Setup completed\n";
        } catch (const std::exception& e) {
            std::cout << "[TobiiMaker " << recorder_id_ << "] Setup failed: " << e.what() << "\n";
            throw;
        }
    }

    void warmup() {
        std::cout << "[TobiiMaker " << recorder_id_ << "] warmup done\n";
    }

    void run() {
        is_gate_open_ = true;
        std::cout << "[TobiiMaker " << recorder_id_ << "] started\n";
    }

    void stop() {
        is_gate_open_ = false;
        if (gaze_csv_file_.is_open()) {
            gaze_csv_file_.close();
        }
        std::cout << "[TobiiMaker " << recorder_id_ << "] stopped\n";
    }

    void writeGazeData(const TobiiGazeData& data) {
        if (!is_gate_open_.load()) return;
        _saveGazeCSV(data);
    }

    void writeEyeImage(const TobiiEyeImageData& data) {
        if (!is_gate_open_.load()) return;
        _saveEyeImageJPG(data);
    }

private:
    void _createOutputDirectories() {
        try {
            if (!std::filesystem::exists(output_directory_)) {
                std::filesystem::create_directories(output_directory_);
            }
            
            std::string eye_images_dir = output_directory_ + "/eye_images";
            if (!std::filesystem::exists(eye_images_dir)) {
                std::filesystem::create_directories(eye_images_dir);
            }
            
        } catch (const std::exception& e) {
            throw TobiiMakerError("Failed to create output directory: " + std::string(e.what()), 2201);
        }
    }

    void _setupGazeCSV() {
        if (!tobii_gonfig.save_gaze_csv) return;
        
        std::string csv_path = output_directory_ + "/gaze_data.csv";
        gaze_csv_file_.open(csv_path);
        
        if (!gaze_csv_file_.is_open()) {
            throw TobiiMakerError("Failed to create gaze CSV file", 2202);
        }
        
        // Write CSV header
        gaze_csv_file_ << "timestamp_us,device_timestamp,"
                      << "left_x,left_y,left_valid,left_pupil,"
                      << "right_x,right_y,right_valid,right_pupil,"
                      << "left_3d_x,left_3d_y,left_3d_z,left_3d_valid,"
                      << "right_3d_x,right_3d_y,right_3d_z,right_3d_valid\n";
    }

    void _saveGazeCSV(const TobiiGazeData& data) {
        if (!gaze_csv_file_.is_open()) return;
        
        auto time_since_epoch = data.timestamp_.time_since_epoch();
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time_since_epoch).count();
        
        gaze_csv_file_ << microseconds << ","
                      << data.device_timestamp_ << ","
                      << data.left_eye_.x << ","
                      << data.left_eye_.y << ","
                      << (data.left_eye_.validity ? 1 : 0) << ","
                      << data.left_eye_.pupil_diameter << ","
                      << data.right_eye_.x << ","
                      << data.right_eye_.y << ","
                      << (data.right_eye_.validity ? 1 : 0) << ","
                      << data.right_eye_.pupil_diameter << ","
                      << data.left_eye_3d_.x << ","
                      << data.left_eye_3d_.y << ","
                      << data.left_eye_3d_.z << ","
                      << (data.left_eye_3d_.validity ? 1 : 0) << ","
                      << data.right_eye_3d_.x << ","
                      << data.right_eye_3d_.y << ","
                      << data.right_eye_3d_.z << ","
                      << (data.right_eye_3d_.validity ? 1 : 0) << "\n";
        
        // 즉시 flush하여 데이터 저장 보장
        gaze_csv_file_.flush();
    }

    void _saveEyeImageJPG(const TobiiEyeImageData& data) {
        if (!tobii_gonfig.save_eye_images) return;
        
        std::string camera_name = (data.camera_id_ == 0) ? "left" : "right";
        int& image_index = (data.camera_id_ == 0) ? left_eye_image_index_ : right_eye_image_index_;
        
        auto time_since_epoch = data.timestamp_.time_since_epoch();
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time_since_epoch).count();
        
        std::stringstream filename;
        filename << output_directory_ << "/eye_images/" 
                 << camera_name << "_eye_" 
                 << std::setfill('0') << std::setw(6) << ++image_index
                 << "_" << microseconds << ".jpg";
        
        std::ofstream file(filename.str(), std::ios::binary);
        if (file.is_open()) {
            file.write(reinterpret_cast<const char*>(data.raw_data_.data()), data.raw_data_.size());
            file.close();
        }
    }
};

// ==========================================
// TobiiRecorder Method Implementations (여기에 추가)
// ==========================================

void TobiiRecorder::_gazeDataCallback(TobiiResearchGazeData* gaze_data) {
    try {
        if (!is_recording_.load()) return;
        
        gaze_count_.fetch_add(1);
        
        if (tobii_maker_ || tobii_streamer_) {
            TobiiGazeData converted_data;
            converted_data.timestamp_ = std::chrono::system_clock::now();
            converted_data.device_timestamp_ = gaze_data->device_time_stamp;
            
            // Left eye
            converted_data.left_eye_.x = gaze_data->left_eye.gaze_point.position_on_display_area.x;
            converted_data.left_eye_.y = gaze_data->left_eye.gaze_point.position_on_display_area.y;
            converted_data.left_eye_.validity = gaze_data->left_eye.gaze_point.validity == TOBII_RESEARCH_VALIDITY_VALID;
            converted_data.left_eye_.pupil_diameter = gaze_data->left_eye.pupil_data.diameter;
            
            // Right eye
            converted_data.right_eye_.x = gaze_data->right_eye.gaze_point.position_on_display_area.x;
            converted_data.right_eye_.y = gaze_data->right_eye.gaze_point.position_on_display_area.y;
            converted_data.right_eye_.validity = gaze_data->right_eye.gaze_point.validity == TOBII_RESEARCH_VALIDITY_VALID;
            converted_data.right_eye_.pupil_diameter = gaze_data->right_eye.pupil_data.diameter;
            
            // 3D positions
            converted_data.left_eye_3d_.x = gaze_data->left_eye.gaze_origin.position_in_user_coordinates.x;
            converted_data.left_eye_3d_.y = gaze_data->left_eye.gaze_origin.position_in_user_coordinates.y;
            converted_data.left_eye_3d_.z = gaze_data->left_eye.gaze_origin.position_in_user_coordinates.z;
            converted_data.left_eye_3d_.validity = gaze_data->left_eye.gaze_origin.validity == TOBII_RESEARCH_VALIDITY_VALID;
            
            converted_data.right_eye_3d_.x = gaze_data->right_eye.gaze_origin.position_in_user_coordinates.x;
            converted_data.right_eye_3d_.y = gaze_data->right_eye.gaze_origin.position_in_user_coordinates.y;
            converted_data.right_eye_3d_.z = gaze_data->right_eye.gaze_origin.position_in_user_coordinates.z;
            converted_data.right_eye_3d_.validity = gaze_data->right_eye.gaze_origin.validity == TOBII_RESEARCH_VALIDITY_VALID;
            
            if (tobii_maker_) {
                tobii_maker_->writeGazeData(converted_data);
            }
            
            if (tobii_streamer_) {
                tobii_streamer_->updateGaze(converted_data);
            }
        }
    } catch (const std::exception& e) {
        std::cout << "[TobiiRecorder " << recorder_id_ << "] Gaze callback error: " << e.what() << "\n";
    }
}

void TobiiRecorder::_eyeImageCallback(TobiiResearchEyeImage* eye_image) {
    try {
        if (!is_recording_.load()) return;
        
        if (eye_image->camera_id == 0) {
            left_eye_image_count_.fetch_add(1);
        } else {
            right_eye_image_count_.fetch_add(1);
        }
        
        if (tobii_maker_) {
            TobiiEyeImageData image_data;
            image_data.timestamp_ = std::chrono::system_clock::now();
            image_data.device_timestamp_ = eye_image->device_time_stamp;
            image_data.camera_id_ = eye_image->camera_id;
            image_data.width_ = eye_image->width;
            image_data.height_ = eye_image->height;
            
            image_data.raw_data_.resize(eye_image->data_size);
            std::memcpy(image_data.raw_data_.data(), eye_image->data, eye_image->data_size);
            
            tobii_maker_->writeEyeImage(image_data);
        }
    } catch (const std::exception& e) {
        std::cout << "[TobiiRecorder " << recorder_id_ << "] Eye image callback error: " << e.what() << "\n";
    }
}

void TobiiRecorder::_bufferOverflowCallback(TobiiResearchNotification* notification) {
    if (notification->notification_type == TOBII_RESEARCH_NOTIFICATION_STREAM_BUFFER_OVERFLOW) {
        std::cout << "[TobiiRecorder " << recorder_id_ << " Warning] Buffer overflow in " 
                  << notification->value.text << " stream\n";
    }
}


// ==========================================
// TobiiMultiManager Class
// ==========================================

class TobiiMultiManager {
private:
    // managers
    std::unique_ptr<TobiiManager> tobii_manager_;
    std::unique_ptr<TobiiRecorder> tobii_recorder_;
    std::unique_ptr<TobiiMaker> tobii_maker_;
    std::unique_ptr<TobiiStreamer> tobii_streamer_;
    
    // control
    std::atomic<bool> record_signal_;
    std::atomic<bool> stop_signal_;
    std::thread monitor_thread_;

public:
    TobiiMultiManager() {
        record_signal_ = false;
        stop_signal_ = false;
    }

    ~TobiiMultiManager() {
        stop();
    }

    void init() {
        tobii_manager_ = std::make_unique<TobiiManager>();
        tobii_recorder_ = std::make_unique<TobiiRecorder>(0);
        tobii_maker_ = std::make_unique<TobiiMaker>(0);
        tobii_streamer_ = std::make_unique<TobiiStreamer>(0);
    }

    void setup() {
        if (!tobii_gonfig.enable_tobii) {
            std::cout << "[TobiiMultiManager] Tobii disabled\n";
            return;
        }

        std::cout << "[TobiiMultiManager] Starting setup...\n";
        try {
            std::cout << "[TobiiMultiManager] Setting up TobiiManager...\n";
            tobii_manager_->setup();
            
            std::cout << "[TobiiMultiManager] Setting up TobiiRecorder...\n";
            tobii_recorder_->setup(tobii_manager_->getEyeTracker());
            
            std::cout << "[TobiiMultiManager] Setting up TobiiMaker...\n";
            tobii_maker_->setup();
            
            std::cout << "[TobiiMultiManager] Setting up TobiiStreamer...\n";
            tobii_streamer_->setup();

            // 연결 설정
            tobii_recorder_->setTobiiMaker(tobii_maker_.get());
            tobii_recorder_->setTobiiStreamer(tobii_streamer_.get());
            
            std::cout << "[TobiiMultiManager] Setup completed successfully\n";
        } catch (const std::exception& e) {
            std::cout << "[TobiiMultiManager] Setup failed: " << e.what() << "\n";
            throw TobiiManagerError("Setup failed: " + std::string(e.what()), 2300);
        }
    }

    void warmup() {
        if (!tobii_gonfig.enable_tobii) return;
        
        tobii_manager_->warmup();
        tobii_recorder_->warmup();
        tobii_maker_->warmup();
        tobii_streamer_->warmup();
    }

    void run() {
        if (!tobii_gonfig.enable_tobii) return;
        
        tobii_manager_->run();
        tobii_recorder_->run();
        tobii_maker_->run();
        tobii_streamer_->run();
        
        record_signal_ = true;
        monitor_thread_ = std::thread([this]() { _monitorRecording(); });
    }

    void stop() {
        record_signal_ = false;
        stop_signal_ = true;
        
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
        
        if (tobii_gonfig.enable_tobii) {
            tobii_recorder_->stop();
            tobii_maker_->stop();
            tobii_streamer_->stop();
            tobii_manager_->stop();
        }
        
        _printSummary();
    }

private:
    void _monitorRecording() {
        auto start_time = std::chrono::steady_clock::now();
        
        std::cout << "[TobiiMultiManager] Recording started (continuous mode)...\n";
        
        // 무한 루프 - stop_signal_이 true가 될 때까지 계속
        while (!stop_signal_.load()) {
            sleep_ms(1000);
            
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time
            ).count();
            
            std::cout << "[TobiiMultiManager] Recording... " << elapsed << "s"
                      << " | Gaze: " << tobii_recorder_->getGazeCount()
                      << " | Left Eye: " << tobii_recorder_->getLeftEyeImageCount()
                      << " | Right Eye: " << tobii_recorder_->getRightEyeImageCount() << "\r" << std::flush;
            
            // 스트리밍 상태 업데이트
            if (tobii_streamer_) {
                tobii_streamer_->updateStatus(
                    tobii_recorder_->getGazeCount(),
                    tobii_recorder_->getLeftEyeImageCount(),
                    tobii_recorder_->getRightEyeImageCount(),
                    elapsed
                );
            }
        }
        
        std::cout << "\n[TobiiMultiManager] Recording stopped by user\n";
    }

    void _printSummary() {
        if (!tobii_gonfig.enable_tobii) return;
        
        std::cout << "\n=== Tobii Recording Summary ===\n";
        std::cout << "Gaze data points: " << tobii_recorder_->getGazeCount() << "\n";
        std::cout << "Left eye images: " << tobii_recorder_->getLeftEyeImageCount() << "\n";
        std::cout << "Right eye images: " << tobii_recorder_->getRightEyeImageCount() << "\n";
        std::cout << "Output directory: " << tobii_gonfig.output_directory << "\n";
    }
};

// ==========================================
// Main Function
// ==========================================

int main(int argc, char* argv[]) {
    // Setup exception handlers first
    ExceptionHandler::setupTerminateHandler();
    
    std::cout << "=== Tobii Eye Tracker Test Program ===\n";
    std::cout << "[Debug] Exception handlers initialized\n";
    
    try {
        // Parse command line arguments
        std::cout << "[Debug] Parsing command line arguments...\n";
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--output" && i + 1 < argc) {
                tobii_gonfig.output_directory = argv[i + 1];
                i++;
            } else if (arg == "--stream_fps" && i + 1 < argc) {
                tobii_gonfig.stream_fps = std::atoi(argv[i + 1]);
                i++;
            } else if (arg == "--no_stream") {
                tobii_gonfig.enable_streaming = false;
            }
        }
        
        std::cout << "[Config] Output directory: " << tobii_gonfig.output_directory << "\n";
        std::cout << "[Config] Stream FPS: " << tobii_gonfig.stream_fps << "\n";
        
        std::cout << "[Debug] Creating TobiiMultiManager...\n";
        TobiiMultiManager multiManager;
        
        std::cout << "[Debug] Calling init()...\n";
        multiManager.init();
        
        std::cout << "[Debug] Calling setup()...\n";
        multiManager.setup();
        
        std::cout << "[Debug] Calling warmup()...\n";
        multiManager.warmup();
        
        std::cout << "[Debug] Calling run()...\n";
        multiManager.run();
        
        // 무한 대기 - 외부 신호로만 종료 가능
        std::cout << "[Info] Recording in continuous mode...\n";
        std::cout << "[Info] Use stop API or Ctrl+C to terminate\n";
        
        // 프로그램이 종료되지 않도록 무한 대기
        while (true) {
            sleep_ms(1000);
            // 여기서는 아무것도 하지 않고 대기만 함
            // FastAPI의 stop 요청이나 Ctrl+C로만 종료 가능
        }
        
    } catch (const TobiiManagerError& e) {
        std::cout << "[ERROR] TobiiManager: " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        return e.getCode();
    } catch (const TobiiRecorderError& e) {
        std::cout << "[ERROR] TobiiRecorder: " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        return e.getCode();
    } catch (const TobiiMakerError& e) {
        std::cout << "[ERROR] TobiiMaker: " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        return e.getCode();
    } catch (const Error& e) {
        std::cout << "[ERROR] " << e.getType() << ": " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        return e.getCode();
    } catch (const std::runtime_error& e) {
        std::cout << "[ERROR] Runtime error: " << e.what() << "\n";
        return -2;
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Standard exception: " << e.what() << "\n";
        return -3;
    } catch (...) {
        std::cout << "[ERROR] Unknown exception occurred\n";
        return -4;
    }
    
    return 0;
}