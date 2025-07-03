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
#include <array>
#include <mutex>
#include <algorithm>
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

struct GazeVector {
    double x, y, z;
    double magnitude;
    bool validity;
};

struct ScreenIntersection {
    double x, y, z;
    double screen_x, screen_y;
    bool valid_intersection;
};

struct QualityMetrics {
    double tracking_confidence;
    double eye_distance;
    double head_position_stability;
};

struct Enhanced3DGazeData {
    std::chrono::system_clock::time_point timestamp_;
    int64_t device_timestamp_;
    
    // Original data
    TobiiGazeData::EyeGazePoint left_eye_, right_eye_;
    TobiiGazeData::EyePosition3D left_eye_3d_, right_eye_3d_;
    
    // Enhanced calculations
    GazeVector left_gaze_vector_, right_gaze_vector_, averaged_gaze_vector_;
    ScreenIntersection screen_hit_point_;
    QualityMetrics quality_;
};

struct ScreenConfig {
    double width_mm = 531.0;
    double height_mm = 298.0;
    double distance_mm = 600.0;
    double pixel_width = 1920;
    double pixel_height = 1080;
    
    struct Pose {
        double x = 0, y = 0, z = 0;
        double roll = 0, pitch = 0, yaw = 0;
    } screen_pose_;
};

// ==========================================
// Config
// ==========================================

struct TobiiConfig {
    bool enable_tobii = true;
    std::string output_directory = "tobii_output";
    bool save_gaze_csv = true;
    bool save_eye_images = true;
    int warmup_duration = 2;

    // calibration settings - 추가
    bool load_calibration = false;
    std::string calibration_file_path = "";  // 빈 문자열이면 자동 검색
    
    // streaming settings
    bool enable_streaming = true;
    int stream_fps = 30;
    bool stream_gaze = true;
    bool stream_status = true;
    
    // Enhanced 3D visualization - 추가
    bool enable_enhanced_3d = true;
    
    struct ScreenSettings {
        double width_mm = 510.0;
        double height_mm = 287.0; 
        double distance_mm = 600.0;
        double pixel_width = 1920;
        double pixel_height = 1080;
    } screen;
};


TobiiConfig tobii_gonfig;

struct EnhancedTobiiConfig {
    TobiiConfig base_config;
    
    struct VisualizationConfig {
        bool enable_3d_calculation = true;
        bool stream_gaze_vectors = true;
        bool stream_screen_intersections = true;
        bool stream_trajectory_buffer = true;
        
        ScreenConfig screen;
        
        int trajectory_buffer_size = 1000;
        double min_tracking_confidence = 0.7;
    } visualization;
};


EnhancedTobiiConfig enhanced_tobii_config;

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
// GazeCalculator
// ==========================================

class GazeCalculator {
private:
    ScreenConfig screen_config_;
    
    bool isValidData(const TobiiGazeData& data) const {
        auto isValidFloat = [](double val) {
            return !std::isnan(val) && !std::isinf(val);
        };
        
        bool left_valid = data.left_eye_.validity && 
                         isValidFloat(data.left_eye_.x) && 
                         isValidFloat(data.left_eye_.y);
                         
        bool right_valid = data.right_eye_.validity && 
                          isValidFloat(data.right_eye_.x) && 
                          isValidFloat(data.right_eye_.y);
                          
        bool left_3d_valid = data.left_eye_3d_.validity &&
                            isValidFloat(data.left_eye_3d_.x) &&
                            isValidFloat(data.left_eye_3d_.y) &&
                            isValidFloat(data.left_eye_3d_.z);
                            
        bool right_3d_valid = data.right_eye_3d_.validity &&
                             isValidFloat(data.right_eye_3d_.x) &&
                             isValidFloat(data.right_eye_3d_.y) &&
                             isValidFloat(data.right_eye_3d_.z);
        
        return (left_valid || right_valid) && (left_3d_valid || right_3d_valid);
    }
    

public:
    GazeCalculator(const ScreenConfig& config = ScreenConfig()) : screen_config_(config) {}
    
    Enhanced3DGazeData calculateEnhancedGaze(const TobiiGazeData& raw_data) {
        Enhanced3DGazeData enhanced;
        enhanced.timestamp_ = raw_data.timestamp_;
        enhanced.device_timestamp_ = raw_data.device_timestamp_;
        enhanced.left_eye_ = raw_data.left_eye_;
        enhanced.right_eye_ = raw_data.right_eye_;
        enhanced.left_eye_3d_ = raw_data.left_eye_3d_;
        enhanced.right_eye_3d_ = raw_data.right_eye_3d_;
        
        // 데이터 유효성 검사 추가
        if (!isValidData(raw_data)) {
            // Invalid data - return with zero values
            enhanced.left_gaze_vector_ = {0, 0, 0, 0, false};
            enhanced.right_gaze_vector_ = {0, 0, 0, 0, false};
            enhanced.averaged_gaze_vector_ = {0, 0, 0, 0, false};
            enhanced.screen_hit_point_ = {0, 0, 0, 0, 0, false};
            enhanced.quality_ = {0, 0, 0};
            return enhanced;
        }
        
        // Calculate gaze vectors
        enhanced.left_gaze_vector_ = calculateGazeDirection(raw_data.left_eye_3d_, raw_data.left_eye_);
        enhanced.right_gaze_vector_ = calculateGazeDirection(raw_data.right_eye_3d_, raw_data.right_eye_);
        enhanced.averaged_gaze_vector_ = calculateAverageGazeVector(enhanced.left_gaze_vector_, enhanced.right_gaze_vector_);
        
        // Calculate screen intersection
        enhanced.screen_hit_point_ = calculateScreenIntersection(enhanced.averaged_gaze_vector_, 
                                                               getAverageEyePosition(raw_data));
        
        // Calculate quality metrics
        enhanced.quality_ = calculateTrackingQuality(raw_data);
        
        return enhanced;
    }
    
    bool isValidData(const TobiiGazeData& data) {
        return !std::isnan(data.left_eye_.x) && 
               !std::isnan(data.right_eye_.x) &&
               data.left_eye_.validity || data.right_eye_.validity;
    }

private:
    GazeVector calculateGazeDirection(const TobiiGazeData::EyePosition3D& eye_pos, 
                                    const TobiiGazeData::EyeGazePoint& gaze_2d) {
        GazeVector vec;
        vec.validity = eye_pos.validity && gaze_2d.validity;
        
        if (!vec.validity) {
            vec.x = vec.y = vec.z = vec.magnitude = 0.0;
            return vec;
        }
        
        // Convert 2D gaze point to 3D direction
        // Assuming screen at z = screen_distance
        double screen_x = (gaze_2d.x - 0.5) * screen_config_.width_mm;
        double screen_y = (gaze_2d.y - 0.5) * screen_config_.height_mm;
        double screen_z = screen_config_.distance_mm;
        
        // Direction vector from eye to screen point
        vec.x = screen_x - eye_pos.x;
        vec.y = screen_y - eye_pos.y;
        vec.z = screen_z - eye_pos.z;
        
        // Normalize
        vec.magnitude = sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
        if (vec.magnitude > 0) {
            vec.x /= vec.magnitude;
            vec.y /= vec.magnitude;
            vec.z /= vec.magnitude;
        }
        
        return vec;
    }
    
    GazeVector calculateAverageGazeVector(const GazeVector& left, const GazeVector& right) {
        GazeVector avg;
        
        if (left.validity && right.validity) {
            avg.x = (left.x + right.x) / 2.0;
            avg.y = (left.y + right.y) / 2.0;
            avg.z = (left.z + right.z) / 2.0;
            avg.magnitude = sqrt(avg.x*avg.x + avg.y*avg.y + avg.z*avg.z);
            avg.validity = true;
        } else if (left.validity) {
            avg = left;
        } else if (right.validity) {
            avg = right;
        } else {
            avg.x = avg.y = avg.z = avg.magnitude = 0.0;
            avg.validity = false;
        }
        
        return avg;
    }
    
    ScreenIntersection calculateScreenIntersection(const GazeVector& gaze_vec, 
                                                 const TobiiGazeData::EyePosition3D& eye_pos) {
        ScreenIntersection intersection;
        intersection.valid_intersection = gaze_vec.validity && eye_pos.validity;
        
        if (!intersection.valid_intersection) {
            intersection.x = intersection.y = intersection.z = 0.0;
            intersection.screen_x = intersection.screen_y = 0.0;
            return intersection;
        }
        
        // Ray-plane intersection (assuming screen at z = distance)
        double t = (screen_config_.distance_mm - eye_pos.z) / gaze_vec.z;
        
        intersection.x = eye_pos.x + t * gaze_vec.x;
        intersection.y = eye_pos.y + t * gaze_vec.y;
        intersection.z = screen_config_.distance_mm;
        
        // Convert to screen coordinates (0-1)
        intersection.screen_x = (intersection.x / screen_config_.width_mm) + 0.5;
        intersection.screen_y = (intersection.y / screen_config_.height_mm) + 0.5;
        
        // Check if within screen bounds
        intersection.valid_intersection = (intersection.screen_x >= 0 && intersection.screen_x <= 1 &&
                                         intersection.screen_y >= 0 && intersection.screen_y <= 1);
        
        return intersection;
    }
    
    QualityMetrics calculateTrackingQuality(const TobiiGazeData& data) {
        QualityMetrics quality;
        
        // Simple confidence based on validity
        int valid_count = (data.left_eye_.validity ? 1 : 0) + (data.right_eye_.validity ? 1 : 0) +
                         (data.left_eye_3d_.validity ? 1 : 0) + (data.right_eye_3d_.validity ? 1 : 0);
        quality.tracking_confidence = valid_count / 4.0;
        
        // Eye distance
        if (data.left_eye_3d_.validity && data.right_eye_3d_.validity) {
            double dx = data.left_eye_3d_.x - data.right_eye_3d_.x;
            double dy = data.left_eye_3d_.y - data.right_eye_3d_.y;
            double dz = data.left_eye_3d_.z - data.right_eye_3d_.z;
            quality.eye_distance = sqrt(dx*dx + dy*dy + dz*dz);
        } else {
            quality.eye_distance = 0.0;
        }
        
        // Head stability (simplified)
        quality.head_position_stability = quality.tracking_confidence;
        
        return quality;
    }
    
    TobiiGazeData::EyePosition3D getAverageEyePosition(const TobiiGazeData& data) {
        TobiiGazeData::EyePosition3D avg;
        
        if (data.left_eye_3d_.validity && data.right_eye_3d_.validity) {
            avg.x = (data.left_eye_3d_.x + data.right_eye_3d_.x) / 2.0;
            avg.y = (data.left_eye_3d_.y + data.right_eye_3d_.y) / 2.0;
            avg.z = (data.left_eye_3d_.z + data.right_eye_3d_.z) / 2.0;
            avg.validity = true;
        } else if (data.left_eye_3d_.validity) {
            avg = data.left_eye_3d_;
        } else if (data.right_eye_3d_.validity) {
            avg = data.right_eye_3d_;
        } else {
            avg.x = avg.y = avg.z = 0.0;
            avg.validity = false;
        }
        
        return avg;
    }
};


// ==========================================
// GazeTrajectoryBuffer
// ==========================================

class GazeTrajectoryBuffer {
private:
    static const size_t BUFFER_SIZE = 1000;
    std::array<Enhanced3DGazeData, BUFFER_SIZE> buffer_;
    
    size_t write_index_ = 0;
    size_t count_ = 0;
    std::mutex buffer_mutex_;
    
public:
    void addPoint(const Enhanced3DGazeData& data) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        buffer_[write_index_] = data;
        write_index_ = (write_index_ + 1) % BUFFER_SIZE;
        if (count_ < BUFFER_SIZE) count_++;
    }
    
    std::vector<Enhanced3DGazeData> getRecentTrajectory(size_t num_points) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        std::vector<Enhanced3DGazeData> trajectory;
        
        size_t points_to_get = (num_points < count_) ? num_points : count_;
        trajectory.reserve(points_to_get);
        
        for (size_t i = 0; i < points_to_get; i++) {
            size_t index = (write_index_ - points_to_get + i + BUFFER_SIZE) % BUFFER_SIZE;
            trajectory.push_back(buffer_[index]);
        }
        
        return trajectory;
    }
};


// ==========================================
// TobiiStreamer Class
// ==========================================

class TobiiStreamer {
protected:
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
        gaze_interval_ms_ = 1000 / tobii_gonfig.stream_fps;
        status_interval_ms_ = 1000;
        total_gaze_count_ = 0;
    }

    virtual ~TobiiStreamer() {
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

    virtual void updateGaze(const TobiiGazeData& data) {
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

protected:
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


class EnhancedTobiiStreamer : public TobiiStreamer {
private:
    GazeCalculator calculator_;
    GazeTrajectoryBuffer trajectory_buffer_;
    ScreenConfig screen_config_;
    
public:
    EnhancedTobiiStreamer(int streamer_id, const ScreenConfig& config = ScreenConfig()) 
        : TobiiStreamer(streamer_id), calculator_(config), screen_config_(config) {}
    
    void updateGaze(const TobiiGazeData& data) override {  // override 추가
        updateEnhancedGaze(data);
    }

    void updateEnhancedGaze(const TobiiGazeData& data) {
        if (!is_streaming_.load() || !tobii_gonfig.stream_gaze) {
            return;
        }

        // Calculate enhanced data
        Enhanced3DGazeData enhanced = calculator_.calculateEnhancedGaze(data);
        trajectory_buffer_.addPoint(enhanced);
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_gaze_output_).count();
            
        if (elapsed >= gaze_interval_ms_) {
            _outputEnhancedGazeData(enhanced);
            last_gaze_output_ = now;
            total_gaze_count_.fetch_add(1);
            
            // 주기적으로 상태 출력
            static int debug_counter = 0;
            if (++debug_counter % 30 == 1) {  // 1초마다 한 번
                std::cout << "[EnhancedTobiiStreamer] Processed " << total_gaze_count_.load() 
                        << " enhanced gaze points\n";
            }
        }
    }
    
private:
    void _outputEnhancedGazeData(const Enhanced3DGazeData& data) {
        std::ostringstream gaze_json;
        gaze_json << std::fixed << std::setprecision(6);
        
        auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            data.timestamp_.time_since_epoch()).count();
            
        gaze_json << "{"
                 << "\"timestamp\":" << timestamp_ms << ","
                 << "\"device_timestamp\":" << data.device_timestamp_ << ","
                 << "\"3d_visualization\":{"
                 << "\"left_eye_ray\":{"
                 << "\"origin\":[" << data.left_eye_3d_.x << "," << data.left_eye_3d_.y << "," << data.left_eye_3d_.z << "],"
                 << "\"direction\":[" << data.left_gaze_vector_.x << "," << data.left_gaze_vector_.y << "," << data.left_gaze_vector_.z << "],"
                 << "\"valid\":" << (data.left_gaze_vector_.validity ? "true" : "false")
                 << "},"
                 << "\"right_eye_ray\":{"
                 << "\"origin\":[" << data.right_eye_3d_.x << "," << data.right_eye_3d_.y << "," << data.right_eye_3d_.z << "],"
                 << "\"direction\":[" << data.right_gaze_vector_.x << "," << data.right_gaze_vector_.y << "," << data.right_gaze_vector_.z << "],"
                 << "\"valid\":" << (data.right_gaze_vector_.validity ? "true" : "false")
                 << "},"
                 << "\"average_ray\":{"
                 << "\"direction\":[" << data.averaged_gaze_vector_.x << "," << data.averaged_gaze_vector_.y << "," << data.averaged_gaze_vector_.z << "],"
                 << "\"valid\":" << (data.averaged_gaze_vector_.validity ? "true" : "false")
                 << "},"
                 << "\"screen_hit\":{"
                 << "\"point_3d\":[" << data.screen_hit_point_.x << "," << data.screen_hit_point_.y << "," << data.screen_hit_point_.z << "],"
                 << "\"screen_coords\":[" << data.screen_hit_point_.screen_x << "," << data.screen_hit_point_.screen_y << "],"
                 << "\"valid\":" << (data.screen_hit_point_.valid_intersection ? "true" : "false")
                 << "},"
                 << "\"quality\":{"
                 << "\"confidence\":" << data.quality_.tracking_confidence << ","
                 << "\"eye_distance\":" << data.quality_.eye_distance << ","
                 << "\"stability\":" << data.quality_.head_position_stability
                 << "},"
                 << "\"screen_config\":{"
                 << "\"width_mm\":" << screen_config_.width_mm << ","
                 << "\"height_mm\":" << screen_config_.height_mm << ","
                 << "\"distance_mm\":" << screen_config_.distance_mm
                 << "}"
                 << "}"
                 << "}";
        
        _outputEvent("ENHANCED_GAZE_DATA", gaze_json.str());
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

    // calibration info - 추가
    bool calibration_loaded_;
    std::string loaded_calibration_file_;
public:
    TobiiManager() {
        enabled_ = false;
        eye_tracker_ = nullptr;
        calibration_loaded_ = false;
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
            _loadCalibrationData();  // calibration 로딩 추가
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
    bool isCalibrationLoaded() const { return calibration_loaded_; }
    const std::string& getLoadedCalibrationFile() const { return loaded_calibration_file_; }
private:
    void _loadCalibrationData() {
        // calibration_file_path가 명시적으로 지정된 경우에만 로딩 시도
        if (tobii_gonfig.calibration_file_path.empty()) {
            std::cout << "[TobiiManager] No calibration file specified - running without calibration\n";
            calibration_loaded_ = false;
            return;
        }
        
        // 파일 경로가 지정되었으면 자동으로 load_calibration을 true로 설정
        tobii_gonfig.load_calibration = true;
        
        std::string calibration_file = tobii_gonfig.calibration_file_path;
        
        // 파일 존재 여부 확인
        if (!std::filesystem::exists(calibration_file)) {
            std::cout << "[TobiiManager] Specified calibration file not found: " 
                      << calibration_file << " - running without calibration\n";
            calibration_loaded_ = false;
            return;
        }
        
        try {
            std::cout << "[TobiiManager] Loading calibration from: " << calibration_file << "\n";
            
            // Read calibration file
            std::ifstream file(calibration_file, std::ios::binary | std::ios::ate);
            if (!file.is_open()) {
                throw TobiiManagerError("Failed to open calibration file: " + calibration_file, 2006);
            }
            
            std::streamsize file_size = file.tellg();
            file.seekg(0, std::ios::beg);
            
            if (file_size <= 0) {
                throw TobiiManagerError("Calibration file is empty: " + calibration_file, 2007);
            }
            
            // Allocate buffer and read data
            std::vector<uint8_t> calibration_buffer(file_size);
            if (!file.read(reinterpret_cast<char*>(calibration_buffer.data()), file_size)) {
                throw TobiiManagerError("Failed to read calibration file: " + calibration_file, 2008);
            }
            file.close();
            
            // Create calibration data structure
            TobiiResearchCalibrationData calibration_data;
            calibration_data.data = calibration_buffer.data();
            calibration_data.size = static_cast<size_t>(file_size);
            
            // Apply calibration to eye tracker
            TobiiResearchStatus status = tobii_research_apply_calibration_data(eye_tracker_, &calibration_data);
            
            if (status != TOBII_RESEARCH_STATUS_OK) {
                throw TobiiManagerError("Failed to apply calibration data. Status: " + std::to_string(status), 2009);
            }
            
            calibration_loaded_ = true;
            loaded_calibration_file_ = calibration_file;
            
            std::cout << "[TobiiManager] Calibration successfully loaded and applied (" 
                      << file_size << " bytes)\n";
                      
        } catch (const std::exception& e) {
            std::cout << "[TobiiManager] Failed to load calibration: " << e.what() 
                      << " - continuing without calibration\n";
            calibration_loaded_ = false;
            // Continue without calibration rather than failing completely
        }
    }

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
            
            // EnhancedTobiiStreamer 대신 기존 방식 사용
            EnhancedTobiiStreamer* enhanced_streamer = dynamic_cast<EnhancedTobiiStreamer*>(tobii_streamer_);
            if (enhanced_streamer) {
                enhanced_streamer->updateEnhancedGaze(converted_data);
            } else if (tobii_streamer_) {
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

        if (tobii_gonfig.enable_enhanced_3d) {
            ScreenConfig screen_config;
            screen_config.width_mm = tobii_gonfig.screen.width_mm;
            screen_config.height_mm = tobii_gonfig.screen.height_mm;
            screen_config.distance_mm = tobii_gonfig.screen.distance_mm;
            screen_config.pixel_width = tobii_gonfig.screen.pixel_width;
            screen_config.pixel_height = tobii_gonfig.screen.pixel_height;
            
            tobii_streamer_ = std::make_unique<EnhancedTobiiStreamer>(0, screen_config);
            std::cout << "[TobiiMultiManager] Enhanced 3D visualization enabled\n";
        } else {
            tobii_streamer_ = std::make_unique<TobiiStreamer>(0);
            std::cout << "[TobiiMultiManager] Basic streaming enabled\n";
        }
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
        
        // calibration 상태 추가
        if (tobii_manager_->isCalibrationLoaded()) {
            std::cout << "Calibration: Loaded (" << tobii_manager_->getLoadedCalibrationFile() << ")\n";
        } else {
            std::cout << "Calibration: Not loaded\n";
        }
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
            } else if (arg == "--enhanced_3d" && i + 1 < argc) {
                tobii_gonfig.enable_enhanced_3d = (std::string(argv[i + 1]) == "true");
                i++;
            } else if (arg == "--calibration_file" && i + 1 < argc) {
                tobii_gonfig.calibration_file_path = argv[i + 1];
                // 파일 경로가 지정되면 자동으로 calibration 로딩 활성화
                tobii_gonfig.load_calibration = true;
                i++;
            } else if (arg == "--screen_distance" && i + 1 < argc) {
                tobii_gonfig.screen.distance_mm = std::atof(argv[i + 1]);
                i++;
            } else if (arg == "--screen_width" && i + 1 < argc) {
                tobii_gonfig.screen.width_mm = std::atof(argv[i + 1]);
                i++;
            } else if (arg == "--screen_height" && i + 1 < argc) {
                tobii_gonfig.screen.height_mm = std::atof(argv[i + 1]);
                i++;
            }
        }

        std::cout << "[Config] Output directory: " << tobii_gonfig.output_directory << "\n";
        std::cout << "[Config] Stream FPS: " << tobii_gonfig.stream_fps << "\n";
        std::cout << "[Config] Enhanced 3D: " << (tobii_gonfig.enable_enhanced_3d ? "enabled" : "disabled") << "\n";
        
        if (tobii_gonfig.load_calibration && !tobii_gonfig.calibration_file_path.empty()) {
            std::cout << "[Config] Calibration file: " << tobii_gonfig.calibration_file_path << "\n";
        } else {
            std::cout << "[Config] Calibration: Not specified (running without calibration)\n";
        }
        
        std::cout << "[Config] Screen: " << tobii_gonfig.screen.width_mm << "x" << tobii_gonfig.screen.height_mm << "mm at " << tobii_gonfig.screen.distance_mm << "mm\n";
                
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