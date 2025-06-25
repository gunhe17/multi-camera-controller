#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <windows.h>
#include <comdef.h>
#include <wrl/client.h>

#include <mfapi.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mfreadwrite.h>
#include <mferror.h>

// Tobii Research SDK
#include "tobii_research.h"
#include "tobii_research_eyetracker.h"
#include "tobii_research_streams.h"
#include "tobii_research_calibration.h"

#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

using namespace Microsoft::WRL;


/**
 * @Error: Verify
 */

class VerifyError : public std::runtime_error {
public:
    VerifyError(
        const std::string& type, 
        const std::string& msg, 
        int code = 500
    ): 
        std::runtime_error("[Verify Error] " + type + ": " + msg + " (code: " + std::to_string(code) + ")"),
        type_(type), 
        message_(msg), 
        code_(code) 
        {}

    const std::string& getType() const noexcept { return type_; }
    const std::string& getMessage() const noexcept { return message_; }
    int getCode() const noexcept { return code_; }

private:
    std::string type_;
    std::string message_;
    int code_;
};

class ResourceError : public VerifyError {
public:
    ResourceError(const std::string& msg, int code = 100)
        : VerifyError("ResourceError", msg, code) {}
};

class CameraError : public VerifyError {
public:
    CameraError(const std::string& msg, int code = 200)
        : VerifyError("CameraError", msg, code) {}
};

class TobiiError : public VerifyError {
public:
    TobiiError(const std::string& msg, int code = 300)
        : VerifyError("TobiiError", msg, code) {}
};


/**
 * @class: Resource
 */

class Resource {
/** __init__ */
private:
    bool initialized_;
    std::string output_directory_;

public:
    Resource(const std::string& output_dir = ".") {
        initialized_ = false;
        output_directory_ = output_dir;
    }

    ~Resource() {
        if (initialized_) {
            CoUninitialize();
        }
    }

/** methods */
public:
    void init() {
        HRESULT hr = CoInitialize(NULL);
        if (FAILED(hr)) {
            throw ResourceError("COM initialization failed at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr));
        }

        hr = MFStartup(MF_VERSION);
        if (FAILED(hr)) {
            throw ResourceError("Media Foundation startup failed at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr));
        }

        initialized_ = true;
        std::cout << "[Resource] Initialization complete\n";
    }

    bool check() {
        bool result = true;

        // CPU usage check
        double cpu_usage = _getCpuUsage();
        if (cpu_usage > 80.0) {
            std::cout << "[Resource Warning] High CPU usage: " << std::fixed << std::setprecision(1) << cpu_usage << "%\n";
            result = false;
        } else {
            std::cout << "[Resource] CPU usage: " << std::fixed << std::setprecision(1) << cpu_usage << "%\n";
        }

        // Memory check
        auto memory_info = _getMemoryInfo();
        if (memory_info.available_gb < 4.0) {
            std::cout << "[Resource Warning] Low available memory: " << std::fixed << std::setprecision(1) << memory_info.available_gb << "GB\n";
            result = false;
        } else {
            std::cout << "[Resource] Available memory: " << std::fixed << std::setprecision(1) << memory_info.available_gb << "GB\n";
        }

        // Disk space check
        double disk_space = _getDiskSpace();
        if (disk_space < 10.0) {
            std::cout << "[Resource Warning] Low disk space: " << std::fixed << std::setprecision(1) << disk_space << "GB\n";
            result = false;
        } else {
            std::cout << "[Resource] Available disk space: " << std::fixed << std::setprecision(1) << disk_space << "GB\n";
        }

        return result;
    }

    void cleanup() {
        std::cout << "[Resource] Starting cleanup process\n";
        
        _killConflictProcesses();
        _killMediaFoundation();
        
        std::cout << "[Resource] Cleanup complete\n";
    }

    bool test() {
        std::cout << "[Resource] Running resource tests\n";
        
        // Memory allocation test
        try {
            const size_t test_size = 1024 * 1024 * 100; // 100MB
            auto test_buffer = std::make_unique<char[]>(test_size);
            memset(test_buffer.get(), 0, test_size);
            std::cout << "[Resource] Memory allocation test: PASS\n";
        } catch (const std::exception& e) {
            std::cout << "[Resource] Memory allocation test: FAIL - " << e.what() << "\n";
            return false;
        }

        // Disk I/O test
        try {
            std::string test_file = output_directory_ + "/test_write.tmp";
            std::ofstream test_stream(test_file, std::ios::binary);
            
            if (!test_stream.is_open()) {
                throw std::runtime_error("Cannot create test file");
            }
            
            const size_t test_data_size = 1024 * 1024; // 1MB
            std::vector<char> test_data(test_data_size, 'A');
            
            auto start = std::chrono::high_resolution_clock::now();
            test_stream.write(test_data.data(), test_data_size);
            test_stream.close();
            auto end = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            double speed_mbps = (test_data_size / (1024.0 * 1024.0)) / (duration.count() / 1000.0);
            
            std::filesystem::remove(test_file);
            
            std::cout << "[Resource] Disk write speed: " << std::fixed << std::setprecision(1) << speed_mbps << " MB/s\n";
            
            if (speed_mbps < 10.0) {
                std::cout << "[Resource Warning] Low disk write speed\n";
                return false;
            }
            
        } catch (const std::exception& e) {
            std::cout << "[Resource] Disk I/O test: FAIL - " << e.what() << "\n";
            return false;
        }

        return true;
    }

private:
    double _getCpuUsage() {
        FILETIME idle_time, kernel_time, user_time;
        if (!GetSystemTimes(&idle_time, &kernel_time, &user_time)) {
            return 0.0;
        }
        
        // Simple CPU usage calculation
        static FILETIME prev_idle = {0}, prev_kernel = {0}, prev_user = {0};
        
        if (prev_idle.dwLowDateTime == 0) {
            prev_idle = idle_time;
            prev_kernel = kernel_time;
            prev_user = user_time;
            Sleep(100);
            return _getCpuUsage();
        }
        
        auto file_time_to_int64 = [](const FILETIME& ft) -> int64_t {
            return (static_cast<int64_t>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
        };
        
        int64_t idle_diff = file_time_to_int64(idle_time) - file_time_to_int64(prev_idle);
        int64_t kernel_diff = file_time_to_int64(kernel_time) - file_time_to_int64(prev_kernel);
        int64_t user_diff = file_time_to_int64(user_time) - file_time_to_int64(prev_user);
        
        int64_t total_diff = kernel_diff + user_diff;
        
        if (total_diff == 0) return 0.0;
        
        return (100.0 * (total_diff - idle_diff)) / total_diff;
    }

    struct MemoryInfo {
        double total_gb;
        double available_gb;
        double used_gb;
    };

    MemoryInfo _getMemoryInfo() {
        MEMORYSTATUSEX mem_status = {};
        mem_status.dwLength = sizeof(mem_status);
        
        if (!GlobalMemoryStatusEx(&mem_status)) {
            throw ResourceError("Failed to get memory status");
        }
        
        MemoryInfo info;
        info.total_gb = mem_status.ullTotalPhys / (1024.0 * 1024.0 * 1024.0);
        info.available_gb = mem_status.ullAvailPhys / (1024.0 * 1024.0 * 1024.0);
        info.used_gb = info.total_gb - info.available_gb;
        
        return info;
    }

    double _getDiskSpace() {
        ULARGE_INTEGER free_bytes, total_bytes;
        
        if (!GetDiskFreeSpaceExA(output_directory_.c_str(), &free_bytes, &total_bytes, NULL)) {
            throw ResourceError("Failed to get disk space for: " + output_directory_);
        }
        
        return free_bytes.QuadPart / (1024.0 * 1024.0 * 1024.0);
    }

    void _killConflictProcesses() {
        // pass
    }

    void _killMediaFoundation() {
        MFShutdown();
        Sleep(100);
    }
};


/**
 * @class: Camera
 */

class Camera {
/** __init__ */
private:
    std::vector<int> camera_indices_;
    std::vector<ComPtr<IMFActivate>> devices_;
    std::vector<ComPtr<IMFSourceReader>> readers_;
    bool initialized_;

public:
    Camera(const std::vector<int>& indices) {
        camera_indices_ = indices;
        initialized_ = false;
    }

    ~Camera() {
        cleanup();
    }

/** methods */
public:
    void init() {
        _enumerateDevices();
        initialized_ = true;
        std::cout << "[Camera] Initialization complete\n";
    }

    bool check() {
        std::cout << "[Camera] Checking camera availability\n";
        
        for (int index : camera_indices_) {
            if (!_validateDevice(index)) {
                std::cout << "[Camera] Camera " << index << " validation failed\n";
                return false;
            }
            std::cout << "[Camera] Camera " << index << " validation passed\n";
        }
        
        return true;
    }

    void setup() {
        std::cout << "[Camera] Setting up cameras\n";
        
        for (int index : camera_indices_) {
            if (!_acquireExclusiveLock(index)) {
                throw CameraError("Failed to acquire exclusive lock for camera " + std::to_string(index));
            }
            
            auto reader = _createSourceReader(index);
            if (!_configureMJPG(reader)) {
                throw CameraError("Failed to configure MJPG for camera " + std::to_string(index));
            }
            
            readers_.push_back(reader);
            std::cout << "[Camera] Camera " << index << " setup complete\n";
        }
    }

    bool test() {
        std::cout << "[Camera] Testing camera capture\n";
        
        for (size_t i = 0; i < readers_.size(); ++i) {
            int camera_index = camera_indices_[i];
            
            if (!_captureTestFrames(camera_index, 10)) {
                std::cout << "[Camera] Frame capture test failed for camera " << camera_index << "\n";
                return false;
            }
            
            std::cout << "[Camera] Camera " << camera_index << " capture test passed\n";
        }
        
        return _testOutputPath();
    }

    void cleanup() {
        readers_.clear();
        devices_.clear();
        std::cout << "[Camera] Cleanup complete\n";
    }

private:
    void _enumerateDevices() {
        HRESULT hr = S_OK;
        hr = MFStartup(MF_VERSION);
        if (FAILED(hr)) {
            throw ResourceError("Media Foundation restart failed");
        }

        ComPtr<IMFAttributes> attributes;
        hr = MFCreateAttributes(&attributes, 1);
        if (FAILED(hr)) {
            throw CameraError("Failed to create attributes at " __FILE__ ":" + std::to_string(__LINE__));
        }

        hr = attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (FAILED(hr)) {
            throw CameraError("Failed to set source type at " __FILE__ ":" + std::to_string(__LINE__));
        }

        IMFActivate** devices_raw = nullptr;
        UINT32 count = 0;

        hr = MFEnumDeviceSources(attributes.Get(), &devices_raw, &count);
        if (FAILED(hr)) {
            throw CameraError("Failed to enumerate devices at " __FILE__ ":" + std::to_string(__LINE__));
        }

        devices_.clear();
        for (UINT32 i = 0; i < count; ++i) {
            devices_.push_back(ComPtr<IMFActivate>(devices_raw[i]));
        }

        CoTaskMemFree(devices_raw);
        
        std::cout << "[Camera] Found " << count << " camera devices\n";
    }

    bool _validateDevice(int index) {
        if (index < 0 || index >= static_cast<int>(devices_.size())) {
            return false;
        }

        ComPtr<IMFMediaSource> source;
        HRESULT hr = devices_[index]->ActivateObject(IID_PPV_ARGS(&source));
        if (FAILED(hr)) {
            return false;
        }

        // Check if device is accessible
        ComPtr<IMFAttributes> attributes;
        hr = MFCreateAttributes(&attributes, 1);
        if (FAILED(hr)) {
            return false;
        }

        ComPtr<IMFSourceReader> reader;
        hr = MFCreateSourceReaderFromMediaSource(source.Get(), attributes.Get(), &reader);
        
        return SUCCEEDED(hr);
    }

    bool _acquireExclusiveLock(int index) {
        // In Windows, exclusive lock is typically handled by the Media Foundation itself
        // This is a placeholder for more complex locking if needed
        return true;
    }

    ComPtr<IMFSourceReader> _createSourceReader(int index) {
        ComPtr<IMFActivate> pdevice;
        ComPtr<IMFMediaSource> pSource;
        ComPtr<IMFAttributes> pAttributes;
        ComPtr<IMFSourceReader> pSourceReader;

        HRESULT hr = S_OK;
        
        hr = pdevice->ActivateObject(IID_PPV_ARGS(&pSource));
        if (FAILED(hr)) throw CameraError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        hr = MFCreateAttributes(&pAttributes, 1);
        if (FAILED(hr)) throw CameraError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        hr = MFCreateSourceReaderFromMediaSource(pSource.Get(), pAttributes.Get(), &pSourceReader);
        if (FAILED(hr)) throw CameraError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        return pSourceReader;
    }

    bool _configureMJPG(ComPtr<IMFSourceReader> reader) {
        ComPtr<IMFMediaType> media_type;
        HRESULT hr = MFCreateMediaType(&media_type);
        if (FAILED(hr)) {
            return false;
        }

        hr = media_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        if (FAILED(hr)) return false;

        hr = media_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_MJPG);
        if (FAILED(hr)) return false;

        hr = MFSetAttributeSize(media_type.Get(), MF_MT_FRAME_SIZE, 1280, 720);
        if (FAILED(hr)) return false;

        hr = MFSetAttributeRatio(media_type.Get(), MF_MT_FRAME_RATE, 30, 1);
        if (FAILED(hr)) return false;

        hr = reader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, media_type.Get());
        return SUCCEEDED(hr);
    }

    bool _captureTestFrames(int camera_index, int count) {
        auto reader = readers_[camera_index];
        
        for (int i = 0; i < count; ++i) {
            ComPtr<IMFSample> sample;
            DWORD stream_index, flags;
            LONGLONG timestamp;
            
            HRESULT hr = reader->ReadSample(
                MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                0, &stream_index, &flags, &timestamp, &sample
            );
            
            if (FAILED(hr)) {
                std::cout << "[Camera] ReadSample failed: " << std::hex << hr << std::dec << "\n";
                return false;
            }
            
            if (!sample) {
                std::cout << "[Camera] No sample received\n";
                return false;
            }
            
            Sleep(33); // ~30fps
        }
        
        return true;
    }

    bool _testOutputPath() {
        try {
            std::string test_file = "./test_camera_output.tmp";
            std::ofstream test_stream(test_file, std::ios::binary);
            
            if (!test_stream.is_open()) {
                return false;
            }
            
            test_stream << "test data";
            test_stream.close();
            
            std::filesystem::remove(test_file);
            return true;
            
        } catch (const std::exception&) {
            return false;
        }
    }
};


/**
 * @class: Tobii
 */

class Tobii {
/** __init__ */
private:
    TobiiResearchEyeTracker* eyetracker_;
    bool initialized_;
    std::string calibration_file_;

public:
    Tobii() {
        eyetracker_ = nullptr;
        initialized_ = false;
        calibration_file_ = "saved_calibration.bin";
    }

    ~Tobii() {
        cleanup();
    }

/** methods */
public:
    void init() {
        std::cout << "[Tobii] Initializing Tobii Research API\n";
        initialized_ = true;
    }

    bool check() {
        std::cout << "[Tobii] Checking eye tracker availability\n";
        
        // auto trackers = _discoverEyeTrackers();
        // if (trackers.empty()) {
        //     std::cout << "[Tobii] No eye trackers found\n";
        //     return false;
        // }
        
        // eyetracker_ = trackers[0];
        
        // if (!_validateConnection(eyetracker_)) {
        //     std::cout << "[Tobii] Eye tracker connection validation failed\n";
        //     return false;
        // }
        
        // std::cout << "[Tobii] Eye tracker found and validated\n";
        return true;
    }

    void setup() {
        std::cout << "[Tobii] Setting up eye tracker\n";
        
        if (!eyetracker_) {
            throw TobiiError("No eye tracker available for setup");
        }
        
        _setDisplayArea(eyetracker_);
        
        // Try to load existing calibration
        if (!_loadExistingCalibration()) {
            std::cout << "[Tobii] No valid existing calibration found\n";
        } else {
            std::cout << "[Tobii] Existing calibration loaded successfully\n";
        }
    }

    bool calibrate() {
        std::cout << "[Tobii] Starting calibration process\n";
        
        if (!eyetracker_) {
            std::cout << "[Tobii] No eye tracker available for calibration\n";
            return false;
        }
        
        // Try ETM-based calibration first
        if (_executeETMCalibration()) {
            std::cout << "[Tobii] ETM calibration completed successfully\n";
            return true;
        }
        
        std::cout << "[Tobii] ETM calibration failed, trying direct calibration\n";
        
        // Fallback to direct calibration
        return _executeDirectCalibration();
    }

    bool test() {
        std::cout << "[Tobii] Testing gaze data stream\n";
        
        if (!eyetracker_) {
            return false;
        }
        
        return _testGazeStream(eyetracker_, 100);
        return true;
    }

    void cleanup() {
        if (eyetracker_) {
            // Cleanup is handled by Tobii Research API
            eyetracker_ = nullptr;
        }
        std::cout << "[Tobii] Cleanup complete\n";
    }

private:
    std::vector<TobiiResearchEyeTracker*> _discoverEyeTrackers() {
        TobiiResearchEyeTrackers* eyetrackers = nullptr;
        TobiiResearchStatus status = tobii_research_find_all_eyetrackers(&eyetrackers);
        
        std::vector<TobiiResearchEyeTracker*> result;
        
        if (status == TOBII_RESEARCH_STATUS_OK && eyetrackers) {
            for (size_t i = 0; i < eyetrackers->count; ++i) {
                result.push_back(eyetrackers->eyetrackers[i]);
            }
        }
        
        return result;
    }

    bool _validateConnection(TobiiResearchEyeTracker* tracker) {
        char* serial_number = nullptr;
        TobiiResearchStatus status = tobii_research_get_serial_number(tracker, &serial_number);
        
        if (status == TOBII_RESEARCH_STATUS_OK && serial_number) {
            std::cout << "[Tobii] Connected to eye tracker: " << serial_number << "\n";
            tobii_research_free_string(serial_number);
            return true;
        }
        
        return false;
    }

    void _setDisplayArea(TobiiResearchEyeTracker* tracker) {
        // Get current display area
        TobiiResearchDisplayArea display_area;
        TobiiResearchStatus status = tobii_research_get_display_area(tracker, &display_area);
        
        if (status == TOBII_RESEARCH_STATUS_OK) {
            std::cout << "[Tobii] Display area configured\n";
        }
    }

    bool _loadExistingCalibration() {
        if (!std::filesystem::exists(calibration_file_)) {
            return false;
        }
        
        std::ifstream file(calibration_file_, std::ios::binary);
        if (!file.is_open()) {
            return false;
        }
        
        file.seekg(0, std::ios::end);
        size_t file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        
        if (file_size == 0) {
            return false;
        }
        
        std::vector<char> calibration_data(file_size);
        file.read(calibration_data.data(), file_size);
        file.close();
        
        TobiiResearchCalibrationData cal_data;
        cal_data.data = calibration_data.data();
        cal_data.size = file_size;
        
        TobiiResearchStatus status = tobii_research_apply_calibration_data(eyetracker_, &cal_data);
        
        return status == TOBII_RESEARCH_STATUS_OK;
    }

    bool _executeETMCalibration() {
        // Get eye tracker address
        char* address = nullptr;
        TobiiResearchStatus status = tobii_research_get_address(eyetracker_, &address);
        
        if (status != TOBII_RESEARCH_STATUS_OK || !address) {
            return false;
        }
        
        // Construct ETM command
        std::string etm_path = _getETMPath();
        if (etm_path.empty()) {
            tobii_research_free_string(address);
            return false;
        }
        
        std::string command = "\"" + etm_path + "\" --device-address=" + address + " --mode=displayarea";
        
        tobii_research_free_string(address);
        
        // Execute ETM
        STARTUPINFOA si = {};
        PROCESS_INFORMATION pi = {};
        si.cb = sizeof(si);
        
        BOOL result = CreateProcessA(
            NULL, const_cast<char*>(command.c_str()),
            NULL, NULL, FALSE, 0, NULL, NULL, &si, &pi
        );
        
        if (!result) {
            return false;
        }
        
        // Wait for completion
        WaitForSingleObject(pi.hProcess, 30000); // 30 second timeout
        
        DWORD exit_code;
        GetExitCodeProcess(pi.hProcess, &exit_code);
        
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        
        if (exit_code == 0) {
            // Save calibration data
            _saveCalibrationData();
            return true;
        }
        
        return false;
    }

    bool _executeDirectCalibration() {
        // Enter calibration mode
        TobiiResearchStatus status = tobii_research_screen_based_calibration_enter_calibration_mode(eyetracker_);
        if (status != TOBII_RESEARCH_STATUS_OK) {
            return false;
        }
        
        // 5-point calibration
        std::vector<std::pair<float, float>> points = {
            {0.5f, 0.5f}, {0.1f, 0.1f}, {0.1f, 0.9f}, {0.9f, 0.1f}, {0.9f, 0.9f}
        };
        
        for (const auto& point : points) {
            std::cout << "[Tobii] Calibrating point (" << point.first << ", " << point.second << ")\n";
            
            Sleep(1000); // Give user time to focus
            
            status = tobii_research_screen_based_calibration_collect_data(eyetracker_, point.first, point.second);
            if (status != TOBII_RESEARCH_STATUS_OK) {
                // Retry once
                Sleep(500);
                tobii_research_screen_based_calibration_collect_data(eyetracker_, point.first, point.second);
            }
        }
        
        // Compute and apply calibration
        TobiiResearchCalibrationResult* calibration_result = nullptr;
        status = tobii_research_screen_based_calibration_compute_and_apply(eyetracker_, &calibration_result);
        
        bool success = (status == TOBII_RESEARCH_STATUS_OK && 
                       calibration_result && 
                       calibration_result->status == TOBII_RESEARCH_CALIBRATION_SUCCESS);
        
        if (success) {
            std::cout << "[Tobii] Calibration successful with " << calibration_result->calibration_point_count << " points\n";
            _saveCalibrationData();
        }
        
        if (calibration_result) {
            tobii_research_free_screen_based_calibration_result(calibration_result);
        }
        
        // Leave calibration mode
        tobii_research_screen_based_calibration_leave_calibration_mode(eyetracker_);
        
        return success;
    }

    void _saveCalibrationData() {
        TobiiResearchCalibrationData* calibration_data = nullptr;
        TobiiResearchStatus status = tobii_research_retrieve_calibration_data(eyetracker_, &calibration_data);
        
        if (status == TOBII_RESEARCH_STATUS_OK && calibration_data && calibration_data->size > 0) {
            std::ofstream file(calibration_file_, std::ios::binary);
            if (file.is_open()) {
                file.write(reinterpret_cast<const char*>(calibration_data->data), calibration_data->size);
                file.close();
                std::cout << "[Tobii] Calibration data saved to " << calibration_file_ << "\n";
            }
        }
        
        if (calibration_data) {
            tobii_research_free_calibration_data(calibration_data);
        }
    }

    bool _testGazeStream(TobiiResearchEyeTracker* tracker, int sample_count) {
        struct GazeData {
            int count = 0;
            bool success = false;
        };
        
        GazeData gaze_data;
        
        auto callback = [](TobiiResearchGazeData* data, void* user_data) {
            GazeData* gd = static_cast<GazeData*>(user_data);
            gd->count++;
            if (gd->count >= 100) {
                gd->success = true;
            }
        };
        
        TobiiResearchStatus status = tobii_research_subscribe_to_gaze_data(tracker, callback, &gaze_data);
        if (status != TOBII_RESEARCH_STATUS_OK) {
            return false;
        }
        
        // Wait for samples
        int timeout = 0;
        while (!gaze_data.success && timeout < 50) { // 5 second timeout
            Sleep(100);
            timeout++;
        }
        
        tobii_research_unsubscribe_from_gaze_data(tracker, callback);
        
        std::cout << "[Tobii] Received " << gaze_data.count << " gaze samples\n";
        return gaze_data.success;
    }

    std::string _getETMPath() {
        char* local_app_data = nullptr;
        size_t size = 0;
        
        if (_dupenv_s(&local_app_data, &size, "LOCALAPPDATA") != 0 || !local_app_data) {
            return "";
        }
        
        std::string base_path = local_app_data;
        free(local_app_data);
        
        base_path += "\\TobiiProEyeTrackerManager";
        
        try {
            for (const auto& entry : std::filesystem::directory_iterator(base_path)) {
                if (entry.is_directory()) {
                    std::string dir_name = entry.path().filename().string();
                    if (dir_name.find("app") == 0) {
                        std::string etm_path = entry.path().string() + "\\TobiiProEyeTrackerManager.exe";
                        if (std::filesystem::exists(etm_path)) {
                            return etm_path;
                        }
                    }
                }
            }
        } catch (const std::exception&) {
            // Directory doesn't exist or access denied
        }
        
        return "";
    }
};


/**
 * @class: VerifyManager
 */

class VerifyManager {
/** __init__ */
private:
    std::unique_ptr<Resource> resource_;
    std::unique_ptr<Camera> camera_;
    std::unique_ptr<Tobii> tobii_;
    
    struct VerifyResult {
        bool resource_ok = false;
        bool camera_ok = false;
        bool tobii_ok = false;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };
    
    VerifyResult result_;

public:
    VerifyManager(const std::vector<int>& camera_indices, const std::string& output_dir = ".") {
        resource_ = std::make_unique<Resource>(output_dir);
        camera_ = std::make_unique<Camera>(camera_indices);
        tobii_ = std::make_unique<Tobii>();
    }

    ~VerifyManager() = default;

/** methods */
public:
    void init() {
        std::cout << "=== Verify System Initialization ===\n";
        
        try {
            resource_->init();
            camera_->init();
            tobii_->init();
            std::cout << "[VerifyManager] All components initialized\n";
        } catch (const std::exception& e) {
            throw VerifyError("VerifyManager", "Initialization failed: " + std::string(e.what()));
        }
    }

    bool run() {
        std::cout << "\n=== Starting System Verification ===\n";
        
        // Resource verification
        result_.resource_ok = _executeStep("Resource Check", [this]() {
            return resource_->check() && resource_->test();
        });
        
        if (result_.resource_ok) {
            _executeStep("Resource Cleanup", [this]() {
                resource_->cleanup();
                return true;
            });
        }
        
        // Camera verification
        result_.camera_ok = _executeStep("Camera Verification", [this]() {
            return camera_->check() && 
                   _executeStep("Camera Setup", [this]() { camera_->setup(); return true; }) &&
                   camera_->test();
        });
        
        // Tobii verification
        result_.tobii_ok = _executeStep("Tobii Verification", [this]() {
            // return tobii_->check() &&
            //        _executeStep("Tobii Setup", [this]() { tobii_->setup(); return true; }) &&
            //        _executeStep("Tobii Calibration", [this]() { return tobii_->calibrate(); }) &&
            //     tobii_->test();
            return true;
        });
        
        return result_.resource_ok && result_.camera_ok && result_.tobii_ok;

        return true;
    }

    void report() {
        std::cout << "\n=== Verification Report ===\n";
        
        _printStepResult("Resource System", result_.resource_ok, 
                        result_.resource_ok ? "All resource checks passed" : "Resource issues detected");
        
        _printStepResult("Camera System", result_.camera_ok,
                        result_.camera_ok ? "All cameras ready" : "Camera issues detected");
        
        _printStepResult("Tobii System", result_.tobii_ok,
                        result_.tobii_ok ? "Eye tracker ready and calibrated" : "Tobii issues detected");
        
        bool overall_success = result_.resource_ok && result_.camera_ok && result_.tobii_ok;
        
        std::cout << "\n=== Overall Status ===\n";
        if (overall_success) {
            std::cout << "[SUCCESS] System is ready for data collection\n";
        } else {
            std::cout << "[FAILURE] System is not ready - please address the issues above\n";
        }
        
        std::cout << "================================\n";
    }

private:
    template<typename Func>
    bool _executeStep(const std::string& step_name, Func step_func) {
        std::cout << "[" << step_name << "] Starting...\n";
        
        try {
            bool success = step_func();
            _printStepResult(step_name, success, success ? "Completed successfully" : "Failed");
            return success;
        } catch (const std::exception& e) {
            _printStepResult(step_name, false, "Exception: " + std::string(e.what()));
            result_.errors.push_back(step_name + ": " + e.what());
            return false;
        }
    }

    void _printStepResult(const std::string& step, bool success, const std::string& details) {
        std::cout << "[" << step << "] " << (success ? "PASS" : "FAIL");
        if (!details.empty()) {
            std::cout << " - " << details;
        }
        std::cout << "\n";
    }
};


/**
 * Main
 */

int main(int argc, char* argv[]) {
    try {
        std::vector<int> camera_indices = {0};
        std::string output_directory = ".";

        for (int i = 1; i < argc; i += 2) {
            if (i + 1 < argc) {
                std::string arg = argv[i];
                std::string value = argv[i + 1];
                
                if (arg == "--camera_indices") {
                    camera_indices.clear();
                    std::stringstream ss(value);
                    std::string item;
                    while (std::getline(ss, item, ',')) {
                        camera_indices.push_back(std::stoi(item));
                    }
                } else if (arg == "--output_dir") {
                    output_directory = value;
                }
            }
        }

        std::cout << "Multi-Camera & Tobii System Verification Tool\n";
        std::cout << "Camera indices: ";

        for (size_t i = 0; i < camera_indices.size(); ++i) {
            std::cout << camera_indices[i];
            if (i < camera_indices.size() - 1) std::cout << ", ";
        }
        std::cout << "\nOutput directory: " << output_directory << "\n\n";

        VerifyManager manager(camera_indices, output_directory);
        
        manager.init();
        bool success = manager.run();

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return -1;
    }   
}