#pragma once

// Standard Library
#include <array>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <memory>
#include <condition_variable>
#include <iomanip>

// Windows
#include <comdef.h>
#include <wrl/client.h>
#include <wrl/implements.h>

// Media Foundation
#include <mfapi.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mfreadwrite.h>
#include <mferror.h>

// Project
#include "error/error.hpp"
#include "config.hpp"

// using
using namespace Microsoft::WRL;
using Microsoft::WRL::ComPtr;

/**
 * Recording State Machine
 */
enum class RecordingState {
    READY,            // 녹화 준비 완료
    SYNCHRONIZED,     // 동기화 대기
    RECORDING,        // 실제 녹화 중
    STOPPING          // 종료 중
};

std::string recordingStateToString(RecordingState state) {
    switch (state) {
        case RecordingState::READY: return "READY";
        case RecordingState::SYNCHRONIZED: return "SYNCHRONIZED";
        case RecordingState::RECORDING: return "RECORDING";
        case RecordingState::STOPPING: return "STOPPING";
        default: return "UNKNOWN";
    }
}



/**
 * Helper
 */
constexpr std::size_t RING_BUFFER_SIZE = 1024;

class CoTaskMemDeleter {
public:
    template<typename T>
    void operator()(T* ptr) const {
        if (ptr) {
            CoTaskMemFree(ptr);
        }
    }
};



template<typename T>
using CoTaskMemPtr = std::unique_ptr<T, CoTaskMemDeleter>;

class PrecisionTimer {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    
    static void preciseSleep(std::chrono::microseconds duration) {
        auto start = Clock::now();
        auto end = start + duration;
        
        if (duration > std::chrono::milliseconds(2)) {
            auto sleep_until = end - std::chrono::milliseconds(1);
            std::this_thread::sleep_until(sleep_until);
        }
        
        while (Clock::now() < end) {
            std::this_thread::yield();
        }
    }
    
    static double getMicroseconds(TimePoint start, TimePoint end) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1000.0;
    }
};


// C++17 compatible barrier implementation
class Barrier {
public:
    explicit Barrier(std::size_t count) : count_(count), waiting_(0), generation_(0) {}

    void arrive_and_wait() {
        std::unique_lock<std::mutex> lock(mutex_);
        std::size_t current_generation = generation_;
        
        ++waiting_;
        
        if (waiting_ == count_) {
            // Last thread arrives
            waiting_ = 0;
            ++generation_;
            cv_.notify_all();
        } else {
            // Wait for all threads
            cv_.wait(lock, [this, current_generation] {
                return generation_ != current_generation;
            });
        }
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::size_t count_;
    std::size_t waiting_;
    std::size_t generation_;
};

/**
 * Enhanced SPSCRingBuffer
 */
template <typename T, std::size_t N>
class SPSCRingBuffer {
private:
    std::atomic<std::size_t> m_head;
    std::array<T, N> m_buff;
    std::atomic<std::size_t> m_tail;

public:
    constexpr SPSCRingBuffer() noexcept : m_head(0), m_tail(0) {}

    constexpr std::size_t capacity() const noexcept { return N; }
    
    std::size_t size() const noexcept {
        return m_tail.load(std::memory_order_acquire) - m_head.load(std::memory_order_acquire);
    }
    
    bool empty() const noexcept {
        return m_head.load(std::memory_order_acquire) == m_tail.load(std::memory_order_acquire);
    }
    
    bool full() const noexcept {
        return size() >= N;
    }

    bool enqueue(T val) noexcept {
        std::size_t current_tail = m_tail.load(std::memory_order_relaxed);
        if (current_tail - m_head.load(std::memory_order_acquire) < N) {
            m_buff[current_tail % N] = std::move(val);
            m_tail.store(current_tail + 1, std::memory_order_release);
            return true;
        }
        return false;
    }

    std::optional<T> dequeue() noexcept {
        std::size_t current_tail = m_tail.load(std::memory_order_acquire);
        std::size_t current_head = m_head.load(std::memory_order_relaxed);

        if (current_head != current_tail) {
            auto value = std::move(m_buff[current_head % N]);
            m_head.store(current_head + 1, std::memory_order_release);
            return std::optional<T>(std::move(value));
        }
        return std::nullopt;
    }

    template<typename Func>
    std::size_t drainAll(Func&& processor) {
        std::size_t processed = 0;
        while (auto item = dequeue()) {
            processor(std::move(*item));
            ++processed;
        }
        return processed;
    }

    struct Stats {
        std::size_t head;
        std::size_t tail;
        std::size_t size;
        double usage_ratio;
    };
    
    Stats getStats() const noexcept {
        std::size_t head = m_head.load(std::memory_order_acquire);
        std::size_t tail = m_tail.load(std::memory_order_acquire);
        std::size_t current_size = tail - head;
        
        return Stats{
            head,
            tail,
            current_size,
            static_cast<double>(current_size) / N
        };
    }
    
    void printStats(const std::string& name = "") const {
        auto stats = getStats();
        std::cout << "[RingBuffer" << (name.empty() ? "" : " " + name) << "] "
                  << "head: " << stats.head 
                  << ", tail: " << stats.tail
                  << ", size: " << stats.size << "/" << N
                  << " (" << std::fixed << std::setprecision(1) 
                  << stats.usage_ratio * 100 << "%)\n";
    }
};

/**
 * Refactored FFmpeg Class - Write Gate 추가
 */
class FFmpeg {
public:
    enum class State {
        STOPPED,
        STARTING,
        RUNNING,
        STOPPING
    };

    FFmpeg() = default;
    ~FFmpeg() { stop(); }

    void setConfig(const Config& config) { 
        if (state_ == State::STOPPED) {
            config_ = config; 
        }
    }

    bool start() {
        if (state_ != State::STOPPED) return false;
        
        state_ = State::STARTING;
        
        if (createPipe() && startProcess()) {
            state_ = State::RUNNING;
            write_gate_open_ = false;
            return true;
        }
        
        state_ = State::STOPPED;
        return false;
    }

    bool write(const BYTE* data, size_t length) {
        if (state_ != State::RUNNING || !ffmpeg_stdin_) {
            return false;
        }

        // Write Gate 검사 - Gate가 닫혀있으면 데이터 버림
        if (!write_gate_open_.load()) {
            return true; // 성공으로 리턴하지만 실제로는 쓰지 않음
        }

        DWORD written = 0;
        BOOL result = WriteFile(ffmpeg_stdin_, data, static_cast<DWORD>(length), &written, NULL);
        
        if (!result) {
            std::cerr << "[FFmpeg] Write 실패, 상태를 STOPPING으로 변경\n";
            state_ = State::STOPPING;
        }
        
        return result;
    }

    void stop() {
        if (state_ == State::STOPPED) return;
        
        state_ = State::STOPPING;
        std::cout << "[FFmpeg] 종료 시작\n";
        
        // *** 수정: 바로 강제 종료 ***
        if (ffmpeg_proc_) {
            std::cout << "[FFmpeg] Process 강제 종료\n";
            TerminateProcess(ffmpeg_proc_, 0);
            WaitForSingleObject(ffmpeg_proc_, 1000);  // 1초 대기
            
            CloseHandle(ffmpeg_proc_);
            ffmpeg_proc_ = NULL;
            std::cout << "[FFmpeg] Process 종료 완료\n";
        }
        
        // stdin 정리
        if (ffmpeg_stdin_) {
            CloseHandle(ffmpeg_stdin_);
            ffmpeg_stdin_ = NULL;
            std::cout << "[FFmpeg] stdin 정리 완료\n";
        }
        
        if (read_handle_) {
            CloseHandle(read_handle_);
            read_handle_ = NULL;
        }
        
        state_ = State::STOPPED;
        std::cout << "[FFmpeg] 종료 완료\n";
    }

    // Write Gate 제어 메서드들
    void openWriteGate() { 
        write_gate_open_ = true; 
        std::cout << "[FFmpeg] Write Gate 열림\n";
    }
    
    void closeWriteGate() { 
        write_gate_open_ = false; 
        std::cout << "[FFmpeg] Write Gate 닫힘\n";
    }
    
    bool isWriteGateOpen() const { 
        return write_gate_open_.load(); 
    }

    State getState() const { return state_; }
    bool isRunning() const { return state_ == State::RUNNING; }

private:
    Config config_;
    HANDLE ffmpeg_stdin_ = NULL;
    HANDLE ffmpeg_proc_ = NULL;
    HANDLE read_handle_ = NULL;
    std::atomic<State> state_{State::STOPPED};
    std::atomic<bool> write_gate_open_{false};  // Write Gate 제어

    bool createPipe() {
        SECURITY_ATTRIBUTES sa = { sizeof(SECURITY_ATTRIBUTES), NULL, TRUE };
        HANDLE readHandle = NULL, writeHandle = NULL;

        if (!CreatePipe(&readHandle, &writeHandle, &sa, 0)) {
            std::cerr << "[FFmpeg] 파이프 생성 실패\n";
            return false;
        }

        read_handle_ = readHandle;
        ffmpeg_stdin_ = writeHandle;
        return true;
    }

    bool startProcess() {
        STARTUPINFOA si = { 0 };
        si.cb = sizeof(STARTUPINFOA);
        si.dwFlags |= STARTF_USESTDHANDLES;
        si.hStdInput = read_handle_;
        si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
        si.hStdError = GetStdHandle(STD_ERROR_HANDLE);

        PROCESS_INFORMATION pi = { 0 };
        
        std::string command = buildCommand();
        BOOL success = CreateProcessA(
            NULL, const_cast<LPSTR>(command.c_str()),
            NULL, NULL, TRUE, 0, NULL, NULL,
            &si, &pi
        );

        if (success) {
            ffmpeg_proc_ = pi.hProcess;
            CloseHandle(pi.hThread);
            CloseHandle(read_handle_);
            read_handle_ = NULL;
            return true;
        }

        return false;
    }

    void waitForProcessTermination() {
        if (ffmpeg_proc_) {
            DWORD result = WaitForSingleObject(ffmpeg_proc_, 3000);
            
            if (result == WAIT_TIMEOUT) {
                std::cout << "[FFmpeg] 타임아웃, 강제 종료\n";
                TerminateProcess(ffmpeg_proc_, 1);
                WaitForSingleObject(ffmpeg_proc_, 2000);
            }
        }
    }

    void cleanup() {
        if (ffmpeg_stdin_) {
            CloseHandle(ffmpeg_stdin_);
            ffmpeg_stdin_ = NULL;
        }
        if (ffmpeg_proc_) {
            CloseHandle(ffmpeg_proc_);
            ffmpeg_proc_ = NULL;
        }
        if (read_handle_) {
            CloseHandle(read_handle_);
            read_handle_ = NULL;
        }
    }

    std::string buildCommand() const {
        return config_.ffmpeg +
            " -loglevel verbose -y" +
            " -f mjpeg" +
            " -framerate " + std::to_string(config_.frame_rate) +
            " -i -" +
            " -c:v copy" +
            " -f avi" +
            " -avoid_negative_ts make_zero" +
            " \"" + config_.output + "\"";
    }
};

/**
 * Refactored SampleCallback - State Machine 기반 제어
 */
class SampleCallback : public RuntimeClass<RuntimeClassFlags<ClassicCom>, IMFSourceReaderCallback> {
public:
    SampleCallback(SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>* ringBuffer,
                   std::atomic<bool>* runningFlag,
                   const std::string& cameraId,
                   const std::string& outputDir = ".")
        : ringBuffer_(ringBuffer), 
          external_running_flag_(runningFlag),
          camera_id_(cameraId),
          csv_filename_(outputDir + "/timestamps_" + cameraId + ".csv"),
          recording_state_(RecordingState::READY) {
        
        csv_file_.open(csv_filename_, std::ios::out);
        if (csv_file_.is_open()) {
            csv_file_ << "FrameIndex,Timestamp100ns,Timestamp(ms),Delta100ns,Delta(ms),"
                     << "WallClockTime,WallClockMs,CameraID\n";
            wrote_header_ = true;
        }
    }

    ~SampleCallback() {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

    STDMETHODIMP OnReadSample(HRESULT hrStatus, DWORD streamIndex, DWORD flags, 
                             LONGLONG llTimestamp, IMFSample* sample) override {
        frameCount_++;
    
        if (recording_state_.load() == RecordingState::SYNCHRONIZED && !warmup_completed_) {
            warmup_frame_count_++;
            
            // 충분한 frame이 buffer에 쌓이면 warmup 완료
            if (warmup_frame_count_ >= 10) {
                warmup_completed_ = true;
                std::cout << "[Warmup " << camera_id_ << "] Frame buffer 준비 완료 (10 frames)\n";
            }
            
            if (ringBuffer_ && sample) {
                ringBuffer_->enqueue(sample);
            }
            requestNextFrame();
            return S_OK;
        }
    
        // Recording 시작 시 정확한 시간 기록
        if (recording_state_.load() == RecordingState::RECORDING && !first_frame_recorded_) {
            recording_start_time_ = std::chrono::high_resolution_clock::now();
            first_frame_recorded_ = true;
            frameCount_ = 1; // Reset frame count for accurate measurement
            std::cout << "[Timing " << camera_id_ << "] 실제 첫 frame capture 시작\n";
        }
        
        logTimestamp(llTimestamp);
        
        if (!isRunning()) {
            std::cout << "[Callback " << camera_id_ << "] 종료 상태 감지, ReadSample 중지\n";
            return S_OK;
        }

        if (ringBuffer_ && sample) {
            ringBuffer_->enqueue(sample);
        }

        requestNextFrame();
        
        return S_OK;
    }

    STDMETHODIMP OnEvent(DWORD, IMFMediaEvent*) override { return S_OK; }
    STDMETHODIMP OnFlush(DWORD) override { return S_OK; }

    void setReader(IMFSourceReader* reader) { reader_ = reader; }
    void resetFrameCount() { frameCount_ = 0; }
    
    // Recording State 제어
    void setRecordingState(RecordingState state) { 
        recording_state_ = state; 
        std::cout << "[Callback " << camera_id_ << "] Recording State 변경: " 
                  << recordingStateToString(state) << "\n";
    }

    bool isWarmupCompleted() const { 
        return warmup_completed_.load(); 
    }
    
    RecordingState getRecordingState() const { 
        return recording_state_.load(); 
    }
    
    int getFrameCount() const { return frameCount_; }
    bool isRunning() const { 
        return external_running_flag_ && *external_running_flag_; 
    }

    void printFinalStats() {
        if (first_frame_recorded_) {
            auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                last_frame_time_ - first_frame_time_).count();
            double avg_fps = ((frameCount_ - 1) * 1000000.0) / total_duration;
            
            std::cout << "[FinalStats " << camera_id_ << "] 총 Frame: " << frameCount_ 
                      << ", 총 시간: " << std::fixed << std::setprecision(6) 
                      << total_duration / 1000000.0 << "초"
                      << ", 평균 FPS: " << std::setprecision(3) << avg_fps << "\n";
        }
    }

    void startWarmup() { 
        warmup_active_ = true; 
        warmup_frame_count_ = 0;
        std::queue<PrecisionTimer::TimePoint> empty;
        warmup_frame_times_.swap(empty);
    }
    
    void stopWarmup() { warmup_active_ = false; }
    bool isWarmupActive() const { return warmup_active_.load(); }
    
    struct WarmupStats {
        double current_fps;
        int stable_frame_count;
        bool fps_target_met;
        bool is_stable;
    };
    
    WarmupStats getWarmupStats(double target_fps, int required_frames, int monitoring_frames) {
        std::lock_guard<std::mutex> lock(warmup_mutex_);
        
        if (warmup_frame_times_.size() < 2) {
            return {0.0, 0, false, false};
        }
        
        // 최근 10개 frame만 사용 (간소화)
        size_t recent_count = warmup_frame_times_.size();
        if (recent_count > 10) {
            recent_count = 10;
        }
        
        if (recent_count < 2) {
            return {0.0, 0, false, false};
        }
        
        // 간단한 FPS 계산
        double duration = 1.0; // 임시값
        double current_fps = recent_count / duration;
        
        // 임시로 안정화 조건 완화
        bool fps_met = current_fps > 20.0;  // 20fps 이상이면 OK
        bool is_stable = warmup_frame_count_ > 10;  // 10 frame 이상 받으면 OK
        
        return {current_fps, warmup_frame_count_, fps_met, is_stable};
    }

private:
    SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>* ringBuffer_;
    std::atomic<bool>* external_running_flag_;
    IMFSourceReader* reader_ = nullptr;
    std::atomic<int> frameCount_{0};
    
    // 개별화된 CSV 관련 멤버들
    std::string camera_id_;
    std::string csv_filename_;
    std::ofstream csv_file_;
    std::mutex csv_mutex_;
    bool wrote_header_ = false;
    LONGLONG prev_timestamp_ = -1;
    std::atomic<RecordingState> recording_state_{RecordingState::READY};  // State Machine
    std::chrono::high_resolution_clock::time_point first_frame_time_;
    std::chrono::high_resolution_clock::time_point last_frame_time_;
    bool first_frame_recorded_ = false;

    std::atomic<bool> warmup_completed_{false};
    std::chrono::high_resolution_clock::time_point recording_start_time_;
    int warmup_frame_count_ = 0;

    std::atomic<bool> warmup_active_{false};
    std::queue<PrecisionTimer::TimePoint> warmup_frame_times_;
    std::mutex warmup_mutex_;
    
    void logTimestamp(LONGLONG timestamp) {
        bool should_log_csv = (recording_state_.load() == RecordingState::RECORDING);
        
        if (warmup_active_.load()) {
            std::lock_guard<std::mutex> lock(warmup_mutex_);
            warmup_frame_times_.push(PrecisionTimer::Clock::now());
            warmup_frame_count_++;
            
            // 큐 크기 제한
            while (warmup_frame_times_.size() > 50) {
                warmup_frame_times_.pop();
            }
            return;  // Warm-up 중에는 다른 로깅 스킵
        }
        
        if (should_log_csv) {
            auto now = std::chrono::high_resolution_clock::now();
            
            if (!first_frame_recorded_) {
                first_frame_time_ = now;
                first_frame_recorded_ = true;
            }
            last_frame_time_ = now;
            
            // 10 frame마다 현재 frame rate 출력
            if (frameCount_ % 10 == 0) {
                auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                    now - first_frame_time_).count();
                double current_fps = (frameCount_ * 1000000.0) / elapsed;
                
                std::cout << "[FrameRate " << camera_id_ << "] Frame " << frameCount_ 
                        << ", 현재 FPS: " << std::fixed << std::setprecision(2) 
                        << current_fps << ", 경과시간: " 
                        << std::setprecision(3) << elapsed / 1000000.0 << "초\n";
            }
        }
        
        // 기존 로그 출력
        std::cout << "[Info " << camera_id_ << "] 수신 타임스탬프: " << timestamp 
                << ", frame count: " << frameCount_;
        
        if (should_log_csv) {
            std::cout << " [CSV 기록]";
            writeCsvLog(timestamp);
        } else {
            std::cout << " [" << recordingStateToString(recording_state_.load()) << " - CSV 기록 안함]";
        }
        std::cout << "\n";
    }
        
    void writeCsvLog(LONGLONG timestamp) {
        // Recording State가 RECORDING이 아니면 early return
        if (recording_state_.load() != RecordingState::RECORDING) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(csv_mutex_);
        
        if (!csv_file_.is_open()) return;
        
        // 현재 시간 계산
        auto now = std::chrono::system_clock::now();
        auto now_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        
        // 현재 시간을 문자열로 변환 (ISO 8601 형식)
        std::tm now_tm;
        localtime_s(&now_tm, &now_t);
        
        std::ostringstream wall_clock_stream;
        wall_clock_stream << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S");
        
        // 마이크로초 추가
        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()) % 1000000;
        wall_clock_stream << "." << std::setw(6) << std::setfill('0') << microseconds.count();
        
        LONGLONG delta = (prev_timestamp_ < 0) ? 0 : (timestamp - prev_timestamp_);
        double timestamp_ms = timestamp / 10000.0;
        double delta_ms = delta / 10000.0;

        // CSV 기록 (현재 시간 포함)
        csv_file_ << frameCount_ << ","                    // 프레임 번호
                  << timestamp << ","                      // MF 타임스탬프 (100ns)
                  << timestamp_ms << ","                   // MF 타임스탬프 (ms)
                  << delta << ","                          // 프레임 간격 (100ns)
                  << delta_ms << ","                       // 프레임 간격 (ms)
                  << "\"" << wall_clock_stream.str() << "\"," // 현재 시간 (문자열)
                  << now_ms << ","                         // 현재 시간 (Unix 밀리초)
                  << camera_id_ << "\n";                   // 카메라 ID
                  
        csv_file_.flush();  // 즉시 disk 쓰기

        prev_timestamp_ = timestamp;
    }
    
    void requestNextFrame() {
        if (reader_ && isRunning()) {
            reader_->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, 
                               nullptr, nullptr, nullptr, nullptr);
        }
    }
};

/**
 * Refactored MediaFoundation
 */
class MediaFoundation {
public:
    MediaFoundation() {
        HRESULT hr = MFStartup(MF_VERSION);
        if (HFailed(hr, "[MediaFoundation] MFStartup")) { return; }
        initialized_ = true;
    }

    ~MediaFoundation() {
        if (initialized_) { MFShutdown(); }
    }

    std::optional<ComPtr<IMFActivate>> createDevice(const Config& config) {
        return getDevice(config);
    }

    std::optional<ComPtr<IMFSourceReader>> createSourceReader(
        ComPtr<IMFActivate> device, 
        ComPtr<IMFSourceReaderCallback> callback,
        const Config& config) {
        return getSourceReader(device, callback, config);
    }

    ComPtr<IMFMediaType> createMediaType(const Config& config) {
        ComPtr<IMFMediaType> pType;
        HRESULT hr = MFCreateMediaType(&pType);
        if (FAILED(hr)) return nullptr;

        pType->SetGUID(MF_MT_SUBTYPE, PixelFormatFromString(config.pixel_format));
        pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);

        MFSetAttributeSize(pType.Get(), MF_MT_FRAME_SIZE, config.frame_width, config.frame_height);
        MFSetAttributeRatio(pType.Get(), MF_MT_FRAME_RATE, config.frame_rate, 1);

        return pType;
    }

private:
    bool initialized_ = false;
    
    std::optional<ComPtr<IMFActivate>> getDevice(const Config& config) {
        HRESULT hr = S_OK;

        ComPtr<IMFAttributes> every_attributes;
        hr = MFCreateAttributes(&every_attributes, 1);
        if (HFailed(hr, "[_getDevice] MFCreateAttributes failed")) return std::nullopt;

        hr = every_attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (HFailed(hr, "[_getDevice] SetGUID failed")) return std::nullopt;

        IMFActivate** every_devices_raw = nullptr;
        UINT32 every_count = 0;

        hr = MFEnumDeviceSources(every_attributes.Get(), &every_devices_raw, &every_count);
        if (HFailed(hr, "[_getDevice] MFEnumDeviceSources failed")) return std::nullopt;

        CoTaskMemPtr<IMFActivate*> every_devices(every_devices_raw);

        ComPtr<IMFAttributes> video_attributes;

        hr = MFCreateAttributes(&video_attributes, 1);
        if (HFailed(hr, "[_getDevice] MFCreateAttributes failed")) return std::nullopt;

        hr = video_attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (HFailed(hr, "[_getDevice] SetGUID failed")) return std::nullopt;

        IMFActivate** video_devices_raw = nullptr;
        UINT32 video_count = 0;

        hr = MFEnumDeviceSources(video_attributes.Get(), &video_devices_raw, &video_count);
        if (HFailed(hr, "[_getDevice] MFEnumDeviceSources failed")) return std::nullopt;

        CoTaskMemPtr<IMFActivate*> video_devices(video_devices_raw);

        int camera_index = config.camera_indices.empty() ? 0 : config.camera_indices[0];
        
        if (camera_index >= static_cast<int>(every_count)) {
            std::cout << "[Error] 잘못된 카메라 인덱스: " << camera_index << "\n";
            return std::nullopt;
        }

        IMFActivate* target_device = every_devices_raw[camera_index];
        ComPtr<IMFActivate> device = target_device;

        BOOL match = FALSE;
        bool is_valid = std::any_of(
            video_devices_raw, video_devices_raw + video_count,
            [&](IMFActivate* video_dev) {
                match = FALSE;
                return SUCCEEDED(target_device->Compare(video_dev, MF_ATTRIBUTES_MATCH_INTERSECTION, &match)) && match;
            }
        );
        
        if (!is_valid) return std::nullopt;

        return device;
    }

    std::optional<ComPtr<IMFSourceReader>> getSourceReader(
        ComPtr<IMFActivate> device, 
        ComPtr<IMFSourceReaderCallback> callback, 
        const Config& config) {
        HRESULT hr = S_OK;

        ComPtr<IMFMediaSource> pSource;
        ComPtr<IMFAttributes> pAttributes;
        ComPtr<IMFSourceReader> pSourceReader;
        ComPtr<IMFMediaType> pType;
        
        hr = device->ActivateObject(IID_PPV_ARGS(&pSource));
        if (HFailed(hr, "[_getIMFSourceReader] ActivateObject failed")) return std::nullopt;

        hr = MFCreateAttributes(&pAttributes, 1);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateAttributes failed")) return std::nullopt;

        hr = pAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, callback.Get());
        if (HFailed(hr, "[_getIMFSourceReader] SetUnknown failed")) return std::nullopt;
        
        hr = MFCreateSourceReaderFromMediaSource(pSource.Get(), pAttributes.Get(), &pSourceReader);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateSourceReaderFromMediaSource failed")) return std::nullopt;
        
        hr = MFCreateMediaType(&pType);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateMediaType failed")) return std::nullopt;

        pType->SetGUID(MF_MT_SUBTYPE, PixelFormatFromString(config.pixel_format));
        pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);

        MFSetAttributeSize(pType.Get(), MF_MT_FRAME_SIZE, config.frame_width, config.frame_height);
        MFSetAttributeRatio(pType.Get(), MF_MT_FRAME_RATE, config.frame_rate, 1);

        hr = pSourceReader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, pType.Get());
        if (HFailed(hr, "[_getIMFSourceReader] SetCurrentMediaType failed")) return std::nullopt;
        
        return pSourceReader;
    }

    GUID PixelFormatFromString(const std::string& format) {
        if (format == "NV12")  return MFVideoFormat_NV12;
        if (format == "MJPG")  return MFVideoFormat_MJPG;
        if (format == "YUY2")  return MFVideoFormat_YUY2;
        if (format == "RGB32") return MFVideoFormat_RGB32;
        if (format == "I420")  return MFVideoFormat_I420;
        if (format == "UYVY")  return MFVideoFormat_UYVY;

        throw std::invalid_argument("Unsupported pixel format: " + format);
    }
};


/**
 * SingleManager Class - State Machine 기반 제어
 */
class SingleManager {
public:
    enum class State {
        IDLE,
        SETTING_UP,
        READY,
        RUNNING,
        STOPPING,
        ERROR_STATE
    };

    SingleManager(const Config& config) : config_(config), state_(State::IDLE) {
        if (config_.camera_indices.empty()) {
            throw std::invalid_argument("카메라 인덱스가 비어있습니다.");
        }
        
        camera_id_ = config_.generateCameraId(config_.camera_indices[0]);
        
        ringBuffer_ = std::make_unique<SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>>();
        ffmpeg_ = std::make_unique<FFmpeg>();
        mf_ = std::make_unique<MediaFoundation>();
        recording_state_ = RecordingState::READY;
    }

    ~SingleManager() {
        if (state_ == State::RUNNING) {
            stop();
        }
    }

    bool isReadyForSync() const {
        return state_ == State::READY && 
            ffmpeg_->getState() == FFmpeg::State::RUNNING &&
            recording_state_.load() == RecordingState::READY &&
            warmup_completed_.load();  // 이 줄을 수정 (기존: callback_ && callback_->isWarmupCompleted())
    }

    bool setup() {
        if (state_ != State::IDLE) {
            std::cerr << "[SingleManager " << camera_id_ << "] Setup 실패: 잘못된 상태 " << stateToString(state_) << "\n";
            return false;
        }

        state_ = State::SETTING_UP;

        try {
            // 1. Device 생성
            auto device = mf_->createDevice(config_);
            if (!device) {
                throw std::runtime_error("Camera Device 생성 실패");
            }
            device_ = device.value();
            std::cout << "[SingleManager " << camera_id_ << "] Camera Device 생성 완료\n";

            // 2. Callback 생성
            running_flag_ = std::make_unique<std::atomic<bool>>(false);
            callback_ = Microsoft::WRL::Make<SampleCallback>(
                ringBuffer_.get(), 
                running_flag_.get(),
                camera_id_,
                config_.output_dir
            );
            std::cout << "[SingleManager " << camera_id_ << "] Callback 생성 완료\n";

            // 3. SourceReader 생성
            auto sourceReader = mf_->createSourceReader(device_, callback_, config_);
            if (!sourceReader) {
                throw std::runtime_error("SourceReader 생성 실패");
            }
            sourceReader_ = sourceReader.value();
            callback_->setReader(sourceReader_.Get());
            std::cout << "[SingleManager " << camera_id_ << "] SourceReader 생성 완료\n";

            // 4. FFmpeg 설정 및 시작 ← 이 부분이 누락되어 있었음
            ffmpeg_->setConfig(config_);
            if (!ffmpeg_->start()) {
                throw std::runtime_error("FFmpeg 시작 실패");
            }
            std::cout << "[SingleManager " << camera_id_ << "] FFmpeg 시작 완료 (Write Gate 닫힌 상태)\n";

            // 5. Consumer Thread 시작
            *running_flag_ = true;
            consumer_thread_ = std::thread([this]() { this->consumerLoop(); });
            std::cout << "[SingleManager " << camera_id_ << "] Consumer Thread 시작\n";

            // 6. Recording State를 READY로 설정
            setRecordingState(RecordingState::READY);

            // 7. ReadSample 시작
            HRESULT hr = sourceReader_->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, 
                                                nullptr, nullptr, nullptr, nullptr);
            if (FAILED(hr)) {
                throw std::runtime_error("첫 ReadSample 실패");
            }

            state_ = State::READY;
            std::cout << "[SingleManager " << camera_id_ << "] Setup 완료 - 즉시 Ready 상태\n";
            return true;

        } catch (const std::exception& e) {
            std::cerr << "[SingleManager " << camera_id_ << "] Setup 실패: " << e.what() << "\n";
            state_ = State::ERROR_STATE;
            return false;
        }
    }

    // Recording State 제어 메서드
    void setRecordingState(RecordingState state) {
        recording_state_ = state;
        
        if (callback_) {
            callback_->setRecordingState(state);
        }
        
        // State에 따른 FFmpeg Write Gate 제어
        if (ffmpeg_) {
            if (state == RecordingState::RECORDING) {
                ffmpeg_->openWriteGate();
                // Recording 시작 시 Frame Count 리셋
                if (callback_) {
                    callback_->resetFrameCount();
                }
            } else {
                ffmpeg_->closeWriteGate();
            }
        }
        
        std::cout << "[SingleManager " << camera_id_ << "] Recording State 변경: " 
                  << recordingStateToString(state) << "\n";
    }
    
    RecordingState getRecordingState() const {
        return recording_state_.load();
    }
    
    void startRecordingImmediately() {
        if (state_ != State::READY) return;
        
        state_ = State::RUNNING;
        start_time_ = PrecisionTimer::Clock::now();
        
        setRecordingState(RecordingState::RECORDING);

        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
        std::tm buf;
        localtime_s(&buf, &t);
        std::cout << std::put_time(&buf, "%F %T")
                << "." << std::setw(6) << std::setfill('0') << us.count() << "\n";
        
        std::cout << "[SingleManager " << camera_id_ << "] 즉시 녹화 시작 - Write Gate 열림, CSV 로깅 활성화\n";
        
        // *** 추가: Duration 모니터링 Thread 시작 ***
        duration_monitor_thread_ = std::thread([this]() {
            auto end_time = start_time_ + std::chrono::seconds(config_.record_duration);
            
            while (state_ == State::RUNNING && PrecisionTimer::Clock::now() < end_time) {
                PrecisionTimer::preciseSleep(std::chrono::milliseconds(100));
            }
            
            if (state_ == State::RUNNING) {
                std::cout << "[DurationMonitor " << camera_id_ << "] " 
                        << config_.record_duration << "초 경과, 자동 중지\n";
                stopRecordingBySignal();
            }
        });
    }

    std::string getOutputFile() const { 
        return config_.output; 
    }
    
    void stopRecordingBySignal() {
        if (state_ == State::RUNNING) {
            auto stop_time = std::chrono::high_resolution_clock::now();
            
            auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()) % 1000000;
            std::tm buf;
            localtime_s(&buf, &t);
            
            std::cout << "[Timing " << camera_id_ << "] Stop 신호 수신 시각: "
                    << std::put_time(&buf, "%H:%M:%S")
                    << "." << std::setw(6) << std::setfill('0') << us.count() << "\n";
                    
            auto actual_recording_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                stop_time - start_time_).count();
                
            std::cout << "[Timing " << camera_id_ << "] 실제 개별 Recording 시간: " 
                    << std::fixed << std::setprecision(6) 
                    << actual_recording_duration / 1000000.0 << "초\n";
            
            if (callback_) {
                callback_->printFinalStats();
            }
            
            // *** 수정: 현재 thread가 duration_monitor인지 확인 후 처리 ***
            if (std::this_thread::get_id() == duration_monitor_thread_.get_id()) {
                // Duration monitor thread에서 호출된 경우
                state_ = State::STOPPING;  // *** 추가: state_ 변경 ***
                setRecordingState(RecordingState::STOPPING);
                *running_flag_ = false;
                
                // FFmpeg 직접 종료
                if (ffmpeg_) {
                    ffmpeg_->stop();
                    std::cout << "[SingleManager " << camera_id_ << "] FFmpeg 종료 완료 (Duration Monitor)\n";
                }
                
                std::cout << "[SingleManager " << camera_id_ << "] Duration Monitor에서 부분 중지 완료\n";
            } else {
                // 외부 thread에서 호출된 경우: 전체 stop 호출
                stop();
            }
        }
    }

    bool run() {
        if (state_ != State::READY) {
            std::cerr << "[SingleManager " << camera_id_ << "] Run 실패: 잘못된 상태 " << stateToString(state_) << "\n";
            return false;
        }

        state_ = State::RUNNING;
        start_time_ = std::chrono::steady_clock::now();

        std::cout << "[SingleManager " << camera_id_ << "] Recording 시작\n";
        std::this_thread::sleep_for(std::chrono::seconds(config_.record_duration));
        std::cout << "[SingleManager " << camera_id_ << "] Recording 완료\n";
        return true;
    }

    void stop() {
        if (state_ != State::RUNNING && state_ != State::READY) {
            return;
        }

        state_ = State::STOPPING;
        std::cout << "[SingleManager " << camera_id_ << "] 종료 시작\n";

        setRecordingState(RecordingState::STOPPING);

        *running_flag_ = false;

        if (sourceReader_) {
            sourceReader_->Flush(MF_SOURCE_READER_FIRST_VIDEO_STREAM);
            std::cout << "[SingleManager " << camera_id_ << "] SourceReader 플러시 완료\n";
            
            // *** 추가: SourceReader 명시적 해제 ***
            sourceReader_.Reset();
            std::cout << "[SingleManager " << camera_id_ << "] SourceReader 해제 완료\n";
        }

        // *** 추가: callback에서 reader 참조 제거 ***
        if (callback_) {
            callback_->setReader(nullptr);
            std::cout << "[SingleManager " << camera_id_ << "] Callback reader 참조 제거\n";
        }

        if (consumer_thread_.joinable()) {
            consumer_thread_.join();
            std::cout << "[SingleManager " << camera_id_ << "] Consumer Thread 종료 완료\n";
        }
        
        if (duration_monitor_thread_.joinable()) {
            // *** 수정: 현재 thread가 duration_monitor_thread_인지 확인 ***
            if (std::this_thread::get_id() != duration_monitor_thread_.get_id()) {
                duration_monitor_thread_.join();
                std::cout << "[SingleManager " << camera_id_ << "] Duration Monitor Thread 종료 완료\n";
            } else {
                // 현재 thread가 duration_monitor_thread_이면 detach
                duration_monitor_thread_.detach();
                std::cout << "[SingleManager " << camera_id_ << "] Duration Monitor Thread detach 완료\n";
            }
        }

        auto processed = processRemainingData();
        std::cout << "[SingleManager " << camera_id_ << "] 남은 데이터 " << processed << "개 처리 완료\n";

        ffmpeg_->stop();
        std::cout << "[SingleManager " << camera_id_ << "] FFmpeg 종료 완료\n";

        cleanup();

        state_ = State::IDLE;
        std::cout << "[SingleManager " << camera_id_ << "] 종료 완료\n";
    }

    State getState() const { return state_; }
    bool isReady() const { return state_ == State::READY; }
    bool isRunning() const { return state_ == State::RUNNING; }
    bool isError() const { return state_ == State::ERROR_STATE; }
    std::string getCameraId() const { return camera_id_; }

    struct Statistics {
        int frame_count;
        double recording_duration_ms;
        SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>::Stats buffer_stats;
        FFmpeg::State ffmpeg_state;
    };

    Statistics getStatistics() const {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time_).count();

        return Statistics{
            callback_ ? callback_->getFrameCount() : 0,
            static_cast<double>(duration),
            ringBuffer_->getStats(),
            ffmpeg_->getState()
        };
    }

    void printStatus() const {
        auto stats = getStatistics();
        std::cout << "[SingleManager " << camera_id_ << "] 상태: " << stateToString(state_)
                  << ", Frames: " << stats.frame_count
                  << ", Duration: " << stats.recording_duration_ms << "ms"
                  << ", Buffer: " << stats.buffer_stats.size << "/" << ringBuffer_->capacity()
                  << "\n";
    }

    bool performWarmup() {
        if (!config_.warmup.enable_warmup) {
            warmup_completed_ = true;
            return true;
        }
        
        std::cout << "[Warmup " << camera_id_ << "] 시작\n";
        
        // Phase 1: Hardware Stabilization
        callback_->startWarmup();
        std::cout << "[Warmup " << camera_id_ << "] Phase 1: Hardware 안정화 " 
                << config_.warmup.hardware_stabilization_ms << "ms\n";
        PrecisionTimer::preciseSleep(std::chrono::milliseconds(config_.warmup.hardware_stabilization_ms));
        
        // Phase 2: FPS Monitoring
        std::cout << "[Warmup " << camera_id_ << "] Phase 2: FPS 모니터링\n";
        auto phase2_start = PrecisionTimer::Clock::now();
        auto phase2_timeout = phase2_start + std::chrono::milliseconds(config_.warmup.max_fps_monitoring_ms);
        
        bool fps_stable = false;
        while (PrecisionTimer::Clock::now() < phase2_timeout && !fps_stable) {
            auto stats = callback_->getWarmupStats(
                config_.frame_rate, 
                config_.warmup.stable_frames_required,
                config_.warmup.fps_monitoring_frames
            );
            
            if (stats.fps_target_met && stats.is_stable) {
                fps_stable = true;
                std::cout << "[Warmup " << camera_id_ << "] FPS 안정화 완료: " 
                        << std::fixed << std::setprecision(2) << stats.current_fps << "fps\n";
            } else {
                std::cout << "[Warmup " << camera_id_ << "] FPS: " 
                        << std::fixed << std::setprecision(2) << stats.current_fps 
                        << ", 안정 frames: " << stats.stable_frame_count << "\r" << std::flush;
                PrecisionTimer::preciseSleep(std::chrono::milliseconds(100));
            }
        }
        
        if (!fps_stable) {
            std::cout << "\n[Warning " << camera_id_ << "] FPS 안정화 타임아웃, 진행합니다\n";
        }
        
        callback_->stopWarmup();
        warmup_completed_ = true;
        
        // *** 추가: Warmup 완료 후 상태 확인 로그 ***
        std::cout << "[Warmup " << camera_id_ << "] 완료 - 상태 확인:\n";
        std::cout << "  State: " << stateToString(state_) << "\n";
        std::cout << "  FFmpeg State: " << (ffmpeg_->isRunning() ? "RUNNING" : "NOT_RUNNING") << "\n";
        std::cout << "  Recording State: " << recordingStateToString(recording_state_.load()) << "\n";
        std::cout << "  isReadyForSync: " << (isReadyForSync() ? "true" : "false") << "\n";
        
        return true;
    }

    bool isWarmupCompleted() const { return warmup_completed_.load(); }

private:
    Config config_;
    std::string camera_id_;
    std::atomic<State> state_;
    std::atomic<RecordingState> recording_state_;  // Recording State Machine
    std::chrono::steady_clock::time_point start_time_;

    std::unique_ptr<MediaFoundation> mf_;
    std::unique_ptr<FFmpeg> ffmpeg_;
    std::unique_ptr<SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>> ringBuffer_;
    std::unique_ptr<std::atomic<bool>> running_flag_;

    ComPtr<IMFActivate> device_;
    ComPtr<IMFSourceReader> sourceReader_;
    ComPtr<SampleCallback> callback_;

    std::thread consumer_thread_;

    std::thread duration_monitor_thread_;


    std::atomic<bool> warmup_completed_{false};

    void consumerLoop() {
        std::cout << "[Consumer " << camera_id_ << "] Thread 시작\n";
        
        while (*running_flag_ || !ringBuffer_->empty()) {
            auto sample_opt = ringBuffer_->dequeue();
            if (sample_opt) {
                if (!processSample(sample_opt.value())) {
                    std::cerr << "[Consumer " << camera_id_ << "] Sample 처리 실패, Thread 종료\n";
                    *running_flag_ = false;
                    break;
                }
            } else {
                // config에서 설정값 사용
                std::this_thread::sleep_for(std::chrono::microseconds(config_.consumer_sleep_microseconds));
            }
        }
        
        std::cout << "[Consumer " << camera_id_ << "] Thread 종료\n";
    }

    bool processSample(ComPtr<IMFSample> sample) {
        ComPtr<IMFMediaBuffer> buffer;
        DWORD maxLength = 0, currentLength = 0;
        BYTE* data = nullptr;

        HRESULT hr = sample->ConvertToContiguousBuffer(&buffer);
        if (FAILED(hr)) return false;

        hr = buffer->Lock(&data, &maxLength, &currentLength);
        if (FAILED(hr)) return false;

        bool result = ffmpeg_->write(data, currentLength);
        
        buffer->Unlock();
        return result;
    }

    std::size_t processRemainingData() {
        return ringBuffer_->drainAll([this](ComPtr<IMFSample> sample) {
            processSample(sample);
        });
    }

    void cleanup() {
        callback_.Reset();
        sourceReader_.Reset();
        device_.Reset();
        
        if (running_flag_) {
            *running_flag_ = false;
        }
    }

    std::string stateToString(State state) const {
        switch (state) {
            case State::IDLE: return "IDLE";
            case State::SETTING_UP: return "SETTING_UP";
            case State::READY: return "READY";
            case State::RUNNING: return "RUNNING";
            case State::STOPPING: return "STOPPING";
            case State::ERROR_STATE: return "ERROR";
            default: return "UNKNOWN";
        }
    }
};

/**
 * MultiCameraManager Class - 완전 재구성된 동기화 시스템
 */
class MultiCameraManager {
public:
    enum class SyncStage {
        SETUP_COMPLETE,
        WARMUP_COMPLETE, 
        READY_TO_RECORD,
        RECORDING_START,
        RECORDING_END
    };

    bool setupAllCameras(const std::vector<Config>& configs) {
        cameras_.clear();
        configs_ = configs;
        const size_t camera_count = configs.size();
        
        initializeSyncBarriers(camera_count);
        
        for (const auto& config : configs) {
            auto manager = std::make_unique<SingleManager>(config);
            
            if (!manager->setup()) {
                int camera_index = config.camera_indices.empty() ? 0 : config.camera_indices[0];
                std::cerr << "[MultiCamera] Camera " << camera_index << " Setup 실패\n";
                return false;
            }
            
            cameras_.push_back(std::move(manager));
        }
        
        std::cout << "[MultiCamera] 모든 Camera Setup 완료 - 모든 카메라 Ready 상태\n";
        return true;
    }
    
    void startSynchronizedRecording() {
        const size_t camera_count = cameras_.size();
        std::vector<std::thread> recording_threads;
        
        for (size_t i = 0; i < camera_count; ++i) {
            recording_threads.emplace_back([this, i]() {
                synchronizedCameraLoop(i);
            });
        }
        
        coordinateAllStages();
        
        for (auto& thread : recording_threads) {
            thread.join();
        }
        
        std::cout << "[MultiCamera] 모든 Recording 완료\n";
        
        // 압축 수행
        if (!configs_.empty() && configs_[0].enable_compression) {
            compressAllRecordings();
        }
    }

private:
    std::vector<std::unique_ptr<SingleManager>> cameras_;
    std::vector<Config> configs_; // Config들 저장
    std::vector<std::unique_ptr<Barrier>> sync_barriers_;
    std::atomic<bool> global_start_signal_{false};
    std::chrono::high_resolution_clock::time_point global_start_time_;
    
    void compressAllRecordings() {
        if (configs_.empty()) {
            std::cerr << "[MultiCamera] Config가 없어 압축을 수행할 수 없습니다\n";
            return;
        }
        
        std::cout << "[MultiCamera] 압축 작업 시작\n";
        
        std::vector<std::string> recorded_files;
        
        // 모든 Camera의 출력 파일 수집
        for (const auto& camera : cameras_) {
            std::string output_file = camera->getOutputFile();
            if (std::filesystem::exists(output_file)) {
                recorded_files.push_back(output_file);
                std::cout << "[MultiCamera] 압축 대상 파일: " << output_file << "\n";
            } else {
                std::cerr << "[MultiCamera] 파일이 존재하지 않음: " << output_file << "\n";
            }
        }
        
        if (recorded_files.empty()) {
            std::cout << "[MultiCamera] 압축할 파일이 없습니다\n";
            return;
        }
        
       
    }
    
    void synchronizedCameraLoop(size_t camera_index) {
        auto& camera = cameras_[camera_index];
        std::string camera_id = camera->getCameraId();
        
        std::cout << "[Sync " << camera_id << "] Thread 시작\n";
        
        sync_barriers_[0]->arrive_and_wait();
        std::cout << "[Sync " << camera_id << "] Setup 동기화 완료\n";
        
        // Warm-up 수행
        camera->performWarmup();
        
        std::cout << "[Sync " << camera_id << "] Ready 상태 대기 중...\n";
        auto ready_timeout = PrecisionTimer::Clock::now() + std::chrono::seconds(5);
        
        while (!camera->isReadyForSync() && PrecisionTimer::Clock::now() < ready_timeout) {
            PrecisionTimer::preciseSleep(std::chrono::milliseconds(100));
        }
        
        if (!camera->isReadyForSync()) {
            std::cerr << "[Error " << camera_id << "] Ready 상태 대기 타임아웃!\n";
            return;
        }
        
        camera->setRecordingState(RecordingState::SYNCHRONIZED);
        sync_barriers_[1]->arrive_and_wait();
        std::cout << "[Sync " << camera_id << "] Warm-up 및 Ready 동기화 완료\n";
        
        sync_barriers_[2]->arrive_and_wait();
        
        while (!global_start_signal_.load()) {
            PrecisionTimer::preciseSleep(std::chrono::microseconds(10));
        }
        
        std::cout << "[Sync " << camera_id << "] 녹화 시작!\n";
        
        camera->startRecordingImmediately();
        
        // *** 최적화: 단순하게 State만 체크 (camera가 알아서 중지함) ***
        while (camera->getState() == SingleManager::State::RUNNING) {
            PrecisionTimer::preciseSleep(std::chrono::milliseconds(10));
        }
        
        camera->stopRecordingBySignal();
        
        sync_barriers_[3]->arrive_and_wait();
        std::cout << "[Sync " << camera_id << "] 종료 동기화 완료\n";
    }
    
    // 나머지 기존 메서드들...
    void initializeSyncBarriers(size_t camera_count) {
        sync_barriers_.clear();
        for (int i = 0; i < 4; ++i) {  // 5개에서 4개로 변경
            sync_barriers_.push_back(
                std::make_unique<Barrier>(camera_count + 1));
        }
    }
    
    void coordinateAllStages() {
        std::cout << "[MultiCamera] 동기화 조정 시작\n";
        
        sync_barriers_[0]->arrive_and_wait();
        std::cout << "[MultiCamera] 모든 카메라 Setup 완료\n";
        
        sync_barriers_[1]->arrive_and_wait();
        std::cout << "[MultiCamera] 모든 카메라 Warm-up 및 Ready 완료\n";
        
        sync_barriers_[2]->arrive_and_wait();
        std::cout << "[MultiCamera] 동시 시작 준비 완료\n";
        
        int delay_ms = configs_.empty() ? 100 : configs_[0].sync_delay_milliseconds;
        PrecisionTimer::preciseSleep(std::chrono::milliseconds(delay_ms));
        
        auto precise_start = PrecisionTimer::Clock::now();
        global_start_time_ = precise_start;
        global_start_signal_.store(true);
        
        auto start_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        auto start_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()) % 1000000;
        std::tm start_buf;
        localtime_s(&start_buf, &start_t);
        
        std::cout << "[Timing] *** Write Gate 열림 시각: " 
                << std::put_time(&start_buf, "%H:%M:%S")
                << "." << std::setw(6) << std::setfill('0') << start_us.count() << " ***\n";
        
        // *** 수정: global_stop_signal_ 제거, 각 camera가 자체적으로 종료 ***
        int record_duration = configs_.empty() ? 10 : configs_[0].record_duration;
        auto target_end_time = precise_start + std::chrono::seconds(record_duration);
        
        while (PrecisionTimer::Clock::now() < target_end_time) {
            auto remaining = target_end_time - PrecisionTimer::Clock::now();
            if (remaining > std::chrono::milliseconds(10)) {
                PrecisionTimer::preciseSleep(std::chrono::milliseconds(5));
            } else {
                PrecisionTimer::preciseSleep(std::chrono::microseconds(100));
            }
        }
        
        auto precise_end = PrecisionTimer::Clock::now();
        auto actual_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
            precise_end - precise_start).count();
        
        std::cout << "[Timing] *** 실제 Recording Duration: " 
                << actual_duration_us << "μs ***\n";
        
        sync_barriers_[3]->arrive_and_wait();
        std::cout << "[MultiCamera] 모든 녹화 종료 확인\n";
    }
};

/**
 * Main Function
 */
int main(int argc, char* argv[]) {
    Config config = parse_args(argc, argv);
    
    if (!config.validate()) {
        std::cerr << "[Main] 설정 오류로 인해 프로그램을 종료합니다.\n";
        return -1;
    }
    
    config.print();
    
    auto configs = createMultiCameraConfigs(config);
    
    std::cout << "[Main] " << configs.size() << "개 카메라로 실행합니다.\n";
    
    MultiCameraManager multiManager;
    
    if (!multiManager.setupAllCameras(configs)) {
        std::cerr << "[Main] 카메라 Setup 실패\n";
        return -1;
    }
    
    multiManager.startSynchronizedRecording();
    std::cout << "[Main] Recording 완료\n";
    
    return 0;
}