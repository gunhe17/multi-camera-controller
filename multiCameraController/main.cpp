#pragma once

// Standard Library
#include <array>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <chrono>
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
 * Refactored FFmpeg Class
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
            return true;
        }
        
        state_ = State::STOPPED;
        return false;
    }

    bool write(const BYTE* data, size_t length) {
        if (state_ != State::RUNNING || !ffmpeg_stdin_) {
            return false;
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
        
        sendQuitCommand();
        waitForProcessTermination();
        cleanup();
        
        state_ = State::STOPPED;
    }

    State getState() const { return state_; }
    bool isRunning() const { return state_ == State::RUNNING; }

private:
    Config config_;
    HANDLE ffmpeg_stdin_ = NULL;
    HANDLE ffmpeg_proc_ = NULL;
    HANDLE read_handle_ = NULL;
    std::atomic<State> state_{State::STOPPED};

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

    void sendQuitCommand() {
        if (ffmpeg_stdin_) {
            const char quit_cmd = 'q';
            DWORD written = 0;
            WriteFile(ffmpeg_stdin_, &quit_cmd, 1, &written, NULL);
        }
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
 * Refactored SampleCallback
 */
class SampleCallback : public RuntimeClass<RuntimeClassFlags<ClassicCom>, IMFSourceReaderCallback> {
public:
    SampleCallback(SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>* ringBuffer,
                   std::atomic<bool>* runningFlag)
        : ringBuffer_(ringBuffer), external_running_flag_(runningFlag) {}

    STDMETHODIMP OnReadSample(HRESULT hrStatus, DWORD streamIndex, DWORD flags, 
                             LONGLONG llTimestamp, IMFSample* sample) override {
        frameCount_++;
        
        logTimestamp(llTimestamp);
        
        if (!isRunning()) {
            std::cout << "[Callback] 종료 상태 감지, ReadSample 중지\n";
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
    
    int getFrameCount() const { return frameCount_; }
    bool isRunning() const { 
        return external_running_flag_ && *external_running_flag_; 
    }

private:
    SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>* ringBuffer_;
    std::atomic<bool>* external_running_flag_;
    IMFSourceReader* reader_ = nullptr;
    std::atomic<int> frameCount_{0};
    
    void logTimestamp(LONGLONG timestamp) {
        std::cout << "[Info] 수신 타임스탬프: " << timestamp 
                  << ", frame count: " << frameCount_ << "\n";
        
        writeCsvLog(timestamp);
    }
    
    void writeCsvLog(LONGLONG timestamp) {
        static std::ofstream csv("timestamps.csv", std::ios::out);
        static bool wroteHeader = false;
        static LONGLONG prevTimestamp = -1;

        if (csv.is_open()) {
            if (!wroteHeader) {
                csv << "FrameIndex,Timestamp100ns,Timestamp(ms),Delta100ns,Delta(ms)\n";
                wroteHeader = true;
            }

            LONGLONG delta = (prevTimestamp < 0) ? 0 : (timestamp - prevTimestamp);
            double timestamp_ms = timestamp / 10000.0;
            double delta_ms = delta / 10000.0;

            csv << frameCount_ << "," << timestamp << "," << timestamp_ms << ","
                << delta << "," << delta_ms << "\n";

            prevTimestamp = timestamp;
        }
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
 * SingleManager Class
 */
class SingleManager {
public:
    enum class State {
        IDLE,
        SETTING_UP,
        READY,
        WARMING_UP,
        WARMED_UP,
        RUNNING,
        STOPPING,
        ERROR_STATE
    };

    SingleManager(const Config& config) : config_(config), state_(State::IDLE) {
        if (config_.camera_indices.empty()) {
            throw std::invalid_argument("카메라 인덱스가 비어있습니다.");
        }
        
        ringBuffer_ = std::make_unique<SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>>();
        ffmpeg_ = std::make_unique<FFmpeg>();
        mf_ = std::make_unique<MediaFoundation>();
    }

    ~SingleManager() {
        if (state_ == State::RUNNING) {
            stop();
        }
    }

    bool setup() {
        if (state_ != State::IDLE) {
            std::cerr << "[SingleManager] Setup 실패: 잘못된 상태 " << stateToString(state_) << "\n";
            return false;
        }

        state_ = State::SETTING_UP;

        try {
            auto device = mf_->createDevice(config_);
            if (!device) {
                throw std::runtime_error("Camera Device 생성 실패");
            }
            device_ = device.value();
            std::cout << "[SingleManager] Camera Device 생성 완료\n";

            running_flag_ = std::make_unique<std::atomic<bool>>(false);
            callback_ = Microsoft::WRL::Make<SampleCallback>(ringBuffer_.get(), running_flag_.get());
            std::cout << "[SingleManager] Callback 생성 완료\n";

            auto sourceReader = mf_->createSourceReader(device_, callback_, config_);
            if (!sourceReader) {
                throw std::runtime_error("SourceReader 생성 실패");
            }
            sourceReader_ = sourceReader.value();
            callback_->setReader(sourceReader_.Get());
            std::cout << "[SingleManager] SourceReader 생성 완료\n";

            ffmpeg_->setConfig(config_);
            std::cout << "[SingleManager] FFmpeg 설정 완료\n";

            state_ = State::READY;
            std::cout << "[SingleManager] Setup 완료\n";
            return true;

        } catch (const std::exception& e) {
            std::cerr << "[SingleManager] Setup 실패: " << e.what() << "\n";
            state_ = State::ERROR_STATE;
            return false;
        }
    }

    bool warmup() {
        if (state_ != State::READY) {
            std::cerr << "[SingleManager] Warmup 실패: 잘못된 상태 " << stateToString(state_) << "\n";
            return false;
        }

        state_ = State::WARMING_UP;

        try {
            if (!ffmpeg_->start()) {
                throw std::runtime_error("FFmpeg 시작 실패");
            }
            std::cout << "[SingleManager] FFmpeg 시작 완료\n";

            *running_flag_ = true;
            consumer_thread_ = std::thread([this]() { this->consumerLoop(); });
            std::cout << "[SingleManager] Consumer Thread 시작\n";

            HRESULT hr = sourceReader_->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, 
                                                  nullptr, nullptr, nullptr, nullptr);
            if (FAILED(hr)) {
                throw std::runtime_error("첫 ReadSample 실패");
            }

            std::this_thread::sleep_for(std::chrono::seconds(3));
            callback_->resetFrameCount();

            state_ = State::WARMED_UP;
            std::cout << "[SingleManager] Warmup 완료\n";
            return true;

        } catch (const std::exception& e) {
            std::cerr << "[SingleManager] Warmup 실패: " << e.what() << "\n";
            state_ = State::ERROR_STATE;
            return false;
        }
    }

    bool run() {
        if (state_ != State::WARMED_UP) {
            std::cerr << "[SingleManager] Run 실패: 잘못된 상태 " << stateToString(state_) << "\n";
            return false;
        }

        state_ = State::RUNNING;
        start_time_ = std::chrono::steady_clock::now();

        std::cout << "[SingleManager] Recording 시작\n";
        std::this_thread::sleep_for(std::chrono::seconds(config_.record_duration));
        std::cout << "[SingleManager] Recording 완료\n";
        return true;
    }

    void stop() {
        if (state_ != State::RUNNING && state_ != State::WARMED_UP) {
            return;
        }

        state_ = State::STOPPING;
        std::cout << "[SingleManager] 종료 시작\n";

        *running_flag_ = false;

        if (sourceReader_) {
            sourceReader_->Flush(MF_SOURCE_READER_FIRST_VIDEO_STREAM);
            std::cout << "[SingleManager] SourceReader 플러시 완료\n";
        }

        if (consumer_thread_.joinable()) {
            consumer_thread_.join();
            std::cout << "[SingleManager] Consumer Thread 종료 완료\n";
        }

        auto processed = processRemainingData();
        std::cout << "[SingleManager] 남은 데이터 " << processed << "개 처리 완료\n";

        ffmpeg_->stop();
        std::cout << "[SingleManager] FFmpeg 종료 완료\n";

        cleanup();

        state_ = State::IDLE;
        std::cout << "[SingleManager] 종료 완료\n";
    }

    State getState() const { return state_; }
    bool isReady() const { return state_ == State::READY || state_ == State::WARMED_UP; }
    bool isRunning() const { return state_ == State::RUNNING; }
    bool isError() const { return state_ == State::ERROR_STATE; }

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
        std::cout << "[SingleManager] 상태: " << stateToString(state_)
                  << ", Frames: " << stats.frame_count
                  << ", Duration: " << stats.recording_duration_ms << "ms"
                  << ", Buffer: " << stats.buffer_stats.size << "/" << ringBuffer_->capacity()
                  << "\n";
    }

private:
    Config config_;
    std::atomic<State> state_;
    std::chrono::steady_clock::time_point start_time_;

    std::unique_ptr<MediaFoundation> mf_;
    std::unique_ptr<FFmpeg> ffmpeg_;
    std::unique_ptr<SPSCRingBuffer<ComPtr<IMFSample>, RING_BUFFER_SIZE>> ringBuffer_;
    std::unique_ptr<std::atomic<bool>> running_flag_;

    ComPtr<IMFActivate> device_;
    ComPtr<IMFSourceReader> sourceReader_;
    ComPtr<SampleCallback> callback_;

    std::thread consumer_thread_;

    void consumerLoop() {
        std::cout << "[Consumer] Thread 시작\n";
        
        while (*running_flag_ || !ringBuffer_->empty()) {
            auto sample_opt = ringBuffer_->dequeue();
            if (sample_opt) {
                if (!processSample(sample_opt.value())) {
                    std::cerr << "[Consumer] Sample 처리 실패, Thread 종료\n";
                    *running_flag_ = false;
                    break;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
        
        std::cout << "[Consumer] Thread 종료\n";
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
            case State::WARMING_UP: return "WARMING_UP";
            case State::WARMED_UP: return "WARMED_UP";
            case State::RUNNING: return "RUNNING";
            case State::STOPPING: return "STOPPING";
            case State::ERROR_STATE: return "ERROR";
            default: return "UNKNOWN";
        }
    }
};


/**
 * MultiCameraManager Class (C++17 compatible)
 */
class MultiCameraManager {
public:
    bool setupAllCameras(const std::vector<Config>& configs) {
        cameras_.clear();
        
        for (const auto& config : configs) {
            auto manager = std::make_unique<SingleManager>(config);
            
            if (!manager->setup()) {
                int camera_index = config.camera_indices.empty() ? 0 : config.camera_indices[0];
                std::cerr << "[MultiCamera] Camera " << camera_index << " Setup 실패\n";
                return false;
            }
            
            cameras_.push_back(std::move(manager));
        }
        
        std::cout << "[MultiCamera] 모든 Camera Setup 완료\n";
        return true;
    }
    
    bool warmupAllCameras() {
        for (auto& camera : cameras_) {
            if (!camera->warmup()) {
                std::cerr << "[MultiCamera] Warmup 실패\n";
                return false;
            }
        }
        
        std::cout << "[MultiCamera] 모든 Camera Warmup 완료\n";
        return true;
    }
    
    void startSynchronizedRecording() {
        Barrier sync_barrier(cameras_.size() + 1);
        
        std::vector<std::thread> recording_threads;
        
        for (auto& camera : cameras_) {
            recording_threads.emplace_back([&camera, &sync_barrier]() {
                sync_barrier.arrive_and_wait();
                camera->run();
            });
        }
        
        sync_barrier.arrive_and_wait();
        std::cout << "[MultiCamera] 동시 Recording 시작!\n";
        
        for (auto& thread : recording_threads) {
            thread.join();
        }
        
        std::cout << "[MultiCamera] 모든 Recording 완료\n";
    }
    
private:
    std::vector<std::unique_ptr<SingleManager>> cameras_;
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
    
    if (!multiManager.warmupAllCameras()) {
        std::cerr << "[Main] 카메라 Warmup 실패\n";
        return -1;
    }
    
    multiManager.startSynchronizedRecording();
    std::cout << "[Main] Recording 완료\n";
    
    return 0;
}