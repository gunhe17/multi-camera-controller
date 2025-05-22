#pragma once

// Standard Library
#include <algorithm>
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
#include "config.hpp"


/**
 * Tester: ...
 */
class Tester {
public:
    Tester() { QueryPerformanceFrequency(&freq_); }

    void start() { QueryPerformanceCounter(&tstart_); }

    void stop() {
        QueryPerformanceCounter(&tstop_);
        elapsed_sec_ = static_cast<double>(tstop_.QuadPart - tstart_.QuadPart) / freq_.QuadPart;
        double elapsed_ms = elapsed_sec_ * 1000.0;

        // 콘솔 출력
        std::cout << "[Timer] Elapsed: " << elapsed_ms << " ms\n";

        // CSV 로그 출력
        std::ofstream log_file("./lab/lab3/output/output.csv", std::ios::app);  // 경로 슬래시 '\' 대신 '/' 사용 권장
        if (log_file.is_open()) {
            log_file << "Timer" << "," << elapsed_ms << "\n";
        }
    }


    double elapsed_sec() const { return elapsed_sec_; }
    double elapsed_ms()  const { return elapsed_sec_ * 1000.0; }

private:
    LARGE_INTEGER freq_{};
    LARGE_INTEGER tstart_{};
    LARGE_INTEGER tstop_{};
    double elapsed_sec_ = 0.0;
};


/**
 *  Logger: csv
 */
struct FrameLog {
    size_t frame_index;
    LONGLONG timestamp_100ns;
    bool is_null_sample;
};


/**
 *  Helper: Error Check
 */
inline bool HFailed(HRESULT hr, const char* message) {
    if (FAILED(hr)) {
        std::ostringstream oss;
        oss << "[Error] " << message << " failed: 0x" << std::hex << hr << "\n";
        
        std::cout << oss.str();
        std::cout << oss.str();
        
        return true;
    }
    return false;
}

inline bool CFailed(HRESULT hr, const char* message) {
    if (FAILED(hr)) {
        std::ostringstream oss;
        oss << "[Error] " << message << " failed: 0x" << std::hex << hr << "\n";
        
        std::cout << oss.str();
        std::cout << oss.str();
        
        return true;
    }
    return false;
}


/**
 * Helper: FrameQuene
 */
using Microsoft::WRL::ComPtr;

class FrameQueue {
public:
    void push(ComPtr<IMFSample> sample) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(sample);

        ++total_pushed_;
    }

    ComPtr<IMFSample> pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return nullptr;

        ComPtr<IMFSample> sample = queue_.front();
        queue_.pop();
        return sample;
    }

    size_t total_pushed() const {
        return total_pushed_;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
private:
    std::queue<ComPtr<IMFSample>> queue_;
    mutable std::mutex mutex_;
    std::atomic<size_t> total_pushed_ = 0;
};


/**
 *  Helper: Callback
 */
using namespace Microsoft::WRL;

class SampleCallback : public RuntimeClass<RuntimeClassFlags<ClassicCom>, IMFSourceReaderCallback> {
public:
    // common
    STDMETHODIMP OnReadSample(HRESULT hrStatus, DWORD, DWORD, LONGLONG timestamp, IMFSample* sample) override {
        auto t0 = std::chrono::high_resolution_clock::now();
        ++read_sample_called_;

        // for logging
        {
            std::lock_guard<std::mutex> lock(log_mutex_);
            logs_.push_back(FrameLog{
                read_sample_called_,
                timestamp,
                sample == nullptr
            });
        }

        // if (FAILED(hrStatus)) {
        //     std::cerr << "Error receiving sample: 0x" << std::hex << hrStatus << std::endl;
        //     return hrStatus;
        // }

        // if (!sample) {
        //     ++null_sample_count_;
        // }

        if (frameQueue_ && sample) {
            // frameQueue_->push(sample);
            // auto t1 = std::chrono::high_resolution_clock::now();
            // std::cout << "Push time (us): " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "\n";
        }

        if (reader_) {
            reader_->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, nullptr, nullptr, nullptr, nullptr);
        }

        return S_OK;
    }
    STDMETHODIMP OnEvent(DWORD, IMFMediaEvent*) override { return S_OK; }
    STDMETHODIMP OnFlush(DWORD) override { return S_OK; }

    // helper
    void setReader(IMFSourceReader* reader) {
        reader_ = reader;
    }

    void setFrameQueue(FrameQueue* queue) {
        frameQueue_ = queue;
    }

    // print
    size_t read_sample_called() const {
        return read_sample_called_;
    }

    size_t null_sample_count() const {
        return null_sample_count_;
    }

    void save_logs_to_csv(const std::string& filename = "./lab/lab3/result/frame_log.csv") const {
        std::ofstream out(filename);
        out << "frame_index,timestamp_100ns,is_null_sample\n";

        for (const auto& log : logs_) {
            out << log.frame_index << ","
                << log.timestamp_100ns << ","
                << (log.is_null_sample ? "true" : "false") << "\n";
        }

        std::cout << "[Info] 로그 저장 완료: " << filename << "\n";
    }


private:
    IMFSourceReader* reader_ = nullptr;
    FrameQueue* frameQueue_ = nullptr;

    // index
    std::atomic<size_t> read_sample_called_ = 0;
    std::atomic<size_t> null_sample_count_ = 0;

    // for logging
    std::vector<FrameLog> logs_;
    std::mutex log_mutex_;
};


/**
 *  Helper: MediaFoundation
 */
using Microsoft::WRL::ComPtr;

class MediaFoundation {
public:
    // common
    MediaFoundation() {
        HRESULT hr = MFStartup(MF_VERSION);

        if (HFailed(hr, "[MediaFoundation] MFStartup")) { return; }
        else { initialized_ = true; }
    }

    ~MediaFoundation() {
        if (initialized_) { MFShutdown(); }
    }

    // helper
    std::optional<ComPtr<IMFActivate>> getDevice(Config config) {
        HRESULT hr = S_OK;

        // get every devices
        ComPtr<IMFAttributes> every_attributes;
        hr = MFCreateAttributes(&every_attributes, 1);
        if (HFailed(hr, "[_getDevice] MFCreateAttributes failed")) std::nullopt;

        hr = every_attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (HFailed(hr, "[_getDevice] SetGUID failed")) std::nullopt;

        IMFActivate** every_devices = nullptr;
        UINT32 every_count = 0;

        hr = MFEnumDeviceSources(every_attributes.Get(), &every_devices, &every_count);
        if (HFailed(hr, "[_getDevice] MFEnumDeviceSources failed")) std::nullopt;

        // get video devices
        ComPtr<IMFAttributes> video_attributes;

        hr = MFCreateAttributes(&video_attributes, 1);
        if (HFailed(hr, "[_getDevice] MFCreateAttributes failed")) std::nullopt;

        hr = video_attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (HFailed(hr, "[_getDevice] SetGUID failed")) std::nullopt;

        IMFActivate** video_devices = nullptr;
        UINT32 video_count = 0;

        hr = MFEnumDeviceSources(video_attributes.Get(), &video_devices, &video_count);
        if (HFailed(hr, "[_getDevice] MFEnumDeviceSources failed")) std::nullopt;

        // get target device
        IMFActivate* target_device = every_devices[config.camera_index];
        ComPtr <IMFActivate> device = target_device;

        // check
        BOOL match = FALSE;
        bool is_valid = std::any_of(
            video_devices, video_devices + video_count,
            [&](IMFActivate* video_dev) {
                match = FALSE;
                return SUCCEEDED(target_device->Compare(video_dev, MF_ATTRIBUTES_MATCH_INTERSECTION, &match)) && match;
            }
        );
        
        // clean up
        CoTaskMemFree(every_devices);
        CoTaskMemFree(video_devices);

        // return
        if (!is_valid) std::nullopt;

        return device;
    }

    std::optional<ComPtr<SampleCallback>> getCallback(FrameQueue& frameQueue) {
        auto callback = Microsoft::WRL::Make<SampleCallback>();

        callback->setFrameQueue(&frameQueue);

        return callback;
    }

    std::optional<ComPtr<IMFSourceReader>> getSourceReader(ComPtr<IMFActivate> device, ComPtr<SampleCallback> callback, Config config) {
        HRESULT hr = MFStartup(MF_VERSION);

        ComPtr<IMFMediaSource> pSource;
        ComPtr<IMFAttributes> pAttributes;
        ComPtr<IMFSourceReader> pSourceReader;
        ComPtr<IMFMediaType> pType;
        
        // create device source
        hr = device->ActivateObject(IID_PPV_ARGS(&pSource));
        if (HFailed(hr, "[_getIMFSourceReader] ActivateObject failed")) return std::nullopt;

        // create callback attributes
        hr = MFCreateAttributes(&pAttributes, 1);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateAttributes failed")) return std::nullopt;

        hr = pAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, callback.Get());
        if (HFailed(hr, "[_getIMFSourceReader] SetUnknown failed")) return std::nullopt;
        
        // create source reader
        hr = MFCreateSourceReaderFromMediaSource(pSource.Get(), pAttributes.Get(), &pSourceReader);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateSourceReaderFromMediaSource failed")) return std::nullopt;
        
        // create media type        
        hr = MFCreateMediaType(&pType);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateMediaType failed")) return std::nullopt;

        // set media attributes
        pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);

        MFSetAttributeSize(pType.Get(), MF_MT_FRAME_SIZE, config.frame_width, config.frame_height);
        MFSetAttributeRatio(pType.Get(), MF_MT_FRAME_RATE, config.frame_rate, 1);

        hr = pSourceReader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, pType.Get());
        if (HFailed(hr, "[_getIMFSourceReader] SetCurrentMediaType failed")) return std::nullopt;

        // + register callback
        callback->setReader(pSourceReader.Get());
        
        // return
        return pSourceReader;
    }

    HRESULT getSource(ComPtr<IMFSourceReader> source_reader, Config config) {
        HRESULT hr = S_OK;

        hr = source_reader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, nullptr, nullptr, nullptr, nullptr);
        if (HFailed(hr, "[_getSource] ReadSample failed")) return hr;

        return hr;
    }

private:
    bool initialized_ = false;
};


/**
 *  Main
 */
int main(int argc, char* argv[]) {
    
    // Initialize
    HRESULT hr = S_OK;
    Tester tester;
    Config config = parse_args(argc, argv);
    MediaFoundation mf;

    // get device
    auto device = mf.getDevice(config);
    if (!device) {
        std::cout << "[Error] 카메라를 찾을 수 없습니다.\n";
        return -1;
    }
    std::cout << "[Info] 카메라를 찾았습니다.\n";

    // get Quene
    auto frameQueue = FrameQueue();
    std::cout << "[Info] FrameQueue를 생성했습니다.\n";

    // get callback
    auto callback = mf.getCallback(frameQueue);
    if (!callback) {
        std::cout << "[Error] IMFSourceReaderCallback을 생성할 수 없습니다.\n";
        return -1;
    }
    std::cout << "[Info] IMFSourceReaderCallback을 생성했습니다.\n";

    // get source reader
    auto source_reader = mf.getSourceReader(device.value(), callback.value(), config);
    if (!source_reader) {
        std::cout << "[Error] IMFSourceReader를 생성할 수 없습니다.\n";
        return -1;
    }
    std::cout << "[Info] IMFSourceReader를 생성했습니다.\n";

    /**
     * TEST: run
     */
    std::cout << "[Info] 녹화를 시작합니다.\n";

    tester.start();

    hr = mf.getSource(source_reader.value(), config);
    if (HFailed(hr, "[_getSource] ReadSample failed")) return -1;

    std::this_thread::sleep_for(std::chrono::seconds(config.duration_time));

    tester.stop();
    std::cout << "[Info] 녹화를 종료합니다.\n";

    // print
    size_t expected = config.frame_rate * config.duration_time;
    size_t received = frameQueue.total_pushed();
    size_t called = callback.value() -> read_sample_called();
    size_t null_sample = callback.value() -> null_sample_count();

    std::cout << "[Result] 예상 프레임 수: " << expected << "\n";
    std::cout << "[Result] 수신된 프레임 수: " << received << "\n";
    std::cout << "[Debug] OnReadSample 호출 수: " << called << "\n";
    std::cout << "[Debug] null sample 수: " << null_sample << "\n";
    std::cout << "[Result] 손실률: " << ((expected - received) * 100.0 / expected) << "%\n";

    callback.value()->save_logs_to_csv();

    return 0;
}