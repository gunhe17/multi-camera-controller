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

// using
using namespace Microsoft::WRL;
using Microsoft::WRL::ComPtr;


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
 * Helper: SPSCRingBuffer
 */
template <typename T, std::size_t N>
class SPSCRingBuffer {
private:
    std::atomic<std::size_t> m_head;
    std::array<T, N> m_buff;
    std::atomic<std::size_t> m_tail;

public:
    constexpr SPSCRingBuffer() noexcept : m_head(0), m_tail(0) {}

    constexpr std::size_t size() noexcept { return N; }

    bool enqueue(T val) noexcept {
        std::size_t current_tail = m_tail.load(std::memory_order_relaxed);
        if (current_tail - m_head.load(std::memory_order_relaxed) < N) {
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
            m_head.store(current_head + 1, std::memory_order_relaxed);
            return std::optional(std::move(value));
        }
        return std::nullopt;
    }

    void debug_print_tail() {
        std::cout << "tail: " << m_tail.load(std::memory_order_relaxed) << std::endl;
    }

    void debug_print_head() {
        std::cout << "head: " << m_head.load(std::memory_order_relaxed) << std::endl;
    }
};


/**
 *  Helper: Callback
 */
class SampleCallback : public RuntimeClass<RuntimeClassFlags<ClassicCom>, IMFSourceReaderCallback> {
public:
    // common
    STDMETHODIMP OnReadSample(HRESULT hrStatus, DWORD streamIndex, DWORD flags, LONGLONG llTimestamp, IMFSample* sample) override {
        std::cout << "[Info] 수신 타임스탬프 (llTimestamp): " << llTimestamp << " (100ns 단위)\n";

        if (ringBuffer_ && sample) {
            ringBuffer_->enqueue(sample);
            ringBuffer_->dequeue();
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

    void setSPSCRingBuffer(SPSCRingBuffer<ComPtr<IMFSample>, 1024>* ringBuffer) {
        ringBuffer_ = ringBuffer;
    }

private:
    IMFSourceReader* reader_ = nullptr;
    SPSCRingBuffer<ComPtr<IMFSample>, 1024>* ringBuffer_ = nullptr;
};


/**
 *  Helper: MediaFoundation
 */
class MediaFoundation {
public:
    // initial
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

    SPSCRingBuffer<ComPtr<IMFSample>, 1024> getRingBuffer() {
        return SPSCRingBuffer<ComPtr<IMFSample>, 1024>();
    }

    std::optional<ComPtr<SampleCallback>> getCallback(SPSCRingBuffer<ComPtr<IMFSample>, 1024>& ringBuffer) {
        auto callback = Microsoft::WRL::Make<SampleCallback>();

        callback->setSPSCRingBuffer(&ringBuffer);

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
        pType->SetGUID(MF_MT_SUBTYPE, config.pixel_format);
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

    Config config = parse_args(argc, argv);
    MediaFoundation mf;

    // get device
    auto device = mf.getDevice(config);
    if (!device) {
        std::cout << "[Error] 카메라를 찾을 수 없습니다.\n";
        return -1;
    }
    std::cout << "[Info] 카메라를 찾았습니다.\n";

    // get Queue
    auto ringBuffer = mf.getRingBuffer();
    std::cout << "[Info] SPSCRingBuffer를 생성했습니다.\n";

    // get callback
    auto callback = mf.getCallback(ringBuffer);
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

    // get some Source
    auto source = mf.getSource(source_reader.value(), config);
    if (HFailed(source, "[Error] getSource를 실행할 수 없습니다.\n")) return -1;
    std::cout << "[Info] getSource를 실행했습니다.\n";   

    // wait callback run
    std::this_thread::sleep_for(std::chrono::seconds(30));

    return 0;
}