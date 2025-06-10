#pragma once

// Standard Library
#include <array>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
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


/**
 *  Helper: Callback
 */

class SampleCallback : public RuntimeClass<RuntimeClassFlags<ClassicCom>, IMFSourceReaderCallback> {
public:
    STDMETHODIMP OnReadSample(HRESULT hrStatus, DWORD streamIndex, DWORD flags, LONGLONG llTimestamp, IMFSample* sample) override {
        frame_count++;
        std::cout << "[Info] 수신 타임스탬프 (llTimestamp): " << llTimestamp << " (100ns 단위), 현재 프레임: " << frame_count << "\n";

        if (reader_) {
            reader_->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, nullptr, nullptr, nullptr, nullptr);
        }

        return S_OK;
    }
    STDMETHODIMP OnEvent(DWORD, IMFMediaEvent*) override { return S_OK; }
    STDMETHODIMP OnFlush(DWORD) override { return S_OK; }

    void setReader(IMFSourceReader* reader) {
        reader_ = reader;
    }

    void initCount() {
        frame_count = 0;
    }

private:
    IMFSourceReader* reader_ = nullptr;

    int frame_count = 0;
};


/**
 *  Helper: MediaFoundation
 */

class MediaFoundation {
public:
    MediaFoundation() {
        HRESULT hr = MFStartup(MF_VERSION);

        if (HFailed(hr, "[MediaFoundation] MFStartup")) { return; }
        else { initialized_ = true; }
    }

    ~MediaFoundation() {
        if (initialized_) { MFShutdown(); }
    }

    std::optional<ComPtr<IMFActivate>> getDevice(Config config) {
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

        if (config.camera_index >= every_count) {
            std::cout << "[Error] 잘못된 카메라 인덱스: " << config.camera_index << "\n";
            return std::nullopt;
        }

        IMFActivate* target_device = every_devices_raw[config.camera_index];
        ComPtr <IMFActivate> device = target_device;

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

    std::optional<ComPtr<SampleCallback>> getCallback() {
        auto callback = Microsoft::WRL::Make<SampleCallback>();

        return callback;
    }

    std::optional<ComPtr<IMFSourceReader>> getSourceReader(ComPtr<IMFActivate> device, ComPtr<SampleCallback> callback, Config config) {
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

        callback->setReader(pSourceReader.Get());
        
        return pSourceReader;
    }
    inline GUID PixelFormatFromString(const std::string& format) {
        if (format == "NV12")  return MFVideoFormat_NV12;
        if (format == "MJPG")  return MFVideoFormat_MJPG;
        if (format == "YUY2")  return MFVideoFormat_YUY2;
        if (format == "RGB32") return MFVideoFormat_RGB32;
        if (format == "I420")  return MFVideoFormat_I420;
        if (format == "UYVY")  return MFVideoFormat_UYVY;

        throw std::invalid_argument("Unsupported pixel format: " + format);
    }

    HRESULT getSource(ComPtr<IMFSourceReader> source_reader, ComPtr<SampleCallback> callback, Config config) {
        HRESULT hr = S_OK;

        callback->initCount();

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
    HRESULT hr = S_OK;

    Config config = parse_args(argc, argv);
    MediaFoundation mf;

    auto device = mf.getDevice(config);
    if (!device) {
        std::cout << "[Error] 카메라를 찾을 수 없습니다.\n";
        return -1;
    }
    std::cout << "[Info] 카메라를 찾았습니다.\n";

    auto callback = mf.getCallback();
    if (!callback) {
        std::cout << "[Error] IMFSourceReaderCallback을 생성할 수 없습니다.\n";
        return -1;
    }
    std::cout << "[Info] IMFSourceReaderCallback을 생성했습니다.\n";

    auto source_reader = mf.getSourceReader(device.value(), callback.value(), config);
    if (!source_reader) {
        std::cout << "[Error] IMFSourceReader를 생성할 수 없습니다.\n";
        return -1;
    }
    std::cout << "[Info] IMFSourceReader를 생성했습니다.\n";

    /**
     * Run
     */

    //  warm-up
    auto warmUp = mf.getSource(source_reader.value(), callback.value(), config);
    if (HFailed(warmUp, "[Error] warmUp를 실행할 수 없습니다.\n")) {
        return -1;
    }
    std::cout << "[Info] warmUp를 실행했습니다.\n";

    std::this_thread::sleep_for(std::chrono::seconds(3));

    // run
    auto source = mf.getSource(source_reader.value(), callback.value(), config);
    if (HFailed(source, "[Error] getSource를 실행할 수 없습니다.\n")) {
        return -1;
    }
    std::cout << "[Info] getSource를 실행했습니다.\n";

    std::this_thread::sleep_for(std::chrono::seconds(config.record_duration));
}