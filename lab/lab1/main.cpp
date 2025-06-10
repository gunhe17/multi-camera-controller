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

    std::optional<ComPtr<IMFSourceReader>> getSourceReader(ComPtr<IMFActivate> device, Config config) {
        HRESULT hr = MFStartup(MF_VERSION);

        ComPtr<IMFMediaSource> pSource;
        ComPtr<IMFAttributes> pAttributes;
        ComPtr<IMFSourceReader> pSourceReader;
        ComPtr<IMFMediaType> pType;
        
        // create device source
        hr = device->ActivateObject(IID_PPV_ARGS(&pSource));
        if (HFailed(hr, "[_getIMFSourceReader] ActivateObject failed")) return std::nullopt;

        // create attributes
        hr = MFCreateAttributes(&pAttributes, 1);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateAttributes failed")) return std::nullopt;
        
        // create source reader
        hr = MFCreateSourceReaderFromMediaSource(pSource.Get(), pAttributes.Get(), &pSourceReader);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateSourceReaderFromMediaSource failed")) return std::nullopt;
        
        // create media type        
        hr = MFCreateMediaType(&pType);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateMediaType failed")) return std::nullopt;
        
        // return
        return pSourceReader;
    }

    void getMediaType(ComPtr<IMFSourceReader> reader, Config config) {
        DWORD index = 0;
        ComPtr<IMFMediaType> pType;

        while (SUCCEEDED(reader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, index, &pType))) {
            GUID subtype = {};
            UINT32 width = 0, height = 0, fpsNum = 0, fpsDen = 1;

            pType->GetGUID(MF_MT_SUBTYPE, &subtype);
            MFGetAttributeSize(pType.Get(), MF_MT_FRAME_SIZE, &width, &height);
            MFGetAttributeRatio(pType.Get(), MF_MT_FRAME_RATE, &fpsNum, &fpsDen);

            float fps = fpsDen ? static_cast<float>(fpsNum) / fpsDen : 0.0f;

            if (width >= config.frame_width && height >= config.frame_height &&
                (fpsDen != 0 && fpsNum >= config.frame_rate * fpsDen)) {
                
                std::string format = GuidToFourCC(subtype);
                std::wstring formatW(format.begin(), format.end());

                std::wcout << L"[" << index << L"] "
                        << L"Format: " << formatW
                        << L", Resolution: " << width << L"x" << height
                        << L", FPS: " << fps
                        << std::endl;
            }

            pType.Reset();
            index++;
        }

        if (index == 0) {
            std::wcout << L"No supported media types found." << std::endl;
        }
    }
    inline std::string GuidToFourCC(GUID guid) {
        char fourcc[5] = {
            static_cast<char>(guid.Data1 & 0xFF),
            static_cast<char>((guid.Data1 >> 8) & 0xFF),
            static_cast<char>((guid.Data1 >> 16) & 0xFF),
            static_cast<char>((guid.Data1 >> 24) & 0xFF),
            '\0'
        };
        return std::string(fourcc);
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

    // get source reader
    auto source_reader = mf.getSourceReader(device.value(), config);
    if (!source_reader) {
        std::cout << "[Error] IMFSourceReader를 생성할 수 없습니다.\n";
        return -1;
    }
    std::cout << "[Info] IMFSourceReader를 생성했습니다.\n";


    /**
     *  Run
     */ 
    mf.getMediaType(source_reader.value(), config);

    return 0;
}