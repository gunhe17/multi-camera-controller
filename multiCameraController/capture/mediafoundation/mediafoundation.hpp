#pragma once

#include "../common/frame_types.hpp"
#include "../common/ffmpeg_input.hpp"
#include "../config/config.hpp"
#include "../util/error.hpp"

#include <algorithm>
#include <optional>
#include <vector>
#include <cstdint>

#include <wrl/client.h>

#include <mfapi.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mfreadwrite.h>
#include <mftransform.h>
#include <mferror.h>

using Microsoft::WRL::ComPtr;


/**
 * Class: MediaFoundation
 */

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
    std::optional<ComPtr<IMFActivate>> getDevice(int camera_index) {
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
        IMFActivate* target_device = every_devices[camera_index];
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
        ComPtr<IMFSourceReader> pSourceReader;
        ComPtr<IMFMediaType> pType;
        
        // create source
        hr = device->ActivateObject(IID_PPV_ARGS(&pSource));
        if (HFailed(hr, "[_getIMFSourceReader] ActivateObject failed")) return std::nullopt;
        
        // create source reader
        hr = MFCreateSourceReaderFromMediaSource(pSource.Get(), nullptr, &pSourceReader);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateSourceReaderFromMediaSource failed")) return std::nullopt;
        
        // create media type        
        hr = MFCreateMediaType(&pType);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateMediaType failed")) return std::nullopt;
    
        // return
        return pSourceReader;
    }

    std::optional<FFmpegInput> record(ComPtr<IMFSourceReader> source_reader, int camera_index, uint64_t frame_index) {
        HRESULT hr = S_OK;

        std::cout << "[record] 카메라 " << camera_index << " 샘플 읽기 시작\n";
        
        // read sample
        ComPtr<IMFSample> sample;
        DWORD stream_index = 0;
        DWORD flags = 0;
        LONGLONG sample_time = 0;
        hr = source_reader->ReadSample(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM,
            0,
            &stream_index,
            &flags,
            &sample_time,
            &sample
        );
        if (HFailed(hr, "[record] ReadSample failed")) return std::nullopt;

        if (!sample) {
            std::cout << "[record] ReadSample 성공했지만 샘플 없음\n";
            return std::nullopt;
        }

        std::cout << "[record] ReadSample 성공 (샘플 있음)" << std::endl;


        // to frame
        uint64_t rel_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        FrameMeta meta{
            frame_index++,
            static_cast<uint64_t>(sample_time * 100),
            rel_us,
            camera_index
        };
        std::cout << "[record] FrameMeta 생성 성공 - frame_index: " << meta.frame_index << "\n";

        // to buffer
        ComPtr<IMFMediaBuffer> pBuffer;
        hr = sample->ConvertToContiguousBuffer(&pBuffer);
        if (FAILED(hr)) return std::nullopt;
        std::cout << "[record] ConvertToContiguousBuffer 성공\n";

        BYTE* pData = nullptr;
        DWORD maxLength = 0, currentLength = 0;
        hr = pBuffer->Lock(&pData, &maxLength, &currentLength);
        if (FAILED(hr)) return std::nullopt;
        std::cout << "[record] Lock 성공 - maxLength: " << maxLength << ", currentLength: " << currentLength << "\n";

        std::vector<uint8_t> rawData(pData, pData + currentLength);
        pBuffer->Unlock();

        // to ffmpeg
        return FFmpegInput{ pData, currentLength };
    }

private:
    bool initialized_ = false;
};