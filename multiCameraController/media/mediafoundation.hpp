#pragma once


#include "../config/config.hpp"
#include "../util/error.hpp"

#include <algorithm>
#include <mfapi.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mfreadwrite.h>
#include <wrl/client.h>
#include <optional>


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
        ComPtr<IMFMediaSource> source;
        ComPtr<IMFSourceReader> sourceReader;
        ComPtr<IMFMediaType> type;
        
        ComPtr<IMFSourceReaderCallback> callback = new SampleCallback();
        
        // create source
        hr = device->ActivateObject(IID_PPV_ARGS(&source));
        if (HFailed(hr, "[_getIMFSourceReader] ActivateObject failed")) return std::nullopt;

        // create source reader
        ComPtr<IMFAttributes> attributes;
        hr = MFCreateAttributes(&attributes, 1);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateAttributes failed")) return std::nullopt;

        hr = attributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, callback.Get());
        if (HFailed(hr, "[_getIMFSourceReader] SetGUID failed")) return std::nullopt;
        
        hr = MFCreateSourceReaderFromMediaSource(source.Get(), attributes.Get(), &sourceReader);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateSourceReaderFromMediaSource failed")) return std::nullopt;

        callback->setReader(sourceReader.Get());
        
        // create media type        
        hr = MFCreateMediaType(&type);
        if (HFailed(hr, "[_getIMFSourceReader] MFCreateMediaType failed")) return std::nullopt;

        // set media attributes
        type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        type->SetGUID(MF_MT_SUBTYPE, config.pixel_format_guid);
        type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);

        MFSetAttributeSize(type.Get(), MF_MT_FRAME_SIZE, config.frame_width, config.frame_height);
        MFSetAttributeRatio(type.Get(), MF_MT_FRAME_RATE, config.frame_rate, 1);

        hr = sourceReader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, type.Get());
        if (HFailed(hr, "[_getIMFSourceReader] SetCurrentMediaType failed")) return std::nullopt;
        
        // return
        return sourceReader;
    }

    void warmup(ComPtr<IMFSourceReader> reader) {
        for (int i = 0; i < 30; ++i) {
            ComPtr<IMFSample> sample;
            DWORD streamIndex, flags;
            LONGLONG timestamp;
            reader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, &streamIndex, &flags, &timestamp, &sample);
        }
    }

    void capture(ComPtr<IMFSourceReader> reader) {
        ComPtr<IMFSample> sample;
        DWORD streamIndex = 0, flags = 0;
        LONGLONG timestamp = 0;

        do {
            reader->ReadSample(
                MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                0,
                &streamIndex,
                &flags,
                &timestamp,
                &sample
            );
        } 
        while (!sample);
    }

private:
    bool initialized_ = false;
};



