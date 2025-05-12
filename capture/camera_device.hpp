#ifndef CAMERA_DEVICE_HPP
#define CAMERA_DEVICE_HPP

#include "../include/common.hpp"
#include "timestamp_logger.hpp"
#include "../ffmpeg/ffmpeg_writer.hpp"

#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mfobjects.h>
#include <mferror.h>
#include <comdef.h>

#include <thread>
#include <atomic>
#include <iostream>
#include <chrono>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

class CameraDevice {
public:
    CameraDevice(const CaptureConfig& config, TimestampLogger& logger)
        : config_(config), logger_(logger), is_running_(false),
          pSource_(nullptr), pSourceReader_(nullptr), writer_(config) {}

    bool initialize() {
        HRESULT hr = MFStartup(MF_VERSION);
        if (FAILED(hr)) {
            std::cerr << "MFStartup 실패\n";
            return false;
        }

        IMFAttributes* pAttributes = nullptr;
        hr = MFCreateAttributes(&pAttributes, 1);
        if (FAILED(hr)) return false;

        hr = pAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (FAILED(hr)) return false;

        IMFActivate** ppDevices = nullptr;
        UINT32 count = 0;
        hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);
        if (FAILED(hr) || config_.camera_id >= static_cast<int>(count)) return false;

        hr = ppDevices[config_.camera_id]->ActivateObject(IID_PPV_ARGS(&pSource_));
        if (FAILED(hr)) return false;

        hr = MFCreateSourceReaderFromMediaSource(pSource_, nullptr, &pSourceReader_);
        if (FAILED(hr)) return false;

        // 🔧 미디어 타입을 YUY2로 명시 설정 (FFmpeg에서 yuyv422와 대응됨)
        IMFMediaType* pType = nullptr;
        hr = MFCreateMediaType(&pType);
        if (FAILED(hr)) return false;

        pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_YUY2);  // YUY2 = yuyv422
        pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
        MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, config_.width, config_.height);
        MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, config_.fps, 1);

        hr = pSourceReader_->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, pType);
        pType->Release();

        if (FAILED(hr)) {
            std::cerr << "MediaType 설정 실패\n";
            return false;
        }

        for (UINT32 i = 0; i < count; ++i) ppDevices[i]->Release();
        CoTaskMemFree(ppDevices);
        pAttributes->Release();

        if (!writer_.start()) {
            std::cerr << "FFmpegWriter 시작 실패\n";
            return false;
        }

        std::cout << "[Camera " << config_.camera_id << "] Initialized.\n";
        return true;
    }

    void start(std::chrono::steady_clock::time_point T0) {
        is_running_ = true;
        capture_thread_ = std::thread([this, T0]() {
            int frame_index = 0;

            while (is_running_) {
                DWORD streamIndex, flags;
                LONGLONG sampleTime;
                IMFSample* pSample = nullptr;

                HRESULT hr = pSourceReader_->ReadSample(
                    MF_SOURCE_READER_FIRST_VIDEO_STREAM,
                    0, &streamIndex, &flags, &sampleTime, &pSample);

                if (FAILED(hr) || !pSample) continue;

                auto rel_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::nanoseconds(sampleTime * 100)).count();

                FrameMeta meta{
                    frame_index,
                    static_cast<uint64_t>(sampleTime * 100),
                    rel_us,
                    config_.camera_id
                };
                logger_.log(meta);

                // 콘솔 출력 (30프레임마다 한 번)
                if (frame_index % 30 == 0) {
                    std::cout << "[Camera " << config_.camera_id << "] frame " << frame_index
                            << " (timestamp: " << meta.timestamp_ns << " ns)\n";
                }

                frame_index++;

                // 프레임 버퍼 추출 및 FFmpeg로 전달
                IMFMediaBuffer* pBuffer = nullptr;
                BYTE* pData = nullptr;
                DWORD maxLength = 0, currentLength = 0;

                hr = pSample->ConvertToContiguousBuffer(&pBuffer);
                if (SUCCEEDED(hr)) {
                    hr = pBuffer->Lock(&pData, &maxLength, &currentLength);
                    if (SUCCEEDED(hr)) {
                        bool ok = writer_.write(pData, currentLength);
                        if (!ok) {
                            std::cerr << "[Camera " << config_.camera_id << "] FFmpeg 종료 감지 → 루프 종료\n";
                            is_running_ = false;
                            break;
                        }       
                        pBuffer->Unlock();
                    }
                    pBuffer->Release();
                }

                pSample->Release();
            }
        });
    }


    void stop() {
        is_running_ = false;
        if (capture_thread_.joinable())
            capture_thread_.join();

        // 먼저 writer 종료 → 모든 프레임 write 및 ffmpeg 인코딩 완료 대기
        writer_.stop();

        // Media Foundation 자원 해제
        if (pSourceReader_) pSourceReader_->Release();
        if (pSource_) pSource_->Release();

        MFShutdown();  // 🔹 모든 Media Foundation 객체 해제 후 호출
    }


private:
    CaptureConfig config_;
    TimestampLogger& logger_;
    std::thread capture_thread_;
    std::atomic<bool> is_running_;

    IMFMediaSource* pSource_;
    IMFSourceReader* pSourceReader_;

    FFmpegWriter writer_;
};

#endif // CAMERA_DEVICE_HPP
