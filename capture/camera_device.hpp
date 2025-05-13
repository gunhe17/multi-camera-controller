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
#include <iomanip>
#include <condition_variable>
#include <mutex>

#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "ole32.lib")

class CameraDevice {
public:
    CameraDevice(const CaptureConfig& config,
                 TimestampLogger& logger,
                 std::shared_ptr<std::condition_variable> cv,
                 std::shared_ptr<std::mutex> mutex,
                 std::shared_ptr<bool> go_flag)
        : config_(config),
          logger_(logger),
          sync_cv_(cv),
          sync_mutex_(mutex),
          sync_go_flag_(go_flag),
          is_running_(false),
          pSource_(nullptr),
          pSourceReader_(nullptr),
          writer_(config) {}

    bool initialize() {
        HRESULT hr = MFStartup(MF_VERSION);
        if (FAILED(hr)) {
            std::cerr << "MFStartup 실패\n";
            return false;
        }

        IMFAttributes* pAttributes = nullptr;
        hr = MFCreateAttributes(&pAttributes, 1);
        if (FAILED(hr)) return false;

        hr = pAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                                  MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (FAILED(hr)) return false;

        IMFActivate** ppDevices = nullptr;
        UINT32 count = 0;
        hr = MFEnumDeviceSources(pAttributes, &ppDevices, &count);
        if (FAILED(hr) || config_.camera_id >= static_cast<int>(count)) return false;

        hr = ppDevices[config_.camera_id]->ActivateObject(IID_PPV_ARGS(&pSource_));
        if (FAILED(hr)) return false;

        hr = MFCreateSourceReaderFromMediaSource(pSource_, nullptr, &pSourceReader_);
        if (FAILED(hr)) return false;

        // Media Type 설정: YUY2
        IMFMediaType* pType = nullptr;
        hr = MFCreateMediaType(&pType);
        if (FAILED(hr)) return false;

        pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_YUY2);
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

    void start() {
        is_running_ = true;

        capture_thread_ = std::thread([this]() {
            // 동기화 대기
            {
                std::unique_lock<std::mutex> lock(*sync_mutex_);
                sync_cv_->wait(lock, [this]() { return *sync_go_flag_; });
            }

            auto thread_now = std::chrono::steady_clock::now();
            auto thread_us = std::chrono::duration_cast<std::chrono::microseconds>(
                thread_now.time_since_epoch()).count();

            std::cout << "[thread] Camera " << config_.camera_id
                      << " @ " << (thread_us / 1000000) << "."
                      << std::setw(6) << std::setfill('0') << (thread_us % 1000000)
                      << " sec (thread started)\n";

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
                    frame_index++,
                    static_cast<uint64_t>(sampleTime * 100),
                    rel_us,
                    config_.camera_id
                };
                logger_.log(meta);

                IMFMediaBuffer* pBuffer = nullptr;
                BYTE* pData = nullptr;
                DWORD maxLength = 0, currentLength = 0;

                hr = pSample->ConvertToContiguousBuffer(&pBuffer);
                if (SUCCEEDED(hr)) {
                    hr = pBuffer->Lock(&pData, &maxLength, &currentLength);
                    if (SUCCEEDED(hr)) {
                        writer_.write(pData, currentLength);
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
        if (capture_thread_.joinable()) capture_thread_.join();

        writer_.stop();

        if (pSourceReader_) pSourceReader_->Release();
        if (pSource_) pSource_->Release();
        MFShutdown();
    }

private:
    CaptureConfig config_;
    TimestampLogger& logger_;
    std::thread capture_thread_;
    std::atomic<bool> is_running_;

    IMFMediaSource* pSource_;
    IMFSourceReader* pSourceReader_;
    FFmpegWriter writer_;

    std::shared_ptr<std::condition_variable> sync_cv_;
    std::shared_ptr<std::mutex> sync_mutex_;
    std::shared_ptr<bool> sync_go_flag_;
};

#endif // CAMERA_DEVICE_HPP
