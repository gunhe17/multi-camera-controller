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



/*******************
 * 
 *      FFmpeg
 * 
 *******************/

/**
 * Class: FFmpeg
 */
class FFmpeg {
public:
    bool start() {
        SECURITY_ATTRIBUTES sa = { sizeof(SECURITY_ATTRIBUTES), NULL, TRUE };
        HANDLE readHandle = NULL, writeHandle = NULL;

        if (!CreatePipe(&readHandle, &writeHandle, &sa, 0)) {
            std::cerr << "[FFmpeg] 파이프 생성 실패\n";
            return false;
        }

        STARTUPINFOA si = { 0 };
        si.cb = sizeof(STARTUPINFOA);
        si.dwFlags |= STARTF_USESTDHANDLES;
        si.hStdInput = readHandle;
        si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
        si.hStdError = GetStdHandle(STD_ERROR_HANDLE);


        PROCESS_INFORMATION pi = { 0 };
        
        std::string command = buildCommand();
        BOOL success = CreateProcessA(
            NULL,
            const_cast<LPSTR>(command.c_str()),
            NULL, NULL, TRUE, 0, NULL, NULL,
            &si, &pi
        );
        if (!success) {
            std::cerr << "FFmpeg 실행 실패\n";
            CloseHandle(readHandle);
            CloseHandle(writeHandle);
            return false;
        }

        CloseHandle(readHandle);  // FFmpeg가 사용하는 읽기 핸들 닫기

        ffmpeg_proc_ = pi.hProcess;
        ffmpeg_stdin_ = writeHandle;
        
        CloseHandle(pi.hThread);  // 쓰레드 핸들은 사용하지 않음
        return true;
    }

    bool write(const BYTE* data, size_t length) {
        if (!ffmpeg_stdin_) return false;

        DWORD written = 0;
        return WriteFile(ffmpeg_stdin_, data, static_cast<DWORD>(length), &written, NULL);
    }

    void stop() {
        std::cout << "[FFmpeg] 종료 시도 중...\n";

        if (ffmpeg_stdin_) {
            std::cout << "[FFmpeg] stdin 파이프 닫는 중...\n";
            CloseHandle(ffmpeg_stdin_);
            ffmpeg_stdin_ = NULL;
            std::cout << "[FFmpeg] stdin 파이프 닫힘\n";
        } else {
            std::cout << "[FFmpeg] stdin 파이프는 이미 닫혀 있음\n";
        }

        if (ffmpeg_proc_) {
            std::cout << "[FFmpeg] FFmpeg 프로세스 종료 대기 중...\n";
            DWORD result = WaitForSingleObject(ffmpeg_proc_, INFINITE);
            std::cout << "[FFmpeg] FFmpeg 프로세스 종료 감지 중 ...\n";

            if (result == WAIT_OBJECT_0) {
                std::cout << "[FFmpeg] FFmpeg 프로세스 종료 감지됨\n";
            } else {
                std::cerr << "[FFmpeg] FFmpeg 프로세스 종료 대기 실패 (코드: " << result << ")\n";
            }

            CloseHandle(ffmpeg_proc_);
            ffmpeg_proc_ = NULL;
            std::cout << "[FFmpeg] FFmpeg 프로세스 핸들 닫힘\n";
        } else {
            std::cout << "[FFmpeg] FFmpeg 프로세스 핸들은 이미 닫혀 있음\n";
        }

        std::cout << "[FFmpeg] 종료 완료\n";
    }

    // helper
    void setConfig(const Config& config) {
        config_ = config;
    }

private:
    std::string FFmpeg::buildCommand() const {
        return config_.ffmpeg +
            " -loglevel verbose -report -y" +
            " -f mjpeg" +
            " -framerate " + std::to_string(config_.frame_rate) +
            " -i -" +
            " -c:v copy" +
            " \"" + config_.output + "\"";
    }
    Config config_;
    HANDLE ffmpeg_stdin_ = NULL;
    HANDLE ffmpeg_proc_ = NULL;
};


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
        }

        if (external_running_flag_ && !(*external_running_flag_)) {
            std::cout << "[Callback] 종료 상태 감지, ReadSample 중지\n";
            return S_OK;
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

    void setFFmpeg(FFmpeg* ffmpeg) {
        ffmpeg_ = ffmpeg;
    }

    void setRunningFlag(std::atomic<bool>* flag) {
        external_running_flag_ = flag;
    }

private:
    IMFSourceReader* reader_ = nullptr;
    SPSCRingBuffer<ComPtr<IMFSample>, 1024>* ringBuffer_ = nullptr;
    FFmpeg* ffmpeg_ = nullptr;

    std::atomic<bool>* external_running_flag_ = nullptr;
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

    /**
     *  FFmpeg
     */

    FFmpeg ffmpeg;

    ffmpeg.setConfig(config);
    if (!ffmpeg.start()) {
        std::cerr << "[Main] FFmpeg 실행 실패\n";
        return -1;
    }

    callback.value()->setFFmpeg(&ffmpeg);

    std::atomic<bool> running = true;
    std::thread consumer_thread([&]() {
        while (running) {
            auto sample_opt = ringBuffer.dequeue();
            if (sample_opt) {
                ComPtr<IMFMediaBuffer> buffer;
                DWORD maxLength = 0, currentLength = 0;
                BYTE* data = nullptr;

                if (SUCCEEDED(sample_opt.value()->ConvertToContiguousBuffer(&buffer)) &&
                    SUCCEEDED(buffer->Lock(&data, &maxLength, &currentLength))) {

                    if (!ffmpeg.write(data, currentLength)) {
                        std::cerr << "[Consumer] FFmpeg write 실패, 쓰레드 종료\n";
                        break;
                    }

                    buffer->Unlock();
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    });


    // 30초 대기 후 종료
    std::this_thread::sleep_for(std::chrono::seconds(config.duration));

    // 1. Read 중단
    source_reader.value()->Flush(MF_SOURCE_READER_FIRST_VIDEO_STREAM);

    // 2. Consumer 쓰레드 종료
    running = false;
    consumer_thread.join();

    // 3. stdin 파이프 닫기 → 반드시 이 순서
    ffmpeg.stop();  // 내부에서 CloseHandle(ffmpeg_stdin_) 호출됨

    MFShutdown();            // Media Foundation 종료
}