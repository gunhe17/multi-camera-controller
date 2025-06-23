#pragma once

#include <array>
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <chrono>
#include <filesystem>
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

#include <comdef.h>
#include <wrl/client.h>
#include <wrl/implements.h>

#include <mfapi.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mfreadwrite.h>
#include <mferror.h>

#include "error/error.hpp"
#include "gonfig.hpp"

using namespace Microsoft::WRL;
using Microsoft::WRL::ComPtr;


/**
 * @Helper:  
 */

class COMDeleter {
public:
    template<typename T>
    void operator()(T* ptr) const {
        if (ptr) {
            CoTaskMemFree(ptr);
        }
    }
};
template<typename T>
using COMPtr = std::unique_ptr<T, COMDeleter>;


/**
 * @class: Buffer
 */

// TODO: on config?
struct BufferData {
    // sample
    ComPtr<IMFSample> sample_;
    // time
    std::chrono::system_clock::time_point sys_time_;
    LONGLONG mf_ts_;

    BufferData() {
        
    }

    BufferData(
        ComPtr<IMFSample> s, 
        std::chrono::system_clock::time_point sys_time,
        LONGLONG mf_ts
    ) {
        sample_ = std::move(s);
        sys_time_ = sys_time;
        mf_ts_ = mf_ts;
    }
};

constexpr std::size_t RING_BUFFER_SIZE = 1024;
template <typename T, std::size_t N>
class Buffer {
/** __init__ */
private:
    std::atomic<std::size_t> m_head;
    std::array<T, N> m_buff;
    std::atomic<std::size_t> m_tail;
public:
    constexpr Buffer() noexcept : m_head(0), m_tail(0) {}
    
/** methods */
public:
    // monitor
    constexpr std::size_t capacity() const noexcept { 
        return N;
    }
    std::size_t size() const noexcept {
        return m_tail.load(std::memory_order_acquire) - m_head.load(std::memory_order_acquire);
    }
    bool empty() const noexcept {
        return m_head.load(std::memory_order_acquire) == m_tail.load(std::memory_order_acquire);
    }

    // helper
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

        // TODO: 중단할 수 없는 오류
        return std::nullopt;
    }
};


/**
 * @class: MediaFoundation
 */

class MediaFoundation {
/** __init__ */
private:
    bool initialized_;

public:
    MediaFoundation() {
        HRESULT hr = MFStartup(MF_VERSION);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        initialized_ = true;
    }
    ~MediaFoundation() {
        if (initialized_) MFShutdown();
    }

/** methods */
public:
    ComPtr<IMFActivate> setupDevice(
        int camera_id
    ) {
        auto device = _createDevice(camera_id);

        return device;
    }

    ComPtr<IMFSourceReader> setupReader(
        ComPtr<IMFActivate> device,
        ComPtr<IMFSourceReaderCallback> callback
    ) {
        auto reader = _createSourceReader(device, callback);

        return reader;
    }

private:
    /**
     * Device
     */
    ComPtr<IMFActivate> _createDevice(int index) {
        HRESULT hr = S_OK;

        // every devices
        ComPtr<IMFAttributes> every_attributes;
        hr = MFCreateAttributes(&every_attributes, 1);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        hr = every_attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        IMFActivate** every_devices_raw = nullptr;
        UINT32 every_count = 0;

        hr = MFEnumDeviceSources(every_attributes.Get(), &every_devices_raw, &every_count);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        COMPtr<IMFActivate*> every_devices(every_devices_raw);


        // video devices
        ComPtr<IMFAttributes> video_attributes;

        hr = MFCreateAttributes(&video_attributes, 1);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));


        hr = video_attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));


        IMFActivate** video_devices_raw = nullptr;
        UINT32 video_count = 0;

        hr = MFEnumDeviceSources(video_attributes.Get(), &video_devices_raw, &video_count);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        COMPtr<IMFActivate*> video_devices(video_devices_raw);


        // check
        int camera_index = index;
        if (camera_index >= static_cast<int>(every_count)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

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
        if (!is_valid) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        return device;
    }
    
    /**
     * Reader
     */
    ComPtr<IMFSourceReader> _createSourceReader(
        ComPtr<IMFActivate> device, 
        ComPtr<IMFSourceReaderCallback> callback
    ) {
        HRESULT hr = S_OK;

        ComPtr<IMFMediaSource> pSource;
        ComPtr<IMFAttributes> pAttributes;
        ComPtr<IMFSourceReader> pSourceReader;
        ComPtr<IMFMediaType> pType;
        
        hr = device->ActivateObject(IID_PPV_ARGS(&pSource));
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        hr = MFCreateAttributes(&pAttributes, 1);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        hr = pAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, callback.Get());
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));
        
        hr = MFCreateSourceReaderFromMediaSource(pSource.Get(), pAttributes.Get(), &pSourceReader);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));
        
        hr = MFCreateMediaType(&pType);
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        pType->SetGUID(MF_MT_SUBTYPE, _format(gonfig.pixel_format));
        pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);

        MFSetAttributeSize(pType.Get(), MF_MT_FRAME_SIZE, gonfig.frame_width, gonfig.frame_height);
        MFSetAttributeRatio(pType.Get(), MF_MT_FRAME_RATE, gonfig.frame_rate, 1);

        hr = pSourceReader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, pType.Get());
        if (FAILED(hr)) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

        return pSourceReader;
    }
    GUID _format(const std::string& format) {
        if (format == "NV12")  return MFVideoFormat_NV12;
        if (format == "MJPG")  return MFVideoFormat_MJPG;
        if (format == "YUY2")  return MFVideoFormat_YUY2;
        if (format == "RGB32") return MFVideoFormat_RGB32;
        if (format == "I420")  return MFVideoFormat_I420;
        if (format == "UYVY")  return MFVideoFormat_UYVY;
        
        // TODO: 존재하지 않는 경우에 대한 오류
        throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));
    }
};

class Callback : public RuntimeClass<RuntimeClassFlags<ClassicCom>, IMFSourceReaderCallback> {
/** __init__ */
private:
    Buffer<BufferData, RING_BUFFER_SIZE>* buffer_;
    IMFSourceReader* reader_;
public:
    Callback() {}
    ~Callback() {}

/** methods */
public:
    // OnEvent
    STDMETHODIMP OnReadSample(
        HRESULT hrStatus, 
        DWORD streamIndex, 
        DWORD flags, 
        LONGLONG llTimestamp, 
        IMFSample* sample
    ) override {
        auto ts = std::chrono::system_clock::now();

        if (FAILED(hrStatus)) {
            std::cout << "OnReadSample failed with HR: " << std::hex << hrStatus << "\n";
            next();
            return S_OK;
        }
        
        if (!sample) {
            std::cout << "Sample is null\n";
            next();
            return S_OK;
        }

        buffer_->enqueue(
            BufferData(
                ComPtr<IMFSample>(sample), 
                std::chrono::system_clock::now(), 
                llTimestamp
            )
        );

        next();

        return S_OK;
    }
    STDMETHODIMP OnEvent(DWORD, IMFMediaEvent*) override { return S_OK; }
    STDMETHODIMP OnFlush(DWORD) override { return S_OK; }

    // helper
    void setupReader(IMFSourceReader* reader) { 
        reader_ = reader;
    }
    void setupBuffer(Buffer<BufferData, RING_BUFFER_SIZE>* buffer) {
        buffer_ = buffer;
    }
private:
    void next() {
        reader_->ReadSample(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, nullptr, nullptr, nullptr, nullptr
        );
    }
};


/**
 * @class: Maker
 */

class MediaMaker {
/** __init__ */
private:
    enum class State {
        STOPPED,
        STARTING,
        RUNNING,
        STOPPING
    };
    // monitor
    std::atomic<State> state_;
    std::atomic<bool> is_gate_open_;

    int image_index_ = 0;
    int camera_id_ = 0;

    // functional
    HANDLE read_handle_ = NULL;
    HANDLE write_handle_ = NULL;
    HANDLE ffmpeg_proc_ = NULL;
public:
    MediaMaker(int camera_id = 0) {
        camera_id_ = camera_id;
        state_ = State::STOPPED;
    }

    ~MediaMaker() { 
        stop(); 
    }

/** methods */
public:
    void warmup() {
        // status
        if (state_ != State::STOPPED) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));  
        state_ = State::STARTING;
        
        // run
        if (gonfig.output_mode == Config::OutputMode::VIDEO) {
            makeVideo();
        }
        if (gonfig.output_mode == Config::OutputMode::IMAGE) {
            makeImage();
        }

        // status
        state_ = State::RUNNING;
        is_gate_open_ = false;
    }

    void run() {
        is_gate_open_ = true; 
    }

    void stop() {
        state_ = State::STOPPING;
        is_gate_open_ = false;

        if (gonfig.output_mode == Config::OutputMode::VIDEO) {
            stopVideo();
        }
        if (gonfig.output_mode == Config::OutputMode::IMAGE) {
            stopImage();
        }
    }

    // unique
    void write(const BYTE* data, size_t size) {
        if (!is_gate_open_) return;

        if (gonfig.output_mode == Config::OutputMode::VIDEO) {
            writeVideo(data, size);
        }
        if (gonfig.output_mode == Config::OutputMode::IMAGE) {
            writeImage(data, size);
        }
    }
private:
    /**
     * Image
     */
    void makeImage() {
        image_index_ = 0;
    }

    // write
    void writeImage(const BYTE* data, size_t size) {
        std::ofstream file(_filename(), std::ios::binary);

        if (file.is_open()) {
            file.write(reinterpret_cast<const char*>(data), size);
            file.close();

            std::cout << image_index_ << "is created\n";
        }
    }

    std::string _filename() {
        std::ostringstream filename;

        filename << gonfig.output_directory << "/";
        filename << gonfig.output_filename << "_cam" << camera_id_;
        filename << "_" << std::setw(6) << std::setfill('0') << image_index_;
        filename << "." << gonfig.image_format;

        std::string name = filename.str();

        image_index_ += 1;

        return name;
    }

    // stop
    void stopImage() {
    }

    /**
     * Video
     */
    void makeVideo() {
        _createPipe();
        _createProcess();
    }

    // write
    void writeVideo(const BYTE* data, size_t size) {
        DWORD written = 0;

        WriteFile(
            write_handle_, data, static_cast<DWORD>(size), &written, NULL
        );
    }

    bool _createPipe() {
        SECURITY_ATTRIBUTES sa = { sizeof(SECURITY_ATTRIBUTES), NULL, TRUE };
        HANDLE readHandle = NULL;
        HANDLE writeHandle = NULL;

        bool pipe = CreatePipe(&readHandle, &writeHandle, &sa, 0);
        if (!pipe) {
            //pipe 생성 실패
            throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));
        }

        read_handle_ = readHandle;
        write_handle_ = writeHandle;

        return true;
    };

    bool _createProcess() {
        STARTUPINFOA si = {};
        PROCESS_INFORMATION pi{};
        std::string command = "" +
            gonfig.ffmpeg_path +
            " -loglevel verbose -y" +
            " -f mjpeg" +
            " -framerate " + std::to_string(gonfig.frame_rate) +
            " -i -" +
            " -c:v copy" +
            " -f avi" +
            " -avoid_negative_ts make_zero" +
            " \"" +
                gonfig.output_filename +
                "_cam" + std::to_string(camera_id_) +
                "." + gonfig.video_format + "\"";

        si.cb = sizeof(STARTUPINFOA);
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = read_handle_;
        si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
        si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
        
        BOOL process = CreateProcessA(NULL, const_cast<LPSTR>(command.c_str()), NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi);
        if (!process) {
            //process 생성 실패
            throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));
        }

        ffmpeg_proc_ = pi.hProcess;

        CloseHandle(pi.hThread);
        CloseHandle(read_handle_);

        return true;
    }

    // stop
    void stopVideo() {
        if (write_handle_) {
            CloseHandle(write_handle_);
            write_handle_ = NULL;
        }

        if (ffmpeg_proc_) {
            TerminateProcess(ffmpeg_proc_, 0);

            DWORD result = WaitForSingleObject(ffmpeg_proc_, 2000);

            CloseHandle(ffmpeg_proc_);
            ffmpeg_proc_ = NULL;
        }

        if (read_handle_) {
            CloseHandle(read_handle_);
            read_handle_ = NULL;
        }
    }
};

class CSVMaker {
/** __init__ */
private:
    enum class State {
        STOPPED,
        STARTING,
        RUNNING,
        STOPPING
    };
    // monitor
    std::atomic<State> state_;
    std::atomic<bool> is_gate_open_;

    int image_index_ = 0;
    int camera_id_ = 0;
    
    // file
    std::ofstream file_;
    std::mutex file_mutex_;
public:
    CSVMaker(int camera_id) {
        camera_id_ = camera_id; 
        state_ = State::STOPPED;
    }

    ~CSVMaker() { 
        stop(); 
    }

/** methods */
public:
    void warmup() {
        // status
        if (state_ != State::STOPPED) throw std::runtime_error("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));
        state_ = State::STARTING;
        
        // run
        std::string filename = "" +
            gonfig.output_directory +
            "/" +  gonfig.output_filename +
            "_camera_" + std::to_string(camera_id_) +
            ".csv";
        
        file_.open(filename);
        if (!file_.is_open()) throw std::runtime_error("CSV 파일 생성 실패: " + filename);
        
        file_ << "FrameIndex,Timestamp100ns,TimestampMs,WallClockTime,CameraID\n";

        // status
        state_ = State::RUNNING;
        is_gate_open_ = false;
    }

    void run() {
        is_gate_open_ = true;
    }

    void stop() {
        // monitor
        state_ = State::STOPPING;
        is_gate_open_ = false;

        std::lock_guard<std::mutex> lock(file_mutex_);
        if (file_.is_open()) file_.close();
    }

    // unique
    void write(const BufferData& sample_data) {
        std::cout << "[CSVMaker] Writing data for camera " << camera_id_ << " is gate open? " << is_gate_open_ << "\n";

        if (!is_gate_open_) return;

        std::lock_guard<std::mutex> lock(file_mutex_);

        std::cout << "[CSVMaker] Writing data for camera " << camera_id_ << " at index " << image_index_ << "\n";
        if (!file_.is_open()) return;

        image_index_++;

        double timestamp_ms = sample_data.mf_ts_ / 10000.0;
        std::string wall_clock_str = _wallclock(sample_data.sys_time_);

        file_ 
            << image_index_ << ","
            << sample_data.mf_ts_ << ","
            << std::fixed << std::setprecision(3) << timestamp_ms << ","
            << "\"" << wall_clock_str << "\","
            << camera_id_ << "\n";

        file_.flush();
    }
private:
    std::string _wallclock(const std::chrono::system_clock::time_point& tp) {
        std::time_t time_t = std::chrono::system_clock::to_time_t(tp);
        std::tm tm;
        localtime_s(&tm, &time_t);

        auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
            tp.time_since_epoch()) % 1000000;

        char buffer[64];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm);

        std::ostringstream oss;
        oss << buffer << "." << std::setw(6) << std::setfill('0') << microseconds.count();
        return oss.str();
    }
};


/**
 * @class: Manager
 */

class Sampler {
/** __init__ */
private:
    // class
    Buffer<BufferData, RING_BUFFER_SIZE>* buffer_;
    MediaMaker* media_maker_;
    CSVMaker* csv_maker_;

    // multiple
    std::thread processor_thread_;
    
    // monitor
    std::atomic<bool> running_flag_{false};
public:
    Sampler(
        Buffer<BufferData, RING_BUFFER_SIZE>* buffer, 
        MediaMaker* media_maker,
        CSVMaker* csv_maker
    ) {
        buffer_ = buffer;
        media_maker_ = media_maker;
        csv_maker_ = csv_maker;
    }
    
    ~Sampler() {
        stop();
    }

/** methods */
public:
    void warmup() {
        running_flag_ = true;

        processor_thread_ = std::thread([this]() {
            _loop();
        });
    }

    void stop() {
        running_flag_ = false;

        if (processor_thread_.joinable()) {
            processor_thread_.join();
        }
    }

public:
    void _loop() {
        while (running_flag_.load()) {
            auto inbuffer = buffer_->dequeue();
            if (!inbuffer) {
                std::this_thread::sleep_for(std::chrono::microseconds(gonfig.consumer_sleep_microseconds));
                continue;
            }

            // media maker
            auto [sample, size, mediaBuffer] = _frame(inbuffer->sample_);
            media_maker_->write(sample, size);

            // csv maker
            csv_maker_->write(*inbuffer);

            if (mediaBuffer) mediaBuffer->Unlock();
        }
    }

    std::tuple<BYTE*, DWORD, ComPtr<IMFMediaBuffer>> _frame(ComPtr<IMFSample> sample) {
        ComPtr<IMFMediaBuffer> buffer;
        DWORD maxLength = 0, currentLength = 0;
        BYTE* data = nullptr;

        HRESULT hr = sample->ConvertToContiguousBuffer(&buffer);
        if (FAILED(hr)) throw std::runtime_error("ConvertToContiguousBuffer failed");

        hr = buffer->Lock(&data, &maxLength, &currentLength);
        if (FAILED(hr)) throw std::runtime_error("Lock failed");

        return { data, currentLength, buffer };
    }
};

class Manager {
/** __init__ */
private:
    // identifier
    int camera_id_;

    // class
    std::unique_ptr<MediaFoundation> mf_;
    ComPtr<IMFActivate> device_;
    ComPtr<Callback> callback_;
    ComPtr<IMFSourceReader> reader_;

    std::unique_ptr<Buffer<BufferData, RING_BUFFER_SIZE>> buffer_;

    std::unique_ptr<MediaMaker> mediaMaker_;
    std::unique_ptr<CSVMaker> csvMaker_;
    
    std::unique_ptr<Sampler> sampler_;
public:
    Manager(
        int camera_id
    ) {
        // identifier
        camera_id_ = camera_id;

        // class
        mf_ = std::make_unique<MediaFoundation>();
        device_ = ComPtr<IMFActivate>();
        callback_ = Microsoft::WRL::Make<Callback>();
        reader_ = ComPtr<IMFSourceReader>();

        buffer_ = std::make_unique<Buffer<BufferData, RING_BUFFER_SIZE>>();

        mediaMaker_ = std::make_unique<MediaMaker>(camera_id_);
        csvMaker_ = std::make_unique<CSVMaker>(camera_id_);
        
        sampler_ = std::make_unique<Sampler>(
            buffer_.get(),
            mediaMaker_.get(),
            csvMaker_.get()
        );
    }

    ~Manager() {
    }

/** methods */
public:
    void init() {
        // empty
    }

    void setup() {
        // set media foundation
        device_ = mf_->setupDevice(camera_id_);
        reader_ = mf_->setupReader(device_, callback_);

        // set callback
        callback_->setupReader(reader_.Get());
        callback_->setupBuffer(buffer_.get());
    }

    void warmup() {
        // media foundation
        reader_->ReadSample(
            MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, nullptr, nullptr, nullptr, nullptr
        );

        // sampler
        sampler_->warmup();

        // media maker
        mediaMaker_->warmup();

        // csv maker
        csvMaker_->warmup();

        // warmup
        std::this_thread::sleep_for(std::chrono::seconds(gonfig.warmup_duration));
        
        std::cout << "[Manager " << camera_id_ << "] warmup done\n";
    }

    void run() {
        // media maker
        mediaMaker_->run();

        // csv maker
        csvMaker_->run();
    }

    void stop() {
        // sampler
        sampler_->stop();

        // media maker
        mediaMaker_->stop();

        // log maker
        csvMaker_->stop();

        std::cout << "[Manager " << camera_id_ << "] cleanup done\n";
    }
private:
    void _sampleLoop() {
        while (true) {
            auto inbuffer = buffer_->dequeue();
            if (!inbuffer) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(gonfig.consumer_sleep_microseconds)
                );
                continue;
            }

            auto [data, size, mediaBuffer] = _createFrame(inbuffer->sample_);

            mediaMaker_->write(data, size);

            // clean up
            if (mediaBuffer) mediaBuffer->Unlock();
        }
    }
    std::tuple<BYTE*, DWORD, ComPtr<IMFMediaBuffer>> _createFrame(ComPtr<IMFSample> sample) {
        ComPtr<IMFMediaBuffer> buffer;
        DWORD maxLength = 0, currentLength = 0;
        BYTE* data = nullptr;

        HRESULT hr = sample->ConvertToContiguousBuffer(&buffer);
        if (FAILED(hr)) throw std::runtime_error("ConvertToContiguousBuffer failed at " __FILE__ ":" + std::to_string(__LINE__));

        hr = buffer->Lock(&data, &maxLength, &currentLength);
        if (FAILED(hr)) throw std::runtime_error("Lock failed at " __FILE__ ":" + std::to_string(__LINE__));

        return { data, currentLength, buffer };
    }
};


class Barrier {
/** __init__ */
private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::size_t count_;
    std::size_t waiting_;
    std::size_t generation_;
public:
    explicit Barrier(
        std::size_t count
    ): count_(count), waiting_(0), generation_(0) {}

/** methods */
public:
    void arrive_and_wait() {
        std::unique_lock<std::mutex> lock(mutex_);
        std::size_t current_generation = generation_;

        ++waiting_;
        
        if (waiting_ == count_) {
            waiting_ = 0;
            ++generation_;
            cv_.notify_all();
        } else {
            cv_.wait(lock, [this, current_generation] {
                return generation_ != current_generation;
            });
        }
    }
};

class Timer {
/** __init__ */
    public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;

/** methods */
public:
    void microSl(uint64_t us) const {
        _preciseSleep(std::chrono::microseconds(us));
    }

    void milliSl(uint64_t ms) const {
        _preciseSleep(std::chrono::milliseconds(ms));
    }

private:
    static void _preciseSleep(std::chrono::microseconds duration) {
        auto start = Clock::now();
        auto end = start + duration;
        
        if (duration > std::chrono::milliseconds(2)) {
            auto sleep_until = end - std::chrono::milliseconds(1);
            std::this_thread::sleep_until(sleep_until);
        }
        
        while (Clock::now() < end) {
            std::this_thread::yield();
        }
    }
};

class MultiManager {
/** __init__ */
private:
    enum Stage {
        INIT = 0, 
        SETUP, 
        WARMUP, 
        RUN,
        END,
        NUM_STAGES
    };
    
    // multiple
    std::vector<std::unique_ptr<Manager>> managers_;
    std::vector<std::unique_ptr<Barrier>> barriers_;
    std::vector<std::thread> threads_;
    
    // helper
    Timer timer_;

    // state
    std::atomic<bool> record_signal_{false};
public:
    MultiManager() {
        managers_.clear();
        barriers_.clear();
        threads_.clear();
    }

/** methods */
public:
    // common
    void setup() {
        // manager
        for (int index : gonfig.camera_indices) {
            _manager(index);
        }

        // barrier
        for (int i = 0; i < NUM_STAGES; ++i) {
            _barrier();
        }
    }

    void run() {        
        for (size_t i = 0; i < managers_.size(); ++i) {
            threads_.emplace_back([this, i]() {
                try {
                    auto& manager = *managers_[i];

                    /**
                     * RUN
                     */

                    // 1st
                    barriers_[INIT]->arrive_and_wait();
                    manager.init();

                    // 2nd
                    barriers_[SETUP]->arrive_and_wait();
                    manager.setup();

                    // 3rd
                    barriers_[WARMUP]->arrive_and_wait();
                    manager.warmup();

                    // 4th
                    barriers_[RUN]->arrive_and_wait();
                    while (!record_signal_.load()) timer_.microSl(10);
                    
                    manager.run();

                    // 5th
                    barriers_[END]->arrive_and_wait();
                    manager.stop();

                } catch (const std::exception& e) {
                    std::cout << "[Thread Exception] " << e.what() << std::endl;
                };
            });
        }
        
        std::thread coordinator_thread([this]() {
            _coordinate();
        });
        
        coordinator_thread.join();
        for (auto& thread : threads_) {
            thread.join();
        }
    }

private:
    // Setup
    void _barrier() {
        auto barrier = std::make_unique<Barrier>(managers_.size()+1);

        barriers_.emplace_back(std::move(barrier));
    }

    void _manager(int index) {
        auto manager = std::make_unique<Manager>(index);

        managers_.push_back(std::move(manager));
    }
    
    // Run
    void _coordinate() {
        barriers_[INIT]->arrive_and_wait();

        barriers_[SETUP]->arrive_and_wait();

        barriers_[WARMUP]->arrive_and_wait(); 

        barriers_[RUN]->arrive_and_wait();
        _run();

        barriers_[END]->arrive_and_wait();
    }

    // run
    void _run() {
        auto start = Timer::Clock::now();
        auto end = start + std::chrono::seconds(gonfig.record_duration);
        
        // record start
        record_signal_.store(true);

        while (Timer::Clock::now() < end) {
            auto remaining = end - Timer::Clock::now();
            
            if (remaining > std::chrono::milliseconds(10)) {
                timer_.milliSl(5);
            } else {
                timer_.microSl(100);
            }
        }

        // record end
        record_signal_.store(false);

        auto actual_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            Timer::Clock::now() - start
        ).count();
        
        // log
        std::cout << "[MultiManager] 정확한 녹화 시간: " 
            << actual_duration << "μs (" 
            << std::fixed << std::setprecision(6) 
            << actual_duration / 1000000.0 << "초)\n";
    }
};


/**
 *  Main
 */

// GLOBAL
Config gonfig;

int main(int argc, char* argv[]) {
    // Windows COM
    CoInitialize(NULL);

    // Run
    try{
        // call g(lobal c)onfig
        gonfig = ConfigManager().config(argc, argv);

        // call Multi
        MultiManager multiManager;
        
        multiManager.setup();

        multiManager.run();

    } catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    
    // Windows COM
    CoUninitialize();
}