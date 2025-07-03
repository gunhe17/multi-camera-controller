#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <iomanip>
#include <filesystem>
#include <sstream>
#include <memory>
#include <string>
#include <exception>
#include <array>
#include <algorithm>
#include <mutex>
#include <optional>
#include <condition_variable>

#include <comdef.h>
#include <wrl/client.h>
#include <wrl/implements.h>

#include <mfapi.h>
#include <mfidl.h>
#include <mfobjects.h>
#include <mfreadwrite.h>
#include <mferror.h>

// Tobii Research API headers
#include "tobii_research.h"
#include "tobii_research_eyetracker.h"
#include "tobii_research_streams.h"

#include "error/error.hpp"
#include "gonfig.hpp"

using namespace Microsoft::WRL;
using Microsoft::WRL::ComPtr;

// Windows specific
#if _WIN32 || _WIN64
#include <windows.h>
static void sleep_ms(int time) {
    Sleep(time);
}
#else
#include <unistd.h>
static void sleep_ms(int time) {
    usleep(time * 1000);
}
#endif


/**
 * @Exception
 */

class ExceptionHandler {
public:
    static void setupTerminateHandler() {
        std::set_terminate([]() {
            std::cout << "[FATAL] Unhandled exception occurred! Program terminating.\n";
            try {
                auto eptr = std::current_exception();
                if (eptr) {
                    std::rethrow_exception(eptr);
                }
            } catch (const std::exception& e) {
                std::cout << "[FATAL] Exception: " << e.what() << "\n";
            } catch (...) {
                std::cout << "[FATAL] Unknown exception type\n";
            }
            std::abort();
        });
    }
};

class TobiiManagerError : public Error {
public:
    TobiiManagerError(const std::string& msg, int code = 2000)
        : Error("TobiiManagerError", msg, code) {}
};

class TobiiRecorderError : public Error {
public:
    TobiiRecorderError(const std::string& msg, int code = 2100)
        : Error("TobiiRecorderError", msg, code) {}
};

class TobiiMakerError : public Error {
public:
    TobiiMakerError(const std::string& msg, int code = 2200)
        : Error("TobiiMakerError", msg, code) {}
};


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


// ==========================================
// Camera Manager
// ==========================================

/**
 * @class: Buffer
 */

struct BufferData {
    // sample
    ComPtr<IMFSample> sample_;
    // time
    std::chrono::system_clock::time_point sys_time_;
    LONGLONG mf_ts_;

    BufferData() {}

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
    bool enqueue(T val) noexcept {
        std::size_t current_tail = m_tail.load(std::memory_order_relaxed);
        if (current_tail - m_head.load(std::memory_order_acquire) < N) {
            m_buff[current_tail % N] = std::move(val);
            m_tail.store(current_tail + 1, std::memory_order_release);
            return true;
        }

        // is failed
        std::cout << "[Buffer Warning] Buffer가 overflow 되었습니다.\n";
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

        // is failed
        // std::cout << "[Buffer Warning] Buffer가 underflow 되었습니다.\n";
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
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

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
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        hr = every_attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        IMFActivate** every_devices_raw = nullptr;
        UINT32 every_count = 0;

        hr = MFEnumDeviceSources(every_attributes.Get(), &every_devices_raw, &every_count);
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        COMPtr<IMFActivate*> every_devices(every_devices_raw);


        // video devices
        ComPtr<IMFAttributes> video_attributes;

        hr = MFCreateAttributes(&video_attributes, 1);
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");


        hr = video_attributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");


        IMFActivate** video_devices_raw = nullptr;
        UINT32 video_count = 0;

        hr = MFEnumDeviceSources(video_attributes.Get(), &video_devices_raw, &video_count);
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        COMPtr<IMFActivate*> video_devices(video_devices_raw);


        // check
        int camera_index = index;
        if (camera_index >= static_cast<int>(every_count)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

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
        if (!is_valid) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));

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
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        hr = MFCreateAttributes(&pAttributes, 1);
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        hr = pAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, callback.Get());
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        hr = MFCreateSourceReaderFromMediaSource(pSource.Get(), pAttributes.Get(), &pSourceReader);
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        hr = MFCreateMediaType(&pType);
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        pType->SetGUID(MF_MT_SUBTYPE, _format(gonfig.pixel_format));
        pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);

        MFSetAttributeSize(pType.Get(), MF_MT_FRAME_SIZE, gonfig.frame_width, gonfig.frame_height);
        MFSetAttributeRatio(pType.Get(), MF_MT_FRAME_RATE, gonfig.frame_rate, 1);

        hr = pSourceReader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, pType.Get());
        if (FAILED(hr)) throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

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
        throw MediaFoundationError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));
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
            std::cout << "[Callback Warning] Sample을 읽을 수 없습니다. (" << std::hex << hrStatus << std::dec << ")\n";
            next();
            return S_OK;
        }
        
        if (!sample) {
            std::cout << "[Callback Warning] Sample이 비어있습니다.\n";
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
        if (state_ != State::STOPPED) throw MediaMakerError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));  
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

        bool is_open = file.is_open();

        if (!is_open) {
            std::cout << "[MediaMaker Warning] Image file open failed for camera " << camera_id_ << ": " << _filename() << "\n";
            return;
        }

        file.write(reinterpret_cast<const char*>(data), size);
        
        if (file.fail()) {
            std::cout << "[MediaMaker Warning] Image write failed for camera " << camera_id_ << " at index " << image_index_ << "\n";
        } else {
            std::cout << "[MediaMaker] Image " << image_index_ << " created for camera " << camera_id_ << "\n";
        }

        file.close();
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

        BOOL result = WriteFile(
            write_handle_, data, static_cast<DWORD>(size), &written, NULL
        );
        if (!result || written != size) {
            std::cout << "[MediaMaker Warning] Video write failed for camera " << camera_id_ << ", expected: " << size << ", written: " << written << "\n";
        }
    }

    bool _createPipe() {
        SECURITY_ATTRIBUTES sa = { sizeof(SECURITY_ATTRIBUTES), NULL, TRUE };
        HANDLE readHandle = NULL;
        HANDLE writeHandle = NULL;

        bool pipe = CreatePipe(&readHandle, &writeHandle, &sa, 0);
        if (!pipe) {
            throw MediaMakerError("Pipe creation failed", GetLastError());
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
            throw MediaMakerError("FFmpeg process creation failed", GetLastError());
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
        if (state_ != State::STOPPED) {
            throw CSVMakerError("Invalid state: not SETTING_UP at " __FILE__ ":" + std::to_string(__LINE__));
        }
        state_ = State::STARTING;
        
        // run
        std::string filename = "" +
            gonfig.output_directory +
            "/" +  gonfig.output_filename +
            "_camera_" + std::to_string(camera_id_) +
            ".csv";
        
        file_.open(filename);
        if (!file_.is_open()) throw CSVMakerError("CSV 파일 생성 실패: " + filename);
        
        file_ << "FrameIndex,Timestamp100ns,TimestampMs,WallClockTime,CameraID\n";
        if (file_.fail()) throw CSVMakerError("CSV 헤더 쓰기 실패");

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
        if (!is_gate_open_) return;

        std::lock_guard<std::mutex> lock(file_mutex_);

        if (!file_.is_open()) {
            std::cout << "[CSVMaker Warning] File not open for camera " << camera_id_ << "\n";
            return;
        }

        double timestamp_ms = sample_data.mf_ts_ / 10000.0;
        std::string wall_clock_str = _wallclock(sample_data.sys_time_);

        file_ 
            << image_index_ << ","
            << sample_data.mf_ts_ << ","
            << std::fixed << std::setprecision(3) << timestamp_ms << ","
            << "\"" << wall_clock_str << "\","
            << camera_id_ << "\n";

        file_.flush();

        image_index_++;
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
                // buffer의 dequeue에서 exception 처리
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

        HRESULT hr;

        hr = sample->ConvertToContiguousBuffer(&buffer);
        if (FAILED(hr)) throw SamplerError("ConvertToContiguousBuffer failed at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");
        
        hr = buffer->Lock(&data, &maxLength, &currentLength);
        if (FAILED(hr)) throw SamplerError("Lock failed at " __FILE__ ":" + std::to_string(__LINE__) + "\n" + std::to_string(hr) + "\n");

        return { data, currentLength, buffer };
    }
};

class CamManager {
/** __init__ */
private:
    int camera_id_;
    
    std::unique_ptr<MediaFoundation> mf_;
    ComPtr<IMFActivate> device_;
    ComPtr<Callback> callback_;
    ComPtr<IMFSourceReader> reader_;
    std::unique_ptr<Buffer<BufferData, RING_BUFFER_SIZE>> buffer_;
    std::unique_ptr<MediaMaker> mediaMaker_;
    std::unique_ptr<CSVMaker> csvMaker_;
    std::unique_ptr<Sampler> sampler_;

public:
    CamManager(int camera_id) {
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
    };
    ~CamManager() = default;

/** methods */
public:
    void init() {
        std::cout << "[CamManager " << camera_id_ << "] init done\n";
    }

    void setup() {
        std::cout << "[CamManager " << camera_id_ << "] Starting setup...\n";
        try {
            device_ = mf_->setupDevice(camera_id_);
            reader_ = mf_->setupReader(device_, callback_);
            callback_->setupReader(reader_.Get());
            callback_->setupBuffer(buffer_.get());
            std::cout << "[CamManager " << camera_id_ << "] Setup completed\n";
        } catch (const std::exception& e) {
            std::cout << "[CamManager " << camera_id_ << "] Setup failed: " << e.what() << "\n";
            throw;
        }
    }

    void warmup() {
        std::cout << "[CamManager " << camera_id_ << "] Starting warmup...\n";
        try {
            reader_->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, nullptr, nullptr, nullptr, nullptr);
            sampler_->warmup();
            mediaMaker_->warmup();
            csvMaker_->warmup();
            std::this_thread::sleep_for(std::chrono::seconds(gonfig.warmup_duration));
            std::cout << "[CamManager " << camera_id_ << "] warmup done\n";
        } catch (const std::exception& e) {
            std::cout << "[CamManager " << camera_id_ << "] Warmup failed: " << e.what() << "\n";
            throw;
        }
    }

    void run() {
        std::cout << "[CamManager " << camera_id_ << "] Starting run...\n";
        try {
            mediaMaker_->run();
            csvMaker_->run();
            std::cout << "[CamManager " << camera_id_ << "] Run started\n";
        } catch (const std::exception& e) {
            std::cout << "[CamManager " << camera_id_ << "] Run failed: " << e.what() << "\n";
            throw;
        }
    }

    void stop() {
        std::cout << "[CamManager " << camera_id_ << "] Stopping...\n";
        try {
            sampler_->stop();
            mediaMaker_->stop();
            csvMaker_->stop();
            std::cout << "[CamManager " << camera_id_ << "] cleanup done\n";
        } catch (const std::exception& e) {
            std::cout << "[CamManager " << camera_id_ << "] Stop failed: " << e.what() << "\n";
        }
    }

    int getCameraId() const { return camera_id_; }
};


// ==========================================
// Tobii Manager
// ==========================================

struct TobiiGazeData {
    std::chrono::system_clock::time_point timestamp_;
    int64_t device_timestamp_;
    
    struct EyeGazePoint {
        double x, y;
        bool validity;
        double pupil_diameter;
    } left_eye_, right_eye_;
    
    struct EyePosition3D {
        double x, y, z;
        bool validity;
    } left_eye_3d_, right_eye_3d_;
};

class TobiiDevice {
/** __init__ */
private:
    TobiiResearchEyeTracker* eye_tracker_;
    std::string address_;
    std::string serial_number_;

public:
    TobiiDevice() : eye_tracker_(nullptr) {}

/** methods */
public:
    void setup() {
        std::cout << "[TobiiDevice] Starting setup...\n";
        try {
            _findEyeTrackers();
            _setupDeviceInfo();
            std::cout << "[TobiiDevice] Setup completed\n";
        } catch (const std::exception& e) {
            std::cout << "[TobiiDevice] Setup failed: " << e.what() << "\n";
            throw;
        }
    }

    void warmup() {
        std::cout << "[TobiiDevice] warmup done\n";
    }

    void stop() {
        std::cout << "[TobiiDevice] stopped\n";
    }

    TobiiResearchEyeTracker* getEyeTracker() const { return eye_tracker_; }

private:
    void _findEyeTrackers() {
        TobiiResearchEyeTrackers* eyetrackers = nullptr;
        TobiiResearchStatus status = tobii_research_find_all_eyetrackers(&eyetrackers);
        
        if (status != TOBII_RESEARCH_STATUS_OK) {
            throw TobiiManagerError("Failed to find eye trackers. Status: " + std::to_string(status), 2003);
        }
        
        if (eyetrackers->count == 0) {
            tobii_research_free_eyetrackers(eyetrackers);
            throw TobiiManagerError("No eye trackers found", 2004);
        }
        
        std::cout << "[TobiiDevice] Found " << eyetrackers->count << " eye tracker(s)\n";
        eye_tracker_ = eyetrackers->eyetrackers[0];
        tobii_research_free_eyetrackers(eyetrackers);
    }

    void _setupDeviceInfo() {
        char* address = nullptr;
        char* serial_number = nullptr;
        char* device_name = nullptr;
        char* model = nullptr;
        
        tobii_research_get_address(eye_tracker_, &address);
        tobii_research_get_serial_number(eye_tracker_, &serial_number);
        tobii_research_get_device_name(eye_tracker_, &device_name);
        tobii_research_get_model(eye_tracker_, &model);
        
        std::cout << "[TobiiDevice] Device Info:\n";
        std::cout << "  Address: " << (address ? address : "N/A") << "\n";
        std::cout << "  Model: " << (model ? model : "N/A") << "\n";
        std::cout << "  Serial: " << (serial_number ? serial_number : "N/A") << "\n";
        std::cout << "  Name: " << (device_name ? device_name : "N/A") << "\n";
        
        address_ = address ? address : "";
        serial_number_ = serial_number ? serial_number : "";
        
        tobii_research_free_string(address);
        tobii_research_free_string(serial_number);
        tobii_research_free_string(device_name);
        tobii_research_free_string(model);
    }
};

class TobiiRecorder {
/** __init__ */
private:
    int recorder_id_;
    std::atomic<bool> is_recording_;
    TobiiResearchEyeTracker* eye_tracker_;
    std::atomic<int> gaze_count_;
    std::atomic<int> left_eye_image_count_;
    std::atomic<int> right_eye_image_count_;

public:
    TobiiRecorder(int recorder_id) : recorder_id_(recorder_id), is_recording_(false), 
                                     eye_tracker_(nullptr), gaze_count_(0), 
                                     left_eye_image_count_(0), right_eye_image_count_(0) {}

/** methods */
public:
    void setup(TobiiResearchEyeTracker* eye_tracker) {
        std::cout << "[TobiiRecorder " << recorder_id_ << "] Starting setup...\n";
        if (!eye_tracker) {
            throw TobiiRecorderError("Eye tracker is null", 2104);
        }
        
        try {
            eye_tracker_ = eye_tracker;
            _subscribeToStreams();
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Setup completed\n";
        } catch (const std::exception& e) {
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Setup failed: " << e.what() << "\n";
            throw;
        }
    }

    void warmup() {
        std::cout << "[TobiiRecorder " << recorder_id_ << "] warmup done\n";
    }

    void run() {
        is_recording_ = true;
        std::cout << "[TobiiRecorder " << recorder_id_ << "] recording started\n";
    }

    void stop() {
        is_recording_ = false;
        if (eye_tracker_) {
            _unsubscribeFromStreams();
        }
        std::cout << "[TobiiRecorder " << recorder_id_ << "] stopped\n";
    }

    int getGazeCount() const { return gaze_count_.load(); }
    int getLeftEyeImageCount() const { return left_eye_image_count_.load(); }
    int getRightEyeImageCount() const { return right_eye_image_count_.load(); }

private:
    void _subscribeToStreams() {
        TobiiResearchStatus status;
        
        // Subscribe to gaze data
        status = tobii_research_subscribe_to_gaze_data(eye_tracker_, 
            [](TobiiResearchGazeData* gaze_data, void* user_data) {
                auto* recorder = static_cast<TobiiRecorder*>(user_data);
                recorder->_gazeDataCallback(gaze_data);
            }, this);
        
        if (status != TOBII_RESEARCH_STATUS_OK) {
            throw TobiiRecorderError("Failed to subscribe to gaze data. Status: " + std::to_string(status), 2102);
        }
        
        // Subscribe to eye images
        status = tobii_research_subscribe_to_eye_image(eye_tracker_, 
            [](TobiiResearchEyeImage* eye_image, void* user_data) {
                auto* recorder = static_cast<TobiiRecorder*>(user_data);
                recorder->_eyeImageCallback(eye_image);
            }, this);
        
        if (status != TOBII_RESEARCH_STATUS_OK) {
            throw TobiiRecorderError("Failed to subscribe to eye images. Status: " + std::to_string(status), 2103);
        }
        
        std::cout << "[TobiiRecorder " << recorder_id_ << "] Successfully subscribed to streams\n";
    }

    void _unsubscribeFromStreams() {
        tobii_research_unsubscribe_from_gaze_data(eye_tracker_, 
            [](TobiiResearchGazeData* gaze_data, void* user_data) {
                auto* recorder = static_cast<TobiiRecorder*>(user_data);
                recorder->_gazeDataCallback(gaze_data);
            });
        
        tobii_research_unsubscribe_from_eye_image(eye_tracker_, 
            [](TobiiResearchEyeImage* eye_image, void* user_data) {
                auto* recorder = static_cast<TobiiRecorder*>(user_data);
                recorder->_eyeImageCallback(eye_image);
            });
        
        std::cout << "[TobiiRecorder " << recorder_id_ << "] Unsubscribed from streams\n";
    }

    void _gazeDataCallback(TobiiResearchGazeData* gaze_data) {
        try {
            if (!is_recording_.load()) return;
            gaze_count_.fetch_add(1);
        } catch (const std::exception& e) {
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Gaze callback error: " << e.what() << "\n";
        } catch (...) {
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Unknown gaze callback error\n";
        }
    }

    void _eyeImageCallback(TobiiResearchEyeImage* eye_image) {
        try {
            if (!is_recording_.load()) return;
            
            if (eye_image->camera_id == 0) {
                left_eye_image_count_.fetch_add(1);
            } else {
                right_eye_image_count_.fetch_add(1);
            }
        } catch (const std::exception& e) {
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Eye image callback error: " << e.what() << "\n";
        } catch (...) {
            std::cout << "[TobiiRecorder " << recorder_id_ << "] Unknown eye image callback error\n";
        }
    }
};

class TobiiMaker {
/** __init__ */
private:
    int recorder_id_;
    std::atomic<bool> is_gate_open_;
    std::string output_directory_;
    std::ofstream gaze_csv_file_;

public:
    TobiiMaker(int recorder_id) : recorder_id_(recorder_id), is_gate_open_(false) {
        output_directory_ = gonfig.tobii_output_directory;
    }

    ~TobiiMaker() {
        stop();
    }

/** methods */
public:
    void setup() {
        std::cout << "[TobiiMaker " << recorder_id_ << "] Starting setup...\n";
        try {
            _createOutputDirectories();
            _setupGazeCSV();
            std::cout << "[TobiiMaker " << recorder_id_ << "] Setup completed\n";
        } catch (const std::exception& e) {
            std::cout << "[TobiiMaker " << recorder_id_ << "] Setup failed: " << e.what() << "\n";
            throw;
        }
    }

    void warmup() {
        std::cout << "[TobiiMaker " << recorder_id_ << "] warmup done\n";
    }

    void run() {
        is_gate_open_ = true;
        std::cout << "[TobiiMaker " << recorder_id_ << "] started\n";
    }

    void stop() {
        is_gate_open_ = false;
        if (gaze_csv_file_.is_open()) {
            gaze_csv_file_.close();
        }
        std::cout << "[TobiiMaker " << recorder_id_ << "] stopped\n";
    }

private:
    void _createOutputDirectories() {
        try {
            if (!std::filesystem::exists(output_directory_)) {
                std::filesystem::create_directories(output_directory_);
            }
            
            std::string eye_images_dir = output_directory_ + "/eye_images";
            if (!std::filesystem::exists(eye_images_dir)) {
                std::filesystem::create_directories(eye_images_dir);
            }
        } catch (const std::exception& e) {
            throw TobiiMakerError("Failed to create output directory: " + std::string(e.what()), 2201);
        }
    }

    void _setupGazeCSV() {
        if (!gonfig.save_gaze_csv) return;
        
        std::string csv_path = output_directory_ + "/gaze_data.csv";
        gaze_csv_file_.open(csv_path);
        
        if (!gaze_csv_file_.is_open()) {
            throw TobiiMakerError("Failed to create gaze CSV file", 2202);
        }
        
        gaze_csv_file_ << "timestamp_us,device_timestamp,left_x,left_y,left_valid,left_pupil,right_x,right_y,right_valid,right_pupil\n";
    }
};

class TobiiManager {
/** __init__ */
private:
    int manager_id_;
    std::unique_ptr<TobiiDevice> tobii_device_;
    std::unique_ptr<TobiiRecorder> tobii_recorder_;
    std::unique_ptr<TobiiMaker> tobii_maker_;
    std::atomic<bool> enabled_;

public:
    TobiiManager() : manager_id_(0), enabled_(false) {}

/** methods */
public:
    void init() {
        if (!gonfig.enable_tobii) {
            std::cout << "[TobiiManager] Tobii disabled\n";
            return;
        }
        
        std::cout << "[TobiiManager] Initializing...\n";
        tobii_device_ = std::make_unique<TobiiDevice>();
        tobii_recorder_ = std::make_unique<TobiiRecorder>(manager_id_);
        tobii_maker_ = std::make_unique<TobiiMaker>(manager_id_);
        std::cout << "[TobiiManager] init done\n";
    }

    void setup() {
        if (!gonfig.enable_tobii) return;
        
        std::cout << "[TobiiManager] Starting setup...\n";
        try {
            tobii_device_->setup();
            tobii_recorder_->setup(tobii_device_->getEyeTracker());
            tobii_maker_->setup();
            enabled_ = true;
            std::cout << "[TobiiManager] Setup completed\n";
        } catch (const std::exception& e) {
            std::cout << "[TobiiManager] Setup failed: " << e.what() << "\n";
            enabled_ = false;
            throw;
        }
    }

    void warmup() {
        if (!enabled_) return;
        
        std::cout << "[TobiiManager] Starting warmup...\n";
        tobii_device_->warmup();
        tobii_recorder_->warmup();
        tobii_maker_->warmup();
        sleep_ms(gonfig.warmup_duration * 1000);
        std::cout << "[TobiiManager] warmup done\n";
    }

    void run() {
        if (!enabled_) return;
        
        std::cout << "[TobiiManager] Starting run...\n";
        tobii_recorder_->run();
        tobii_maker_->run();
        std::cout << "[TobiiManager] Run started\n";
    }

    void stop() {
        if (!enabled_) return;
        
        std::cout << "[TobiiManager] Stopping...\n";
        tobii_recorder_->stop();
        tobii_maker_->stop();
        tobii_device_->stop();
        std::cout << "[TobiiManager] cleanup done\n";
    }

    bool isEnabled() const { return enabled_.load(); }
    int getManagerId() const { return manager_id_; }
    
    // For summary
    int getGazeCount() const { 
        return enabled_ && tobii_recorder_ ? tobii_recorder_->getGazeCount() : 0; 
    }
    int getLeftEyeImageCount() const { 
        return enabled_ && tobii_recorder_ ? tobii_recorder_->getLeftEyeImageCount() : 0; 
    }
    int getRightEyeImageCount() const { 
        return enabled_ && tobii_recorder_ ? tobii_recorder_->getRightEyeImageCount() : 0; 
    }
};


/**
 * @Class: Multi
 */

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
    
    // Camera managers
    std::vector<std::unique_ptr<CamManager>> cam_managers_;
    
    // Tobii manager
    std::unique_ptr<TobiiManager> tobii_manager_;
    
    // Synchronization
    std::vector<std::unique_ptr<Barrier>> barriers_;
    std::vector<std::thread> cam_threads_;
    std::thread tobii_thread_;
    
    // Control
    Timer timer_;
    std::atomic<bool> record_signal_{false};
    std::atomic<bool> stop_signal_{false};
    std::thread stop_thread_;

public:
    MultiManager() {
        cam_managers_.clear();
        barriers_.clear();
        cam_threads_.clear();
    }

/** methods */
public:
    void setup() {
        std::cout << "[MultiManager] Starting setup...\n";
        try {
            // Setup camera managers
            for (int index : gonfig.camera_indices) {
                _camManager(index);
            }

            // Setup tobii manager
            _tobiiManager();

            // Setup barriers
            for (int i = 0; i < NUM_STAGES; ++i) {
                _barrier();
            }
            
            std::cout << "[MultiManager] Setup completed\n";
        } catch (const std::exception& e) {
            std::cout << "[MultiManager] Setup failed: " << e.what() << "\n";
            throw MultiManagerError("Setup failed: " + std::string(e.what()), 1002);
        }
    }

    void run() {
        if (cam_managers_.empty() && !gonfig.enable_tobii) {
            throw MultiManagerError("No managers available for execution", 1005);
        }

        std::cout << "[MultiManager] Starting execution...\n";

        // Start stop monitoring
        stop_thread_ = std::thread([this]() { _stopStdin(); });

        // Launch camera threads
        for (size_t i = 0; i < cam_managers_.size(); ++i) {
            cam_threads_.emplace_back([this, i]() { _camCoordination(i); });
        }

        // Launch tobii thread
        if (gonfig.enable_tobii) {
            std::cout << "[MultiManager] Creating Tobii thread...\n";
            tobii_thread_ = std::thread([this]() { _tobiiCoordination(); });
        }

        // Coordinator
        std::thread coordinator_thread([this]() { _coordinate(); });
        coordinator_thread.join();

        // Stop
        stop_signal_.store(true);

        if (stop_thread_.joinable()) {
            stop_thread_.detach();
        }

        // Wait for all threads
        for (auto& thread : cam_threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }

        if (tobii_thread_.joinable()) {
            tobii_thread_.join();
        }
        
        _printSummary();
        std::cout << "[MultiManager] All threads terminated\n";
    }

private:
    void _camManager(int camera_id) {
        auto manager = std::make_unique<CamManager>(camera_id);
        cam_managers_.push_back(std::move(manager));
    }

    void _tobiiManager() {
        tobii_manager_ = std::make_unique<TobiiManager>();
    }

    void _barrier() {
        std::size_t participant_count = cam_managers_.size();
        
        if (gonfig.enable_tobii) {  // enabled_ 대신 config 직접 확인
            participant_count += 1;
            std::cout << "[MultiManager] Added Tobii to barrier, total participants: " << participant_count << "\n";
        }
        participant_count += 1; // coordinator
        
        auto barrier = std::make_unique<Barrier>(participant_count);
        barriers_.push_back(std::move(barrier));
    }

    void _camCoordination(size_t manager_index) {
        try {
            auto& manager = cam_managers_[manager_index];
            
            // Synchronized stages
            barriers_[INIT]->arrive_and_wait();
            manager->init();

            barriers_[SETUP]->arrive_and_wait();
            manager->setup();

            barriers_[WARMUP]->arrive_and_wait();
            manager->warmup();

            barriers_[RUN]->arrive_and_wait();
            while (!record_signal_.load() && !stop_signal_.load()) {
                timer_.microSl(10);
            }
            if (!stop_signal_.load()) {
                manager->run();
            }

            barriers_[END]->arrive_and_wait();
            manager->stop();
            
        } catch (const std::exception& e) {
            std::cout << "[MultiManager] Camera coordination failed: " << e.what() << "\n";
        }
    }

    void _tobiiCoordination() {
        try {
            std::cout << "[MultiManager] Tobii coordination started\n";
            
            // Synchronized stages
            barriers_[INIT]->arrive_and_wait();
            std::cout << "[MultiManager] Calling tobii_manager_->init()\n";
            tobii_manager_->init();

            barriers_[SETUP]->arrive_and_wait();
            std::cout << "[MultiManager] Calling tobii_manager_->setup()\n";
            tobii_manager_->setup();

            barriers_[WARMUP]->arrive_and_wait();
            std::cout << "[MultiManager] Calling tobii_manager_->warmup()\n";
            tobii_manager_->warmup();

            barriers_[RUN]->arrive_and_wait();
            while (!record_signal_.load() && !stop_signal_.load()) {
                timer_.microSl(10);
            }
            if (!stop_signal_.load()) {
                std::cout << "[MultiManager] Calling tobii_manager_->run()\n";
                tobii_manager_->run();
            }

            barriers_[END]->arrive_and_wait();
            std::cout << "[MultiManager] Calling tobii_manager_->stop()\n";
            tobii_manager_->stop();
            
        } catch (const std::exception& e) {
            std::cout << "[MultiManager] Tobii coordination failed: " << e.what() << "\n";
            // Tobii 실패해도 camera recording은 계속 진행
        }
    }

    void _coordinate() {
        try {
            barriers_[INIT]->arrive_and_wait();
            barriers_[SETUP]->arrive_and_wait();
            barriers_[WARMUP]->arrive_and_wait();
            barriers_[RUN]->arrive_and_wait();
            _run();
            barriers_[END]->arrive_and_wait();
        } catch (const std::exception& e) {
            std::cout << "[MultiManager] Coordination failed: " << e.what() << "\n";
        }
    }

    void _run() {
        auto start = Timer::Clock::now();
        auto end = start + std::chrono::seconds(gonfig.record_duration);
        
        record_signal_.store(true);

        while (Timer::Clock::now() < end && !stop_signal_.load()) {
            auto remaining = end - Timer::Clock::now();
            
            if (remaining > std::chrono::milliseconds(10)) {
                timer_.milliSl(5);
            } else {
                timer_.microSl(100);
            }
        }

        record_signal_.store(false);

        auto actual_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            Timer::Clock::now() - start
        ).count();
        
        std::cout << "[MultiManager] Recording time: " 
                  << actual_duration << "μs (" 
                  << std::fixed << std::setprecision(6) 
                  << actual_duration / 1000000.0 << "s)\n";
    }

    void _stopStdin() {
        std::string line;
        while (!stop_signal_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (std::cin.rdbuf()->in_avail() > 0) {
                std::getline(std::cin, line);
                if (line == "stop") {
                    stop_signal_.store(true);
                    break;
                }
            }
        }
    }

    void _printSummary() {
        std::cout << "\n=== Recording Summary ===\n";
        std::cout << "Camera managers: " << cam_managers_.size() << "\n";
        
        if (tobii_manager_->isEnabled()) {
            std::cout << "Tobii enabled: YES\n";
            std::cout << "Gaze data points: " << tobii_manager_->getGazeCount() << "\n";
            std::cout << "Left eye images: " << tobii_manager_->getLeftEyeImageCount() << "\n";
            std::cout << "Right eye images: " << tobii_manager_->getRightEyeImageCount() << "\n";
            std::cout << "Tobii output: " << gonfig.tobii_output_directory << "\n";
        } else {
            std::cout << "Tobii enabled: NO\n";
        }
    }
};

/**
 *  Main
 */

// GLOBAL
Config gonfig;

int main(int argc, char* argv[]) {
    // Setup exception handlers
    ExceptionHandler::setupTerminateHandler();
    
    // Windows COM
    CoInitialize(NULL);

    std::cout << "=== Multi-Camera + Tobii Controller ===\n";

    try {
        // Parse config (camera + tobii)
        std::cout << "[Main] Parsing configuration...\n";
        gonfig = ConfigManager().config(argc, argv);

        // Create unified MultiManager
        std::cout << "[Main] Creating MultiManager...\n";
        MultiManager multiManager;
        
        std::cout << "[Main] Setting up...\n";
        multiManager.setup();
        
        std::cout << "[Main] Running...\n";
        multiManager.run();

        std::cout << "[Main] Program completed successfully\n";

    } catch (const TobiiManagerError& e) {
        std::cout << "[ERROR] TobiiManager: " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        CoUninitialize();
        return e.getCode();
    } catch (const TobiiRecorderError& e) {
        std::cout << "[ERROR] TobiiRecorder: " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        CoUninitialize();
        return e.getCode();
    } catch (const TobiiMakerError& e) {
        std::cout << "[ERROR] TobiiMaker: " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        CoUninitialize();
        return e.getCode();
    } catch (const MultiManagerError& e) {
        std::cout << "[ERROR] MultiManager: " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        CoUninitialize();
        return e.getCode();
    } catch (const Error& e) {
        std::cout << "[ERROR] " << e.getType() << ": " << e.getMessage() << " (code: " << e.getCode() << ")\n";
        CoUninitialize();
        return e.getCode();
    } catch (const std::runtime_error& e) {
        std::cout << "[ERROR] Runtime error: " << e.what() << "\n";
        CoUninitialize();
        return -2;
    } catch (const std::exception& e) {
        std::cout << "[ERROR] Standard exception: " << e.what() << "\n";
        CoUninitialize();
        return -3;
    } catch (...) {
        std::cout << "[ERROR] Unknown exception occurred\n";
        CoUninitialize();
        return -4;
    }

    CoUninitialize();
    return 0;
}