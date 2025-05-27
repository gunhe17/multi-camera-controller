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

        STARTUPINFOA si = { sizeof(STARTUPINFOA) };
        si.dwFlags = STARTF_USESTDHANDLES;
        si.hStdInput = readHandle;
        si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
        si.hStdError = GetStdHandle(STD_ERROR_HANDLE);

        PROCESS_INFORMATION pi = {};
        std::string command = buildCommand();

        if (!CreateProcessA(NULL, const_cast<LPSTR>(command.c_str()),
            NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi)) {
            std::cerr << "[FFmpeg] FFmpeg 실행 실패\n";
            CloseHandle(readHandle);
            CloseHandle(writeHandle);
            return false;
        }

        CloseHandle(readHandle);  // FFmpeg가 사용하는 읽기 핸들 닫기

        ffmpeg_stdin_ = writeHandle;
        ffmpeg_proc_ = pi.hProcess;
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
    std::string buildCommand() const {
        return config_.ffmpeg +
            " -loglevel error -y" +
            " -f rawvideo" +
            " -video_size " + std::to_string(config_.frame_width) + "x" + std::to_string(config_.frame_height) +
            " -framerate " + std::to_string(config_.frame_rate) +
            " -i - -t " + std::to_string(config_.duration) +
            " -c:v libx264 -preset ultrafast \"" + config_.output + "\"";
    }

    Config config_;
    HANDLE ffmpeg_stdin_ = NULL;
    HANDLE ffmpeg_proc_ = NULL;
};


/**
 *  Main
 */
int main(int argc, char* argv[]) {
    HRESULT hr = MFStartup(MF_VERSION);
    if (FAILED(hr)) {
        std::cerr << "[Main] Media Foundation 초기화 실패\n";
        return -1;
    }

    Config config = parse_args(argc, argv);
    FFmpeg ffmpeg;
    ffmpeg.setConfig(config);

    if (!ffmpeg.start()) {
        std::cerr << "[Main] FFmpeg 실행 실패\n";
        return -1;
    }

    std::cout << "[Main] 프레임 전송 시작\n";

    const int total_frames = config.frame_rate * config.duration;
    const size_t frame_size = config.frame_width * config.frame_height * 2; // YUYV = 2 bytes/pixel

    std::vector<BYTE> dummy_frame(frame_size, 128);  // 회색 프레임

    for (int i = 0; i < total_frames; ++i) {
        if (!ffmpeg.write(dummy_frame.data(), dummy_frame.size())) {
            std::cerr << "[Main] 프레임 쓰기 실패 at index " << i << "\n";
            break;
        }
        std::cout << "[Main] 프레임 전송 중: " << (i + 1) << "/" << total_frames << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / config.frame_rate));
    }

    std::cout << "[Main] 프레임 전송 완료, FFmpeg 종료 대기 중...\n";

    // 프레임 전송 완료
    std::cout << "[Main] 프레임 전송 완료, 파이프 종료...\n";

    // 반드시 파이프 먼저 닫아야 FFmpeg가 입력 종료로 인식함
    ffmpeg.stop();  // 내부에서 ffmpeg_stdin_과 ffmpeg_proc_ 모두 닫음


    MFShutdown();
    return 0;
}
