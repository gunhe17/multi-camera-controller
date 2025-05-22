#ifndef FFMPEG_WRITER_HPP
#define FFMPEG_WRITER_HPP

#include "config\config.hpp"

#include <windows.h>
#include <string>
#include <iostream>


class FFmpegWriter {
public:
    FFmpegWriter(Config config)
        : config_(config), ffmpeg_stdin_(NULL), ffmpeg_proc_(NULL) {}

    bool start() {

        std::string command = "\"" + config_.ffmpeg_path + "\"" +
            " -loglevel debug -y" + 
            " -f rawvideo -pixel_format yuyv422" +
            " -video_size " + std::to_string(config_.frame_width) + 
            "x" + std::to_string(config_.frame_height) +
            " -framerate " + std::to_string(config_.frame_rate) +
            " -i - -t " + std::to_string(config_.duration_time) +
            " -c:v libx264 -preset ultrafast \"" + config_.output_path + "\"";

        SECURITY_ATTRIBUTES sa = { sizeof(SECURITY_ATTRIBUTES), NULL, TRUE };
        HANDLE writeHandle, readHandle;

        if (!CreatePipe(&readHandle, &writeHandle, &sa, 0)) {
            std::cout << "파이프 생성 실패\n";
            return false;
        }
        std::cout << "[FFmpegWriter] 파이프 생성 완료\n";

        STARTUPINFOA si = { 0 };
        si.cb = sizeof(STARTUPINFOA);
        si.dwFlags |= STARTF_USESTDHANDLES;
        si.hStdInput = readHandle;
        si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
        si.hStdError = GetStdHandle(STD_ERROR_HANDLE);        

        PROCESS_INFORMATION pi = { 0 };

        BOOL success = CreateProcessA(
            NULL,
            const_cast<LPSTR>(command.c_str()),
            NULL, NULL, TRUE, 0, NULL, NULL,
            &si, &pi
        );
        if (!success) {
            std::cout << "FFmpeg 실행 실패, GetLastError(): " << GetLastError() << "\n";
            CloseHandle(readHandle);
            CloseHandle(writeHandle);
            return false;
        }
        std::cout << "[FFmpegWriter] FFmpeg 프로세스 생성 완료\n";

        CloseHandle(readHandle);  // ffmpeg가 읽는 쪽은 닫기

        ffmpeg_proc_ = pi.hProcess;
        ffmpeg_stdin_ = writeHandle;
        return true;
    }

    // raw frame write
    bool write(const BYTE* data, size_t length) {
        if (!ffmpeg_stdin_) return false;

        DWORD written;
        return WriteFile(ffmpeg_stdin_, data, static_cast<DWORD>(length), &written, NULL);
    }

    void stop() {
        std::cout << "[FFmpegWriter] stop() 호출됨 - 종료 시도 시작\n";

        if (ffmpeg_proc_) {
            std::cout << "[FFmpegWriter] FFmpeg 프로세스 종료 대기 중...\n";
            DWORD result = WaitForSingleObject(ffmpeg_proc_, INFINITE);  // 최대 5초 대기

            if (result == WAIT_TIMEOUT) {
                std::cout << "[WARN] FFmpeg 종료 타임아웃. 강제 종료합니다.\n";
                TerminateProcess(ffmpeg_proc_, 1);
            } else {
                std::cout << "[FFmpegWriter] FFmpeg 정상 종료 감지됨\n";
            }

            CloseHandle(ffmpeg_proc_);
            ffmpeg_proc_ = NULL;
        }

        std::cout << "[FFmpegWriter] stop() 완료 - 자원 정리 끝\n";
    }


private:
    Config config_;
    HANDLE ffmpeg_stdin_;
    HANDLE ffmpeg_proc_;
};

#endif // FFMPEG_WRITER_HPP