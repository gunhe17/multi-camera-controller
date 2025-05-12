#ifndef TIMESTAMP_LOGGER_HPP
#define TIMESTAMP_LOGGER_HPP

#include "../include/common.hpp"
#include "../util/thread_safe_queue.hpp"

#include <fstream>
#include <thread>
#include <atomic>
#include <vector>
#include <iostream>

class TimestampLogger {
public:
    TimestampLogger(const std::string& filename)
        : stop_flag_(false)
    {
        std::cout << "[TimestampLogger] 로그 파일 열기 시도: " << filename << "\n";
        log_file_.open(filename);

        if (!log_file_.is_open()) {
            std::cerr << "[TimestampLogger] 로그 파일 열기 실패: " << filename << "\n";
            throw std::runtime_error("Failed to open log file: " + filename);
        }

        log_file_ << "frame_index,timestamp_ns,relative_us,camera_id\n";

        std::cout << "[TimestampLogger] 로그 쓰레드 시작\n";
        worker_ = std::thread([this]() { this->process(); });
    }

    ~TimestampLogger() {
        stop();
    }

    void log(const FrameMeta& meta) {
        queue_.push(meta);
    }

    void stop() {
        if (!stop_flag_) {
            stop_flag_ = true;
            queue_.push(FrameMeta{-1, 0, 0, -1}); // 종료 신호
            if (worker_.joinable()) {
                std::cout << "[TimestampLogger] 로그 쓰레드 종료 대기 중...\n";
                worker_.join();
                std::cout << "[TimestampLogger] 로그 쓰레드 종료 완료\n";
            }
        }
    }

private:
    std::ofstream log_file_;
    ThreadSafeQueue<FrameMeta> queue_;
    std::thread worker_;
    std::atomic<bool> stop_flag_;

    void process() {
        std::cout << "[TimestampLogger] 로그 쓰레드 루프 진입\n";
        while (!stop_flag_) {
            auto meta = queue_.pop();
            if (meta.frame_index == -1 && meta.camera_id == -1)
                break;

            log_file_ << meta.frame_index << ","
                      << meta.timestamp_ns << ","
                      << meta.relative_us << ","
                      << meta.camera_id << "\n";
        }
        std::cout << "[TimestampLogger] 로그 파일 닫는 중\n";
        log_file_.flush();
        log_file_.close();
        std::cout << "[TimestampLogger] 로그 파일 닫기 완료\n";
    }
};

#endif // TIMESTAMP_LOGGER_HPP
