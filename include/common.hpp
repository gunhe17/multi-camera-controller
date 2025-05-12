#ifndef COMMON_HPP
#define COMMON_HPP

#include <string>
#include <cstdint>

// 프레임 메타데이터 구조체
struct FrameMeta {
    int frame_index;
    uint64_t timestamp_ns;    // sampleTime (100ns 단위 → ns 단위로 변환됨)
    int64_t relative_us;      // T₀ 기준 상대 시간 (마이크로초)
    int camera_id;            // 장치 구분용 ID
};

// 캡처 설정 구조체
struct CaptureConfig {
    int camera_id;
    int width = 1280;
    int height = 720;
    int fps = 30;
    int duration_sec = 10;
    std::string ffmpeg_path = "ffmpeg"; // ffmpeg 경로 (필요 시 수정 가능)
    std::string output_filename;        // 출력 영상 파일명 (예: cam_0.mp4)
    std::string log_filename;           // 로그 CSV 파일명 (예: cam_0_log.csv)
};

#endif // COMMON_HPP
