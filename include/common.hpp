#ifndef COMMON_HPP
#define COMMON_HPP

#include <string>
#include <cstdint>

// ?꾨젅??硫뷀??곗씠??援ъ“泥?
struct FrameMeta {
    int frame_index;
    uint64_t timestamp_ns;    // sampleTime (100ns ?⑥쐞 ??ns ?⑥쐞濡?蹂?섎맖)
    int64_t relative_us;      // T? 湲곗? ?곷? ?쒓컙 (留덉씠?щ줈珥?
    int camera_id;            // ?μ튂 援щ텇??
};

// 罹≪쿂 ?ㅼ젙 援ъ“泥?
struct CaptureConfig {
    int camera_id;
    int width = 1280;
    int height = 720;
    int fps = 30;
    std::string ffmpeg_path = "ffmpeg"; // 寃쎈줈媛 ?ㅻⅤ?ㅻ㈃ ?섏젙 ?꾩슂
    std::string output_filename;        // cam_0.mp4 ? 媛숈? ?뚯씪紐?
    std::string log_filename;           // cam_0_log.csv ? 媛숈? 濡쒓렇?뚯씪
};

#endif // COMMON_HPP
