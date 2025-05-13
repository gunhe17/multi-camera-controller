#include "capture/camera_manager.hpp"
#include "include/common.hpp"

#include <thread>
#include <chrono>
#include <vector>
#include <iostream>
#include <filesystem>

int main(int argc, char* argv[]) {
    if (argc != 13) {
        std::cerr << "Usage: <width> <height> <fps> <duration_sec> <ffmpeg_path> <output_dir> <cam0> <cam1> <cam2> <cam3> <cam4> <cam5>\n";
        return 1;
    }

    int width = std::stoi(argv[1]);
    int height = std::stoi(argv[2]);
    int fps = std::stoi(argv[3]);
    int duration = std::stoi(argv[4]);
    std::string ffmpeg_path = argv[5];
    std::string output_dir = argv[6];

    std::vector<CaptureConfig> configs;
    for (int i = 0; i < 6; ++i) {
        int cam_id = std::stoi(argv[7 + i]);
        if (cam_id >= 0) {
            std::string out_base = output_dir + "/cam_" + std::to_string(cam_id);
            configs.push_back(CaptureConfig{
                cam_id,
                width,
                height,
                fps,
                duration,
                ffmpeg_path,
                out_base + ".mp4",
                out_base + "_log.csv"
            });
        }
    }

    std::filesystem::create_directories(output_dir);

    CameraManager manager(configs);
    if (!manager.initialize()) return 1;

    manager.start_all();
    std::this_thread::sleep_for(std::chrono::seconds(duration + 5));
    manager.stop_all();

    std::cout << "촬영 완료\n";
    return 0;
}
