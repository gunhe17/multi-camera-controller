#pragma once

#include <cstdint>
#include <vector>

struct FrameMeta {
    uint64_t frame_index = 0;
    uint64_t timestamp_ns;
    uint64_t relative_us;
    int camera_id;
};