#pragma once

#include <cstdint>
#include <vector>

#include "common\frame_types.hpp"


struct FFmpegInput {
    BYTE* data;
    DWORD length;
};