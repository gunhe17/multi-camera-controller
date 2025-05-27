#pragma once

#include <Windows.h>
#include <iostream>
#include <sstream>


/**
 *  @method: HFailed
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