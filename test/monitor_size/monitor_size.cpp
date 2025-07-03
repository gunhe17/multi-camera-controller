#include <windows.h>
#include <iostream>

int main() {
    DISPLAY_DEVICE dd;
    dd.cb = sizeof(dd);
    int deviceIndex = 0;

    while (EnumDisplayDevices(NULL, deviceIndex, &dd, 0)) {
        if (dd.StateFlags & DISPLAY_DEVICE_ACTIVE) {
            std::wcout << L"Monitor " << deviceIndex << L": " << dd.DeviceName << std::endl;

            DEVMODE dm = {};
            dm.dmSize = sizeof(dm);

            if (EnumDisplaySettings(dd.DeviceName, ENUM_CURRENT_SETTINGS, &dm)) {
                std::wcout << L"  Resolution: " << dm.dmPelsWidth << L"x" << dm.dmPelsHeight << std::endl;
                std::wcout << L"  Position: (" << dm.dmPosition.x << L", " << dm.dmPosition.y << L")" << std::endl;

                // Create DC for this monitor
                HDC hdc = CreateDC(NULL, dd.DeviceName, NULL, NULL);
                if (hdc) {
                    int width_mm = GetDeviceCaps(hdc, HORZSIZE);
                    int height_mm = GetDeviceCaps(hdc, VERTSIZE);
                    std::wcout << L"  Physical size: " << width_mm << L"mm x " << height_mm << L"mm" << std::endl;
                    DeleteDC(hdc);
                } else {
                    std::wcout << L"  [!] Failed to create HDC for physical size." << std::endl;
                }
            } else {
                std::wcout << L"  [!] Failed to get settings." << std::endl;
            }
        }

        deviceIndex++;
        ZeroMemory(&dd, sizeof(dd));
        dd.cb = sizeof(dd);
    }

    return 0;
}
