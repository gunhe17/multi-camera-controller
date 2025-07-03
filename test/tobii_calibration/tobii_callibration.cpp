#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <windows.h>
#include "tobii_research_eyetracker.h"
#include "tobii_research_calibration.h"
#include "tobii_research_streams.h"

// Constants
#define NUM_CALIBRATION_POINTS 13
#define TARGET_RADIUS 15
#define WINDOW_CLASS_NAME "TobiiCalibrationWindow"

// Global variables
typedef struct {
    HWND hwnd;
    RECT rect;
    bool is_primary;
} MonitorWindow;

MonitorWindow* monitor_windows = NULL;
int monitor_count = 0;
int selected_monitor_option = 0; // 0: 모든 모니터, 1~N: 특정 모니터

int current_point_index = 0;

// 13-point calibration grid
TobiiResearchNormalizedPoint2D calibration_points[NUM_CALIBRATION_POINTS] = {
    {0.1f, 0.1f}, {0.3f, 0.1f}, {0.5f, 0.1f}, {0.7f, 0.1f}, {0.9f, 0.1f},
    {0.1f, 0.3f},               {0.5f, 0.3f},               {0.9f, 0.3f},
    {0.1f, 0.5f}, {0.3f, 0.5f}, {0.5f, 0.5f}, {0.7f, 0.5f}, {0.9f, 0.5f}
};

// Function declarations
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData);
void gaze_callback(TobiiResearchGazeData* gaze_data, void* user_data);
bool init_tobii_system(TobiiResearchEyeTracker** eyetracker);
bool setup_monitors();
int select_monitor();
void cleanup_monitors();
void draw_calibration_target(HDC hdc, RECT window_rect, TobiiResearchNormalizedPoint2D point, bool is_active);
void show_progress(HDC hdc, RECT window_rect, int current, int total);
bool perform_calibration(TobiiResearchEyeTracker* eyetracker);
bool save_calibration_data(TobiiResearchEyeTracker* eyetracker);
void sleep_ms(int milliseconds);

// Utility function for cross-platform sleep
void sleep_ms(int milliseconds) {
    Sleep(milliseconds);
}

// Monitor enumeration callback
BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData) {
    MONITORINFO mi;
    mi.cbSize = sizeof(MONITORINFO);
    
    if (GetMonitorInfo(hMonitor, &mi)) {
        MonitorWindow* temp = (MonitorWindow*)realloc(monitor_windows, (monitor_count + 1) * sizeof(MonitorWindow));
        if (!temp) {
            printf("Failed to allocate memory for monitor windows!\n");
            return FALSE;
        }
        monitor_windows = temp;
        
        monitor_windows[monitor_count].rect = mi.rcMonitor;
        monitor_windows[monitor_count].is_primary = (mi.dwFlags & MONITORINFOF_PRIMARY) != 0;
        monitor_windows[monitor_count].hwnd = NULL;
        
        monitor_count++;
        
        printf("Monitor %d: %dx%d at (%d,%d) %s\n", 
               monitor_count,
               mi.rcMonitor.right - mi.rcMonitor.left,
               mi.rcMonitor.bottom - mi.rcMonitor.top,
               mi.rcMonitor.left, mi.rcMonitor.top,
               (mi.dwFlags & MONITORINFOF_PRIMARY) ? "(Primary)" : "");
    }
    
    return TRUE;
}

// Window procedure
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hwnd, &ps);
            
            RECT rect;
            GetClientRect(hwnd, &rect);
            
            // Fill background with gray
            HBRUSH hBrush = CreateSolidBrush(RGB(128, 128, 128));
            FillRect(hdc, &rect, hBrush);
            DeleteObject(hBrush);
            
            // Draw current calibration target (항상 빨간색)
            if (current_point_index < NUM_CALIBRATION_POINTS) {
                draw_calibration_target(hdc, rect, calibration_points[current_point_index], false);
                show_progress(hdc, rect, current_point_index + 1, NUM_CALIBRATION_POINTS);
            }
            
            EndPaint(hwnd, &ps);
            return 0;
        }
        
        case WM_KEYDOWN:
            if (wParam == VK_ESCAPE) {
                PostQuitMessage(0);
            }
            return 0;
            
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    
    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

// Draw calibration target
void draw_calibration_target(HDC hdc, RECT window_rect, TobiiResearchNormalizedPoint2D point, bool is_active) {
    int window_width = window_rect.right - window_rect.left;
    int window_height = window_rect.bottom - window_rect.top;
    
    int x = (int)(point.x * window_width);
    int y = (int)(point.y * window_height);
    
    // 항상 빨간색 (is_active 매개변수 무시)
    COLORREF color = RGB(255, 0, 0);
    HBRUSH hBrush = CreateSolidBrush(color);
    HPEN hPen = CreatePen(PS_SOLID, 2, RGB(0, 0, 0));
    
    SelectObject(hdc, hBrush);
    SelectObject(hdc, hPen);
    
    // Draw circle
    Ellipse(hdc, x - TARGET_RADIUS, y - TARGET_RADIUS, 
            x + TARGET_RADIUS, y + TARGET_RADIUS);
    
    DeleteObject(hBrush);
    DeleteObject(hPen);
}

// Show progress indicator
void show_progress(HDC hdc, RECT window_rect, int current, int total) {
    char progress_text[100];
    sprintf(progress_text, "Calibration Point %d/%d", current, total);
    
    SetTextColor(hdc, RGB(255, 255, 255));
    SetBkColor(hdc, RGB(128, 128, 128));
    
    RECT text_rect = {10, 10, 300, 40};
    DrawTextA(hdc, progress_text, -1, &text_rect, DT_LEFT | DT_TOP);
}

// Setup monitor windows
// setup_monitors() 함수를 다음과 같이 수정
bool setup_monitors() {
    printf("Detecting monitors...\n");
    
    // Enumerate all monitors
    EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, 0);
    
    if (monitor_count == 0) {
        printf("No monitors detected!\n");
        return false;
    }
    
    // Monitor selection
    selected_monitor_option = select_monitor();
    
    if (selected_monitor_option <= monitor_count) {
        printf("Selected: Monitor %d\n", selected_monitor_option);
    } else {
        printf("Selected: All Monitors\n");
    }
    
    // Register window class
    WNDCLASSA wc = {0};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.lpszClassName = WINDOW_CLASS_NAME;
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    
    if (!RegisterClassA(&wc)) {
        printf("Failed to register window class!\n");
        return false;
    }
    
    // Create windows based on selection
    if (selected_monitor_option <= monitor_count) {
        // Create window for selected monitor only
        int i = selected_monitor_option - 1;
        RECT rect = monitor_windows[i].rect;
        int width = rect.right - rect.left;
        int height = rect.bottom - rect.top;
        
        monitor_windows[i].hwnd = CreateWindowExA(
            WS_EX_TOPMOST,
            WINDOW_CLASS_NAME,
            "Tobii Calibration",
            WS_POPUP,
            rect.left, rect.top, width, height,
            NULL, NULL, GetModuleHandle(NULL), NULL
        );
        
        if (!monitor_windows[i].hwnd) {
            printf("Failed to create window for monitor %d!\n", i + 1);
            return false;
        }
        
        ShowWindow(monitor_windows[i].hwnd, SW_SHOW);
        UpdateWindow(monitor_windows[i].hwnd);
        
        printf("Calibration window created on Monitor %d.\n", i + 1);
    } else {
        // Create windows for all monitors
        for (int i = 0; i < monitor_count; i++) {
            RECT rect = monitor_windows[i].rect;
            int width = rect.right - rect.left;
            int height = rect.bottom - rect.top;
            
            monitor_windows[i].hwnd = CreateWindowExA(
                WS_EX_TOPMOST,
                WINDOW_CLASS_NAME,
                "Tobii Calibration",
                WS_POPUP,
                rect.left, rect.top, width, height,
                NULL, NULL, GetModuleHandle(NULL), NULL
            );
            
            if (!monitor_windows[i].hwnd) {
                printf("Failed to create window for monitor %d!\n", i + 1);
                return false;
            }
            
            ShowWindow(monitor_windows[i].hwnd, SW_SHOW);
            UpdateWindow(monitor_windows[i].hwnd);
        }
        
        printf("Calibration windows created on all %d monitors.\n", monitor_count);
    }
    
    return true;
}

int select_monitor() {
    printf("\n=== Monitor Selection ===\n");
    for (int i = 0; i < monitor_count; i++) {
        RECT rect = monitor_windows[i].rect;
        int width = rect.right - rect.left;
        int height = rect.bottom - rect.top;
        
        printf("%d. Monitor %d: %dx%d at (%d,%d) %s\n", 
               i + 1, i + 1,
               width, height,
               rect.left, rect.top,
               monitor_windows[i].is_primary ? "(Primary)" : "");
    }
    printf("%d. All Monitors (동시 표시)\n", monitor_count + 1);
    
    int choice;
    while (true) {
        printf("\nSelect monitor (1-%d): ", monitor_count + 1);
        if (scanf("%d", &choice) == 1) {
            if (choice >= 1 && choice <= monitor_count + 1) {
                return choice;
            }
        }
        printf("Invalid selection. Please try again.\n");
        // Clear input buffer
        int c;
        while ((c = getchar()) != '\n' && c != EOF);
    }
}

// Cleanup monitor windows
void cleanup_monitors() {
    for (int i = 0; i < monitor_count; i++) {
        if (monitor_windows[i].hwnd) {
            DestroyWindow(monitor_windows[i].hwnd);
        }
    }
    
    if (monitor_windows) {
        free(monitor_windows);
        monitor_windows = NULL;
    }
    
    monitor_count = 0;
    UnregisterClassA(WINDOW_CLASS_NAME, GetModuleHandle(NULL));
}

// Initialize Tobii system
bool init_tobii_system(TobiiResearchEyeTracker** eyetracker) {
    printf("Initializing Tobii system...\n");
    
    TobiiResearchEyeTrackers* eyetrackers = NULL;
    TobiiResearchStatus status = tobii_research_find_all_eyetrackers(&eyetrackers);
    
    if (status != TOBII_RESEARCH_STATUS_OK || eyetrackers->count == 0) {
        printf("No eye trackers found! Status: %d\n", status);
        return false;
    }
    
    *eyetracker = eyetrackers->eyetrackers[0];
    
    char* serial_number = NULL;
    tobii_research_get_serial_number(*eyetracker, &serial_number);
    printf("Using eye tracker: %s\n", serial_number);
    tobii_research_free_string(serial_number);
    
    // Don't free eyetrackers here - we need the eyetracker reference
    return true;
}

// Perform calibration process
bool perform_calibration(TobiiResearchEyeTracker* eyetracker) {
    printf("Starting calibration process...\n");
        
    // Enter calibration mode
    TobiiResearchStatus status = tobii_research_screen_based_calibration_enter_calibration_mode(eyetracker);
    if (status != TOBII_RESEARCH_STATUS_OK) {
        printf("Failed to enter calibration mode! Status: %d\n", status);
        return false;
    }
    
    printf("Calibration mode entered. Starting data collection...\n");
    
    // Collect data for each point
    for (current_point_index = 0; current_point_index < NUM_CALIBRATION_POINTS; current_point_index++) {
        TobiiResearchNormalizedPoint2D* point = &calibration_points[current_point_index];
        
        printf("Calibrating point %d/%d at (%.1f, %.1f)\n", 
               current_point_index + 1, NUM_CALIBRATION_POINTS, point->x, point->y);
        
        // Show target on selected monitor(s)
        if (selected_monitor_option <= monitor_count) {
            int i = selected_monitor_option - 1;
            InvalidateRect(monitor_windows[i].hwnd, NULL, TRUE);
            UpdateWindow(monitor_windows[i].hwnd);
        } else {
            for (int i = 0; i < monitor_count; i++) {
                InvalidateRect(monitor_windows[i].hwnd, NULL, TRUE);
                UpdateWindow(monitor_windows[i].hwnd);
            }
        }
        
        printf("Show a point on screen at (%.1f, %.1f)\n", point->x, point->y);
        printf("Please look at the red target...\n");
        
        // Wait for user to focus (Tobii calibration.c style)
        sleep_ms(700);
        
        printf("Collecting data at (%.1f, %.1f)\n", point->x, point->y);
        
        // Process Windows messages during data collection
        MSG msg;
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                printf("Calibration cancelled by user.\n");
                return false;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        
        // Collect calibration data
        status = tobii_research_screen_based_calibration_collect_data(eyetracker, point->x, point->y);
        if (status != TOBII_RESEARCH_STATUS_OK) {
            printf("Failed to collect data for point %d. Retrying...\n", current_point_index + 1);
            // Retry once (like Tobii example)
            status = tobii_research_screen_based_calibration_collect_data(eyetracker, point->x, point->y);
            if (status != TOBII_RESEARCH_STATUS_OK) {
                printf("Failed to collect data for point %d after retry!\n", current_point_index + 1);
            }
        }
        
        printf("Data collected for point %d.\n", current_point_index + 1);
        sleep_ms(300); // Brief pause before next point
    }
    
    // Compute and apply calibration
    printf("Computing and applying calibration...\n");
    TobiiResearchCalibrationResult* calibration_result = NULL;
    status = tobii_research_screen_based_calibration_compute_and_apply(eyetracker, &calibration_result);
    
    bool success = false;
    if (status == TOBII_RESEARCH_STATUS_OK && 
        calibration_result->status == TOBII_RESEARCH_CALIBRATION_SUCCESS) {
        printf("Calibration successful! Collected data from %zu points.\n", 
               calibration_result->calibration_point_count);
        success = true;
    } else {
        printf("Calibration failed! Status: %d\n", status);
    }
    
    // Cleanup
    if (calibration_result) {
        tobii_research_free_screen_based_calibration_result(calibration_result);
    }
    
    // Leave calibration mode
    tobii_research_screen_based_calibration_leave_calibration_mode(eyetracker);
        
    return success;
}

// Save calibration data to file
bool save_calibration_data(TobiiResearchEyeTracker* eyetracker) {
    printf("Saving calibration data...\n");
    
    TobiiResearchCalibrationData* calibration_data = NULL;
    TobiiResearchStatus status = tobii_research_retrieve_calibration_data(eyetracker, &calibration_data);
    
    if (status != TOBII_RESEARCH_STATUS_OK) {
        printf("Failed to retrieve calibration data! Status: %d\n", status);
        return false;
    }
    
    if (calibration_data->size == 0) {
        printf("No calibration data available!\n");
        tobii_research_free_calibration_data(calibration_data);
        return false;
    }
    
    // Create filename with timestamp
    SYSTEMTIME st;
    GetLocalTime(&st);
    char filename[256];
    sprintf(filename, "calibration_%04d%02d%02d_%02d%02d%02d.bin",
            st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    
    FILE* file = fopen(filename, "wb");
    if (!file) {
        printf("Failed to create calibration file: %s\n", filename);
        tobii_research_free_calibration_data(calibration_data);
        return false;
    }
    
    size_t written = fwrite(calibration_data->data, calibration_data->size, 1, file);
    fclose(file);
    
    if (written != 1) {
        printf("Failed to write calibration data to file!\n");
        tobii_research_free_calibration_data(calibration_data);
        return false;
    }
    
    printf("Calibration data saved to: %s (%zu bytes)\n", filename, calibration_data->size);
    
    tobii_research_free_calibration_data(calibration_data);
    return true;
}

// Main function
int main() {
    printf("=== Tobii 13-Point Multi-Monitor Calibration System ===\n\n");
    
    TobiiResearchEyeTracker* eyetracker = NULL;
    bool success = false;
    
    // Initialize Tobii system
    if (!init_tobii_system(&eyetracker)) {
        printf("Failed to initialize Tobii system!\n");
        goto cleanup;
    }
    
    // Setup monitor windows
    if (!setup_monitors()) {
        printf("Failed to setup monitor windows!\n");
        goto cleanup;
    }
    
    printf("\nCalibration windows created on all monitors.\n");
    printf("Press ESC to cancel calibration at any time.\n");
    printf("Look at each red target point. It will turn green when detected.\n\n");
    
    sleep_ms(2000); // Give user time to read instructions
    
    // Perform calibration
    if (perform_calibration(eyetracker)) {
        printf("\nCalibration completed successfully!\n");
        
        // Save calibration data
        if (save_calibration_data(eyetracker)) {
            printf("Calibration data saved successfully!\n");
            success = true;
        } else {
            printf("Warning: Failed to save calibration data!\n");
        }
    } else {
        printf("\nCalibration failed or was cancelled!\n");
    }
    
cleanup:
    cleanup_monitors();
    
    if (success) {
        printf("\n=== Calibration System Completed Successfully ===\n");
        printf("You can now use the saved calibration file for gaze tracking.\n");
    } else {
        printf("\n=== Calibration System Failed ===\n");
    }
    
    printf("Press Enter to exit...");
    getchar();
    
    return success ? 0 : 1;
}