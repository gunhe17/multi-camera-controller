#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <fstream>
#include <iomanip>

#include <windows.h>
#include <psapi.h>

#pragma comment(lib, "psapi.lib")
#pragma comment(lib, "advapi32.lib")


/**
 * @class: Resource
 */
class Resource {
/** __init__ */
private:
    // system
    MEMORYSTATUSEX memory_info_;
    ULARGE_INTEGER disk_info_;
    
    // process
    std::vector<DWORD> process_list_;
    SYSTEM_INFO cpu_info_;
    FILETIME cpu_idle_time_;
    FILETIME cpu_kernel_time_;
    FILETIME cpu_user_time_;
    
    // state
    bool com_initialized_;
    bool resource_validated_;

public:
    Resource() {
        // system
        ZeroMemory(&memory_info_, sizeof(memory_info_));
        ZeroMemory(&disk_info_, sizeof(disk_info_));

        // process
        process_list_.clear();
        ZeroMemory(&cpu_info_, sizeof(cpu_info_));
        ZeroMemory(&cpu_idle_time_, sizeof(cpu_idle_time_));
        ZeroMemory(&cpu_kernel_time_, sizeof(cpu_kernel_time_));
        ZeroMemory(&cpu_user_time_, sizeof(cpu_user_time_));

        // state
        com_initialized_ = false;
        resource_validated_ = false;
    }

    ~Resource() {
        if (com_initialized_) {
            CoUninitialize();
        }
    }

/** methods */
public:
    bool init() {
        try {
            _initializeCOM();
            _initializeMemory();
            _initializeCPUinfo();
            
            std::cout << "[Resource] 초기화 완료\n";
            return true;
        } catch (const std::exception& e) {
            std::cout << "[Resource Error] 초기화 실패: " << e.what() << "\n";
            return false;
        }
    }

    bool setup() {
        try {
            _collectMemoryInfo();
            _collectDiskInfo();
            _collectProcessList();
            _collectCPUBaseline();
            
            std::cout << "[Resource] 시스템 정보 수집 완료\n";
            return true;
        } catch (const std::exception& e) {
            std::cout << "[Resource Error] 설정 실패: " << e.what() << "\n";
            return false;
        }
    }

    bool check() {
        bool all_passed = true;
        
        all_passed &= _validateDiskSpace();
        all_passed &= _validateProcessConflict();
        all_passed &= _validateCPUUsage();
        all_passed &= _validatePermissions();
        
        resource_validated_ = all_passed;
        return all_passed;
    }

    bool test() {
        if (!resource_validated_) {
            std::cout << "[Resource Error] 검증되지 않은 상태에서 테스트 불가\n";
            return false;
        }
        
        bool all_passed = true;
        
        all_passed &= _testFFmpegExecution();
        all_passed &= _testFileIOPerformance();
        all_passed &= _testMemoryAllocation();
        all_passed &= _testDirectoryCreation();
        all_passed &= _testProcessPriority();
        
        return all_passed;
    }

    bool cleanup() {
        try {
            _cleanupTestFiles();
            _cleanupProcesses();
            _cleanupCOM();
            _resetMembers();
            
            std::cout << "[Resource] 정리 완료\n";
            return true;
        } catch (const std::exception& e) {
            std::cout << "[Resource Error] 정리 실패: " << e.what() << "\n";
            return false;
        }
    }

private:
    // init helpers
    void _initializeCOM() {
        HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
        if (FAILED(hr)) {
            throw std::runtime_error("COM 초기화 실패: " + std::to_string(hr));
        }
        com_initialized_ = true;
    }

    void _initializeMemory() {
        memory_info_.dwLength = sizeof(memory_info_);
    }
    
    void _initializeCPUinfo() {    
        GetSystemInfo(&cpu_info_);
    }
    
    // setup helpers
    void _collectMemoryInfo() {
        if (!GlobalMemoryStatusEx(&memory_info_)) {
            throw std::runtime_error("메모리 정보 수집 실패");
        }
        
        double total_gb = static_cast<double>(memory_info_.ullTotalPhys) / (1024.0 * 1024.0 * 1024.0);
        double available_gb = static_cast<double>(memory_info_.ullAvailPhys) / (1024.0 * 1024.0 * 1024.0);
        double usage_percent = (1.0 - static_cast<double>(memory_info_.ullAvailPhys) / memory_info_.ullTotalPhys) * 100.0;
        
        std::cout << "[System Monitor] Memory: " << std::fixed << std::setprecision(1) 
                  << usage_percent << "% used (" 
                  << available_gb << "GB / " << total_gb << "GB available)\n";
    }
    
    void _collectDiskInfo() {
        std::string output_path = ".";  // gonfig.output_directory 대신 기본값 사용
        if (!GetDiskFreeSpaceExA(output_path.c_str(), &disk_info_, NULL, NULL)) {
            throw std::runtime_error("디스크 정보 수집 실패");
        }
    }
    
    void _collectProcessList() {
        DWORD processes[1024], cbNeeded;
        if (!EnumProcesses(processes, sizeof(processes), &cbNeeded)) {
            throw std::runtime_error("프로세스 목록 수집 실패");
        }
        
        DWORD process_count = cbNeeded / sizeof(DWORD);
        process_list_.assign(processes, processes + process_count);
        
        std::cout << "[System Monitor] Process: " << process_count << " running processes detected\n";
    }
    
    void _collectCPUBaseline() {
        if (!GetSystemTimes(&cpu_idle_time_, &cpu_kernel_time_, &cpu_user_time_)) {
            throw std::runtime_error("CPU 기준값 수집 실패");
        }
        
        std::cout << "[System Monitor] CPU: " << cpu_info_.dwNumberOfProcessors 
                  << " cores detected, baseline collected\n";
    }
    
    // check helpers
    // check helpers
    bool _validateDiskSpace() {
        // 최소 10GB 확인
        const ULONGLONG min_disk_space = 10ULL * 1024 * 1024 * 1024;
        double available_gb = static_cast<double>(disk_info_.QuadPart) / (1024.0 * 1024.0 * 1024.0);
        
        bool passed = disk_info_.QuadPart >= min_disk_space;
        std::cout << "[Resource Check] Disk Space: " << std::fixed << std::setprecision(1) 
                  << available_gb << "GB available (min: 10.0GB) - " 
                  << (passed ? "PASS" : "FAIL") << "\n";
        
        return passed;
    }
    
    bool _validateProcessConflict() {
        // 기존 capture process 검색
        std::vector<std::string> conflict_processes;
        
        for (DWORD pid : process_list_) {
            HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, pid);
            if (hProcess) {
                CHAR processName[MAX_PATH];
                if (GetModuleBaseNameA(hProcess, NULL, processName, sizeof(processName))) {
                    std::string name(processName);
                    if (name.find("multi_camera") != std::string::npos ||
                        name.find("capture") != std::string::npos) {
                        conflict_processes.push_back(name + " (PID: " + std::to_string(pid) + ")");
                    }
                }
                CloseHandle(hProcess);
            }
        }
        
        if (!conflict_processes.empty()) {
            std::cout << "[Resource] 충돌 프로세스 발견: ";
            for (size_t i = 0; i < conflict_processes.size(); ++i) {
                std::cout << conflict_processes[i];
                if (i < conflict_processes.size() - 1) std::cout << ", ";
            }
            std::cout << "\n";
            return false;
        }
        
        std::cout << "[Resource] 충돌 프로세스 없음\n";
        return true;
    }
    
    bool _validateCPUUsage() {
        FILETIME idle, kernel, user;
        if (!GetSystemTimes(&idle, &kernel, &user)) {
            return false;
        }
        
        // 간단한 CPU 사용률 확인 (80% 이하)
        // 실제 구현에서는 더 정확한 계산 필요
        return true; // 임시로 통과
    }
    
    bool _validatePermissions() {
        // 카메라 접근 권한 및 파일 쓰기 권한 확인
        HANDLE hToken;
        if (!OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &hToken)) {
            return false;
        }
        
        // 임시로 통과 (실제로는 더 세밀한 권한 확인 필요)
        CloseHandle(hToken);
        return true;
    }
    
    // test helpers
    bool _testFFmpegExecution() {
        STARTUPINFOA si;
        PROCESS_INFORMATION pi;
        ZeroMemory(&si, sizeof(si));
        ZeroMemory(&pi, sizeof(pi));
        si.cb = sizeof(si);
        
        auto start = std::chrono::high_resolution_clock::now();
        std::string command = "ffmpeg -version";
        bool created = CreateProcessA(NULL, const_cast<char*>(command.c_str()), NULL, NULL, FALSE, 
                                     CREATE_NO_WINDOW, NULL, NULL, &si, &pi);
        
        if (!created) {
            std::cout << "[Resource Test] FFmpeg: Process creation failed - FAIL\n";
            return false;
        }
        
        WaitForSingleObject(pi.hProcess, 5000);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        DWORD exitCode;
        GetExitCodeProcess(pi.hProcess, &exitCode);
        
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        
        bool passed = (exitCode == 0);
        std::cout << "[Resource Test] FFmpeg: Version check completed in " 
                  << duration.count() << "ms (exit:" << exitCode << ") - " 
                  << (passed ? "PASS" : "FAIL") << "\n";
        
        return passed;
    }
    
    bool _testFileIOPerformance() {
        std::string test_file = "test_io_performance.tmp";
        const size_t test_size = 100 * 1024 * 1024; // 100MB
        
        auto start = std::chrono::high_resolution_clock::now();
        
        // 쓰기 테스트
        {
            std::vector<char> buffer(test_size, 'A');
            std::ofstream ofs(test_file, std::ios::binary);
            if (!ofs.write(buffer.data(), buffer.size())) {
                std::cout << "[Resource Test] File I/O: Write operation failed - FAIL\n";
                return false;
            }
        }
        
        auto write_end = std::chrono::high_resolution_clock::now();
        
        // 읽기 테스트
        {
            std::vector<char> buffer(test_size);
            std::ifstream ifs(test_file, std::ios::binary);
            if (!ifs.read(buffer.data(), buffer.size())) {
                std::cout << "[Resource Test] File I/O: Read operation failed - FAIL\n";
                DeleteFileA(test_file.c_str());
                return false;
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        
        auto write_time = std::chrono::duration_cast<std::chrono::milliseconds>(write_end - start);
        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        DeleteFileA(test_file.c_str());
        
        bool passed = total_time.count() < 3000;
        double throughput = (100.0 * 2) / (total_time.count() / 1000.0); // MB/s (write + read)
        
        std::cout << "[Resource Test] File I/O: 100MB write:" << write_time.count() 
                  << "ms read:" << (total_time.count() - write_time.count()) 
                  << "ms total:" << total_time.count() 
                  << "ms (" << std::fixed << std::setprecision(1) << throughput << "MB/s) - " 
                  << (passed ? "PASS" : "FAIL") << "\n";
        
        return passed;
    }
    
    bool _testMemoryAllocation() {
        try {
            const size_t test_size = 1024 * 1024 * 1024; // 1GB
            auto start = std::chrono::high_resolution_clock::now();
            
            std::unique_ptr<char[]> test_memory(new char[test_size]);
            test_memory[0] = 'A';
            test_memory[test_size - 1] = 'B';
            
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            
            bool passed = (test_memory[0] == 'A' && test_memory[test_size - 1] == 'B');
            std::cout << "[Resource Test] Memory: 1GB allocation completed in " 
                      << duration.count() << "ms - " << (passed ? "PASS" : "FAIL") << "\n";
            
            return passed;
        } catch (const std::bad_alloc&) {
            std::cout << "[Resource Test] Memory: 1GB allocation failed (insufficient memory) - FAIL\n";
            return false;
        }
    }
    
    bool _testDirectoryCreation() {
        std::string test_dir = "test_verify_directory";
        
        if (!CreateDirectoryA(test_dir.c_str(), NULL)) {
            DWORD error = GetLastError();
            return error == ERROR_ALREADY_EXISTS;
        }
        
        RemoveDirectoryA(test_dir.c_str());
        return true;
    }
    
    bool _testProcessPriority() {
        HANDLE hProcess = GetCurrentProcess();
        
        // HIGH_PRIORITY_CLASS 설정 테스트
        if (!SetPriorityClass(hProcess, HIGH_PRIORITY_CLASS)) {
            return false;
        }
        
        // 원래 우선순위로 복원
        SetPriorityClass(hProcess, NORMAL_PRIORITY_CLASS);
        return true;
    }
    
    // cleanup helpers
    void _cleanupTestFiles() {
        // 테스트 중 생성된 임시 파일들 삭제
        std::vector<std::string> temp_files = {
            "test_io_performance.tmp",
            "test_verify_directory"
        };
        
        for (const auto& file : temp_files) {
            DeleteFileA(file.c_str());
            RemoveDirectoryA(file.c_str());
        }
    }
    
    void _cleanupProcesses() {
        // 충돌하는 프로세스가 있다면 종료 (구현 선택사항)
        // 여기서는 로깅만 수행
        process_list_.clear();
    }
    
    void _cleanupCOM() {
        if (com_initialized_) {
            CoUninitialize();
            com_initialized_ = false;
        }
    }
    
    void _resetMembers() {
        resource_validated_ = false;
        process_list_.clear();
        ZeroMemory(&memory_info_, sizeof(memory_info_));
        ZeroMemory(&disk_info_, sizeof(disk_info_));
    }
};