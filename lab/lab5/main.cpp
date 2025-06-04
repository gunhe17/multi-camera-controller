#include <windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>


int main() {
    SECURITY_ATTRIBUTES sa = { sizeof(SECURITY_ATTRIBUTES), NULL, TRUE };
    HANDLE readHandle = NULL, writeHandle = NULL;

    if (!CreatePipe(&readHandle, &writeHandle, &sa, 0)) {
        std::cerr << "[Test] 파이프 생성 실패\n";
        return 1;
    }

    STARTUPINFOA si = { sizeof(si) };
    si.dwFlags |= STARTF_USESTDHANDLES;
    si.hStdInput = readHandle;
    si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
    si.hStdError = GetStdHandle(STD_ERROR_HANDLE);

    PROCESS_INFORMATION pi = {};
    std::string cmd =
        "ffmpeg -y -f mjpeg -framerate 10 -i - -t 3 -c:v copy out.mkv";

    if (!CreateProcessA(NULL, const_cast<LPSTR>(cmd.c_str()),
                        NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi)) {
        std::cerr << "[Test] FFmpeg 실행 실패\n";
        return 1;
    }

    CloseHandle(readHandle);  // FFmpeg가 읽는 쪽 핸들은 닫음

    std::ifstream file("sample.jpeg", std::ios::binary);
    std::vector<BYTE> jpeg_data((std::istreambuf_iterator<char>(file)), {});

    for (int i = 0; i < 10; ++i) {
        DWORD written;
        WriteFile(writeHandle, jpeg_data.data(), jpeg_data.size(), &written, NULL);
        std::cout << "[Test] Frame " << i + 1 << " 전송됨\n";
        Sleep(100);
    }

    std::cout << "[Test] 파이프 닫음\n";
    CloseHandle(writeHandle);

    WaitForSingleObject(pi.hProcess, INFINITE);
    std::cout << "[Test] FFmpeg 종료 완료\n";

    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    return 0;
}
