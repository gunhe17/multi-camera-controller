## How to run

build & run
```
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /std:c++17 /EHsc /W3 /O2 /D_CRT_SECURE_NO_WARNINGS /wd4819 multiCameraController\main.cpp mf.lib mfplat.lib mfreadwrite.lib mfuuid.lib ole32.lib /Fe:multiCameraController\bin\multi_camera.exe

multiCameraController\bin\multi_camera.exe --camera_indices 0 --record_duration 5
```

video to img
```
ffmpeg -i output.avi frames/frame_%06d.png
```

<br>

## About code

### Exception Rule

- `raise`
    <br>: .exe를 중단해야하는 경우

<br>

- `std::cout + return`
    <br>: .exe를 중단하지 않아도 되는 경우

*C++의 throw()는 호출 스택을 따라 상위 함수로 전파되며, 가장 가까운 try{}의 catch{}에서 포착됩니다.

| 상속 계층                   | 예외 클래스 이름                           | 헤더 파일          | 주요 사용 목적 / 상황           |
| ----------------------- | ----------------------------------- | -------------- | ----------------------- |
| `std::exception`        | `std::exception`                    | `<exception>`  | 모든 표준 예외의 기반 클래스        |
| ├─ `std::logic_error`   | `std::logic_error`                  | `<stdexcept>`  | 논리적 프로그래밍 오류 (버그)       |
| │                       | ├─ `std::invalid_argument`          | `<stdexcept>`  | 잘못된 인자                  |
| │                       | ├─ `std::domain_error`              | `<stdexcept>`  | 정의역 오류 (e.g. sqrt(-1))  |
| │                       | ├─ `std::length_error`              | `<stdexcept>`  | 컨테이너 최대 길이 초과           |
| │                       | └─ `std::out_of_range`              | `<stdexcept>`  | 잘못된 인덱스 접근              |
| └─ `std::runtime_error` | `std::runtime_error`                | `<stdexcept>`  | 런타임 시점의 외부 오류           |
|                         | ├─ `std::range_error`               | `<stdexcept>`  | 수 표현 범위 초과              |
|                         | ├─ `std::overflow_error`            | `<stdexcept>`  | 산술 오버플로우                |
|                         | └─ `std::underflow_error`           | `<stdexcept>`  | 산술 언더플로우                |
|                         | `std::ios_base::failure`            | `<ios>`        | 파일, 입출력 실패              |
|                         | `std::filesystem::filesystem_error` | `<filesystem>` | 파일 시스템 작업 실패 (C++17 이상) |
|                         | `std::bad_alloc`                    | `<new>`        | 메모리 할당 실패 (`new` 실패)    |
|                         | `std::bad_cast`                     | `<typeinfo>`   | `dynamic_cast` 실패       |
|                         | `std::bad_typeid`                   | `<typeinfo>`   | `typeid` 사용 오류          |
|                         | `std::bad_exception`                | `<exception>`  | unexpected 예외 처리        |


### Code

각 class의 common 함수 정의.

init()

setup()

run()


**MultiManager**
- setup()
    - _barrier()
    - _manager()
- run()