### 배경

촬영 기기와 program을 연결하기 이전, 올바른 WindowMediaFoundation의 구현을 시험하고자 구현하게 되었다.

### 실험 설계

기능의 정상적인 작동 확인

### 실행

build
```sql
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab2\media_inspector.cpp /Fe:lab\lab2\bin\media_inspector.exe ^
    mfplat.lib mfreadwrite.lib mf.lib mfuuid.lib ole32.lib

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab2\main.cpp /Fe:lab\lab2\bin\main.exe ^
    mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib
```

run
```
lab\lab2\bin\media_inspector.exe

lab\lab2\bin\main.exe --camera_index 0 --frame_rate 30 --resolution 720p --pixel_format NV12
```


### 결과

***1st***

media_inspector.cpp의 출력 결과를 이용해 장치의 올바른 setting을 구현한다.
(lab\lab2\bin\media_inspector.exe)

***result***:
| 항목 | 값 |
| ---------------- | ------------------------------------- |
| **해상도** | 1280x720 |
| **프레임레이트** | 30 fps |
| **서브타입 (픽셀 포맷)** | `3231564E` → **NV12** |
| **인터레이스 모드** | 2 = `MFVideoInterlace_MixedInterlaceOrProgressive` |
| **샘플 크기** | (미출력: 일반적으로 NV12는 설정 안 해도 됨) |

| 항목               | 값                                   |
| ---------------- | ----------------------------------- |
| **해상도**          | 1280x720                            |
| **프레임레이트**       | 30 fps                              |
| **서브타입 (픽셀 포맷)** | `47504A4D` → **MJPG (Motion JPEG)** |
| **인터레이스 모드**     | 2                                   |
| **샘플 크기**        | 2,764,800 bytes (약 2.6MB/frame)     |


***2nd***

(lab\lab2\bin\main.exe --camera_index 0 --frame_rate 30 --resolution 720p --pixel_format NV12) \
frameQuene에 push하는 작업을 수행할 때와 생략할 때를 비교해보겠다.
