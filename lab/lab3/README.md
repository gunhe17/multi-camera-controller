### 배경

IMFSourceReader의 비동기 방식(ReadSample + IMFSourceReaderCallback)을 사용하여,
KernelStream의 프레임 버퍼를 실시간으로 가져온 뒤, 사용자 정의 FrameQueue에 저장하는 구조의 호환성 및 안정성을 검증하기 위한 테스트를 수행하였습니다.

### 실험 설계

기법:
- 각 프레임의 타임스탬프 기록 및 저장
- 프레임 간 시간 간격(Δms) 분석
- null sample, OnReadSample 누락 여부 확인

### 실행

build
```sql
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab3\main.cpp /Fe:lab\lab3\main.exe ^
    mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib
```

run
```
lab\lab3\main.exe --camera_index 0 --frame_rate 30 --duration_time 30 --resolution 720p --pixel_format YUY2
```

### 결과

***1st***

우선은 IMFSourceReader의 callback 방식에 대한 검증이 필요하다. 모든 프레임을 유실 없이 잘 가져오는지 확인해야한다.


***2nd***

(lab\lab3\main.exe --camera_index 0 --frame_rate 30 --duration_time 300 --resolution 720p --pixel_format YUY2) \
frameQuene에 push하는 작업을 수행할 때와 생략할 때를 비교해보겠다.
