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

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab3\main.cpp /Fe:lab\lab3\bin\main.exe mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib propsys.lib ole32.lib
```

run
```
lab\lab3\bin\main.exe --camera_index 0 --frame_rate 30 --resolution 720p --pixel_format MJPG
```

### 결과

***1st***

| 구성             | 수신 프레임 수 | 기존 프레임 수 |
| --------------- | -------- | -------- |
| PC + Logitech   | 862~1    | 863      |
| LG + Logitech   | 845      | X        |
| LG + inner cam. | 896      | 897      |

Queue 도입 이후 프레임 수신은 환경과 관계없이 거의 완벽하게 작동하며, 첫 프레임 유실 외에는 프레임 드롭이 없는 안정적인 구조로 판단된다.

+LG의 inner cam과 logitech의 cam은 어떤 사양이 상이한 결과로 촬영 결과물이 크게 차이나는지에 대한 조사가 필요하다.