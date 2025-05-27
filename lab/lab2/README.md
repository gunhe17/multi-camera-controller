### 배경

촬영 기기와 program을 연결하기 이전, 올바른 WindowMediaFoundation의 구현을 시험하고자 구현하게 되었다.

### 실험 설계

기능의 정상적인 작동 확인

### 실행

build
```sql
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab2\main.cpp /Fe:lab\lab2\bin\main.exe ^
    mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib
```

run
```
lab\lab2\bin\main.exe --camera_index 0 --frame_rate 30 --resolution 720p --pixel_format MJPG
```


### 결과

***1st***

(lab\lab2\bin\main.exe --camera_index 0 --frame_rate 30 --resolution 720p --pixel_format MJPG) \
pixel_format의 경우 MJPG를 사용했을 때 프레임의 수집 갯수가 더 많았다.

NV12 - 853
MJPG - 863

***Issue***:    
    프레임 속도에 비례해 일정 간격으로 지연이 반복되는 현상이 관찰되었다. 30fps의 경우 약 10f, 10fps의 경우 약 3-4f을 주기로 지연이 반복되는 현상이었다.

    - [ ]  🤔:
        - [ ]  HW 촬영 장치의 문제인가?
            - [ ]  Logitech cam → O / 약 10개를 주기로 한다. 총 863개의 frame이 수집된다.
            - [ ]  LG gram cam → O / 약 12개를 주기로 한다. 총 897개의 frame이 수집된다.
            
            → 촬영 기기의 성능이 영향을 미치는 걸까? 대역폭의 문제일까?