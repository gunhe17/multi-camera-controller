### 실험 목적

media foundation이 지원하는 option 중, 사용하는 촬영 기기에 가장 적합한 option을 선택하기 위해 구현됨.


### 실행

build
```sql
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab1\main.cpp /Fe:lab\lab1\bin\main.exe mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib propsys.lib ole32.lib
```

run
```
lab\lab1\bin\main.exe
```


### 결과

***1st***

[7] Format: {32595559-0000-0010-8000-00AA00389B71}, Resolution: 160x90, FPS: 30

NV12 + 160x90 + fps 30

[294] Format: MJPG, Resolution: 1280x720, FPS: 30
[296] Format: MJPG, Resolution: 1280x720, FPS: 24
[298] Format: MJPG, Resolution: 1280x720, FPS: 20
[300] Format: MJPG, Resolution: 1280x720, FPS: 15
[302] Format: MJPG, Resolution: 1280x720, FPS: 10
[304] Format: MJPG, Resolution: 1280x720, FPS: 7.5
[306] Format: MJPG, Resolution: 1280x720, FPS: 5