### 배경

FFMPEG의 올바른 설계를 시험하기 위해 구현되었다.

### 실험 설계


### 실행

build
```sql
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab5\main.cpp /Fe:lab\lab5\bin\main.exe
```

run
```
lab\lab5\bin\main.exe --camera_index 0 --frame_rate 30 --resolution 720p --pixel_format MJPG
```

### 결과

***1st***

