### to build

(remote ssh)

```
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

where cl

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil multiCameraController/main.cpp /Fe:multiCameraController/bin/main.exe mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib ole32.lib uuid.lib

dir capture.exe
```

### to run

(remote ssh)
shutdown /r /t 0

ffmpeg path: C:\ffmpeg\bin\ffmpeg.exe

```
multiCameraController\bin\main.exe
```