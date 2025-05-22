### to build

(remote ssh)

```
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

where cl

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil multiCameraController/main.cpp ^
    /Fe:capture.exe ^
    mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib ole32.lib uuid.lib

dir capture.exe
```

### to run

(remote ssh)

ffmpeg path: C:\ffmpeg\bin\ffmpeg.exe

```
capture.exe --camera_indexes <camera_indexes> --frame_rate <frame_rate> --warmup_time <warmup_time> --duration_time <duration_time> --pixel_format <pixel_format> --resolution <resolution> --ffmpeg_path <ffmpeg_path> --output_path <ffmpeg_path>
```

capture.exe --camera_indexes 0 --frame_rate 30 --warmup_time 20 --pixel_format MJPG --resolution 720p --ffmpeg_path C:\ffmpeg\bin\ffmpeg.exe --output_path .\output\result.mp4