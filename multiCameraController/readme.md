
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /std:c++17 /EHsc /W3 /O2 /D_CRT_SECURE_NO_WARNINGS /wd4819 multiCameraController\main.cpp mf.lib mfplat.lib mfreadwrite.lib mfuuid.lib ole32.lib /Fe:multiCameraController\bin\multi_camera.exe

multiCameraController\bin\multi_camera.exe --camera_indices 0 --record_duration 5

ffmpeg -i output.avi frames/frame_%06d.png
