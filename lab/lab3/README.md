### 실험 목적

frame의 유실이 0일 수 있는가?


### 실행

build
```sql
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab3\main.cpp /Fe:lab\lab3\bin\main.exe mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib propsys.lib ole32.lib
```

run
```
lab\lab3\bin\main.exe
```


### 결과

***1st***

[7] Format: {32595559-0000-0010-8000-00AA00389B71}, Resolution: 160x90, FPS: 30

NV12 + 160x90 + fps 30