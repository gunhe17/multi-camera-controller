### 실험 목적

Microsoft-Windows-USB-UCX가 수신하는 event와 Media Foundation의 readSample()은 일치할 것인가?


### 실행

build
```sql
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab2\main.cpp /Fe:lab\lab2\bin\main.exe mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib propsys.lib ole32.lib
```

run
```
lab\lab2\bin\main.exe
```


### 결과
