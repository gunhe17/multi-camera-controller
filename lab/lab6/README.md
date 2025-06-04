### 배경

PC와 촬영 기기 중 프레임 지연 및 유실의 책임이 누구에게 있는지 실험하기 위해 구현되었다.

### 실험 설계

### 실행

build
```sql
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil lab\lab6\main.cpp /Fe:lab\lab6\bin\main.exe mfplat.lib mf.lib mfreadwrite.lib mfuuid.lib propsys.lib ole32.lib
```

event desc
```
wevtutil gp "Microsoft-Windows-USB-UCX" /f:xml > ucx_manifest.xml
```

run
```
lab\lab6\bin\main.exe
```


### 결과

***1st***

장비?

