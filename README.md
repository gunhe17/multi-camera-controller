### to build

(remote ssh)

```
cmd.exe /k ""C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat""

where cl

cl /EHsc /std:c++17 /Iinclude /Icapture /Iutil main.cpp /Fe:capture.exe

dir capture.exe
```