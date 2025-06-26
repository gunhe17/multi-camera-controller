cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /std:c++17 /EHsc /W3 /O2 multiVerifier/main.cpp /Fe:multiVerifier/bin/verify.exe

multiVerifier\bin\verify.exe