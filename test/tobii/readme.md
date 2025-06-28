# Windows (Visual Studio)
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /std:c++17 /EHsc /I "C:\Users\user\Workspace\TobiiPro\64\include" test/tobii/tobii_test.cpp tobii_research.lib /link /LIBPATH:"C:\Users\user\Workspace\TobiiPro\64\lib" /OUT:test/tobii/bin/tobii_test.exe

test\tobii\bin\tobii_test.exe