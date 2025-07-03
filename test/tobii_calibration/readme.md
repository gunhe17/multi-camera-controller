# Windows (Visual Studio)
cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

cl /std:c++17 /EHsc /I "C:\Users\user\Workspace\TobiiPro\64\include" test/tobii_calibration/tobii_callibration.cpp /link /LIBPATH:"C:\Users\user\Workspace\TobiiPro\64\lib" tobii_research.lib user32.lib gdi32.lib kernel32.lib /OUT:test/tobii_calibration/bin/tobii_callibration.exe

test\tobii_calibration\bin\tobii_callibration.exe
