cmd.exe /k "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"


cl /std:c++17 /EHsc test/arducam/arducam_test.cpp test/arducam/argtable3/argtable3.c /I test/arducam/argtable3 /I "C:/Users/user/Workspace/evk_sdk/include" /link /LIBPATH:"C:/Users/user/Workspace/evk_sdk/lib" arducam_evk_cpp_sdk.lib /OUT:test/arducam/bin/arducam_test.exe


test\arducam\bin\arducam_test.exe