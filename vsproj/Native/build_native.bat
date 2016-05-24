:: Windows version of halide 64 bit
pushd %1
SET HL_TARGET=x86-64-avx
SET HL_NUMTHREADS=4
SET HL_DEBUG_CODEGEN=1
echo "calling native application"
start /wait ./Native.exe 
echo "building library"
lib /OUT:"%2\lib\native_x.lib" /MACHINE:X64 /NOLOGO *.obj
echo "copying headers"
move /Y *.h %2\include\native

:: Windows version of halide 32 bit
::pushd %1
del *.obj
SET HL_TARGET=x86-32-avx
SET HL_NUMTHREADS=1
SET HL_DEBUG_CODEGEN=1
echo "calling native application 32bit"
start /wait ./Native.exe 
echo "building 32bit library"
lib /OUT:"%2\lib32\native_x.lib" *.obj /MACHINE:X86 /NOLOGO


:::: Android stuff
::echo "Android sources"
::SET HL_TARGET=arm-32-android
::SET HL_NUMTHREADS=2
::SET HL_DEBUG_CODEGEN=1
::start /wait ./Native.exe
::move /Y *.s %2\src\
