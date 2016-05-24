:: Windows version of halide

::set path
echo "setting path"
set PATH=%PATH%;%VSDEV_BIN32_PATH%

:: Windows version of halide 32 bit
pushd %1
SET HL_TARGET=x86-32-avx
SET HL_NUMTHREADS=4
SET HL_DEBUG_CODEGEN=1
echo "calling native application"
start /wait ./Native.exe 
echo "building library"
lib /OUT:"%2\lib32\native_x.lib" *.obj /MACHINE:X86 /NOLOGO
echo "copying headers"
move /Y *.h %2\include\native

