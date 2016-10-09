Environment variables that should be set and example settings:

VSDEV_INC_PATH=D:\LIB\argtable2\include;D:\LIB\Halide\include;D:\LIB\opencv-2.4.13\build\include
VSDEV_LIB32_PATH=D:\LIB\argtable2\lib32;D:\LIB\Halide\Release;D:\LIB\opencv-2.4.13\build\x86\vc12\lib
VSDEV_BIN_PATH=D:\LIB\opencv-2.4.13\build\x86\vc12\bin;D:\LIB\Halide\Release

Don't forget to add BIN path to system path: ;%VSDEV_BIN_PATH%