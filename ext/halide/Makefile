CXX ?= g++

INC_HALIDE = $(HALIDE_PATH)/include
LIB_HALIDE = $(HALIDE_PATH)/bin

ANDROID_DSTFOLDER := ../src/
HL_MODULES = convertYCbCr_x.o getChromeMaskNV21_tuple.o getChromeMaskYCbCr_map_x.o getChromeMaskYCbCr_mask_x.o smooth3x3_ui8.o computeGradients3x3_Y.o computeGradients3x3_X.o
HL_MODULES_S = $(HL_MODULES:%.o=%.s)
TARGET = x86-64-avx
TARGET_ANDROID = arm-32-android
DEBUG_CODEGEN = 0
THREADS =1
THREADS_ANDROID = 4
TRACE=0

all: native_lib native_s native_h

native_x: native_x.cpp
	$(CXX) native_x.cpp -I $(INC_HALIDE) $(LIB_HALIDE)/libHalide.a -o native_x -lpthread -ldl

$(HL_MODULES): native_x
	HL_TRACE=$(TRACE) HL_TARGET=$(TARGET) HL_NUMTHREADS=$(THREADS) HL_DEBUG_CODEGEN=$(DEBUG_CODEGEN) ./native_x

$(HL_MODULES_S): native_x
	HL_TRACE=$(TRACE) HL_TARGET=$(TARGET_ANDROID) HL_NUMTHREADS=$(THREADS_ANDROID) HL_DEBUG_CODEGEN=$(DEBUG_CODEGEN) ./native_x

native_lib: ../lib/native_x.a

../lib/native_x.a: $(HL_MODULES)
	ar rcs $@ $^
	
native_s: $(HL_MODULES_S)
	mkdir -p $(ANDROID_DSTFOLDER)
	cp *.s $(ANDROID_DSTFOLDER)

native_h: native_lib native_s
	cp *.h ../include/native/

clean:
	rm ../lib/native_x.a *.s *.o *.h native_x
	rm ../include/native/*.h
	rm $(ANDROID_DSTFOLDER)*.s
