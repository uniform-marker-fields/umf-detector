LOCAL_PATH := $(call my-dir)
OPENCV_ANDROID_SDK := C:/LIBS/OpenCV-android-sdk

include $(CLEAR_VARS)

OPENCV_CAMERA_MODULES := off
OPENCV_LIB_TYPE:=STATIC

include $(OPENCV_ANDROID_SDK)/sdk/native/jni/OpenCV.mk

LOCAL_MODULE := UMFDetector

MY_FILES := umf_android.cpp decisiontree.cpp edgel_detector.cpp model.cpp edge_dir_detector.cpp\
grid_detector.cpp marker.cpp umf.cpp tracker.cpp
EXT_SOURCES := $(wildcard $(LOCAL_PATH)/../../../ext/src/*.s)
MY_SOURCE_FILES := $(wildcard $(LOCAL_PATH)/../../../src/util/*.cpp) $(MY_FILES:%=$(LOCAL_PATH)/../../../src/%) $(EXT_SOURCES)

LOCAL_SRC_FILES := $(MY_SOURCE_FILES:$(LOCAL_PATH)/%=%)
LOCAL_CXXFLAGS += -DANDROID_CV -std=c++0x -I $(LOCAL_PATH)/../../../ext/include -Wall --param max-inline-insns-single=2048
LOCAL_CXXFLAGS += -O3 -v
#LOCAL_CXXFLAGS += -marm -mfloat-abi=softfp -mfpu=vfp
#LOCAL_CXXFLAGS += -march=native -mfloat-abi=softfp -mfpu=neon
#LOCAL_CXXFLAGS += -g -ggdb -O3
#LOCAL_CFLAGS += -g -ggdb -O3
## profiler
#LOCAL_CXXFLAGS += -pg
LOCAL_LDLIBS    += -lm -llog -ljnigraphics -lGLESv2 -lGLESv1_CM -lEGL

include $(BUILD_SHARED_LIBRARY)

