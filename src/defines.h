#ifndef __UMF_DEFINES_H__
#define __UMF_DEFINES_H__


#ifndef UMF
#if defined(__WIN32__) || defined(_WIN32)
#define UMF __declspec( dllexport )
#else
#define UMF
#endif
#endif

#ifdef ANDROID
#define UMF_ANDROID
#endif

#ifndef UMF_USE_NATIVE
//#define UMF_USE_NATIVE
#endif

#if !defined(UMF_ANDROID)
#define UMF_USE_OPENCV
#define UMF_USE_OPENCV_MINIMUM
#endif

#if defined(ANDROID_CV)
#define UMF_ANDROID
#define UMF_USE_OPENCV
#endif


#ifndef UMF_USE_GLFW
//#define UMF_USE_GLFW
#endif

#ifdef UMF_ANDROID
#define UMF_USE_GLES
#endif

#ifdef UMF_USE_OPENCV_MINIMUM
#define DRAW_CV
#endif

#if defined(UMF_USE_GLFW) || defined(UMF_USE_GLES)
#define DRAW_GL
#undef DRAW_CV
#endif


#ifndef HAVE_FIREWIRE_LIB
//#define HAVE_FIREWIRE_LIB
#endif

#define UMF_DEBUG

#ifdef UMF_DEBUG
//#define UMF_DEBUG_COUNT_PIXELS
#define UMF_DEBUG_DRAW
//#define UMF_DEBUG_TIMING
//#define UMF_DEBUG_MSG
#endif

#include <stdint.h>

#include "util/fixed_class.h"

namespace fixp{
    using namespace fixedpoint;
}


namespace umf {


//clockwise rotation position
//    0
//270 .  90
//   180
enum LOCATION_ROTATION {
    LOC_ROT_0 = 0,
    LOC_ROT_90,
    LOC_ROT_180,
    LOC_ROT_270,
    LOC_ROT_COUNT
};

struct Location
{
    int r, c, rotation;

    static Location invalid;

    inline bool operator==(const Location &other) const {
        return (this->r == other.r) && (this->c == other.c) && (this->rotation == other.rotation);
    }

};


enum EDGE_TYPE_CODE {
    EDGE_DIRECTION_EQUAL = 0,
    EDGE_DIRECTION_RIGHTDOWN,
    EDGE_DIRECTION_LEFTUP,
    EDGE_DIRECTION_INVALID,
    EDGE_DIRECTION_COUNT
};

typedef int8_t EdgeType;


}


#endif
