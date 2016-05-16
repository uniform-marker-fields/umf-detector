#ifndef __UMF_UTIL_STREAM_FACTORY_H__
#define __UMF_UTIL_STREAM_FACTORY_H__

#include <string>
#include "../defines.h"
#include "image.h"

using namespace std;

namespace umf {

// Abstract base class for accessing images
class UMF ImageFactory {
public:

    virtual ~ImageFactory() {}

    virtual int getImage(ImageRGB *, bool internalBuffer = true) = 0;

    virtual int init(void *) = 0;
    virtual void release() = 0;
    inline int getWidth() { return this->width; }
    inline int getHeight() { return this->height; }
    inline int getChannelCount() {return this->channels; }
    inline float getFrameCount() { return this->frameCount; }
protected:
    int width;
    int height;
    int channels;
    float frameCount;
};

// Use the static function to get factories
// possible types: OPENCV, IEEE1394 (firewire library required)
// For android see the android project
class UMF StreamFactory
{
public:
    static ImageFactory *GetImageFactory(string type);
};



}

#endif
