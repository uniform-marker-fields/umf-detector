#ifndef __UMF_UTIL_FIREWIRE_FACTORY_H__
#define __UMF_UTIL_FIREWIRE_FACTORY_H__

#include "stream_factory.h"

#ifdef HAVE_FIREWIRE_LIB

#include <dc1394/dc1394.h>
#include <stdint.h>
#include <inttypes.h>

namespace umf
{

/**
 * Image factory for firewire connected cameras. Requires firewire library
 */
class FirewireFactory: public ImageFactory
{
    virtual ~FirewireFactory() {}

    virtual int getImage(Image<unsigned char, 3> *, bool internalBuffer = true);

    virtual int init(void *);
    virtual void release();
private:

    dc1394camera_t *camera;
    dc1394_t * d;
    dc1394video_mode_t videoMode;
    dc1394color_coding_t colorCoding;
    uint32_t bits;
    dc1394framerate_t dc1394Framerate;
    dc1394video_frame_t *prevFrame;
    bool convert;
};


}

#endif

#endif
