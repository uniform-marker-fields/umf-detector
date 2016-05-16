#include "stream_factory.h"
#include "opencv_factory.h"
#include "firewire_factory.h"
#include "../defines.h"

namespace umf {

ImageFactory *StreamFactory::GetImageFactory(string type)
{
#ifdef UMF_USE_OPENCV_MINIMUM
    if (type == "OPENCV") return new OpenCVImageFactory();
#endif
#ifdef HAVE_FIREWIRE_LIB
    if (type == "IEEE1394") return new FirewireFactory();
#endif
    return NULL;
}

}
