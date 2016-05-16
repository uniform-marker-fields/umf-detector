#ifndef __UMF_UTIL_OPENCV_FACTORY_H__
#define __UMF_UTIL_OPENCV_FACTORY_H__

#include "../defines.h"
#include "image.h"
#include "stream_factory.h"

#ifdef UMF_USE_OPENCV_MINIMUM

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <string>

namespace umf
{

typedef struct
{
    bool file;
    std::string filename;
    int cameraIndex;
	int width;
	int height;
} CVImageInitStruct;

/**
 * Load images, videos or camera using OpenCV library
 */

class OpenCVImageFactory: public ImageFactory
{
public:
    virtual ~OpenCVImageFactory() {}

    virtual int getImage(ImageRGB *, bool internalBuffer = true);
    virtual int init(void *);
    virtual void release();
private:
    CvCapture *cam;
    IplImage *prevImg;
    bool inputImage;
	bool imageRead;
    bool inputFile;
};

}


#endif //use opencv

#endif
