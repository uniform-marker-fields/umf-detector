#ifndef CHROMA_CHROMAKEY_HPP_
#define CHROMA_CHROMAKEY_HPP_

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <vector>


#ifndef CHROMA_EXPORT
#if defined(__WIN32__) || defined(_WIN32)
#define CHROMA_EXPORT __declspec(dllexport)
#else
#define CHROMA_EXPORT
#endif
#endif

namespace chroma
{
	CHROMA_EXPORT void getChromeMask(IplImage *imgYCbCr, IplImage *mask);
	CHROMA_EXPORT void getChromaMaskHighQ(IplImage *imgYCrCb, IplImage *mask, std::vector<cv::Vec2b> cbcrMedian);
}

#endif

