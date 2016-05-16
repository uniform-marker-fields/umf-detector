#include "image.h"
#include "../defines.h"
#include "chromakey.h"

#ifdef UMF_USE_OPENCV

#include <opencv/cv.h>
#include <opencv/highgui.h>

#endif

namespace umf {

template <typename T>
static inline T clamp(T x, T a, T b)
{
    return x < a ? a : (x > b ? b : x);
}


template <class T, int NCHAN>
void getChromeMask(Image<T, NCHAN> * /*img*/, ImageGray *mask)
{
    if(mask)
    {
        memset(mask->data, 255, mask->height*mask->widthstep);
    }
}

template <>
void getChromeMask(ImageRGB *srcRGB, ImageGray *mask)
{
    if(srcRGB == NULL || mask == NULL || srcRGB->width != mask->width || srcRGB->height != mask->height)
    {
        return;
    }

    for(int xi = 0; xi < srcRGB->width; xi++)
    {
        for(int yi = 0; yi < srcRGB->height; yi++)
        {
            unsigned char* pB = srcRGB->get2D(xi, yi);
            short R = *pB;
            short G = *(pB + 1);
            short B = *(pB + 2);

            int maskVal = static_cast<float>(G - std::max(B, R));
			maskVal *= 10;
			if(maskVal < 0) maskVal = 0;
			if(maskVal > 255) maskVal = 255;

            //first create the mask than convert the pixels
            *(mask->get2D(xi,yi)) = maskVal;
            //((uchar *)(mask->data + yi*mask->widthstep))[xi*mask->channels] = (uchar) clamp(maskVal*1e5, 0, 255);
        }
    }

    //TODO implement this for ours, too
#ifdef UMF_USE_OPENCV
	if(mask)
	{
		IplImage *maskCV = cvCreateImageHeader(cvSize(mask->width, mask->height), IPL_DEPTH_8U, 1);
		maskCV->widthStep = mask->widthstep;
		maskCV->imageData = maskCV->imageDataOrigin = mask->data;

		cvErode(maskCV, maskCV);
		cvDilate(maskCV, maskCV);
		cvDilate(maskCV, maskCV);
		cvSmooth(maskCV, maskCV, CV_GAUSSIAN, 5, 0);
		cvReleaseImageHeader(&maskCV);
	}
#endif
}

UMF void getChromeMaskYCbCr(ImageGray *srcYCbCr, ImageGray *mask, ImageGray *map)
{
	int width = srcYCbCr->width/3;
	int height = srcYCbCr->height;
	const int BLUENESS_THRESH = 105;
	const int REDNESS_THRESH = 120;
    //const int MASK_THRESH = 100;

	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			int invCb = 255 - static_cast<int>(*srcYCbCr->get2D(3*x + 1, y));
			int invCr = 255 - static_cast<int>(*srcYCbCr->get2D(3*x + 2, y));
			
			if(mask)
			{
				int maskval = (invCb - BLUENESS_THRESH)*(invCr - REDNESS_THRESH);
				unsigned char* p = mask->get2D(x, y);
				*p = static_cast<unsigned char>(maskval > 0 ? (maskval < 255 ? maskval : 255) : 0);
				//*p = static_cast<unsigned char>(maskval > MASK_THRESH?255:0);
			}

			if(map)
			{
				unsigned char* p = map->get2D(x, y);
                //unsigned char* intensity = srcYCbCr->get2D(3*x, y);
				
				//invCr = (invCr - 128)*3 + 128;

                int temp = 2*(invCb - invCr) + 128;
                //int temp = static_cast<int>(*intensity);
				*p = clamp(temp, 0, 255);
				/*
				*p = clamp(temp + static_cast<int>(*intensity), 0, 255);
				*(p + 1) = clamp(temp + static_cast<int>(*(intensity + 1)), 0, 255);
				*(p + map->widthstep) = clamp(temp + static_cast<int>(*(intensity + srcYUV->widthstep)), 0, 255);
				*(p + map->widthstep + 1) = clamp(temp + static_cast<int>(*(intensity + srcYUV->widthstep)), 0, 255);
				*/
			}
		}
	}

#ifdef UMF_USE_OPENCV

	if(map)
	{
		IplImage *cvimg = cvCreateImageHeader(cvSize(map->width, map->height), IPL_DEPTH_8U, 1);
		cvimg->widthStep = map->widthstep;
		cvimg->imageData = cvimg->imageDataOrigin = map->data;

		cvSmooth(cvimg, cvimg, CV_GAUSSIAN, 5, 0);
		//cvEqualizeHist(cvimg, cvimg);
		//cvConvertScale(cvimg, cvimg, 2.0, -128);
		cvReleaseImageHeader(&cvimg);
	}

	if(mask)
	{
		IplImage *maskCV = cvCreateImageHeader(cvSize(mask->width, mask->height), IPL_DEPTH_8U, 1);
		maskCV->widthStep = mask->widthstep;
		maskCV->imageData = maskCV->imageDataOrigin = mask->data;

		cvErode(maskCV, maskCV);
		cvDilate(maskCV, maskCV);
		cvDilate(maskCV, maskCV);
		cvSmooth(maskCV, maskCV, CV_GAUSSIAN, 5, 0);
		cvReleaseImageHeader(&maskCV);
	}
#endif
}

void getChromeMaskNV21(ImageGray *srcYUV, ImageGray *mask, ImageGray *map)
{
	int width = srcYUV->width;
	int height = srcYUV->height*2/3;
	const int BLUENESS_THRESH = 105;
	const int REDNESS_THRESH = 120;
	const int MASK_THRESH = 100;

	for(int y = 0; y < height/2; y++)
	{
		for(int x = 0; x < width/2; x++)
		{
			int invCb = 255 - static_cast<int>(*srcYUV->get2D(2*x, height + y));
			int invCr = 255 - static_cast<int>(*srcYUV->get2D(2*x + 1, height + y));
			
			if(mask)
			{
				int maskval = (invCb - BLUENESS_THRESH)*(invCr - REDNESS_THRESH);
				unsigned char* p = mask->get2D(2*x, 2*y);
				*p = *(p + 1) = *(p + mask->widthstep) = *(p + mask->widthstep + 1) = static_cast<unsigned char>(maskval > MASK_THRESH?255:0);
			}

			if(map)
			{
				unsigned char* p = map->get2D(2*x, 2*y);
				//unsigned char* intensity = srcYUV->get2D(2*x, 2*y);

				int temp = 2*(invCr - invCb) + 128;
				*p = *(p + 1) = *(p + map->widthstep) = *(p + map->widthstep + 1) = clamp(temp , 0, 255);
				/*
				*p = clamp(temp + static_cast<int>(*intensity), 0, 255);
				*(p + 1) = clamp(temp + static_cast<int>(*(intensity + 1)), 0, 255);
				*(p + map->widthstep) = clamp(temp + static_cast<int>(*(intensity + srcYUV->widthstep)), 0, 255);
				*(p + map->widthstep + 1) = clamp(temp + static_cast<int>(*(intensity + srcYUV->widthstep)), 0, 255);
				*/
			}
		}
	}
}

//SPECIALIZATION INSTANCES

template void getChromeMask(ImageGray*, ImageGray*);

}
