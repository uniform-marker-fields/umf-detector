#include "chroma.hpp"
#include <math.h>


#ifndef M_PI_4
#define M_PI_4 0.78539816339744830961566084581988
#endif

#ifndef M_PI 
#define M_PI       3.14159265358979323846
#endif

#ifdef _DEBUG
#define CV_DEBUG_SUFFIX "d"
#else 
#define CV_DEBUG_SUFFIX
#endif

#define CV_SHORTVERSION CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)  CV_DEBUG_SUFFIX
#pragma comment(lib, "opencv_core" CV_SHORTVERSION ".lib")
#pragma comment(lib, "opencv_imgproc" CV_SHORTVERSION ".lib")

namespace chroma
{
 

void getChromeMask(IplImage *imgYCbCr, IplImage *mask)
{

	int width = imgYCbCr->width;
	int height = imgYCbCr->height;
	const int BLUENESS_THRESH = 120;
	const int REDNESS_THRESH = 120;
	const int MASK_THRESH = 100;

	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			int invCb = 255 - static_cast<int>(((unsigned char*)(imgYCbCr->imageData))[y*imgYCbCr->widthStep + x*imgYCbCr->nChannels + 1]);
			int invCr = 255 - static_cast<int>(((unsigned char*)(imgYCbCr->imageData))[y*imgYCbCr->widthStep + x*imgYCbCr->nChannels + 2]);
			
			if(mask)
			{
				int maskval = (invCb - BLUENESS_THRESH)*(invCr - REDNESS_THRESH);
				unsigned char* p = &(((unsigned char*)(mask->imageData))[y*mask->widthStep + x]);
				*p = static_cast<unsigned char>(maskval > MASK_THRESH?255:0);
			}
		}
	}

	if(mask)
	{
		cvErode(mask, mask);
		cvDilate(mask, mask);
		cvDilate(mask, mask);
		cvSmooth(mask, mask, CV_GAUSSIAN, 5, 0);
	}

}

float step(float threshold, float value) {
	return (value > threshold) ? 1.f : 0.f;
}

float smoothstep(float lowthresh, float highthresh, float value) {
	return (value > highthresh) ? 1.0 : ( (value < lowthresh) ? 0.0 : ( (value - lowthresh)/(highthresh - lowthresh)) );
}

/**
 * Replicate the shader functionality. The chroma colors are passed as CbCr chanels cbcrMedian.
 */ 
void getChromaMaskHighQ(IplImage *imgYCrCb, IplImage *mask, std::vector<cv::Vec2b> cbcrMedian)
{
	int colorCount = cbcrMedian.size();
	int width = imgYCrCb->width;
	int height = imgYCrCb->height;

	const float angleThreshold = tan(10.0*M_PI/180);
	const float angleThreshold2 = tan(25.*M_PI/180);

	const float angleThresholdInv = 1. / angleThreshold;
	const float angleThreshold2Inv = 1. / angleThreshold2;

	std::vector<cv::Mat> rotations(colorCount);
	for (int colorIndex = 0; colorIndex < colorCount; colorIndex++) {
		const float keyCb = static_cast<float>(cbcrMedian[colorIndex][0]) / 128.0f - 0.98;
		const float keyCr = static_cast<float>(cbcrMedian[colorIndex][1]) / 128.0f - 0.98;

		const float keyl = sqrt(keyCb*keyCb + keyCr*keyCr);
		
		const float cosKey = keyCb / keyl;
		const float sinKey = keyCr / keyl;

		cv::Mat xzRot(2, 2, CV_32FC1);
		xzRot.at<float>(0, 0) = cosKey;
		xzRot.at<float>(0, 1) = -sinKey;
		xzRot.at<float>(1, 0) = sinKey;
		xzRot.at<float>(1, 1) = cosKey;

		rotations[colorIndex] = xzRot;
	}

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			unsigned char Crchar = ((unsigned char*)(imgYCrCb->imageData))[y*imgYCrCb->widthStep + x*imgYCrCb->nChannels + 1];
			unsigned char Cbchar = ((unsigned char*)(imgYCrCb->imageData))[y*imgYCrCb->widthStep + x*imgYCrCb->nChannels + 2];

			float cr = static_cast<float>(Crchar) / 128.0f - 0.98f;
			float cb = static_cast<float>(Cbchar) / 128.0f - 0.98f;

			float alpha = 1.f;

			for (int colorIndex = 0; colorIndex < colorCount; colorIndex++) {
				const cv::Mat xzRot = rotations[colorIndex];
				//rotate by the rotation matrix
				cv::Vec2f xzColor(cb*xzRot.at<float>(0, 0) + cr*xzRot.at<float>(1, 0), cb*xzRot.at<float>(0, 1) + cr*xzRot.at<float>(1, 1));

				xzColor *= 2.f;

				float kfg = xzColor[0] - fabs(xzColor[1])/angleThreshold2;
				float maskValue = 1.f;
				if (xzColor[1] != 0) {
					maskValue = smoothstep(0.05, 0.5, kfg);/*smoothstep(angleThreshold2Inv, angleThresholdInv, xzColor[0] / abs(xzColor[1]));*/
				}
				else {
					maskValue = smoothstep(0.05, 0.5, kfg);
				}

				float currAlpha = 1.f - maskValue;
				if (currAlpha < alpha) {
					alpha = currAlpha;
				}
			}
			
			if (mask)
			{
				unsigned char foregroundByte = static_cast<unsigned char>(alpha * 255);
				unsigned char* p = &(((unsigned char*)(mask->imageData))[y*mask->widthStep + x]);
				*p = foregroundByte;
			}
		}
	}

}


} //end of namespace

