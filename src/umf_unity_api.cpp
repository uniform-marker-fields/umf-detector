
#include <string>
#include "defines.h"
#include "umf_unity_api.h"
#include "umf.h"
#include "util/image.h"
#include "util/umfdebug.h"
#include "util/native_x.h"

using namespace umf;

#ifdef UMF_USE_OPENCV_MINIMUM

#include <opencv/cv.h>
#include <opencv/highgui.h>

#ifdef _DEBUG
#define CV_DEBUG_SUFFIX "d"
#else 
#define CV_DEBUG_SUFFIX
#endif

#ifdef __OPENCV_OLD_CV_H__

	#ifdef CV_MAJOR_VERSION
	//if compiled in releas mode remove the "d" suffix
		#define CV_SHORTVERSION CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)  CV_DEBUG_SUFFIX
		#pragma comment(lib, "opencv_core" CV_SHORTVERSION ".lib")
		#pragma comment(lib, "opencv_highgui" CV_SHORTVERSION ".lib")
		#pragma comment(lib, "opencv_imgproc" CV_SHORTVERSION ".lib")
		#pragma comment(lib, "opencv_features2d" CV_SHORTVERSION ".lib")
		#pragma comment(lib, "opencv_calib3d" CV_SHORTVERSION ".lib")
		#pragma comment(lib, "opencv_video" CV_SHORTVERSION ".lib")
	#else
		#pragma comment(lib, "opencv_core220.lib")
		#pragma comment(lib, "opencv_highgui220.lib")
		#pragma comment(lib, "opencv_imgproc220.lib") 
		#pragma comment(lib, "opencv_features2d220.lib") 
	#endif

#else
	#pragma comment(lib, "cxcore210.lib")
	#pragma comment(lib, "cv210.lib")
	#pragma comment(lib, "highgui210.lib") 
#endif

#endif

#pragma comment(lib, "native_x.lib")


/* DETECTOR ************************************************************/

UMF int __stdcall umf_set_frame(DetectorProperties* detector, unsigned char* frame)
{
    ImageRGB *currentImage = (ImageRGB *) detector->currentImg;
    int SRC_CHANNELS = 4;
    
    char *pd = currentImage->data;
    char *pf = (char*) frame;
    
    pf += (detector->textureHeight - 1)*detector->textureWidth*SRC_CHANNELS;

    //it's upside down
    for(int i = 0; i < detector->textureHeight; i++)
    {
        for(int j = 0; j < detector->textureWidth; j++)
        {

            *(pd++) = *(pf++);
            *(pd++) = *(pf++);
            *(pd++) = *(pf++);
            pf++; //rgba
        }
        
        pf -= 2*detector->textureWidth*SRC_CHANNELS;
    }
    
    return 1;
}

UMF int __stdcall umf_detect(DetectorProperties *detector, float timeout)
{
	static int frame = 0;
	frame++;

	DetectorResult *result = (DetectorResult *) detector->currentResult;

	UMFDetector<UMF_DETECTOR_CHANNELS> *umfDet = (UMFDetector<UMF_DETECTOR_CHANNELS> *) detector->detector;
    ImageRGB *img = (ImageRGB *) detector->currentImg;

	if(umfDet != NULL && img != NULL)
	{
        UMFDebug *dbg = UMFDSingleton::Instance();
        dbg->setRenderer(NULL);

#if UMF_DETECTOR_CHANNELS == 1

		ImageGray *grayImg;
		if ((umfDet->getFlags() & UMF_FLAG_CHROMAKEY) != 0)
		{
			grayImg = new ImageGray(img->width * 3, img->height);

			IplImage* cimg = cvCreateImageHeader(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
			cimg->imageData = cimg->imageDataOrigin = img->data;
			cimg->widthStep = img->widthstep;

			cvSmooth(cimg, cimg, CV_GAUSSIAN, 5, 5);

			cvReleaseImageHeader(&cimg);

#ifndef UMF_USE_NATIVE
			convertToYCbCr(img, grayImg);
#else
			umfnative::convertToYCbCr(img, grayImg);
#endif

		}
		else {
			grayImg = new ImageGray(img->width, img->height, true, img->width);
			convertToGrayscale(img, grayImg);
		}
        bool success = false;
        try{
		    success = umfDet->update(grayImg, timeout);
        } catch(DetectionTimeoutException &/*e*/)
        {
            success = false;
        }

		delete grayImg;
#else
        bool success = false;
        try {
		    success = umfDet->update(img, timeout);
        } catch(DetectionTimeoutException &e)
        {
            success = false;
            return -2;
        }
#endif

		if(success)
		{
			
			double position[3];
			double rotation[4];
			umfDet->model.getCameraPosRot(position, rotation);
			
			result->positionX = position[0];
			result->positionY = position[1];
			result->positionZ = position[2];

			result->quatX = rotation[1];
			result->quatY = rotation[2];
			result->quatZ = rotation[3];
			result->quatW = rotation[0];
		}

		return success ? umfDet->model.getCorrespondences().size() : 0;
	} 

	return -1;
}

UMF void __stdcall umf_get_result(DetectorProperties* detector, DetectorResult* result)
{
	DetectorResult *cached = (DetectorResult *) detector->currentResult;
	result->positionX = cached->positionX;
	result->positionY = cached->positionY;
	result->positionZ = cached->positionZ;
	
	result->quatX = cached->quatX;
	result->quatY = cached->quatY;
	result->quatZ = cached->quatZ;
	result->quatW = cached->quatW;
}



/* INIT/DELETE *********************************************************/


const char* HUGE_MARKER_STR = ""
"25\n"
"25\n"
"4;0\n"
"12\n"
"2;4;4;1;1;3;0;4;4;4;4;0;4;4;3;4;2;0;1;4;3;1;0;2;2\n"
"1;2;2;4;2;0;1;3;0;3;2;0;4;0;0;2;3;4;2;1;2;0;1;3;3\n"
"0;0;1;2;2;1;3;1;4;4;0;2;4;2;3;2;2;3;0;0;3;3;0;1;1\n"
"4;2;3;1;4;0;0;4;0;1;0;0;0;3;1;3;4;4;0;4;0;3;1;3;4\n"
"0;2;0;2;0;2;0;4;0;2;4;0;0;0;4;4;3;0;0;1;0;1;0;4;2\n"
"4;3;0;1;1;1;0;4;1;3;3;0;1;0;4;4;0;1;1;4;3;3;3;1;4\n"
"2;4;0;4;2;3;1;1;0;0;3;4;3;4;1;3;0;1;2;0;4;1;3;1;3\n"
"0;2;4;1;4;1;4;4;1;4;1;4;0;4;4;1;3;1;4;1;0;2;0;4;3\n"
"2;4;1;0;0;1;0;4;1;2;1;2;0;1;4;3;0;2;0;4;4;0;4;2;4\n"
"1;4;3;1;3;3;1;0;2;1;1;3;3;1;3;0;0;3;1;3;4;3;0;0;1\n"
"4;0;0;4;4;1;4;4;1;4;1;1;2;2;3;4;1;2;4;2;0;2;0;2;3\n"
"3;2;3;3;4;2;4;3;4;4;2;1;1;3;0;2;3;2;2;1;4;3;0;3;3\n"
"2;0;1;2;0;1;4;3;0;0;4;1;0;1;1;2;3;2;3;0;3;0;4;4;0\n"
"1;4;3;0;4;0;1;4;0;1;1;3;1;3;2;3;3;0;2;0;0;3;0;0;4\n"
"0;0;2;2;0;2;1;2;1;4;2;1;2;0;4;3;1;0;4;4;4;1;3;0;2\n"
"0;4;1;3;4;2;4;2;0;3;0;3;2;0;4;0;1;4;4;0;1;0;4;2;2\n"
"4;3;0;0;0;4;1;1;4;0;3;4;3;1;1;2;4;1;2;4;0;3;4;0;4\n"
"3;2;4;0;0;4;4;4;1;0;4;3;1;0;4;3;1;3;4;0;0;2;2;3;2\n"
"2;3;2;2;1;0;3;4;1;2;2;2;1;3;0;4;0;2;0;2;4;2;1;3;1\n"
"3;0;4;4;4;0;4;0;2;3;1;1;1;4;0;0;3;0;3;2;0;4;4;4;1\n"
"4;0;1;1;3;0;2;4;1;3;4;3;0;1;1;3;1;3;0;4;2;4;2;0;1\n"
"0;2;2;0;0;3;1;0;2;4;0;2;4;4;3;4;2;4;1;1;2;3;3;1;4\n"
"3;1;4;0;4;0;3;1;4;2;3;4;3;1;4;3;3;2;2;1;0;1;3;2;0\n"
"4;4;1;4;3;1;3;0;1;1;2;2;0;3;3;2;0;4;2;0;4;3;2;1;2\n"
"4;1;1;0;3;2;2;2;0;4;0;1;2;0;0;4;3;1;0;2;1;2;3;3;4";

const char* SMALL_MARKER_STR = ""
"14\n"
"10\n"
"4;0\n"
"12\n"
"2;4;4;1;1;3;0;4;4;4;4;0;4;4\n"
"1;2;2;4;2;0;1;3;0;3;2;0;4;0\n"
"0;0;1;2;2;1;3;1;4;4;0;2;4;2\n"
"4;2;3;1;4;0;0;4;0;1;0;0;0;3\n"
"0;2;0;2;0;2;0;4;0;2;4;0;0;0\n"
"4;3;0;1;1;1;0;4;1;3;3;0;1;0\n"
"2;4;0;4;2;3;1;1;0;0;3;4;3;4\n"
"0;2;4;1;4;1;4;4;1;4;1;4;0;4\n"
"2;4;1;0;0;1;0;4;1;2;1;2;0;1\n"
"1;4;3;1;3;3;1;0;2;1;1;3;3;1";

const char* POSTER_MARKER_STR = ""
"14\n"
"8\n"
"3;0\n"
"12\n"
"4;4;4;0;4;3;2;2;1;0;0;0;2;3\n"
"1;3;2;0;4;3;1;4;4;4;4;1;0;4\n"
"3;2;2;4;4;3;0;2;1;0;3;2;2;0\n"
"3;2;1;1;4;1;2;2;3;4;4;4;0;4\n"
"3;4;3;1;4;3;3;0;1;4;2;0;0;4\n"
"0;1;2;4;3;2;4;1;4;1;0;3;0;0\n"
"3;3;4;0;1;2;3;3;0;1;1;4;0;0\n"
"0;1;1;0;1;4;4;2;1;3;3;2;4;0";


const char* CHROMA_MARKER_STR = ""
"14\n"
"10\n"
"4;0\n"
"4\n"
"2;0;1;2;1;2;2;2;0;0;1;1;0;2\n"
"2;2;2;0;0;2;0;0;1;2;2;1;2;0\n"
"0;2;0;1;0;1;2;1;0;2;0;0;1;1\n"
"0;0;2;0;1;2;0;2;0;2;0;0;1;0\n"
"2;2;1;1;2;1;1;2;0;0;0;1;2;2\n"
"1;0;0;2;0;1;2;0;2;0;2;0;2;0\n"
"2;1;0;0;2;0;2;0;1;0;1;2;2;2\n"
"1;1;1;2;2;1;2;2;1;0;0;2;0;2\n"
"1;2;0;2;2;0;1;1;2;2;1;1;2;1\n"
"1;0;1;0;2;0;2;0;2;0;0;0;1;0\n";

const char* CHROMA_MARKER_NEW_STR = ""
	"16\n"
	"12\n"
	"3;0\n"
	"4\n"
	"2;1;0;1;0;1;2;0;1;2;1;2;1;2;0;1\n"
	"1;0;2;2;1;0;2;1;2;0;0;2;1;0;1;2\n"
	"2;1;1;2;0;1;1;0;1;0;2;1;2;2;2;1\n"
	"1;2;1;0;2;1;2;1;0;2;0;1;2;0;0;2\n"
	"2;0;0;2;1;2;0;1;2;1;0;2;1;1;2;0\n"
	"0;2;0;1;0;2;1;2;2;0;1;1;0;1;2;1\n"
	"2;1;2;0;2;1;0;0;1;2;0;2;1;2;1;0\n"
	"2;0;1;0;2;0;2;1;2;2;2;0;2;0;2;0\n"
	"1;2;0;0;1;0;2;2;0;1;2;1;2;1;1;2\n"
	"0;2;2;1;0;1;2;1;1;2;1;0;1;2;0;0\n"
	"1;2;0;2;1;0;0;2;0;1;1;0;2;1;1;2\n"
	"2;1;2;1;0;1;2;0;2;0;2;2;1;0;2;0\n";


UMF int __stdcall umf_create_detector(int width, int height, float near, float far, float fov, DetectorProperties *props)
{
	//create an RGB detector
	UMFDetector<UMF_DETECTOR_CHANNELS> *detector = new UMFDetector<UMF_DETECTOR_CHANNELS>(UMF_FLAG_ITER_REFINE|UMF_FLAG_SUBPIXEL|UMF_FLAG_SUBWINDOWS|UMF_FLAG_TRACK_POS);
	if (props->chroma > 0) {
		detector->setFlags(detector->getFlags() | UMF_FLAG_CHROMAKEY);
	}
    detector->setTrackingFlags(UMF_TRACK_SCANLINES | UMF_TRACK_MARKER | UMF_TRACK_CORNERS);
	
    
	float aspectRatio = width*1.0f/height;

	props->textureWidth = width;
	props->textureHeight = height;
    
    props->bufferSize = width*height*3; //we should get an rgb image
    props->currentImg = new ImageRGB(width, height, true, -1);

	props->currentResult = new DetectorResult;

	//load marker
	if(!detector->loadMarker(CHROMA_MARKER_STR))
	{
		return 3;
	}

	Eigen::Matrix3d cameraMatrix;
    float focal = props->textureHeight/(2*tan(fov/2));
    Eigen::Vector2f imgSize(props->textureWidth, props->textureHeight);
    cameraMatrix << focal, 0, imgSize[0]/2,
            0, focal, imgSize[1]/2,
            0, 0, 1;

    Eigen::VectorXd distCoeffs(8);
    distCoeffs << 0, 0, 0, 0, 0, 0, 0, 0;
	
	detector->model.setCameraProperties(cameraMatrix, distCoeffs);
    detector->model.setPnPFlags(PNP_FLAG_COMPUTE_CAMERA | PNP_FLAG_GL_PROJECTION_MV | PNP_FLAG_SWAP_Y | PNP_FLAG_FILTER_REPR);

	detector->tracker.setSubSampling(Tracker::SUBSAMPLE_1);

	props->detector = detector;

	return 0;
}

UMF int __stdcall umf_set_marker_str(DetectorProperties *props, char *str)
{
	UMFDetector<UMF_DETECTOR_CHANNELS> *umfDet = (UMFDetector<UMF_DETECTOR_CHANNELS> *) props->detector;

	if(!umfDet)
	{
		return 1;
	}

	if(!umfDet->loadMarker(str))
	{
		return 2;
	}

	return 0;
}

UMF void __stdcall umf_free_detector(DetectorProperties *detector)
{
#ifdef UMF_DEBUG_TIMING
	float overallTime = 100.f;
	umf::UMFDebug *dbg = umf::UMFDSingleton::Instance();

	stringstream ss;
	std::vector< std::pair<double, std::string> > timing;
	dbg->getUniqLog(timing);
	for (std::vector< std::pair<double, std::string> >::iterator it = timing.begin(); it != timing.end(); it++)
	{
		ss << it->second << ":" << it->first << " ";
		if (it->second == std::string("OVRL"))
		{
			overallTime = it->first;
		}
	}
	printf("Timing: %s ", ss.str().c_str());
#endif

	delete ((DetectorResult*) detector->currentResult);
	delete ((UMFDetector<UMF_DETECTOR_CHANNELS> *) detector->detector);
    delete ((ImageRGB*) detector->currentImg);

	detector->currentResult = NULL;
	detector->detector = NULL;
    detector->currentImg = NULL;

	UMFDSingleton::Release();
}
