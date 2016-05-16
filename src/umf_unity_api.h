#ifndef _UMF_DETECTOR_API_
#define _UMF_DETECTOR_API_

#ifndef UMF
#if defined(__WIN32__) || defined(_WIN32)
#define UMF __declspec(dllexport)
#else
#define UMF
#endif
#endif

#define UMF_DETECTOR_CHANNELS 1
#define UMF_API_DEBUG 0

#ifndef _WIN32 || defined('__CYGWIN__')
#define __stdcall
#endif

typedef struct 
{
    float positionX;
    float positionY;
    float positionZ;

	float quatX;
	float quatY;
	float quatZ;
	float quatW;
} DetectorResult;

   
typedef struct
{
    int textureWidth;
    int textureHeight;
    int bufferSize;
	int chroma;
	void *detector;
	void *currentResult;
    void *currentImg;
} DetectorProperties;

#ifdef __cplusplus
extern "C" {
#endif


/*****************Detection******************************************************************************/

//conflicts with umf_grab_frame
UMF int __stdcall umf_set_frame(DetectorProperties* detector, unsigned char* image); 

//detection
UMF int __stdcall umf_detect(DetectorProperties *detector, float timeout = 0.0);

//get detection result - should be locked to sync the camera matrix
UMF void __stdcall umf_get_result(DetectorProperties *detector, DetectorResult *result);

/*****************Detection END**************************************************************************/

/******** INIT/DELETE ****************************************************************************/

UMF int __stdcall umf_create_detector(int width, int height, float near, float far, float fov, DetectorProperties *props);

UMF int __stdcall umf_set_marker_str(DetectorProperties *props, char *str);

UMF void __stdcall umf_free_detector(DetectorProperties *detector);


#ifdef __cplusplus
}
#endif

#endif
