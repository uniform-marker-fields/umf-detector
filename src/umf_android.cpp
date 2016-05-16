#include <jni.h>
#include <android/log.h>
#include <android/bitmap.h>

#include <Eigen/Core>
#include <math.h>
#include <EGL/egl.h>

#define UMF_DETECTOR_CHANNELS 1
//DEPENDING WHETHER USING UNITY as RENDERER UNCOMMENT THIS
//it bascially cleares the scene if not using unity
#define UMF_UNITY

#include "umf.h"
#include "util/umfdebug.h"
#include "util/renderer.h"
#include "util/gl_util.h"
#include "util/chromakey.h"
#include "util/native_x.h"
#include <algorithm>
#include <glm/gtc/type_ptr.hpp>
#include <unistd.h>

#ifdef UMF_ANDROID_PROFILING
#include "prof.h"
#endif

#define  LOG_TAG    "UmfDetector"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGD(...)  __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

extern "C" {
//detector
JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_init(JNIEnv * env, jobject obj,  jint width, jint height, jfloat fovx, jfloat fovy, jboolean chroma);
JNIEXPORT jint JNICALL Java_com_quadrati_umfdetector_UMFNative_setMarkerCSV(JNIEnv * env, jobject obj, jstring markerCSV);
JNIEXPORT jint JNICALL Java_com_quadrati_umfdetector_UMFNative_setMarkerXML(JNIEnv * env, jobject obj, jstring markerXML);
JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_free(JNIEnv * env, jobject obj);
JNIEXPORT jint JNICALL Java_com_quadrati_umfdetector_UMFNative_updateYUV(JNIEnv * env, jobject thiz, jbyteArray yuvData, jint width, jint height, jobject result);

//renderer
JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_initGLSetPreviewSize(JNIEnv * env, jobject obj,  jint camWidth, jint camHeight);
JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_initGL(JNIEnv * env, jobject obj,  jint width, jint height, jboolean chroma);
JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_stepDraw(JNIEnv * env, jobject obj);

JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_initGLTextured(JNIEnv * env, jobject obj,  jint width, jint height, jint textureID, jboolean chroma);
JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_stepDrawTextured(JNIEnv * env, jobject obj);

JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_updateCameraTexture(JNIEnv * env, jobject obj, jint texID, jfloatArray yuvData);
JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_updateCameraPreview(JNIEnv * env, jobject obj, jbyteArray yuvData, jint width, jint height);
JNIEXPORT jfloat JNICALL Java_com_quadrati_umfdetector_UMFNative_getRendererFovy(JNIEnv * env, jobject obj);
};


void nf_initDetector(void **detPointer, int width, int height, float fov = M_PI_4, bool chroma = false);
void nf_freeDetector(void **detPointer);

JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_init(JNIEnv * env, jobject obj,  jint width, jint height, jfloat fovx, jfloat fovy, jboolean chroma)
{
#ifdef UMF_ANDROID_PROFILING
    setenv("CPUPROFILE_FREQUENCY", "200", 1); /* Change to 500 interrupts per second */
    monstartup("libUMFDetector.so");
#endif

    jclass cls = env->FindClass("com/quadrati/umfdetector/UMFNative");
    jfieldID fid = env->GetFieldID(cls, "detectorPointer", "J");
    //jlong detectorPointer = env->GetLongField(obj, fid);

    float fwidth = tanf(fovx * M_PI/180.0f * 0.5f);
	float fheight = tanf(fovy * M_PI/180.0f * 0.5f);

	float fAspectRatio = fwidth/fheight;
	float aspectRatio = width*1.0f/ height;

	if (fAspectRatio < 1.51f) //this should mean we are cropping vertically we have a 4:3 or 1:1 chip and if we want to get widescreen, we just crop the top and bottom
	{
		//focal length in pixels
		float flength = 1.0f / tan(fovx * M_PI/180.0 * 0.5f);
		fovy = 2.0f * 180.0f/M_PI * atan2f(1.0f/aspectRatio, flength);
		LOGI("Fovx: %.2f Fovy: %.2f ", fovx, fovy);
	}

    void *detector = NULL;
    nf_initDetector(&detector, width, height, fovy, (chroma > 0));
    LOGI("Set detector pointer: %i with fid: %i\n", (int) detector, (long)fid);

    env->SetLongField(obj, fid, (long) detector);
	env->DeleteLocalRef(cls);
}

JNIEXPORT jint JNICALL Java_com_quadrati_umfdetector_UMFNative_setMarkerCSV(JNIEnv * env, jobject obj, jstring markerCSV)
{
	jclass cls = env->FindClass("com/quadrati/umfdetector/UMFNative");
	jfieldID fid = env->GetFieldID(cls, "detectorPointer", "J");
	LOGI("Field id: %i\n", (int)fid);

	jlong detectorPointer = env->GetLongField(obj, fid);

	void *detectorP = (void *)detectorPointer;
	if (detectorP == NULL)
	{
		env->DeleteLocalRef(cls);
		return -4;
	}
	umf::UMFDetector<UMF_DETECTOR_CHANNELS> *detector = (umf::UMFDetector<UMF_DETECTOR_CHANNELS> *) detectorP;

	int retval = -1;

	if (detector != NULL) {
		const char *marker_str = env->GetStringUTFChars(markerCSV, JNI_FALSE);

		if (!detector->loadMarker(marker_str))
		{
			retval = -2;
		}

		env->ReleaseStringUTFChars(markerCSV, marker_str);
		retval = 0;
	}


	env->DeleteLocalRef(cls);
	return retval;
}

JNIEXPORT jint JNICALL Java_com_quadrati_umfdetector_UMFNative_setMarkerXML(JNIEnv * env, jobject obj, jstring markerXML)
{
	jclass cls = env->FindClass("com/quadrati/umfdetector/UMFNative");
	jfieldID fid = env->GetFieldID(cls, "detectorPointer", "J");
	jlong detectorPointer = env->GetLongField(obj, fid);

	void *detectorP = (void *)detectorPointer;
	if (detectorP == NULL)
	{
		env->DeleteLocalRef(cls);
		return -4;
	}
	umf::UMFDetector<UMF_DETECTOR_CHANNELS> *detector = (umf::UMFDetector<UMF_DETECTOR_CHANNELS> *) detectorP;
	int retval = -1;

	if (detector != NULL) {
		const char *marker_str = env->GetStringUTFChars(markerXML, JNI_FALSE);

		if (!detector->loadMarkerXML(marker_str))
		{
			retval = -2;
		}

		env->ReleaseStringUTFChars(markerXML, marker_str);
		retval = 0;
	}


	env->DeleteLocalRef(cls);
	return retval;
}

JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_free(JNIEnv * env, jobject obj)
{
    jclass cls = env->FindClass("com/quadrati/umfdetector/UMFNative");
    jfieldID fid = env->GetFieldID(cls, "detectorPointer", "J");
    jlong detectorPointer = env->GetLongField(obj, fid);

    void *detector = (void *) detectorPointer;

    LOGI("Free detector pointer: %i\n", (int) detectorPointer);

    if(detector != NULL)
    {
        nf_freeDetector(&detector);
        detector = NULL;
    }

    env->SetLongField(obj, fid, (long) detector);

#ifdef UMF_ANDROID_PROFILING
    moncleanup();
#endif
	
	env->DeleteLocalRef(cls);
}

JNIEXPORT jint JNICALL Java_com_quadrati_umfdetector_UMFNative_updateYUV(JNIEnv * env, jobject thiz, jbyteArray yuvData, jint width, jint height, jobject result)
{
    jclass cls = env->FindClass("com/quadrati/umfdetector/UMFNative");
    jfieldID fid = env->GetFieldID(cls, "detectorPointer", "J");
    jlong detectorPointer = env->GetLongField(thiz, fid);

    static int count = 0;
    const int countMax = 100;
    static double timeSum = 0;

    umf::Timer timer;
    timer.start();

    void *detectorP = (void *) detectorPointer;

    if(detectorP == NULL)
    {
        return 2;
    }
    static int calls = 0;

    bool success = false;

    umf::UMFDetector<UMF_DETECTOR_CHANNELS> *detector = (umf::UMFDetector<UMF_DETECTOR_CHANNELS> *) detectorP;

    jboolean framecopy = false;

    char * imgdata = (char*) env->GetByteArrayElements(yuvData, &framecopy);


	umf::ImageGray *img;

	if ((detector->getFlags() & umf::UMF_FLAG_CHROMAKEY) != 0) {
		img = new umf::ImageGray(width, height * 3 / 2, false);
	}
	else { // if not in chromakeying mode, just use the Y channel
		img = new umf::ImageGray(width, height, false);
	}
    img->data = imgdata;


    float timeout = -1;
    try{
		success = detector->update(img, timeout);
    } catch(umf::DetectionTimeoutException &e)
    {
        success = false;
    }

    if(success) {
        jclass clazz = env->FindClass("com/quadrati/umfdetector/UMFNative$Result");
        jfieldID positionx = env->GetFieldID(clazz, "positionX", "F");
        jfieldID positiony = env->GetFieldID(clazz, "positionY", "F");
        jfieldID positionz = env->GetFieldID(clazz, "positionZ", "F");
        jfieldID quatw = env->GetFieldID(clazz, "quatW", "F");
        jfieldID quatx = env->GetFieldID(clazz, "quatX", "F");
        jfieldID quaty = env->GetFieldID(clazz, "quatY", "F");
        jfieldID quatz = env->GetFieldID(clazz, "quatZ", "F");


        double position[3];
		double rotation[4];
		detector->model.getCameraPosRot(position, rotation);
        
        env->SetFloatField(result, positionx, (float)position[0]);
        env->SetFloatField(result, positiony, (float)position[1]);
        env->SetFloatField(result, positionz, (float)position[2]);
        
        env->SetFloatField(result, quatw, (float)rotation[0]);
        env->SetFloatField(result, quatx, (float)rotation[1]);
        env->SetFloatField(result, quaty, (float)rotation[2]);
        env->SetFloatField(result, quatz, (float)rotation[3]);

    }

//#define bitmapout
#ifdef bitmapout
    umf::ImageGray *mask = new umf::ImageGray(width, height);
    umf::ImageGray *map = new umf::ImageGray(width, height);

    umfnative::getChromeMaskNV21(img, mask, map);

    jmethodID mid = env->GetMethodID(cls, "SaveImage", "()V");

    fid = env->GetFieldID(cls, "bitmap", "Landroid/graphics/Bitmap;");
    jobject bitmap = env->GetObjectField(thiz, fid);

    AndroidBitmapInfo info;
    int ret = AndroidBitmap_getInfo(env, bitmap, &info);

    LOGD("width:%d height:%d stride:%d", info.width, info.height, info.stride);

    void *bitmapPixels;
    ret = AndroidBitmap_lockPixels(env, bitmap, &bitmapPixels);
    uint8_t * bsrc = (uint8_t*) bitmapPixels;
    for(int y = 0; y < height && y < info.height; y++)
    {
    	for(int x = 0; x < width && x < info.width; x++ )
    	{
    		bsrc[y*info.stride + x*4 + 0] =
    				bsrc[y*info.stride + x*4 + 1] = bsrc[y*info.stride + x*4 + 2] = *(mask->get2D(x, y));
    		bsrc[y*info.stride + x*4 + 3] = 255;
    	}
    }
    AndroidBitmap_unlockPixels(env, bitmap);

    env->CallVoidMethod(thiz, mid);
#endif

    delete img;

    float overallTime = 100.f;
#ifdef UMF_DEBUG_TIMING
    umf::UMFDebug *dbg = umf::UMFDSingleton::Instance();
    dbg->logEvent((double) detector->edgelDetect.getEdgels().size(), "EC");

    stringstream ss;
    std::vector< std::pair<double, std::string> > timing;
    dbg->getUniqLog(timing);
    for(std::vector< std::pair<double, std::string> >::iterator it = timing.begin(); it != timing.end(); it++)
    {
        ss << it->second << ":" << it->first << " ";
        if(it->second == std::string("OVRL"))
        {
            overallTime = it->first;
        }
    }
    LOGI("Timing: %s ", ss.str().c_str());
    LOGI("Resolution: %ix%i", width, height);
#endif
#ifdef UMF_DEBUG_COUNT_PIXELS
    calls++;
    LOGI("Footpring: %f", dbg->getPixels()*1.0/(width*height*calls));
#endif


    timeSum += timer.stop();
    count++;
    if(count == countMax)
    {
    	LOGI("Avg Time: %f", timeSum/count);
    	timeSum = 0;
    	count = 0;
    }

    env->ReleaseByteArrayElements(yuvData, (jbyte*) imgdata, JNI_ABORT);
	env->DeleteLocalRef(cls);

    return success ? overallTime : 0;
}

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


void nf_initDetector(void **detPointer, int width, int height, float fov, bool chroma)
{
    //create an RGB detector
    umf::UMFDetector<UMF_DETECTOR_CHANNELS> *detector =
    		new umf::UMFDetector<UMF_DETECTOR_CHANNELS>(umf::UMF_FLAG_ITER_REFINE | umf::UMF_FLAG_TRACK_POS|
    				umf::UMF_FLAG_SUBWINDOWS | umf::UMF_FLAG_SUBPIXEL );
    
	if (chroma) {
		detector->setFlags(detector->getFlags() | umf::UMF_FLAG_CHROMAKEY);
	}
    detector->setTrackingFlags(umf::UMF_TRACK_MARKER | /*umf::UMF_TRACK_SCANLINES |*/ umf::UMF_TRACK_CORNERS );

    //detector->model.getMarker()->setDecisionTreeMinHeight(-1);
    //load marker
    if(!detector->loadMarker(CHROMA_MARKER_STR))
    {
        return;
    }

    Eigen::Matrix3d cameraMatrix;
    float focal = height/(2*tanf(fov*M_PI/360.0f));

    Eigen::Vector2f imgSize(width, height);

	LOGI("Focal: %f", focal);

    cameraMatrix << focal, 0, imgSize[0]/2,
            0, focal, imgSize[1]/2,
            0, 0, 1;

    Eigen::VectorXd distCoeffs(8);
    distCoeffs << 0, 0, 0, 0, 0, 0, 0, 0;

    detector->model.setCameraProperties(cameraMatrix, distCoeffs);
	detector->model.setPnPFlags(umf::PNP_FLAG_COMPUTE_CAMERA | umf::PNP_FLAG_GL_PROJECTION_MV | umf::PNP_FLAG_SWAP_Y | umf::PNP_FLAG_FILTER_REPR/*| umf::PNP_FLAG_RIGHT_HANDED*/);
    //detector->model.getMarker()->setLocationMinCorrectPercentage(75);

    *detPointer = detector;
}

void nf_freeDetector(void **detPointer)
{
    delete (umf::UMFDetector<UMF_DETECTOR_CHANNELS> *) (*detPointer);
}


JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_initGLSetPreviewSize(JNIEnv * env, jobject obj,  jint camWidth, jint camHeight)
{
	umf::GLESRenderer *renderer = umf::GLESRendererSingleton::Instance();
	renderer->setPreviewSize(camWidth, camHeight);
}

JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_initGLTextured(JNIEnv * env, jobject obj, jint width, jint height, jint textureID, jboolean chroma)
{

	umf::GLESRenderer *renderer = umf::GLESRendererSingleton::Instance();
	renderer->initTextured(width, height, chroma != 0);
	renderer->setImageTexture(textureID, glm::mat4(1.0));
#ifdef UMF_DEBUG
	 umf::UMFDebug *dbg = umf::UMFDSingleton::Instance();
	 dbg->setRenderer(renderer);
#endif
}
JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_stepDrawTextured(JNIEnv * env, jobject obj)
{
	umf::GLESRenderer *renderer = umf::GLESRendererSingleton::Instance();
#ifndef UMF_UNITY
	renderer->clear();
#endif
	renderer->updateTextured();
}


JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_initGL(JNIEnv * env, jobject obj, jint width, jint height, jboolean chroma)
{
	umf::GLESRenderer *renderer = umf::GLESRendererSingleton::Instance();
	renderer->init(width, height, chroma != 0);
#ifdef UMF_DEBUG
	 umf::UMFDebug *dbg = umf::UMFDSingleton::Instance();
	 dbg->setRenderer(renderer);
#endif
}

JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_stepDraw(JNIEnv * env, jobject obj)
{
	umf::GLESRenderer *renderer = umf::GLESRendererSingleton::Instance();
#ifndef UMF_UNITY
	renderer->clear();
#endif
	renderer->update();
	//umf::checkGlError("somewhere there is something wrong");
}


JNIEXPORT jfloat JNICALL Java_com_quadrati_umfdetector_UMFNative_getRendererFovy(JNIEnv * env, jobject thiz)
{
	umf::GLESRenderer *renderer = umf::GLESRendererSingleton::Instance();
	jclass cls = env->FindClass("com/quadrati/umfdetector/UMFNative");
	jfieldID fid = env->GetFieldID(cls, "detectorPointer", "J");
	jlong detectorPointer = env->GetLongField(thiz, fid);

	void *detectorP = (void *) detectorPointer;
	if(detectorP == NULL)
	{
		return M_PI/4;
	}
	umf::UMFDetector<UMF_DETECTOR_CHANNELS> *detector = (umf::UMFDetector<UMF_DETECTOR_CHANNELS> *) detectorP;

	const Eigen::Matrix3d &cameraMatrix = detector->model.getCameraMatrix();
	float focalLength = cameraMatrix(0,0);

	float width = cameraMatrix(0, 2)*2; //should be the middle of the screen
	float height = cameraMatrix(1, 2)*2;

	float textureAspect = width / height;
	float screenAspect = (float)renderer->getWidth() / (float)renderer->getHeight();

	//scale focal length according to which side remains the same
	if (textureAspect < screenAspect)
	{
		//only if the texture is higher then we got
		//the width remains the same
		focalLength *= renderer->getWidth()/width;
	} else {
		focalLength *= renderer->getHeight()/height;
	}
	//ok now compute game fov
	float gameFovy = 2* 180.0/M_PI * atan2( renderer->getHeight() / 2, focalLength );

	env->DeleteLocalRef(cls);
	return gameFovy;
}


JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_updateCameraTexture(JNIEnv * env, jobject obj, jint texID, jfloatArray mvp)
{
	jboolean framecopy = false;
	float *texMVP = (float*) env->GetFloatArrayElements(mvp, &framecopy);
	glm::mat4 glTexMVP = glm::make_mat4(texMVP);
	umf::GLESRenderer *renderer = umf::GLESRendererSingleton::Instance();
	renderer->setImageTexture(texID, glTexMVP);

	env->ReleaseFloatArrayElements(mvp, texMVP, JNI_ABORT);
}

JNIEXPORT void JNICALL Java_com_quadrati_umfdetector_UMFNative_updateCameraPreview(JNIEnv * env, jobject obj, jbyteArray yuvData, jint width, jint height)
{
	jboolean framecopy = false;
	char * imgdata = (char*) env->GetByteArrayElements(yuvData, &framecopy);
	umf::ImageGray *img = new umf::ImageGray(width, height*3/2, false);

	img->data = imgdata;
	//umf::ImageGray *map = new umf::ImageGray(width, height*3/2);
	//std::fill_n(map->get2D(0, height), map->widthstep*height/2, 128);
	//getChromeMaskNV21(img, map, NULL);
	umf::GLESRenderer *renderer = umf::GLESRendererSingleton::Instance();
	if(renderer->isInited())
	{
		renderer->setImageNV21(img);
	}
	//delete map;
	delete img;

    env->ReleaseByteArrayElements(yuvData, (jbyte*) imgdata, JNI_ABORT);
}
