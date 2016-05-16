#ifndef _UMF_RENDERER_H_
#define _UMF_RENDERER_H_

#include "image.h"
#include "../defines.h"
#include "singleton.h"



#ifdef UMF_USE_OPENCV_MINIMUM

#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#ifdef UMF_USE_GLFW

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#endif

#ifdef UMF_USE_GLES
#include "shaders.h"
#include <glm/glm.hpp>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <EGL/egl.h>
#endif

//Renderers for debug and rendering as background texture for android and Unity
//When debug drawing is necessary, please use the OpenCV renderer


namespace umf
{

//abstract class - interface
class UMF Renderer
{
public:
	virtual int init(int width, int height, bool chromakey) = 0;
	virtual void clear() = 0;

	virtual int setImageRGB(ImageRGB *bgImg) = 0;
	virtual int update() = 0;
	virtual void destroy() = 0;

	virtual int getWidth() = 0;
	virtual int getHeight() = 0;
};


#ifdef UMF_USE_OPENCV_MINIMUM

class UMF OpenCVRenderer: public Renderer
{
public:
	virtual int init(int width, int height, bool chromakey);
	virtual void clear();
	
	virtual int setImageRGB(ImageRGB *bgImg);
	virtual int update();
	virtual void destroy();
	
	virtual int getWidth() { return this->width; };
	virtual int getHeight() { return this->height; };
	
	IplImage *getCVImg() { return this->cvImg; }
private:
	IplImage *cvImg;
	int width, height;
	bool chromakey;
};


typedef Singleton<OpenCVRenderer> OpenCVRendererSingleton;

#endif

#ifdef UMF_USE_GLFW

class UMF GLRenderer: public Renderer
{
public:
	virtual int init(int width, int height, bool chromakey);
	virtual void clear();
	
	virtual int setImageRGB(ImageRGB *bgImg);

	virtual int update();
	virtual void destroy();

	virtual int getWidth() { return this->width; };
	virtual int getHeight() { return this->height; };
private:
	GLFWwindow* window;
	GLuint cameraTexID;
	GLuint cameraShader;

	int width, height;
	bool chromakey;

};

typedef Singleton<GLRenderer> GLRendererSingleton;

#endif

#ifdef UMF_USE_GLES

/**
 * Renderer for Android-like devices (which use OpenGL ES 2.0)
 */
class UMF GLESRenderer: public Renderer
{
public:
	GLESRenderer(): Renderer(),
		width(1), height(1),
		camWidth(1), camHeight(1),
		inited(false),shouldUpdate(false),
		yuv(NULL)
		 { }
	virtual int init(int width, int height, bool chromakey); //initialize, when setImagRGB or setImageNV21 is used to updae
	virtual int initTextured(int width, int height, bool chromakey); //initialize when external texture will be provided (setImageTexture)
	virtual void setPreviewSize(int camWidth, int camHeight);
	
	virtual void clear();
	
	virtual int setImageRGB(ImageRGB *bgImg);
	virtual int setImageNV21(ImageGray *yuvImg); //Assign image in android native NV21 format
	virtual int setImageTexture(GLuint texture, glm::mat4 mvp); //set the external texture ID and modelview matrix

	virtual int update(); //update the internal texture by uploading to GPU
	virtual int updateTextured(); //update when an external texture was used (Android)
	virtual void destroy(); //free GL ES resources

	virtual int getWidth() { return this->width; };
	virtual int getHeight() { return this->height; };

	bool isInited() { return this->inited;}
private:
	GLuint cameraTexRGB;
	glm::mat4 textureMVP;
	GLuint cameraTexIDy;
	GLuint cameraTexIDuv;
	GLuint cameraShader;
	QuadVertex vertices[4];

	void updateTexture(ImageGray* img);
	void updateVertices();

	int width, height;
	bool chromakey;
	int camWidth, camHeight;
	bool volatile inited, shouldUpdate;
	ImageGray * volatile yuv;
	GLuint elementBuffer, vertexBuffer, texCoordBuffer;
};

typedef Singleton<GLESRenderer> GLESRendererSingleton;

#endif

}
#endif
