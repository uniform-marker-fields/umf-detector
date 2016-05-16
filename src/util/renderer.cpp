
#include "renderer.h"
#include "draw.h"

#ifdef UMF_USE_GLFW
#include "gl_util.h"
#include "shaders.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#endif

#ifdef UMF_USE_GLES
#include "gl_util.h"
#include "shaders.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#endif

#ifdef UMF_ANDROID
#include <jni.h>
#include <android/log.h>

#define  LOG_TAG    "umf_gl"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#endif

#ifdef UMF_USE_OPENCV_MINIMUM

#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

namespace umf {

#ifdef UMF_USE_GLFW

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
	fflush(stderr);
}

//singleton definition
GLRenderer* Singleton<GLRenderer>::m_pInstance = nullptr;

int GLRenderer::init(int width, int height, bool chromakey)
{
	this->width = width;
	this->height = height;
	this->chromakey = chromakey;
    glfwSetErrorCallback(error_callback);

	// Initialize the library 
    if (!glfwInit())
		return EXIT_FAILURE;

    // Create a windowed mode window and its OpenGL context 
	window = glfwCreateWindow(width, height, "UMF debug GL", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
		return EXIT_FAILURE;
    }

    // Make the window's context current 
    glfwMakeContextCurrent(window);

	checkGlError("GLFW inited");

	glewInit();

	checkGlError("GLEW inited");

	this->cameraTexID = -1;
	glGenTextures(1, &this->cameraTexID);
	glBindTexture(GL_TEXTURE_2D, this->cameraTexID);
	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0); //allocate data

	checkGlError("Texture created");
	//todo use chromakey if needed
	this->cameraShader = createProgram(shQuadVertex, shQuadFragment);

	checkGlError("ShaderProgram created");

	return EXIT_SUCCESS;
}


int GLRenderer::update()
{
	
	//draw the cmera texture in the background
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->cameraTexID);
	
	glUseProgram(this->cameraShader);
	GLuint posA = glGetAttribLocation(this->cameraShader, "pos");
	GLuint texA = glGetAttribLocation(this->cameraShader, "texCoord");
	GLuint texU = glGetUniformLocation(this->cameraShader, "texImg");
	GLuint mvpU = glGetUniformLocation(this->cameraShader, "mvp");

	
	//enable vertex pos
	glEnableVertexAttribArray(posA);
	glVertexAttribPointer(posA, 3, GL_FLOAT, GL_FALSE, sizeof(QuadVertex), &QuadReverseFullScreen[0].Position);

	
	//enable texcoord
	glEnableVertexAttribArray(texA);
	glVertexAttribPointer(texA, 2, GL_FLOAT, GL_FALSE, sizeof(QuadVertex), &QuadReverseFullScreen[0].TexCoord);

	
	//set texture uniform
	glUniform1i(texU, 0);
	glm::mat4 mvp(1.0);
	glUniformMatrix4fv(mvpU, 1, GL_FALSE, glm::value_ptr(mvp));

	
	glDrawArrays(GL_QUADS, 0, 4);
	

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisableVertexAttribArray(texA);
	glDisableVertexAttribArray(posA);
	glUseProgram(0);

	//SDL_GL_SwapWindow(this->window);
	//SDL_Delay(30);
	
	glfwSwapBuffers(this->window);

	glfwPollEvents();
	
	return EXIT_SUCCESS;
}

void GLRenderer::destroy()
{
	glfwTerminate();
}

void GLRenderer::clear()
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
}

int GLRenderer::setImageRGB(ImageRGB *image)
{
	glBindTexture(GL_TEXTURE_2D, this->cameraTexID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image->width, image->height, 0, GL_RGB, GL_UNSIGNED_BYTE, image->data);
	return EXIT_SUCCESS;
}

#endif

#ifdef UMF_USE_OPENCV_MINIMUM

template<>
OpenCVRenderer* Singleton<OpenCVRenderer>::m_pInstance = NULL;

int OpenCVRenderer::init(int width, int height, bool chromakey)
{
	cvNamedWindow("UMF opencv output");
	this->width = width;
	this->height = height;
    this->cvImg = NULL;
	this->chromakey = chromakey;
	return EXIT_SUCCESS;
}

void OpenCVRenderer::clear()
{
    if(this->cvImg != NULL)
	{
		cvSet(this->cvImg, CV_RGB(0, 0, 0));
	}
}
	
int OpenCVRenderer::setImageRGB(ImageRGB *bgImg)
{
    if(this->cvImg != NULL &&
		((this->cvImg->width != bgImg->width) || (this->cvImg->height != bgImg->height) || (this->cvImg->widthStep != bgImg->widthstep))
		)
	{
		cvReleaseImageData(this->cvImg->imageData);
		cvReleaseImageHeader(&(this->cvImg));
        this->cvImg = NULL;
	}

    if(this->cvImg == NULL)
	{
		this->cvImg = cvCreateImageHeader(cvSize(bgImg->width, bgImg->height), IPL_DEPTH_8U, 3);
		this->cvImg->widthStep = bgImg->widthstep;

		cvCreateImageData(this->cvImg);
	}

	memcpy(this->cvImg->imageData, bgImg->data, bgImg->widthstep*bgImg->height*sizeof(char));
	
	cvCvtColor(this->cvImg, this->cvImg, CV_RGB2BGR);
	return EXIT_SUCCESS;
}

int OpenCVRenderer::update()
{
	cvShowImage("UMF opencv output", this->cvImg);
	return EXIT_SUCCESS;
}

void OpenCVRenderer::destroy()
{
	cvDestroyAllWindows();
    if (this->cvImg != NULL) {
		cvReleaseImageData(this->cvImg);
		cvReleaseImageHeader(&(this->cvImg));
	}
    this->cvImg = NULL;
}

#endif

#ifdef UMF_USE_GLES

//singleton instant
template<>
GLESRenderer* Singleton<GLESRenderer>::m_pInstance = nullptr;

int GLESRenderer::init(int width, int height, bool chromakey)
{
	this->width = width;
	this->height = height;
	this->chromakey = chromakey;
    //the gles context should be already inited

	this->cameraTexIDy = -1;
	glGenTextures(1, &this->cameraTexIDy);
	glBindTexture(GL_TEXTURE_2D, this->cameraTexIDy);
	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, 0); //allocate data

	glGenTextures(1, &this->cameraTexIDuv);
	glBindTexture(GL_TEXTURE_2D, this->cameraTexIDuv);
	// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	// when texture area is large, bilinear filter the original
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

	// the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, width/2, height/2, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, 0); //allocate data

	checkGlError("Texture created");
	if (chromakey) {
		this->cameraShader = createProgram(shQuadVertex, shQuadNV21FragmentGreen);
	}
	else {
		this->cameraShader = createProgram(shQuadVertex, shQuadNV21Fragment);
	}

	checkGlError("ShaderProgram created");

	for(int i = 0; i < 4; i++)
	{
		this->vertices[i] = QuadReverseFullScreen[i];
	}
	this->updateVertices();
	
    glViewport(0, 0, width, height);

	this->inited = true;

	return EXIT_SUCCESS;
}

int GLESRenderer::initTextured(int width, int height, bool chromakey)
{
	this->width = width;
	this->height = height;
	this->chromakey = chromakey;
    //the gles context should be already inited
	this->textureMVP = glm::mat4(1.0);

	if (chromakey) {
		this->cameraShader = createProgram(shQuadVertex, shQuadESExtTexFragmentGreen2);
	}
	else {
		this->cameraShader = createProgram(shQuadVertex, shQuadESExtTexFragment);
	}

	checkGlError("ShaderProgram created");

	for(int i = 0; i < 4; i++)
	{
		this->vertices[i] = QuadReverseFullScreen[i];
	}
	
	this->updateVertices();

	
	checkGlError("init");
	glGenBuffers(1, &elementBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBuffer);
	
	checkGlError("binding element");
	GLubyte indices[] = {0, 1, 2, 3};
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	
	checkGlError("element buffer");

	glGenBuffers(1, &vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(QuadVertex)*4, this->vertices, GL_STATIC_DRAW);
	
	checkGlError("vertex buffer");

	
    glViewport(0, 0, width, height);

	this->inited = true;

	return EXIT_SUCCESS;
}


void GLESRenderer::setPreviewSize(int width, int height)
{
	this->camWidth = width;
	this->camHeight = height;
	this->updateVertices();
}


int GLESRenderer::update()
{
	if(this->shouldUpdate)
	{
		this->updateTexture(this->yuv);
		this->shouldUpdate = false;
	}

	//draw the camera texture in the background
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, this->cameraTexIDy);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, this->cameraTexIDuv);
	//checkGlError("Binding textures");
	
	glUseProgram(this->cameraShader);
	//checkGlError("use program");
	GLuint posA = glGetAttribLocation(this->cameraShader, "pos");
	//checkGlError("pos");
	GLuint texA = glGetAttribLocation(this->cameraShader, "texCoord");
	//checkGlError("texcoord");
	GLuint texUY = glGetUniformLocation(this->cameraShader, "texImgY");
	//checkGlError("teximgy");
	GLuint texUUV = glGetUniformLocation(this->cameraShader, "texImgUV");
	//checkGlError("teximguv");
	GLuint mvpU = glGetUniformLocation(this->cameraShader, "mvp");
	//checkGlError("mvp");

	
	//enable vertex pos
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glVertexAttribPointer(posA, 3, GL_FLOAT, GL_FALSE, sizeof(QuadVertex), (void *) offsetof(QuadVertex, Position));
	glEnableVertexAttribArray(posA);
	
	//enable texcoord
	glVertexAttribPointer(texA, 2, GL_FLOAT, GL_FALSE, sizeof(QuadVertex), (void *) offsetof(QuadVertex, TexCoord));
	glEnableVertexAttribArray(texA);

	//checkGlError("setting attrib pointers");
	
	//set texture uniform
	glUniform1i(texUY, 0);
	glUniform1i(texUUV, 1);
	glm::mat4 mvp(1.0);
	glUniformMatrix4fv(mvpU, 1, GL_FALSE, glm::value_ptr(mvp));

	//checkGlError("setting uniforms");
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	
	//glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBuffer);
	glDrawElements(GL_TRIANGLE_FAN, 1, GL_UNSIGNED_BYTE, 0);
	
	
	glDisable(GL_BLEND);
	//checkGlError("drawing");
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisableVertexAttribArray(texA);
	glDisableVertexAttribArray(posA);
	glUseProgram(0);

	//checkGlError("deactivating");
	
	return EXIT_SUCCESS;
}

int GLESRenderer::updateTextured()
{
	
	checkGlError("before buffers");
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	checkGlError("unbind buffers1");
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	checkGlError("unbind buffers2");
	glDisable(GL_CULL_FACE);
	//glDisable(GL_DEPTH_TEST);

	//draw the camera texture in the background
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_EXTERNAL_OES, this->cameraTexRGB);
	checkGlError("Binding textures");
	
	glUseProgram(this->cameraShader);
	checkGlError("use program");
	GLuint posA = glGetAttribLocation(this->cameraShader, "pos");
	checkGlError("pos");
	GLuint texA = glGetAttribLocation(this->cameraShader, "texCoord");
	checkGlError("texcoord");
	GLuint texImg = glGetUniformLocation(this->cameraShader, "texImg");
	checkGlError("teximguv");
	GLuint mvpU = glGetUniformLocation(this->cameraShader, "mvp");
	checkGlError("mvp");

	GLuint mvpT = glGetUniformLocation(this->cameraShader, "texMVP");

	//enable vertex pos
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glEnableVertexAttribArray(posA);
	glVertexAttribPointer(posA, 3, GL_FLOAT, GL_FALSE, sizeof(QuadVertex), (void *) offsetof(QuadVertex, Position));
	
	//enable texcoord
	glEnableVertexAttribArray(texA);
	glVertexAttribPointer(texA, 2, GL_FLOAT, GL_FALSE, sizeof(QuadVertex), (void *) offsetof(QuadVertex, TexCoord));

	checkGlError("setting attrib pointers");
	
	//set texture uniform
	glUniform1i(texImg, 0);
	glm::mat4 mvp(1.0);
	glUniformMatrix4fv(mvpU, 1, GL_FALSE, glm::value_ptr(mvp));
	glUniformMatrix4fv(mvpT, 1, GL_FALSE, glm::value_ptr(this->textureMVP));

	checkGlError("setting uniforms");
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	
	//glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementBuffer);
	glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_BYTE, 0);
	
	glDisable(GL_BLEND);
	checkGlError("drawing");
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisableVertexAttribArray(texA);
	glDisableVertexAttribArray(posA);
	glUseProgram(0);

	checkGlError("deactivating");
	
	return EXIT_SUCCESS;
}

void GLESRenderer::destroy()
{
	if(this->yuv != NULL)
	{
		//do nothing or remove textures
		ImageGray *pp = this->yuv;
		this->yuv = NULL;
		delete pp;
	}
}

void GLESRenderer::clear()
{
	glClearColor(0.8f, 0.0f, 0.9f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
}

void GLESRenderer::updateVertices()
{
	float cameraRatio = this->camWidth*1.0f/this->camHeight;
	float glRatio = this->width*1.0f/this->height;
	
	if(cameraRatio > glRatio) //camera is wider than gl window
	{
		float widthOffset = (1.0f - glRatio/cameraRatio)*0.5f;
		this->vertices[0].TexCoord[0] = this->vertices[1].TexCoord[0] = 1.0f - widthOffset;
		this->vertices[2].TexCoord[0] = this->vertices[3].TexCoord[0] = widthOffset;
		
		this->vertices[0].TexCoord[1] = this->vertices[3].TexCoord[1] = 1.0f;
		this->vertices[1].TexCoord[1] = this->vertices[2].TexCoord[1] = 0.0f;
	} else if(cameraRatio < glRatio) {
		float heightOffset = (1.0f - cameraRatio/glRatio)*0.5f;
		this->vertices[0].TexCoord[1] = this->vertices[3].TexCoord[1] = 1.0f - heightOffset;
		this->vertices[1].TexCoord[1] = this->vertices[2].TexCoord[1] = heightOffset;
		
		this->vertices[0].TexCoord[0] = this->vertices[1].TexCoord[0] = 1.0f;
		this->vertices[2].TexCoord[0] = this->vertices[3].TexCoord[0] = 0.0f;
	}

	if (this->chromakey == false) {
		this->vertices[0].Position[2] = 0.99f;
		this->vertices[1].Position[2] = 0.99f;
		this->vertices[2].Position[2] = 0.99f;
		this->vertices[3].Position[2] = 0.99f;
	}

	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(QuadVertex)*4, this->vertices, GL_STATIC_DRAW);
}

int GLESRenderer::setImageRGB(ImageRGB *img)
{
	//TODO do something useful here
	assert(false);
	return false;
}


void GLESRenderer::updateTexture(ImageGray *image)
{
	int width = image->width;
	int height = image->height*2/3;
	glBindTexture(GL_TEXTURE_2D, this->cameraTexIDy);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0,
		GL_LUMINANCE, GL_UNSIGNED_BYTE, image->get2D(0, 0));
	
	glBindTexture(GL_TEXTURE_2D, this->cameraTexIDuv);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE_ALPHA, width/2, height/2, 0,
		GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, image->get2D(0, height));

	//this->updateVertices(width, height);
}

int GLESRenderer::setImageNV21(ImageGray *image)
{
	if(this->yuv == NULL)
	{
		this->yuv = new ImageGray(image->width, image->height, true, image->widthstep);
	} else if(this->yuv->width != image->width || this->yuv->height != image->height)
	{
		delete this->yuv;
		this->yuv = new ImageGray(image->width, image->height, true, image->widthstep);
	}
	memcpy(this->yuv->data, image->data, image->widthstep*image->height);
	this->shouldUpdate = true;
	//this->updateTexture(this->yuv);
	
	return EXIT_SUCCESS;
}


int GLESRenderer::setImageTexture(GLuint texture, glm::mat4 mvp)
{
	this->cameraTexRGB = texture;
	//this->textureMVP = mvp;
	return EXIT_SUCCESS;
}

#endif


}
