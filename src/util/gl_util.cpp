#include "gl_util.h"

#ifdef UMF_ANDROID
#include <jni.h>
#include <android/log.h>

#define  LOG_TAG    "umf_gl"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#else
#include <cstdio>
#endif


namespace umf{

#ifdef DRAW_GL

void checkGlError(const char* op) {
    for (GLint error = glGetError(); error; error
            = glGetError()) {
#ifdef UMF_ANDROID
        LOGI("after %s() glError (0x%x)\n", op, error);
#else
				fprintf(stderr, "after %s() glError (0x%x)\n", op, error);
				fflush(stderr);
#endif
    }
}

GLuint loadShader(GLenum shaderType, const char* pSource) {
    GLuint shader = glCreateShader(shaderType);
	//bool printlog = true;
    if (shader) {
        glShaderSource(shader, 1, &pSource, NULL);
        glCompileShader(shader);
        GLint compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if (compiled != GL_TRUE) {


            GLint infoLen = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
#ifdef UMF_ANDROID
            LOGE("Shader compilation failed :( infolen: %i\n", infoLen);
#endif
            if (infoLen) {
                char* buf = (char*) malloc(infoLen);
                if (buf) {
                    glGetShaderInfoLog(shader, infoLen, NULL, buf);
#ifdef UMF_ANDROID
                    LOGE("Shader compilation %d:\n%s\n",
                            shaderType, buf);
#else
					printf("Could not compile shader %d:\n%s\n",
                            shaderType, buf);
#endif
                    free(buf);
                }
            }
			if(compiled != GL_TRUE)
			{
                glDeleteShader(shader);
                shader = 0;
			}
        }
    } else {
#ifdef UMF_ANDROID
		LOGE("Unable to create shader.");
#endif
	}
    return shader;
}

GLuint createProgram(const char* pVertexSource, const char* pFragmentSource) {
    GLuint vertexShader = loadShader(GL_VERTEX_SHADER, pVertexSource);
    if (!vertexShader) {
#ifdef UMF_ANDROID
			LOGE("Something wrong with vertex shader\n");
#endif
        return 0;
    }

    GLuint pixelShader = loadShader(GL_FRAGMENT_SHADER, pFragmentSource);
    if (!pixelShader) {
#ifdef UMF_ANDROID
		LOGE("Something wrong with fragment shader\n");
#endif
        return 0;
    }
#ifdef UMF_ANDROID
	LOGE("Everything fine with shaders.\n");
#endif

    GLuint program = glCreateProgram();
    if (program) {
        glAttachShader(program, vertexShader);
        checkGlError("glAttachShader");
        glAttachShader(program, pixelShader);
        checkGlError("glAttachShader");
        glLinkProgram(program);
        GLint linkStatus = GL_FALSE;
        glGetProgramiv(program, GL_LINK_STATUS, &linkStatus);
        if (linkStatus != GL_TRUE) {
            GLint bufLength = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &bufLength);
            if (bufLength) {
                char* buf = (char*) malloc(bufLength);
                if (buf) {
                    glGetProgramInfoLog(program, bufLength, NULL, buf);
#ifdef UMF_ANDROID
                    LOGE("Could not link program:\n%s\n", buf);
#else
					printf("Could not link program:\n%s\n", buf);
#endif
                    free(buf);
                }
            }
            glDeleteProgram(program);
            program = 0;
        }
    }
	//either way we can delete shaders, since until it is attached to the shader program, nothing can happen to it
	glDeleteShader(vertexShader);
	glDeleteShader(pixelShader);
#ifdef UMF_ANDROID
	LOGE("Linked program program:\n");
#endif

    return program;
}

#endif

}