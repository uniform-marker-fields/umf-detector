#ifndef _UMF_GL_UTIL_H_
#define _UMF_GL_UTIL_H_

#include "../defines.h"

#ifdef UMF_USE_GLFW

#include <GL/glew.h>
#endif

#ifdef UMF_ANDROID
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <GLES/gl.h>
#include <GLES/glext.h>
#endif

namespace umf
{
#ifdef DRAW_GL
    /**
     * Check OpenGL error and store the corresponding text in op
     */
	void checkGlError(const char* op);
    
    /**
     * load and compile an OpenGL shader
     * @param shaderType fragment/vertex shader
     * @param pSource The shader source code
     * @return the shader code, that can be used for linking
     */
	GLuint loadShader(GLenum shaderType, const char* pSource);

    /**
     * Create a program from vertex and fragment shader sources. Internally uses loadShader
     *
     * @param pVertexSource The source code for the vertex shader
     * @param pFragmentSource The source code for the fragment shader
     * @return The program ID, that can be activitated for OpenGL rendering
     */
	GLuint createProgram(const char* pVertexSource, const char* pFragmentSource);
#endif
}

#endif