package com.quadrati.umfdetector;

import java.nio.ByteBuffer;

import android.annotation.TargetApi;
import android.graphics.*;
import android.opengl.GLES20;
import android.os.Build;
import android.util.Log;

@TargetApi(Build.VERSION_CODES.HONEYCOMB)
class UMFSurface extends SurfaceTexture {

	static String TAG = "UMFSurface";
	private int width, height;
	byte[] data;
	private int texID;
	private int fbID;
	private int copyProgram;
			
	
	public UMFSurface(int texName, int width, int height) {
		super(texName);
		this.setWidth(width);
		this.setHeight(height);
		data = new byte[width*height*4];
		this.setTexID(texName);
	}
	
	public int createProgram(String vertexSource, String fragmentSource) {
	    int vertexShader = loadShader(GLES20.GL_VERTEX_SHADER, vertexSource);
	    int pixelShader = loadShader(GLES20.GL_FRAGMENT_SHADER, fragmentSource);

	    int program = GLES20.glCreateProgram();
	    if (program != 0) {
	        GLES20.glAttachShader(program, vertexShader);
	        GLES20.glAttachShader(program, pixelShader);
	        GLES20.glLinkProgram(program);
	        int[] linkStatus = new int[1];
	        GLES20.glGetProgramiv(program, GLES20.GL_LINK_STATUS, linkStatus, 0);
	        if (linkStatus[0] != GLES20.GL_TRUE) {
	            Log.e(TAG, "Could not link program: ");
	            Log.e(TAG, GLES20.glGetProgramInfoLog(program));
	            GLES20.glDeleteProgram(program);
	            program = 0;
	        }
	    }
	    return program;
	}

	private int loadShader(int shaderType, String source) {
	    int shader = GLES20.glCreateShader(shaderType);
	        if (shader != 0) {
	            GLES20.glShaderSource(shader, source);
	            GLES20.glCompileShader(shader);
	            int[] compiled = new int[1];
	            GLES20.glGetShaderiv(shader, GLES20.GL_COMPILE_STATUS, compiled, 0);
	            if (compiled[0] == 0) {
	                Log.e(TAG, "Could not compile shader " + shaderType + ":");
	                Log.e(TAG, GLES20.glGetShaderInfoLog(shader));
	                GLES20.glDeleteShader(shader);
	                shader = 0;
	            }
	        }
	        return shader;
	}
	
	@TargetApi(Build.VERSION_CODES.ICE_CREAM_SANDWICH_MR1)
	public void setupFB()
	{
		int[] framebuffers = new int[1];
		GLES20.glGenFramebuffers(1, framebuffers, 0);
		this.fbID = framebuffers[0];
		GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, this.fbID);
		int[] renderbuffers = new int[2];
		GLES20.glGenRenderbuffers(2, renderbuffers, 0);
		//int[] textures = new int[1];
		//GLES20.glGenTextures(1, textures, 0);
		//GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, textures[0]);
		//GLES20.glTexImage2D(GLES20.GL_TEXTURE_2D, 0, GLES20.GL_RGBA, width, height, 0, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, ByteBuffer.allocate(0));
		//GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0);
		//GLES20.glFramebufferTexture2D(GLES20.GL_FRAMEBUFFER, GLES20.GL_COLOR_ATTACHMENT0, GLES20.GL_TEXTURE_2D, textures[0], 0);
		GLES20.glBindRenderbuffer(GLES20.GL_RENDERBUFFER, renderbuffers[0]);
		GLES20.glRenderbufferStorage(GLES20.GL_RENDERBUFFER, GLES20.GL_RGB565, width, height);
		GLES20.glFramebufferRenderbuffer(GLES20.GL_FRAMEBUFFER, GLES20.GL_COLOR_ATTACHMENT0, GLES20.GL_RENDERBUFFER, renderbuffers[0]);
		
		GLES20.glBindRenderbuffer(GLES20.GL_RENDERBUFFER, renderbuffers[1]);
		GLES20.glRenderbufferStorage(GLES20.GL_RENDERBUFFER, GLES20.GL_DEPTH_COMPONENT16, width, height);
		GLES20.glFramebufferRenderbuffer(GLES20.GL_FRAMEBUFFER, GLES20.GL_DEPTH_ATTACHMENT, GLES20.GL_RENDERBUFFER, renderbuffers[1]);
		
		int status = GLES20.glCheckFramebufferStatus(GLES20.GL_FRAMEBUFFER);
		if(status != GLES20.GL_FRAMEBUFFER_COMPLETE)
		{
			Log.e("UMF", "Unable to complete framebuffer");
		}
		GLES20.glViewport(0, 0, width, height);
		
		GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0);
	}
	
	@TargetApi(Build.VERSION_CODES.ICE_CREAM_SANDWICH_MR1)
	public void updateData()
	{
		//#TODO try with gles transparent flag set - otherwise we are limited to 565 textures
		GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, this.fbID);
		GLES20.glReadPixels(0, 0, width, height, GLES20.GL_RGB, GLES20.GL_UNSIGNED_SHORT_5_6_5, ByteBuffer.wrap(data));
		GLES20.glBindFramebuffer(GLES20.GL_FRAMEBUFFER, 0);
	}

	public int getWidth() {
		return width;
	}

	private void setWidth(int width) {
		this.width = width;
	}

	public int getHeight() {
		return height;
	}

	private void setHeight(int height) {
		this.height = height;
	}

	public int getTexID() {
		return texID;
	}

	public void setTexID(int texID) {
		this.texID = texID;
	}

	public int getCopyProgram() {
		return copyProgram;
	}

	public void setCopyProgram(int copyProgram) {
		this.copyProgram = copyProgram;
	}

}
