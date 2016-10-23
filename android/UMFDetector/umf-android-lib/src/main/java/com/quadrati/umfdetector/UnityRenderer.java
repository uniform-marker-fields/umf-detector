

package com.quadrati.umfdetector;

import java.io.IOException;

import com.quadrati.umfdetector.Detector.RendererCallback;
import com.quadrati.umfdetector.GL2View.Semaphore;

import android.annotation.TargetApi;
import android.graphics.SurfaceTexture;
import android.graphics.SurfaceTexture.OnFrameAvailableListener;
import android.hardware.Camera;
import android.opengl.GLES20;
import android.os.Build;
import android.util.Log;

public class UnityRenderer implements RendererCallback {


    private static String TAG = "UnityRenderer";
    public UMFSurface sTexture;
    public int mTexture;
    
    public class Semaphore {
  	  private int available = 0;

  	  public synchronized void take() {
  	    this.available++;
  	    this.notify();
  	  }

  	  public synchronized void release() throws InterruptedException{
  	    while(this.available <= 0) wait();
  	    this.available--;
  	  }
  	  
  	  public synchronized boolean test() {
  		  return this.available > 0;
  	  }
  	}

    protected volatile Semaphore shouldUpdate = new Semaphore();
    protected volatile Semaphore shouldUpdateTexture = new Semaphore();
    private boolean videoSynchronized = true; 
	
	@Override
	public void init(Camera cam) {

		Camera.Size s = cam.getParameters().getPreviewSize();
		this.textureUpdater = null;
		this.setCameraSize(s.width, s.height);
		if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
			this.initTexture(cam);
		}
		Log.i(TAG, "Initializing renderer stuff for UMF");
	}
	
	@TargetApi(Build.VERSION_CODES.HONEYCOMB)
	private class TexturedUpdater implements OnFrameAvailableListener
	{
		UnityRenderer rend;
		public TexturedUpdater(UnityRenderer r)
		{
			rend = r;
			Log.i(TAG, "Set update renderer");
		}

		@Override
		public void onFrameAvailable(SurfaceTexture surfaceTexture) {
			rend.shouldUpdate.take();
		}
	}
	
	private TexturedUpdater textureUpdater;
	
	@TargetApi(Build.VERSION_CODES.HONEYCOMB)
	private void initTexture(Camera cam)
	{
		int[] textures = new int[1];
		GLES20.glGenTextures(1, textures, 0);
		this.mTexture = textures[0];
		Camera.Size s = cam.getParameters().getPreviewSize();
		this.sTexture = new UMFSurface(this.mTexture, s.width, s.height);
		try {
			cam.setPreviewTexture(this.sTexture);
		} catch (IOException e) {
			e.printStackTrace();
		}
		this.textureUpdater = new TexturedUpdater(this);
		this.sTexture.setOnFrameAvailableListener(this.textureUpdater);
	}

	@Override
	public void onUpdateRenderer(byte[] data, int width, int height, Camera cam) {
		//since it runs in other thread, this is safe
		UMFNative.updateCameraPreview(data, width, height);
	}
	
	private void setCameraSize(int width, int height)
	{
		UMFNative.initGLSetPreviewSize(width, height);
	}

	@TargetApi(Build.VERSION_CODES.HONEYCOMB)
	private void onDrawFrameTextured()
	{
		if(this.sTexture != null && this.shouldUpdate.test()){
			this.shouldUpdate.take();
			this.sTexture.updateTexImage();
			if(this.videoSynchronized && this.shouldUpdateTexture.test())
			{
				try {
					this.shouldUpdateTexture.release();
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				float[] arr = new float[16];
				UMFNative.updateCameraTexture(this.mTexture, arr);
			}
		}
		
		UMFNative.stepDrawTextured();
	}
	
    public void onDrawFrame() {
    	if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
    		this.onDrawFrameTextured();
    	} else {
    		UMFNative.stepDraw();
    	}
    }

    //should be called after the detection is started and GL is already initialized
    public void onRendererChanged(int width, int height, boolean chroma) {
    	if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
    		UMFNative.initGLTextured(width, height, this.mTexture, chroma);
    	} else {
    		UMFNative.initGL(width, height, chroma);
    	}

    	Log.i(TAG, "Initialized GL renderer.");
    }

	@Override
	public void synchronizeFrames() {
		if(this.videoSynchronized)
		{
			this.shouldUpdateTexture.take();
		}
		
	}
	
}
