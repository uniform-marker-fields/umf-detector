package com.quadrati.umfdetector;

import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.os.Build;
import android.util.Log;

public class Detector implements PreviewCallback {
    private static String TAG = "UMFDetector";
	
	public static interface DetectionCallback {
		
		public void onUMFDetected(int success);
	}
	
	public static interface RendererCallback {
		//should return back the data to the camera as buffer
		public void init(Camera cam);
		public void onUpdateRenderer(byte[] data, int width, int height, Camera cam);
		public void synchronizeFrames();
	}
	
	public UMFNative ndetector;
	private UMFNative.Result resultCache;
	private DetectionCallback detcb;
	private RendererCallback rendercb;
	private Camera camera;
	private boolean updateRender;
	private Camera.Size cameraSize;
	private boolean textured;

	public void setChroma(boolean chroma) {
		this.chroma = chroma;
	}
	public int setMarkerCSV(String csvContents) { return this.ndetector.setMarkerCSV(csvContents);}
	public int setMarkerXML(String xmlContents) { return this.ndetector.setMarkerXML(xmlContents);}

	private boolean chroma;
	
	
	public Detector()
	{
		ndetector = new UMFNative();
		resultCache = null;
		detcb = null;
		updateRender = true;
		setRenderCallback(null);
		textured = false;
		chroma = false;
	}

	
	public void setCallback(DetectionCallback cb)
	{
		this.detcb = cb;
	}
	
	/**
	 * Some openGL renderer should be already initialized, otherwise might return false values
	 * @return fovy to use for OpenGL rendering
	 */
	public float getRendererFovy()
	{
		return this.ndetector.getRendererFovy();
	}

	public void startDetection(Camera camera) {
		Camera.Parameters p = camera.getParameters();
        Camera.Size mcamsize = p.getPreviewSize();

		this.camera = camera;
		
		this.cameraSize = mcamsize;

		Log.v("UMFD", "Fovx: " + p.getHorizontalViewAngle() + " Fovy: " + p.getVerticalViewAngle());
		ndetector.init(mcamsize.width, mcamsize.height, p.getHorizontalViewAngle(), p.getVerticalViewAngle(), chroma);
		
		int format = p.getPreviewFormat();
		int bitsp = ImageFormat.getBitsPerPixel(format);
		int bufSize = bitsp*mcamsize.width*mcamsize.height/8;
		
		if(this.rendercb != null)
		{
			this.rendercb.init(camera);
		}
		
		if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
	        textured = true;
	        //byte[] buffer1 = new byte[bufSize];
			//byte[] buffer2 = new byte[bufSize];

			//camera.addCallbackBuffer(buffer1);
			//camera.addCallbackBuffer(buffer2);
	        //camera.setPreviewCallbackWithBuffer(this);
	        camera.setPreviewCallback(this);
	    } else {
			//startup application
			//the camera parameters should already be set
			byte[] buffer1 = new byte[bufSize];
			byte[] buffer2 = new byte[bufSize];
			//byte[] buffer3 = new byte[bufSize];
			//byte[] buffer4 = new byte[bufSize];
		
			camera.addCallbackBuffer(buffer1);
			camera.addCallbackBuffer(buffer2);
			//camera.addCallbackBuffer(buffer3);
			//camera.addCallbackBuffer(buffer4);
	
			camera.setPreviewCallbackWithBuffer(this);
	    }
		
        this.camera.startPreview();
	}
	
	public void stopDetection() {
		if(this.textured)
		{
			this.camera.setPreviewCallback(null);
		} else {
			this.camera.setPreviewCallbackWithBuffer(null);
		}
		this.camera.stopPreview();
		ndetector.free();
		this.camera = null;
	}
	
	static long startTime = 0;
	static long fullTime = 0;
	static long frameCount = 0;
	
	@Override
	public void onPreviewFrame(byte[] data, Camera camera) {
		
		if(startTime == 0)
		{
			startTime = System.currentTimeMillis();
		}
		long sTime = System.currentTimeMillis();
		
		UMFNative.Result tempresult = new UMFNative.Result();
		int success = ndetector.updateYUV(data, cameraSize.width, cameraSize.height, tempresult);
		//Log.i("UMF", "data received");
		if(success > 0)
		{
			this.resultCache = tempresult;
		}
		
		if(this.detcb != null)
		{
			this.detcb.onUMFDetected(success);
		}

		if(!textured && this.rendercb != null)
		{
			this.rendercb.onUpdateRenderer(data, cameraSize.width, cameraSize.height, this.camera);
		} 
		//add back buffer to the front
		//camera.addCallbackBuffer(data);
		this.rendercb.synchronizeFrames();
		
		long eTime = System.currentTimeMillis();
		fullTime += eTime - sTime;
		frameCount++;
		if(frameCount > 100)
		{
			Log.i(TAG, "Avg java time: "+(fullTime*1.0/frameCount));
			Log.i(TAG, "Avg FPS: "+ (frameCount*1000.0/(eTime-startTime)));
			startTime = eTime;
			frameCount = 0;
			fullTime = 0;
		}
	}
	
	public void getPosition(float[] center)
	{
		if(resultCache != null)
		{
			center[0] = resultCache.positionX;
			center[1] = resultCache.positionY;
			center[2] = resultCache.positionZ;
		} else {
		
			center[0] = -1;
			center[1] = -1;
			center[2] = -1;
		}
	}
	
	public void getRotation(float[] quaternion)
	{
		if(resultCache != null)
		{
			quaternion[0] = resultCache.quatW;
			quaternion[1] = resultCache.quatX;
			quaternion[2] = resultCache.quatY;
			quaternion[3] = resultCache.quatZ;
		} else {
			quaternion[0] = 1;
			quaternion[1] = 0;
			quaternion[2] = 0;
			quaternion[3] = 0;
		}
	}

	public boolean isUpdateRender() {
		return updateRender;
	}

	public void setUpdateRender(boolean updateRender) {
		this.updateRender = updateRender;
	}

	public RendererCallback getRenderCallback() {
		return rendercb;
	}

	public void setRenderCallback(RendererCallback rendercb) {
		this.rendercb = rendercb;
	}

}
