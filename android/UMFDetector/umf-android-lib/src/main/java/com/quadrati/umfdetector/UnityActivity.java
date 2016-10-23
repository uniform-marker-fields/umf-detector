package com.quadrati.umfdetector;

import java.util.List;

import android.hardware.Camera;
import android.hardware.Camera.CameraInfo;
import android.hardware.Camera.Size;
import android.util.Log;


public class UnityActivity implements Detector.DetectionCallback{

	private static String TAG = "UMFUnityActivity";
	private Detector mUMF;
	private UnityRenderer mRend;
	Camera mCamera;
    int numberOfCameras;
    int cameraCurrentlyLocked;
	int defaultCameraId;
	
	public void onCreate() {
        
		mUMF = new Detector();
		mUMF.setCallback(this);
		
		mRend = new UnityRenderer();
        
		// Find the total number of cameras available
		
        numberOfCameras = Camera.getNumberOfCameras();
    
        // Find the ID of the default camera
        CameraInfo cameraInfo = new CameraInfo();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.getCameraInfo(i, cameraInfo);
            if (cameraInfo.facing == CameraInfo.CAMERA_FACING_BACK) {
                defaultCameraId = i;
            }
        }
    }


	public void onResume(int displayWidth, int displayHeight) {
		this.onResume(displayWidth, displayHeight, false);
	}
	
	public void onResume(int displayWidth, int displayHeight, boolean chroma)
	{
		mCamera = Camera.open();
        Camera.Parameters p = mCamera.getParameters();
        p.setPreviewSize(640, 480);

        List<Size> sizes = p.getSupportedPictureSizes();
        Size result = null;
        for (int i=0;i<sizes.size();i++){
            result = (Size) sizes.get(i);
            Log.i("PictureSize", "Supported Size. Width: " + result.width + "height : " + result.height); 
        }
        mCamera.setParameters(p);
        cameraCurrentlyLocked = defaultCameraId;
		mUMF.setChroma(chroma);
		mUMF.setRenderCallback(mRend);
        mUMF.startDetection(mCamera);
        mRend.onRendererChanged(displayWidth, displayHeight, chroma);
	}
	
	public void onRenderUpdate()
	{
		this.mRend.onDrawFrame();
	}
	
	public void onPause()
	{
		if (mCamera != null) {
    		mUMF.setRenderCallback(null);
        	mUMF.stopDetection();
            mCamera.release();
            mCamera = null;
        }
	}

	@Override
	public void onUMFDetected(int success) {
		if(mUMF != null)
		{
			float[] cposition = new float[3];
			mUMF.getPosition(cposition);
			
			//Log.i(TAG, String.format("%d - %.1f;%.1f;%.1f", success, cposition[0], cposition[1], cposition[2]));
		}
	}

	public int loadCSV(String contents) {
		int loadResult = mUMF.setMarkerCSV(contents);
		Log.i(TAG, String.format("Changing marker has returned: %d", loadResult));
		return loadResult;
	}

	public int loadXML(String contents) {
		int loadResult = mUMF.setMarkerXML(contents);
		Log.i(TAG, String.format("Changing marker has returned: %d", loadResult));
		return loadResult;
	}
	
	public float getRendererFovy()
	{
		return mUMF.ndetector.getRendererFovy();
	}
	
	public float[] getPosition()
	{
		float[] cposition = new float[3];
		mUMF.getPosition(cposition);
		return cposition;
	}
	
	public float[] getPositionRotation()
	{
		float[] cposition = new float[3];
		mUMF.getPosition(cposition);
		
		float[] cquat = new float[4];
		mUMF.getRotation(cquat);
		
		float[] posrot = new float[7];
		posrot[0] = cposition[0];
		posrot[1] = cposition[1];
		posrot[2] = cposition[2];
		posrot[3] = cquat[1];
		posrot[4] = cquat[2];
		posrot[5] = cquat[3];
		posrot[6] = cquat[0];
		//Log.i(TAG, "Get pos and rotation");
		return posrot;
	}
}
