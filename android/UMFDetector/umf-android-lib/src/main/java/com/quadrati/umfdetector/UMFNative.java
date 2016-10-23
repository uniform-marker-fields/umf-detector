package com.quadrati.umfdetector;

import java.io.File;
import java.io.FileOutputStream;
import java.util.Random;

import android.graphics.Bitmap;
import android.os.Environment;

public class UMFNative {
	
	public static class Result {

	    public float positionX;
	    public float positionY;
	    public float positionZ;

		public float quatX;
		public float quatY;
		public float quatZ;
		public float quatW;
	}
	
	public long detectorPointer;
	public Bitmap bitmap;
	
	UMFNative(){
		this.detectorPointer = 0;
		this.bitmap = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
	}
	

	/**
	 * @param width the current view width
	 * @param height the current view height
	 */
	public static native void initGLSetPreviewSize(int camWidth, int camHeight);
	public static native void initGL(int width, int height, boolean chroma);
	public static native void stepDraw();
	

	public static native void initGLTextured(int width, int height, int textureID, boolean chroma);
	public static native void stepDrawTextured();
	public static native void updateCameraTexture(int id, float[] mvp);
	
	public static native void updateCameraPreview(byte[] yuvData, int width, int height);
	
	public native void init(int width, int height, float fovx, float fovy, boolean chroma);
	public native int setMarkerCSV(String contents);
	public native int setMarkerXML(String contents);
	public native void free();
	
	public native int updateYUV( byte[] yuvData, int width, int height, Result result);
	public native int update(byte[] data, int width, int height, Result result);
	
	public native float getRendererFovy();
	
	public void SaveImage() {

	    String root = Environment.getExternalStorageDirectory().toString();
	    File myDir = new File(root + "/saved_images");    
	    myDir.mkdirs();
	    Random generator = new Random();
	    int n = 10000;
	    n = generator.nextInt(n);
	    String fname = "Image-"+ n +".jpg";
	    File file = new File (myDir, fname);
	    if (file.exists ()) file.delete (); 
	    try {
	           FileOutputStream out = new FileOutputStream(file);
	           this.bitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
	           out.flush();
	           out.close();

	    } catch (Exception e) {
	           e.printStackTrace();
	    }
	}
	
	static {
		System.loadLibrary("UMFDetector");
	}

}

