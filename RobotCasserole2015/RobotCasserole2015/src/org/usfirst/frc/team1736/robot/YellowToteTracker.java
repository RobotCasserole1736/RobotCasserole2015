package org.usfirst.frc.team1736.robot;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.TimerTask;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.vision.AxisCamera;

/*
 * @author: Elijah Kaufman and his team
 * 
 * Note From Author: If you use this code or the algorithm
 * please give credit to Elijah Kaufman and FRC team 3019, Firebird Robotics
 */

/*
 * @author: Nick Dunne, Robot Casserole 1736
 * 
 * Removed need for javax.ImageIO by using CV VideoCapture
 */


public class YellowToteTracker{
	
	java.util.Timer timer;
	double period = 200;
	
	private class TrackerTask extends TimerTask
	{
		private YellowToteTracker toteTracker;
		
		public TrackerTask(YellowToteTracker tracker)
		{
			if (tracker == null)
			{
				throw new NullPointerException("Given Tracker was null");
			}
			toteTracker = tracker;
		}
		
		public void run()
		{
			toteTracker.processImage();
		}
	}
	static NetworkTable table;
	VideoCapture cap;
	public static final String CAMERA_URL = "http://10.17.36.11/mjpg/video.mjpg"; 
	//Color constants
	public static final Scalar 
	Red = new Scalar(0, 0, 255),
	Blue = new Scalar(255, 0, 0),
	Green = new Scalar(0, 255, 0),
	Yellow = new Scalar(0, 255, 255),
		//for yellow
		thresh_Lower = new Scalar(0,150,150),
		thresh_Higher = new Scalar(50,255,255),
		//for grey tote
		grey_Lower = new Scalar(48,60,35),
		grey_higher = new Scalar(81,84,54);
	
	static final boolean Process_Gray = false;
	public static ArrayList<MatOfPoint> contours = new ArrayList<>();
	public static Mat frame,grey,original;
	static int counter = 0;
	
	Image axisFrame;
	AxisCamera camera;
	
	/**
	 * Only call the constructor after doing a System.Load("/path/to/libopencv_2410.so"); second(s) earlier
	 */
	public YellowToteTracker() {
		//initialize network table
		table = NetworkTable.getTable("SmartDashboard");
		
		//Opens a new connection to the Axis camera
		cap = new VideoCapture();
		cap.open("http://10.17.36.11/mjpg/video.mjpg");
		
		//Create image Mat's
		original = new Mat();
		frame = new Mat();
		grey = new Mat();
		
//		axisFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
//		camera = new AxisCamera("10.17.36.11");
		
		timer = new java.util.Timer();
		timer.schedule(new TrackerTask(this), 0L, (long)(period *1000));
	}
	
	/**
	 * This should be called any time you would like to process an image and update the SmartDashboard values
	 */
	public synchronized void processImage(){
		try {
			//time for the OpenCV fun!
			
			//Read the latest image from the camera feed
			cap.read(original);
			frame = original.clone();
			if(Process_Gray)
				grey = original;
			//applies a threshhold in the form of BlueGreenRed
			Core.inRange(original, thresh_Lower, thresh_Higher, frame);
			
			Imgproc.dilate(frame, frame, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2)));
			Imgproc.dilate(frame, frame, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2)));
			Imgproc.dilate(frame, frame, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2)));
			Imgproc.erode(frame, frame, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2)));
			Imgproc.erode(frame, frame, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2)));
			Imgproc.erode(frame, frame, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2)));

			//find the cluster of particles
			Imgproc.findContours(frame, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
			//iterating through the list of contours and removing the ones with an "area" less then 100
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				if(matOfPoint.width() * matOfPoint.height() < 200){
					iterator.remove();
				}
			}
			MatOfPoint biggestContour = null;
			
			if(contours.size() > 0)
				biggestContour = contours.get(0);
			
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				if(matOfPoint.width() * matOfPoint.height() < biggestContour.width() * biggestContour.height()){
					biggestContour = matOfPoint;
				}
			}
			//if theres only one contour dont do the silly math of the bounding rectangles
			table.putNumber("Contours", contours.size());
			if(contours.size() >= 1){
					Rect rec1 = Imgproc.boundingRect(biggestContour);
					Core.rectangle(original, rec1.tl(), rec1.br(), Yellow);
					String string = "TargetFound at X:" + (rec1.tl().x + rec1.br().x) / 2 + "Y:" + (rec1.tl().y + rec1.br().y) / 2;
					Core.putText(original, string, new Point(200,frame.size().height-10), Core.FONT_HERSHEY_PLAIN, 1, Red);
					table.putNumber("Off Center", (rec1.x + rec1.width/2 - frame.width()/2));
					table.putNumber("ToteToScreen", rec1.width / 640.0);
					table.putBoolean("TargetVisible", true);
//					com.ni.vision.NIVision.Image im = com.ni.vision.NIVision.imaqCreateImage(com.ni.vision.NIVision.ImageType.IMAGE_RGB, 0);
//					byte[] bytes = new byte[frame.width() * frame.height() * frame.channels()];
//					original.get(0, 0, bytes);
//					ByteBuffer buf = ByteBuffer.wrap(bytes);
//					com.ni.vision.NIVision.RawData data = new com.ni.vision.NIVision.RawData(buf);
//					com.ni.vision.NIVision.imaqArrayToImage(im, data, frame.width(), frame.height());
//					CameraServer.getInstance().setImage(im);
			}
			else
				table.putBoolean("TargetVisible", false);
			
			//releases the mats from ram...frees up so much memory!
			original.release();
			grey.release();
			frame.release();
			contours.clear();
		//mostly for debugging but errors happen
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();	
		}
	}
}
