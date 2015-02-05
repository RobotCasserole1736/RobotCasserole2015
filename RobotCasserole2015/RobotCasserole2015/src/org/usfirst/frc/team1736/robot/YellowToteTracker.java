package org.usfirst.frc.team1736.robot;

import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

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
		thresh_Lower = new Scalar(0,110,110),
		thresh_Higher = new Scalar(50,255,255),
		//for grey tote
		grey_Lower = new Scalar(48,60,35),
		grey_higher = new Scalar(81,84,54);
	
	static final boolean Process_Gray = false;
	public static ArrayList<MatOfPoint> contours = new ArrayList<>();
	public static Mat frame,grey,original;
	static int counter = 0;
	
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
	}
	
	/**
	 * This should be called any time you would like to process an image and update the SmartDashboard values
	 */
	public void processImage(){
		try {
			//time for the OpenCV fun!
			
			//Read the latest image from the camera feed
			cap.read(original);
			frame = original.clone();
			if(Process_Gray)
				grey = original;
			//applies a threshhold in the form of BlueGreenRed
			Core.inRange(original, thresh_Lower, thresh_Higher, frame);
			//find the cluster of particles
			Imgproc.findContours(frame, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
			//iterating through the list of contours and removing the ones with an "area" less then 100
			for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
				MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
				if(matOfPoint.width() * matOfPoint.height() < 20){
					iterator.remove();
				}
			}
			//if theres only one contour dont do the silly math of the bounding rectangles
			if(contours.size() == 1){
					Rect rec1 = Imgproc.boundingRect(contours.get(0));
					Core.rectangle(original, rec1.tl(), rec1.br(), Yellow);
					String string = "TargetFound at X:" + (rec1.tl().x + rec1.br().x) / 2 + "Y:" + (rec1.tl().y + rec1.br().y) / 2;
					Core.putText(original, string, new Point(200,frame.size().height-10), Core.FONT_HERSHEY_PLAIN, 1, Red);
			}
			//i wonder if there are two totes...
			else if(contours.size() == 2 || contours.size() == 4){
				ArrayList<Rect> target1 = new ArrayList<Rect>();
				ArrayList<Rect> target2 = new ArrayList<Rect>();
					target1.add(Imgproc.boundingRect(contours.get(0)));
					target1.add(Imgproc.boundingRect(contours.get(1)));
					if(contours.size() == 4){
						target2.add(Imgproc.boundingRect(contours.get(2)));
						target2.add(Imgproc.boundingRect(contours.get(3)));
					}
				Point tl = target1.get(0).tl();
				Point br = target1.get(0).br();
				//outer bounds of one rectangle
				for(Rect rec : target1){
					if(tl.x > rec.tl().x){
						tl.x = rec.tl().x;
					}
					if(tl.y > rec.tl().y){
						tl.y = rec.tl().y;
					}
					if(br.x < rec.br().x){
						br.x = rec.br().x;
					}
					if(br.y < rec.br().y){
						br.y = rec.br().y;
					}
					table.putBoolean("TargetVisible", true);
				}
				Rect bb1 = new Rect(tl, br);
				Core.rectangle(original, bb1.br(),bb1.tl(), Yellow);
				Rect bb2 = null;
				//the 4 L will make 4 contours
				//if there was 4 contours then fill the outer bounds of the second rectangle
				if(!target2.isEmpty()){
					tl = target2.get(0).tl();
					br = target2.get(0).br();
					for(Rect rec : target2){
						if(tl.x > rec.tl().x){
							tl.x = rec.tl().x;
						}
						if(tl.y > rec.tl().y){
							tl.y = rec.tl().y;
						}
						if(br.x < rec.br().x){
							br.x = rec.br().x;
						}
						if(br.y < rec.br().y){
							br.y = rec.br().y;
						}
					}
					bb2 = new Rect(tl, br);
					table.putBoolean("TargetVisible", true);
				}
				String string;
				if(null != bb2){
					Core.rectangle(original, bb2.br(),bb2.tl(), Yellow);
					//checking to see if the totes line up, because stacked totes line up!
					if(Math.abs(bb1.x-bb2.x) < 5){
						string = "Two totes stacked";
						Core.line(original, new Point(bb1.x+bb1.width/2,bb1.y+bb1.height/2),
								new Point(bb2.x+bb2.width/2,bb2.y+bb2.height/2), Green);
					}
					else{
					string = "Two totes found";
					}
				}
				else{
					//knowing how far off the center of the image you can set your robot to align with it, essentially going towards it
					string = "Off center by " + (bb1.x + bb1.width/2 - 330) + " pixels";
					table.putNumber("Off Center", (bb1.x + bb1.width/2 - 330));
					table.putNumber("ToteToScreen", bb1.width / 640.0);
				}
				//putting the data in a visible form
				Core.putText(original, string, new Point(200,frame.size().height-10), Core.FONT_HERSHEY_PLAIN, 1, Red);
				table.putBoolean("TargetVisible", true);
			}
			else if(contours.size() == 3){
				ArrayList<Rect> rect = new ArrayList<>();
				for(MatOfPoint mOP : contours){
					rect.add(Imgproc.boundingRect(mOP));
					Rect temp = rect.get(rect.size()-1);
					Core.rectangle(original, temp.br(),temp.tl(), Red);
				}
				for(int i = 1; i < contours.size();i++){
					if(Math.abs(rect.get(i).y - rect.get(i-1).y) < 5){
						
						Rect r1 = rect.get(i);
						Rect r2 = rect.get(i-1);
						Point tl = r1.tl();
						Point br = r1.br();
						if(tl.x > r2.tl().x){
							tl.x = r2.tl().x;
						}
						if(tl.y > r2.tl().y){
							tl.y = r2.tl().y;
						}
						if(br.x < r2.br().x){
							br.x = r2.br().x;
						}
						if(br.y < r2.br().y){
							br.y = r2.br().y;
						}
						break;
					}
				}
				table.putBoolean("TargetVisible", true);
			}
			else{
				table.putBoolean("TargetVisible", false);
				for(MatOfPoint mOP : contours){
					Rect rec = Imgproc.boundingRect(mOP);
					Core.rectangle(original, rec.tl(), rec.br(), Red);
				}
				contours.clear();
			}
			if(Process_Gray){
				//processing for grey totes
				Core.inRange(grey, grey_Lower, grey_higher, grey);
				Imgproc.findContours(grey, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
				Imgproc.cvtColor(grey, grey, Imgproc.COLOR_GRAY2BGR);
				//iterating through the list of contours and removing the ones with an "area" less then 100
				for (Iterator<MatOfPoint> iterator = contours.iterator(); iterator.hasNext();) {
					MatOfPoint matOfPoint = (MatOfPoint) iterator.next();
					if(matOfPoint.width() * matOfPoint.height() < 100){
						iterator.remove();
					}	
				}
				Rect bb;
				for(MatOfPoint mop : contours){
					bb = Imgproc.boundingRect(mop);
					Core.rectangle(grey, bb.tl(), bb.br(), Red);
				}
			}
			//gui on the image
			Core.line(original, new Point(frame.width()/2,100),new Point(frame.width()/2,frame.height()-100), Blue);
			Core.line(original, new Point(150,frame.height()/2),new Point(frame.width()-150,frame.height()/2), Blue);
			//releases the mats from ram...frees up so much memory!
			original.release();
			grey.release();
			frame.release();
		//mostly for debugging but errors happen
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();	
		}
	}
}
