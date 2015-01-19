package org.usfirst.frc.team1736.robot;

import java.nio.ByteBuffer;
import java.nio.MappedByteBuffer;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.Point;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class CameraVision 
{
	final String RIGHT_CAMERA_IP = "10.17.36.11";
	final String LEFT_CAMERA_IP = "10.17.36.12";
	
	AxisCamera 	cameraLeft = new AxisCamera(LEFT_CAMERA_IP), 
			cameraRight = new AxisCamera(RIGHT_CAMERA_IP);
	
	Image 	imageLeft = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0), 
		imageRight = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
	
	NIVision.Rect rect;
	NIVision.Point destLoc;
	NIVision.Point lineStart = new NIVision.Point(10, 10);
	NIVision.Point lineEnd = new NIVision.Point(10, 100);
	
	int h, w;
	
	
	public void useCamera()
	{
		boolean freshImage = cameraLeft.isFreshImage();
		
		cameraLeft.getImage(imageLeft);
		cameraRight.getImage(imageRight);
		
		if(freshImage)
		{
			h = NIVision.imaqGetImageSize(imageLeft).height;
			w = NIVision.imaqGetImageSize(imageLeft).width;
		}
		
		rect = new NIVision.Rect(0, 0, h/2, w/2);
		destLoc = new NIVision.Point(0,0);
		
		NIVision.imaqCopyRect(imageRight, imageLeft, rect, destLoc);
		NIVision.imaqDrawLineOnImage(imageRight, imageRight, DrawMode.DRAW_VALUE, lineStart, lineEnd, 5);
		CameraServer.getInstance().setImage(imageRight);
	}
	
}
