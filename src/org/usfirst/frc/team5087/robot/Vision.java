package org.usfirst.frc.team5087.robot;

/**
 * @author james
 *
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Stack;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Vision
{
	Mat		hsv_;
	Mat 	image_;

	Mat 	temp_;

	List<MatOfPoint> contours_;
	List<RotatedRect> rectangles_;

	Scalar	lowHSV_;
	Scalar	highHSV_;
	
    Stack<Mat>	visionControl_;

	Robot	robot_;
	
	Thread	visionThread_;

	/*
	 * Constructor.
	 */
	
	Vision(Robot _robot)
	{
		robot_	= _robot;
		
		hsv_	= new Mat();
		image_ 	= new Mat();

		temp_ 	= new Mat();
		
		// Allocate the storage for the generated contours.

		contours_ = new ArrayList<MatOfPoint>();
		rectangles_ = new ArrayList<RotatedRect>();

		// Create the low and high HSV values to generate the contours.
		
		lowHSV_  = new Scalar(50, 100, 100);			// 55, 100, 100
		highHSV_ = new Scalar(95, 255, 255);			// 95, 255, 255

    	visionControl_ = new Stack<Mat>();

		// Thread to process the image.

        visionThread_ = new Thread(() ->
        {
            while(!Thread.interrupted())
        	{
            	if(visionControl_.empty() == false)
            	{
            		// Process the image.
            		
            		process(visionControl_.pop());

            		// Request a new image to process.
            		
            		robot_.request(Robot.FRONT_IMAGE);
            	}
            	else
            	{
            		try
            		{
						Thread.sleep(50);
					}
            		
            		catch (InterruptedException e)
            		{
						e.printStackTrace();
					}
            	}
        	}
        });
        
		visionThread_.setDaemon(true);
		visionThread_.start();

		// Request a new image to start the processing.
		
		robot_.request(Robot.FRONT_IMAGE);
	}
	
	/*
	 * Receive the image to be processed.
	 */
	
	public void give(Mat _image)
	{
		visionControl_.push(_image);
	}
	
	/*
	 * Show the last grabbed contours on the screen.
	 */
	
	public void show(Mat _image)
	{
		synchronized(rectangles_)
		{
			if(rectangles_.isEmpty() == false)
			{
				Point[] vertices = new Point[4];
				
				for(RotatedRect rectangle : rectangles_)
				{
					rectangle.points(vertices);
					
					MatOfPoint points = new MatOfPoint(vertices);
					
					Imgproc.drawContours(_image,
										 Arrays.asList(points),
										 -1,
										 Colours.RED,
										 2);
				}
			}
		}
	}
	
	/*
	 * Process the images and grab the co-ords of the two boxes that should be on screen.
	 */
	
	void process(Mat _image)
	{
		// Convert the grabbed image to HSV format so we can work with it.

		Imgproc.cvtColor(_image,
						 hsv_,
						 Imgproc.COLOR_BGR2HSV);

        _image.release();

		// Grab an black and white image with white as the selected area.

		Core.inRange(hsv_,
					 lowHSV_,
					 highHSV_,
					 image_);

		hsv_.release();

		// Clear the previous contours and grab the new ones.
		
		synchronized(contours_)
		{
			contours_.clear();
			
			Imgproc.findContours(image_,
								 contours_,
								 temp_,
								 Imgproc.RETR_EXTERNAL,
								 Imgproc.CHAIN_APPROX_SIMPLE);
		}
		
		image_.release();
        temp_.release();

		// Grab the co-ords of the corners of the box(es) from the contour list.

		synchronized(rectangles_)
		{
			rectangles_.clear();
			
			for(MatOfPoint contour : contours_)
			{
				if(contour.toArray().length < 4)
				{
					continue;
				}
				
				RotatedRect	rectangle = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
				
				if((rectangle.size.height < 8) || (rectangle.size.width < 8))
				{
					continue;
				}
				
				rectangles_.add(rectangle);
			}
		}
	}
}