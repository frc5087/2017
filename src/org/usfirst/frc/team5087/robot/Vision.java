/**
 * 
 */

package org.usfirst.frc.team5087.robot;

/**
 * @author james
 *
 */

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Vision
{
	Mat		hsv_;
	Mat 	image_;

	Mat 	temp_;

	List<MatOfPoint> contours_;

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

		// Create the low and high HSV values to generate the contours.
		
		lowHSV_  = new Scalar(55, 100, 100);
		highHSV_ = new Scalar(95, 255, 255);

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
		
	}

	/*
	 * Process the images and grab the co-ords of the two boxes that should be on screen.
	 */
	
	void process(Mat _image)
	{
		// Convert the grabbed image to HSV format so we can work with it.

		Imgproc.cvtColor(_image, hsv_,
						 Imgproc.COLOR_BGR2HSV);

		// Grab an black and white image with white as the selected area.

		Core.inRange(hsv_, lowHSV_, highHSV_, image_);

		hsv_.release();

		// Clear the previous contours and grab the new ones.
		
		contours_.clear();
		
		Imgproc.findContours(image_, contours_, temp_,
							 Imgproc.RETR_LIST,
							 Imgproc.CHAIN_APPROX_SIMPLE);

		image_.release();
        temp_.release();

		// Grab the co-ords of the corners of the box(es) from the contour list.

        _image.release();
	}
}