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

	/*
	 * Constructor.
	 */
	
	Vision()
	{
		hsv_	 = new Mat();
		image_ 	 = new Mat();

		temp_ 	 = new Mat();
		
		// Allocate the storage for the generated contours.

		contours_ = new ArrayList<MatOfPoint>();

		// Create the low and high HSV values to generate the contours.
		
		lowHSV_  = new Scalar(55, 100, 100);
		highHSV_ = new Scalar(95, 255, 255);
	}

	/*
	 * Process the images and grab the co-ords of the two boxes that should be on screen.
	 */
	
	void Process(Mat _original)
	{
		// Convert the grabbed image to HSV format so we can work with it.

		Imgproc.cvtColor(_original, hsv_,
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

        // Send the information to the main code.
		
	}
}