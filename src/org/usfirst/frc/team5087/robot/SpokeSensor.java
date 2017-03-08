package org.usfirst.frc.team5087.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.AnalogInput;

public class SpokeSensor
{
    static final int		SENSORS		= 6;						// Number of sensors to scan.

	static final int		SPOKES			= 5;						// Spokes on the gear.
	static final double	SPOKE_ANGLE	= 360.0 / SPOKES;			// Angle between spokes.
	
	static final double	SENSOR_ANGLE	= SPOKE_ANGLE / SENSORS;

    static final double	COVERED		= 2.0;						// Value when fully covered.
    static final double	UNCOVERED		= 4.6;						// Value when completely uncovered.
    
    static final double	FRACTION		= SENSOR_ANGLE / (UNCOVERED - COVERED);

	private	AnalogInput[]	sensor_ = new AnalogInput[SENSORS];

	/*
	 * Constructor.
	 */
	
	SpokeSensor()
	{
		for(int i = 0; i < SENSORS; ++i)
		{
			sensor_[i] = new AnalogInput(i);
		}
	}

	/*
	 * Return the angle of the spoke, or -1 if spoke not found. 
	 */
	
	public double position()
	{
		double	ret = -1;
		
		double	angle = 0;
		
		// Only a maximum of two sensors can be covered at once.
		
		for(int i = 0; i < SENSORS; ++i)
		{
			double voltage0 = sensor_[i].getVoltage();
			
			if(voltage0 < UNCOVERED)
			{
				double	voltage1 = sensor_[(i + 1) % SENSORS].getVoltage();
				
				if(voltage1 < UNCOVERED)
				{
					angle += (voltage0 - COVERED) * FRACTION;
				}
				
				ret = angle;
				
				break;
			}
			
			angle += (SPOKE_ANGLE / SENSORS);
		}
		
		return ret;
	}
	
	/*
	 * Show the gear rotation on screen.
	 */
	
	public void show(Mat _image, double _rotation)
	{
		double centery = 40;
		double	centerx = 320 - centery;
		double radius0 = 5;
		double radius1 = 30;
		double radius2 = 35;

		if(_rotation != -1)
		{
			double angle = Math.toRadians(_rotation - 90);
			double step  = Math.toRadians(SPOKE_ANGLE);
			
			Point center = new Point(centerx, centery);
			
			for(int i = 0; i < SPOKES; ++i)
			{
				double xs = centerx + (radius0 * Math.cos(angle));
				double ys = centery + (radius0 * Math.sin(angle));

				double xe = centerx + (radius1 * Math.cos(angle));
				double ye = centery + (radius1 * Math.sin(angle));

				Point	ps = new Point(xs, ys);
				Point	pe = new Point(xe, ye);

				Imgproc.line(_image, ps, pe, Colours.YELLOW);

				angle += step;
			}

			angle = Math.toRadians((_rotation + (SPOKE_ANGLE / 4)) - 90);
			step  = Math.toRadians(SPOKE_ANGLE / 2);

			for(int i = 0; i < (SPOKES * 2); ++i)
			{
				double xs = centerx + (radius1 * Math.cos(angle));
				double ys = centery + (radius1 * Math.sin(angle));

				double xe = centerx + (radius2 * Math.cos(angle));
				double ye = centery + (radius2 * Math.sin(angle));

				Point	ps = new Point(xs, ys);
				Point	pe = new Point(xe, ye);

				Imgproc.line(_image, ps, pe, Colours.YELLOW);

				angle += step;
				
			}
			
			Imgproc.circle(_image, center, (int) radius0, Colours.YELLOW);
			Imgproc.circle(_image, center, (int) radius1, Colours.YELLOW);
		}
		else
		{
			Point	tl = new Point(centerx - radius1, centery - radius1);
			Point	tr = new Point(centerx + radius1, centery - radius1);
			Point	bl = new Point(centerx - radius1, centery + radius1);
			Point	br = new Point(centerx + radius1, centery + radius1);

			Imgproc.line(_image, tl, br, Colours.RED);
			Imgproc.line(_image, tr, bl, Colours.RED);
		}
	}
}