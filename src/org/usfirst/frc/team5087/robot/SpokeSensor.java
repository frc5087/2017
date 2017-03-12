package org.usfirst.frc.team5087.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.AnalogInput;

public class SpokeSensor
{
	static final boolean	SHOW_VOLTAGE	= true;
	static final boolean SHOW_SPOKES	= true;
	static final boolean SHOW_TEETH		= false;
	
    static final int		SENSORS		= 6;						// Number of sensors to scan.

	static final int		SPOKES			= 5;						// Spokes on the gear.
	static final double	SPOKE_ANGLE	= 360.0 / SPOKES;			// Angle between spokes.
	static final int		SPOKE_WIDTH	= 2;						// Used for on-screen display.
	
	static final double	SENSOR_ANGLE	= SPOKE_ANGLE / SENSORS;

    static final double	COVERED		= 2.5;						// Value when fully covered.
    static final double	UNCOVERED		= 4.3;						// Value when completely uncovered.
    
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
	 * A negative rotation is used on screen as we are looking through the gear.
	 */
	
	@SuppressWarnings("unused")
	public void show(Mat _image, double _rotation)
	{
		if(SHOW_VOLTAGE == true)
		{
			System.out.printf("%1.2f %1.2f %1.2f %1.2f %1.2f %1.2f\n",
					   sensor_[0].getVoltage(),
					   sensor_[1].getVoltage(),
					   sensor_[2].getVoltage(),
					   sensor_[3].getVoltage(),
					   sensor_[4].getVoltage(),
					   sensor_[5].getVoltage());
		}
		
		double centery = 40;
		double	centerx = 320 - centery;
		double radius0 = 5;
		double radius1 = 30;
		double radius2 = 35;
		
		double rotation = _rotation;
		
		Scalar colour = Colours.YELLOW;
		
		if(_rotation == -1)
		{
			rotation = 0;
			
			colour = Colours.DARK_YELLOW;
		}
		
		Point center = new Point(centerx, centery);
		
		double angle, step;
		
		double	xs, ys, xe, ye;

		if(SHOW_SPOKES == true)
		{
			angle = Math.toRadians(-rotation - 90);
			step  = Math.toRadians(SPOKE_ANGLE);
			
			for(int i = 0; i < SPOKES; ++i)
			{
				xs = centerx + (radius0 * Math.cos(angle));
				ys = centery + (radius0 * Math.sin(angle));

				xe = centerx + (radius1 * Math.cos(angle));
				ye = centery + (radius1 * Math.sin(angle));

				Point	ps = new Point(xs, ys);
				Point	pe = new Point(xe, ye);

				Imgproc.line(_image, ps, pe, colour, SPOKE_WIDTH);
				
				angle += step;
			}
			
			Imgproc.circle(_image, center, (int) radius0, colour, SPOKE_WIDTH);
			Imgproc.circle(_image, center, (int) radius1, colour, SPOKE_WIDTH);
		}

		if(SHOW_TEETH == true)
		{
			angle = Math.toRadians((-rotation + (SPOKE_ANGLE / 4)) - 90);
			step  = Math.toRadians(SPOKE_ANGLE / 2);

			for(int i = 0; i < (SPOKES * 2); ++i)
			{
				xs = centerx + (radius1 * Math.cos(angle));
				ys = centery + (radius1 * Math.sin(angle));

				xe = centerx + (radius2 * Math.cos(angle));
				ye = centery + (radius2 * Math.sin(angle));

				Point	ps = new Point(xs, ys);
				Point	pe = new Point(xe, ye);

				Imgproc.line(_image, ps, pe, colour, SPOKE_WIDTH * 3);

				angle += step;
			}
		}
			
		if(_rotation == -1)
		{
			Point	tl = new Point(centerx - radius1, centery - radius1);
			Point	tr = new Point(centerx + radius1, centery - radius1);
			Point	bl = new Point(centerx - radius1, centery + radius1);
			Point	br = new Point(centerx + radius1, centery + radius1);

			Imgproc.line(_image, tl, br, Colours.RED, SPOKE_WIDTH);
			Imgproc.line(_image, tr, bl, Colours.RED, SPOKE_WIDTH);
		}
	}
}