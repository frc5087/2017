package org.usfirst.frc.team5087.robot;

/**
 * @author marsd
 *
 */

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.FeedbackDeviceStatus;

public class Movement 
{
	public static final int LEFT  = 0;
	public static final int RIGHT = 1;
	
    private final double mm = 25.4;					// mm to an inch.

    private final double Di = 6.0;                  	// Wheel diameter.
    private final double Ti = 1.0;                  	// Tread width.
    private final double Wi = 20.75 - Ti;           	// Distance between wheel centers.

    private final double rW = (Di * mm) / 2.0;      	// Radius of drive wheel.
    private final double b  = (Wi * mm);            	// Distance between center of each wheel tread.
	
    private int[]		last_ = new int[2];
	private CANTalon[]  tal_ = new CANTalon[2];
	private String[]  names_ = new String[] { "left", "right" };

	private double x_;
	private double y_;
	private double r_;
	
	/*
	 * Pass the left hand side and right hand side CANTalon's for us to use to read the encoders.
	 */
	
	Movement(CANTalon _left, CANTalon _right)
	{
		System.out.println("-> Movement()");
		
		tal_[LEFT]  = _left;
		tal_[RIGHT] = _right;

		CheckSensor(LEFT);
		CheckSensor(RIGHT);

		System.out.println("<- Movement()");
	}

	/*
	 * Check the sensor is present on the specified side.
	 * 
	 * true  - sensor is there.
	 * false - sensor not found.
	 */

	boolean CheckSensor(int _side)
	{
		System.out.println("-> CheckSensor(" + _side + ")");

		boolean ret = true;
		
		if(tal_[_side].isSensorPresent(FeedbackDevice.CtreMagEncoder_Absolute)
		!= FeedbackDeviceStatus.FeedbackStatusPresent)
		{
			ret = false;
			
			System.out.println("Sensor on " + names_[_side] + " side missing.");
		}
		else
		{
			last_[_side] = position(_side);
			
			tal_[_side].setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			tal_[_side].reverseSensor(false);				
			
			System.out.println("Sensor on " + names_[_side] + " side configured.");
		}

		System.out.println("<- " + ret + " CheckSensor()");

		return ret;
	}

	/*
	 * General information dump code.
	 */
	
	void info(int _side)
	{
		System.out.println(names_[_side] + " side:");
		
		System.out.println("PulseWidthPosition ....... " + tal_[_side].getPulseWidthPosition());
		System.out.println("PulseWidthRiseToFallUs ... " + tal_[_side].getPulseWidthRiseToFallUs());
		System.out.println("PulseWidthRiseToRiseUs ... " + tal_[_side].getPulseWidthRiseToRiseUs());
		System.out.println("PulseWidthVelocity ....... " + tal_[_side].getPulseWidthVelocity());
	}
	
	int position(int _side)
	{
		return tal_[_side].getPulseWidthPosition();
	}

	/*
	 * Return the relative movement of a side wheel - 1.0 is a full rotation.
	 */
	
	double movement(int _side)
	{
		final int FLUTTER = 2;
		
		int now = position(_side);
		
		int diff = last_[_side] - now;
		
		last_[_side] = now;
		
		double ret = 0.0;
		
		if((diff < -FLUTTER) || (diff > FLUTTER))
		{
			ret = (double) diff / 4096.0;
			
			if(_side == RIGHT)
			{
				ret = -ret;
			}
		}
		
		return ret; 
	}
    
	/*
     * Set the start position and rotation of the robot.
    */

	public void setXYR(double _x, double _y, double _r)
	{
		x_ = _x;
		y_ = _y;
		r_ = _r;
	}

	/*
     * Get the current position and rotation of the robot.
	 */

	public double getX()
	{
    	return x_;
	}

	public double getY()
	{
    	return y_;
	}

	public double getR()
	{
		return r_;
	}

	/*
     * Update the current position and rotation of the robot from the rotation of each wheel.
	 */

	public void update(double _ml, double _mr)
	{
		double aL = _mr * Math.PI * 2.0;
		double aR = _ml * Math.PI * 2.0;
    
		double r = Math.toRadians(r_);

		if(_ml == _mr)
		{
			x_ = x_ + ((((aR + aL) * rW) / 2) * Math.cos(r));
			y_ = y_ + ((((aR + aL) * rW) / 2) * Math.sin(r));
		}
		else
		{
			double a = r + (((aR - aL) * rW) / b);

			if(_ml != -_mr)
			{
				x_ = x_ - ((((aR + aL) * b) / ((aR - aL) * 2)) * (Math.sin(r) - Math.sin(a)));
				y_ = y_ + ((((aR + aL) * b) / ((aR - aL) * 2)) * (Math.cos(r) - Math.cos(a)));
			}

			r_ = Math.toDegrees(a);
		}
	}
	
	/*
	 * Calculate the direction to be to move towards the specified point.
	 */

	public double direction(double _x, double _y)
	{
    	double dx = _x - x_;
    	double dy = _y - y_;

    	double a = Math.atan2(dy, dx);

    	return Math.toDegrees(a);
	}

	/*
	 * Calculate the distance to travel to get to the specified point.
	 */
	
	public double distance(double _x, double _y)
	{
    	double dx = _x - x_;
    	double dy = _y - y_;

    	return Math.sqrt((dx * dx) + (dy * dy));
	}
}