package org.usfirst.frc.team5087.robot;

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
}