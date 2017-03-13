package org.usfirst.frc.team5087.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

public class MotorControl
{
    static final int	LEFT_MASTER	= 0;
    static final int	LEFT_SLAVE		= 1;
    static final int	RIGHT_MASTER	= 2;
    static final int	RIGHT_SLAVE	= 3;
    
	CANTalon[]	talons_ = new CANTalon[4];

	MotorControl()
	{
    	// Motor controllers for the robot movement.

    	if(InstalledHardware.DRIVE == true)
    	{
    		// Setup the left hand side where #4 has the left gear-box sensor.
    		
        	talons_[LEFT_MASTER]	= new CANTalon(4);
        	talons_[LEFT_SLAVE]	= new CANTalon(8);
        	
        	talons_[LEFT_SLAVE].changeControlMode(TalonControlMode.Follower);
        	talons_[LEFT_SLAVE].set(talons_[LEFT_MASTER].getDeviceID());
        	
        	// Setup the right hand side where #2 has the right gear-box sensor.
        	
        	talons_[RIGHT_MASTER]	= new CANTalon(2);
        	talons_[RIGHT_SLAVE]	= new CANTalon(1);

        	talons_[RIGHT_SLAVE].changeControlMode(TalonControlMode.Follower);
        	talons_[RIGHT_SLAVE].set(talons_[RIGHT_MASTER].getDeviceID());

        	// Setup the basic information for all of the motor controllers.

        	for(int i = 0; i < 4; ++i)
        	{
            	talons_[i].configMaxOutputVoltage(12.0);
            	talons_[i].configNominalOutputVoltage(12.0, 12.0);
            	talons_[i].configPeakOutputVoltage(12.0, 12.0);
        	}
    	}
	}
	
	CANTalon left()
	{
		return talons_[LEFT_MASTER];
	}
	
	CANTalon right()
	{
		return talons_[RIGHT_MASTER];
	}
}