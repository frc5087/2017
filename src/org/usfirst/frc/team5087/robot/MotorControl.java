package org.usfirst.frc.team5087.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Timer;

public class MotorControl
{
    static final int	LEFT_MASTER	= 0;
    static final int	LEFT_SLAVE		= 1;
    static final int	RIGHT_MASTER	= 2;
    static final int	RIGHT_SLAVE	= 3;
    
	CANTalon[]	talons_ = new CANTalon[4];

	/*
	 * Main constructor.
	 * 
	 * Allocates and sets up the Talon motor controllers for driving the robot. 
	 */
	
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
            	talons_[i].configMaxOutputVoltage(12.0f);
            	talons_[i].configNominalOutputVoltage(0.0f, 0.0f);
            	talons_[i].configPeakOutputVoltage(12.0f, 12.0f);
        	}
    	}
	}
	
	/*
	 * Return the Talon that controls the left hand side motors.
	 */
	
	CANTalon left()
	{
		return talons_[LEFT_MASTER];
	}

	/*
	 * Return the Talon that controls the right hand side motors.
	 */
	
	CANTalon right()
	{
		return talons_[RIGHT_MASTER];
	}

	/*
	 * Calculate the PID values for the left and right motors.
	 * 
	 * @param	_talon			Talon SRX to configure.
	 * @param	_slot			Talon SRX slot to store results in.
	 * @param	_direction		-1 for backwards and +1 for forwards.
	 */
	
	static final int	SENSOR_ROTATION	= 4096;			// Ticks per single rotation of the sensor.
	static final int	ROTATION_TIME		= 100;			// 100ms.
	
	void configure(CANTalon _talon, int _slot, int _direction)
	{
		double	F = 0.0f;				// Feed Forward Gain.
		double	P = 0.0f;
		double	I = 0.0f;
		double	D = 0.0f;
		
		double	MMCV	= 0.0f;			// Motion Magic Cruise Velocity.
		double	MMA		= 0.0f;			// Motion Magic Acceleration.
		
		StringBuilder sb = new StringBuilder();

		_talon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		_talon.reverseSensor(true);

		// Percent voltage mode.

		_talon.changeControlMode(TalonControlMode.PercentVbus);

		// Set full power, either forwards or backwards.
		
		_talon.set((double) _direction);

		wait(500);
		
		// Run the motor for 10 seconds and grab the average RPM.

		double	rpm		 = 0.0f;
		int		rpmcount = 0;

		double	start = Timer.getFPGATimestamp() + 5;

		while(Timer.getFPGATimestamp() < start)
		{
			rpm = rpm + _talon.getSpeed();

			++rpmcount;
		}
		
		// Check the sensor is in the correct direction.
		
		if(Math.signum(rpm) != Math.signum((double) _direction))
		{
			System.out.println("Warning: sensor direction not correct.");
		}

		// Calculate the simple average of all the RPM values (TODO Do we need the +ve value only?).
		
		rpm = Math.abs(rpm / rpmcount);
		
		System.out.println("RPM:" + rpm);

		_talon.set(0.0f);

		wait(500);

	/*
		sb.append("\tout:");
    	System.out.println(sb.toString());
		sb.setLength(0);
	*/
		
	/*	
		_talon.setProfile(_slot);
		
		_talon.setF(0);
		_talon.setP(0);
		_talon.setI(0);
		_talon.setD(0);
		_talon.setMotionMagicCruiseVelocity(453);
		_talon.setMotionMagicAcceleration(453);
	*/
		
	/*

-> test()
Warning: sensor direction not correct.
Left +ve
RPM:190.48635537701435
Warning: sensor direction not correct.
Left -ve
RPM:190.2341896437771
Warning: sensor direction not correct.
Right +ve
RPM:187.91177266278683
Warning: sensor direction not correct.
Right -ve
RPM:196.22088342636326
<- test()

	*/

	}

	/*
	 * Wait for a number of ms.
	 */
	
	void wait(int _for)
	{
		try
		{
			Thread.sleep(_for);
		}
		
		catch(InterruptedException e)
		{
			e.printStackTrace();
		}

	}
}