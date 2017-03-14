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
	 * Calculate and return the RPM value at full speed.
	 * 
	 * @param	_talon			Talon SRX to configure.
	 * @param	_slot			Talon SRX slot to store results in.
	 * @param	_direction		-1 for backwards and +1 for forwards.
	 */
	
	double RPM(CANTalon _talon, int _slot, int _direction)
	{
		_talon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		_talon.reverseSensor(true);

		// Percent voltage mode.

		_talon.changeControlMode(TalonControlMode.PercentVbus);

		// Set full power, either forwards or backwards.
		
		_talon.set((double) _direction);

		wait(250);
		
		// Run the motor for 5 seconds and grab the average RPM.

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

		// Stop the motor running for the moment.
		
		_talon.set(0.0f);

		wait(250);

		return rpm;
	}

	/*
	 * Configure the Talon.
	 * 
	 * @param	_talon			Talon SRX to configure.
	 * @param	_slot			Talon SRX slot to store results in.
	 * @param	_rpm			Max RPM to run at.
	 */
	
	static final int	RATE		= 4096;						// Ticks per single rotation.
	static final int	NATIVE		= 100;						// 100ms.
	static final int	DIVIDER	= 60 * (1000 / NATIVE);
	
	void configure(CANTalon _talon, int _slot, double _rpm)
	{
		double	F = (_rpm * RATE) / DIVIDER;					// Feed Forward Gain.
		double	P = 0.0f;
		double	I = 0.0f;
		double	D = 0.0f;
		
		double	MMCV	= _rpm * 0.90f;							// Motion Magic Cruise Velocity (90%).
		double	MMA		= _rpm * 0.90f;							// Motion Magic Acceleration (90%).
		
		StringBuilder sb = new StringBuilder();

		_talon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		_talon.reverseSensor(true);
		
		save(_talon, _slot, F, P, I, D, MMCV, MMA);

		_talon.setPosition(0.0f);

		_talon.changeControlMode(TalonControlMode.MotionMagic);
		_talon.set(RATE * 4);									// Rotate to position.

	/*	
		// Run the motor for 5 seconds and grab the average RPM.

		double	rpm		 = 0.0f;
		int		rpmcount = 0;

		double	start = Timer.getFPGATimestamp() + 5;

		while(Timer.getFPGATimestamp() < start)
		{
			rpm = rpm + _talon.getSpeed();

			++rpmcount;
		}
	*/
		
		// We're done - turn the motor off.
		
		_talon.changeControlMode(TalonControlMode.PercentVbus);

		_talon.set(0.0f);

		wait(250);

	/*
		sb.append("\tout:");
    	System.out.println(sb.toString());
		sb.setLength(0);
	*/
		
	/*	
		
	*/
	}

	/*
	 * Save the configuration information to the correct slot.
	 * 
	 * @param	_talon
	 * @param	_slot
	 * @param	_F
	 * @param	_P
	 * @param	_I
	 * @param	_D
	 * @param	_MMCV
	 * @param	_MMA
	 */
	
	void save(CANTalon _talon, int _slot,
			  double _F, double _P, double _I, double _D, double _MMCV, double _MMA)
	{
		_talon.setProfile(_slot);

		_talon.setF(_F);
		_talon.setP(_P);
		_talon.setI(_I);
		_talon.setD(_D);
		
		_talon.setMotionMagicCruiseVelocity(_MMCV);
		_talon.setMotionMagicAcceleration(_MMA);
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