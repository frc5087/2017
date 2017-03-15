package org.usfirst.frc.team5087.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Timer;

/**
 * MotorControl class for the CANTalon motors.
 * 
 * @author	James Fisher
 */

public class MotorControl
{
	static final boolean	SHOW_CONFIG	= true;

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

        	talons_[LEFT_MASTER].setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        	talons_[LEFT_MASTER].reverseOutput(false);
        	talons_[LEFT_MASTER].reverseSensor(true);	

        	// Setup the right hand side where #2 has the right gear-box sensor.
        	
        	talons_[RIGHT_MASTER]	= new CANTalon(2);
        	talons_[RIGHT_SLAVE]	= new CANTalon(1);

        	talons_[RIGHT_SLAVE].changeControlMode(TalonControlMode.Follower);
        	talons_[RIGHT_SLAVE].set(talons_[RIGHT_MASTER].getDeviceID());

        	talons_[RIGHT_MASTER].setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        	talons_[RIGHT_MASTER].reverseOutput(true);
        	talons_[RIGHT_MASTER].reverseSensor(false);	

        	// Setup the basic information for all of the motor controllers.

        	for(int i = 0; i < 4; ++i)
        	{
        		talons_[i].setPosition(0.0f);						// Reset encoder to zero.
            	talons_[i].enableBrakeMode(true);					// When not moving, brake.
            	
            	talons_[i].configMaxOutputVoltage(12.0f);
            	talons_[i].configNominalOutputVoltage(0.0f, 0.0f);
            	talons_[i].configPeakOutputVoltage(+12.0f, -12.0f);
        	}
    	}
	}
	
	/**
	 * Return the Talon that controls the left hand side motors.
	 * 
	 * @return	CANTalon object for left motor output.
	 */
	
	CANTalon left()
	{
		return talons_[LEFT_MASTER];
	}

	/**
	 * Return the Talon that controls the right hand side motors.
	 * 
	 * @return	CANTalon object for right motor output.
	 */
	
	CANTalon right()
	{
		return talons_[RIGHT_MASTER];
	}
	
	/**
	 * Arcade drive implements single stick driving. This function lets you directly provide
	 * joystick values from any source.
	 *
	 * @param	_move		The value to use for forwards/backwards
	 * @param	_rotate		The value to use for the rotate right/left
	 * @param	_squared	If set, decreases the sensitivity at low speeds
	 */

	public void arcadeDrive(double _move, double _rotate, boolean _squared)
	{
	    double left;
	    double right;

	    _move = limit(_move);
	    _rotate = limit(_rotate);

	    if(_squared == true)
	    {
	    	if(_move >= 0.0)
	    	{
	    		_move = _move * _move;
	    	}
	    	else
	    	{
	    		_move = -(_move * _move);
	    	}
	    	
	    	if(_rotate >= 0.0)
	    	{
	    		_rotate = _rotate * _rotate;
	    	}
	    	else
	    	{
	    		_rotate = -(_rotate * _rotate);
	    	}
	    }

	    if(_move > 0.0)
	    {
	    	if(_rotate > 0.0)
	    	{
	    		left = _move - _rotate;
	    		right = Math.max(_move, _rotate);
	    	}
	    	else
	    	{
	    		left = Math.max(_move, -_rotate);
	    		right = _move + _rotate;
	    	}
	    }
	    else
	    {
	    	if(_rotate > 0.0)
	    	{
	    		left = -Math.max(-_move, _rotate);
	    		right = _move + _rotate;
	    	}
	    	else
	    	{
	    		left = _move - _rotate;
	    		right = -Math.max(-_move, -_rotate);
	    	}
	    }
	    
	    left = limit(left);
	    right = limit(right);

	    left().changeControlMode(TalonControlMode.PercentVbus);
	    left().set(left);
	    
	    right().changeControlMode(TalonControlMode.PercentVbus);
	    right().set(right);
	}

	/**
	 * Calculate and return the RPM value at full speed.
	 * 
	 * @param	_talon		Talon SRX to configure.
	 * @param	_slot		Talon SRX slot to store results in.
	 * @param	_direction	-1 for backwards and +1 for forwards.
	 * 
	 * @return				Average RPM of the motor output.
	 */
	
	double RPM(CANTalon _talon, int _slot, int _direction)
	{
		_talon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		_talon.reverseSensor(true);

		// Percent voltage mode.

		_talon.changeControlMode(TalonControlMode.PercentVbus);
		_talon.set((double) _direction);

		wait(250);
		
		// Run the motor for 5 seconds and grab the average RPM.

		double	rpm		 = 0.0f;
		int		rpmcount = 0;

		double	start = Timer.getFPGATimestamp() + 5.0f;

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

		// Calculate the simple average of all the RPM values.
		
		rpm = Math.abs(rpm / rpmcount);
		
		System.out.println("RPM:" + rpm);

		// Stop the motor running for the moment.
		
		_talon.set(0.0f);

		wait(250);

		return rpm;
	}

	/**
	 * Configure the Talon.
	 * 
	 * @param	_talon	Talon SRX to configure.
	 * @param	_slot	Talon SRX slot to store results in.
	 * @param	_rpm	Max RPM to run at.
	 */
	
	static final int	RATE		= 4096;						// Ticks per single rotation.
	static final int	NATIVE		= 100;						// 100ms.
	static final int	DIVIDER	= 60 * (1000 / NATIVE);
	
	void configure(CANTalon _talon, int _slot, double _rpm)
	{
		double	F = 1023.0f / ((_rpm * RATE) / DIVIDER);		// See section 12.8.3.
		double	P = 6.0f;
		double	I = 0.0001f;
		double	D = P * 10.0f;

		int		IZ= 50;
		
		double	MMCV	= _rpm * 1.0f;							// See section 12.8.4.
		double	MMA		= _rpm * 1.0f;							// See section 12.8.4.

		configureSlot(_talon, _slot, F, P, I, D, IZ, MMCV, MMA);
		
//		System.out.println("ERR:" + runMotionMagic(_talon, +2.0f));
	}

	/**
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
	
	void configureSlot(CANTalon _talon, int _slot,
						double _F, double _P, double _I, double _D,
						int IZ, double _MMCV, double _MMA)
	{
		_talon.setProfile(_slot);

		_talon.setF(_F);
		_talon.setP(_P);
		_talon.setI(_I);
		_talon.setD(_D);
		
		_talon.setIZone(IZ);
		
		_talon.setMotionMagicCruiseVelocity(_MMCV);
		_talon.setMotionMagicAcceleration(_MMA);

		if(SHOW_CONFIG == true)
		{
			System.out.println("F:" + _F +
							   " P:" + _P + " I:" + _I + " D:" + _D +
							   " IZ:" + IZ +
							   " MMCV:" + _MMCV + " MMA:" + _MMA);
		}
	}

	/**
	 * Run the Motion Magic profile for a few seconds, or if the position is good, earlier.
	 * 
	 * @param	_talon		Talon to run the Motion Magic on.
	 * @param	_position	Position to adjust by.
	 * 
	 * @return				Value of getClosedLoopError().
	 */

	int runMotionMagic(CANTalon _talon, double _position)
	{
		_position += _talon.getPosition();							// Adjust the current position.
		
		_talon.changeControlMode(TalonControlMode.MotionMagic);
		_talon.set(_position);										// Number of wheel rotations.
		
		int		count = 0;
		
		double	start = Timer.getFPGATimestamp() + 4.0f;			// Run for about 4 seconds.

		while(Timer.getFPGATimestamp() < start)
		{
			System.out.printf("%d,%f,%f,%f\n", ++count,
				_talon.getPosition(),								// Encoder position.
				_talon.getError() / RATE,							// Difference between set pos and current pos.
				_talon.getSpeed()									// Speed in RPM.
			);

			if(Math.abs(Math.abs(_position) - Math.abs(_talon.getPosition())) < 0.002f)
			{
				break;
			}
		}

		return _talon.getClosedLoopError();
	}

	/**
	 * Limit the specified number to -1...+1
	 * 
	 * @param	_number	Value to limit.
	 * 
	 * @return	Limited number.
	 */
	
	double limit(double _number)
	{
		if(_number < -1.0f)
		{
			_number = -1.0f;
		}
		else
		{
			if(_number > +1.0f)
			{
				_number = +1.0f;
			}
		}
		
		return _number;
	}
	
	/**
	 * Wait for a number of ms.
	 * 
	 * @param	_for	Number of ms to wait for.
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