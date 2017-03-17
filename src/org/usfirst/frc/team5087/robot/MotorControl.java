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
	static final boolean	SHOW_CONFIG	= false;
	static	final boolean	SHOW_THEREYET	= false;
	static	final boolean	SHOW_DRIVING	= false;

	static final double	MAGIC_TIMEOUT	= 5.0f;
	static final double	MAGIC_CURRENT	= 70.0f;
	
    static	final int	LEFT_MASTER	= 0;
    static	final int	RIGHT_MASTER	= 1;
    static	final int	LEFT_SLAVE		= 2;
    static	final int	RIGHT_SLAVE	= 3;

    double[]	positions_	= new double[2];
    double[]	timeouts_	= new double[2];
   
	CANTalon[]	talons_		= new CANTalon[4];

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

        	setup(false);
    	}
	}

	/**
	 * Setup all the configuration for the talons.
	 */
	
	void setup(boolean _auto)
	{
		// Configure the left hand side.

    	talons_[LEFT_MASTER].reverseOutput(false);
    	talons_[LEFT_MASTER].reverseSensor(true);	

    	// Configure the right hand side.

    	talons_[RIGHT_MASTER].reverseOutput(true);
    	talons_[RIGHT_MASTER].reverseSensor(false);	

    	if(_auto == false)
    	{
            talons_[LEFT_SLAVE].changeControlMode(TalonControlMode.Follower);
            talons_[LEFT_SLAVE].set(talons_[LEFT_MASTER].getDeviceID());

            talons_[RIGHT_SLAVE].changeControlMode(TalonControlMode.Follower);
            talons_[RIGHT_SLAVE].set(talons_[RIGHT_MASTER].getDeviceID());
    	}
    	
    	// Setup the basic information for all of the motor controllers.

    	for(int i = 0; i < 4; ++i)
    	{
    		if(i < 2)
    		{
    			positions_[i] = 0.0f;							// Saved positions to zero.
    			
    			talons_[i].setPosition(0.0f);					// Reset encoder to zero.
    	    	talons_[i].setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

//    	    	talons_[i].changeControlMode(TalonControlMode.PercentVbus);
//            	talons_[i].set(0.0f);
    		}
    		
//    		talons_[i].enableBrakeMode(true);					// When not moving, brake.

        	talons_[i].configMaxOutputVoltage(12.0f);
        	talons_[i].configNominalOutputVoltage(0.0f, 0.0f);
        	talons_[i].configPeakOutputVoltage(+12.0f, -12.0f);
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
	 * Start moving forwards/backwards by the specified distance.
	 * 
	 * @param	_mm
	 * @param	_rpm
	 */
	
	void move(double _mm, double _rpm)
	{
		configure(LEFT_MASTER,  0, _rpm);
		configure(RIGHT_MASTER, 0, _rpm);

		double	distance = _mm / Dimensions.wheelCircumferenceMM;

		MotionMagic(LEFT_MASTER,  distance);
		MotionMagic(RIGHT_MASTER, distance);
		
//		System.out.println("Moving by " + _mm + "mm (" + distance + " wheel rotations).");
	}

	/**
	 * Adjust the rotation by the required amount.
	 * 
	 * @param _angle
	 */
	
	void rotate(double _angle, double _rpm)
	{
		configure(LEFT_MASTER,  0, _rpm);
		configure(RIGHT_MASTER, 0, _rpm);
		
		double	curcumference = Dimensions.wheelGapMM * Math.PI;
		
		double angle = Math.abs(_angle) / 360.0f;
		
		double	mm = angle * curcumference;
		
		double	distance = mm / Dimensions.wheelCircumferenceMM;

		if(_angle < 0.0f)
		{
			MotionMagic(LEFT_MASTER,  -distance);
			MotionMagic(RIGHT_MASTER, +distance);
		}
		else
		{
			MotionMagic(LEFT_MASTER,  +distance);
			MotionMagic(RIGHT_MASTER, -distance);
		}

//		System.out.println("Moving by " + mm + "mm (" + distance + " wheel rotations).");
	}

	/**
	 * Report if the Motion Magic has finished or not. 
	 * 
	 * @param	_talon	Talon index to check.
	 * 
	 * @return	false=not there yet, true=we are there.
	 */
	
	@SuppressWarnings("unused")
	boolean areWeThereYet(int _talon)
	{
		boolean	ret = stopMotion();
		
		if(ret == false)
		{
			CANTalon	talon = talons_[_talon];
			
			if(SHOW_THEREYET == true)
			{
				System.out.printf("%d,%2.6f,%2.6f,%2.6f,%2.6f\n",
						_talon,												// Talon index.
						talon.getPosition(),								// Encoder position.
						talon.getError() / RATE,							// Difference between set pos and current pos.
						talon.getSpeed(),									// Speed in RPM.
	    				talon.getOutputCurrent()
					);
			}

			if(Math.abs(Math.abs(positions_[_talon]) - Math.abs(talon.getPosition())) < 0.003f)
			{
				ret = true;
			}
			
			if(Timer.getFPGATimestamp() > timeouts_[_talon])
			{
				ret = true;
				
				System.out.println("****************");
				System.out.println("** TIMEOUT #" + _talon + " **");
				System.out.println("****************");
			}
		}

		return ret;
	}

	/**
	 * Return the max current draw from the Talons.
	 * @return
	 */
	
	double maxCurrent()
	{
		double left  = left().getOutputCurrent(); 
		double right = right().getOutputCurrent();
		
		return Math.max(left, right);
	}

	/**
	 * See if the current draw is too high.
	 * 
	 * @return
	 */
	
	boolean stopMotion()
	{
		boolean ret = false;
		
		if(maxCurrent() >= MAGIC_CURRENT)
		{
			ret = true;
		}
		
		return ret;
	}
	/**
	 * 
	 */
	
	void allDone()
	{
		left().changeControlMode(TalonControlMode.PercentVbus);
		left().set(0.0f);

		right().changeControlMode(TalonControlMode.PercentVbus);
		right().set(0.0f);
		
		Timer.delay(0.10f);
	}
	
	/**
	 * Arcade drive implements single stick driving. This function lets you directly provide
	 * joystick values from any source.
	 *
	 * @param	_move		The value to use for forwards/backwards
	 * @param	_rotate		The value to use for the rotate right/left
	 * @param	_squared	If set, decreases the sensitivity at low speeds
	 */

	@SuppressWarnings("unused")
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
	    right = -limit(right);	// -ve as we configure the right motor in the correct direction.

	    left().changeControlMode(TalonControlMode.PercentVbus);
	    left().set(left);
	    
	    right().changeControlMode(TalonControlMode.PercentVbus);
	    right().set(right);

	    if(SHOW_DRIVING == true)
		{
	    	System.out.println("L:" + left().getSpeed() + " R:" + right().getSpeed() + " ROT:" + _rotate);
		}
	}

	/**
	 * Calculate and return the RPM value at full speed.
	 * 
	 * @param	_talon		Talon index to configure.
	 * @param	_slot		Talon SRX slot to store results in.
	 * @param	_direction	-1 for backwards and +1 for forwards.
	 * 
	 * @return				Average RPM of the motor output.
	 */
	
	double RPM(int _talon, int _slot, int _direction)
	{
		CANTalon	talon = talons_[_talon];
		
		talon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		talon.reverseSensor(true);

		// Percent voltage mode.

		talon.changeControlMode(TalonControlMode.PercentVbus);
		talon.set((double) _direction);

		wait(250);
		
		// Run the motor for 5 seconds and grab the average RPM.

		double	rpm		 = 0.0f;
		int		rpmcount = 0;

		double	start = Timer.getFPGATimestamp() + 5.0f;

		while(Timer.getFPGATimestamp() < start)
		{
			rpm = rpm + talon.getSpeed();

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
		
		talon.set(0.0f);

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
	
	static final double	RATE		= 4096.0f;				// Ticks per single rotation.
	static final double	NATIVE		= 100.0f;				// 100ms.
	static final double	DIVIDER	= 60.0f * (1000.0f / NATIVE);
	
	void configure(int _talon, int _slot, double _rpm)
	{
		double	F = 1023.0f / ((_rpm * RATE) / DIVIDER);		// See section 12.8.3.
		double	P = 4.0f;
		double	I = 0.0002f;
		double	D = P * 10.0f;

		int		IZ= 50;
		
		double	MMCV	= _rpm * 1.00f;							// See section 12.8.4.
		double	MMA		= _rpm * 1.00f;							// See section 12.8.4.

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
	
	@SuppressWarnings("unused")
	void configureSlot(int _talon, int _slot,
						double _F, double _P, double _I, double _D,
						int IZ, double _MMCV, double _MMA)
	{
		CANTalon talon = talons_[_talon];

		talon.setProfile(_slot);

		talon.setF(_F);
		talon.setP(_P);
		talon.setI(_I);
		talon.setD(_D);
		
		talon.setIZone(IZ);
		
		talon.setMotionMagicCruiseVelocity(_MMCV);
		talon.setMotionMagicAcceleration(_MMA);

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
	 * @param	_talon		Talon index to run the Motion Magic on.
	 * @param	_position	Position to adjust by.
	 */

	void MotionMagic(int _talon, double _position)
	{
		CANTalon	talon = talons_[_talon];

		talon.setPosition(0.0f);									// Reset encoder to zero.

		positions_[_talon] = _position;								// Set the current position.
//		positions_[_talon] = _position + talon.getPosition();		// Adjust the current position.
		
		talon.clearIAccum();
		talon.changeControlMode(TalonControlMode.MotionMagic);		// Set the CAN Talon mode.
		talon.set(positions_[_talon]);								// Number of wheel rotations.
		
		timeouts_[_talon] = Timer.getFPGATimestamp() + MAGIC_TIMEOUT;
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