
package org.usfirst.frc.team5087.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import java.util.Stack;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/*
 * 
 */

public class Robot extends SampleRobot
{
    // Commands to send to the camera control thread.
    
    static final int FRONT_CAMERA	= 0;
    static final int REAR_CAMERA	= 1;
    static final int FRONT_IMAGE	= 2;

    // Climb state machine.
    
    static final int CLIMB_START	= 0;
    static final int CLIMB_ATTACH	= 1;
    static final int CLIMB_CLIMBING	= 2;
    static final int CLIMB_STOPPED	= 3;

    // List of installed hardware on the robot.
    
    final	boolean	installedDrive_			= true;
    final	boolean	installedGyro_			= false;
    final	boolean	installedSpokeSensor_	= true;
    final	boolean	installedJoystick_		= true;
    final	boolean	installedFrontCamera_	= true;
    final	boolean	installedRearCamera_	= true;
    final	boolean	installedClimb_			= true;
    final	boolean	installedGearDrop_		= true;
    
    static final int	DRIVE_LEFT_SLAVE		= 0;
    static final int	DRIVE_LEFT_MASTER		= 1;
    static final int	DRIVE_RIGHT_SLAVE		= 2;
    static final int	DRIVE_RIGHT_MASTER	= 3;
    
    double angleSetpoint = 0.0;
    
    final double pGain = .006; //proportional turning constant

    //gyro calibration constant, may need to be adjusted;
    //gyro value of 360 is set to correspond to one full revolution
    
    final double voltsPerDegreePerSecond = .0128;
    
    Vision				vision_;
    
    SpokeSensor			spokesensor_;
    
	Thread 				visionThread_;

    Stack<Integer>		cameraControl_;
    
    int					cameraInUse_;
    
    UsbCamera			usbFrontCamera_;
    UsbCamera			usbRearCamera_;
    
    CvSink				cvFrontSink_;
    CvSink				cvRearSink_;
    
    CvSource			outputStream_;
    
    double				speedLimit_;

	private	CANTalon[]	talons_ = new CANTalon[6];

    CANTalon			climbLeft_;
    CANTalon			climbRight_;

    int					climbState_;

    double				climbRate_;
    double				climbSpeed_;
    double				climbMaxSpeed_;
    
    Movement			movement_;

    RamsRobotDrive		drive_;
    AnalogGyro			gyro_;
    Joystick			joystick_;
    Solenoid			drop_;

	boolean 			dropGearButton_;
	boolean			cameraSwitchButton_;
	
    /*
     * Main robot constructor - this is called before RobotInit().
     */
    
    @SuppressWarnings("unused")
	public Robot()
    {
    	System.out.println("-> Robot()");
    	
    	cameraControl_ = new Stack<Integer>();
    	
    	cameraSwitchButton_ = false;
    	
    	// Motor controllers for the robot movement.

    	if(installedDrive_ == true)
    	{
    		// Talon #4 has the left gear-box sensor.
    		
        	talons_[DRIVE_LEFT_MASTER]	= new CANTalon(4);
        	talons_[DRIVE_LEFT_SLAVE]		= new CANTalon(8);
        	
        	talons_[DRIVE_LEFT_SLAVE].changeControlMode(TalonControlMode.Follower);
        	talons_[DRIVE_LEFT_SLAVE].set(talons_[DRIVE_LEFT_MASTER].getDeviceID());
        	
        	// Talon #2 has the right gear-box sensor.
        	
        	talons_[DRIVE_RIGHT_MASTER]	= new CANTalon(2);
        	talons_[DRIVE_RIGHT_SLAVE]	= new CANTalon(1);

        	talons_[DRIVE_RIGHT_SLAVE].changeControlMode(TalonControlMode.Follower);
        	talons_[DRIVE_RIGHT_SLAVE].set(talons_[DRIVE_RIGHT_MASTER].getDeviceID());

        	// Not sure if these need setting, but lets do it anyway.

        	for(int i = 0; i < 4; ++i)
        	{
            	talons_[i].configMaxOutputVoltage(12.0);
            	talons_[i].configNominalOutputVoltage(12.0, 12.0);
            	talons_[i].configPeakOutputVoltage(12.0, 12.0);
        	}

        	movement_ = new Movement(talons_[DRIVE_LEFT_MASTER],
        							 talons_[DRIVE_RIGHT_MASTER]);
        
        	drive_ = new RamsRobotDrive(talons_[DRIVE_LEFT_MASTER],
        								talons_[DRIVE_RIGHT_MASTER]);
        	
            drive_.setExpiration(0.1f);
            
            speedLimit_ = 0.90;			// Max of 50% speed for movement.
    	}
    	
    	if(installedClimb_ == true)
    	{
        	climbLeft_  = new CANTalon(16);
        	climbRight_ = new CANTalon(17);

        	climbLeft_.configMaxOutputVoltage(12.0);
        	climbLeft_.configNominalOutputVoltage(12.0, 12.0);
        	climbLeft_.configPeakOutputVoltage(12.0,  12.0);

        	climbRight_.configMaxOutputVoltage(12.0);
        	climbRight_.configNominalOutputVoltage(12.0, 12.0);
        	climbRight_.configPeakOutputVoltage(12.0,  12.0);

        	climbSpeed_		= 0.0;
    		climbRate_		= 0.0;
    		climbMaxSpeed_	= 0.0;
    		
        	climbState_ = CLIMB_START;
    	}
    	
    	if(installedSpokeSensor_ == true)
    	{
    		spokesensor_ = new SpokeSensor();
    	}

        // Variables for the gyro.

    	if(installedGyro_ == true)
    	{
            gyro_ = new AnalogGyro(0);
            
            gyro_.initGyro();
            gyro_.reset();
    	}
    	
        // Allocate a new joystick for the robot control.

    	if(installedJoystick_ == true)
    	{
    		joystick_ = new Joystick(0);
    	}

        // Variables required to handle dropping the gear.

        if(installedGearDrop_ == true)
        {
            drop_ = new Solenoid(0);
            
            drop_.set(false);
            
            dropGearButton_ = false;
        }
        
    	System.out.println("<- Robot()");
    }
    
    // This code is run when the robot is first started up. It's called after the constructor,
    // but at this point we are guaranteed that the WPIlib is ready for use. 

    /*
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.SampleRobot#robotInit()
     */
    
    @SuppressWarnings("unused")
	public void robotInit()
    {
    	System.out.println("-> robotInit()");
        
        visionThread_ = new Thread(() ->
        {
        	System.out.println("-> Thread()");

        	// Create the Mat data required by the capture code.
        	
			Mat	original = new Mat();
			
			Mat copyimage = new Mat();

			// Points for the cross-hair lines.
			
    		Point crossH0 = new Point(160 - 8, 120);
    		Point crossH1 = new Point(160 + 8, 120);

    		Point crossV0 = new Point(160, 120 - 8);
    		Point crossV1 = new Point(160, 120 + 8);

    		double spokerotation;
    		
        	cameraInUse_ = FRONT_CAMERA;

        	// Front camera is used for dropping the gear off and will be used by the OpenCV code.
    
        	if(installedFrontCamera_ == true)
        	{
            	usbFrontCamera_ = CameraServer.getInstance().startAutomaticCapture(0);

                usbFrontCamera_.setResolution(320, 240);
                usbFrontCamera_.setFPS(15);

                cvFrontSink_ = CameraServer.getInstance().getVideo(usbFrontCamera_);
            	
        		cvFrontSink_.setEnabled(true);
            	cvFrontSink_.setSource(usbFrontCamera_);
            	
                /*
                
                // Grab the JSON file and save into a table for this.

                VideoProperty camera0[] = usbFrontCamera_.enumerateProperties();

                for(int i = camera0.length(); ++i)
                {
                	
                }
                */

            	vision_ = new Vision(this);
        	}
            
            // The rear camera is only used when we are looking for the rope.

        	if(installedRearCamera_ == true)
        	{
            	usbRearCamera_ = CameraServer.getInstance().startAutomaticCapture(1);

                usbRearCamera_.setResolution(320, 240);
                usbRearCamera_.setFPS(15);

            	cvRearSink_ = CameraServer.getInstance().getVideo(usbRearCamera_);

        		cvRearSink_.setEnabled(false);
            	cvRearSink_.setSource(usbRearCamera_);
        	}
        	
        	// If either camera is installed, create the output stream.
        	
    		if((installedFrontCamera_ == true) || (installedRearCamera_ == true))
    		{
    			outputStream_ = CameraServer.getInstance().putVideo("Robot Camera", 320, 240);
    		}
    		    		
        	System.out.println("Running ...");
        	
            while(!Thread.interrupted())
            {
            	if(installedSpokeSensor_ == true)
            	{
            		spokerotation = spokesensor_.position();
            	}
            	
            	// Display the correct video.

        		original.release();	// TODO should we do this?

            	switch(cameraInUse_)
            	{
            		case FRONT_CAMERA :
            		{
            			if(installedFrontCamera_ == true)
            			{
                    		cvFrontSink_.grabFrameNoTimeout(original);
            			}
            			
                		break;
            		}
            		
            		case REAR_CAMERA :
            		{
            			if(installedRearCamera_ == true)
            			{
                    		cvRearSink_.grabFrameNoTimeout(original);
            			}

                		break;
            		}
            		
            		default :
            		{
            			System.out.println(cameraInUse_ + " - invalid camera id.");
            			
            			break;
            		}
            	}

            	// Switch cameras if requested by the main application.
            	
            	if(cameraControl_.empty() == false)
            	{
            		int control = (int) cameraControl_.pop();
            		
            		switch(control)
            		{
        				case FRONT_CAMERA :
        				{
        					if(installedFrontCamera_ == true)
        					{
        						cameraInUse_ = FRONT_CAMERA;

        						cvFrontSink_.setEnabled(true);
        						cvRearSink_.setEnabled(false);
        					}

                			break;
        				}
        			
            			case REAR_CAMERA :
            			{
            				if(installedRearCamera_ == true)
            				{
            					cameraInUse_ = REAR_CAMERA;

                        		cvFrontSink_.setEnabled(false);
                        		cvRearSink_.setEnabled(true);
            				}

                    		break;
            			}
            			
            			case FRONT_IMAGE :
            			{
            				if(installedFrontCamera_ == true)
            				{
            					original.copyTo(copyimage);
            					
            					vision_.give(copyimage);
            				}
            				
            				break;
            			}
            			
            			default :
            			{
                			System.out.println(control + " - invalid control code.");
                			
            				break;
            			}
            		}
            	}

        		if((installedFrontCamera_ == true) || (installedRearCamera_ == true))
        		{
            		vision_.show(original);
            		
                	if(installedSpokeSensor_ == true)
                	{
                		spokesensor_.show(original, spokerotation);
                	}
            	
            		// Display the cross-hair in the centre of the screen.
            		
            		Imgproc.line(original, crossH0, crossH1, Colours.MAGENTA);
            		Imgproc.line(original, crossV0, crossV1, Colours.MAGENTA);

                    outputStream_.putFrame(original);
        		}
        		else
        		{
            		try
            		{
						Thread.sleep(50);
					}
            		
            		catch (InterruptedException e)
            		{
						e.printStackTrace();
					}
        		}
            }

            System.out.println("Stopped.");
            
        	System.out.println("<- Thread()");

        });
        
		visionThread_.setDaemon(true);
		visionThread_.start();
		
    	System.out.println("<- robotInit()");
    }

    /*
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.SampleRobot#disabled()
     */
    
    public void disabled()
    {
    	System.out.println("-> disabled()");
    
    	System.out.println("<- disabled()");
    }

    /*
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.SampleRobot#autonomous()
     */
    
    public void autonomous()
    {
    	System.out.println("-> autonomous()");
        
        while(isEnabled())
        {
        }
        
    	System.out.println("<- autonomous()");
    }

    /*
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.SampleRobot#operatorControl()
     */
    
    @SuppressWarnings("unused")
	public void operatorControl()
    {
    	System.out.println("-> operatorControl()");
    	
    	if(installedDrive_ == true)
    	{
    		drive_.setSafetyEnabled(true);
    	}
        
        while(isOperatorControl() && isEnabled())
        {
        	moveRobot();
        	
        	dropGear();
        	
        	switchCamera();
        	
        	climbRope();
        	
        	if(installedSpokeSensor_ == true)
        	{
        		System.out.println("S:" + spokesensor_.position());	// TODO fix this!
        	}
        	
            Timer.delay(0.005);
        }
        
    	System.out.println("<- operatorControl()");
    }

    /*
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.SampleRobot#test()
     */
    
    public void test()
    {
    	System.out.println("-> test()");
    	
    	System.out.println("<- test()");
    }

    /*
     * Send a request to the camera thread.
     */
    
    public void request(int _request)
    {
		cameraControl_.push((Integer) _request);
    }
    
    /*
     * Handle the movement of the robot and adjust depending on the camera being used
     * at the time. 
     */
    
    @SuppressWarnings("unused")
	private void moveRobot()
    {
//      double turningValue;
        
//      gyro.setSensitivity(voltsPerDegreePerSecond); //calibrates gyro values to equal degrees

//    	turningValue =  (angleSetpoint - gyro.getAngle()) * pGain;

    	if(installedDrive_ == true)
    	{
        	switch(cameraInUse_)
        	{
        		case FRONT_CAMERA :
        		{
                	drive_.arcadeDrive(-joystick_.getY() * speedLimit_,
                					   -joystick_.getX() * speedLimit_, true);
                	
        			break;
        		}
        		
        		case REAR_CAMERA :
        		{
                	drive_.arcadeDrive(+joystick_.getY() * speedLimit_,
    								   -joystick_.getX() * speedLimit_, true);
                	
        			break;
        		}
        		
        		default :
        		{
        			break;
        		}
        	}
    	}
    }
    
	/*
	 * If the <A> button is pressed and held, open the gear/cog holder, otherwise
	 * close the gear/cog holder.
	 */
    
    @SuppressWarnings("unused")
	private void dropGear()
    {
    	if(installedJoystick_ == true)
    	{
            if(joystick_.getRawButton(Controller.BUTTON_A) == true)
            {
            	dropGearButton_ = true;

            	if(installedGearDrop_ == true)
            	{
            		drop_.set(true);
            	}
            }
            else
            {
        		if(dropGearButton_ == true)
            	{
            		dropGearButton_ = false;
            		
                	if(installedGearDrop_ == true)
                	{
                		drop_.set(false);
                	}
            	}
            }
    	}
    }

    /*
     * The <START> button on the joystick switches the camera we are viewing which also
     * switches the controls on the robot.
     */
    
    @SuppressWarnings("unused")
	private void switchCamera()
    {
    	if(installedJoystick_ == true)
    	{
            if(joystick_.getRawButton(Controller.BUTTON_START) == true)
            {
            	if(cameraSwitchButton_ == false)
            	{
            		if(cameraInUse_ == FRONT_CAMERA)
            		{
            			if(installedRearCamera_ == true)
            			{
            				request(REAR_CAMERA);
            			}
            		}
            		else
            		{
            			if(installedFrontCamera_ == true)
            			{
            				request(FRONT_CAMERA);
            			}
            		}
            		
            		cameraSwitchButton_ = true;
            	}
            }
            else
            {
            	if(cameraSwitchButton_ == true)
            	{
            		cameraSwitchButton_ = false;
            	}
            }
    	}
    }

    /*
     * Handle the climbing of the robot, only if we are viewing via the rear camera.
     */

    private int counter_ = 0;
    
    @SuppressWarnings("unused")
	private void climbRope()
    {
    	if((installedClimb_ == true)
    	&& (cameraInUse_ == REAR_CAMERA))
    	{
    		// Debug code to track any issues.
    		
    		if((++counter_ % 10) == 0)
    		{
    			System.out.format("%1.4f[%1.4f]:%2.4f/%2.4f\n",
    				climbSpeed_,
    				climbRate_,
    				climbLeft_.getOutputCurrent(),
    				climbRight_.getOutputCurrent());
    		}
    		
    		// Set the motor climb speed.
    		
			climbLeft_.set(-climbSpeed_);
			climbRight_.set(-climbSpeed_);

			// Adjust the climbing speed.
			
    		climbSpeed_ += climbRate_;
			
			if(climbSpeed_ > climbMaxSpeed_)
			{
				climbSpeed_ = climbMaxSpeed_;
				
				climbRate_  = 0.0;
			}

			// Grab the max current draw.
			
    		double current = Math.max(climbLeft_.getOutputCurrent(),
					   				   climbRight_.getOutputCurrent());

    		// Run the state machine for the climb.
    		
    		switch(climbState_)
    		{
    			case CLIMB_START :
    			{
    				climbSpeed_	   = 0.1;
    				
    				climbRate_ 	   = 0.005;
    				climbMaxSpeed_ = 0.25;
    				
    				climbState_	   = CLIMB_ATTACH;
    				
    				break;
    			}
    			
    			case CLIMB_ATTACH :
    			{
    				if(current >= 3.0)
    				{
    					climbRate_	   = 0.001;
    					climbMaxSpeed_ = 0.6;
    					
    					climbState_	   = CLIMB_CLIMBING;
    				}
    				
        			break;
    			}
    			
    			case CLIMB_CLIMBING :
    			{
            		if(current >= 15.0)
            		{
            			climbState_ = CLIMB_STOPPED;
            			
            			climbSpeed_    = 0.0;
            			climbRate_	   = 0.0;
            			climbMaxSpeed_ = 0.0;
            		}
            		
        			break;
    			}
    			
    			case CLIMB_STOPPED :
    			{
    				// Allow small movements to ensure the pad light is lit.
    				
        			break;
    			}
    		}
    	}
    }
}