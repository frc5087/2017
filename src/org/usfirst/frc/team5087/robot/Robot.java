
package org.usfirst.frc.team5087.robot;

import com.ctre.CANTalon;

import java.util.Stack;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    
    double				speedLimit_ = 1.0f;

    CANTalon			climbLeft_;
    CANTalon			climbRight_;

    int					climbState_;

    double				climbRate_;
    double				climbSpeed_;
    double				climbMaxSpeed_;
    
    Movement			movement_;

    MotorControl		motor_;
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

    	if(InstalledHardware.DRIVE == true)
    	{
    		motor_ = new MotorControl();

        	movement_ = new Movement(motor_.left(), motor_.right());
        
            speedLimit_ = 1.0;
    	}
    	
    	if(InstalledHardware.CLIMB == true)
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
    	
    	if(InstalledHardware.SPOKE_SENSOR == true)
    	{
    		spokesensor_ = new SpokeSensor();
    	}

        // Allocate a new joystick for the robot control.

    	if(InstalledHardware.JOYSTICK == true)
    	{
    		joystick_ = new Joystick(0);
    	}

        // Variables required to handle dropping the gear.

        if(InstalledHardware.GEAR_DROP == true)
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
    
        	if(InstalledHardware.FRONT_CAMERA == true)
        	{
            	usbFrontCamera_ = CameraServer.getInstance().startAutomaticCapture(0);

                usbFrontCamera_.setResolution(320, 240);
                usbFrontCamera_.setFPS(15);

                cvFrontSink_ = CameraServer.getInstance().getVideo(usbFrontCamera_);
            	
        		cvFrontSink_.setEnabled(true);
            	cvFrontSink_.setSource(usbFrontCamera_);
            	
            	vision_ = new Vision(this);
        	}
            
            // The rear camera is only used when we are looking for the rope.

        	if(InstalledHardware.REAR_CAMERA == true)
        	{
            	usbRearCamera_ = CameraServer.getInstance().startAutomaticCapture(1);

                usbRearCamera_.setResolution(320, 240);
                usbRearCamera_.setFPS(15);

            	cvRearSink_ = CameraServer.getInstance().getVideo(usbRearCamera_);

        		cvRearSink_.setEnabled(false);
            	cvRearSink_.setSource(usbRearCamera_);
        	}
        	
        	// If either camera is installed, create the output stream.
        	
    		if((InstalledHardware.FRONT_CAMERA == true)
    		|| (InstalledHardware.REAR_CAMERA == true))
    		{
    			outputStream_ = CameraServer.getInstance().putVideo("Robot Camera", 320, 240);
    		}
    		    		
        	System.out.println("Running ...");
        	
            while(!Thread.interrupted())
            {
            	if(InstalledHardware.SPOKE_SENSOR == true)
            	{
            		spokerotation = spokesensor_.position();
            	}
            	
            	// Display the correct video.

        		original.release();

            	switch(cameraInUse_)
            	{
            		case FRONT_CAMERA :
            		{
            			if(InstalledHardware.FRONT_CAMERA == true)
            			{
                    		cvFrontSink_.grabFrameNoTimeout(original);
            			}
            			
                		break;
            		}
            		
            		case REAR_CAMERA :
            		{
            			if(InstalledHardware.REAR_CAMERA == true)
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
        					if(InstalledHardware.FRONT_CAMERA == true)
        					{
        						cameraInUse_ = FRONT_CAMERA;

        						cvFrontSink_.setEnabled(true);
        						cvRearSink_.setEnabled(false);
        					}

                			break;
        				}
        			
            			case REAR_CAMERA :
            			{
            				if(InstalledHardware.REAR_CAMERA == true)
            				{
            					cameraInUse_ = REAR_CAMERA;

                        		cvFrontSink_.setEnabled(false);
                        		cvRearSink_.setEnabled(true);
            				}

                    		break;
            			}
            			
            			case FRONT_IMAGE :
            			{
            				if(InstalledHardware.FRONT_CAMERA == true)
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

        		if((InstalledHardware.FRONT_CAMERA == true)
        		|| (InstalledHardware.FRONT_CAMERA == true))
        		{
            		vision_.show(original);
            		
                	if(InstalledHardware.SPOKE_SENSOR == true)
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

    /**
     * See if a requested move has completed, or we've finished auto mode.
     * 
     * @return	false=interrupted, true=magic finished.
     */
    
	boolean areWeThereYet()
    {
    	boolean	ret = false;
    	
    	boolean	one = motor_.areWeThereYet(motor_.LEFT_MASTER);
    	boolean	two = motor_.areWeThereYet(motor_.RIGHT_MASTER);
    	
   		if((one = true) && (two == true))
   		{
   			motor_.allDone();
   			
    		ret = true;
    	}
    
    	return ret;
    }

    /**
     * Wait until we have completed the move, or auto is over.
     * 
     * @return
     */
    
    public boolean done()
    {
		while(isEnabled() && ((areWeThereYet() == false)))
    	{
            Timer.delay(0.005f);
    	}

		return isEnabled();
    }
    
    static double RPM = 400.0f;
    
    /*
     * (non-Javadoc)
     * @see edu.wpi.first.wpilibj.SampleRobot#autonomous()
     */
    
    public void autonomous()
    {
    	System.out.println("-> autonomous()");

    	if(InstalledHardware.DRIVE == true)
    	{
        	motor_.setup(false);
    	}
    	
    	String	command = SmartDashboard.getString("DB/String 0", "?").toLowerCase();
    	
    	System.out.println(command);
    	
    	double adj = +1.0f;

    	if(command.charAt(0) == 'b')
    	{
    		adj = -1.0f;
    	}
    	
    	switch(command.charAt(1))
    	{
    		case '1' :
    		{
    			motor_.move(2614.0f - 393.0f, RPM);
    			
    			if(done() == false) break;
    			
    			motor_.rotate(+59.0f * adj, RPM);

    			if(done() == false) break;

    			motor_.move(2203.0f - 393.0f, RPM);

    			if(done() == false) break;

        		drop_.set(true);

    			motor_.move(-500.0f, RPM / 2.0f);

    			if(done() == false) break;

        		drop_.set(false);

    			motor_.move(-(2203.0f - 500.0f - 393.0f), RPM);

    			if(done() == false) break;

    			motor_.rotate(-59.0f * adj, RPM);

    			if(done() == false) break;

    			motor_.move((4706.0f - (2614.0f - 393.0f)) + (7148.0f - 1000.0f), RPM);

    			done();

    			break;
    		}
    		
    		case '2' :
    		{
    			motor_.move(2840.0f - 393.0f - 393.0f, 250.0f);
    			
    			if(done() == false) break;
    			
        		drop_.set(true);
        		
        		Timer.delay(0.5f);

    			motor_.move(-2840.0f - 393.0f - 393.0f - 400.0f, 125.0f);

    			if(done() == false) break;

        		drop_.set(false);

    			motor_.rotate(-45.0f * adj, RPM);

    			if(done() == false) break;

    			motor_.move(4949.0f, RPM);

    			if(done() == false) break;

    			motor_.rotate(+45.0f * adj, RPM);

    			if(done() == false) break;

    			motor_.move(+6000.0f, RPM);

    			done();
    			
    			break;
    		}
    	
    		case '3' :
    		{
    			motor_.move(2146.0f - 393.0f, RPM);
    			
    			if(done() == false) break;
    			
    			motor_.rotate(+59.0f * adj, RPM);

    			if(done() == false) break;

    			motor_.move(2238.0f - 393.0f, RPM);

    			if(done() == false) break;

        		drop_.set(true);

    			motor_.move(-500.0f, RPM / 2.0f);

    			if(done() == false) break;

        		drop_.set(false);

    			motor_.move(-(2238.0f - 500.0f - 393.0f), RPM);

    			if(done() == false) break;

    			motor_.rotate(+59.0f * adj, RPM);

    			if(done() == false) break;

    			motor_.move(4706.0f - (2146.0f - 393.0f), RPM);

    			if(done() == false) break;

    			motor_.rotate(-45.0f * adj, RPM);

    			if(done() == false) break;

    			motor_.move(7000.0f, RPM);

    			if(done() == false) break;

    			motor_.rotate(+45.0f * adj, RPM);

    			done();

    			break;
    		}
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

    	if(InstalledHardware.DRIVE == true)
    	{
        	motor_.setup(false);
    	}

    	Scheduler.getInstance().removeAll();
    	
        while(isOperatorControl() && isEnabled())
        {
        	moveRobot();
        	
        	slowMove();
        	
        	dropGear();
        	
        	switchCamera();
        	
        	climbRope();

        /*
        	if(InstalledHardware.SPOKE_SENSOR == true)
        	{
        		System.out.println("S:" + spokesensor_.position());	// TODO fix this!
        	}
        */
        	
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

    	motor_.setup(true);
    	
    	motor_.rotate(+90.0f, RPM / 2.0f);
//    	motor_.move(+500.0f, 200.0f);

    	done();

    	motor_.rotate(-90.0f, RPM / 2.0f);
//    	motor_.move(-500.0f, 200.0f);

    	done();
    	
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
    	if(InstalledHardware.DRIVE == true)
    	{
        	switch(cameraInUse_)
        	{
        		case FRONT_CAMERA :
        		{
                	motor_.arcadeDrive(-joystick_.getY() * speedLimit_, -joystick_.getX() * speedLimit_, true);
                	
        			break;
        		}
        		
        		case REAR_CAMERA :
        		{
                	motor_.arcadeDrive(+joystick_.getY() * speedLimit_, -joystick_.getX() * speedLimit_, true);
                	
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
     * Allow slow movement.
     */
    
    public void slowMove()
    {
    	if((InstalledHardware.JOYSTICK == true)
    	&& (InstalledHardware.DRIVE == true))
    	{
        	switch(joystick_.getPOV(0))
        	{
        		case 45 :
        		case 90 :
        		case 135 :
        		{
            		motor_.arcadeDrive(0.0, -0.45f, true);

        			break;
        		}
        		
        		case 225 :
        		case 270 :
        		case 315 :
        		{
            		motor_.arcadeDrive(0.0, +0.55f, true);

            		break;
        		}
        		
        		default :
        		{
//        			System.out.println(controller.getPOV(0));
        			
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
    	if(InstalledHardware.JOYSTICK == true)
    	{
            if(joystick_.getRawButton(Controller.BUTTON_A) == true)
            {
            	dropGearButton_ = true;

            	if(InstalledHardware.GEAR_DROP == true)
            	{
            		drop_.set(true);
            	}
            }
            else
            {
        		if(dropGearButton_ == true)
            	{
            		dropGearButton_ = false;
            		
                	if(InstalledHardware.GEAR_DROP == true)
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
    	if(InstalledHardware.JOYSTICK == true)
    	{
            if(joystick_.getRawButton(Controller.BUTTON_START) == true)
            {
            	if(cameraSwitchButton_ == false)
            	{
            		if(cameraInUse_ == FRONT_CAMERA)
            		{
            			if(InstalledHardware.REAR_CAMERA == true)
            			{
            				request(REAR_CAMERA);
            			}
            		}
            		else
            		{
            			if(InstalledHardware.FRONT_CAMERA == true)
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
    	if((InstalledHardware.CLIMB == true)
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
            		if(current >= 17.0)
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