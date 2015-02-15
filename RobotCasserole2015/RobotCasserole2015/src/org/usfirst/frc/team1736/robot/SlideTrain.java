package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SlideTrain extends PIDSubsystem{
	
	protected SpeedController frontLeftMotor;
    protected SpeedController frontRightMotor;
    protected SpeedController backLeftMotor;
    protected SpeedController backRightMotor;
    protected SpeedController slideMotor;
    protected I2CGyro gyro = null;
    
    protected Encoder leftEncoder;
    protected Encoder rightEncoder;
    
    final static int leftEncoderID = 2;
    final static int leftEncoderID_2 = 3;
    
    final static int rightEncoderID = 4;
    final static int rightEncoderID_2 = 5;
    
	double frontLeftMotorValue;
	double frontRightMotorValue;
	double backLeftMotorValue;
	double backRightMotorValue;
	double slideMotorValue;
    
    double setPoint = 0; //Closed loop desired angle (set by driver) In Degrees
    volatile double PIDOutput = 0; //corrective effort commanded by PID algorithm
    
    //PID correction factor
    double K1 = 1;
    double K2 = 1;
    //Traversal correction factor
    double K3 = 0.4;
    double K4 = 0.4;
    //Feed Forward constants
    double K5 = 0.5;
    double K6 = 0.5;
    
    double previous_R_XAxis_value = 0;
    
    double setPointMultiplier = 288 * .02; //Multiplicative factor to tune max rate of rotation in closed loop
    
    Boolean needsToLoop = false;
    int numberOfLoops = 0;
    final int PIDWaitLoop = 27;
    
    static double lastTime = 0;
	
	public SlideTrain(SpeedController leftFrontMotor, SpeedController leftBackMotor, SpeedController rightFrontMotor, 
					  SpeedController rightBackMotor, SpeedController slideMotor, I2CGyro gyro, double P, double I, double D)
	{
		super("SlideTrain",P,I,D);
		this.frontLeftMotor = leftFrontMotor;
		this.frontRightMotor = rightFrontMotor;
		this.backLeftMotor = leftBackMotor;
		this.backRightMotor = rightBackMotor;
		this.slideMotor = slideMotor;
		this.gyro = gyro;
		
		leftEncoder = new Encoder(leftEncoderID, leftEncoderID_2);
		rightEncoder = new Encoder(rightEncoderID, rightEncoderID_2);
		
		//PID Subsystem stuff
		getPIDController().setContinuous(true); //Allow the PID algorithm to correct for error by traversing through the 360<->0 degrees.
		                                        //this is allowable because rotation is continuous (as supposed to a slide motion)
		getPIDController().setOutputRange(-1, 1); //we allow the output range to match that of the motor values
		getPIDController().setInputRange(0, Math.PI*2); //Input is a full circle in radians
	}
	
	//Open loop drive
	public void arcadeDrive(double y, double z, double x, double slideCorrection, boolean squaredInputs)
	{
		//If the user wants to square the joystick axes inputs, do so, but preserve the sign of the input.
		//note this algorithm only works because joystick comes in in the range -1 to 1
		if(squaredInputs)
		{
			if (y >= 0.0) {
                y = (y * y);
            } else {
                y = -(y * y);
            }
            if (z >= 0.0) {
                z = (z * z);
            } else {
                z = -(z * z);
            }
            if (x >= 0.0) {
                x = (x * x);
            } else {
                x = -(x * x);
            }
            if (slideCorrection >= 0.0) {
                slideCorrection = (slideCorrection * slideCorrection);
            } else {
                slideCorrection = -(slideCorrection * slideCorrection);
            }
		}
		
		//calculate output motor values
		//limit between -1 and 1
		//see github wiki for explanation of equations
		frontRightMotorValue = ((limit((y + (slideCorrection * x)) - z)));
		frontLeftMotorValue = (limit((y - (slideCorrection * x)) + z));
		backRightMotorValue = ((limit((y + (slideCorrection * x)) - z)));
		backLeftMotorValue = (limit((y - (slideCorrection * x)) + z));
		slideMotorValue = (limit(x));
	}
	
	//Closed loop control
	public void PIDarcadeDrive(double lDirectionRad, double lMagnitude, double R_XAxis, boolean squaredInputs)
	{
		double twistValue = 0;
		//square the magnititude input if requested (for better low-speed control)
		if(squaredInputs)
		{
			lMagnitude *= lMagnitude;
			twistValue = R_XAxis/ Math.abs(R_XAxis) * R_XAxis*R_XAxis;
		}
		else if(!squaredInputs)
		{
			twistValue = R_XAxis;
		}
		
		//make a local copy of the current gyroscope value (radians)
		double gyroValue = getGyroValue();
		
		setPointMultiplier = 288 * (Timer.getFPGATimestamp() - lastTime);
		
		lastTime = Timer.getFPGATimestamp();
		
		//calculate the pose angle setpoint 
		//increase/decrease the setpoint based on the current value of the 
		//right joystick x axis. Wrap the value between 0 and 360 degrees.
		if(twistValue > 0.15 && setPoint > 0)
		{
			setPoint = (setPoint + (twistValue * setPointMultiplier)) % 360;
		}
		else if(twistValue < -0.15 && setPoint > 0)
		{
			setPoint = (setPoint + (twistValue * setPointMultiplier)) % 360;
		}
		else if(twistValue < -0.15 && setPoint == 0)
		{
			setPoint = (360 + (twistValue * setPointMultiplier)) % 360;
		}
		else if(twistValue > 0.15 && setPoint == 0)
		{
			setPoint = (setPoint + (twistValue * setPointMultiplier)) % 360;
		}
		else if(setPoint < 0)
		{
			setPoint = (360 + (twistValue * setPointMultiplier)) % 360;
		}
		SmartDashboard.putNumber("Setpoint", setPoint);
		SmartDashboard.putNumber("PIDOutput", PIDOutput);
		SmartDashboard.putNumber("Gyro", gyroValue);
		SmartDashboard.putNumber("Left Encoder Distance", leftEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder Distance", rightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder Rate", leftEncoder.getRate());
		SmartDashboard.putNumber("Right Encoder Rate", rightEncoder.getRate());
		
		//From deadzone to out of deadzone
		if((previous_R_XAxis_value < 0.15 && previous_R_XAxis_value > -0.15) && (twistValue > 0.15 || twistValue < -0.15))
		{
			//System.out.println("From deadzone to out of deadzone");
			getPIDController().reset();
			needsToLoop = false;
			numberOfLoops = 0;
		}
		//From out of deadzone to deadzone
		else if((previous_R_XAxis_value > 0.15 || previous_R_XAxis_value < -0.15) && (twistValue < 0.15 && twistValue > -0.15))
		{
			//System.out.println("From out of deadzone to deadzone");
			needsToLoop = true;
		}		
		
		if(needsToLoop)
		{
			//System.out.println("Needs to loop");
			numberOfLoops++;
			if(numberOfLoops >= PIDWaitLoop)
			{
				numberOfLoops = 0;
				needsToLoop = false;
				setPoint = gyroValue;
				setSetpoint(Math.toRadians(setPoint));
				getPIDController().reset();
				getPIDController().enable();
			}
		}
		
		//Convert the setpoint to radians and send that value to the PID controller
		setSetpoint(Math.toRadians(setPoint));
		
		//Print setpoint to Riolog for debugging
		//System.out.print(setPoint + "\n");
		
		
		//Perform motor Calculations. See github wiki for explanation of equations
		//Note that PIDOutput is given a value asynchronously by the PID controller
		double m1_traversal = (lMagnitude * Math.sin((Math.PI/2) - lDirectionRad));
		double m1_rotational = ((-1) * PIDOutput) * K1;
		double m1_traversal_correction = (-1 * Math.sin(lDirectionRad)) * lMagnitude * K3;
		double m1_feed_forward = (K5 * R_XAxis);
		
		double m2_traversal = (lMagnitude * Math.sin((Math.PI/2) - lDirectionRad));
		double m2_rotational = PIDOutput * K2;
		double m2_traversal_correction = (Math.sin(lDirectionRad)) * lMagnitude * K4;
		double m2_feed_forward = (K6 * R_XAxis);
		
		double m3_traversal = lMagnitude * Math.cos((Math.PI/2) - lDirectionRad);
		
		//Add all contributers to each motor's value and limit the result to the proper range
		frontLeftMotorValue = limit((m1_traversal + m1_rotational + m1_traversal_correction + m1_feed_forward));
		backLeftMotorValue = limit((m1_traversal + m1_rotational + m1_traversal_correction + m1_feed_forward));
		frontRightMotorValue = limit((m2_traversal + m2_rotational + m2_traversal_correction - m2_feed_forward));
		backRightMotorValue = limit((m2_traversal + m2_rotational + m2_traversal_correction - m2_feed_forward));
		slideMotorValue = limit((m3_traversal));
		
		previous_R_XAxis_value = twistValue;
	}
	
	public void PIDTurning(double degrees)
	{
		if(degrees >= 0)
		{
			setPoint = degrees;
		}
		else if(degrees <= 0)
		{
			setPoint = 360 - degrees;
		}
		
		setSetpoint(Math.toRadians(setPoint));
		
		double m1_rotational = ((-1) * PIDOutput) * K1;
		double m2_rotational = PIDOutput * K2;
		
		frontLeftMotorValue = m1_rotational;
		backLeftMotorValue = m1_rotational;
		
		frontRightMotorValue = m2_rotational;
		backRightMotorValue = m2_rotational;
	}
	
	//limit an input to the valid range for motor outputs (-1 to 1)
    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    public void driveStraight(double distance)
    {
    	if(leftEncoder.getDistance() < (distance / 2) || rightEncoder.getDistance() < (distance / 2))
    	{
    		PIDarcadeDrive(0, 1, 0, false);
    	}
    	else if((leftEncoder.getDistance() < ((3/4) * distance)) || (rightEncoder.getDistance() < ((3/4) * distance))) 
    	
    	{
    		PIDarcadeDrive(0, 0.5, 0, false);
    	}
    	else if(((leftEncoder.getDistance() >= distance) || (rightEncoder.getDistance() >= distance)))
    	{
    		PIDarcadeDrive(0, 0, 0, false);
    	}
    }
    
    public void turnRight(double degrees)
    {
    	PIDTurning(degrees);
    }
    
    public void turnLeft(double degrees)
    {    	
    	PIDTurning(-1 * degrees);
    }
    
    public void resetEncoders()
    {
    	leftEncoder.reset();
    	rightEncoder.reset();
    }
    
    public double getGyroValue()
    {
    	double gyroValue = (gyro.get_gyro_angle()*(Math.PI/180)) % (2*Math.PI);
    	if(gyro.get_gyro_angle() < 0)
    	{
    		gyroValue += 2*Math.PI;
    	}
    	return gyroValue;
    }
    
    //PID controller override. This function is called by the PID controller thread asynchronously from the 
    //rest of the robot code. It should always return the current input value to the PID (for us, this is from the gyroscope)
	@Override
	protected double returnPIDInput() {
		return getGyroValue(); //in radians
	}

	//PID output usage function. 
	//This function will be called asynchronously by the PID controller thread. Every time it is called, we need to do 
	//something with the argument "output", which represents the most recent output calculation from the PID loop.
	//For our purposes, we just write it to a variable in SlideTrain for later usage in closed loop calculations.
	@Override
	protected void usePIDOutput(double output) {
		PIDOutput = -1 * output;
	}

	//I have literally no idea what this function is for. Apparently you HAVE to override it. Ah well.
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public void zeroAngle()
	{
		setPoint = gyro.get_gyro_angle();
		setSetpoint(Math.toRadians(setPoint));
		getPIDController().reset();
		getPIDController().enable();
	}
	
    public void strafe(int direction)
    {
    	if(direction == 1)
    		PIDarcadeDrive(Math.toRadians(270), 0.25, 0, true);
    	else if(direction == -1)
    		PIDarcadeDrive(Math.toRadians(90), 0.25, 0, true);
    	else
    		PIDarcadeDrive(0, 0, 0, true);
        		
    }
    
	
}
