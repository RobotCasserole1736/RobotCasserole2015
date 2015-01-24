
package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	//*Constants*
	
	//-Component IDS
	final static int JOY1_INT = 0;
	
	final static int LEFTROBOT_FRONTMOTOR_ID = 0;
	final static int LEFTROBOT_BACKMOTOR_ID = 1;
	final static int RIGHTROBOT_FRONTMOTOR_ID = 3;
	final static int RIGHTROBOT_BACKMOTOR_ID = 2;
	final static int SLIDE_MOTOR_ID = 4;
	
	//-Controller Buttons
	final static int XBOX_A_BUTTON = 1;
	final static int XBOX_B_BUTTON = 2;
	final static int XBOX_X_BUTTON = 3;
	final static int XBOX_Y_BUTTON = 4;
	final static int XBOX_LEFT_BUTTON = 5;
	final static int XBOX_RIGHT_BUTTON = 6;
	final static int XBOX_SELECT_BUTTON = 7;
	final static int XBOX_START_BUTTON = 8;
	final static int XBOX_LSTICK_BUTTON = 9;
	final static int XBOX_RSTICK_BUTTON = 10;
	
	//-Controller Axes
	final static int XBOX_LSTICK_XAXIS = 0;
	final static int XBOX_LSTICK_YAXIS = 1;
	final static int XBOX_LTRIGGER_AXIS = 2;
	final static int XBOX_RTRIGGER_AXIS = 3;
	final static int XBOX_RSTICK_XAXIS = 4;
	final static int XBOX_RSTICK_YAXIS = 5;
	
	//-Slide Correction Tuning Value
	final static double slideTune = .4;
	
	//PID Values
	final static double P = 1.0;
	final static double I = 0.01;
	final static double D = 1.2;
	
	//-Gyro Values
    final static int GYRO_ID = 0;
    final static double GYRO_SENSITIVITY = 0.007;
    double gyroValue;
	
    //-Closed/Open Loop
    final static boolean openLoop = false;
	
	//*Declaring robot parts*
	//-Joystick
	Joystick joy1;
	
	//-DriveTrain Motors
	VictorSP frontLeftMotor;
	VictorSP backLeftMotor;
	VictorSP frontRightMotor;
	VictorSP backRightMotor;
	VictorSP slideMotor;
	
	//-DriveTrain
	SlideTrain slideTrain;	
	
	//-Gyro
	Gyro gyro;
	
    public void robotInit() {

    	//Joystick
    	joy1 = new Joystick(JOY1_INT);
    	//Motors
    	frontLeftMotor = new VictorSP(LEFTROBOT_FRONTMOTOR_ID);
    	backLeftMotor = new VictorSP(LEFTROBOT_BACKMOTOR_ID);
    	frontRightMotor = new VictorSP(RIGHTROBOT_FRONTMOTOR_ID);
    	backRightMotor = new VictorSP(RIGHTROBOT_BACKMOTOR_ID);
    	slideMotor = new VictorSP(SLIDE_MOTOR_ID);
    	//Drive Train
    	slideTrain = new SlideTrain(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, slideMotor, P, I, D);
    	slideTrain.enable();
    	gyro = new Gyro(GYRO_ID);
		gyro.initGyro();
		gyro.setPIDSourceParameter(PIDSourceParameter.kAngle);
		gyro.setSensitivity(GYRO_SENSITIVITY);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	//acquire gyroscope value, convert to radians, wrap to proper range
    	gyroValue = (gyro.getAngle()*(Math.PI/180)) % (2*Math.PI);
    	if(gyro.getAngle() < 0)
    		gyroValue += 2*Math.PI;
    	
    	//Run either open- or closed-loop control, depending on what is requested
    	if(!openLoop)
    		slideTrain.PIDarcadeDrive(joy1.getDirectionRadians(), joy1.getMagnitude(), joy1.getRawAxis(XBOX_RSTICK_XAXIS), true, gyroValue);
    	else
    		slideTrain.arcadeDrive((-1 *joy1.getRawAxis(XBOX_LSTICK_YAXIS)), joy1.getRawAxis(XBOX_RSTICK_XAXIS), joy1.getRawAxis(XBOX_LSTICK_XAXIS), slideTune, true);
        
    	//Set motor values. Note right motor values are inverted due to the physical
    	//configuration of the robot drivetrain
		frontRightMotor.set(-slideTrain.frontRightMotorValue);
		frontLeftMotor.set(slideTrain.frontLeftMotorValue);
		backRightMotor.set(-slideTrain.backRightMotorValue);
		backLeftMotor.set(slideTrain.backLeftMotorValue);
		slideMotor.set(slideTrain.slideMotorValue);
		
		//Spit some debug info out to the Riolog
		System.out.print("Front Right Motor: " + String.format( "%.2f", slideTrain.frontRightMotorValue) + " ");
		System.out.print("Front Left Motor: " + String.format( "%.2f", slideTrain.frontLeftMotorValue) + " ");
		System.out.print("Back Right Motor: " + String.format( "%.2f", slideTrain.backRightMotorValue) + " ");
		System.out.print("Back Left Motor: " + String.format( "%.2f", slideTrain.backLeftMotorValue) + " ");
		System.out.print("Slide Motor: " + String.format( "%.2f", slideTrain.slideMotorValue) + " ");
		System.out.println("Gyro: " + String.format( "%.2f", gyroValue) + " PID Value:" + String.format( "%.2f", slideTrain.PIDOutput));
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
