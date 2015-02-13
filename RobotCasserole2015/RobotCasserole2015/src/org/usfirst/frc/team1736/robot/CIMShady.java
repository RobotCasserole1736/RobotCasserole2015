
package org.usfirst.frc.team1736.robot;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class CIMShady extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	//*Constants*
	
	//-Component IDS
	final static int JOY1_INT = 0;
	final static int JOY2_INT = 0;
	
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
	final static double P = .60;
	final static double I = .075;
	final static double D = 2;
//	
//	//PID Values
//	final static double P = 0;
//	final static double I = 0;
//	final static double D = 0;

	//-Elevator
	final static int ELEVATOR_MOTOR_ID = 0;
	final static double ELEVATOR_P = 0;
	final static double ELEVATOR_I = 0;
	final static double ELEVATOR_D = 0;
	final static int ENCODER_A = 0;
	final static int ENCODER_B = 0;
	final static int BOTTOM_SENSOR_ID = 0;
	final static int TOP_SENSOR_ID = 0;
	final static double MIN_RETRACT_HEIGHT = .05;
	
	boolean Elev_PID = true;

//	//-Gyro Values
//    final static int GYRO_ID = 0;
//    final static double GYRO_SENSITIVITY = 0.007;
    double gyroValue;
    
    //-Compressor IDs
    final static int PRESSURE_SENSOR_ID = 2;
	
    //-Closed/Open Loop
    final static boolean openLoop = false;
	
    //-Autonomous mode
    int autonomousMode = 0;
    
	//*Declaring robot parts*
	//-Joystick
	Joystick joy1;
	Joystick joy2;
	
	//-DriveTrain Motors
	VictorSP frontLeftMotor;
	VictorSP backLeftMotor;
	VictorSP frontRightMotor;
	VictorSP backRightMotor;
	VictorSP slideMotor;
	
	//-Compressor
	Compressor compressor;
	
	//-Solenoids
	Solenoid solenoidInOut;
	Solenoid solenoidOpenClose;
	
	//-Elevator
	Elevator elevator;
	
	//-DriveTrain
	SlideTrain slideTrain;	
	
//	//-Gyro
//	Gyro gyro;
	I2CGyro gyro;
	
	//-Lidar
	LIDAR lidar;
	
	//-Treemap for Elevator Levels
//	TreeMap<Integer, Double> levels;
	//0 is starting level
	Integer currentLevel = 0;
	
	AnalogInput pressureSensor;
	
	int[] levels;
	
    public void robotInit() {

    	//Joystick
    	joy1 = new Joystick(JOY1_INT);
    	joy2 = new Joystick(JOY2_INT);
    	
    	//Motors
    	frontLeftMotor = new VictorSP(LEFTROBOT_FRONTMOTOR_ID);
    	backLeftMotor = new VictorSP(LEFTROBOT_BACKMOTOR_ID);
    	frontRightMotor = new VictorSP(RIGHTROBOT_FRONTMOTOR_ID);
    	backRightMotor = new VictorSP(RIGHTROBOT_BACKMOTOR_ID);
    	slideMotor = new VictorSP(SLIDE_MOTOR_ID);
    	
//    	//Gyro
//    	gyro = new Gyro(GYRO_ID);
//		gyro.initGyro();
//		gyro.setPIDSourceParameter(PIDSourceParameter.kAngle);
//		gyro.setSensitivity(GYRO_SENSITIVITY);
		
    	//Drive Train
    	slideTrain = new SlideTrain(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, slideMotor, gyro, P, I, D);
    	slideTrain.enable();
    	
    	//Compressor
		compressor = new Compressor();
		compressor.start();
		pressureSensor = new AnalogInput(PRESSURE_SENSOR_ID);
		
		//Elevator
		solenoidInOut = new Solenoid(0);
    	solenoidOpenClose = new Solenoid(1);
    	elevator = new Elevator(ELEVATOR_MOTOR_ID, ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, ENCODER_A, ENCODER_B, BOTTOM_SENSOR_ID, TOP_SENSOR_ID);
		elevator.enable();
    	
//    	levels = new TreeMap<Integer, Double>();
//    	levels.put(0, (double) 0);
//    	levels.put(1, (double) 625);
//    	levels.put(2, (double) 1250);
//    	levels.put(3, (double) 1875);
//    	levels.put(4, (double) 2500);
    	
    	lidar = new LIDAR();
    	lidar.start();
    	
    	levels = new int[5];
    	levels[0] = 0;
    	levels[1] = 625;
    	levels[2] = 1250;
    	levels[3] = 1875;
    	levels[4] = 2500;
    	
    	SmartDashboard.putNumber("Autonomous Mode:", autonomousMode);
    }

    public void autonomousInit() {
    	
    	SmartDashboard.getNumber("Autonomous Mode:");
    	slideTrain.lastTime = Timer.getFPGATimestamp();
    	
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    	switch(autonomousMode)
    	{
    	case 1:
    		
    		break;
    	case 2:
    		
    		break;
    		
    	case 3:
    		
    		break;
    	}
    	
    }

    public void teleopInit() {
    	
    	slideTrain.lastTime = Timer.getFPGATimestamp();
    	
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	System.out.println("Lidar = " + lidar.getDistanceIn());
    	
    	//acquire gyroscope value, convert to radians, wrap to proper range
    	gyroValue = (gyro.get_gyro_angle()*(Math.PI/180)) % (2*Math.PI);
    	if(gyro.get_gyro_angle() < 0)
    		gyroValue += 2*Math.PI;
    	
    	//Run either open- or closed-loop control, depending on what is requested
    	if(!openLoop)
    		slideTrain.PIDarcadeDrive(joy1.getDirectionRadians(), joy1.getMagnitude(), joy1.getRawAxis(XBOX_RSTICK_XAXIS), true);
    	else
    		slideTrain.arcadeDrive((-1 *joy1.getRawAxis(XBOX_LSTICK_YAXIS)), joy1.getRawAxis(XBOX_RSTICK_XAXIS), joy1.getRawAxis(XBOX_LSTICK_XAXIS), slideTune, true);
    	if(!slideTrain.getPIDController().isEnable())
    		gyro.reset_gyro_angle();
    		
    	if(joy2.getRawButton(1))
    	{
    		solenoidInOut.set(true);
    	}
    	else if(joy2.getRawButton(2) && elevator.returnPIDInput() > MIN_RETRACT_HEIGHT)
    	{
    		solenoidInOut.set(false);
    	}
    	if(joy2.getRawButton(3))
    	{
    		solenoidOpenClose.set(true);
    	}
    	else if(joy2.getRawButton(4))
    	{
    		solenoidOpenClose.set(false);
    	}
    	
    	if(joy2.getRawButton(XBOX_RIGHT_BUTTON) && currentLevel >= 0 && Elev_PID)
    	{
    		elevator.setSetpoint(levels[currentLevel + 1]);
    		if(currentLevel > 0)
    		{
    			currentLevel = currentLevel + 1;
    		}
    	}
    	else if(joy2.getRawButton(XBOX_LEFT_BUTTON) && currentLevel >= 0)
    	{
    		elevator.setSetpoint(levels[currentLevel - 1]);
    		if(currentLevel > 0)
    		{
    			currentLevel = currentLevel - 1;
    		}
    	}
    	else if(joy2.getRawButton(XBOX_SELECT_BUTTON) && joy2.getRawButton(XBOX_START_BUTTON))
    	{
    		elevator.disable();
    		Elev_PID = false;
    	}
    	else if(joy2.getRawButton(XBOX_RIGHT_BUTTON) && !Elev_PID)
    	{
    		elevator.setMotorSpeed(0.5);
    	}
    	else if(joy2.getRawButton(XBOX_LEFT_BUTTON) && !Elev_PID)
    	{
    		elevator.setMotorSpeed(-0.5);
    	}
    	else if(!Elev_PID)
    	{
    		elevator.setMotorSpeed(0);
    	}
    		
        
    	//Set motor values. Note right motor values are inverted due to the physical
    	//configuration of the robot drivetrain
		frontRightMotor.set(-slideTrain.frontRightMotorValue);
		frontLeftMotor.set(slideTrain.frontLeftMotorValue);
		backRightMotor.set(-slideTrain.backRightMotorValue);
		backLeftMotor.set(slideTrain.backLeftMotorValue);
		slideMotor.set(slideTrain.slideMotorValue);
		
		//Spit some debug info out to the Riolog
//		System.out.print("Front Right Motor: " + String.format( "%.2f", slideTrain.frontRightMotorValue) + " ");
//		System.out.print("Front Left Motor: " + String.format( "%.2f", slideTrain.frontLeftMotorValue) + " ");
//		System.out.print("Back Right Motor: " + String.format( "%.2f", slideTrain.backRightMotorValue) + " ");
//		System.out.print("Back Left Motor: " + String.format( "%.2f", slideTrain.backLeftMotorValue) + " ");
//		System.out.print("Slide Motor: " + String.format( "%.2f", slideTrain.slideMotorValue) + " ");
//		System.out.println("Gyro: " + String.format( "%.2f", gyroValue) + " PID Value:" + String.format( "%.2f", slideTrain.PIDOutput));
		SmartDashboard.putNumber("Pressure Voltage", pressureSensor.getAverageVoltage());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
