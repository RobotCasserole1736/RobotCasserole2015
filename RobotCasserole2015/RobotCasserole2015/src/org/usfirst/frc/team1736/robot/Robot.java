
package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	//This is a test.  Just a comment to check out commits and stuff.
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	//*Constants*
	
	//-Component IDS
	final static int JOY1_INT = 0;
	
	final static int LEFTROBOT_FRONTMOTOR_ID = 0;
	final static int LEFTROBOT_BACKMOTOR_ID = 0;
	final static int RIGHTROBOT_FRONTMOTOR_ID = 0;
	final static int RIGHTROBOT_BACKMOTOR_ID = 0;
	
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
	final static double slideTune = .75;
	
	//PID Values
	final static double P = 0;
	final static double I = 0;
	final static double D = 0;
	
	
	//*Declaring robot parts*
	//-Joystick
	Joystick joy1;
	
	//-DriveTrain Motors
	VictorSP leftRobotMotor_Front;
	VictorSP leftRobotMotor_Back;
	VictorSP rightRobotMotor_Front;
	VictorSP rightRobotMotor_Back;
	VictorSP slideMotor;
	
	//-DriveTrain
	SlideTrain slideTrain;	
	
    public void robotInit() {

    	//Joystick
    	joy1 = new Joystick(JOY1_INT);
    	//Motors
    	leftRobotMotor_Front = new VictorSP(LEFTROBOT_FRONTMOTOR_ID);
    	leftRobotMotor_Back = new VictorSP(LEFTROBOT_BACKMOTOR_ID);
    	rightRobotMotor_Front = new VictorSP(RIGHTROBOT_FRONTMOTOR_ID);
    	rightRobotMotor_Back = new VictorSP(RIGHTROBOT_BACKMOTOR_ID);
    	//Drive Train
    	slideTrain = new SlideTrain(leftRobotMotor_Front, leftRobotMotor_Back, rightRobotMotor_Front, rightRobotMotor_Back, slideMotor, P, I, D);
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
        slideTrain.PIDarcadeDrive(joy1, joy1.getRawAxis(XBOX_LSTICK_YAXIS), joy1.getRawAxis(XBOX_RSTICK_XAXIS), joy1.getRawAxis(XBOX_LSTICK_XAXIS), true);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
