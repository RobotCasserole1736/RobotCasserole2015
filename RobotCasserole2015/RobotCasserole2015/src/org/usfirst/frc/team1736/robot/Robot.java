
package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	/*private variables*/
	private Joystick stick;
	private VictorSP Motor1;
	private VictorSP Motor2;
	private VictorSP Motor3;
	
	/*Robot config section*/
	private static final int DRIVER_JOYSTICK_INDEX = 0;
	private static final int MOTOR_1_PORT = 0;
	private static final int MOTOR_2_PORT = 1;
	private static final int MOTOR_3_PORT = 2;
	
	private static final double OPEN_LOOP_CROT_GAIN = 0.3;
	
	/*Drive Type functions*/
	private void openLoopRobotOriented() {
		double x = stick.getX();
		double y = stick.getY();
		double z = stick.getTwist();
		
		Motor1.set(limit(y + z - OPEN_LOOP_CROT_GAIN * x, 1, -1));
		Motor2.set(limit(y - z + OPEN_LOOP_CROT_GAIN * x, 1, -1));
		Motor3.set(x);
		
		
	}
	private void closedLoopRobotOriented() {
		double x = stick.getX();
		double y = stick.getY();
		double z = stick.getTwist();
	}
	private void closedLoopFeildOriented() {
		double x = stick.getX();
		double y = stick.getY();
		double z = stick.getTwist();
	}

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	stick = new Joystick(DRIVER_JOYSTICK_INDEX);
    	Motor1 = new VictorSP(MOTOR_1_PORT);
    	Motor2 = new VictorSP(MOTOR_2_PORT);
    	Motor3 = new VictorSP(MOTOR_3_PORT);
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
        
    	openLoopRobotOriented(); //Call driver-to-robot mapping fcn.
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    
    /*helper fcns*/
    private double limit(double in, double max, double min) {
    	if(in > max)
    		return max;
    	else if(in < min)
    		return min;
    	else
    		return in;
    }
    
    
}
