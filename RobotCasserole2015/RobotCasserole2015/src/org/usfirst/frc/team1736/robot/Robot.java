
package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Joystick joy1;
	PIDTestMotor motor;
	PowerDistributionPanel pd;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	joy1 = new Joystick(0);
    	motor = new PIDTestMotor(0.000511168926576436579993214543543234243534243567162351565, 0, 0.000035647587453456365254365469767575467576436535, 0, 5000);
    	motor.enable();
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
    	
        if(joy1.getRawButton(1))
        	motor.setSetpoint(625);
        else if(joy1.getRawButton(2))
        	motor.setSetpoint(1250);
        else if(joy1.getRawButton(3))
        	motor.setSetpoint(1875);
        else if(joy1.getRawButton(4))
        	motor.setSetpoint(2500);
        SmartDashboard.putNumber("Motor Speed", motor.returnPIDInput());
        SmartDashboard.putNumber("Setpoint", motor.getSetpoint());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	
    }
    
}
