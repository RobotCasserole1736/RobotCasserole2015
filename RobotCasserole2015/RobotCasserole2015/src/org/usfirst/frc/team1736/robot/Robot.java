
package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	YellowToteTracker tracker;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	//This line must be called well before (couple of seconds) the OpenCV functions are used
    	System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    public void teleopInit()
    {
    	tracker = new YellowToteTracker();
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	tracker.processImage();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
