package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.SpeedController;

public class SlideTrain {
	
	protected SpeedController m_frontLeftMotor;
    protected SpeedController m_frontRightMotor;
    protected SpeedController m_backLeftMotor;
    protected SpeedController m_backRightMotor;
    protected SpeedController m_slideMotor;
	
	public SlideTrain(SpeedController leftFrontMotor, SpeedController leftBackMotor, SpeedController rightFrontMotor, 
					  SpeedController rightBackMotor, SpeedController slideMotor)
	{
		m_frontLeftMotor = leftFrontMotor;
		m_frontRightMotor = leftBackMotor;
		m_backLeftMotor = rightFrontMotor;
		m_backRightMotor = rightBackMotor;
		m_slideMotor = slideMotor;
		
	}
	
	public void arcadeDrive(double y, double z, double x, double slideCorrection, boolean squaredInputs)
	{
		
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
		
		m_frontRightMotor.set(limit((y + (slideCorrection * x)) - z));
		m_frontLeftMotor.set(limit((y - (slideCorrection * x)) + z));
		m_backRightMotor.set(limit((y + (slideCorrection * x)) - z));
		m_backLeftMotor.set(limit((y - (slideCorrection * x)) + z));
		m_slideMotor.set(limit(x));
	}
	
    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }
	
}
