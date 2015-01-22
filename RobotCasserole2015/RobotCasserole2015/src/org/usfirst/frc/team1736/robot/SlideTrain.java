package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class SlideTrain extends PIDSubsystem{
	
	protected SpeedController m_frontLeftMotor;
    protected SpeedController m_frontRightMotor;
    protected SpeedController m_backLeftMotor;
    protected SpeedController m_backRightMotor;
    protected SpeedController m_slideMotor;
    
    final static int GYRO_ID = 0;
    final static double GYRO_SENSITIVITY = 0.007;
    double setPoint = 0;
    double PIDOutput;
    //M1
    double K1 = 1;
    double K2 = 1;
    //M2
    double K3 = 1;
    double K4 = 1;
    
    double setPointMultiplier = 1;
    
    Gyro gyro;
	
	public SlideTrain(SpeedController leftFrontMotor, SpeedController leftBackMotor, SpeedController rightFrontMotor, 
					  SpeedController rightBackMotor, SpeedController slideMotor, double P, double I, double D)
	{
		super("SlideTrain",P,I,D);
		m_frontLeftMotor = leftFrontMotor;
		m_frontRightMotor = rightFrontMotor;
		m_backLeftMotor = leftBackMotor;
		m_backRightMotor = rightBackMotor;
		m_slideMotor = slideMotor;
		gyro = new Gyro(GYRO_ID);
		gyro.initGyro();
		gyro.setPIDSourceParameter(PIDSourceParameter.kAngle);
		gyro.setSensitivity(GYRO_SENSITIVITY);
		getPIDController().setContinuous(true);
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
		
		m_frontRightMotor.set((limit((y + (slideCorrection * x)) - z)) * -1);
		m_frontLeftMotor.set(limit((y - (slideCorrection * x)) + z));
		m_backRightMotor.set((limit((y + (slideCorrection * x)) - z)) * -1);
		m_backLeftMotor.set(limit((y - (slideCorrection * x)) + z));
		m_slideMotor.set(limit(x));
	}
	
	public void PIDarcadeDrive(Joystick joy, double L_XAxis, double L_YAxis, double R_XAxis, boolean squaredInputs)
	{
		//PID Subsystem stuff
		setOutputRange(-1, 1);
		
		
		if(R_XAxis > 0.5 && setPoint > 0)
		{
			setPoint = (setPoint + (R_XAxis * setPointMultiplier)) % 360;
		}
		else if(R_XAxis < -0.5 && setPoint > 0)
		{
			setPoint = (setPoint + (R_XAxis * setPointMultiplier)) % 360;
		}
		else if(R_XAxis < -0.5 && setPoint == 0)
		{
			setPoint = (360 + (R_XAxis * setPointMultiplier)) % 360;
		}
		else if(R_XAxis > 0.5 && setPoint == 0)
		{
			setPoint = (setPoint + (R_XAxis * setPointMultiplier)) % 360;
		}
		else if(setPoint < 0)
		{
			setPoint = (360 + (R_XAxis * setPointMultiplier)) % 360;
		}
		
		setSetpoint(setPoint);
		
		System.out.print(setPoint + "\n");
		
		double m1_traversal = (joy.getMagnitude() * Math.sin((Math.PI/2) - joy.getDirectionRadians()));
		double m1_rotational = (((-1) * PIDOutput) * (gyro.pidGet() - setPoint)) * K1;
		double m1_traversal_correction = (-1 * Math.sin(joy.getDirectionRadians())) * joy.getMagnitude() * K2;
		
		double m2_traversal = (joy.getMagnitude() * Math.sin((Math.PI/2) - joy.getDirectionRadians()));
		double m2_rotational = (PIDOutput * (gyro.pidGet() - setPoint)) * K3;
		double m2_traversal_correction = (Math.sin(joy.getDirectionRadians())) * joy.getMagnitude() * K4;
		
		double m3_traversal = joy.getMagnitude() * Math.cos((Math.PI/2) - joy.getDirectionRadians());
		
		m_frontRightMotor.set(m1_traversal + m1_rotational + m1_traversal_correction);
		m_frontLeftMotor.set(m1_traversal + m1_rotational + m1_traversal_correction);
		m_backRightMotor.set(m2_traversal + m2_rotational + m2_traversal_correction);
		m_backLeftMotor.set(m2_traversal + m2_rotational + m2_traversal_correction);
		m_slideMotor.set(m3_traversal);
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

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return gyro.pidGet();
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub
		PIDOutput = output;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
}
