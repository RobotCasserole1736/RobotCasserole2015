package org.usfirst.frc.team1736.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.PIDController;

public class I2CGyro 
{
	I2CGyro m_gyro;
	
	java.util.Timer m_controlLoop;

	double m_period = 20;
	
    private class GyroTask extends TimerTask 
    {
        private PIDController m_controller;

        public GyroTask(I2CGyro gyro) 
        {
            if (gyro == null) 
            {
                throw new NullPointerException("Given PIDController was null");
            }
            m_gyro = gyro;
        }

        @Override
        public void run() 
        {
            m_gyro.periodic_update();
        }
    }
    
    public I2CGyro()
    {
        m_controlLoop = new java.util.Timer();
        m_controlLoop.schedule(new GyroTask(this), 0L, (long) (m_period * 1000));
    }
    
    public synchronized void periodic_update()
    {
    	
    }
    
	public synchronized double get_gyro_angle()
	{
		return 0; //change
	}

	public synchronized void reset_gyro_angle()
	{
		
	}
    
}
