package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TalonSRX;

public class OpenLoopArm 
{
	TalonSRX motor;
	
	DigitalInput bottomSensor, topSensor;
	
	public OpenLoopArm(int motorID, int bottomSensor, int topSensor)
	{
		motor = new TalonSRX(motorID);
		this.bottomSensor = new DigitalInput(bottomSensor);
		this.topSensor = new DigitalInput(topSensor);
	}
	
	public void move(double motorValue)
	{
		if(bottomSensor.get() || topSensor.get())
			motor.set(0);
		else
			motor.set(motorValue);
	}
	
	public boolean atEnd()
	{
		if(bottomSensor.get() || topSensor.get())
			return true;
		else
			return false;
	}
}
