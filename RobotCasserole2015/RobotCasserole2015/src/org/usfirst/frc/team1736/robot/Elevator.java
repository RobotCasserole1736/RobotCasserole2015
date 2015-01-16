package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class Elevator extends PIDSubsystem
{
	Talon motor;
	Encoder encoder;
	DigitalInput zeroSensor;
	
	final int MAX_DISTANCE = 1000;
	final int DISTANCE_PER_PULSE = 1;
	
	public Elevator(int motorID, int p, int i, int d, int EncoderA, int EncoderB, int zeroSensor)
	{
		super("Elevator", p, i, d);
		
		motor = new Talon(motorID);
		encoder = new Encoder(EncoderA, EncoderB, true, EncodingType.k4X);
		encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
		this.zeroSensor = new DigitalInput(zeroSensor);
		
		setInputRange(0, MAX_DISTANCE);
		setOutputRange(-1, 1);
	}

	@Override
	protected double returnPIDInput() 
	{
		// TODO Auto-generated method stub
		return encoder.getDistance();
	}

	@Override
	protected void usePIDOutput(double output) 
	{
		// TODO Auto-generated method stub
		motor.set(output);
	}

	@Override
	protected void initDefaultCommand() 
	{
		// TODO Auto-generated method stub
		
	}
	
}
