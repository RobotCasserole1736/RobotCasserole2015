package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class Elevator extends PIDSubsystem
{
	TalonSRX motor;
	Encoder encoder;
	DigitalInput bottomSensor, topSensor;
	
	final int MAX_DISTANCE = 1000;
	final double DISTANCE_PER_PULSE = 1/360 * 12/34;
	final double MAX_PERIOD = 0.1;
	final double MIN_RATE = 10;
	final boolean REVERSE_DIRECTION = true;
	final double startHeight = 2.0; //change
	
	boolean lastState = false;
	
	public Elevator(int motorID, double p, double i, double d, int EncoderA, int EncoderB, int bottomSensor, int topSensor)
	{
		super("Elevator", p, i, d);
		
		motor = new TalonSRX(motorID);
		encoder = new Encoder(EncoderA, EncoderB, true, EncodingType.k4X);
		encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
		encoder.setMaxPeriod(MAX_PERIOD);
		encoder.setMinRate(MIN_RATE);
		encoder.setReverseDirection(REVERSE_DIRECTION);
		this.bottomSensor = new DigitalInput(bottomSensor);
		this.topSensor = new DigitalInput(topSensor);
		
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
		if((output > 0 && topSensor.get()) || (output < 0 && bottomSensor.get()))
			motor.set(0);
		else
			motor.set(output);
		
		if(bottomSensor.get() && !lastState)
		{
			getPIDController().reset();
			getPIDController().enable();
			encoder.reset();
		}
		lastState = bottomSensor.get();
	}

	@Override
	protected void initDefaultCommand() 
	{
		// TODO Auto-generated method stub
		
	}
	
	public class ElevatorPositions
	{
		//change values
		final double Tote1Height = 1.0;
		final double Tote2Height = 2.0;
		final double Tote3Height = 3.0;
	}
	
}
