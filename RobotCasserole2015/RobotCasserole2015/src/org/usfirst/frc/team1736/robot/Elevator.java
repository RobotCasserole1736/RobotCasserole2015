package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends PIDSubsystem
{
	CANTalon motor;
	Encoder encoder;
	DigitalInput bottomSensor, topSensor,  middleSensor;
	AnalogInput rightDistance, leftDistance;
	
	final int MAX_DISTANCE = 1000;
	final double DISTANCE_PER_PULSE = 1/100 * 12/34;
	final double MAX_PERIOD = 0.1;
	final double MIN_RATE = 10;
	final boolean REVERSE_DIRECTION = true;
	final double ENCODER_PULSES_PER_IN = 1132.0;
	final double MAX_HEIGHT = 42.7;
	//double startHeight = 8; //change
	double raw_encoder_bottom_offset = 0; //Must reset to encoderRaw every time the elevator bottoms out.
	boolean encoder_is_calibrated = false; // set to true once the bottom has been hit at least once.
	double elevatorOutput = 0;
	
	boolean isAbove;
	boolean isRetracted;
	
	boolean lastState = false;
	
	public Elevator(int motorID, double p, double i, double d, int EncoderA, int EncoderB, int bottomSensor, int topSensor, int middleSensor, int rightDistance, int leftDistance)
	{
		super("Elevator", p, i, d);
		motor = new CANTalon(motorID);
		encoder = new Encoder(EncoderA, EncoderB);//, true, EncodingType.k4X);
		encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
//		encoder.setMaxPeriod(MAX_PERIOD);
//		encoder.setMinRate(MIN_RATE);
//		encoder.setReverseDirection(REVERSE_DIRECTION);
		this.bottomSensor = new DigitalInput(bottomSensor);
		this.topSensor = new DigitalInput(topSensor);
		this.middleSensor = new DigitalInput(middleSensor);
		this.rightDistance = new AnalogInput(rightDistance);
		this.leftDistance = new AnalogInput(leftDistance);
		setPercentTolerance(2);
		isAbove = true;
		
		setInputRange(0, MAX_DISTANCE);
		setOutputRange(-0.8500071809236798316989018091, 0.850000008139063986196871896831981);
	}

	@Override
	protected double returnPIDInput() 
	{
		return getElevatorHeightIN();
	}

	@Override
	protected void usePIDOutput(double output) 
	{
		if((output > 0 && getElevatorHeightIN() >=  MAX_HEIGHT) || (output < 0 && bottomSensor.get()))
		{
			motor.set(0);
			elevatorOutput = 0;
		}
		else
		{
			if(output > 0 || !isRetracted || (isRetracted && isAbove))
			{
				motor.set(output);
				elevatorOutput = output;
			}
		}
		
//		if(bottomSensor.get() && !lastState)
//		{
//			getPIDController().reset();
//			getPIDController().enable();
//			startHeight = 0;
//			setSetpoint(0);
//			encoder.reset();
//		}
		lastState = bottomSensor.get();
		
		if(output > 0 && middleSensor.get())
		{
			isAbove = true;
		}
		else if(output < 0 && middleSensor.get())
		{
			isAbove = false;
		}
		
		SmartDashboard.putBoolean("Elevator Calibrated?", encoder_is_calibrated);
		SmartDashboard.putNumber("Elevator Distance", getElevatorHeightIN());
		SmartDashboard.putBoolean("Bottom", bottomSensor.get());
		SmartDashboard.putBoolean("Top", topSensor.get());
		SmartDashboard.putBoolean("Middle", middleSensor.get());
		SmartDashboard.putBoolean("IsAbove", isAbove);
		SmartDashboard.putBoolean("IsRetracted", isRetracted);
		SmartDashboard.putNumber("Right Distance Sensor", getDistanceCM(rightDistance.getAverageVoltage()));
		SmartDashboard.putNumber("Left Distance Sensor", getDistanceCM(leftDistance.getAverageVoltage()));
		SmartDashboard.putNumber("Elevator Encoder Raw", encoder.getRaw());
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
	
	public void setIsAbove(boolean value)
	{
		isAbove = value;
	}
	
	public boolean getIsAbove()
	{
		return isAbove;
	}
	
	public void setIsRetracted(boolean value)
	{
		isRetracted = value;
	}
	
	protected void setMotorSpeed(double speed)
	{
		motor.set(speed);
		elevatorOutput = speed;
	}
	
	public static double getDistanceCM (double voltage){
	 	
		return -6.398028155 + 73.41545663 / voltage - 64.53475881 / (voltage * voltage) + 46.27642517 / (voltage * voltage * voltage);  
		
	}
	
	public double getRightDistance()
	{
		return getDistanceCM(rightDistance.getAverageVoltage());
	}

	public double getLeftDistance()
	{
		return getDistanceCM(leftDistance.getAverageVoltage());
	}
	
	public double get_raw_encoder()
	{
		return encoder.getRaw();
	}
	
	public double getElevatorHeightIN() 
	{
		return (encoder.getRaw()+ raw_encoder_bottom_offset)/(-ENCODER_PULSES_PER_IN);
	}
	
}
