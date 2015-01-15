package org.usfirst.frc.team1736.robot;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

//Simple class for tuning a PID motor controller using SmartDashboard
//
public class PIDTestMotor extends PIDSubsystem {
	PowerDistributionPanel pd;
	SpeedController motor;
	Encoder encoder;
	int pdPort;
	int motorPort;
	double maxSpeed = 500; //Need to define this based on motor selected
	double speedReadings[] = {0,0,0,0,0,0,0,0,0,0};
	LinkedList<Double> speedQueue = new LinkedList<Double>();
	
	public PIDTestMotor(double p, double i, double d, int motorPort, double maxSpeed) {
		super("PIDTestMotor", p, i, d);
		encoder = new Encoder(0, 1, true, EncodingType.k4X);
		encoder.setDistancePerPulse(1);
		this.motorPort = motorPort;
		setInputRange(0, maxSpeed);
		setOutputRange(0, 1);
		motor = new Talon(motorPort);
		this.maxSpeed = maxSpeed;
		for (int idx=0; idx<10; idx++)
		{
			speedQueue.offer(0.0);
		}
	}

	@Override
	protected double returnPIDInput() {
		speedQueue.removeFirst();
		speedQueue.offer(encoder.getRate());
		double sum = 0;
		for(int idx = 0; idx < speedQueue.size(); idx++)
		{
			sum += speedQueue.get(idx);
		}
		return sum/10;
	}

	@Override
	protected void usePIDOutput(double output) {
		motor.set(output);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public double getSpeed() {
		return encoder.getRate();
	}

}
