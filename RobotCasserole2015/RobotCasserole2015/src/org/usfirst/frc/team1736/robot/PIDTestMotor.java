package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

//Simple class for tuning a PID motor controller using SmartDashboard
//
public class PIDTestMotor extends PIDSubsystem {
	PowerDistributionPanel pd;
	SpeedController motor;
	int pdPort;
	int motorPort;
	double stallCurrent = 20; //Need to define this based on motor selected

	public PIDTestMotor(double p, double i, double d, int pdPort, int motorPort, double stallCurrent) {
		super("PIDTestMotor", p, i, d);
		pd = new PowerDistributionPanel();
		this.pdPort = pdPort;
		this.motorPort = motorPort;
		setInputRange(0, stallCurrent);
		setOutputRange(-1, 1);
		motor = new VictorSP(motorPort);
		this.stallCurrent = stallCurrent;
	}

	@Override
	protected double returnPIDInput() {
		return pd.getCurrent(pdPort);
	}

	@Override
	protected void usePIDOutput(double output) {
		motor.set(output);
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	public double getStallCurrent() {
		return stallCurrent;
	}

}
