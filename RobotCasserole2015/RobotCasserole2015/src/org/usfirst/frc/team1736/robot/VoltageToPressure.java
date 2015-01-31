package org.usfirst.frc.team1736.robot;

import edu.wpi.first.wplib.smartdashboard;

public class VoltageToPressure {

	
	
	public double PressureSensor(double voltage) {
		return (voltage - 0.5) * (75 / 2);
	}

	public double PressureOutput(double voltage){
		SmartDashboard.putNumber(“Pressure”, PressureSensor(voltage));
	}
	}