package org.usfirst.frc.team1736.robot;

public class VoltageToDistance {
	VoltageToDistance(){
		
	}
	
	public static double VoltageInput (double voltage){
	 	
		return -6.398028155 + 73.41545663 / voltage - 64.53475881 / (voltage * voltage) + 46.27642517 / (voltage * voltage * voltage);  
		
	}

}
