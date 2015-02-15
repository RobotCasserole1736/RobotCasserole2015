package org.usfirst.frc.team1736.robot;

public class Plant {
	
	//Plant parameters
	
	//elevator
	private final double ELEVATOR_MAX_HEIGHT_IN = 75;
	private final double ELEVATOR_MIN_HEIGHT_IN = 0;
	private final double ELEVATOR_BOTTOM_SENSOR_POS_IN = ELEVATOR_MIN_HEIGHT_IN;
	private final double ELEVATOR_MID_SENSOR_POS_IN = ELEVATOR_MIN_HEIGHT_IN + 7;
	private final double ELEVATOR_TOP_SENSOR_POS_IN = ELEVATOR_MAX_HEIGHT_IN;
	private final double ELEVATOR_MAX_UP_SPEED_IN_PER_SEC = 5;
	private final double ELEVATOR_MAX_DOWN_SPEED_IN_PER_SEC = 8;
	
	private final double ARM_MAX_DISTANCE_IN = 40;
	private final double ARM_MIN_DISTANCE_IN = 0;
	private final double ARM_MAX_EXTEND_SPEED_IN_PER_SEC = 40;
	private final double ARM_MAX_RETRACT_SPEED_IN_PER_SEC = 40;
	
	//Mag sensors
	private final double MAG_SEN_DETECT_HALF_RANGE_IN = 0.5;
	
	//Drivetrain
	private final double DRIVETRAIN_MOTOR_1_MAX_TRAVERSAL_SPEED_FT_PER_SEC = 5;
	private final double DRIVETRAIN_MOTOR_2_MAX_TRAVERSAL_SPEED_FT_PER_SEC = 5;
	private final double DRIVETRAIN_MOTOR_3_MAX_TRAVERSAL_SPEED_FT_PER_SEC = 5;
	private final double DRIVETRAIN_ROTATE_RATE_SCALER_M12 = 0.1; //converts the velocity difference between motors 1/2 into a rotation
	private final double DRIVETRAIN_ROTATE_RATE_SCALER_M3 = 0.05; //converts the velocity of motor 3 into a rotation
	
	//elevator state
	double cur_elevator_height = ELEVATOR_MAX_HEIGHT_IN;
	double cur_elevator_motor_effort = 0; // range -1 to 1 for motor control
	
	//arm state
	boolean desired_arm_state = false; //false = retracted, true = extended
	double cur_arm_extension_distance = 0;
	
	//drivetrain location state
	double cur_motor_1_effort = 0; //range -1 to 1 for motor control
	double cur_motor_2_effort = 0;
	double cur_motor_3_effort = 0;
	
	double cur_motor_1_encoder_distance = 0; //encoder distance readings
	double cur_motor_2_encoder_distance = 0;
	double cur_motor_3_encoder_distance = 0;
	
	double cur_robot_x_pos = 0;
	double cur_robot_y_pos = 0;
	double cur_robot_pose_angle = 0; //in radians?
	
	//current sensor readings
	boolean elevator_bottom_sensor_state = false;
	boolean elevator_mid_sensor_state = false;
	boolean elevator_top_sensor_state = false;
	
	
	Plant(){ //what do we need to do on init???

	}

	//Plant sensor feedback functions
	public double getPlantGyroAngle(){
		return cur_robot_pose_angle;
	}
	
	public double getPlantEncoder1DistanceIn(){
		return cur_motor_1_encoder_distance;
	}
	
	public double getPlantEncoder2DistanceIn(){
		return cur_motor_2_encoder_distance;
	}
	
	public double getPlantEncoder3DistanceIn(){
		return cur_motor_3_encoder_distance;
	}
	
	public double getPlantElevatorEncoderValueIn(){
		return cur_elevator_height;
	}
	
	public boolean getPlantElevatorBottomSwitchVal(){
		return elevator_bottom_sensor_state;
	}
	
	public boolean getPlantElevatorMidSwitchVal(){
		return elevator_mid_sensor_state;
	}
	
	public boolean getPlantElevatorTopSwitchVal(){
		return elevator_top_sensor_state;
	}
	
	//Plant motor and soleniod set functions
	public void setDrivetrainMotor1Value(double setval){
		cur_motor_1_effort = limit_double(setval, 1.0, -1.0);
	}
	
	public void setDrivetrainMotor2Value(double setval){
		cur_motor_2_effort = limit_double(setval, 1.0, -1.0);
	}

	public void setDrivetrainMotor3Value(double setval){
		cur_motor_3_effort = limit_double(setval, 1.0, -1.0);
	}
	
	public void setElevatorMotorValue(double setval){
		cur_elevator_motor_effort = limit_double(setval, 1.0, -1.0);	
	}
	
	public void setArmsSolenoidOut(){
	
	}
	
	public void setArmsSolenoidIn(){
		
	}
	
	public void setGrabberIn(){
		
	}
	
	public void setGrabberOut(){
		
	}
	
	
	//Call every control loop to get updated values
	public void evalElevatorPlant(){
		
	}
	
	public void evalDrivetrainPlant(){
		
	}
	
	private double limit_double(double in, double max, double min){
		if(in > max)
			return max;
		else if (in < min)
			return min;
		else 
			return in;
	}
}
