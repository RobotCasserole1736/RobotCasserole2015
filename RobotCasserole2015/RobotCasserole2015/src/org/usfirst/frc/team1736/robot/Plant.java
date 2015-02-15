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
	private double cur_elevator_height = ELEVATOR_MAX_HEIGHT_IN;
	private double cur_elevator_motor_effort = 0; // range -1 to 1 for motor control
	
	//arm state
	private boolean desired_arm_state = false; //false = retracted, true = extended
	private double cur_arm_extension_distance = 0;
	
	//drivetrain location state
	private double cur_motor_1_effort = 0; //range -1 to 1 for motor control
	private double cur_motor_2_effort = 0;
	private double cur_motor_3_effort = 0;
	
	private double cur_motor_1_encoder_distance = 0; //encoder distance readings
	private double cur_motor_2_encoder_distance = 0;
	private double cur_motor_3_encoder_distance = 0;
	
	private double cur_robot_x_pos = 0;
	private double cur_robot_y_pos = 0;
	private double cur_robot_pose_angle = 0; //in radians?
	
	//current sensor readings
	private boolean elevator_bottom_sensor_state = false;
	private boolean elevator_mid_sensor_state = false;
	private boolean elevator_top_sensor_state = false;
	
	private double time_of_last_call = System.nanoTime()/1000000000;
	
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
		desired_arm_state = true;
	}
	
	public void setArmsSolenoidIn(){
		desired_arm_state = false;
	}
	
	//Call every control loop to get updated values
	public void evalElevatorPlant(){
		double prev_time = time_of_last_call;
		time_of_last_call = System.nanoTime()/1000000000;
		double time_delta = prev_time - time_of_last_call;
		
		//adjust elevator height, within upper/lower limits
		if(cur_elevator_motor_effort > 0)
			cur_elevator_height += cur_elevator_motor_effort*ELEVATOR_MAX_DOWN_SPEED_IN_PER_SEC*time_delta;
		else if (cur_elevator_motor_effort < 0)
			cur_elevator_height += cur_elevator_motor_effort*ELEVATOR_MAX_UP_SPEED_IN_PER_SEC*time_delta;
		
		cur_elevator_height = limit_double(cur_elevator_height, ELEVATOR_MAX_HEIGHT_IN, ELEVATOR_MIN_HEIGHT_IN);
		
		//set elevator height sensors based on current height
		//sensors will trigger whenever the elevator is within the magnetic sensor's detection range
		if((cur_elevator_height < ELEVATOR_BOTTOM_SENSOR_POS_IN + MAG_SEN_DETECT_HALF_RANGE_IN) && (cur_elevator_height > ELEVATOR_BOTTOM_SENSOR_POS_IN - MAG_SEN_DETECT_HALF_RANGE_IN))
			elevator_bottom_sensor_state = true;
		else
			elevator_bottom_sensor_state = false;
		
		if((cur_elevator_height < ELEVATOR_MID_SENSOR_POS_IN + MAG_SEN_DETECT_HALF_RANGE_IN) && (cur_elevator_height > ELEVATOR_MID_SENSOR_POS_IN - MAG_SEN_DETECT_HALF_RANGE_IN))
			elevator_mid_sensor_state = true;
		else
			elevator_mid_sensor_state = false;
		
		if((cur_elevator_height < ELEVATOR_TOP_SENSOR_POS_IN + MAG_SEN_DETECT_HALF_RANGE_IN) && (cur_elevator_height > ELEVATOR_TOP_SENSOR_POS_IN - MAG_SEN_DETECT_HALF_RANGE_IN))
			elevator_top_sensor_state = true;
		else
			elevator_top_sensor_state = false;
		
		//extend or retract arm.s
		//pneumatic, so the only control is the boolean input
		if(desired_arm_state == true)
			cur_arm_extension_distance += ARM_MAX_EXTEND_SPEED_IN_PER_SEC*time_delta;
		else
			cur_arm_extension_distance += -1*ARM_MAX_RETRACT_SPEED_IN_PER_SEC*time_delta;
		
		cur_arm_extension_distance = limit_double(cur_arm_extension_distance, ARM_MAX_DISTANCE_IN, ARM_MIN_DISTANCE_IN);
		
		
		//check for broken robot
		if(cur_arm_extension_distance != ARM_MAX_DISTANCE_IN)
		
		
		
		
	}
	
	public void evalDrivetrainPlant(){
		double prev_time = time_of_last_call;
		time_of_last_call = System.nanoTime()/1000000000;
		double time_delta = prev_time - time_of_last_call;
		
		
		
		
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
