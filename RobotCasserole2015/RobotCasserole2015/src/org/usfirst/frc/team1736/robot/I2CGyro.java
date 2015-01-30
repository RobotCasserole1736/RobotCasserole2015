//Copyright FRC Team 1736 2015. All Rights Reserved
//
// I2C Gyro class - driver for L3G4200D MEMS Gyro
// Includes filtering and descrete integral for angle calculation.

package org.usfirst.frc.team1736.robot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;


public class I2CGyro {
	I2C gyro; //wpilibj object for an i2c device
	
	//I2C device address
	private static final int I2C_ADDR = 0b01101000; //Assume SDO is grounded
	
	//Gyro internal register addresses
	private static final int WHOAMI_REG_ADDR = 0x0F;
	private static final int CTRL_REG1_ADDR = 0x20;
	private static final int CTRL_REG2_ADDR = 0x21;
	private static final int CTRL_REG3_ADDR = 0x22;
	private static final int CTRL_REG4_ADDR = 0x23;
	private static final int CTRL_REG5_ADDR = 0x24;
	private static final int TEMP_REG_ADDR = 0x26;
	private static final int OUTZ_L_REG_ADDR = 0x2C;
	private static final int OUTZ_H_REG_ADDR = 0x2D;
	private static final int STATUS_REG_ADDR = 0x27;
	private static final int AUTO_INCRIMENT_REG_PTR_MASK = 0x80;
	
	//Expected contents of the WHOAMI register
	private static final byte WHOAMI_EXPECTED = (byte) 0b11010011;
	
	//integrator result (current angle)
	private volatile double angle;
	
	//current angular velocity
	private volatile double gyro_z_val_deg_per_sec;
	
	//how frequently the gyro gets read
	public static final int GYRO_READ_PERIOD_MS = 20;
	
	//how many times we read the gyro at the start to figure out the zero-offset
	public static final int GYRO_INIT_READS = 50;
	
	//Nominal Z value
	public short zero_motion_offset;
	
	//Conversion factor from bits to degrees per sec
	//we hardcode max scale to be 500 degrees per second
	//full scale is a signed 16 bit number
	//so the conversion is 500/(MAX_INT_16)
	private static final double degPerSecPerLSB = 0.01525925;
	
	private Thread gyro_read_thread;
	
	
	//Constructor initalizes the data associated with the gyro, starts talking to it over I2C
	//sets inital register values
	//reads from the gyro a few times to figure out the zero-offset value
	//sets the zero-offset value
	I2CGyro(){ 
		byte[] rx_byte = {0};
		
		gyro = new I2C(Port.kOnboard, I2C_ADDR);
		
		//Validate that we are actually plugged into a gyro
		gyro.read(WHOAMI_REG_ADDR, 1, rx_byte);
		if(WHOAMI_EXPECTED != rx_byte[0]){
			System.out.println("WARNING: WhoAmI register mismatch for Gyro!");
		}
		
		//Control register setup
		
		//Enable all three axes, and disable power down.
		//Yes, this is one of these devices which defaults to powered off.
		//It's stupid.
		gyro.write(CTRL_REG1_ADDR, 0b00001111);
		
		//Disable onboard High Pass Filter
		gyro.write(CTRL_REG2_ADDR, 0b00000000);
		
		//Disable all interrupt trigger pins
		gyro.write(CTRL_REG3_ADDR, 0b0000000);
		
		//Set Full-Scale range to 500Deg/Sec
		gyro.write(CTRL_REG4_ADDR, 0b00010000);

		//Disable Onboard FIFO
		gyro.write(CTRL_REG5_ADDR, 0b0000000);
		
		//sleep breifly to ensure settings take effect
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			System.out.println("ERROR YOU INTERRUPTED ME WHILE I'm SLEEPING!!!");
			e.printStackTrace();
		}
		
		//Read from the gyro a few times to get the zero-movement offset.
		System.out.print("Calibrating gyro, do not touch robot...");
		zero_motion_offset = 0;
		double gyro_zero_read_accumulator = 0;
		for(int i = 0; i < GYRO_INIT_READS; i++){
			gyro_zero_read_accumulator += (double)read_gyro_z_reg();
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				System.out.println("ERROR YOU INTERRUPTED ME WHILE I'm SLEEPING!!!");
				e.printStackTrace();
			}
		}
		zero_motion_offset = (short)((double)gyro_zero_read_accumulator/(double)GYRO_INIT_READS);
		System.out.println("Done! \nDetermined a zero-offset of " + zero_motion_offset);
		
		
	}
	
	public double get_gyro_z(){
		return gyro_z_val_deg_per_sec;
	}
	
	//return the gyro angle (in degrees from 0 to 360)
	public double get_gyro_angle(){
		return angle;
		
	}
	
	//reset the current angle to zero
	public void reset_gyro_angle(){
		angle = 0;		
	}
	
	private short read_gyro_z_reg(){
		byte[] buffer_low_and_high_bytes = {0, 0}; //buffer for I2C to read into
		gyro.read(OUTZ_L_REG_ADDR|AUTO_INCRIMENT_REG_PTR_MASK, 2, buffer_low_and_high_bytes); //read high and low bytes.
		short ret_val = (short)(((buffer_low_and_high_bytes[1] << 8) | (buffer_low_and_high_bytes[0] & 0xFF)) - (short)zero_motion_offset);
		System.out.println(ret_val);
		return (ret_val); //assemble bytes into 16 bit result
		
	}
	

	public void periodic_update() { 
			gyro_z_val_deg_per_sec = (double)read_gyro_z_reg()*degPerSecPerLSB;
			angle = angle + (double)GYRO_READ_PERIOD_MS*gyro_z_val_deg_per_sec/1000;
	}

}
