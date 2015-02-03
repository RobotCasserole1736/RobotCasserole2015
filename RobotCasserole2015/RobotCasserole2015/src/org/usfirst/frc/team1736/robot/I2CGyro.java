//Copyright FRC Team 1736 2015. All Rights Reserved
//
// I2C Gyro class - driver for L3G4200D MEMS Gyro
// Includes filtering and descrete integral for angle calculation.

package org.usfirst.frc.team1736.robot;
import java.util.Arrays;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;


public class I2CGyro {
	I2C gyro; //wpilibj object for an i2c device
	
	I2CGyro m_gyro;
	
	java.util.Timer m_controlLoop;

	double m_period_multithread = 10;
	
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
	
	//Buffer for current and previous gyro values
	//[0] is current, [1] is prev, [2] is prev. prev....
	private volatile double[] gyro_z_val_deg_per_sec = {0, 0, 0};
	
	//how many times we read the gyro at the start to figure out the zero-offset
	public static final int GYRO_INIT_READS = 500;
	
	//Nominal Z value
	public short zero_motion_offset;
	
	//Max range before we declare the gyro overloaded
	private final short gyro_max_limit = 0x7FF0;
	
	//Deadzone - if gyro reading is less than this, assume it's actually zero.
	public static final double gyro_deadzone = 0.1; //(in deg/sec)
	
	//Conversion factor from bits to degrees per sec
	//we hardcode max scale to be 2000 degrees per second
	//full scale is a signed 16 bit number
	//so the conversion is 2000/(MAX_INT_16)
	//However, the datasheet says 70mdps. so we use that.
	private static final double degPerSecPerLSB = 0.07;//2000.0/32767;
	
	private long system_time_at_last_call = 0;
	private boolean periodic_called_once = false;
	
	//Gyro integration mode: 0 = linear interpolation, 1 = simpson's method
	private static final int integration_method = 1;
	
	//state for calculating simpsons method of numerical integration
	private volatile int cur_interrupt_state = 0;
	//0 means no data in buffers yet
	//1 means 1 datapoint in buffer.
	//2 means two pieces of data in buffer. Add a third and calculate output!
	
	//Filter Constants
	private static final int FILTER_LENGTH = 27;
	
	//coefficents for a low-pass-filter
	//27 tap FIR, sampling frequency of 50Hz
	//passband from 0 to 10 Hz, 4.06db ripple, unit gain
	//stopband from 12 to 25 Hz, -40.23db ripple
	private static final double[] FILTER_COEFS = {
		0.008630882178726856,
		0.041305991054314525,
		0.05886136073477571,
		0.03686795552479373,
		-0.016953072341001792,
		-0.040853244115447225,
		-0.00033666281168864186,
		0.050871136082244466,
		0.026656021882141712,
		-0.0616917753456848,
		-0.08155386643224148,
		0.069639050952534,
		0.3096965081057878,
		0.427464797306881,
		0.3096965081057878,
		0.069639050952534,
		-0.08155386643224148,
		-0.0616917753456848,
		0.026656021882141712,
		0.050871136082244466,
		-0.00033666281168864186,
		-0.040853244115447225,
		-0.016953072341001792,
		0.03686795552479373,
		0.05886136073477571,
		0.041305991054314525,
		0.008630882178726856,	
	};
	
	//circular buffer to hold filter input
	private double[] filter_circ_buffer = new double[FILTER_LENGTH];
	//index to latest value in buffer
	//should be mathed modulo FILTER_LENGTH
	private int filter_buf_start_pointer = 0;
	
	
	//Median filter length
	private static final int MED_FILT_LEN= 10;
	private double[] med_filt_buffer = new double[MED_FILT_LEN];
	
	//Constructor initalizes the data associated with the gyro, starts talking to it over I2C
	//sets inital register values
	//reads from the gyro a few times to figure out the zero-offset value
	//sets the zero-offset value
	I2CGyro(){ 
		byte[] rx_byte = {0};
		
		filter_buf_start_pointer = 0;
		
		gyro = new I2C(Port.kOnboard, I2C_ADDR);
		
		//Validate that we are actually plugged into a gyro
		gyro.read(WHOAMI_REG_ADDR, 1, rx_byte);
		if(WHOAMI_EXPECTED != rx_byte[0]){
			System.out.println("WARNING: WhoAmI register mismatch for Gyro!");
		}
		
		//Control register setup
		
		//Set output data rate to 400 and LPF cutoff to 25Hz
		//Enable only Z axis, and disable power down.
		//Yes, this is one of these devices which defaults to powered off.
		//It's stupid.
		gyro.write(CTRL_REG1_ADDR, 0b10011100);
		
		//enable High Pass Filter, cutoff at 0.5 Hz
		gyro.write(CTRL_REG2_ADDR, 0b00100110);
		
		//Disable all interrupt trigger pins
		gyro.write(CTRL_REG3_ADDR, 0b00000000);
		
		//Set Full-Scale range to 2000Deg/Sec
		gyro.write(CTRL_REG4_ADDR, 0b00100000);

		//Disable Onboard FIFO
		gyro.write(CTRL_REG5_ADDR, 0b00000000);
		
		//sleep breifly to ensure settings take effect
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			System.out.println("ERROR YOU INTERRUPTED ME WHILE I'm SLEEPING!!! DO NOT WAKE THE ANGRY BEAST!!!!1!!!");
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
		
        m_controlLoop = new java.util.Timer();
        m_controlLoop.schedule(new GyroTask(this), 0L, (long) (m_period_multithread));
		
	}
	
	public synchronized double get_gyro_z(){
		return gyro_z_val_deg_per_sec[0];
	}
	
	//return the gyro angle (in degrees from 0 to 360)
	public synchronized double get_gyro_angle(){
		return angle;
		
	}
	
	//return the raw value from the gyro
	public synchronized double get_gyro_z_raw(){
		return (double)read_gyro_z_reg();
	}
	
	//reset the current angle to zero
	public synchronized void reset_gyro_angle(){
		angle = 0;		
	}
	
	private synchronized short read_gyro_z_reg(){
		byte[] buffer_low_and_high_bytes = {0, 0}; //buffer for I2C to read into
		gyro.read(OUTZ_L_REG_ADDR|AUTO_INCRIMENT_REG_PTR_MASK, 2, buffer_low_and_high_bytes); //read high and low bytes.
		short ret_val = (short)(((buffer_low_and_high_bytes[1] << 8) | (buffer_low_and_high_bytes[0] & 0xFF)) - (short)zero_motion_offset);
		if(ret_val > gyro_max_limit || ret_val < -gyro_max_limit)
			System.out.println("!!!!!WARNING GYRO VALUE HAS OVERLOADED!!!!!!!!!!");
		return (ret_val); //assemble bytes into 16 bit result
		
	}
	

	@SuppressWarnings("unused")
	public synchronized void periodic_update() {
		long cur_period_start_time = System.nanoTime();
		//shift existing values
    	gyro_z_val_deg_per_sec[2] = gyro_z_val_deg_per_sec[1];
    	gyro_z_val_deg_per_sec[1] = gyro_z_val_deg_per_sec[0];
    	gyro_z_val_deg_per_sec[0] = gyro_median_filter((double)read_gyro_z_reg()*degPerSecPerLSB);
    	//Apply deadzone
		if(gyro_z_val_deg_per_sec[0] < gyro_deadzone && gyro_z_val_deg_per_sec[0] > -gyro_deadzone)
			gyro_z_val_deg_per_sec[0] = 0;
	
		if(integration_method == 1) {
		    if(cur_interrupt_state == 0) { //initalize variables on first call of asynchronous function
		    	system_time_at_last_call = cur_period_start_time;
		    	cur_interrupt_state = 1;
		    }
		    else if(cur_interrupt_state == 1) {
		    	cur_interrupt_state = 2;
		    }
		    else if(cur_interrupt_state == 2) {
		    	long cur_period_ns = (cur_period_start_time - system_time_at_last_call);
				angle = angle + (double)cur_period_ns/(double)1000000000 * 1/6 * (gyro_z_val_deg_per_sec[2] + 4*gyro_z_val_deg_per_sec[1] + gyro_z_val_deg_per_sec[0]); //simpson's method
				system_time_at_last_call = cur_period_start_time;
				cur_interrupt_state = 1; 
		    }
		}
		else if(integration_method == 0) {
			long cur_period_ns = (cur_period_start_time - system_time_at_last_call);
			angle = angle + (double)cur_period_ns/(double)1000000000 * 1/2 * (gyro_z_val_deg_per_sec[0] + gyro_z_val_deg_per_sec[1]);
			system_time_at_last_call = cur_period_start_time;
		}
	}
	
	//Lowpass filter for gyro.
	//Shifts a new value into the circular buffer
	//outputs the current filter value	
	private synchronized double gyro_LP_filter(double input){
		int circ_buffer_index = 0;
		double accumulator = 0;
		filter_circ_buffer[filter_buf_start_pointer] = input;
		
		for(int i = 0; i < FILTER_LENGTH; i++){
			//Calculate this loop's filter coefficients
			if((filter_buf_start_pointer - i) >= 0)
				circ_buffer_index = (filter_buf_start_pointer - i) % FILTER_LENGTH;
			else
				circ_buffer_index = ((filter_buf_start_pointer - i) % FILTER_LENGTH)+FILTER_LENGTH;
			
			accumulator += filter_circ_buffer[circ_buffer_index]*FILTER_COEFS[i]; //add up filter buffer multiplied by coefficents

		}
		
		filter_buf_start_pointer = (filter_buf_start_pointer + 1) % FILTER_LENGTH ; //shift buffer
		return accumulator; //return filter value
	}
	
	//returns the median of some values
	private synchronized double gyro_median_filter(double input){
		double[] sorted_array = new double[MED_FILT_LEN];
		
		//shift the buffer the really slow way.
		for(int i = MED_FILT_LEN-1; i > 0; i--) {
			med_filt_buffer[i] = med_filt_buffer[i-1];
			sorted_array[i] = med_filt_buffer[i-1];
		}
		med_filt_buffer[0] = input;
		sorted_array[0] = input;
		
		Arrays.sort(sorted_array);
		
	    int middle = sorted_array.length / 2;
	    if (sorted_array.length % 2 == 0)
	    {
	      double left = sorted_array[middle - 1];
	      double right = sorted_array[middle];
	      return (left + right) / 2;
	    }
	    else
	    {
	      return sorted_array[middle];
	    }
	
	}
	

    private class GyroTask extends TimerTask 
    {
        private I2CGyro m_gyro;

        public GyroTask(I2CGyro gyro) 
        {
            if (gyro == null) 
            {
                throw new NullPointerException("Given PIDController was null");
            }
            m_gyro = gyro;
        }

        @Override
        public void run() 
        {
            m_gyro.periodic_update();
        }
    }

}
