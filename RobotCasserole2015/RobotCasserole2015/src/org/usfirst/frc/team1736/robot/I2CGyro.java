///////////////////////////////////////////////////////////////////////////////
// Copyright (c) FRC Team 1736 2015. All Rights Reserved.
///////////////////////////////////////////////////////////////////////////////
//
// CLASS NAME: I2CGyro
// DESCRIPTION: I2C Gyro class - driver for L3G4200D MEMS Gyro
// 				Includes filtering and descrete integral for angle calculation.
//
// NOTES: Multithreaded support. Initializing the gyro kicks off a periodic
//        read function which asynchronously reads from they gyro in the
//        background. 
//     
//        The periodic update function is designed to be called by this 
//        background thread. Each time it is called, it gets the latest value
//        from the gyro, performs any filtering required, and integrates
//        the gyro value to calculate an angle. Class-private variables are 
//        updated by this function. Synchronized public-facing functions
//        are provided to read these variables' values.
//
//
//  CHANGE HISTORY:
//      Jan 19 2015 - Chris G. 
//         - Created
//
//
///////////////////////////////////////////////////////////////////////////////


package org.usfirst.frc.team1736.robot;
import java.util.Arrays; //For median filter sorting
import java.util.TimerTask; //For multithreading support

import edu.wpi.first.wpilibj.I2C; //FRC's internal I2C libraries for the RoboRio
import edu.wpi.first.wpilibj.I2C.Port;


public class I2CGyro {
	
	///////////////////////////////////////////////////////////////////////////////
	//Class & Multithread objects
	///////////////////////////////////////////////////////////////////////////////
	
	//wpilibj object for an i2c device
	//We will name it gyro since it will reference the gyro
	I2C gyro; 
	
	//Internal object required for multi-threading
	I2CGyro m_gyro; 
	
	// Thread to run the periodic update function on 
	java.util.Timer timerThread; 

	//period between periodic function calls
	//in milliseconds
	double m_sample_period_ms = 10; 
	
	///////////////////////////////////////////////////////////////////////////////
	//I2C constants
	///////////////////////////////////////////////////////////////////////////////
	// For a brief primer on what "I2C" is, check out SparkFun's tutorial:
	// https://learn.sparkfun.com/tutorials/i2c
	//
	// Also, reference the wikipedia page:
	// http://en.wikipedia.org/wiki/I%C2%B2C
	//
	// The cliffs-notes on what I2C is:
	//
	// I2C is a method of communicating between a processor and an auxilary device.
	// This communication is accomplished by changing voltages on a bus of wires.
	// Data is sent in "Serial" fashion (each 1/0 bit is sent one after another).
	// Any I2C bus will have to consist of a Clock (SCL) line and a Data (SDA) line.
	// There will also be power supply lines (Vcc and Gnd. Vcc = 3.3V for our gyro).
	// All devices that are part of the I2C bus are either "Masters" or "Slaves".
	// There is only one Master device. In our case, it is the RoboRio.
	// There can be many Slave devices. In our case, the only Slave is the gyro.
	// To "Control" either the Clock or Data lines is to apply a voltage to them.
	// Only the Master will control the Clock line.
	// The Master and Slaves take turns controlling the Data line.
	// The Master can only communicate with one Slave at a time.
	// To exchange data, the Master sends three things:
	//   -First, the device's 7-bit "Address"
	//   -Second, a single bit indicating "Read" or "Write"
	//   -Third, a memory address pointing to some location in the Slave's memory registers.
	// All slaves sit idle until they hear their own address. At this point, they
	//    start listening to the next things on the bus.
	// If the Master desires to Write to the Slave, the Master will put more 8-bit 
	//    packets onto the data bus. The slave will listen for these packets, 
	//    and write them into its own memory registers at the specified locations.
	// If the master desires to Read from the Slave, the Master will wait until the
	//    slave acknowledges that the registers are ready to produce data. 
	//    -Then, the master controls the Clock line, and the Slave writes data bits
	//     onto the Data line. 
	//    -The Master listens for these data bits and stores them.
	// Although not technically required, the Slave device is almost always set up to
	//    act like a piece of RAM. The master reads from and writes to the RAM.
	// You must look at the Datasheet of the slave chip in order to figure out 
	//    what each register address in the slave device represents.
	// The registers will usually represent things like configuration (eg, write "10001000
	//    to enable the device and set its sample rate") or important data
	//    (eg, "the current rotational velocity in radians/sec")
	// Our Gyro's datasheet can be found at:
	//    http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Gyros/3-Axis/CD00265057.pdf
	// READ THE DATASHEET!!!! ALL OF IT!!!!! ALWAYS!!!!!1!!!!
	//
	///////////////////////////////////////////////////////////////////////////////
	
	//I2C device address - the address that the gyro will respond to
	//Our Gyro actually can have two addresses, you switch between them
	// by applying Vcc or Gnd to the "SDO" pin on the gyro breakout board.
	// This is so you could have two gyros on the same I2C line, and be
	// able to talk to the independently.
	//This address assumes our gyro has SDO tied to Gnd.
	private static final int I2C_ADDR = 0b01101000; 
	
	//Gyro internal register addresses
	//These are the addreses of data that we care about
	//It is not a complete list, only the ones we care about.
	private static final int WHOAMI_REG_ADDR = 0x0F;
	private static final int CTRL_REG1_ADDR = 0x20;
	private static final int CTRL_REG2_ADDR = 0x21;
	private static final int CTRL_REG3_ADDR = 0x22;
	private static final int CTRL_REG4_ADDR = 0x23;
	private static final int CTRL_REG5_ADDR = 0x24;
	private static final int OUTZ_L_REG_ADDR = 0x2C;
	private static final int OUTZ_H_REG_ADDR = 0x2D;
	
	//When reading multiple bytes from the gyro, you have
	//the option of causing the pointer into the gyro's memory
	//registers to "auto-increment". This means that after each
	//read, the pointer will be moved to point to the next register.
	//This means that the communication can occur almost twice as fast.
	//It is only useful when reading consecutive registers in a short
	//  period of time.
	//Without auto-increment, you would have to send 
	// "mem_addr, read, next mem_addr, read, next mem_addr, read," etc...
	//But with auto-increment, you can instead just send
	// "mem_addr, read, read, read, read, ..."
	//See? You get your reads faster!
	//In order to use this variable, bitwise OR it with the first register
	//you want to read from, and then read multiple bytes.
	private static final int AUTO_INCRIMENT_REG_PTR_MASK = 0x80;
	
	//Expected contents of the WHOAMI register
	//The WHOAMI (who am I?) register contains some magic number
	//that is always the same. This is useful because upon first connecting
	//with the gyro, we can read this register, and compare it to this
	//expected value. If they match, we can be very confident that we are
	//actually talking with the gyro, and not some other device that 
	//got hooked up by accident.
	private static final byte WHOAMI_EXPECTED = (byte) 0b11010011;
	
	///////////////////////////////////////////////////////////////////////////////
	//Internal Calculations objects
	///////////////////////////////////////////////////////////////////////////////
	
	//The current angle, as measured by the gyroscope
	//'Volatile' because it gets touched by multiple threads asynchronously
	// Volatile will tell the processor that every time it needs to use this
	// variable, it MUST go fetch it from main system memory, and not use
	// a cached value (since another thread may have updated it since the
	// cached value was updated)
	private volatile double angle;
	
	//Buffer for current and previous gyro values
	//[0] is current, [1] is the previous value, [2] is the previous previous value....
	//It is required that we store the last three gyro readings for our integration 
	// method. This will be discussed in greater detail later on.
	private volatile double[] gyro_z_val_deg_per_sec = {0, 0, 0};
	
	
	//When we start up the robot, the gyro might not actually read zero
	// when not moving. To estimate what this "stationary reading offset" is,
	// we will just read the gyro a poop-ton of times right at startup. 
	//We assume since this occurs within a few seconds of turning on the
	// robot and before autonomous or teleop starts, the robot will be still.
	//Since the robot is still, we just average all the readings we take at startup
	// and call that the "stationary reading offset". This constant defines
	// how many readings to take. More will make the zero-offset more accurate
	// but take longer at startup.
	public static final int GYRO_INIT_READS = 500;
	
	//Nominal Z value
	//This is populated at startup with the average value of all the reads we
	// take from the gyro.
	public short zero_motion_offset;
	
	//Max range before we declare the gyro overloaded
	//It's just an arbitrary limit we set before we spit
	//out warnings to the console to warn the developer
	//the angle is no longer accurate, due to the output of the 
	//gyro saturating.
	private final short gyro_max_limit = 0x7FB0; //directly in bits, same scale as gyro's registers
	
	//Deadzone - if gyro reading is less than this, assume it's actually zero.
	//This needs to be very small, as even small gyro values will add up to the
	//angle. Making this more than zero will help reduce the effects of noise
	//in the gyro reading, but also technically reduces angle accuracy by a lot
	//if this is too big. Trial and Error (and sweat, blood, and tears) determined that
	// 0.1 seems to work as a good deadzone.
	//If the gyro seems like it's drifting in angle measurement, increase this a bit. If the 
	//angle is not correct during rotation, reduce this.
	public static final double gyro_deadzone = 0.1; //(in deg/sec)
	
	//Conversion factor from bits to degrees per sec
	//The gyro has only 16 bits to represent the rotational velocity.
	//But you can configure the sensitivity of the gyro to measure 
	// up to 200, 500, or 2000 degrees/sec. The gyro attempts to map
	// these ranges to the 16 bits. Remember that a 16 bit number can only
	// represent 2^16 different things, so increasing the deg/sec setting
	// makes you have a coarser granularity per bit.
	//
	//None of that is really important cuz we NEED the gyro fixed at 2000 deg/s
	//
	//So don't worry about that first paragraph. yah...
	//
	//This 0.07 number is just a constant that comes from the sensor datasheet.
	//It tells you how to convert from a sensor reading in raw bits to a
	// meaningful physical value.
	private static final double degPerSecPerLSB = 0.07;
	
	
	//To accurately determine the angle, we need to know precisely how much time
	// elapsed from the last time we read the gyro to the current time we read the gyro.
	//We will use the built in java System.nanoTime(), which returns
	// the time the program has been running in nanoseconds (very accurate! yay!).
	//We will mark the time at the start of each periodic loop, and store the previous
	//start time in this variable. To figure out the time between samples, we just subtract
	//the previous time reading from the current one. This variable will store the old value.
	private long system_time_at_last_call = 0;
	
	//To calculate the integral, we have a small state machine. In order to
	//make sure the state machine gets properly initalize, we will need to 
	//run some special code the first time the periodic update funciton is called.
	//This variable keeps track of whether we've called the code or not yet.
	//It starts false, and then gets set to true as soon as we've run the periodic
	//code once. It stays true until the software is restarted.
	private boolean periodic_called_once = false;
	
	//Gyro integration mode: 0 = linear interpolation, 1 = simpson's method
	//
	// "Integration" is a fancy schmancy calculus thing.
	// Integration is a fancy schmancy way of saying "summation"
	// Since this is a computer, we only deal with descrete time points.
	// Here's an easy way to visualize an integral:
	// Imagine we have some function of time (say, our gyro angular velocity, changing over time).
	// Here is some "artistic" ascii art of this:
	//
	//   (angular velocity in deg/s)
	//   ^
	//300|
	//   |
	//   |
	//   |
	//200|
	//   |        -------------
	//   |       /              \
	//   |      /                \
	//   |     /                  \
	//100|-----                    \
	//   |                          \
	//   |                           `~~~~~~~~~~~~~~~~~~~~
	//   |
	//   +-----+-------+-------+-------+-------+-------+------> (time (sec))
	//         1       2       3       4       5       6
	//
	// Notice how the angular velocity starts off at about 100 deg/s, goes up to like 180ish, 
	// then back down to 50, more or less.
	//
	// For reasons that are remarkably uninteresting, unless you freakin love math, 
	// the "Integral of gyro angular velocity" over some period of time is equal
	// to the area of the space bounded by the time axis, the curve, and the range of
	// time in question.
	// 
	// For example, given the above curve, the Integral of our gyro angular velocity
	// from 1 second to 2 seconds can be calculated by figuring out the area 
	// "shaded in" with the character "%":
	//
	//   (angular velocity in deg/s)
	//   ^
	//300|
	//   |
	//   |
	//   |
	//200|
	//   |        -------------
	//   |       /%%%%%%        \
	//   |      /%%%%%%%         \
	//   |     /%%%%%%%%          \
	//100|-----%%%%%%%%%           \
	//   |     %%%%%%%%%            \
	//   |     %%%%%%%%%             `~~~~~~~~~~~~~~~~~~~~
	//   |     %%%%%%%%% 
	//   +-----+-------+-------+-------+-------+-------+------> (time (sec))
	//         1       2       3       4       5       6
	//
	// Judging roughly by the numbers involved, the height of that shaded area is about
	// 180ish, and its width is 1, so the integral is around 180. This is how you calculate
	// integrals.
	//
	// Because physics, the Integral of Angular Velocity is Angle.
	//
	// We read angular velocity at regular intervals from the gyroscope.
	// Getting those values is the multithreaded part of this software is supposed to do.
	// Then we must integrate them to figure out the angle.
	//
	// There is one problem, probably one you won't deal with in your Calc 1 course.
	//
	// The issue is that when we read the values from the gyro, we only know the angular
	// velocity at discrete, non-continuous points in time. We must assume something about what
	// the angular velocity does IN BETWEEN our readings in order to actually calculate an integral
	//
	//   (angular velocity in deg/s)
	//   ^
	//300|
	//   |
	//   |
	//   |
	//200|     0 <- Known sample from gyro
	//   |     |
	//   |     |
	//   |     |
	//   |     |         <what happens here???>
	//100|     |       
	//   |     |
	//   |     |                                       0 <- Known sample from gyro
	//   |     |                                       |
	//   +-----+---------------------------------------+------> (time (samples))
	//         1                                       2
	// There are a number of ways to do this. 
	//
	// The easiest thing to do is assume that between our samples, the velocity just goes
	// in a straight line from point to point. We forget for a moment what the line actually
	// looks like, and just assume it's a line. This produces some nice results:
	//
	// This means that the integral between any two points is just the area of a 
	// trapezoid. This area is just (height_1 + height_2)/2 * base.
	// ---Base = time between samples, height_1 = prev sample, height_2 = current sample
	//
	//   (angular velocity in deg/s)
	//   ^
	//300|
	//   |
	//   |
	//   |
	//200|     0 <- Known sample from gyro
	//   |     |\
	//   |     | \
	//   |     |  \  <-assumed linear line between samples
	//   |     |   \     
	//100|     |    \   
	//   |     |int- \
	//   |     |egral 0 <- Known sample from gyro
	//   |     |here  |
	//   +-----+------+------> (time (samples))
	//         1      2     
	//
	// If we do this for every new sample and keep adding up the integrals, we will
	// ideally arrive at the current angle. Note this is a summation. Aka an integrall.
	// 
	// Integral = summation of things. In this case, trapezoids.
	//
	// However, we can probably do better. If we have THREE points, we can instead assume
	// that a PARABOLA exists between the three samples. As you may or may not know, this is 
	// possible because any three points can be used to define a parabola. 
	//
	//
	//   (angular velocity in deg/s)
	//   ^
	//300|
	//   |
	//   |
	//   |
	//200|     0 <- Known samples from gyro
	//   |     |\          |
	//   |     | |         |         0 <- Known sample from gyro
	//   |     |  \        |       / |
	//   |     |   \       |      /  |
	//100|     |    --     V    --   |  <- Assumed parabola between three samples
	//   |     |      \        /     |
	//   |     |       ----0---      |      
	//   |     | Integral  |   here  |
	//   +-----+-----------+---------+------> (time (samples))
	//         1           2         3
	//
	// That is some reall confusing ascii art. None the less.
	// The bottom line is that the geometry gets really intense after this to calculate area.
	// Unless you freakin love math, don't bother to do it yourself. Look it up on wikipedia instead!
	//
	// Turns out some chump named "Simpson" (Homer? maybe?) did the math already for you, and determined
	// that the are under the curve is calculated by a handy formula:
	//
	// Integral = (time between sample 1 and 3)/6 * (sample_1 + 4*sample_2 + sample_3)
	//
	// The disadvantage here is that after each integral calculation, we need to wait for two new samples
	// before we can do another integral calculation. This might slow down the reading. However, for our high sample rate
	// we'll probably be fine.
	// Addionally, Simpson's method assumes that the time between sample_1 and sample_2 is about the same as the time
	// between sample_2 and sample_3. If this is not true, Simpson's method produces lots of error :(
	//
	// In this case, our integration is still a sum. Except here, it's a sum of small regions where
	// one boundary happens to be a parabola.
	//
	// You can switch between these two implementations by choosing 1 or 0 for the below variable. 
	// 1 (Simpson's method) seems to be more accurate, so I'd keep it there for now unless the reading
	// lags too much.
	//
	private static final int integration_method = 1;
	
	//state for calculating simpsons method of numerical integration.
	//Since we need three samples, this keeps track of state in our really simple state machine that
	//ensures we have three samples before calculating the integral
	private volatile int cur_interrupt_state = 0;
	//0 means no data in buffers yet
	//1 means 1 datapoint in buffer.
	//2 means two pieces of data in buffer. Add a third and calculate output!
	
	
	///////////////////////////////////////////////////////////////////////////////
	//FIR Filter Calculations objects
	///////////////////////////////////////////////////////////////////////////////
	//
	// FIR Filters are something every ECE major learns about sometime during
	// their sophomore year of college. They teach you the digital version late on
	// too. To figure out why they do what they do takes a lot of math.
	//
	// I don't like math. So I'm not going to bother telling you about it in these
	// already absurdly long comments.
	//
	// Even the name is nasty. "FIR" means Finite Impulse Response, which is a 
	// whole graduate level course unto itself to explain what it means. 
	//
	// The happy conclusion to months of math is that if you take a weighted
	// average of some number of previous digital samples, you can cut out
	// some of the noise in your input signal. I'll try to focus on how
	// you efficently calculate that weighted average.
	//
	
	//This constant defines how many previous samples we will average.
	//The fancy-schmancy electrical engineering term for this number of samples
	//is "Filter Length"
	private static final int FILTER_LENGTH = 53;
	
	//We assume that our noise causes the signal to change really fast (high frequency)
	//and the signal we care about changes relatively slowly (low frequency)
	
	//These are the coefficents of the filter.
	//"Coefficents" is a fancy-schmancy term for the weights in the weighted average.
	//Notice how they're symmetric (first and last are the same, etc.). There's more math behind that!
	//
	//You have to pick these nubmbers very carefully to eliminate the high frequencies you don't want
	// but not damage the low frequencies you do want.
	//
	// And by "pick carefully" what I mean is go to some website like http://t-filter.appspot.com/fir/index.html
	// and put in the parameters you want for the filter
	// and copy and paste the numbers it gives you. 
	//
	// How does that website work you ask? Answer: Math.
	//
	//Anyhoo, these constants I got are for:
    //
	//53 tap FIR, sampling frequency of 50Hz
	//passband from 0 to 10 Hz, 4.00db ripple, unit gain
	//stopband from 12 to 25 Hz, -40.23db ripple, ideally zero gain
	private static final double[] FILTER_COEFS = {
		0.0023092487065988227,
		0.014096933201830508,
		0.01882863576981003,
		0.02638335286474602,
		0.029521978086505703,
		0.027134360076108553,
		0.018569615380437842,
		0.005598205417752613,
		-0.008101003513074865,
		-0.01797433887398908,
		-0.020258649100198693,
		-0.013651315575190996,
		-0.00020833142072014616,
		0.014965417634094582,
		0.025336496823333894,
		0.025436882849386642,
		0.01331851701527296,
		-0.007998829315667583,
		-0.03080398758687758,
		-0.044851058492634356,
		-0.040775023377656615,
		-0.013618934982137149,
		0.03477631720424106,
		0.09551396923065877,
		0.1548091828592016,
		0.19794181683885595,
		0.2137118638623725,
		0.19794181683885595,
		0.1548091828592016,
		0.09551396923065877,
		0.034776317204241076,
		-0.013618934982137149,
		-0.040775023377656615,
		-0.044851058492634356,
		-0.03080398758687758,
		-0.007998829315667583,
		0.01331851701527296,
		0.025436882849386642,
		0.025336496823333894,
		0.014965417634094582,
		-0.0002083314207201463,
		-0.013651315575190996,
		-0.020258649100198693,
		-0.017974338873989075,
		-0.008101003513074865,
		0.005598205417752613,
		0.018569615380437842,
		0.027134360076108553,
		0.029521978086505696,
		0.026383352864746017,
		0.01882863576981003,
		0.014096933201830508,
		0.0023092487065988227,
	};
	
	//To calculate the weighted average, we will use a "circular buffer" to hold the previous values of the gyroscope.
	//A circular buffer is a fancy schmancy term for an array where, when you try to index past one end of the array, it
	//just loops back to the opposite end. This is useful, for reasons we will see later.
	private double[] filter_circ_buffer = new double[FILTER_LENGTH];
	
	//Now, each time we read the gyro, we will want to place the new gyro value into the circular buffer
	//The newest value from the gyro will replace the oldest value in the buffer
	//and all other values will shift accordingly.
	//There is a processor efficency problem here:
	//If we were using a standard buffer, we would have to have a for loop to shift each value
	//within the buffer to the next location. This operation takes longer, depending on how
	//long the buffer is. This is generally undeseriable, especially considering the filter output
	//value is calculated very frequency. The more efficent we make this, the better.
	//
	//So, to do this, we introduce the "Circular buffer". It's like a queue. However think of it like this:
	// In a standard queue (waiting line), when we want to add someone new to it, we put them in the back of the line, and 
	// make everyone else shift one spot (discarding the oldest guy). But what if that waiting line was actually shaped like
	// a circle, and we just stuck a flag next to the person who was in the front of the line? That way, every time we have 
	// a new person to put into the line, we just move the "front of the line" flag toward the person who has been waiting 
	// the longest, kick the out of the line, and put the new guy by that flag. Now, the guy who's right next to the flag
	// is the newest to the line, the guy behind him is the second newest, and the guy in front of him is actually the oldest.
	//
	// Hopefully that made some sense. Anyway, that's what we're doing here. This next variable just represents that little
	// "Front of the line" flag. It's the index in the circular buffer that is the newest sample.
	private int filter_buf_start_pointer = 0;
	
	///////////////////////////////////////////////////////////////////////////////
	//Median Filter Calculations objects
	///////////////////////////////////////////////////////////////////////////////
	//An alternate to a FIR filter is the Median filter.
	//A median filter is good for when you usually get accurate samples from the gyro,
	//but occasionally there's a value or two which are really off.
	//
	//In the world of ECE, we call this "Shot noise". Not that you need to know that.
	//But if someone asks, the technically correct answer is that median filters are good
	// for getting rid of shot noise.
	//
	// Anyhoo.
	//
	// Median filters are pretty simple. Just store the previous X samples from the gyro,
	// and return the median (not average) of all those values. If you recall what the median
	// is, it's pretty easy to see why outliers on your input will never go to the output of a median filter.
	//
	//It's also nicer cuz the only thing to tune on the filter is "X", or how many previous samples 
	// you consider for your median calcualtion. That's this variable:
	//Median filter length
	private static final int MED_FILT_LEN= 10;
	
	//I'm assuming that MED_FILT_LEN is fairly small, so I didn't bother with a circular buffer on this one.
	//Although, it's perfectly reasonable to use one here.
	private double[] med_filt_buffer = new double[MED_FILT_LEN];
	
	//That's all the data! On to the constructor!
	
	
	///////////////////////////////////////////////////////////////////////////////
	//Gyro Constructor
	///////////////////////////////////////////////////////////////////////////////
	
	//Constructor initalizes the data associated with the gyro, starts talking to it over I2C
	//sets inital register values
	//reads from the gyro a few times to figure out the zero-offset value
	//sets the zero-offset value
	I2CGyro(){ 
		byte[] rx_byte = {0}; //temp variable to store a byte to transmit over I2C
		
		filter_buf_start_pointer = 0; //initalize the poitner to an aribtrary location. I like zero. It's circular, so it doesn't actually matter.
		
		//Call the wpilib's functions to define a new I2C port, using the onboard roboRIO one.
		//It's the one labeled "I2C" and has a diagram, not the one on the expansion port.
		gyro = new I2C(Port.kOnboard, I2C_ADDR); 
		 
		//Validate that we are actually plugged into a gyro by
		//reading the whoami register, and comparing to the expected value
		gyro.read(WHOAMI_REG_ADDR, 1, rx_byte);
		if(WHOAMI_EXPECTED != rx_byte[0]){
			System.out.println("WARNING: WhoAmI register mismatch for Gyro!");
		}
		
		//Control register setup
		//See the datasheet, pages 29-43 to see exactly what each bit is doing.
		//The basic gist of what is accomplished with each write is specified below
		//but the datasheet will give you specifics on how it's accomplished.
		
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
		//probably not actually required, but doesn't hurt either
		try {
			Thread.sleep(10); //in ms
		} catch (InterruptedException e) { //java requires us to catch exceptions, don't know why exactly...
			System.out.println("ERROR YOU INTERRUPTED ME WHILE I'm SLEEPING!!! DO NOT WAKE THE ANGRY BEAST!!!!1!!!");
			e.printStackTrace();
		}
		
		//Read from the gyro a few times to get the zero-movement offset.
		//Be sure the robot doesn't get touched during this, or the gyro will
		//always have the wrong angle!!!
		System.out.print("Calibrating gyro, do not touch robot...");
		zero_motion_offset = 0;
		double gyro_zero_read_accumulator = 0;
		for(int i = 0; i < GYRO_INIT_READS; i++){
			gyro_zero_read_accumulator += (double)read_gyro_z_reg(); //add up all the reads
			try {
				Thread.sleep(10); //pause for 10ms between each read.
			} catch (InterruptedException e) {
				System.out.println("ERROR YOU INTERRUPTED ME WHILE I'm SLEEPING!!!");
				e.printStackTrace();
			}
		}
		//Calculate the average of all the reads by dividing their total sum by the number of reads
		zero_motion_offset = (short)((double)gyro_zero_read_accumulator/(double)GYRO_INIT_READS);
		System.out.println("Done! \nDetermined a zero-offset of " + zero_motion_offset); 
		
		//Kick off the multi-threaded stuff.
		//Will start calling the periodic update function at an interval of m_sample_period_ms,
		//asynchronously from any other code.
		//Java magic here, don't touch!
        timerThread = new java.util.Timer();
        timerThread.schedule(new GyroTask(this), 0L, (long) (m_sample_period_ms));
		
	}
	
	///////////////////////////////////////////////////////////////////////////////
	//Public methods
	///////////////////////////////////////////////////////////////////////////////
	
	//Must be synchronized due to multi-threaded stuff
	
	//Returns the most recent gyro reading in degrees per second
	public synchronized double get_gyro_z(){
		return gyro_z_val_deg_per_sec[0];
	}
	
	//returns the most recently calculated gyro angle in degrees
	//Angle can vary between -Infinity and Infinity, you must wrap this
	// to 0-360 if desired
	public synchronized double get_gyro_angle(){
		return angle;
		
	}
	
	//Resets the current angle of the gyro to zero
	public synchronized void reset_gyro_angle(){
		angle = 0;		
	}
	
	///////////////////////////////////////////////////////////////////////////////
	//Private Methods
	///////////////////////////////////////////////////////////////////////////////
	
	//Initiates a request over I2C to get the Z-rotation data from teh gyro.
	//Assumes all I2C communication initialization has been already done.
	private synchronized short read_gyro_z_reg(){
		byte[] buffer_low_and_high_bytes = {0, 0}; //buffer for I2C to read into
		//read high and low bytes from I2C
		gyro.read(OUTZ_L_REG_ADDR|AUTO_INCRIMENT_REG_PTR_MASK, 2, buffer_low_and_high_bytes); 
		//The desired 16-bit reading is split into two eight-bit bytes in memory and over I2C com's. This line just recombines those
		//two bytes into a 16bit number, and applies the zero-motion-offset.
		//Typecasting magic here, don't touch!
		short ret_val = (short)(((buffer_low_and_high_bytes[1] << 8) | (buffer_low_and_high_bytes[0] & 0xFF)) - (short)zero_motion_offset);
		//Detect if the gyro has exceeded its measurement range
		//If so, print a debugging message.
		if(ret_val > gyro_max_limit || ret_val < -gyro_max_limit)
			System.out.println("!!!!!WARNING GYRO VALUE HAS OVERLOADED!!!!!!!!!!");
		return (ret_val); //return assembled 16-bit result
		
	}
	
	//This function is called at regular intervals by the multithreaded part of the gyro code.
	//It commands a gyro read, scales the raw data to degrees/sec, and then calculates the current angle
	// using the desired integration method
	@SuppressWarnings("unused") //suppress compiler warnings because I swear, we do actually use this function.
	public synchronized void periodic_update() {
		long cur_period_start_time = System.nanoTime(); //Record the time the current sample is being taken at.
		//shift existing values
    	gyro_z_val_deg_per_sec[2] = gyro_z_val_deg_per_sec[1]; //note we discard the oldest sample
    	gyro_z_val_deg_per_sec[1] = gyro_z_val_deg_per_sec[0];
    	gyro_z_val_deg_per_sec[0] = gyro_median_filter((double)read_gyro_z_reg()*degPerSecPerLSB); //Read new value, scale, and add to gyro_z_vals array
    	//Apply deadzone to gyro reading
		if(gyro_z_val_deg_per_sec[0] < gyro_deadzone && gyro_z_val_deg_per_sec[0] > -gyro_deadzone)
			gyro_z_val_deg_per_sec[0] = 0;
	
		//If we're using simpsons method....
		if(integration_method == 1) {
		    if(cur_interrupt_state == 0) { //initalize variables on first call of asynchronous function
		    	system_time_at_last_call = cur_period_start_time; //record First sample time
		    	cur_interrupt_state = 1;
		    }
		    else if(cur_interrupt_state == 1) { //Wait for another sample
		    	cur_interrupt_state = 2;
		    }
		    else if(cur_interrupt_state == 2) { //We've got three samples, go ahead and calculate!
		    	long cur_period_ns = (cur_period_start_time - system_time_at_last_call); //Calculate the amount of time since the last time we calculated an integral
		    	//Integrate using simpsons method
				angle = angle + (double)cur_period_ns/(double)1000000000 * 1/6 * (gyro_z_val_deg_per_sec[2] + 4*gyro_z_val_deg_per_sec[1] + gyro_z_val_deg_per_sec[0]); //simpson's method
				//record current time for usage the next time we try to integrate
				system_time_at_last_call = cur_period_start_time;
				//move the state machine along.
				cur_interrupt_state = 1; 
		    }
		}
		//But if we're using a linear method...
		else if(integration_method == 0) {
			long cur_period_ns = (cur_period_start_time - system_time_at_last_call); //Calculate the amount of time since the last time we calculated an integral
			angle = angle + (double)cur_period_ns/(double)1000000000 * 1/2 * (gyro_z_val_deg_per_sec[0] + gyro_z_val_deg_per_sec[1]); //calculate integral using linear method
			system_time_at_last_call = cur_period_start_time; //record the current sample time for usage next time we calcualte an integral
		}
	}
	
	//Lowpass filter for gyro.
	//Shifts a new value into the circular buffer
	//outputs the current filter value (based on current and previous values given as input)
	private synchronized double gyro_LP_filter(double input){
		int circ_buffer_index = 0;
		double accumulator = 0;
		
		//Add the newest sample to the buffer, discarding the oldest sample
		filter_circ_buffer[filter_buf_start_pointer] = input;
		
		//iterate over the whole circular buffer
		//calculate the output as a weighted average of all the 
		//things currently in the buffer
		for(int i = 0; i < FILTER_LENGTH; i++){
			//wrap the net pointer to the proper range.
			//This is what makes the buffer "circular"
			if((filter_buf_start_pointer - i) >= 0) 
				circ_buffer_index = (filter_buf_start_pointer - i) % FILTER_LENGTH;
			else
				circ_buffer_index = ((filter_buf_start_pointer - i) % FILTER_LENGTH)+FILTER_LENGTH;
			
			//Add up all of the current buffer values, multiplied by the approprate weight (coefficent)
			accumulator += filter_circ_buffer[circ_buffer_index]*FILTER_COEFS[i]; 

		}
		
		//move the "starting flag"
		filter_buf_start_pointer = (filter_buf_start_pointer + 1) % FILTER_LENGTH ; //shift buffer
		return accumulator; //return filter value
	}
	
	//returns the median of some values. Pretty straightworward, figure it out yourself.
	private synchronized double gyro_median_filter(double input){
		double[] sorted_array = new double[MED_FILT_LEN];
		
		//shift the buffer the really slow way.
		for(int i = MED_FILT_LEN-1; i > 0; i--) {
			med_filt_buffer[i] = med_filt_buffer[i-1];
			sorted_array[i] = med_filt_buffer[i-1];
		}
		med_filt_buffer[0] = input;
		sorted_array[0] = input;
		
		//sort the array
		Arrays.sort(sorted_array);
		
		//the median is either the middle value, or
		//the average of the two middle values in 
		//a sorted array.
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
	
	
	//Java multithreading magic. Do not touch.
	//Touching will incour the wrath of Cthulhu, god of java and gyros.
	//May the oceans of 1's and 0's rise to praise him.
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
