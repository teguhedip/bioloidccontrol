/*
 * balance.c - functions for static balancing of the robot  
 *  uses the PID controller and Kalman filter if accelerometer is installed
 * 
 * Version 0.6		18/01/2013
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 */

/*
 * You may freely modify and share this code, as long as you keep this
 * notice intact. Licensed under the Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, this work is provided
 * without any warranty. It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

#include <stdio.h>
#include <math.h>
#include "global.h"
#include "pid.h"
#include "adc.h"
#include "balance.h"
#include "clock.h"

#define PI		3.14159265358979323846		// need this for trig calculations

// ADC related global variables 
extern volatile uint8 adc_sensor_enable[ADC_CHANNELS]; 
extern volatile int16 adc_sensor_val[ADC_CHANNELS]; 	// array of sensor values
extern volatile uint16 adc_gyrox_center;				// gyro x center value
extern volatile uint16 adc_gyroy_center;				// gyro y center value
extern volatile uint16 adc_accelx_center;				// accelerometer x center value
extern volatile uint16 adc_accely_center;				// accelerometer y center value

// Global variables related to the finite state machine that governs execution
extern volatile uint8 bioloid_command;			// current command
extern volatile uint8 last_bioloid_command;		// last command
// and also the current and next motion pages
extern volatile uint8 current_motion_page;
extern volatile uint8 next_motion_page;	
extern uint8 current_step;						// number of the current motion page step
// joint offset values
extern volatile int16 joint_offset[NUM_AX12_SERVOS];

// Input, Output and Setpoint variables for the PID controller (x and y-axis)
extern volatile double pid_input[PID_DIMENSION];
extern volatile double pid_output[PID_DIMENSION];
extern volatile double pid_setpoint[PID_DIMENSION];

// Modified Kalman code using Roll, Pitch, and Yaw from a Wii MotionPlus and X, Y, and Z accelerometers from a Nunchuck.
// Also uses "new" style Init to provide unencrypted data from the Nunchuck to avoid the XOR on each byte.
// Requires the WM+ and Nunchuck to be connected to Arduino pins D3/D4 using the schematic from Knuckles904 and johnnyonthespot
// See http://randomhacksofboredom.blogspot.com/2009/07/motion-plus-and-nunchuck-together-on.html
// Kalman Code By Tom Pycke. http://tom.pycke.be/mav/71/kalman-filtering-of-imu-data
// Original zipped source with very instructive comments: http://tom.pycke.be/file_download/4
//
// Some of this code came via contributions or influences by Adrian Carter, Knuckles904, evilBunny, Ed Simmons, & Jordi Munoz
//
// There are two coordinate systems at play here:
// The body-fixed coordinate system, which is fixed to the platform. That is, the x-axis is always pointing to the nose of the
// aircraft (nosecone on a rocket), the y-axis points to the right, and the z-axis points down on an airplane, (parallel to the
// earth away from you on a rocket).
//
// The aircraft will move with respect to the navigation frame, which is a fixed coordinate system.  
// In that frame, the x-axis points north, the y-axis points east and the z-axis points down. This is called the "NED system", or
// the "Local Tangent Plane" or the "Local Geodetic Frame".
// Created by Duckhead   v0.6   

struct GyroKalman
{
  /* These variables represent our state matrix x */
  double x_angle,
         x_bias;

  /* Our error covariance matrix */
  double P_00,
         P_01,
         P_10,
         P_11;

  /*
   * Q is a 2x2 matrix of the covariance. Because we
   * assume the gyro and accelerometer noise to be independent
   * of each other, the covariances on the / diagonal are 0.
   *
   * Covariance Q, the process noise, from the assumption
   *    x = F x + B u + w
   * with w having a normal distribution with covariance Q.
   * (covariance = E[ (X - E[X])*(X - E[X])' ]
   * We assume is linear with dt
   */
  double Q_angle, Q_gyro;

  /*
   * Covariance R, our observation noise (from the accelerometer)
   * Also assumed to be linear with dt
   */
  double R_angle;
};

//Precomputed constants to avoid floating point division at runtime (too many clock cycles)
static const double RadianToDegree = 57.2957795;    // 180/PI
static const double DegreeToRadian = 0.0174532925;  // PI/180

//The system clock function millis() returns a value in milliseconds.  We need it in seconds.
//Instead of dividing by 1000 we multiply by 0.001 for performance reasons.
static const double SecondsPerMillis = 0.001;

//WM+ variables
/*
 * R represents the measurement covariance noise.  In this case,
 * it is a 1x1 matrix that says that we expect 0.3 rad jitter
 * from the accelerometer.
 */
static const float	R_angle	= .3; //.3 default

/*
 * Q is a 2x2 matrix that represents the process covariance noise.
 * In this case, it indicates how much we trust the accelerometer
 * relative to the gyros.
 *
 * You should play with different values here as the effects are interesting.  Over prioritizing the
 * accelerometers results in fairly inaccurate results.
 */
static const double  Q_angle = 0.002; //(Kalman)
static const double  Q_gyro  = 0.1;   //(Kalman)

// Kalman data structures for each rotational axis
struct GyroKalman rollData;
struct GyroKalman pitchData;

// raw gyro readings
int readingsX;
int readingsY;

// normalized (scaled per WM+ mode) rotation values
int pitch = 0;
int roll = 0;
int pitchAngle = 10;
int rollAngle = 0;

// accelerometer variables
float accelAngleX = 0;	// X angle
float accelAngleY = 0;	// Y angle
float theta = 0;        // Angle of the z-axis to vertical 

// Internal program state variables
unsigned long lastread = 0; // last system clock in millis
int8 startup_counter = 0;

// internal function prototypes
void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle);
void processGyroKalman();
void predict(struct GyroKalman *kalman, double dotAngle, double dt); 
double update(struct GyroKalman *kalman, double angle_m);
void processAccelerometers();
float angleInRadians(int measured); 


// balances the robot to compensate for left-right and forward-backward tilt
// uses Extended Kalman filter to calculate attitude and PID controller for adjustments
void staticRobotBalance()
{
	int16 lr_knee_offset; 
	
	// calculate gyro deviations from center values
	processGyroKalman();							// takes about 820us
	
	// joint offsets - best left unadjusted based on experiments
	int pitch_adjusted = pitchAngle;
	int roll_adjusted = rollAngle;
	// TEST printf("\nKalman Adjustment - Pitch = %i, Roll = %i ", pitch_adjusted, roll_adjusted);

	// we need the kalman  filter to settle before we apply any adjustments
	// from experimenting it seems best to skip the first 35 iterations after executing the BAL command
	if( startup_counter <= 35 ) {
		startup_counter++;
		if( startup_counter == 35 ) printf("\nKalman - proceeding now.\n");
		return;
	}	
	
	// set the new PID input values
	pid_input[0] = (double) pitch_adjusted;		// 0 = x-axis
	pid_input[1] = (double) roll_adjusted;		// 1 = y-axis
		
	// run the PID controller
	if ( pid_compute() == 1 )
	{
		// recalculate joint offsets based on factor 3
		pitch_adjusted = (int16) pid_output[0];
		roll_adjusted  = (int16) pid_output[1];
		
		// TEST
		// printf("  Adjusted PID Output Pitch = %i, Roll = %i", pitch_adjusted, roll_adjusted);
		
		// got new values, adjust joint offsets
#ifdef HUMANOID_TYPEA	// Type A - all 18 servos are present and numbers match
		// forward-backward corrections
		joint_offset[15-1] = pitch_adjusted;		// right ankle (negative values move torso forward)
		joint_offset[16-1] = -pitch_adjusted;		// left ankle (positive values move torso forward)
		joint_offset[11-1] = -pitch_adjusted / 2;	// right hip (negative values lift leg up)
		joint_offset[12-1] = pitch_adjusted / 2;	// left hip (positive values lift leg up)
		// left-right corrections
		joint_offset[9-1]  = roll_adjusted;		// right hip (negative values bend torso to the right)
		joint_offset[10-1] = roll_adjusted;		// left hip (negative values bend torso to the right)
		joint_offset[17-1] = roll_adjusted;		// right ankle (negative values bend torso to the left)
		joint_offset[18-1] = roll_adjusted;		// left ankle (negative values bend torso to the left)
		lr_knee_offset = roll_adjusted / 3;		// left-right knee adjustment is 1/3 of ankle
		joint_offset[13-1] = -lr_knee_offset;	// right knee (negative values bend leg)
		joint_offset[14-1] = lr_knee_offset;	// left knee (positive values bend leg)
#endif
	}
}

/*
 * Initialize everything.
 */
void setupGyroKalman()
{
	initGyroKalman(&rollData, Q_angle, Q_gyro, R_angle);
	initGyroKalman(&pitchData, Q_angle, Q_gyro, R_angle);
	lastread = millis();
}

/*
 * Initialize the kalman structures.
 *
 * kalman    the kalman data structure
 * Q_angle   the process covariance noise for the accelerometers
 * Q_gyro    the process covariance noise for the gyros
 * R_angle   the measurement covariance noise (jitter in the accelerometers)
 */
void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle) 
{
	kalman->Q_angle = Q_angle;
	kalman->Q_gyro  = Q_gyro;
	kalman->R_angle = R_angle;
  
	kalman->P_00 = 0;
	kalman->P_01 = 0;
	kalman->P_10 = 0;
	kalman->P_11 = 0;
}

void processGyroKalman()
{
	unsigned long now = millis();
	double dt = ((double)(now - lastread)) * SecondsPerMillis; //compute the time delta since last iteration, in seconds.

	// Only process delta angles if at least 1/100 of a second has elapsed
	if (dt >= 0.01) 
	{			
		lastread = now;

 		// calculate gyro deviations from center values
		pitch = adc_sensor_val[ADC_GYROX-1] - (int16) adc_gyrox_center;
		roll = adc_sensor_val[ADC_GYROY-1] - (int16) adc_gyroy_center;
		processAccelerometers(); 
    
		readingsX = pitch;	// read from the gyro sensor
		readingsY = roll;	// read from the gyro sensor
    
		predict(&pitchData, readingsX * DegreeToRadian, dt);
		predict(&rollData, readingsY * DegreeToRadian, dt);
  
		pitchAngle = (int16) (update(&pitchData, accelAngleX)*RadianToDegree);
		rollAngle = (int16) (update(&rollData, accelAngleY)*RadianToDegree);

		// TEST: 
		// printf("\nPitch: %i, Roll: %i, AccelX: %i", pitchAngle, rollAngle, (int16)(accelAngleX*RadianToDegree));
	}
}

/*
 * The kalman predict method.  See http://en.wikipedia.org/wiki/Kalman_filter#Predict
 *
 * kalman    the kalman data structure
 * dotAngle  Derivative Of The (D O T) Angle.  This is the change in the angle from the gyro.  This is the value from the
 *           Wii MotionPlus, scaled to fast/slow.
 * dt        the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
 *
 * Note: Tom Pycke's ars.c code was the direct inspiration for this.  However, his implementation of this method was inconsistent
 *       with the matrix algebra that it came from.  So I went with the matrix algebra and tweaked his implementation here.
 */
void predict(struct GyroKalman *kalman, double dotAngle, double dt) 
{
	kalman->x_angle += dt * (dotAngle - kalman->x_bias);
	kalman->P_00 += -1 * dt * (kalman->P_10 + kalman->P_01) + dt*dt * kalman->P_11 + kalman->Q_angle;
	kalman->P_01 += -1 * dt * kalman->P_11;
	kalman->P_10 += -1 * dt * kalman->P_11;
	kalman->P_11 += kalman->Q_gyro;
}

/*
 * The kalman update method.  See http://en.wikipedia.org/wiki/Kalman_filter#Update
 *
 * kalman    the kalman data structure
 * angle_m   the angle observed from the accelerometer, in radians
 */
double update(struct GyroKalman *kalman, double angle_m) 
{
	const double y = angle_m - kalman->x_angle;
	const double S = kalman->P_00 + kalman->R_angle;
	const double K_0 = kalman->P_00 / S;
	const double K_1 = kalman->P_10 / S;

	kalman->x_angle += K_0 * y;
	kalman->x_bias  += K_1 * y;
	
	kalman->P_00 -= K_0 * kalman->P_00;
	kalman->P_01 -= K_0 * kalman->P_01;
	kalman->P_10 -= K_1 * kalman->P_00;
	kalman->P_11 -= K_1 * kalman->P_01;
  
	return kalman->x_angle;
}

/*
 * This function processes the accelerometer data. The execution of this function results 
 * in the updating of 2 globals, accelAngleX, accelAngleY, with the angle in radians.
 * This code assumes:
 *    Accelerometer X maps to Gyro Roll
 *    Accelerometer Y maps to Gyro Pitch
 */
void processAccelerometers ()
{
	// Calculate actual accelerometer values
	int ax_m = adc_sensor_val[ADC_ACCELX-1] - adc_accelx_center;	// center value is ~2500mV, produces acceleration in mg
	int ay_m = adc_sensor_val[ADC_ACCELY-1] - adc_accely_center;	// center value is ~2500mV, produces acceleration in mg

	// The real purpose of this method is to convert the accelerometer values into usable angle values.  
	// Knuckles904 pointed me to this app note: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
	// That paper gives the math for implementing a tilt sensor using a 2 or 3-axis accelerometers. 
	// Roll(X) and pitch(Y) are directly applicable.   
  
	accelAngleX = angleInRadians(ax_m);
	accelAngleY = angleInRadians(ay_m);
}

/*
 * Accelerometer value to radian conversion. Uses small angle approximation for sin(x)
 * and makes sure values stay within limits
 */
float angleInRadians(int measured) 
{
	float x = (float)measured/1000.0;	// convert from mg to g
	if ( x < 0.5 && x > -0.5 ) {
		// for small angles (<30deg) we can use sin(x) ~= x
		return x;
	} else if( x > -1.0 && x < 1.0) {
		return asin(x);
	} else if( x <= -1.0 ) {
		return -PI/2;
	} else {
		return PI/2;
	}	
}

