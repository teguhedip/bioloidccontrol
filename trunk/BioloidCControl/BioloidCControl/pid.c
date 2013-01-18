/*
 * pid.c - proportional, integral, derivative controller with
 *    on-the-fly tuning changes and derivative kick avoidance
 *
 * Written by Peter Lanius
 * Version 0.6		30/11/2011 
 * Based on the Arduino PID library (see below)
 */ 

/*
 * Arduino PID Library - Version 1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 *   http://www.arduino.cc/playground/Code/PIDLibrary
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 */

#include <stdio.h>
#include "global.h"
#include "clock.h"
#include "pid.h"

// we assume that x and y axis use the same tuning parameters 
// since gyro and actuators are the same
double kp;          // (P)roportional Tuning Parameter
double ki;          // (I)ntegral Tuning Parameter
double kd;          // (D)erivative Tuning Parameter

double dispKp;		// we'll hold on to the tuning parameters in user-entered 
double dispKi;		// format for display purposes
double dispKd;		//
    
// Input, Output and Setpoint variables for the PID controller
extern volatile double pid_input[PID_DIMENSION];    
extern volatile double pid_output[PID_DIMENSION]; 
extern volatile double pid_setpoint[PID_DIMENSION]; 
			  
// internal variables
int controller_direction = 0;		// direction - DIRECT or REVERSE
int sample_time = 16;				// fixed controller sample time in ms
unsigned long last_time = 0;		// last_time in millis the controller was run
double integral_term[PID_DIMENSION];	// integral terms
double last_input[PID_DIMENSION];	// last input values
double outMin, outMax;				// assumed to be the same in all dimensions
bool inAuto;						// automatic or manual mode


/*Initialization() *********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void pid_init()
{
	// initialize the input, output and setpoint arrays
	for (uint8 i=0; i<PID_DIMENSION; i++) {
		pid_input[i] = 0;
		pid_output[i] = 0;
		pid_setpoint[i] = 0; 
		integral_term[i] = 0;
	}
	
	// set basic properties
	pid_setOutputLimits(0-OUTPUT_LIMIT, OUTPUT_LIMIT);	// default output limit - see pid.h
    sample_time = SAMPLE_INTERVAL;			// default Controller sample time is 16ms
    pid_setControllerDirection(DIRECT);		// direct mode
	// pid_setControllerDirection(REVERSE);		// reverse mode
	
	// set default tuning values - see pid.h
    pid_setTunings(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);		
	
	// set sample time and initialize timer
	pid_setSampleTime(SAMPLE_INTERVAL);
	last_time = millis() - sample_time;		// initialize time keeping variable		
    inAuto = FALSE;							// don't start the controller until needed
}
 
 
/* Compute() **********************************************************************
 *   This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether
 *   a new pid Output needs to be computed
 **********************************************************************************/ 
int	pid_compute()
{
	double input, error, dInput, output;
	
	// only compute if in automatic mode
	if (!inAuto) return 0;
	
	// check if we are due for a calculation
	unsigned long now = millis();
	int timeChange = (now - last_time);
	
	if ( timeChange >= sample_time )
	{
		for (uint8 i=0; i<PID_DIMENSION; i++)
		{
			// Compute all the working error variables
			input = pid_input[i];
			error = pid_setpoint[i] - input;
			integral_term[i] += (ki * error);
			
			// keep the integral term within limits
			if (integral_term[i] > outMax) {
				integral_term[i] = outMax;
			} else if (integral_term[i] < outMin) {
				integral_term[i] = outMin;
			}			
			
			dInput = (input - last_input[i]);
 
			// Compute PID Output
			output = kp * error + integral_term[i] - kd * dInput;
			// TEST:
			printf("\nCh: %i, PID input=%i, last=%i, output=%i, ", i, (int16)pid_input[i], (int16)last_input[i], (int16)output);
			printf(" Err=%i, dI=%i, Int=%i", (int16)error, (int16)dInput, (int16)integral_term[i]);

			// apply output limits
			if (output > outMax) { 
				output = outMax;
			} else if (output < outMin) {
				output = outMin;
			}			
			pid_output[i] = output;
	  
			// Remember some variables for next time
			last_input[i] = input;
		}
		
		last_time = now;
		return 1;
	} else {
		return 0;
	}
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void pid_setTunings(double Kp, double Ki, double Kd)
{
	// this version of the controller only allows positive tuning parameters
	if ( Kp<0 || Ki<0 || Kd<0 ) return;
 
	// keep the user set values in case they get changed
	dispKp = Kp; 
	dispKi = Ki; 
	dispKd = Kd;
   
	// need to convert ms to s for calculations
	double SampleTimeInSec = ((double)sample_time)/1000;  
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
 
	if ( controller_direction == REVERSE )
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void pid_setSampleTime(int new_sample_time)
{
   if (new_sample_time > 0)
   {
      double ratio  = (double) new_sample_time / (double) sample_time;
      ki *= ratio;
      kd /= ratio;
      sample_time = (unsigned long) new_sample_time;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *  This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,) the output will be a little different. Maybe they'll
 *  be doing a time window and will need 0-8000 or something. Or maybe they'll
 *  want to clamp it from 0-125.  who knows.
 **************************************************************************/
void pid_setOutputLimits(int output_min, int output_max)
{
	// sanity check
	if ( output_min >= output_max ) return;

	// set the output limits
	outMin = (double) output_min;
	outMax = (double) output_max;

	if ( inAuto )
	{
		// make sure outputs do not exceed maximum values
		for (uint8 i=0; i<PID_DIMENSION; i++)
		{
			if (pid_output[i] > outMax) { 
				pid_output[i] = outMax; 
			} else if (pid_output[i] < outMin) {
				pid_output[i] = outMin;
			}
		}			
	 
		// same applies to the integral terms
		for (uint8 i=0; i<PID_DIMENSION; i++)
		{
			if (integral_term[i] > outMax) {
				integral_term[i] = outMax;
			} else if (integral_term[i] < outMin) {
				integral_term[i] = outMin;
			}		
		}
	}
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void pid_setMode(int mode)
{
	bool newAuto = (mode == AUTOMATIC);
	if (newAuto == !inAuto) {  
		/*we just went from manual to auto*/
		pid_initialize();
	}
	inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void pid_initialize()
{
	// set integral term and last input
	for (uint8 i=0; i<PID_DIMENSION; i++)
	{
		integral_term[i] = pid_output[i];
		last_input[i] = pid_input[i];
		// preserve output limits
		if (integral_term[i] > outMax) {
			integral_term[i] = outMax;
		} else if (integral_term[i] < outMin) {
			integral_term[i] = outMin;
		}		
	}
	// initialize time keeping variable		
	last_time = millis() - sample_time;		
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void pid_setControllerDirection(int direction)
{
   if (inAuto && direction != controller_direction)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controller_direction = direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID. They're here for display 
 * purposes. This are the functions the PID Front-end uses for example
 ******************************************************************************/
double pid_getKp() { return  dispKp; }
double pid_getKi() { return  dispKi; }
double pid_getKd() { return  dispKd; }
int pid_getMode() { return  inAuto ? AUTOMATIC : MANUAL; }
int pid_getDirection() { return controller_direction; }

