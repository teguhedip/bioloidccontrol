/*
 * pose.c - functions for assuming poses based on motion pages  
 *	also provides the interpolation to smooth out movement 
 * 
 * Version 0.4		30/09/2011
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to peter_lanius@yahoo.com.au
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

#include <util/delay.h>
#include <stdio.h>
#include "global.h"
#include "pose.h"
#include "dynamixel.h"
#include "clock.h"

// global hardware definition variables
extern const uint8 AX12Servos[MAX_AX12_SERVOS]; 
extern const uint8 AX12_IDS[NUM_AX12_SERVOS];
// should keep the current pose in a global array
extern int16 current_pose[NUM_AX12_SERVOS];

// initial robot position (MotionPage 224 - Balance)
const uint16 InitialValues[NUM_AX12_SERVOS] = {235,788,279,744,462,561,358,666,507,516,341,682,240,783,647,376,507,516}; 
const uint16 InitialPlayTime = 400; // 0.4s is fast enough

// we keep shared variables for goal pose and speed
uint16 goal_pose[NUM_AX12_SERVOS];
uint16 goal_speed[NUM_AX12_SERVOS];


// the new implementation of AVR libc does not allow variables passed to _delay_ms
static inline void delay_ms(uint8 count) {
	while(count--) { 
		_delay_ms(1); 
	} 
}

// read in current servo positions to determine current pose
void readCurrentPose()
{
	// loop over all possible actuators
	for(int i=0; i<NUM_AX12_SERVOS; i++) {
		current_pose[i] = dxl_read_word( AX12_IDS[i], DXL_PRESENT_POSITION_L );
	}
}

// Function to wait out any existing servo movement
void waitForPoseFinish()
{
	uint8 still_moving[NUM_AX12_SERVOS], moving_flag = 0;
	uint8 first_loop = 0;
	
	first_loop = 0;
	// keep looping over all possible actuators until done
	do 
	{
		// reset the flag
		moving_flag = 0;
		
		for (int i=0; i<NUM_AX12_SERVOS; i++) {
			// keep reading the moving state of servos 
			if( first_loop == 0 || still_moving[i] == 1) {
				still_moving[i] = dxl_read_byte( AX12_IDS[i], DXL_MOVING );
				moving_flag += still_moving[i];
			}		
		}
		first_loop = 1;
	} while (moving_flag > 0);

}

// Calculate servo speeds to achieve desired pose timing
// We make the following assumptions:
// AX-12 speed is 59rpm @ 12V which corresponds to 0.170s/60deg
// The AX-12 manual states this as the 'no load speed' at 12V
// We ignore the Moving Speed entry which states that 0x3FF = 114rpm
// Instead we assume that Moving Speed 0x3FF = 59rpm
void calculatePoseServoSpeeds(uint16 time)
{
    int i;
	uint16 travel[NUM_AX12_SERVOS];
	uint32 factor;

	// read the current pose
	readCurrentPose();
	
	printf("\nCalculate Pose Speeds. Time = %i \n", time);
	// determine travel for each servo 
	for (i=0; i<NUM_AX12_SERVOS; i++)
	{
		printf("DXL%i Current, Goal, Travel, Speed:", i+1);
		// find the amount of travel for each servo
		if( goal_pose[i] > current_pose[i]) {
			travel[i] = goal_pose[i] - current_pose[i];
		} else {
			travel[i] = current_pose[i] - goal_pose[i];
		}
		
		// now we can calculate the desired moving speed
		// for 59pm the factor is 847.46 which we round to 847
		// we need to use a temporary 32bit integer to prevent overflow
		factor = (uint32) 847 * travel[i]; 
		goal_speed[i] = (uint16) ( factor / time );
		// if the desired speed exceeds the maximum, we need to adjust
		if (goal_speed[i] > 1023) goal_speed[i] = 1023;
		// we also use a minimum speed of 16 (no science behind this)
		if (goal_speed[i] < 16) goal_speed[i] = 16;
		
		// test print
		printf(" %u, %u, %u, %u\n", current_pose[i], goal_pose[i], travel[i], goal_speed[i]);
	}
	
}


// Moves from the current pose to the goal pose
// using interpolation of steps and delay between steps
// to achieve the required step timing
void moveToGoalPose(uint16 time, uint16 goal[])
{
    int i;
	int commStatus, errorStatus;

	// copy goal to shared variable
	for (i=0; i<NUM_AX12_SERVOS; i++)
		{ goal_pose[i] = goal[i]; }

	// do the setup and calculate speeds
	calculatePoseServoSpeeds(time);

	// write out the goal positions for final frame via sync write
	commStatus = dxl_set_goal_speed(NUM_AX12_SERVOS, AX12_IDS, goal_pose, goal_speed);
	// check for communication error or timeout
	if(commStatus != COMM_RXSUCCESS) {
		// there has been an error, print and break
		printf("\nmoveToGoalPose - ");
		dxl_printCommStatus(commStatus);
		return;
	}
	
	// wait for the movement to finish
	waitForPoseFinish();
	
	// check that we didn't cause any alarms
	for (i=0; i<NUM_AX12_SERVOS; i++) {
		// ping the servo and unpack error code (if any)
		errorStatus = dxl_ping(AX12_IDS[i]);
		if(errorStatus != 0) {
			// there has been an error, disable torque
			commStatus = dxl_write_word(BROADCAST_ID, DXL_TORQUE_ENABLE, 0);
			printf("\nmoveToGoalPose Alarm ID%i - Error Code %i\n", AX12_IDS[i], errorStatus);
			return;
		}
	}	
	
	// all ok, read back current pose
	readCurrentPose();	
}

// move robot to default pose
void moveToDefaultPose()
{
	// assume default pose defined 
	moveToGoalPose(InitialPlayTime, InitialValues);
}