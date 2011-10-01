/*
 * walk.c - Functions for walking
 * 
 * Version 0.4		30/09/2011
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to peter_lanius@yahoo.com.au
 *
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

#include "global.h"
#include "motion_f.h"
#include "walk.h"

// define the walk ready and balance motion pages
#define WALK_READY_MOTION	31		// same for all 3 humanoid robots
#define BALANCE_MOTION		224		// same for all 3 humanoid robots

// global variable that keeps the current motion page
extern uint8 current_motion_page;

// the functions share some variables to keep the walking state
static uint8 walk_command = 0;
static uint8 walk_state = 0;

// initialize for walking - assume walk ready pose
void walk_init()
{
	// reset walk state and command
	walk_state = 0;
	walk_command = 0;
	
	// and get ready for walking!
	executeMotionSequence(WALK_READY_MOTION);
}

// 
void walkExecute(uint8 walk_command)
{
	int StartPage = 0;
	
	// check if walk command equals current state
	if ( walk_command == walk_state ) 
	{
		// nothing to do, already executing the command
		return;
	}	
	
	// next check is to see if we are asking to stop
	if ( walk_command == WALK_COMMAND_STOP )
	{
		// execute exit page for current motion
		executeMotionExitPage();
		return;
	}
	
	// first we need to check if we are walking already
	if ( walk_state != 0 ) 
	{
		// we are already walking, needs different approach 
		//walkShift(walk_command);
		return;
	}	
	
	// not walking yet, shift to WalkReady Page
	if (current_motion_page != BALANCE_MOTION)
	{
		// get ready for walking
		executeMotionSequence(WALK_READY_MOTION);
	}
	
	// now we can calculate the motion page for the walk command
	// all walk command motion pages are in sequence and 12 pages apart each
	StartPage = 12*(walk_command-1) + WALK_READY_MOTION + 1;
	executeMotionSequence(StartPage);
}