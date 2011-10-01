/*
 * walk.h - functions for walking 
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


#ifndef WALK_H_
#define WALK_H_

// define the different walk commands
#define	WALK_COMMAND_STOP				0
#define	WALK_COMMAND_FORWARD			1
#define	WALK_COMMAND_BACKWARD			2
#define	WALK_COMMAND_TURN_LEFT			3
#define	WALK_COMMAND_TURN_RIGHT			4
#define	WALK_COMMAND_LEFT_SIDE			5
#define	WALK_COMMAND_RIGHT_SIDE			6
#define	WALK_COMMAND_FORWARD_LEFTSIDE	7
#define	WALK_COMMAND_FORWARD_RIGHTSIDE	8
#define	WALK_COMMAND_BACKWARD_LEFTSIDE	9
#define	WALK_COMMAND_BACKWARD_RIGHTSIDE	10
#define	WALK_COMMAND_AVOID_LEFT			11
#define	WALK_COMMAND_AVOID_RIGHT		12
#define	WALK_COMMAND_FORWARD_TURNLEFT	13
#define	WALK_COMMAND_FORWARD_TURNRIGHT	14
#define	WALK_COMMAND_BACKWARD_TURNLEFT	15
#define	WALK_COMMAND_BACKWARD_TURNRIGHT	16


// initialize for walking - assume walk ready pose
void walk_init();

// 
void walkExecute(uint8 walk_command);

// 
void walkShift(uint8 walk_command);

#endif /* WALK_H_ */