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

// initialize for walking - assume walk ready pose
void walk_init();

// function to update the walk state
void walkSetWalkState(int command);

// function to retrieve the walk state
// Returns (int) walk state
int walkGetWalkState();

// Function that allows 'seamless' transition between certain walk commands
// Handles transitions between 1. WFWD - WFLS - WFRS and
//                             2. WBWD - WBLS - WBRS
// All other transitions between walk commands have to go via their exit page
// Returns:	(int)	shift flag 0 - nothing happened
//							   1 - new motion page set
int walkShift();

#endif /* WALK_H_ */