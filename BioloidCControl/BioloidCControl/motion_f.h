/*
 * motion_f.h - function prototypes for motion.c
 * 
 * Version 0.4		30/09/2011
 */

/*
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to peter_lanius@yahoo.com.au
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact. Licensed under the Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, this work is provided
 * without any warranty. It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */


#ifndef MOTION_F_H_
#define MOTION_F_H_

// Initialize the motion pages by constructing a table of pointers to each page
// Motion pages are stored in Flash (PROGMEM) - see motion.h
void motionPageInit();

// This function unpacks a motion stored in program memory (Flash) 
// in a struct stored in RAM to allow execution
// StartPage - number of the motion page to be unpacked
void unpackMotion(int StartPage);

// This function executes a single robot motion page defined in motion.h
// StartPage - number of the first motion page in the motion
// Returns StartPage of next motion in sequence (0 - no further motions)
int executeMotion(int StartPage);

// This function executes a robot motion consisting of one or more motion 
// pages defined in motion.h
// StartPage - number of the first motion page in the motion
void executeMotionSequence(int StartPage);

// This function executes the exit page motion for the  
// current motion page
 void executeMotionExitPage();

#endif /* MOTION_F_H_ */