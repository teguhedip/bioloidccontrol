/*
 * balance.h - functions for static balancing of the robot  
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


#ifndef BALANCE_H_
#define BALANCE_H_


// balances the robot to compensate for left-right and forward-backward tilt
// uses the PID controller to calculate adjustments
void staticRobotBalance();

/*
 * Initialize everything.
 */
void setupGyroKalman();

#endif /* BALANCE_H_ */