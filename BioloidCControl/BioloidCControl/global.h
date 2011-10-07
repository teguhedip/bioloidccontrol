/*
 * global.h - Basic definitions for the Robotis Bioloid CM-510 controller. 
 *   
 * Version 0.4		30/09/2011
 * Written by Peter Lanius
 * Please send suggestions and bug fixes to peter_lanius@yahoo.com.au
*/

/*
 * You may freely modify and share this code, as long as you keep this
 * notice intact.  Licensed under the Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, this work is provided
 * without any warranty. It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

#ifndef	_GLOBAL_H_ 
#define _GLOBAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef F_CPU
#define F_CPU 16000000UL	// CM-510 runs at 16 MHz
#endif //!F_CPU

#include <stdint.h>


// **************** HARDWARE DEFINITION *****************
// ******************************************************
// **** Please select your robot type before compile ****
// ******************************************************
#define HUMANOID_TYPEA
// #define HUMANOID_TYPEB
// #define HUMANOID_TYPEC

// Robot specific definitions:
// Number of Dynamixel actuators (see also main program for ID's)
// Number of motion pages in the motion file (and motion.h)
#ifdef HUMANOID_TYPEA
	#define NUM_AX12_SERVOS		18
	#define	NUM_MOTION_PAGES	199 // should be 225 - memory issue
#endif
#ifdef HUMANOID_TYPEB
	#define NUM_AX12_SERVOS		16
	#define	NUM_MOTION_PAGES	227
#endif
#ifdef HUMANOID_TYPEC
	#define NUM_AX12_SERVOS		16
	#define	NUM_MOTION_PAGES	227
#endif

// Dynamixel configuration definitions 
#define MAX_AX12_SERVOS		26
#define MAX_MOTION_STEPS	7

// Top level ADC/Sensor related parameters - adjust as needed
#define SENSOR_READ_INTERVAL	100		// read gyro/DMS every 100ms 
#define BATTERY_READ_INTERVAL	1000	// read battery once every second
#define GYROX_SLIP_ERROR		150		// deviation from 0 interpreted as a slip (150 = 220deg/s rotation)
#define LOW_VOLTAGE_CUTOFF		10500	// 10.5V is a very safe limit for a 11.7V LiPo

// Command List
// To add commands:		1. Add it to the list below
//						2. Update the number of commands below
//						3. If required, add a motion page associated with the command below
//						4. Edit serial.c and update the command string list
//						5. Edit serial.c and update SerialReceiveCommand()
#define NUMBER_OF_COMMANDS				25	// how many commands we recognize
#define COMMAND_STOP					0
#define COMMAND_WALK_FORWARD			1
#define COMMAND_WALK_BACKWARD			2
#define COMMAND_WALK_TURN_LEFT			3
#define COMMAND_WALK_TURN_RIGHT			4
#define COMMAND_WALK_LEFT_SIDE			5
#define COMMAND_WALK_RIGHT_SIDE			6
#define COMMAND_WALK_FWD_LEFT_SIDE		7
#define COMMAND_WALK_FWD_RIGHT_SIDE		8
#define COMMAND_WALK_BWD_LEFT_SIDE		9
#define COMMAND_WALK_BWD_RIGHT_SIDE		10
#define COMMAND_WALK_AVOID_LEFT			11
#define COMMAND_WALK_AVOID_RIGHT		12
#define COMMAND_WALK_FWD_TURN_LEFT		13
#define COMMAND_WALK_FWD_TURN_RIGHT		14
#define COMMAND_WALK_BWD_TURN_LEFT		15
#define COMMAND_WALK_BWD_TURN_RIGHT		16
#define COMMAND_WALK_READY				17
#define COMMAND_SIT						18
#define COMMAND_STAND					19
#define COMMAND_BALANCE					20
#define COMMAND_MOTIONPAGE				21
#define COMMAND_FRONT_GET_UP			22
#define COMMAND_BACK_GET_UP				23
#define COMMAND_RESET					24
#define COMMAND_NOT_FOUND				255

// Motion Pages associated with non-walking commands
// these are the same for all 3 HUMANOID Robot Types
#define COMMAND_WALK_READY_MP			31
#define COMMAND_SIT_MP					25
#define COMMAND_STAND_MP				26
#define COMMAND_BALANCE_MP				14
#define COMMAND_FRONT_GET_UP_MP			27
#define COMMAND_BACK_GET_UP_MP			28
#define COMMAND_RESET_MP				26


// Standard types
typedef uint8_t		uint8;
typedef int8_t		int8;
typedef uint16_t	uint16;
typedef int16_t		int16;
typedef uint32_t	uint32;
typedef int32_t		int32;
typedef uint8_t		bool;

// Boolean types
#define FALSE ((bool)0)
#define TRUE  ((bool)1)

// Additional Bit macros
#define bit_set_hi(Port,Bit)     {Port |=(1<<Bit);}
#define bit_set_lo(Port,Bit)     {Port &= ~(1<<Bit);}
#define bit_toggle(Port,Bit)     {if(Port&(1<<Bit)) {Port &= ~(1<<Bit);} else {Port |=(1<<Bit);}}

/* Constant divide calculation with rounding macro */
#define DIV(Dividend,Divisor) (((Dividend+((Divisor)>>1))/(Divisor)))

// ADC
//     When using IR sensors, must wait ~12 ms between enabling IR PORT
//     and starting conversion
// ADC Enable, Clock 1/64div. prescaler 
#define	ADC_ENABLE		ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1)
// Define which ADC pin to use (also sets reference voltage to AREF pin (5V?))
#define ADC_SELECT(x)	ADMUX = x
// Clear ADC conversion flag, then begin new conversion
#define ADC_BEGIN		ADCSRA |= (1 << ADIF); ADCSRA |= (1 << ADSC)
// True while conversion incomplete (e.g. 'while (ADC_INCOMPLETE);' )
#define ADC_INCOMPLETE	!(ADCSRA & (1 << ADIF))
// number of allowed ADC channels (6 on the CM-510)
#define ADC_CHANNELS	6
// PORTF ADC port pins available
#define ADC_PORT_1		1	// free
#define ADC_PORT_2		2	// GYRO-X
#define ADC_PORT_3		3	// GYRO-Y
#define ADC_PORT_4		4	// DMS
#define ADC_PORT_5		5	// free
#define ADC_PORT_6		6	// free
// ADC Channel settings for CM-510 (default configuration)
#define ADC_BATTERY		0
#define ADC_GYROX		3
#define ADC_GYROY		4
#define ADC_DMS			5

// PORTA
//    Set pin output low to set external header high (inverted via transistor)
#define EXT_PORT_1		0x80
#define EXT_PORT_2		0x40
#define EXT_PORT_3		0x20
#define EXT_PORT_4		0x10
#define EXT_PORT_5		0x08
#define EXT_PORT_6		0x04

// PORTD
// Input high when mic input sufficiently loud
#define MIC_SIGNAL		0x02	// PORTD1

#ifdef __cplusplus
}
#endif

#endif
