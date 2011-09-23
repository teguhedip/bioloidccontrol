/*
 * Serial.h - Functions for the serial port PC interface on the 
 * Robotis CM-510 controller. 
 * Can either use serial cable or Zig2Serial via Zig-110
 *
 * Based on the embedded C library provided by Robotis  
 * Version 0.4		30/09/2011
 * Modified by Peter Lanius
 * Please send suggestions and bug fixes to peter_lanius@yahoo.com.au
 * 
*/

#ifndef _SERIAL_H_
#define _SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/pgmspace.h>

// Define which mode of transport is used
// Use SERIAL_CABLE for Rs-232 cable from USB2Dynamixel to CM-510
// Use ZIG_2_SERIAL for Zig-110 to Zig2Serial attached to USB2Dynamixel
#define SERIAL_CABLE
//#define	ZIG_2_SERIAL

#ifdef	SERIAL_CABLE
  #define MAXNUM_SERIALBUFF	128 // maximum 128byte string (cable)
#else
  #define MAXNUM_SERIALBUFF	256 // maximum 256byte string (Zig2Serial)
#endif
#define DEFAULT_BAUDRATE	34  // 57132(57600)bps

// ZIGBEE
// The three steps that perplex me (PORTD is by default in input mode and none
//  of the example programs change that)
//    1) Turn off pull-up resistor of PORTD PIN7 (UNLISTED ON SCHEMATIC)
//        Suspect this is for the CM-510 with its audio connector (with 
//        a built-in switch) for RS-232 serial link
//    2) Turn off pull-up resistor of PORTD PIN5 (UNLISTED ON SCHEMATIC)
//        Suspect this is for disabling the RS-232 level converter, but if 
//        it really were, shouldn't this pin be an output instead of an input?
//    3) Turn on pull-up resistor of PORTD PIN6 (UNLISTED ON SCHEMATIC)
//        Suspect this is related to the RS-232 level converter and/or a line
//        buffer, but if it really were, shouldn't this pin be an output
//        instead of an input?  
#ifdef ZIG_2_SERIAL
  #define ZIG110_ENABLE	PORTD &= ~0x80; PORTD &= ~0x20; PORTD |= 0x40
// Going by the CM-5/CM-700 schematic on the Robotis support site
// ZIG-100 power/reset control is PORTD PIN4 and should be an output
  #define ZIG110_RESET	DDRC|=0x10; PORTD|=0x10; _delay_ms(10); PORTD&=~0x10
  #define ZIG110_DOWN		DDRC|=0x10; PORTD|=0x10
  #define ZIG110_UP		DDRC|=0x10; PORTD&=~0x10
#endif

// Command List
#define NUMBER_OF_COMMANDS				20	// how many commands we recognize
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
#define COMMAND_SIT						17
#define COMMAND_BALANCE					18
#define COMMAND_MOTIONPAGE				19
#define COMMAND_NOT_FOUND				255

// Top level serial port task
// manages all requests to read from or write to the serial port
// Receives commands from the serial port and writes output (excluding printf)
// Checks the status flag provided by the ISR for operation
void SerialReceiveCommand();

// Serial Port initialization with the specified baud rate
void serial_init(long baudrate);

// write a string to the serial port
void serial_write( unsigned char *pData, int numbyte );

// read a string from the serial port
unsigned char serial_read( unsigned char *pData, int numbyte );

// get the status of the input/output queue
int serial_get_qstate(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H_ */
