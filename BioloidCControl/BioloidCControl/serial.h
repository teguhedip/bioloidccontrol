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

// Top level serial port task
// manages all requests to read from or write to the serial port
// Receives commands from the serial port and writes output (excluding printf)
// Checks the status flag provided by the ISR for operation
// Returns:  int flag = 0 when no new command has been received
//           int flag = 1 when new command has been received
int serialReceiveCommand();

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
