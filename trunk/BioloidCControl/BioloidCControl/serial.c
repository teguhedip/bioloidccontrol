/*
 * Serial.c - Functions for the serial port PC interface on the 
 * Robotis CM-510 controller. 
 * Can either use serial cable or Zig2Serial via Zig-110
 *
 * Based on the embedded C library provided by Robotis  
 * Version 0.6	18/01/2013
 * Modified by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 * 
*/

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <ctype.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "global.h"
#include "serial.h"
#include "rc100.h"


// Command Strings List - kept in Flash to conserve RAM
const char COMMANDSTR0[]  PROGMEM = "STOP";
const char COMMANDSTR1[]  PROGMEM = "WFWD";
const char COMMANDSTR2[]  PROGMEM = "WBWD";
const char COMMANDSTR3[]  PROGMEM = "WLT ";
const char COMMANDSTR4[]  PROGMEM = "WRT ";
const char COMMANDSTR5[]  PROGMEM = "WLSD";
const char COMMANDSTR6[]  PROGMEM = "WRSD";
const char COMMANDSTR7[]  PROGMEM = "WFLS";
const char COMMANDSTR8[]  PROGMEM = "WFRS";
const char COMMANDSTR9[]  PROGMEM = "WBLS";
const char COMMANDSTR10[] PROGMEM = "WBRS";
const char COMMANDSTR11[] PROGMEM = "WAL ";
const char COMMANDSTR12[] PROGMEM = "WAR ";
const char COMMANDSTR13[] PROGMEM = "WFLT";
const char COMMANDSTR14[] PROGMEM = "WFRT";
const char COMMANDSTR15[] PROGMEM = "WBLT";
const char COMMANDSTR16[] PROGMEM = "WBRT";
const char COMMANDSTR17[] PROGMEM = "WRDY";
const char COMMANDSTR18[] PROGMEM = "SIT ";
const char COMMANDSTR19[] PROGMEM = "STND";
const char COMMANDSTR20[] PROGMEM = "BAL ";
const char COMMANDSTR21[] PROGMEM = "M   ";
const char COMMANDSTR22[] PROGMEM = "FGUP";
const char COMMANDSTR23[] PROGMEM = "BGUP";
const char COMMANDSTR24[] PROGMEM = "RSET";
const char COMMANDSTR25[] PROGMEM = "WS  ";
const char COMMANDSTR26[] PROGMEM = "W   ";
const char COMMANDSTR27[] PROGMEM = "SQST";
const char COMMANDSTR28[] PROGMEM = "SQND";
const char *const COMMANDSTR_POINTER[] PROGMEM = { 
COMMANDSTR0, COMMANDSTR1, COMMANDSTR2, COMMANDSTR3, COMMANDSTR4,
COMMANDSTR5, COMMANDSTR6, COMMANDSTR7, COMMANDSTR8, COMMANDSTR9,
COMMANDSTR10, COMMANDSTR11, COMMANDSTR12, COMMANDSTR13, COMMANDSTR14, 
COMMANDSTR15, COMMANDSTR16, COMMANDSTR17, COMMANDSTR18, COMMANDSTR19,
COMMANDSTR20, COMMANDSTR21, COMMANDSTR22, COMMANDSTR23, COMMANDSTR24,
COMMANDSTR25, COMMANDSTR26, COMMANDSTR27, COMMANDSTR28 };

// set up the read buffer
volatile unsigned char gbSerialBuffer[MAXNUM_SERIALBUFF] = {0};
volatile unsigned char gbSerialBufferHead = 0;
volatile unsigned char gbSerialBufferTail = 0;
static FILE *device;
// RC-100 related variables
volatile uint8 rc100_packet_count = 0;

// global variables
extern volatile uint8 bioloid_command;			// current command
extern volatile uint8 last_bioloid_command;		// last command
extern volatile uint8 flag_receive_ready;		// received complete command flag
extern volatile uint8 current_motion_page;		// current motion page
extern volatile uint8 next_motion_page;			// next motion page if we got new command
extern volatile uint8 command_sequence_buffer[50];	// command buffer for sequences

// internal function prototypes
void serial_interpret_command ( void );
void rc100_interpret_command ( void );
void serial_put_queue( unsigned char data );
unsigned char serial_get_queue(void);
int std_putchar(char c,  FILE* stream);
int std_getchar( FILE* stream );

// the new implementation of AVR libc does not allow variables passed to _delay_ms
static inline void delay_ms(uint16 count) {
	while(count--) { 
		_delay_ms(1); 
	} 
}


// ISR for serial receive, Serial Port/ZigBEE use USART1
SIGNAL(USART1_RX_vect)
{
	unsigned char c;
	
	c = UDR1;

// we need two versions of the ISR depending on terminal vs. RC-100 input
// Terminal input version (serial cable or Zig2Serial)
#if defined ZIG_2_SERIAL || defined SERIAL_CABLE
	// check if we have received a CR+LF indicating complete string
	if (c == '\r')
	{
		// command complete, set flag and write termination byte to buffer
		flag_receive_ready = 1;
		serial_put_queue( 0xFF );
		c = '\n';
		std_putchar(c, device);
		// test
		std_putchar(' ', device);
	} 
	else
	{
		// put each received byte into the buffer until full
		serial_put_queue( c );
		// echo the character 
		std_putchar(c, device);
	}
#endif

// RC-100 version, need to assemble 6-byte packets
// The RC-100 uses the communication packet in the form below
//			FF 55 Data_L ~Data_L Data_H ~Data_H
// Example: DATA = 0x1234
// Packet : 0xFF 0x55 0x34 0xCB 0x12 0xED
#ifdef RC100
	// The RC-100 repeats sending packets as long as the button is pressed
	// This means we need to ignore packets not yet processed
	if ( flag_receive_ready == 1 ) {
		return;
	}
	// check if we have received a packet start byte (0xFF)
	// BEWARE: the ~Data_H part of a packet can be 0xFF - need to exclude
	if ( c == 0xFF && rc100_packet_count == 0 ) {
		// new packet start - reset packet flag
		rc100_packet_count = 1;
	} else if ( rc100_packet_count == 1 ) { 
		// second byte needs to be 0x55 for valid packet
		if ( c == 0x55 ) {
			rc100_packet_count++;
		} else {
			// invalid packet, ignore and reset
			rc100_packet_count = 0;
		}
	} else if ( rc100_packet_count >= 2 ) {
		// have valid header, start putting bytes in queue
		if ( rc100_packet_count < 5 ) {
			rc100_packet_count++;
			serial_put_queue( c );
		} else if ( rc100_packet_count == 5 ) {
			// packet is finished, set flag
			serial_put_queue( c );
			flag_receive_ready = 1;
			// reset counter and write termination byte
			rc100_packet_count = 0;
			serial_put_queue( 0xFF );
		}
	} else {
		// ignore byte, not part of a valid packet
		rc100_packet_count == 0;
	}
#endif
}


// initialize the serial port with the specified baud rate
void serial_init(long baudrate)
{
	unsigned short Divisor;

	// in case of ZigBee comms, enable the device
#if defined ZIG_2_SERIAL || defined RC100
	DDRC  = 0x7F;
	PORTC = 0x7E;
	// to enable ZigBee communications we need PD5=low, PD6=high, make PD7 input and turn off pull-up on PD7
	PORTD &= ~0x80; 
	PORTD &= ~0x20; 
	PORTD |= 0x40; 
	// we need to wait for the connection to get established
	delay_ms(500);
#endif

	// set UART register A
	//Bit 7: USART Receive Complete
	//Bit 6: USART Transmit Complete
	//Bit 5: USART Data Register Empty 
	//Bit 4: Frame Error
	//Bit 3: Data OverRun
	//Bit 2: Parity Error
	//Bit 1: Double The USART Transmission Speed
	//Bit 0: Multi-Processor Communication Mode
	UCSR1A = 0b01000010;
	
	// set UART register B
	// bit7: enable rx interrupt
    // bit6: enable tx interrupt
    // bit4: enable rx
    // bit3: enable tx
    // bit2: set sending size(0 = 8bit)
	UCSR1B = 0b10011000;
	
	// set UART register C
	// bit6: communication mode (1 = synchronous, 0 = asynchronous)
    // bit5,bit4: parity bit(00 = no parity) 
    // bit3: stop bit(0 = stop bit 1, 1 = stop bit 2)
    // bit2,bit1: data size(11 = 8bit)
	UCSR1C = 0b00000110;

	// Set baud rate
	Divisor = (unsigned short)(2000000.0 / baudrate) - 1;
	UBRR0H = (unsigned char)((Divisor & 0xFF00) >> 8);
	UBRR0L = (unsigned char)(Divisor & 0x00FF);

	// initialize
	UDR1 = 0xFF;
	gbSerialBufferHead = 0;
	gbSerialBufferTail = 0;

	// open the serial device for printf()
	device = fdevopen( std_putchar, std_getchar );
	
	// reset commands and flags
	bioloid_command = 0;				
	last_bioloid_command = 0;		
	flag_receive_ready = 0;			
}


// Top level serial port task
// manages all requests to read from or write to the serial port
// Receives commands from the serial port and writes output (excluding printf)
// Checks the status flag provided by the ISR for operation
// Returns:  int flag = 0 when no new command has been received
//           int flag = 1 when new command has been received
int serialReceiveCommand()
{
	// check for new command	
	if (flag_receive_ready == 0)
	{
		// nothing to do, go straight back to main loop
		return 0;
	}

// command interpretation depends on terminal vs. RC-100 input	
#if defined ZIG_2_SERIAL || defined SERIAL_CABLE
	serial_interpret_command();
#endif
// RC-100 command interpretation
#ifdef RC100
	rc100_interpret_command();
#endif
	
	// set command received flag only if valid command
	if ( bioloid_command == COMMAND_NOT_FOUND ) {
		return 0;
	} else {
		return 1;
	}
}

// Re-assemble the 4-byte ASCII string into the matching command5
void serial_interpret_command ( void )
{
	char c1, c2, c3, c4, command[6], buffer[6];
	int match;

	// we have a new command, get characters
	c1 = serial_get_queue();
	if ( c1 >= 'a' && c1 <= 'z' ) c1 = toupper(c1); // convert to upper case if required
	command[0] = c1;
	c2 = serial_get_queue();
	if ( c2 >= 'a' && c2 <= 'z' ) c2 = toupper(c2); // convert to upper case if required
	if( c2 == 0xFF ) c2 = ' ';						// pad with blanks to 4 characters
	command[1] = c2;
	c3 = serial_get_queue();
	if ( c3 >= 'a' && c3 <= 'z' ) c3 = toupper(c3); // convert to upper case if required
	if( c3 == 0xFF ) c3 = ' ';						// pad with blanks to 4 characters
	command[2] = c3;
	c4 = serial_get_queue();
	if ( c4 >= 'a' && c4 <= 'z' ) c4 = toupper(c4); // convert to upper case if required
	if( c4 == 0xFF ) c4 = ' ';						// pad with blanks to 4 characters
	command[3] = c4;
	command[4] = 0x00;			// finish the string

	// flush the queue in case we received more than 4 bytes
	do
	{
		// need to do it once even for 4 bytes to get rid of the 0xFF marking the end of string
		serial_get_queue();
	} while (serial_get_qstate() != 0);
	
	// loop over all known commands to find a match
	for (uint8 i=0; i<NUMBER_OF_COMMANDS; i++)
	{
		// the command strings sit in PROGMEM - so need to use pgmspace.h macros
		strcpy_P(buffer, (PGM_P)pgm_read_word(&(COMMANDSTR_POINTER[i])));
		match = strcmp(	buffer, command	);
		if ( match == 0 )
		{
			// we found a match set the command
			last_bioloid_command = bioloid_command;
			bioloid_command = i;
			break;
		}
		
		// if we get to end of loop we haven't found a match
		if ( i== NUMBER_OF_COMMANDS-1 )
		{
			last_bioloid_command = bioloid_command;
			// 0xFF means no match found
			bioloid_command = COMMAND_NOT_FOUND;
		}
	}
	
	// find the motion page associated with the command for non-walk commands
	if ( bioloid_command != COMMAND_NOT_FOUND && bioloid_command >= COMMAND_WALK_READY )
	{
		// cross-check against the definitions in global.h
		switch ( bioloid_command )
		{
			case COMMAND_WALK_READY:
			next_motion_page = COMMAND_WALK_READY_MP;
			break;
			case COMMAND_SIT:
			next_motion_page = COMMAND_SIT_MP;
			break;
			case COMMAND_STAND:
			next_motion_page = COMMAND_STAND_MP;
			break;
			case COMMAND_BALANCE:
			next_motion_page = COMMAND_BALANCE_MP;
			break;
			case COMMAND_BACK_GET_UP:
			next_motion_page = COMMAND_BACK_GET_UP_MP;
			break;
			case COMMAND_FRONT_GET_UP:
			next_motion_page = COMMAND_FRONT_GET_UP_MP;
			break;
			case COMMAND_RESET:
			next_motion_page = COMMAND_RESET_MP;
			break;
		}
	}
	// otherwise it's easier to calculate the motion page for walk commands
	else if( bioloid_command >= COMMAND_WALK_FORWARD && bioloid_command < COMMAND_WALK_READY )
	{
		// all walk command motion pages are in sequence and 12 pages apart each
		next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
	}
	
	// before we leave we need to check for special case of Motion Page command
	if( bioloid_command == COMMAND_NOT_FOUND )
	{
		if ( c1 == 'M' && (c2 >= '0' && c2 <= '9') )
		{
			// we definitely have a motion page, find the number
			bioloid_command = COMMAND_MOTIONPAGE;
			next_motion_page = c2 - 48;	// converts ASCII to number
			// check if next character is still a number
			if ( c3 >= '0' && c3 <= '9' )
			{
				next_motion_page = next_motion_page * 10;
				next_motion_page += (c3-48);
			}
			// check if next character is still a number
			if ( c4 >= '0' && c4 <= '9' )
			{
				next_motion_page = next_motion_page * 10;
				next_motion_page += (c4-48);
			}
		}
	}
	
	// reset the flag
	flag_receive_ready = 0;
	
	// finally echo the command and write new command prompt
	if ( bioloid_command == COMMAND_MOTIONPAGE ) {
		printf( "%c%c%c%c - MotionPageCommand %i\n> ", c1, c2, c3, c4, next_motion_page );
	} else if( bioloid_command != COMMAND_NOT_FOUND ) {
		printf( "%c%c%c%c - Command # %i\n> ", c1, c2, c3, c4, bioloid_command );
	} else {
		printf( "%c%c%c%c \nUnknown Command! \n> ", c1, c2, c3, c4 );
	}
}

// verify validity of packet data 
// match to command assignment for the allowed button combinations
void rc100_interpret_command ( void )
{
	uint8 c1, c2, c3, c4, checksum;
	uint16 rc100_data;
	
	// get the 4 bytes out of the buffer queue
	c1 = serial_get_queue();
	c2 = serial_get_queue();
	c3 = serial_get_queue();
	c4 = serial_get_queue();
	
	// reset data word
	rc100_data = 0;
	
	// check that ~(Byte 4) = Byte 3
	checksum = ~c2;
	if( c1 == checksum )
	{
		// check that ~(Byte 6) = Byte 5
		checksum = ~c4;
		if( c3 == checksum )
		{
			// all checks OK, assemble data word from low and high byte
			rc100_data = (unsigned short)((c3 << 8) & 0xFF00);
			rc100_data += c1;
		}
	}

	// interpret the command
	if ( rc100_data != 0 )
	{
		last_bioloid_command = bioloid_command;
		
		// define your RC-100 button-command assignments here
		switch ( rc100_data )
		{
			case RC100_BTN_U:
				bioloid_command = COMMAND_WALK_FORWARD;
				next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
			break;
			case RC100_BTN_D:
				bioloid_command = COMMAND_WALK_BACKWARD;
				next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
			break;
			case RC100_BTN_L:
				bioloid_command = COMMAND_WALK_TURN_LEFT;
				next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
			break;
			case RC100_BTN_R:
				bioloid_command = COMMAND_WALK_TURN_RIGHT;
				next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
			break;
			case RC100_BTN_U_AND_L:
				bioloid_command = COMMAND_WALK_FWD_LEFT_SIDE;
				next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
			break;
			case RC100_BTN_U_AND_R:
				bioloid_command = COMMAND_WALK_FWD_RIGHT_SIDE;
				next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
			break;
			case RC100_BTN_D_AND_L:
				bioloid_command = COMMAND_WALK_LEFT_SIDE;
				next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
			break;
			case RC100_BTN_D_AND_R:
				bioloid_command = COMMAND_WALK_RIGHT_SIDE;
				next_motion_page = 12*(bioloid_command-1) + COMMAND_WALK_READY_MP + 1;
			break;
			case RC100_BTN_5:
				bioloid_command = COMMAND_SIT;
				next_motion_page = COMMAND_SIT_MP;
			break;
			case RC100_BTN_6:
				bioloid_command = COMMAND_STOP;
			break;
			case RC100_BTN_U_AND_1:
				bioloid_command = COMMAND_FRONT_GET_UP;
				next_motion_page = COMMAND_FRONT_GET_UP_MP;
			break;
			case RC100_BTN_D_AND_1:
				bioloid_command = COMMAND_BACK_GET_UP;
				next_motion_page = COMMAND_BACK_GET_UP_MP;
			break;
			case RC100_BTN_L_AND_1:
				bioloid_command = COMMAND_MOTIONPAGE;
				next_motion_page = 8;
			break;
			case RC100_BTN_R_AND_1:
				bioloid_command = COMMAND_MOTIONPAGE;
				next_motion_page = 11;
			break;
			case RC100_BTN_U_AND_2:
				bioloid_command = COMMAND_MOTIONPAGE;
				next_motion_page = 5;
			break;
			case RC100_BTN_D_AND_2:
				bioloid_command = COMMAND_MOTIONPAGE;
				next_motion_page = 7;
			break;
			case RC100_BTN_L_AND_2:
				bioloid_command = COMMAND_MOTIONPAGE;
				next_motion_page = 2;
			break;
			case RC100_BTN_R_AND_2:
				bioloid_command = COMMAND_MOTIONPAGE;
				next_motion_page = 1;
			break;
			default:
				bioloid_command = COMMAND_NOT_FOUND;
			break;
		}

	} 
	else
	{
		// because the RC-100 sends a 0 byte when you lift the button press
		// we just ignore the 0 byte and continue executing the last command
		last_bioloid_command = bioloid_command;
		bioloid_command = COMMAND_NOT_FOUND;
	}

	/* TEST
	// enable ZigBee communications we need PD5=high, PD6=low, make PD7 input and turn off pull-up on PD7
	PORTD |= 0x80;
	PORTD |= 0x20;
	PORTD &= ~0x40;
	printf("\nRC100-Data = %i, Command = %i, Next MP = %i", rc100_data, bioloid_command, next_motion_page);
	// re-enable ZigBee communications we need PD5=low, PD6=high, make PD7 input and turn off pull-up on PD7
	PORTD &= ~0x80;
	PORTD &= ~0x20;
	PORTD |= 0x40;
	TEST */

	// flush the queue in case we received more than 4 bytes
	do
	{
		// need to do it once even for 4 bytes to get rid of the 0xFF marking the end of string
		serial_get_queue();
	} while (serial_get_qstate() != 0);
	
	// reset the flag
	flag_receive_ready = 0;
}


// write out a data string to the serial port
// return the number of bytes sent
int serial_write( unsigned char *pData, int numbyte )
{
	int count;

	for( count=0; count<numbyte; count++ )
	{
		// wait for the data register to empty
		while(!bit_is_set(UCSR1A,5));
		// before writing the next byte
		UDR1 = pData[count];
	}
	return count;
}

// read a data string from the serial port
unsigned char serial_read( unsigned char *pData, int numbyte )
{
	int count, numgetbyte;
	
	// buffer is empty, nothing to read 
	if( gbSerialBufferHead == gbSerialBufferTail )
		return 0;
	
	// check number of bytes requested does not exceed whats in the buffer
	numgetbyte = serial_get_qstate();
	if( numgetbyte > numbyte )
		numgetbyte = numbyte;
	
	for( count=0; count<numgetbyte; count++ )
		pData[count] = serial_get_queue();
	
	// return how many bytes have been read
	return numgetbyte;
}

// get the number of bytes in the buffer
int serial_get_qstate(void)
{
	short NumByte;
	
	if( gbSerialBufferHead == gbSerialBufferTail )
		// buffer is empty
		NumByte = 0;
	// buffer is used in cyclic fashion
	else if( gbSerialBufferHead < gbSerialBufferTail )
		// head is to the left of the tail
		NumByte = gbSerialBufferTail - gbSerialBufferHead;
	else
		// head is to the right of the tail
		NumByte = MAXNUM_SERIALBUFF - (gbSerialBufferHead - gbSerialBufferTail);
	
	return (int)NumByte;
}

// puts a received byte into the buffer
void serial_put_queue( unsigned char data )
{
	// buffer is full, character is ignored
	if( serial_get_qstate() == (MAXNUM_SERIALBUFF-1) )
		return;
	
	// append the received byte to the buffer
	gbSerialBuffer[gbSerialBufferTail] = data;

	// have reached the end of the buffer, restart at beginning
	if( gbSerialBufferTail == (MAXNUM_SERIALBUFF-1) )
		gbSerialBufferTail = 0;
	else
	// move the tail by one byte
		gbSerialBufferTail++;
}

// get a byte out of the buffer
unsigned char serial_get_queue(void)
{
	unsigned char data;
	
	// buffer is empty, return 0xFF
	if( gbSerialBufferHead == gbSerialBufferTail )
		return 0xff;
	
	// buffer not empty, return next byte	
	data = gbSerialBuffer[gbSerialBufferHead];
		
	// head has reached end of buffer, restart at beginning
	if( gbSerialBufferHead == (MAXNUM_SERIALBUFF-1) )
		gbSerialBufferHead = 0;
	else
	// move the head by one byte
		gbSerialBufferHead++;
		
	return data;
}

// writes a single character to the serial port
// newline is replaced with CR+LF
int std_putchar(char c,  FILE* stream)
{
	char tx[2];
	int result;
	
    if( c == '\n' )
	{
        tx[0] = '\r';
		tx[1] = '\n';
		result = serial_write( (unsigned char*)tx, 2 );
		if ( result != 2 ) {
			return 1;
		} else {
			return 0;
		}
	}
	else
	{
		tx[0] = c;
		result = serial_write( (unsigned char*)tx, 1 );
		if ( result != 1 ) {
			return 1;
		} else {
			return 0;
		}
	}
 
}

// get a single character out of the read buffer
// wait for byte to arrive if none in the buffer
int std_getchar( FILE* stream )
{
    char rx;
	
	while( serial_get_qstate() == 0 );
	rx = serial_get_queue();
	
	if( rx == '\r' )
		rx = '\n';

    return rx;
}
