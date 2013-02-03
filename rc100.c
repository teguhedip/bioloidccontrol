/*
 * rc100.c - Functions for the RC100 remote control 
 * using the Zig-110 module in the remote
 *
 * Based on the embedded C library provided by Robotis  
 * Version 0.7		31/01/2013
 * Modified by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 * 
*/

#include "serial.h"
#include "rc100.h"
#include <avr/io.h>

// RC-100 data packet related variables
unsigned char  gbRcvPacket[6];
unsigned char  gbRcvPacketNum;
unsigned short gwRcvData;
unsigned char  gbRcvFlag;

// Initialise the RC-100 related variables
// The RC-100 uses a 6-byte data packet format
void rc100_initialize( void )
{
	gbRcvFlag = 0;
	gwRcvData = 0;
	gbRcvPacketNum = 0;
	return;
}

// send a data packet to the RC-100
// there is no documentation that the RC-100 will receive / interpret a packet
// return value:	1 - success
//					0 - error
int rc100_tx_data(int data)
{
	unsigned char SndPacket[6];
	unsigned short word = (unsigned short)data;
	unsigned char lowbyte = (unsigned char)(word & 0xff);
	unsigned char highbyte = (unsigned char)((word >> 8) & 0xff);

	// construct the data packet in the RC-100 format
	SndPacket[0] = 0xff;
	SndPacket[1] = 0x55;
	SndPacket[2] = lowbyte;
	SndPacket[3] = ~lowbyte;
	SndPacket[4] = highbyte;
	SndPacket[5] = ~highbyte;
	
	// write the packet to the serial port (ZigBee)
	if( serial_write( SndPacket, 6 ) != 6 )
		return 0;

	return 1;
}

// The RC-100 uses the communication packet in the form below
//			FF 55 Data_L ~Data_L Data_H ~Data_H
// Example: DATA : 0x 1234
// Packet : 0x FF 0x 55 0x 34 0x CB 0x 12 0x ED
// function returns the value of the receive flag
int rc100_rx_check( void )
{
	unsigned char RcvNum;
	unsigned char checksum;
	int i, j;

	// have unprocessed received data word, return
	if( gbRcvFlag == 1 )
		return 1;
	
	// Fill packet buffer
	if( gbRcvPacketNum < 6 )
	{
		RcvNum = serial_read( &gbRcvPacket[gbRcvPacketNum], (6 - gbRcvPacketNum) );
		if( RcvNum != -1 )
			gbRcvPacketNum += RcvNum;
	}
	
	// Find header	
	if( gbRcvPacketNum >= 2 )
	{
		for( i=0; i<gbRcvPacketNum; i++ )
		{
			if( gbRcvPacket[i] == 0xff )
			{
				if(i <= (gbRcvPacketNum - 2))
				{
					if(gbRcvPacket[i+1] == 0x55)
						break;
				}
			}
		}
		if(i > 0)
		{
			if( i == gbRcvPacketNum )
			{
				// Cannot find header
				if(gbRcvPacket[i - 1] == 0xff)
					i--;
			}

			// Remove data before header
			for( j=i; j<gbRcvPacketNum; j++)
			{
				gbRcvPacket[j - i] = gbRcvPacket[j];
			}
			gbRcvPacketNum -= i;
		}
	}
	
	// Verify packet, expect 6 bytes
	if(gbRcvPacketNum == 6)
	{
		// first byte needs to be 0xFF, second byte 0x55
		if(gbRcvPacket[0] == 0xff && gbRcvPacket[1] == 0x55)
		{
			// check that ~(Byte 3) = Byte 4
			checksum = ~gbRcvPacket[3];
			if(gbRcvPacket[2] == checksum)
			{
				// check that ~(Byte 5) = Byte 6
				checksum = ~gbRcvPacket[5];
				if(gbRcvPacket[4] == checksum)
				{
					// all checks OK, assemble data word from low and high byte
					gwRcvData = (unsigned short)((gbRcvPacket[4] << 8) & 0xff00);
					gwRcvData += gbRcvPacket[2];
					// set receive flag
					gbRcvFlag = 1;
				}
			}
		}

		gbRcvPacket[0] = 0x00;
		gbRcvPacketNum = 0;
	}
	return gbRcvFlag;
}

// return the data byte extracted from the packet
int rc100_rx_data( void )
{
	// reset the receive flag
	gbRcvFlag = 0;
	
	// return the data byte
	return (int)gwRcvData;
}
