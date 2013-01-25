/*
 * rc100.h - Functions for the RC100 remote control 
 * using the Zig-110 module in the remote
 *
 * Based on the embedded C library provided by Robotis  
 * Version 0.7		31/01/2013
 * Modified by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 * 
*/
#ifndef _RC100_H_
#define _RC100_H_

#ifdef __cplusplus
extern "C" {
#endif

///////////// device control methods ////////////////////////
void rc100_initialize( void );

////////// communication methods ///////////////////////
int rc100_tx_data( int data );
int rc100_rx_check( void );
int rc100_rx_data( void );

////////// define RC-100 button key value ////////////////
// When more than 2 buttons are pressed, the sum of pressed code values will be sent.
// EX1)  Button U + Button 3 => Code value of Button U (1) + Code value of Button 3 (64) = Sending Value (65)
// EX2)  Button 1 + Button 5 => Code value of Button 1 (16) + Code value of Button 5 (256) = Sending Value (272)
#define RC100_BTN_U		(1)
#define RC100_BTN_D		(2)
#define RC100_BTN_L		(4)
#define RC100_BTN_R		(8)
#define RC100_BTN_1		(16)
#define RC100_BTN_2		(32)
#define RC100_BTN_3		(64)
#define RC100_BTN_4		(128)
#define RC100_BTN_5		(256)
#define RC100_BTN_6		(512)

#ifdef __cplusplus
}
#endif

#endif
