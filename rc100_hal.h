/*
 * RC100_HAL.h - Functions for the RC100 remote control 
 * using the Zig-110 module in the remote
 *
 * Based on the embedded C library provided by Robotis  
 * Version 0.7		31/01/2013
 * Modified by Peter Lanius
 * Please send suggestions and bug fixes to PeterLanius@gmail.com
 * 
*/
#ifndef _RC100_HAL_H_
#define _RC100_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

int rc100_hal_open( int devIndex, float baudrate );
void rc100_hal_close(void);
int rc100_hal_tx( unsigned char *pPacket, int numPacket );
int rc100_hal_rx( unsigned char *pPacket, int numPacket );

#ifdef __cplusplus
}
#endif

#endif
