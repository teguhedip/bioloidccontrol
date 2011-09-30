/*
 * BioloidCControl.c - Replacement firmware for the Robotis Bioloid CM-510 controller
 *   and Bioloid Premium Kit based humanoid robots (Type A/B/C). 
 * 
 * Requires a motion.h file generated by the translate_motion.pl Perl script.
 *
 * Supports all motions, including walking, gyro, DMS, buzzer, LEDs, buttons and
 * serial connection via cable, ZIG2Serial and RC-100.
 *   
 * Version 0.4		30/09/2011 - finite state machine based control loop
 *
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

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "global.h"			// contains basic hardware defines
#include "buzzer.h"
#include "button.h"
#include "led.h"
#include "serial.h"
#include "adc.h"
#include "dynamixel.h"
#include "pose.h"
#include "motion_f.h"
#include "clock.h"

// Array showing which Dynamixel servos are enabled (ID from 0 to 25)
#ifdef HUMANOID_TYPEA
  const uint8 AX12Servos[MAX_AX12_SERVOS] = {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0}; 
  const uint8 AX12_IDS[NUM_AX12_SERVOS] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
#endif
#ifdef HUMANOID_TYPEB
  const uint8 AX12Servos[MAX_AX12_SERVOS] = {0,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0}; 
  const uint8 AX12_IDS[NUM_AX12_SERVOS] = {1,2,3,4,5,6,7,8,11,12,13,14,15,16,17,18};
#endif
#ifdef HUMANOID_TYPEC
  const uint8 AX12Servos[MAX_AX12_SERVOS] = {0,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0}; 
  const uint8 AX12_IDS[NUM_AX12_SERVOS] = {1,2,3,4,5,6,9,10,11,12,13,14,15,16,17,18};
#endif

// store the buzzer melodies in Flash
const char melody1[] PROGMEM = "!L16 cdefgab>cbagfedc";
const char melody2[] PROGMEM = "T240 L8 a gafaeada c+adaeafa";
const char melody3[] PROGMEM = "O6 T40 L16 d#<b<f#<d#<f#<bd#f#";
const char melody4[] PROGMEM = "! O6 L16 dcd<b-d<ad<g d<f+d<gd<ad<b-";
const char melody5[] PROGMEM = "! O3 T40 f.b.f.b.f.b.f.b.";
 
// Define global variables for use in the ISRs
// Button related variables
volatile bool button_up_pressed = FALSE;
volatile bool button_down_pressed = FALSE;
volatile bool button_left_pressed = FALSE;
volatile bool button_right_pressed = FALSE;
volatile bool start_button_pressed = FALSE;

// Buzzer related global variables
volatile unsigned char buzzerFinished = 0;	// flag: 0 while playing
const char *buzzerSequence;

// ADC related global variables 
// By default ports are assigned as follows:
//		GyroX = CM-510 Port3 = ADC3 = PORTF3
//		GyroY = CM-510 Port4 = ADC4 = PORTF4
//		DMS   = CM-510 Port5 = ADC5 = PORTF5
volatile uint16 adc_sensor_enable[ADC_CHANNELS] = {0, 0, 1, 1, 1, 0}; 
volatile uint16 adc_sensor_val[ADC_CHANNELS] = {0, 0, 0, 0, 0, 0}; 	// array of sensor values
volatile uint16 adc_battery_val = 0;	// battery voltage in millivolts
volatile uint16 adc_gyrox_center = 0;	// gyro x center values
volatile uint16 adc_gyroy_center = 0;	// gyro y center values

// Global variables related to the finite state machine that governs execution
volatile uint8 bioloid_command = 0;			// current command
volatile uint8 last_bioloid_command = 0;	// last command
volatile bool  new_command = FALSE;			// flag that we got a new command
volatile uint8 flag_receive_ready = 0;		// received complete command flag

// keep the current pose as global variable
volatile int16 current_pose[NUM_AX12_SERVOS];
// and also the current and next motion pages
volatile uint8 current_motion_page = 0;
volatile uint8 next_motion_page = 0;			// next motion page if we got new command

// the new implementation of AVR libc does not allow variables passed to _delay_ms
static inline void delay_ms(uint16 count) {
	while(count--) { 
		_delay_ms(1); 
	} 
}


int main(void)
{
	// local variables
	int	sensor_flag, command_flag, comm_status, slip_flag;
	
	// Initialization Routines
	led_init();				// switches all 6 LEDs on
	serial_init(57600);		// serial port at 57600 baud
	buzzer_init();			// enable buzzer melodies
	button_init();			// enable push buttons on CM-510
	delay_ms(200);			// wait 0.2s 
	led_off(ALL_LED);		// and switch them back off
	
	// initialize the ADC and take default readings
	adc_init();
	// initialize the clock
	clock_init();
	
	// enable interrupts
	sei();
	// print welcome message
	printf("\n\nBioloid C Control V0.4\n");
	// reset the start button variable, something triggers the interrupt on start-up
	start_button_pressed = FALSE;
	
	// print out default sensor values
	printf("Battery, Gyro X, Y Center = %i %i %i \n\n", adc_battery_val, adc_gyrox_center, adc_gyroy_center);
	
	// initialize motion pages
	motionPageInit();
	
	// Wait for the START Button before going any further
	while(!start_button_pressed)
	{
		// PLAY LED is flashing at 5Hz
		led_toggle(LED_PLAY);
		delay_ms(200);
	}
	// Now turn LED solid
	led_on(LED_PLAY);
	// and reset the start button variable
	start_button_pressed = FALSE;

	// perform high level initialization of Dynamixel bus and servos
	dxl_init(DEFAULT_BAUDNUMBER);

	// assume initial pose
	moveToDefaultPose();

	// write out the command prompt
	printf(	"\nReady for command.\n> ");

	// main command loop
    while(1)
    {
		// Check if we received a new command
		command_flag = serialReceiveCommand();
		
		// check if start button has been pressed and we need to do emergency stop
		if ( start_button_pressed && bioloid_command != COMMAND_STOP )
		{
			// disable torque & reset current command
			comm_status = dxl_write_word(BROADCAST_ID, DXL_TORQUE_ENABLE, 0);
			last_bioloid_command = bioloid_command;
			bioloid_command = COMMAND_STOP;
			command_flag = 1;
			// and reset the start button variable
			start_button_pressed = FALSE;
		} else if ( start_button_pressed && bioloid_command == COMMAND_STOP ) {
			// we are resuming from an emergency stop, restore last command
			bioloid_command = last_bioloid_command;
			last_bioloid_command = COMMAND_STOP;
			command_flag = 1;
			// and reset the start button variable
			start_button_pressed = FALSE;
		}
		
		// Check if we need to read the sensors 
		sensor_flag = adc_readSensors();
		if ( sensor_flag == 1 ) {
			// did read sensors - check if robot slipped
			if( adc_sensor_val[ADC_GYROX-1] - adc_gyrox_center > GYROX_SLIP_ERROR ) slip_flag = -1;  // forward slip
			if( adc_sensor_val[ADC_GYROX-1] - adc_gyrox_center < -GYROX_SLIP_ERROR ) slip_flag = 1;	 // backward slip
			// check battery voltage still within limits
			if ( adc_battery_val < LOW_VOLTAGE_CUTOFF ) {
				// too low - play alarm and stop 
				buzzer_playFromProgramSpace(melody5);
				bioloid_command = last_bioloid_command;
				last_bioloid_command = COMMAND_SIT;
				command_flag = 1;
			}
		}
		
		// set the new command global variable
		if( command_flag == 1 ) {
			new_command = TRUE;
			command_flag = 0;
		}

		// execute motions
		executeMotionSequence();
		
    } // end of main command loop

}