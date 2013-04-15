/*
 * This file is part of the MCBMini firmware.
 * MCBMini is a complete, open-source, flexible and scalable
 * motor control scheme with board designs, firmware and host
 * software.
 * This is the firmware for MCBMini
 * The MCBMini project can be downloaded from:
 * http://code.google.com/p/mcbmini/
 *
 * (c) Sigurdur Orn Adalgeirsson (siggi@alum.mit.edu)
 *
 * MCBMini firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License
 *
 * MCBMini firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the MCBMini firmware.  If not, see <http://www.gnu.org/licenses/>.
 */

// MCBMini
//
// Version:	4.4
// Author:	Sigurdur Orn Adalgeirsson
// Email:	siggi@mit.edu
//
// Date:	13 des. '12
//

#define FIRMWARE_VERSION 	32

#include "circBuffer.h"
#include "mcb_mini.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include "motor.h"
#include "controller.h"

// Declaring it static will zero everything
volatile Controller controller[2];
volatile Motor motor[2];

int main(void){
	// Set stack pointer to top of RAM (MUST HAPPEN BEFORE ANY FUNCTION CALL)
	SPH = (uint8_t)(RAMEND >> 8);
	SPL = (uint8_t)(RAMEND);

	// Use Watchdog timer to reset ourselves if we ever get into trouble
	wdt_enable(WDTO_250MS);
	wdt_reset();

	uint8_t m, i;

	int32_t temp;

	a2d_value_ready_flag = 0;

	// Init avr (pin directions etc.)
	avrInit();

	// Init board settings before we get parameters
	initMotor(&motor[0], 0);
	ctrlInitState(&controller[0], &motor[0], target_buffer_data_A, TARGET_BUFFER_SIZE, actual_buffer_data_A, ACTUAL_BUFFER_SIZE);
	setEXTRAMode( 0, EXTRA_MODE_ANALOG );

	initMotor(&motor[1], 1);
	ctrlInitState(&controller[1], &motor[1], target_buffer_data_B, TARGET_BUFFER_SIZE, actual_buffer_data_B, ACTUAL_BUFFER_SIZE);
	setEXTRAMode( 1, EXTRA_MODE_ANALOG );

	changeLEDMode( LED_MODE_ALL_ON );

	timeout_timer = 0;

	id = 255; // Start with an invalid ID until we read our own

	/*
	 * Begin the control loop !
	 */
	while(1){
        wdt_reset();
		loop_count++;

		/*
		 * 25 iterations into the loop (should be about 250ms) we read our ID
		 * This is to only read the ID after any startup electrical transients have settled
		 */
		if( loop_count == 25 && id == 255 ){
			ATOMIC_BLOCK(ATOMIC_FORCEON){
				readID();
			}
		}

		//
		// Do a bunch of household processing while we wait for our timer to run up
		//
		do{
			// Handle a2d conversion
			if( a2d_value_ready_flag != 0 ){
				handleA2DConversionReady();
				a2d_value_ready_flag = 0;
			}

			// While we are waiting, we also process package buffers
			if( isPackageBufferReady() == 1 ){
				processPackageBuffer();
				// Mark as handled
				circBufferReset(&incoming_buffers[package_buf_index]);
				package_buf_index = ( package_buf_index+1 )%NR_BUFFERS;
			}

			// Check to see if we need to set new ID, this takes a while
			ATOMIC_BLOCK(ATOMIC_FORCEON){
				if( should_change_id != 0 ){
					changeID(new_id);
					should_change_id = 0;
				}
			}
		} while( TCNT0 < pid_update_period );
		TCNT0 = 0;

		// Check to see if we have a comm timeout
		if( timeout_timer > 250 &&  timeout_timer != 255 ){
			controller[0].enable = ENABLE_OFF;
			controller[1].enable = ENABLE_OFF;
			SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_TIMEOUT );
			timeout_timer = 255;	// Set it to a fixed value so that this only happens once
		}

		// Only start timer when either board has been initialized
		if( timeout_timer != 255 && (controller[0].initialized || controller[1].initialized) ){
			timeout_timer++;
		}

		updateLEDState();

		// Here we check to see if we need to update our a2d conversion values
		// This should only be false in some error condition because A2D_ITERATIONS is set so this will always be ready by this time
		if( a2d_counter >= A2D_ITERATIONS ){
			for( m=0; m<2; m++ ){
				motor[m].actual_pot = (int32_t)motor[m].current_pot_acc >> A2D_ITERATIONS_DIV2;
				motor[m].motor_current = (int32_t)motor[m].motor_current_acc  >> A2D_ITERATIONS_DIV2;
				motor[m].current_pot_acc = 0;
				motor[m].motor_current_acc = 0;

				motor[m].extra_analog = (int32_t)motor[m].extra_analog_acc >> A2D_ITERATIONS_DIV2;
				motor[m].extra_analog_acc = 0;
			}

			// Start conversion
			SETBIT(ADCSRA, ADIF);
			SETBIT(ADCSRA, ADSC);

			a2d_counter = 0;
		}

		/*
		 * The start of the actual control loop
		 */
		for( m=0; m<2; m++ )
		{
			// First we update our current encoder value from the fast 16bit interrupt routines
			int16_t temp2;
			ATOMIC_BLOCK(ATOMIC_FORCEON){
				temp2 = motor[m].encoder_increment;
				motor[m].encoder_increment = 0;
			}
			motor[m].actual_enc = LIMIT( motor[m].actual_enc + (int32_t)temp2, LONG_MIN, LONG_MAX );

			// And then we do the actual PID calculations
			ctrlCalculateOutput(&controller[m]);

			/*
			 * And if our channel is enabled then we actually apply these values to the output
			 */
			if( controller[m].enable == ENABLE_ON ){
				// Set the direction of the motor
				if( controller[m].output_direction == OUTPUT_DIRECTION_CW ) changeDirection(&motor[m], DIRECTION_CW);
				else if( controller[m].output_direction == OUTPUT_DIRECTION_CCW ) changeDirection(&motor[m], DIRECTION_CCW);

				// Here we do the slow start filtering
				if( motor[m].slow_enable_step_counter > 0 && motor[m].slow_enable_step > 0 ){
					temp = controller[m].output >> 5; // This is a 32nd part of the temp signal

					for(i=0; i<motor[m].slow_enable_step; i++){
						controller[m].output -= temp;
					}

					motor[m].slow_enable_step_counter--;
					if( motor[m].slow_enable_step_counter == 0 ){
						motor[m].slow_enable_step_counter = motor[m].slow_enable_step_time;
						motor[m].slow_enable_step--;
					}
				}

				if( m == 0 ){	M1_PWM = (uint16_t)controller[m].output; }
				else		{	M2_PWM = (uint16_t)controller[m].output; }
			}
			// Disabled
			else {
				if( m == 0 ){	M1_PWM = 0; }
				else		{	M2_PWM = 0; }
			}
		}
	}

	return 0;
}

void changeLEDMode(unsigned char mode){
	if( mode >= LED_NR_STATES || mode == led_mode) return;
	led_mode = mode;
	led_state = 0;
	led_internal_counter = 0;
}

void updateLEDState(){
	unsigned char on = 0;
	if( led_mode == LED_MODE_ALL_ON ){
		on = 1;
	}
	else if( led_mode == LED_MODE_BLINK_1 ){
		if( (led_state >> 3) % 2 == 0 ) on = 1;
	}
	else if( led_mode == LED_MODE_BLINK_2 ){
		if( (led_state >> 2) % 2 == 0 ) on = 1;
	}
	else if( led_mode == LED_MODE_BLINK_3 ){
		if( (led_state >> 1) % 2 == 0 ) on = 1;
	}
	else if( led_mode == LED_MODE_PULSE_1 ){
		if( led_state == 0 ) on = 1;
	}
	else if( led_mode == LED_MODE_PULSE_2 ){
		if( led_state == 0 || led_state == 3 ) on = 1;
	}
	else if( led_mode == LED_MODE_PULSE_3 ){
		if( led_state == 0 || led_state == 3 || led_state == 6 ) on = 1;
	}
	else if( led_mode == LED_MODE_PULSE_4 ){
		if( led_state == 0 || led_state == 3 || led_state == 6 || led_state == 9 ) on = 1;
	}

	if( on == 1 ) LED_ON;
	else LED_OFF;

	led_internal_counter += 1;
	if( led_internal_counter == LED_MAX_COUNT ){
		led_internal_counter = 0;
		led_state = (led_state+1)%LED_STATE_SIZE;
	}
}


void changeID(uint8_t new_id_val){
	uint8_t i, start, read;

	// This is a paranoid check to make sure we don't get into this function through some weird mechanism
	if( should_change_id != 123){
		return;
	}

	// Write the ID and validate (do maximum 10 times, don't want infinite EEPROM loop)

	read = INVALID_ID;
	for(start=10; start<50; start+=10){
		for(i=0; i<10; i++){
			writeIDInternal(start, new_id_val);
			read = readIDInternal(start);
			if( read == new_id_val ){
				id = new_id_val;
				break;
			}
		}
	}
}

void readID(){
	uint8_t i, start;

	// Write the ID and validate (do maximum 10 times, don't want infinite EEPROM loop)
	id = INVALID_ID;
	for(start=10; start<50; start+=10){
		for(i=0; i<10; i++){
			id = readIDInternal(start);
			if( id != INVALID_ID ){
				break;
			}
		}
		if( id != INVALID_ID ){
			break;
		}
	}
}

void writeIDInternal(uint16_t location, uint8_t new_id_val){
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(location), 1);
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(location+1), 2);
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(location+2), 3);
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(location+3), 4);
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(location+4), new_id_val);
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(location+5), new_id_val+10);

	// Park the EEAR
	EEAR = 100;
}

uint8_t readIDInternal(uint16_t start){
	uint8_t byte1, byte2, byte3, byte4;

	// Read the header
	eeprom_busy_wait();
	byte1 = eeprom_read_byte((uint8_t*)(start));
	eeprom_busy_wait();
	byte2 = eeprom_read_byte((uint8_t*)(start+1));
	eeprom_busy_wait();
	byte3 = eeprom_read_byte((uint8_t*)(start+2));
	eeprom_busy_wait();
	byte4 = eeprom_read_byte((uint8_t*)(start+3));

	if( byte1 == 1 && byte2 == 2 && byte3 == 3 && byte4 == 4 ){
		eeprom_busy_wait();
		byte1 = eeprom_read_byte((uint8_t*)(start+4));
		eeprom_busy_wait();
		byte2 = eeprom_read_byte((uint8_t*)(start+5));

		// Redundancy check
		if( byte2 == (byte1 + 10) ){
			byte4 = byte1;	// Now we accept the id
		}
		else{
			byte4 = INVALID_ID;
		}
	}
	else{
		byte4 = INVALID_ID;
	}

	// Park the EEAR
	EEAR = 100;

	return byte4;
}


//
// Comm functions
//

uint8_t isPackageBufferReady(void){
	uint8_t ret;

	ATOMIC_BLOCK(ATOMIC_FORCEON){
		if( rx_buf_index == package_buf_index ){
			ret = 0;
		}
		else{
			ret = 1;
		}
	}
	return ret;
}


void addIntToTxBufferReversed(int32_t in){
	// Flipped ordering because the buffer is reversed
	addByteToTxBuffer( (in) & 0xff);
	addByteToTxBuffer( (in>>8) & 0xff);
	addByteToTxBuffer( (in>>16) & 0xff);
	addByteToTxBuffer( (in>>24) & 0xff);
}

void addCMDByteToTxBuffer(uint8_t byte){
//	ATOMIC_BLOCK(ATOMIC_FORCEON){
//		if( msg_buffer.length >= 3 ){
//			SETBIT(byte, 7);
//		}
//	}
	addByteToTxBuffer(byte);
}

void addByteToTxBuffer(uint8_t byte){
	write_checksum += byte;
	if( byte == HEADER_BYTE || byte == ESCAPE_BYTE ){
		byte ^= 1;
		circBufferPut(&tx_buffer, ESCAPE_BYTE);
	}
	circBufferPut(&tx_buffer, byte);
}

void clearTxBuffer(){
	circBufferReset( &tx_buffer );
	write_checksum = 0;
}

void sendTxBuffer(uint8_t motor){

	if( tx_buffer.length > 0 ){
		// wait for the transmitter to be ready
		while(tx_ready == FALSE) ;

		ENABLE_TX;
		addByteToTxBuffer( (motor<<7) | id);
		addByteToTxBuffer(write_checksum);
		circBufferPut(&tx_buffer, HEADER_BYTE);


		// We are now holding on to the line until we know for sure that another board should speak
		//		ENABLE_TX;

		// send byte
		UDR0 = circBufferGetFirst(&tx_buffer);
		// set ready state to FALSE
		tx_ready = FALSE;

		write_checksum = 0;
	}
}

signed long readIntFromEndReversed(circBuffer* buffer){
	unsigned char hh, hl, lh, ll;

	hh = circBufferGetLast(buffer);
	hl = circBufferGetLast(buffer);
	lh = circBufferGetLast(buffer);
	ll = circBufferGetLast(buffer);

	return ((int32_t)hh<<24) + ((int32_t)hl<<16) + ((int32_t)lh<<8) + (int32_t)ll;
}

void processPackageBuffer(){

	circBuffer* package_buf;
	int32_t new_target;
	uint8_t cmd, channel, request_response, new_val;
	uint8_t can_override_response = 0;

	package_buf = &incoming_buffers[package_buf_index];

	//
	// Here we begin really processing the data in the packet
	// Assuming only one command in each buffer
	//
	request_response = 0;
	cmd = circBufferGetLast(package_buf);				// Get the id
	channel = cmd >> 7;							// Get the channel bit
	cmd = cmd & 0b01111111;						// Mask the channel bit

	if( cmd != id && cmd != BCAST_ID ){
		return;
	}

	// Set the blink fast mode when we receive our first package
	if( rx_have_received_package == 0 ){
		changeLEDMode( LED_MODE_BLINK_1 );
		rx_have_received_package = 1;
	}

	// Only reset timer in case of our ID (not broadcast)
	if( cmd == id ) timeout_timer = 0;

	cmd = circBufferGetLast(package_buf);
	if(BITSET(cmd, 7)) request_response = 1;	// Extract the response bit
	cmd = cmd & 0b01111111;						// Mask the response bit

	switch( cmd ){

	case CMD_2TARGET_TICK_ACTUAL:
	case CMD_2TARGET_TICK_MOTOR_CURRENT:
	case CMD_2TARGET_TICK_VELOCITY:
	case CMD_2TARGET_TICK_POT:
	case CMD_2TARGET_TICK_ENCODER:
	case CMD_2TARGET_TICK_2ACTUAL:
	case CMD_2TARGET_TICK_2VELOCITY:
	case CMD_2TARGET_TICK_2MOTOR_CURRENT:
	case CMD_2TARGET_TICK_2POT:
	case CMD_2TARGET_TICK_2ENCODER:
		can_override_response = 1;

		for(new_val=0; new_val<2; new_val++){
			new_target = readIntFromEndReversed(package_buf);
			if( new_target != LONG_MAX ){
				circBufferPutLong(&controller[new_val].target_buffer, new_target);
			}
			if( !controller[new_val].initialized && !controller[new_val].notified_initialized ) {
				controller[new_val].notified_initialized = TRUE;
				if( new_val == 0 ){
					SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_UNINITIALIZED_A );
				}
				else{
					SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_UNINITIALIZED_B );
				}
			}
		}

		switch( cmd ){
		case CMD_2TARGET_TICK_ACTUAL:
			if( controller[channel].feedback_mode == FEEDBACK_MODE_POT ){
				if( controller[channel].initialized ){
					addIntToTxBufferReversed(motor[channel].actual_pot);
				}
				else{
					addIntToTxBufferReversed(LONG_MAX);
				}
			}
			else{
				if( controller[channel].initialized ){
					addIntToTxBufferReversed( motor[channel].actual_enc );
				}
				else{
					addIntToTxBufferReversed(LONG_MAX);
				}
			}
			break;
		case CMD_2TARGET_TICK_2ACTUAL:
			if( controller[1].initialized ){
				if( controller[1].feedback_mode == FEEDBACK_MODE_POT ){
					addIntToTxBufferReversed(motor[1].actual_pot);
				}
				else{
					addIntToTxBufferReversed(motor[1].actual_enc);
				}
			}
			else{
				addIntToTxBufferReversed(LONG_MAX);
			}
			if( controller[0].initialized ){
				if( controller[0].feedback_mode == FEEDBACK_MODE_POT ){
					addIntToTxBufferReversed(motor[0].actual_pot);
				}
				else{
					addIntToTxBufferReversed(motor[0].actual_enc);
				}
			}
			else{
				addIntToTxBufferReversed(LONG_MAX);
			}
			break;
		case CMD_2TARGET_TICK_2VELOCITY:
			addIntToTxBufferReversed(controller[1].actual_tick_diff);
			addIntToTxBufferReversed(controller[0].actual_tick_diff);
			break;
		case CMD_2TARGET_TICK_2MOTOR_CURRENT:
			addIntToTxBufferReversed(motor[1].motor_current);
			addIntToTxBufferReversed(motor[0].motor_current);
			break;
		case CMD_2TARGET_TICK_2POT:
			addIntToTxBufferReversed(motor[1].actual_pot);
			addIntToTxBufferReversed(motor[0].actual_pot);
			break;
		case CMD_2TARGET_TICK_2ENCODER:
			addIntToTxBufferReversed(motor[1].actual_enc);
			addIntToTxBufferReversed(motor[0].actual_enc);
			break;
		case CMD_2TARGET_TICK_MOTOR_CURRENT:
			addIntToTxBufferReversed(motor[channel].motor_current);
			break;
		case CMD_2TARGET_TICK_VELOCITY:
			addIntToTxBufferReversed(controller[channel].actual_tick_diff);
			break;
		case CMD_2TARGET_TICK_POT:
			addIntToTxBufferReversed(motor[channel].actual_pot);
			break;
		case CMD_2TARGET_TICK_ENCODER:
			addIntToTxBufferReversed(motor[channel].actual_enc);
			break;
		}
		addCMDByteToTxBuffer(cmd);
		break;

	case CMD_TARGET_TICK:
		if( request_response==1 ){
			addIntToTxBufferReversed(circBufferPeekLastLong(&controller[channel].target_buffer));
			addCMDByteToTxBuffer(cmd);
		}
		else{
			if( !controller[channel].initialized && !controller[channel].notified_initialized ) {
				controller[channel].notified_initialized = TRUE;
				if( channel == 0 ){
					SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_UNINITIALIZED_A );
				}
				else{
					SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_UNINITIALIZED_B );
				}
			}
			new_target = readIntFromEndReversed(package_buf);
			if( new_target != LONG_MAX ){
				circBufferPutLong(&controller[channel].target_buffer, new_target);
			}
		}
		break;

	case CMD_POS_P_GAIN:
		if( request_response==1 ){
			addIntToTxBufferReversed((int32_t)controller[channel].pid.p_gain);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].pid.p_gain = (uint16_t)readIntFromEndReversed(package_buf);
		}
		break;

	case CMD_VEL_P_GAIN:
		if( request_response==1 ){
			addIntToTxBufferReversed((int32_t)controller[channel].pid_vel.p_gain);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].pid_vel.p_gain = (uint16_t)readIntFromEndReversed(package_buf);
		}
		break;

	case CMD_POS_I_GAIN:
		if( request_response==1 ){
			addIntToTxBufferReversed((int32_t)controller[channel].pid.i_gain);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].pid.i_gain = (uint16_t)readIntFromEndReversed(package_buf);
		}
		break;

	case CMD_VEL_I_GAIN:
		if( request_response==1 ){
			addIntToTxBufferReversed((int32_t)controller[channel].pid_vel.i_gain);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].pid_vel.i_gain = (uint16_t)readIntFromEndReversed(package_buf);
		}
		break;

	case CMD_POS_D_GAIN:
		if( request_response==1 ){
			addIntToTxBufferReversed((int32_t)controller[channel].pid.d_gain);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].pid.d_gain = (uint16_t)readIntFromEndReversed(package_buf);
		}
		break;

	case CMD_VEL_D_GAIN:
		if( request_response==1 ){
			addIntToTxBufferReversed((int32_t)controller[channel].pid_vel.d_gain);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].pid_vel.d_gain = (uint16_t)readIntFromEndReversed(package_buf);
		}
		break;

	case CMD_MAX_VELOCITY:
		if( request_response==1 ){
			addIntToTxBufferReversed(controller[channel].maximum_vel);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].maximum_vel = readIntFromEndReversed(package_buf);
		}
		break;

	case CMD_MAX_ACCELERATION:
		if( request_response==1 ){
			addIntToTxBufferReversed(controller[channel].maximum_acc);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].maximum_acc = readIntFromEndReversed(package_buf);
		}
		break;

	case CMD_VEL_TIME_DELTA:
		if( request_response==1 ){
			addByteToTxBuffer(controller[channel].actual_buffer.size>>2);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			new_val = circBufferGetLast(package_buf);
			new_val = LIMIT(new_val, 2, 5);
			controller[channel].actual_buffer.size = new_val<<2;
			circBufferReset(&controller[channel].actual_buffer);
		}
		break;

	case CMD_POS_DOWNSCALE:
		if( request_response==1 ){
			addByteToTxBuffer(controller[channel].pid.downscale);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].pid.downscale = circBufferGetLast(package_buf);
		}
		break;

	case CMD_VEL_DOWNSCALE:
		if( request_response==1 ){
			addByteToTxBuffer(controller[channel].pid_vel.downscale);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].pid_vel.downscale = circBufferGetLast(package_buf);
		}
		break;

	case CMD_ENABLE:
		if( request_response==1 ){
			addByteToTxBuffer(controller[channel].enable);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			new_val = circBufferGetLast(package_buf);
			// If we are enabling from a disable then we set the enable_slow_counter
			if( controller[channel].enable == ENABLE_OFF && new_val == ENABLE_ON ){
				// Initialize the slow enable
				motor[channel].slow_enable_step = SLOW_ENABLE_STEPS;
				motor[channel].slow_enable_step_counter = motor[channel].slow_enable_step_time;
				ctrlClearState(&controller[channel]);
			}

			controller[channel].initialized = 1;	// This assumes that all initial parameters have been set already and enable is the last thing to happen
			controller[channel].enable = new_val;

			// Set the blinking option
			if( controller[0].enable == ENABLE_ON && controller[1].enable == ENABLE_ON ){
				changeLEDMode( LED_MODE_BLINK_3 );
			}
			else if( controller[1].enable == ENABLE_ON ){
				changeLEDMode( LED_MODE_BLINK_2 );
			}
			else if( controller[0].enable == ENABLE_ON ){
				changeLEDMode( LED_MODE_BLINK_2 );
			}
			else {
				changeLEDMode( LED_MODE_BLINK_1 );
			}
		}
		break;

	case CMD_POLARITY:
		if( request_response==1 ){
			addByteToTxBuffer(controller[channel].polarity);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].polarity = circBufferGetLast(package_buf);
		}
		break;

	case CMD_FEEDBACK_MODE:
		if( request_response==1 ){
			addByteToTxBuffer(controller[channel].feedback_mode);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			ctrlChangeFeedback(&controller[channel], circBufferGetLast(package_buf));
		}
		break;

	case CMD_STREAM_MODE:
		if( request_response==1 ){
			addByteToTxBuffer(controller[channel].stream_mode);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			controller[channel].stream_mode = circBufferGetLast(package_buf);
		}
		break;

	case CMD_CONTROL_MODE:
		if( request_response==1 ){
			addByteToTxBuffer(controller[channel].control_mode);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			ctrlChangeControlMode(&controller[channel], circBufferGetLast(package_buf));
		}
		break;

	case CMD_ID:
		if( request_response==1 ){
			addByteToTxBuffer(id);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			// We do some additional checking to see if this is really an id packet
			// because it is not a very reversible action

			new_val = 0;
			if( circBufferGetLast(package_buf) != 1 ) new_val = 1;
			if( circBufferGetLast(package_buf) != 2 ) new_val = 1;
			if( circBufferGetLast(package_buf) != 3 ) new_val = 1;

			new_id = circBufferGetLast(package_buf);

			if( new_val == 0 ){
				should_change_id = 123; // Special value for paranoid checking
			}
			else{
				addByteToTxBuffer(new_val);
				addByteToTxBuffer(ERROR_BAD_ID_PACKET);
				addByteToTxBuffer(CMD_ERROR);
			}

			for(new_val = 0; new_val < 2; new_val++){
				if( controller[new_val].enable == ENABLE_ON){
					addByteToTxBuffer(new_val);
					addByteToTxBuffer(ERROR_SET_PARAM_DURING_ENABLE);
					addByteToTxBuffer(CMD_ERROR);
				}
				controller[new_val].enable = ENABLE_OFF;
			}
		}
		break;

	case CMD_ACTUAL_TICK:
		if( request_response==1 ){
			if( controller[channel].feedback_mode == FEEDBACK_MODE_POT ){
				addIntToTxBufferReversed(motor[channel].actual_pot);
			}
			else{
				addIntToTxBufferReversed(motor[channel].actual_enc);
			}
			addCMDByteToTxBuffer(cmd);
		}
		else{
			// Assume that this always means ticks because it wouldnt make any sense to sense pot values as they are absolute and measured on every update
			motor[channel].actual_enc = readIntFromEndReversed(package_buf);
			ctrlClearState(&controller[channel]);
		}
		break;

	case CMD_OFFSET_ENCODER_TICK:
		if( request_response==1 ){
			;
		}
		else{
			// Assume that this always means ticks because it wouldnt make any sense to sense pot values as they are absolute and measured on every update
			new_target = readIntFromEndReversed(package_buf);
			motor[channel].actual_enc = LIMIT(motor[channel].actual_enc + new_target, LONG_MIN, LONG_MAX);
			ctrlClearState(&controller[channel]);
		}
		break;

	case CMD_MOTOR_CURRENT:
		if( request_response==1 ){
			addIntToTxBufferReversed(motor[channel].motor_current);
			addCMDByteToTxBuffer(cmd);
		}
		break;

	case CMD_ACTUAL_VEL:
		if( request_response==1 ){
			addIntToTxBufferReversed(controller[channel].actual_tick_diff);
			addCMDByteToTxBuffer(cmd);
		}
		break;

	case CMD_PID_OUTPUT:
		if( request_response==1 ){
			addIntToTxBufferReversed(controller[channel].output);
			addCMDByteToTxBuffer(cmd);
		}
		break;

	case CMD_ENCODER_VALUE:
		if( request_response==1 ){
			addIntToTxBufferReversed( motor[channel].actual_enc );
			addCMDByteToTxBuffer(cmd);
		}
		else{
			motor[channel].actual_enc = readIntFromEndReversed(package_buf);
			ctrlClearState(&controller[channel]);
		}
		break;

	case CMD_POT_VALUE:
		if( request_response==1 ){
			addIntToTxBufferReversed(motor[channel].actual_pot);
			addCMDByteToTxBuffer(cmd);
		}
		break;

	case CMD_FIRMWARE_VERSION:
		if( request_response==1 ){
			addIntToTxBufferReversed( FIRMWARE_VERSION );
			addCMDByteToTxBuffer(cmd);
		}
		break;

	case CMD_MAX_PWM_DUTY_CYCLE:
		if( request_response==1 ){
			addIntToTxBufferReversed((int32_t)controller[channel].maximum_pwm);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			new_target = MIN(ABS(readIntFromEndReversed(package_buf)), (int32_t)FULL_PWM);
			controller[channel].maximum_pwm = (uint16_t)new_target;

			if( controller[channel].control_mode == CONTROL_MODE_POS ){
				controller[channel].pid.max_output = controller[channel].maximum_pwm;
			}
			else{
				controller[channel].pid_vel.max_output = controller[channel].maximum_pwm;
			}
		}
		break;

	case CMD_SLOW_ENABLE_TIME:
		if( request_response==1 ){
			addByteToTxBuffer(motor[channel].slow_enable_step_time);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			motor[channel].slow_enable_step_time = circBufferGetLast(package_buf);
		}
		break;

	case CMD_SATURATION:
		if( request_response==1 ){
			addByteToTxBuffer( controller[channel].pid.saturation );
			addCMDByteToTxBuffer(cmd);
		}
		break;

	case CMD_I_COMPONENT:
		if( request_response==1 ){
			addIntToTxBufferReversed( controller[channel].pid.i_component );
			addCMDByteToTxBuffer(cmd);
		}
		break;

	case CMD_PID_UPDATE_PERIOD:
		if( request_response==1 ){
			addByteToTxBuffer( pid_update_period );
			addCMDByteToTxBuffer(cmd);
		}
		else{
			pid_update_period = circBufferGetLast(package_buf);
		}
		break;

	case CMD_REQUEST_MESSAGE:
		if( request_response==1 ){
//			if( msg_buffer.length != 0 ){
//				should_send_message = 1;
//			}
//			else{
				addCMDByteToTxBuffer(CMD_EMPTY_RESPONSE);
//			}
		}
		break;

	case CMD_EXTRA_MODE:
		if( request_response==1 ){
			addByteToTxBuffer(motor[channel].extra_mode);
			addCMDByteToTxBuffer(cmd);
		}
		else{
			setEXTRAMode(channel, circBufferGetLast(package_buf));
		}
		break;

	case CMD_EXTRA_VALUE:
		if( request_response==1 ){
			if( motor[channel].extra_mode == EXTRA_MODE_SWITCH ){
				addIntToTxBufferReversed((int32_t)motor[channel].extra_switch);
			}
			else if( motor[channel].extra_mode == EXTRA_MODE_ANALOG ){
				addIntToTxBufferReversed((int32_t)motor[channel].extra_analog);
			}
			else if( motor[channel].extra_mode == EXTRA_MODE_SERVO ){
				addIntToTxBufferReversed((int32_t)(motor[channel].extra_servo&0xff));
			}
			else if( motor[channel].extra_mode == EXTRA_MODE_OFF ){
				addIntToTxBufferReversed((int32_t)-1);
			}
			addCMDByteToTxBuffer(cmd);
		}
		else{
			if( motor[channel].extra_mode == EXTRA_MODE_SERVO ){
				new_target = readIntFromEndReversed(package_buf);
				ATOMIC_BLOCK(ATOMIC_FORCEON){
					motor[channel].extra_servo = (uint8_t)LIMIT(new_target, 0, 255);
				}
			}
		}
		break;

	case CMD_EMPTY_RESPONSE:
		addCMDByteToTxBuffer(CMD_EMPTY_RESPONSE);
		break;

	default:
		// Wrong command
		addByteToTxBuffer(cmd);
		addByteToTxBuffer(ERROR_BAD_CMD_RECEIVED);
		addByteToTxBuffer(CMD_ERROR);
	}

	/*
	 * Handle various messages
	 */
	if( can_override_response || (tx_buffer.length == 0 && cmd != CMD_ID) ){

		if( BITSET( RX_FLAGS, FLAG_SHOULD_SEND_EXTRA_VAL_A ) ){
			clearTxBuffer();
			addByteToTxBuffer(motor[0].extra_switch);
			addByteToTxBuffer(CMD_EXTRA_VALUE);
			channel = 0;
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_SEND_EXTRA_VAL_A );
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_SEND_EXTRA_VAL_B ) ){
			clearTxBuffer();
			addByteToTxBuffer(motor[1].extra_switch);
			addByteToTxBuffer(CMD_EXTRA_VALUE);
			channel = 1;
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_SEND_EXTRA_VAL_B );
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_NOTIFY_TIMEOUT ) ){
			clearTxBuffer();
			addByteToTxBuffer(CMD_EMPTY_RESPONSE); // Start by sending an empty reply to flush a bad packet in host if it got corrupt during reset or whatever
			sendTxBuffer(channel);
			addByteToTxBuffer(ERROR_TIMEOUT_DISABLE);
			addByteToTxBuffer(CMD_ERROR);
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_TIMEOUT );
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_NOTIFY_FAULT_A ) ){
			clearTxBuffer();
			addByteToTxBuffer(ERROR_FAULT);
			addByteToTxBuffer(CMD_ERROR);
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_FAULT_A );
			channel = 0;
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_NOTIFY_FAULT_B ) ){
			clearTxBuffer();
			addByteToTxBuffer(ERROR_FAULT);
			addByteToTxBuffer(CMD_ERROR);
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_FAULT_B );
			channel = 1;
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_NOTIFY_BUFFER_OVERFLOW ) ){
			clearTxBuffer();
			addByteToTxBuffer(ERROR_BUFFER_OVERFLOW);
			addByteToTxBuffer(CMD_ERROR);
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_BUFFER_OVERFLOW );
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_NOTIFY_PACKET_OVERFLOW ) ){
			clearTxBuffer();
			addByteToTxBuffer(ERROR_PACKET_OVERFLOW);
			addByteToTxBuffer(CMD_ERROR);
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_PACKET_OVERFLOW );
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_NOTIFY_BAD_CHECKSUM ) ){
			clearTxBuffer();
			addByteToTxBuffer(ERROR_BAD_CHECKSUM);
			addByteToTxBuffer(CMD_ERROR);
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_BAD_CHECKSUM );
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_NOTIFY_UNINITIALIZED_A ) ){
			clearTxBuffer();
			addByteToTxBuffer(ERROR_UNINITIALIZED);
			addByteToTxBuffer(CMD_ERROR);
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_UNINITIALIZED_A );
			channel = 0;
		}
		else if( BITSET( RX_FLAGS, FLAG_SHOULD_NOTIFY_UNINITIALIZED_B ) ){
			clearTxBuffer();
			addByteToTxBuffer(ERROR_UNINITIALIZED);
			addByteToTxBuffer(CMD_ERROR);
			CLEARBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_UNINITIALIZED_B );
			channel = 1;
		}
	}

	//
	// Send the message
	//
	sendTxBuffer(channel);

	return;
}

void setEXTRAMode(uint8_t channel, uint8_t mode){
	if( motor[channel].extra_mode == mode ){
		return;
	}
	motor[channel].extra_mode = mode;

	// If either mode is servo mode then we turn on the timer interrupts
	if( motor[0].extra_mode == EXTRA_MODE_SERVO || motor[1].extra_mode == EXTRA_MODE_SERVO ){
		SERVO_TIMER_ENABLE;
	}
	else{
		SERVO_TIMER_DISABLE;
	}

	if( mode == EXTRA_MODE_SWITCH ){
		if( channel == 0 ){
			CLEARBIT(EXTRA1_DDR, EXTRA1);
			SETBIT(EXTRA1_PORT, EXTRA1);
			SETBIT(PCMSK1, EXTRA1);
			motor[channel].extra_switch = BITVAL(EXTRA1_PIN, EXTRA1);
			// Send the current level of the switch
			SETBIT( RX_FLAGS, FLAG_SHOULD_SEND_EXTRA_VAL_A );
		}
		else{
			CLEARBIT(EXTRA2_DDR, EXTRA2);
			SETBIT(EXTRA2_PORT, EXTRA2);
			SETBIT(PCMSK1, EXTRA2);
			motor[channel].extra_switch = BITVAL(EXTRA2_PIN, EXTRA2);
			// Send the current level of the switch
			SETBIT( RX_FLAGS, FLAG_SHOULD_SEND_EXTRA_VAL_B );
		}
	}
	else if( mode == EXTRA_MODE_ANALOG ){
		if( channel == 0 ){
			CLEARBIT(PCMSK1, EXTRA1);
			CLEARBIT(EXTRA1_DDR, EXTRA1);
			CLEARBIT(EXTRA1_PORT, EXTRA1);
		}
		else{
			CLEARBIT(PCMSK1, EXTRA2);
			CLEARBIT(EXTRA2_DDR, EXTRA2);
			CLEARBIT(EXTRA2_PORT, EXTRA2);
		}
	}
	else if( mode == EXTRA_MODE_SERVO ){
		if( channel == 0 ){
			CLEARBIT(PCMSK1, EXTRA1);
			SETBIT(EXTRA1_DDR, EXTRA1);
			CLEARBIT(EXTRA1_PORT, EXTRA1);
		}
		else{
			CLEARBIT(PCMSK1, EXTRA2);
			SETBIT(EXTRA2_DDR, EXTRA2);
			CLEARBIT(EXTRA2_PORT, EXTRA2);
		}
	}
}

void handleA2DConversionReady(){

	// Here we iterate through our 4 analog channels and accumulate their values over a A2D_ITERATIONS number of times
	// Then in the main loop we check to see if those iterations are done and then divide by A2D_ITERATIONS to

	if( a2d_index == 0 ){
		motor[0].current_pot_acc += a2d_value;
		setADCChannel(M2_POT);
	}
	else if( a2d_index == 1 ){
		motor[1].current_pot_acc += a2d_value;
		setADCChannel(M1_CS);
	}
	else if( a2d_index == 2 ){
		motor[0].motor_current_acc += a2d_value;
		setADCChannel(M2_CS);
	}
	else if( a2d_index == 3 ){
		motor[1].motor_current_acc += a2d_value;
		setADCChannel(EXTRA1);
	}
	else if( a2d_index == 4 ){
		motor[0].extra_analog_acc += a2d_value;
		setADCChannel(EXTRA2);
	}
	else if( a2d_index == 5 ){
		motor[1].extra_analog_acc += a2d_value;

		setADCChannel(M1_POT);
		a2d_counter += 1;
	}

	a2d_index = (a2d_index + 1) % 6;
	if( a2d_counter < A2D_ITERATIONS ){
		// Start conversion
		SETBIT(ADCSRA, ADIF);
		SETBIT(ADCSRA, ADSC);
	}
}

void avrInit(void)
{
	// Set everything as inputs
	DDRB = 0b00000000;	// Data direction
	DDRC = 0b00000000;	// Data direction
	DDRD = 0b00000000;	// Data direction

	// Set the motor control pins as outputs
	SETBIT(M1_IN_A_DDR, M1_IN_A);
	SETBIT(M1_IN_B_DDR, M1_IN_B);
	SETBIT(M2_IN_A_DDR, M2_IN_A);
	SETBIT(M2_IN_B_DDR, M2_IN_B);

	// Set diagnostic pins as inputs
	CLEARBIT(M1_DIAG_A_DDR, M1_DIAG_A);
	//	CLEARBIT(M1_DIAG_B_DDR, M1_DIAG_B);
	CLEARBIT(M2_DIAG_A_DDR, M2_DIAG_A);
	//	CLEARBIT(M2_DIAG_B_DDR, M2_DIAG_B);
	// no pull-ups because we have external ones

	// Set up interrupts on the diagnostic pins
	SETBIT(PCICR, PCIE0);

	SETBIT(PCMSK0, M1_DIAG_A);
	//	SETBIT(PCMSK0, M1_DIAG_B);
	SETBIT(PCMSK0, M2_DIAG_A);
	//	SETBIT(PCMSK0, M2_DIAG_B);

	// Enable interrupts for the extra pins
	SETBIT(PCICR, PCIE1);

	// Set the PWM as outputs
	SETBIT(DDRB, 1);
	SETBIT(DDRB, 2);

	/*
	 * We start by enabling interrupts early and especially the encoder ones so that we will be able to track
	 * encoder ticks during startup if we are in a reset condition
	 */

	// Globally enable interrupts
	sei();

	//
	// Init encoders
	//

	// Set extra pins as inputs
	CLEARBIT(EXTRA1_DDR, EXTRA1);
	CLEARBIT(EXTRA2_DDR, EXTRA2);

	// Set encoder pins as inputs
	CLEARBIT(M1_ENCA_DDR, M1_ENCA);
	CLEARBIT(M1_ENCB_DDR, M1_ENCB);
	CLEARBIT(M2_ENCA_DDR, M2_ENCA);
	CLEARBIT(M2_ENCB_DDR, M2_ENCB);

	// Enable pullup resistors
	SETBIT(M1_ENCA_PORT, M1_ENCA);
	SETBIT(M1_ENCB_PORT, M1_ENCB);
	SETBIT(M2_ENCA_PORT, M2_ENCA);
	SETBIT(M2_ENCB_PORT, M2_ENCB);

	// Setting interrupts on any logical change of int0 and int1
	CLEARBIT(EICRA, ISC01);	// int0
	SETBIT(EICRA, ISC00);	// int0
	CLEARBIT(EICRA, ISC11);	// int1
	SETBIT(EICRA, ISC10);	// int1

	// Enable the interrupts
	SETBIT(EIMSK, INT0);
	SETBIT(EIMSK, INT1);

	// Sets the bridge into High z state
	M1_PWM = 0;
	M2_PWM = 0;

	// Init uart
	SETBIT(UCSR0B, RXCIE0);
	SETBIT(UCSR0B, TXCIE0);
	SETBIT(UCSR0B, RXEN0);
	SETBIT(UCSR0B, TXEN0);

	// set baud rate
	UBRR0H = (unsigned char)( UBRRVAL >> 8);
	UBRR0L = (unsigned char) UBRRVAL ;

	tx_ready = TRUE;

	// Set tx pin as input normally
	CLEARBIT(DDRD, 1);
	// Enable the tx pullup resistor
	SETBIT(PORTD, 1);
	DISABLE_TX;

	// Init serial command packet buffers
	circBufferInit(&incoming_buffers[0], buffer_data1, BUFFER_SIZE);
	circBufferInit(&incoming_buffers[1], buffer_data2, BUFFER_SIZE);

	circBufferInit(&tx_buffer, tx_buffer_data, BUFFER_SIZE);

	// Init flags
	RX_FLAGS = 0;

	// Init a/d
	SETBIT(ADCSRA, ADEN);				// enable ADC (turn on ADC power)
	SETBIT(ADCSRA, ADIE);				// enable ADC interrupts

	CLEARBIT(ADMUX, REFS1);				// set reference to AVCC
	SETBIT(ADMUX, REFS0);

	SETBIT(ADCSRA, ADPS2);				// set prescaler to 128
	SETBIT(ADCSRA, ADPS1);
	SETBIT(ADCSRA, ADPS0);

	// Set analog pins as inputs (current sense pins aren't digital pins so they don't need this)
	CLEARBIT(M1_POT_DDR, M1_POT);
	CLEARBIT(M2_POT_DDR, M2_POT);

	a2d_index = 0;
	setADCChannel(M1_POT);
	// Start conversion
	SETBIT(ADCSRA, ADIF);
	SETBIT(ADCSRA, ADSC);

	//
	// Init PWM
	//

	// Set non-inverting pwm mode
	SETBIT(TCCR1A,COM1A1);
	CLEARBIT(TCCR1A,COM1A0);
	SETBIT(TCCR1A,COM1B1);
	CLEARBIT(TCCR1A,COM1B0);

	// No prescaling on clock
	CLEARBIT(TCCR1B,CS12);
	CLEARBIT(TCCR1B,CS11);
	SETBIT(TCCR1B,CS10);

	// Fast PWM mode
	SETBIT(TCCR1B, WGM13);
	SETBIT(TCCR1B, WGM12);
	SETBIT(TCCR1A, WGM11);
	CLEARBIT(TCCR1A, WGM10);

	// Sets PWM frequency at about 18kHz
	FULL_PWM = PROGRAMMED_PWM_FREQ;

	// Set up timer 0 for control loop (keeps it at a constant rate)
	SETBIT(TCCR0B,CS02);
	SETBIT(TCCR0B,CS00);

	//
	// Init the fault interrupt (PCINT11)
	//
	SETBIT(PCICR, PCIE1); // Enable pin change interrupt 1
	SETBIT(PCMSK1, PCINT11); // Further tell it only to listen to PCINT11

	/*
	 * Setup servos
	 */
	servo_state = 0;

	sei();
	// Set up the watchdog timer
	//	wdt_enable(WDTO_120MS); 		// 30 ms should be enough to keep up with the 50Hz master update loop, but barely
}

/*
 * Interrupt routines
 */

EMPTY_INTERRUPT(BADISR_vect)

/*
 * For optimization purposes, all function calls have been skipped in this ISR
 * so circBufferGetLast and circBufferPut etc. have all been written in code at the cost
 * of legibility but gaining more compact and efficient ISR operation
 */
ISR(USART_TX_vect, ISR_NOBLOCK){
	uint8_t data;
	// check if there's data left in the buffer
	if( tx_buffer.length > 0 ) {
		// send byte from top of buffer
		// get the first character from buffer
		data = tx_buffer.databuffer[tx_buffer.index];
		tx_buffer.index = (tx_buffer.index+1)%tx_buffer.size;
		tx_buffer.length--;
		// move index down and decrement length

		UDR0 = data;
	}
	else{
		// Reset buffer
		tx_buffer.index = 0;
		tx_buffer.length = 0;
		tx_ready = TRUE;
		DISABLE_TX;
	}
}

/*
 * For optimization purposes, all function calls have been skipped in this ISR
 * so circBufferGetLast and circBufferPut etc. have all been written in code at the cost
 * of legibility but gaining more compact and efficient ISR operation
 */
ISR(USART_RX_vect){
	uint8_t byte = UDR0;
	sei();

	/*
	 * If we received the header which means a full packet is received
	 */
	if( byte == HEADER_BYTE ){
		// Get the received checksum
		incoming_buffers[rx_buf_index].length -= 1;
		byte = incoming_buffers[rx_buf_index].databuffer[incoming_buffers[rx_buf_index].length];

		rx_checksum -= byte;				// checksum_rcv byte was also added to local checksum when it arrived

		// If checksums don't match reject
		if( rx_checksum != byte ){
			//			DISABLE_TX;									// Release the bus

			// Add a message
			SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_BAD_CHECKSUM );

			incoming_buffers[rx_buf_index].length = 0;	// flush
			incoming_buffers[rx_buf_index].index = 0;	// flush
			rx_checksum = 0;							// Reset checksum
			return;
		}

		// Here we create the package buffer
		rx_buf_index = ( rx_buf_index+1 )%NR_BUFFERS;

		// Here we could check to see if rx_buf_index==package_buf_index in which case we are not processing buffers fast enough
		if( rx_buf_index == package_buf_index ){
			// Add a message
			SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_PACKET_OVERFLOW );
		}
		incoming_buffers[rx_buf_index].length = 0;	// flush
		incoming_buffers[rx_buf_index].index = 0;	// flush
		rx_checksum = 0;							// Reset checksum
		return;
	}

	if( byte == ESCAPE_BYTE ){
		SETBIT( RX_FLAGS, FLAG_NEXT_BYTE_SHOULD_BE_TRANSFORMED );
		return;
	}

	if( BITSET( RX_FLAGS, FLAG_NEXT_BYTE_SHOULD_BE_TRANSFORMED ) ){
		byte ^= 1;
		CLEARBIT( RX_FLAGS, FLAG_NEXT_BYTE_SHOULD_BE_TRANSFORMED );
	}

	// Here we add the incoming byte to the current rx buffer
	if( incoming_buffers[rx_buf_index].length < incoming_buffers[rx_buf_index].size ){
		incoming_buffers[rx_buf_index].databuffer[incoming_buffers[rx_buf_index].length] = byte;
		incoming_buffers[rx_buf_index].length++;
		rx_checksum += byte;
	}
	// If there is no room then we fix it
	else{
		incoming_buffers[rx_buf_index].length = 0;	// flush
		incoming_buffers[rx_buf_index].index = 0;	// flush
		rx_checksum = 0;				// Reset checksum

		// Add a message
		SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_BUFFER_OVERFLOW );
	}
}

// Interrupt handler for ADC complete interrupt.
//ISR(SIG_ADC){
ISR(ADC_vect, ISR_NOBLOCK){
	a2d_value = ADCL | (ADCH<<8);			// Cache the value
	a2d_value_ready_flag = 1;				// Notify main process of the new value
}

// This gets called whenever the EXTRA pins are setup as switches and change value
ISR(PCINT1_vect){
	// If it was EXTRA1
	if( BITVAL(EXTRA1_PIN, EXTRA1) != motor[0].extra_switch ){
		motor[0].extra_switch = BITVAL(EXTRA1_PIN, EXTRA1);
		SETBIT( RX_FLAGS, FLAG_SHOULD_SEND_EXTRA_VAL_A );
	}
	if( BITVAL(EXTRA2_PIN, EXTRA2) != motor[1].extra_switch ){
		motor[1].extra_switch = BITVAL(EXTRA2_PIN, EXTRA2);
		SETBIT( RX_FLAGS, FLAG_SHOULD_SEND_EXTRA_VAL_B );
	}
}

// This is called when the bridge shuts down
ISR(PCINT0_vect){
	/*
	 * For now I am choosing to disregard the FAULT_MODE options
	 * Mostly because they applied better to the SA57A bridge and not so much to the new VNH5019A bridge
	 */

	// First we need to figure out which bridge complained
	if( BITCLEAR(M1_DIAG_A_PIN, M1_DIAG_A) /* || BITCLEAR(M1_DIAG_B_PIN, M1_DIAG_B) */ ){
		controller[0].enable = ENABLE_OFF;
		SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_FAULT_A );
//		addMessage1(CMD_ERROR, 0, ERROR_FAULT);

		// Reset the bridge
		M1_PWM = 0;
		M1_STOP_GND;
		_delay_us(100);
		M1_STOP_VCC;
		_delay_us(100);
//				M1_DISABLE;
		changeLEDMode( LED_MODE_PULSE_2 );
	}

	if( BITCLEAR(M2_DIAG_A_PIN, M2_DIAG_A) /* || BITCLEAR(M2_DIAG_B_PIN, M2_DIAG_B) */ ){
		controller[1].enable = ENABLE_OFF;
		SETBIT( RX_FLAGS, FLAG_SHOULD_NOTIFY_FAULT_B );
//		addMessage1(CMD_ERROR, 1, ERROR_FAULT);

		// Reset the bridge
		M2_PWM = 0;
		M2_STOP_GND;
		_delay_us(100);
		M2_STOP_VCC;
		_delay_us(100);
		//		M2_DISABLE;
		changeLEDMode( LED_MODE_PULSE_3 );
	}
}

/*
 * This handles our servo functionality on the EXTRA pins
 */
volatile uint8_t wait = 0;
volatile int8_t inc = 1;

ISR(TIMER2_COMPA_vect){
	if( servo_state == 0 ){
		if( motor[0].extra_mode == EXTRA_MODE_SERVO ){
			servo_state = 1;
		}
		else if( motor[1].extra_mode == EXTRA_MODE_SERVO ){
			servo_state = 4;
		}
		else{
			return;
		}
	}

	// Here we start the 1ms pulse for extra1
	if( servo_state == 1 ){
		servo_active = motor[0].extra_servo;
		if( motor[0].extra_mode == EXTRA_MODE_SERVO ){
			SETBIT(EXTRA1_PORT, EXTRA1);
		}
		SERVO_SET_SHORT_WAIT;
		servo_state = 2;
	}
	// Here we start the signal length for extra1
	else if( servo_state == 2 ){
		SERVO_PRESCALER_128;
		if(servo_active == 0){
			servo_state = 3;
		}
		else{
			OCR2A = servo_active;
			servo_state = 3;
			return;
		}
	}

	// Here we finish the pulse for extra1
	if( servo_state == 3 ){
		if( motor[0].extra_mode == EXTRA_MODE_SERVO ){
			CLEARBIT(EXTRA1_PORT, EXTRA1);
		}
		if( motor[1].extra_mode == EXTRA_MODE_SERVO ){
			servo_state = 4;
		}
		else{
			servo_state = 10;
		}
	}

	// Here we start the 1ms pulse for extra2
	if( servo_state == 4 ){
		servo_active = motor[1].extra_servo;
		if( motor[1].extra_mode == EXTRA_MODE_SERVO ){
			SETBIT(EXTRA2_PORT, EXTRA2);
		}
		SERVO_SET_SHORT_WAIT;
		servo_state = 5;
	}
	// Here we start the signal length for extra2
	else if( servo_state == 5 ){
		SERVO_PRESCALER_128;
		if(servo_active == 0){
			servo_state = 6;
		}
		else{
			OCR2A = servo_active;
			servo_state = 6;
			return;
		}
	}

	// Here we finish the pulse for extra2
	if( servo_state == 6 ){
		if( motor[1].extra_mode == EXTRA_MODE_SERVO ){
			CLEARBIT(EXTRA2_PORT, EXTRA2);
		}
		servo_state = 10;
	}

	// This is when we do the long wait
	if( servo_state == 10 ){
		SERVO_SET_LONG_WAIT;
		servo_state = 11;
	}
	else if( servo_state == 11 ){
		SERVO_SET_LONG_WAIT;
		servo_state = 0;
	}
}


/*
 * In this implementation, the interrupts can handle the following frequencies of pulsetrains (measured on one pin, assumed to be continuous)
 * - In fully separated mode (that is encoder of motor 1 is fully out of phase with the one of motor 2 which means best separation between the different ISR calls)
 *    : ~ 16kHz
 *
 * - In fully overlapping mode (basically ENCA pins of motor 1 and 2 are tied together as are ENCB, this is a bad situation for the ISRs because they are called at the same time)
 *    : ~ 12.6kHz
 */


// M1_ENCA changed state
ISR(INT0_vect){
	if( BITSET(M1_ENCB_PIN, M1_ENCB) ){
		if( BITSET(M1_ENCA_PIN, M1_ENCA) ) 	motor[0].encoder_increment--;
		else								motor[0].encoder_increment++;
	}
	else{
		if( BITSET(M1_ENCA_PIN, M1_ENCA) ) 	motor[0].encoder_increment++;
		else								motor[0].encoder_increment--;
	}
}

// M2_ENCA changed state
ISR(INT1_vect){
	if( BITSET(M2_ENCB_PIN, M2_ENCB) ){
		if( BITSET(M2_ENCA_PIN, M2_ENCA) ) 	motor[1].encoder_increment--;
		else								motor[1].encoder_increment++;
	}
	else{
		if( BITSET(M2_ENCA_PIN, M2_ENCA) ) 	motor[1].encoder_increment++;
		else								motor[1].encoder_increment--;
	}
}
