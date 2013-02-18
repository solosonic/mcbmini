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
 
 /*
 * mcb_mini.h
 *
 *  Created on: Feb 16, 2011
 *      Author: siggi
 */

#ifndef MCB_MINI_H_
#define MCB_MINI_H_

#include <stdint.h>

#define PROGRAMMED_PWM_FREQ 1100	// For approximately 18kHz
//#define PROGRAMMED_PWM_FREQ 500	// For approximately 50kHz

typedef unsigned char uint8_t;
typedef signed char	int8_t;
typedef unsigned long uint32_t;
typedef signed long	int32_t;

/*
 * Useful macros
 */
#define setADCChannel(ch) 	ADMUX = (ADMUX & ~0x1F) | (ch & 0x1F)

// Nice functions to have predefined
#define BIT(x) (1 << (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SETBIT(x,y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))
#define BITSET(x,y) ((x) & (BIT(y)))
#define BITCLEAR(x,y) !BITSET((x), (y))
#define BITSSET(x,y) (((x) & (y)) == (y))
#define BITSCLEAR(x,y) (((x) & (y)) == 0)
#define BITVAL(x,y) (((x)>>(y)) & 1)

#define MIN(a,b)			((a<b)?(a):(b))
#define MAX(a,b)			((a>b)?(a):(b))
#define ABS(x)				((x>0)?(x):(-x))
#define LIMIT(x,low,high)				((x>high)?(high):((x<low)?(low):(x)))

#define TRUE -1
#define FALSE 0

/*
 * Function declarations
 */
void avrInit(void);
void handleA2DConversionReady(void);
void readID(void);
void changeID(uint8_t);
void writeIDInternal(uint16_t, uint8_t);
uint8_t readIDInternal(uint16_t);

// Comm functions
void handleIncomingRx(uint8_t);
void addMessageInt(uint8_t, uint8_t, int32_t);
void addMessage3(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
void addMessage2(uint8_t, uint8_t, uint8_t, uint8_t);
void addMessage1(uint8_t, uint8_t, uint8_t);
void addMessage(uint8_t, uint8_t);
void addMessageAndSendTxBuffer(void);
void addCMDByteToTxBuffer(uint8_t);

// Package functions
uint8_t isPackageBufferReady(void);
circBuffer* getCurrentPackageBuffer(void);
circBuffer* getCurrentRxBuffer(void);
void markPackageBufferHandled(void);
void createPackageBufferFromRxBuffer(void);
void processPackageBuffer(void);

// Tx buffer functions
void addByteToTxBuffer(uint8_t);
void addIntToTxBufferReversed(int32_t);
long readIntFromEndReversed(circBuffer*);
void sendTxBuffer(uint8_t);
void clearTxBuffer(void);

/*
 * Global constants
 */

#define F_CPU        20000000

#define BAUD_RATE		115200
#define UBRRVAL 		((F_CPU+(BAUD_RATE*8L))/(BAUD_RATE*16L)-1)

// Defines how many steps there are in our ramp up (needs to be 2**ramp_factor -1)
#define SLOW_ENABLE_STEPS	31

/*
 * PWM definitions
 */

#define FULL_PWM 		ICR1
#define M1_PWM	 		OCR1B
#define M2_PWM 			OCR1A

/*
 * Pin definitions
 */

// MISC
#define TX_EN_PORT			PORTD
#define TX_EN_DDR			DDRD
#define TX_EN				4

// ANALOG
#define M1_CS_PORT			PORTC
#define M1_CS_DDR			DDRC
#define M1_CS				0
#define M2_CS_PORT			PORTC
#define M2_CS_DDR			DDRC
#define M2_CS				5
#define M1_POT				6
#define M2_POT				7
#define FAULT_PORT			PORTC
#define FAULT_DDR			DDRC
#define FAULT_PIN			PINC
#define FAULT_COMP			3

// ENCODERS
#define M1_ENCA_PORT		PORTD
#define M1_ENCA_DDR			DDRD
#define M1_ENCA_PIN			PIND
#define M1_ENCA				2
#define M1_ENCB_PORT		PORTC
#define M1_ENCB_DDR			DDRC
#define M1_ENCB_PIN			PINC
#define M1_ENCB				4
#define M2_ENCA_PORT		PORTD
#define M2_ENCA_DDR			DDRD
#define M2_ENCA_PIN			PIND
#define M2_ENCA				3
#define M2_ENCB_PORT		PORTC
#define M2_ENCB_DDR			DDRC
#define M2_ENCB_PIN			PINC
#define M2_ENCB				1

// MOTOR CONTROL
#define M1_1T_PORT			PORTB
#define M1_1T_DDR			DDRB
#define M1_1T				5
#define M1_1B_PORT			PORTC
#define M1_1B_DDR			DDRC
#define M1_1B				2
#define M1_2T_PORT			PORTB
#define M1_2T_DDR			DDRB
#define M1_2T				4
#define M1_2B_PORT			PORTB
#define M1_2B_DDR			DDRB
#define M1_2B				3

#define M2_1T_PORT			PORTB
#define M2_1T_DDR			DDRB
#define M2_1T				0
#define M2_1B_PORT			PORTD
#define M2_1B_DDR			DDRD
#define M2_1B				7
#define M2_2T_PORT			PORTD
#define M2_2T_DDR			DDRD
#define M2_2T				6
#define M2_2B_PORT			PORTD
#define M2_2B_DDR			DDRD
#define M2_2B				5

/*
 * USEFUL SUBROUTINES
 */

// These functions set the TX_EN pin to enable/disable the rs485 tx chip but they also set the tx pin into hi-z state between transmits
#define	ENABLE_TX			SETBIT(UCSR0B, TXEN0); SETBIT(TX_EN_PORT, TX_EN); SETBIT(UCSR0B, TXCIE0)
#define	DISABLE_TX			CLEARBIT(TX_EN_PORT, TX_EN); CLEARBIT(UCSR0B, TXEN0); CLEARBIT(UCSR0B, TXCIE0)


/*
 * Motor control subroutines
 */

// Sub subroutines that should not really be used from user code

#define FORCE_M1_1T_OFF		SETBIT(M1_1T_DDR, M1_1T); CLEARBIT(M1_1T_PORT, M1_1T)
#define FORCE_M1_1T_ON		SETBIT(M1_1T_DDR, M1_1T); SETBIT(M1_1T_PORT, M1_1T)
#define FORCE_M1_1B_OFF		SETBIT(M1_1B_DDR, M1_1B); CLEARBIT(M1_1B_PORT, M1_1B)
#define FORCE_M1_1B_ON		SETBIT(M1_1B_DDR, M1_1B); SETBIT(M1_1B_PORT, M1_1B)
#define PWM_M1_1B			CLEARBIT(M1_1B_DDR, M1_1B)

#define FORCE_M1_2T_OFF		SETBIT(M1_2T_DDR, M1_2T); CLEARBIT(M1_2T_PORT, M1_2T)
#define FORCE_M1_2T_ON		SETBIT(M1_2T_DDR, M1_2T); SETBIT(M1_2T_PORT, M1_2T)
#define FORCE_M1_2B_OFF		SETBIT(M1_2B_DDR, M1_2B); CLEARBIT(M1_2B_PORT, M1_2B)
#define FORCE_M1_2B_ON		SETBIT(M1_2B_DDR, M1_2B); SETBIT(M1_2B_PORT, M1_2B)
#define PWM_M1_2B			CLEARBIT(M1_2B_DDR, M1_2B)

#define FORCE_M2_1T_OFF		SETBIT(M2_1T_DDR, M2_1T); CLEARBIT(M2_1T_PORT, M2_1T)
#define FORCE_M2_1T_ON		SETBIT(M2_1T_DDR, M2_1T); SETBIT(M2_1T_PORT, M2_1T)
#define FORCE_M2_1B_OFF		SETBIT(M2_1B_DDR, M2_1B); CLEARBIT(M2_1B_PORT, M2_1B)
#define FORCE_M2_1B_ON		SETBIT(M2_1B_DDR, M2_1B); SETBIT(M2_1B_PORT, M2_1B)
#define PWM_M2_1B			CLEARBIT(M2_1B_DDR, M2_1B)

#define FORCE_M2_2T_OFF		SETBIT(M2_2T_DDR, M2_2T); CLEARBIT(M2_2T_PORT, M2_2T)
#define FORCE_M2_2T_ON		SETBIT(M2_2T_DDR, M2_2T); SETBIT(M2_2T_PORT, M2_2T)
#define FORCE_M2_2B_OFF		SETBIT(M2_2B_DDR, M2_2B); CLEARBIT(M2_2B_PORT, M2_2B)
#define FORCE_M2_2B_ON		SETBIT(M2_2B_DDR, M2_2B); SETBIT(M2_2B_PORT, M2_2B)
#define PWM_M2_2B			CLEARBIT(M2_2B_DDR, M2_2B)

#define M1_ALL_OFF			FORCE_M1_1T_OFF; FORCE_M1_1B_OFF; FORCE_M1_2T_OFF; FORCE_M1_2B_OFF;
#define M2_ALL_OFF			FORCE_M2_1T_OFF; FORCE_M2_1B_OFF; FORCE_M2_2T_OFF; FORCE_M2_2B_OFF;

// Actually subroutines that should be used from user code
#define M1_CW				M1_ALL_OFF; FORCE_M1_1T_ON; PWM_M1_2B
#define M1_CCW				M1_ALL_OFF; FORCE_M1_2T_ON; PWM_M1_1B
#define M1_STOP				M1_ALL_OFF; FORCE_M1_1B_OFF; FORCE_M1_2B_OFF

#define M2_CW				M2_ALL_OFF; FORCE_M2_1T_ON; PWM_M2_2B
#define M2_CCW				M2_ALL_OFF; FORCE_M2_2T_ON; PWM_M2_1B
#define M2_STOP				M2_ALL_OFF; FORCE_M2_1B_OFF; FORCE_M2_2B_OFF


/*
 * Global variables for both motors and their defaults
 */
uint8_t volatile id;
uint8_t volatile flag_should_change_id;
uint8_t volatile new_id;
uint8_t volatile tx_ready;

uint32_t volatile loop_count = 0;


/*
 * Settings variables for the motors
 */

#define TARGET_BUFFER_NR_LONGS 		3
#define ACTUAL_BUFFER_NR_LONGS 		5

#define TARGET_BUFFER_SIZE 			(TARGET_BUFFER_NR_LONGS << 2)
#define ACTUAL_BUFFER_SIZE 			(ACTUAL_BUFFER_NR_LONGS << 2)
static uint8_t target_buffer_data_A[TARGET_BUFFER_SIZE];
static uint8_t target_buffer_data_B[TARGET_BUFFER_SIZE];
static uint8_t actual_buffer_data_A[ACTUAL_BUFFER_SIZE];
static uint8_t actual_buffer_data_B[ACTUAL_BUFFER_SIZE];

/*
 * a2d vars
 */
#define A2D_ITERATIONS 16			// Needs to be divisible by 2
#define A2D_ITERATIONS_DIV2 4		// This should satisfy 2^(A2D_ITERATIONS_DIV2) = A2D_ITERATIONS

uint8_t volatile a2d_index;
uint8_t volatile a2d_counter;

uint16_t volatile a2d_value;
uint8_t volatile a2d_value_ready_flag;


/*
 * Communication variables
 */

#define INVALID_ID  	126
#define BCAST_ID  		127

#define HEADER_BYTE  	0xAA
#define ESCAPE_BYTE  	0x55

uint8_t volatile next_byte_should_be_transformed = 0;
uint8_t volatile write_checksum = 0;
//uint8_t volatile txbuffer_empty = 1;

uint8_t volatile timeout_timer = 0;

#define NR_BUFFERS		4
#define BUFFER_SIZE		45
#define MSG_BUFFER_SIZE		128

circBuffer msg_buffer;
circBuffer tx_buffer;
circBuffer incoming_buffers[NR_BUFFERS];
uint8_t volatile rx_buf_index = 0;
uint8_t volatile package_buf_index = 0;
uint8_t volatile rx_checksum;

static uint8_t msg_buffer_data[MSG_BUFFER_SIZE];
static uint8_t tx_buffer_data[BUFFER_SIZE];
static uint8_t buffer_data1[BUFFER_SIZE];
static uint8_t buffer_data2[BUFFER_SIZE];
static uint8_t buffer_data3[BUFFER_SIZE];
static uint8_t buffer_data4[BUFFER_SIZE];
static uint8_t buffer_data5[BUFFER_SIZE];
static uint8_t buffer_data6[BUFFER_SIZE];
static uint8_t buffer_data7[BUFFER_SIZE];
static uint8_t buffer_data8[BUFFER_SIZE];

uint8_t volatile debug_msg0;
int32_t volatile debug_msg1;

/*
 * Communication commands
 */
#define CMD_ID								0

#define CMD_POS_P_GAIN						1
#define CMD_POS_I_GAIN						2
#define CMD_POS_D_GAIN						3
#define CMD_MAX_VELOCITY					4
#define CMD_DEADBAND						5

#define CMD_ENABLE							6
#define CMD_POLARITY						7
#define CMD_FEEDBACK_MODE					8
#define CMD_CONTROL_MODE					9

#define CMD_TARGET_TICK						10
#define CMD_ACTUAL_TICK						11
#define CMD_MOTOR_CURRENT					12

#define CMD_2TARGET_TICK_MOTOR_CURRENT		13
#define CMD_2TARGET_TICK_ACTUAL				15

#define CMD_EMPTY_RESPONSE					17
#define CMD_ERROR							18
#define CMD_PID_OUTPUT						19
#define CMD_FAULT_MODE						20

#define CMD_POS_DOWNSCALE					21
#define CMD_ENCODER_VALUE					22
#define CMD_POT_VALUE						23
#define CMD_FIRMWARE_VERSION				24
#define CMD_MAX_PWM_DUTY_CYCLE				25
#define CMD_SLOW_ENABLE_TIME				26

#define CMD_DEBUG							27
#define CMD_OFFSET_ENCODER_TICK				28
#define CMD_SATURATION						29
#define CMD_I_COMPONENT						30
#define CMD_REQUEST_MESSAGE					31

#define CMD_2TARGET_TICK_VELOCITY			34

#define CMD_ACTUAL_VEL						35
#define CMD_VEL_P_GAIN						36
#define CMD_VEL_I_GAIN						37
#define CMD_VEL_D_GAIN						38
#define CMD_VEL_DOWNSCALE					39
#define CMD_MAX_ACCELERATION				40
#define CMD_VEL_TIME_DELTA					41
#define CMD_STREAM_MODE						42

/*
 * Error messages
 */
#define ERROR_BAD_CHECKSUM					0
#define ERROR_BAD_CMD_RECEIVED				1
#define ERROR_UNINITIALIZED		2
#define ERROR_BUFFER_OVERFLOW				3
#define ERROR_TIMEOUT_DISABLE				4
#define ERROR_FAULT							5
#define ERROR_BAD_ID_PACKET					6
#define ERROR_PACKET_OVERFLOW				7
#define ERROR_SET_PARAM_DURING_ENABLE		9
#define ERROR_MSG_BUFFER_OVERFLOW			10



#endif
