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
void changeLEDMode(unsigned char);
void updateLEDState(void);
void setEXTRAMode(uint8_t, uint8_t);

// Comm functions
void handleIncomingRx(uint8_t);
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

#define EXTRA1_PORT			PORTC
#define EXTRA1_DDR			DDRC
#define EXTRA1_PIN			DDRC
#define EXTRA1				3		// PCINT11

#define EXTRA2_PORT			PORTC
#define EXTRA2_DDR			DDRC
#define EXTRA2_PIN			DDRC
#define EXTRA2				2		// PCINT10

#define LED_PORT			PORTB
#define LED_DDR				DDRB
#define LED					0


// ANALOG
#define M1_POT_PORT			PORTC
#define M1_POT_DDR			DDRC
#define M1_POT				0
#define M1_CS				6

#define M2_POT_PORT			PORTC
#define M2_POT_DDR			DDRC
#define M2_POT				5
#define M2_CS				7

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
#define M1_IN_A_PORT		PORTD
#define M1_IN_A_DDR			DDRD
#define M1_IN_A				6		// PCINT22 (PCI2)
#define M1_IN_B_PORT		PORTD
#define M1_IN_B_DDR			DDRD
#define M1_IN_B				5		// PCINT21 (PCI2)
#define M1_DIAG_A_PORT		PORTB
#define M1_DIAG_A_PIN		PINB
#define M1_DIAG_A_DDR		DDRB
#define M1_DIAG_A			4		// PCINT4 (PCI0) X

#define M2_IN_A_PORT		PORTB
#define M2_IN_A_DDR			DDRB
#define M2_IN_A				3		// PCINT3 (PCI0)
#define M2_IN_B_PORT		PORTD
#define M2_IN_B_DDR			DDRD
#define M2_IN_B				7		// PCINT23 (PCI2)
#define M2_DIAG_A_PORT		PORTB
#define M2_DIAG_A_PIN		PINB
#define M2_DIAG_A_DDR		DDRB
#define M2_DIAG_A			5		// PCINT5 (PCI0) X

/*
 * USEFUL SUBROUTINES
 */

// These functions set the TX_EN pin to enable/disable the rs485 tx chip but they also set the tx pin into hi-z state between transmits
#define	ENABLE_TX			SETBIT(UCSR0B, TXEN0); SETBIT(TX_EN_PORT, TX_EN); SETBIT(UCSR0B, TXCIE0)
#define	DISABLE_TX			CLEARBIT(TX_EN_PORT, TX_EN); CLEARBIT(UCSR0B, TXEN0); CLEARBIT(UCSR0B, TXCIE0)


/*
 * Motor control subroutines
 */

// On disable, we disable the pin change interrupt, set the DIAG pin as output and pull the DIAG line low
//#define M1_DISABLE			CLEARBIT(PCMSK0, M1_DIAG_A); CLEARBIT(PCMSK0, M1_DIAG_B); SETBIT(M1_DIAG_A_DDR, M1_DIAG_A); SETBIT(M1_DIAG_B_DDR, M1_DIAG_B); CLEARBIT(M1_DIAG_A_PORT, M1_DIAG_A); CLEARBIT(M1_DIAG_B_PORT, M1_DIAG_B)
//#define M1_ENABLE			CLEARBIT(M1_DIAG_A_DDR, M1_DIAG_A); CLEARBIT(M1_DIAG_B_DDR, M1_DIAG_B); SETBIT(PCMSK0, M1_DIAG_A); SETBIT(PCMSK0, M1_DIAG_B)

// Note that CW and CCW are basically reversed for backwards compatability (changes Polarity setting)
#define M1_CCW				SETBIT(M1_IN_A_PORT, M1_IN_A); CLEARBIT(M1_IN_B_PORT, M1_IN_B)
#define M1_CW				CLEARBIT(M1_IN_A_PORT, M1_IN_A); SETBIT(M1_IN_B_PORT, M1_IN_B)
#define M1_STOP_GND			CLEARBIT(M1_IN_A_PORT, M1_IN_A); CLEARBIT(M1_IN_B_PORT, M1_IN_B)
#define M1_STOP_VCC			SETBIT(M1_IN_A_PORT, M1_IN_A); SETBIT(M1_IN_B_PORT, M1_IN_B)

//#define M2_DISABLE			CLEARBIT(PCMSK0, M2_DIAG_A); CLEARBIT(PCMSK0, M2_DIAG_B); SETBIT(M2_DIAG_A_DDR, M2_DIAG_A); SETBIT(M2_DIAG_B_DDR, M2_DIAG_B); CLEARBIT(M2_DIAG_A_PORT, M2_DIAG_A); CLEARBIT(M2_DIAG_B_PORT, M2_DIAG_B)
//#define M2_ENABLE			CLEARBIT(M2_DIAG_A_DDR, M2_DIAG_A); CLEARBIT(M2_DIAG_B_DDR, M2_DIAG_B); SETBIT(PCMSK0, M2_DIAG_A); SETBIT(PCMSK0, M2_DIAG_B)
#define M2_CCW				SETBIT(M2_IN_A_PORT, M2_IN_A); CLEARBIT(M2_IN_B_PORT, M2_IN_B)
#define M2_CW				CLEARBIT(M2_IN_A_PORT, M2_IN_A); SETBIT(M2_IN_B_PORT, M2_IN_B)
#define M2_STOP_GND			CLEARBIT(M2_IN_A_PORT, M2_IN_A); CLEARBIT(M2_IN_B_PORT, M2_IN_B)
#define M2_STOP_VCC			SETBIT(M2_IN_A_PORT, M2_IN_A); SETBIT(M2_IN_B_PORT, M2_IN_B)


/*
 * Settings for the LED blinking
 */
#define LED_SETUP SETBIT(LED_DDR, LED)
#define LED_ON SETBIT(LED_PORT, LED)
#define LED_OFF CLEARBIT(LED_PORT, LED)

#define LED_STATE_SIZE    	16
#define LED_NR_STATES 		8
#define LED_MAX_COUNT    	12

#define LED_MODE_ALL_ON			0
#define LED_MODE_BLINK_1		1
#define LED_MODE_BLINK_2		2
#define LED_MODE_BLINK_3		3
#define LED_MODE_PULSE_1		4
#define LED_MODE_PULSE_2 		5
#define LED_MODE_PULSE_3		6
#define LED_MODE_PULSE_4		7

volatile unsigned char led_mode = 0;
volatile unsigned char led_state = 0;
volatile unsigned char led_internal_counter = 0;

//unsigned char led_data[LED_NR_STATES][LED_STATE_SIZE] =	{
//		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//		{1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
//		{1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
//		{1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0},
//		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//		{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//		{1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//		{1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0}
//};


/*
 * Servo pin functionality
 */
#define SERVO_LONG_WAIT_TIME    190 // Defines long wait to be approx 15ms
#define SERVO_SHORT_WAIT_TIME   77 // Defines short wait to be approx 0.5ms

#define SERVO_TIMER_ENABLE 			TIMSK2 = (1 << OCIE2A); TCCR2A = (1 << WGM21); TCNT2 = 0; servo_state = 0; SERVO_SET_LONG_WAIT
#define SERVO_TIMER_DISABLE 		TIMSK2 = 0
#define SERVO_PRESCALER_1024		TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20)
#define SERVO_PRESCALER_256			TCCR2B = (1 << CS22) | (1 << CS21) | (0 << CS20)
#define SERVO_PRESCALER_128			TCCR2B = (1 << CS22) | (0 << CS21) | (1 << CS20)
#define SERVO_PRESCALER_64			TCCR2B = (1 << CS22) | (0 << CS21) | (0 << CS20)
#define SERVO_SET_SHORT_WAIT		SERVO_PRESCALER_256; OCR2A = SERVO_SHORT_WAIT_TIME // Set prescaler to 256 and timer to short
#define SERVO_SET_LONG_WAIT			SERVO_PRESCALER_1024; OCR2A = SERVO_LONG_WAIT_TIME // Set prescaler to 1024 and timer to long

volatile unsigned char servo_state;
volatile unsigned char servo_active;

/*
 * Global variables for both motors and their defaults
 */
uint8_t volatile id;
uint8_t volatile new_id;
volatile uint8_t should_change_id;
uint8_t volatile tx_ready;

uint32_t volatile loop_count = 0;

uint8_t volatile pid_update_period = 195;	// Sets the update rate at around 100Hz

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
#define A2D_ITERATIONS_DIV2 	2
#define A2D_ITERATIONS 			1<<A2D_ITERATIONS_DIV2		// Needs to be divisible by 2

uint8_t volatile a2d_index;
uint8_t volatile a2d_counter;

uint16_t volatile a2d_value;
uint8_t volatile a2d_value_ready_flag;

/*
 * Extra pin variables
 */
#define EXTRA_MODE_OFF			0
#define EXTRA_MODE_SWITCH		1
#define EXTRA_MODE_ANALOG		2
#define EXTRA_MODE_SERVO		3

/*
 * Communication variables
 */

#define INVALID_ID  	126
#define BCAST_ID  		127

#define HEADER_BYTE  	0xAA
#define ESCAPE_BYTE  	0x55

uint8_t volatile write_checksum = 0;

uint8_t volatile timeout_timer = 0;

#define NR_BUFFERS			2
#define BUFFER_SIZE			40

circBuffer tx_buffer;
circBuffer incoming_buffers[NR_BUFFERS];
uint8_t volatile rx_buf_index = 0;
uint8_t volatile package_buf_index = 0;
uint8_t volatile rx_checksum;
volatile unsigned char rx_have_received_package = 0;

static uint8_t tx_buffer_data[BUFFER_SIZE];
static uint8_t buffer_data1[BUFFER_SIZE];
static uint8_t buffer_data2[BUFFER_SIZE];
//static uint8_t buffer_data3[BUFFER_SIZE];
//static uint8_t buffer_data4[BUFFER_SIZE];
//static uint8_t buffer_data5[BUFFER_SIZE];
//static uint8_t buffer_data6[BUFFER_SIZE];
//static uint8_t buffer_data7[BUFFER_SIZE];
//static uint8_t buffer_data8[BUFFER_SIZE];

//uint8_t volatile debug_msg0;
//int32_t volatile debug_msg1;

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
#define CMD_EXTRA_MODE						32
#define CMD_EXTRA_VALUE						33

#define CMD_2TARGET_TICK_VELOCITY			34

#define CMD_ACTUAL_VEL						35
#define CMD_VEL_P_GAIN						36
#define CMD_VEL_I_GAIN						37
#define CMD_VEL_D_GAIN						38
#define CMD_VEL_DOWNSCALE					39
#define CMD_MAX_ACCELERATION				40
#define CMD_VEL_TIME_DELTA					41
#define CMD_STREAM_MODE						42

#define CMD_2TARGET_TICK_POT				43
#define CMD_2TARGET_TICK_ENCODER			44

#define CMD_2TARGET_TICK_2ACTUAL			45
#define CMD_2TARGET_TICK_2VELOCITY			46
#define CMD_2TARGET_TICK_2MOTOR_CURRENT		47
#define CMD_2TARGET_TICK_2POT				48
#define CMD_2TARGET_TICK_2ENCODER			49

#define CMD_PID_UPDATE_PERIOD				50

/*
 * Error messages
 */
#define ERROR_BAD_CHECKSUM					0
#define ERROR_BAD_CMD_RECEIVED				1
#define ERROR_UNINITIALIZED					2
#define ERROR_BUFFER_OVERFLOW				3
#define ERROR_TIMEOUT_DISABLE				4
#define ERROR_FAULT							5
#define ERROR_BAD_ID_PACKET					6
#define ERROR_PACKET_OVERFLOW				7
#define ERROR_SET_PARAM_DURING_ENABLE		9
#define ERROR_MSG_BUFFER_OVERFLOW			10

/*
 * Flags used to signal when messages should be sent etc.
 */
volatile uint16_t RX_FLAGS = 0;
#define FLAG_SHOULD_NOTIFY_TIMEOUT				0
#define FLAG_SHOULD_SEND_EXTRA_VAL_A			1
#define FLAG_SHOULD_SEND_EXTRA_VAL_B			2
#define FLAG_SHOULD_NOTIFY_BUFFER_OVERFLOW		3
#define FLAG_SHOULD_NOTIFY_PACKET_OVERFLOW		4
#define FLAG_SHOULD_NOTIFY_BAD_CHECKSUM			5
#define FLAG_SHOULD_NOTIFY_UNINITIALIZED_A		6
#define FLAG_SHOULD_NOTIFY_UNINITIALIZED_B		7
#define FLAG_NEXT_BYTE_SHOULD_BE_TRANSFORMED	8
#define FLAG_SHOULD_NOTIFY_FAULT_A				9
#define FLAG_SHOULD_NOTIFY_FAULT_B				10

#endif
