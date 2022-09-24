/*
 * RemoteControl.h
 *
 * Created: 27-Apr-19 19:12:15
 *  Author: chris

 NEC DECODE PROTOCOL
 */

#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

#include "main.h"

#define 	NUM_KEYS					21
#define		KEY_CH_MINUS				0xFFA25D
#define		KEY_CH						0xFF629D
#define		KEY_CH_PLUS					0xFFE21D
#define		KEY_PREV					0xFF22DD
#define		KEY_NEXT					0xFF02FD
#define		KEY_PLAY_PAUSE				0xFFC23D
#define		KEY_VOL_DOWN				0xFFE01F
#define		KEY_VOL_UP					0xFFA857
#define		KEY_EQ						0xFF906F
#define		KEY_0						0xFF6897
#define		KEY_100						0xFF9867
#define		KEY_200						0xFFB04F
#define		KEY_1						0xFF30CF
#define		KEY_2						0xFF18E7
#define		KEY_3						0xFF7A85
#define		KEY_4						0xFF10EF
#define		KEY_5						0xFF38C7
#define		KEY_6						0xFF5AA5
#define		KEY_7						0xFF42BD
#define		KEY_8						0xFF4AB5
#define		KEY_9						0xFF52AD
#define		KEY_REPEAT					0xFFFFFFFF

#define 	REMOTE_TIMER				TIM14
#define		TCNT_FROM_US(t)				(t/64)						//64us per tick

//pulse timings
#define		COUNT_LOGICAL0_MIN			TCNT_FROM_US(400UL)			//560us
#define		COUNT_LOGICAL0_MAX			TCNT_FROM_US(800UL)

#define		COUNT_LOGICAL1_MIN			TCNT_FROM_US(1300UL)
#define		COUNT_LOGICAL1_MAX			TCNT_FROM_US(1900UL)

#define		COUNT_BURST_MIN				TCNT_FROM_US(8000UL)		//9ms
#define		COUNT_BURST_MAX				TCNT_FROM_US(10000UL)

#define		COUNT_GAP_MIN				TCNT_FROM_US(4000UL)		//4.5ms
#define		COUNT_GAP_MAX				TCNT_FROM_US(5000UL)

#define		COUNT_GAP_REP_MIN			TCNT_FROM_US(2000UL)		//2.2ms
#define		COUNT_GAP_REP_MAX			TCNT_FROM_US(3000UL)

#define 	IR_DATA_READY_FLAG_DATA		1
#define 	IR_DATA_READY_FLAG_REPEAT	2

#define 	IR_PIN_HIGH					LL_GPIO_IsInputPinSet(IR_RECV_GPIO_Port, IR_RECV_Pin)
#define 	IR_PIN_LOW					(!LL_GPIO_IsInputPinSet(IR_RECV_GPIO_Port, IR_RECV_Pin))

enum ir_states {
	IR_IDLE = 0,
	IR_BURST,
	IR_GAP,
	IR_DATA,
	IR_DATA_END,
	IR_FINISH
};

typedef struct {
	volatile uint8_t state;
	uint32_t ir_raw_data;
	volatile uint8_t on_key_press_flag;
} remote_t;

extern remote_t remote;

void remote_init(void);
void remote_print_key(uint32_t);
void on_key_press(void);

#endif /* REMOTECONTROL_H_ */
