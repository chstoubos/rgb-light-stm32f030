/*
 * RemoteControl.h
 *
 * Created: 27-Apr-19 19:12:15
 *  Author: chris

 NEC DECODE PROTOCOL
 */

#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

#define		KEY_CH_MINUS		0xFFA25D
#define		KEY_CH				0xFF629D
#define		KEY_CH_PLUS			0xFFE21D
#define		KEY_PREV			0xFF22DD
#define		KEY_NEXT			0xFF02FD
#define		KEY_PLAY_PAUSE		0xFFC23D
#define		KEY_VOL_DOWN		0xFFE01F
#define		KEY_VOL_UP			0xFFA857
#define		KEY_EQ				0xFF906F
#define		KEY_0				0xFF6897
#define		KEY_100				0xFF9867
#define		KEY_200				0xFFB04F
#define		KEY_1				0xFF30CF
#define		KEY_2				0xFF18E7
#define		KEY_3				0xFF7A85
#define		KEY_4				0xFF10EF
#define		KEY_5				0xFF38C7
#define		KEY_6				0xFF5AA5
#define		KEY_7				0xFF42BD
#define		KEY_8				0xFF4AB5
#define		KEY_9				0xFF52AD
#define		KEY_REPEAT			0xFFFFFFFF

#define		TCNT_FROM_US(t)		(t/64)		//64us per tick

//pulse timings
#define		COUNT_LOGICAL0_MIN			TCNT_FROM_US(400UL)			//560us
#define		COUNT_LOGICAL0_MAX			TCNT_FROM_US(800UL)

#define		COUNT_LOGICAL1_MIN			TCNT_FROM_US(1300UL)
#define		COUNT_LOGICAL1_MAX			TCNT_FROM_US(1900UL)

#define		COUNT_BURST_MIN				TCNT_FROM_US(8000UL)			//9ms
#define		COUNT_BURST_MAX				TCNT_FROM_US(10000UL)

#define		COUNT_GAP_MIN				TCNT_FROM_US(4000UL)			//4.5ms
#define		COUNT_GAP_MAX				TCNT_FROM_US(5000UL)

#define		COUNT_GAP_REP_MIN			TCNT_FROM_US(2000UL)			//2.2ms
#define		COUNT_GAP_REP_MAX			TCNT_FROM_US(3000UL)

enum ir_states{IR_IDLE, IR_BURST, IR_GAP, IR_DATA, IR_DATA_END, IR_FINISH};

typedef struct {
	uint8_t rec_addr;
	uint8_t rec_addr_inv;
	uint8_t rec_cmd;
	uint8_t rec_cmd_inv;
	uint8_t status;      //if the status is 1 then we have new data. when the data are processed this flag is cleared
} NEC_data_t;

extern NEC_data_t rec_data;

void print_key(uint32_t);

#endif /* REMOTECONTROL_H_ */
