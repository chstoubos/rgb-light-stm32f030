/*
 * remote.c
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#include "remote.h"

remote_t remote = {0};

void remote_init(void)
{
	remote.key_presses_num = 0;

	LL_TIM_EnableCounter(REMOTE_TIMER);
	LL_TIM_EnableIT_UPDATE(REMOTE_TIMER);
}

static uint8_t is_no_repeat_key(uint32_t key)
{
	if (key == KEY_VOL_UP || key == KEY_VOL_DOWN ||
		key == KEY_NEXT || key == KEY_PREV ||
		key == KEY_CH_MINUS || key == KEY_CH_PLUS ) {
		return 0;
	}

	return 1;
}

/**
 * @brief Detect long presses in keys 0-9
 *
 * @param key
 * @param last_key
 */
static void on_nums_repeat(uint32_t key, uint32_t last_key)
{
	if (key == last_key) {
		remote.key_presses_num++;

		if (remote.key_presses_num >= LONG_PRESS_KEY_PRESS_NUMS) {
			remote.on_long_press_flag = 1;
			remote.key_presses_num = 0;
		}
	}
	else {
		remote.key_presses_num = 0;
	}

	remote.last_key_pressed = remote.ir_raw_data;
}

/**
 * @brief Long press on numeric key saves the current
 * 	      color in the memory
 *
 * @param key
 */
void remote_on_long_press(uint32_t key)
{
	unsigned int color_idx = 0;
	switch (key) {
		case KEY_0:
			color_idx = 0;
			break;
		case KEY_1:
			color_idx = 1;
			break;
		case KEY_2:
			color_idx = 2;
			break;
		case KEY_3:
			color_idx = 3;
			break;
		case KEY_4:
			color_idx = 4;
			break;
		case KEY_5:
			color_idx = 5;
			break;
		case KEY_6:
			color_idx = 6;
			break;
		case KEY_7:
			color_idx = 7;
			break;
		case KEY_8:
			color_idx = 8;
			break;
		case KEY_9:
			color_idx = 9;
			break;
		case KEY_EQ:
			rgb_ctl_restore_defaults();
			return;
		default:
			return;
	}

	rgb_ctl_custom_color_save(color_idx);
}

void remote_on_key_press(void)
{
	if (remote.on_key_press_flag == IR_DATA_READY_FLAG_REPEAT)
	{
		on_nums_repeat(remote.ir_raw_data, remote.last_key_pressed);

		if (is_no_repeat_key(remote.ir_raw_data)) {
			return;
		}
	}
	else {
		remote.key_presses_num = 0;
	}

	switch (remote.ir_raw_data) {
		case KEY_CH_MINUS:
			rgb_ctl_custom_color_run(STEP_DOWN);
			break;
		case KEY_CH:
			rgb_ctl_custom_change_channel();
			break;
		case KEY_CH_PLUS:
			rgb_ctl_custom_color_run(STEP_UP);
			break;
		case KEY_PREV:
			//speed - for effect
			rgb_ctl_rainbow_set_speed(STEP_DOWN);
			break;
		case KEY_NEXT:
			rgb_ctl_rainbow_set_speed(STEP_UP);
			//speed + for effect
			break;
		case KEY_PLAY_PAUSE:
			rgb_ctl_rainbow_start();
			break;
		case KEY_VOL_DOWN:
			rgb_ctl_set_brightness(STEP_DOWN);
			break;
		case KEY_VOL_UP:
			rgb_ctl_set_brightness(STEP_UP);
			break;
		case KEY_EQ:
			// Restore defaults (?)
			break;
		case KEY_0:
			rgb_ctl_set_fixed_color(WHITE, 100);
			break;
		case KEY_100: 	//night mode
			// sleep w/ timer
			break;
		case KEY_200:
			// sleep w/ timer and stays on during night
			break;
		case KEY_1:
			rgb_ctl_set_fixed_color(RED, 100);
			break;
		case KEY_2:
			rgb_ctl_set_fixed_color(GREEN, 100);
			break;
		case KEY_3:
			rgb_ctl_set_fixed_color(BLUE, 100);
			break;
		case KEY_4:
			rgb_ctl_set_fixed_color(PURPLE, 100);
			break;
		case KEY_5:
			rgb_ctl_set_fixed_color(YELLOW, 100);
			break;
		case KEY_6:
			rgb_ctl_set_fixed_color(ORANGE, 100);
			break;
		case KEY_7:
			rgb_ctl_set_fixed_color(CYAN, 100);
			break;
		case KEY_8:
			rgb_ctl_set_fixed_color(TURQUISE, 100);
			break;
		case KEY_9:
			rgb_ctl_set_fixed_color(PINK, 100);
			break;
		default:
			break;
	}
}

/**
 * Place inside IR pin interrupt
 */
inline void remote_ctl(void)
{
	static int8_t ir_data_bit_cnt = 0;
	static uint32_t ir_data;

	switch (remote.state)
	{
		case IR_IDLE:		//default state - waiting for initial signal
			if(IR_PIN_LOW)
			{
				remote.state = IR_BURST;
			}
			break;
		case IR_BURST:
			if(IR_PIN_HIGH)
			{
				//9ms of burst pulses in 38KHz freq
				if(REMOTE_TIMER->CNT > COUNT_BURST_MIN && REMOTE_TIMER->CNT < COUNT_BURST_MAX)
				{
					remote.state = IR_GAP;
				}
			}
			break;
		case IR_GAP:
			// if GAP 4500us we have data, if GAP 2200us it is a repeat
			if(IR_PIN_LOW)
			{
				//4.5ms space (data)
				if(REMOTE_TIMER->CNT > COUNT_GAP_MIN && REMOTE_TIMER->CNT < COUNT_GAP_MAX)
				{
					remote.state = IR_DATA;
	//LED_PIN_HIGH;LED_PIN_LOW;
					ir_data_bit_cnt = 0;
					ir_data = 0;
				}
				// 2.2ms (repeat)
				else if (REMOTE_TIMER->CNT > COUNT_GAP_REP_MIN && REMOTE_TIMER->CNT < COUNT_GAP_REP_MAX)
				{
					remote.state = IR_IDLE;
					remote.on_key_press_flag = IR_DATA_READY_FLAG_REPEAT;
				}
			}
			break;
		case IR_DATA:
			if(IR_PIN_LOW)
			{
				/*
					we are expecting 32 bit of data
					8bit address - 8bit inverted address - 8bit data - 8bit inverted data [LSB FIRST]
					562.5us burst pulse followed by a 562.5us space is counted as a logical 0 (1.125 ms total)
					562.5us burst pulse followed by a 3x562.5us space is counted as a logical 1 (2.25ms total)

					basika ston diko mou sensora einai to anapodo....
				*/

				ir_data <<= 1;

				if(REMOTE_TIMER->CNT > COUNT_LOGICAL0_MIN && REMOTE_TIMER->CNT < COUNT_LOGICAL0_MAX)
				{

				}
				else
				{
					//logical 1
					//ir_data[ir_data_byte_cnt] |= (1<<ir_data_bit_cnt);
					ir_data |= 0x01;
				}
				if (++ir_data_bit_cnt > 31)
				{
					//data are ready change state
					remote.state = IR_FINISH;
				}
			}
			break;
		case IR_FINISH:
			//endiamesi katastasi, edw tha kanw jump otan trww timeout h paei kati lathos
			remote.state = IR_IDLE;
			remote.ir_raw_data = ir_data;
			remote.on_key_press_flag = IR_DATA_READY_FLAG_DATA;
			break;
		default:
			break;
	}

	REMOTE_TIMER->CNT = 0;
}

