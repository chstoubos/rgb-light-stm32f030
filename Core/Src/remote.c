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
	LL_TIM_EnableCounter(REMOTE_TIMER);
	LL_TIM_EnableIT_UPDATE(REMOTE_TIMER);
}

static uint8_t is_no_repeat_key(uint32_t key)
{
	if (key == KEY_VOL_UP || key == KEY_VOL_DOWN ||
		key == KEY_NEXT || key == KEY_PREV) {
		return 0;
	}

	return 1;
}

void on_key_press(void)
{
	if (remote.on_key_press_flag == IR_DATA_READY_FLAG_REPEAT) {
		if (is_no_repeat_key(remote.ir_raw_data)) {
			return;
		}
	}

	switch (remote.ir_raw_data) {
		case KEY_CH_MINUS:
			rgb_ctl_rainbow_start();
			break;
		case KEY_CH:
			break;
		case KEY_CH_PLUS:
			rgb_ctl_rainbow_stop();
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
			rgb_ctl_set_color(WHITE, 0);
			//power off / sleep
			break;
		case KEY_VOL_DOWN:
			rgb_ctl_set_brightness(STEP_DOWN);
			break;
		case KEY_VOL_UP:
			rgb_ctl_set_brightness(STEP_UP);
			break;
		case KEY_EQ:
			break;
		case KEY_0:
			rgb_ctl_set_color(WHITE, 100);
			break;
		case KEY_100: 	//night mode
			// sleep w/ timer
			break;
		case KEY_200:
			// sleep w/ timer and stays on during night
			break;
		case KEY_1:
			rgb_ctl_set_color(RED, 100);
			break;
		case KEY_2:
			rgb_ctl_set_color(GREEN, 100);
			break;
		case KEY_3:
			rgb_ctl_set_color(BLUE, 100);
			break;
		case KEY_4:
			rgb_ctl_set_color(PURPLE, 100);
			break;
		case KEY_5:
			rgb_ctl_set_color(YELLOW, 100);
			break;
		case KEY_6:
			rgb_ctl_set_color(ORANGE, 100);
			break;
		case KEY_7:
			rgb_ctl_set_color(CYAN, 100);
			break;
		case KEY_8:
			rgb_ctl_set_color(TURQUISE, 100);
			break;
		case KEY_9:
			rgb_ctl_set_color(PINK, 100);
			break;
		default:
			break;
	}
}

