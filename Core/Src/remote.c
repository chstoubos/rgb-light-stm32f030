/*
 * remote.c
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#include "remote.h"

remote_t remote = {0};

const char* const keys_str[] = {
		"KEY_CH_MINUS",
		"KEY_CH",
		"KEY_CH_PLUS",
		"KEY_PREV",
		"KEY_NEXT",
		"KEY_PLAY_PAUSE",
		"KEY_VOL_DOWN",
		"KEY_VOL_UP",
		"KEY_EQ",
		"KEY_0",
		"KEY_100",
		"KEY_200",
		"KEY_1",
		"KEY_2",
		"KEY_3",
		"KEY_4",
		"KEY_5",
		"KEY_6",
		"KEY_7",
		"KEY_8",
		"KEY_9",
};

void remote_init(void)
{
	LL_TIM_EnableCounter(REMOTE_TIMER);
	LL_TIM_EnableIT_UPDATE(REMOTE_TIMER);
}

void remote_print_key(uint32_t key)
{
	switch (key)
	{
		case KEY_CH_MINUS:
			dbg_print("KEY_CH_MINUS\n");
			break;
		case KEY_CH:
			dbg_print("KEY_CH\n");
			break;
		case KEY_CH_PLUS:
			dbg_print("KEY_CH_PLUS\n");
			break;
		case KEY_PREV:
			dbg_print("KEY_PREV\n");
			break;
		case KEY_NEXT:
			dbg_print("KEY_NEXT\n");
			break;
		case KEY_PLAY_PAUSE:
			dbg_print("KEY_PLAY_PAUSE\n");
			break;
		case KEY_VOL_DOWN:
			dbg_print("KEY_VOL_DOWN\n");
			break;
		case KEY_VOL_UP:
			dbg_print("KEY_VOL_UP\n");
			break;
		case KEY_EQ:
			dbg_print("KEY_EQ\n");
			break;
		case KEY_0:
			dbg_print("KEY_0\n");
			break;
		case KEY_100:
			dbg_print("KEY_100\n");
			break;
		case KEY_200:
			dbg_print("KEY_200\n");
			break;
		case KEY_1:
			dbg_print("KEY_1\n");
			break;
		case KEY_2:
			dbg_print("KEY_2\n");
			break;
		case KEY_3:
			dbg_print("KEY_3\n");
			break;
		case KEY_4:
			dbg_print("KEY_4\n");
			break;
		case KEY_5:
			dbg_print("KEY_5\n");
			break;
		case KEY_6:
			dbg_print("KEY_6\n");
			break;
		case KEY_7:
			dbg_print("KEY_7\n");
			break;
		case KEY_8:
			dbg_print("KEY_8\n");
			break;
		case KEY_9:
			dbg_print("KEY_9\n");
			break;
		default:
			break;
	}
}

static uint8_t need_debouncing(uint32_t key)
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
		if (need_debouncing(remote.ir_raw_data)) {
			return;
		}
	}

	switch (remote.ir_raw_data) {
		case KEY_CH_MINUS:
			dbg_print("KEY_CH_MINUS\n");
			break;
		case KEY_CH:
			dbg_print("KEY_CH\n");
			break;
		case KEY_CH_PLUS:
			dbg_print("KEY_CH_PLUS\n");
			break;
		case KEY_PREV:
			dbg_print("KEY_PREV\n");
			//speed - for effect
			break;
		case KEY_NEXT:
			dbg_print("KEY_NEXT\n");
			//speed + for effect
			break;
		case KEY_PLAY_PAUSE:
			dbg_print("KEY_PLAY_PAUSE\n");
			//power off / sleep
			break;
		case KEY_VOL_DOWN:	//brightness -
			dbg_print("KEY_VOL_DOWN\n");
			rgb_ctl_set_brightness(STEP_DOWN);
			break;
		case KEY_VOL_UP:	//brightness +
			dbg_print("KEY_VOL_UP\n");
			rgb_ctl_set_brightness(STEP_UP);
			break;
		case KEY_EQ:
			dbg_print("KEY_EQ\n");
			break;
		case KEY_0:
			dbg_print("KEY_0\n");
			rgb_ctl_set_color(WHITE, 100);
			break;
		case KEY_100:
			dbg_print("KEY_100\n");
			// sleep w/ timer
			break;
		case KEY_200:
			dbg_print("KEY_200\n");
			// sleep w/ timer and stays on during night
			break;
		case KEY_1:
			dbg_print("KEY_1\n");
			rgb_ctl_set_color(RED, 100);
			break;
		case KEY_2:
			dbg_print("KEY_2\n");
			rgb_ctl_set_color(GREEN, 100);
			break;
		case KEY_3:
			dbg_print("KEY_3\n");
			rgb_ctl_set_color(BLUE, 100);
			break;
		case KEY_4:
			dbg_print("KEY_4\n");
			rgb_ctl_set_color(PURPLE, 100);
			break;
		case KEY_5:
			dbg_print("KEY_5\n");
			rgb_ctl_set_color(YELLOW, 100);
			break;
		case KEY_6:
			dbg_print("KEY_6\n");
			rgb_ctl_set_color(ORANGE, 100);
			break;
		case KEY_7:
			dbg_print("KEY_7\n");
			rgb_ctl_set_color(CYAN, 100);
			break;
		case KEY_8:
			dbg_print("KEY_8\n");
			rgb_ctl_set_color(TURQUISE, 100);
			break;
		case KEY_9:
			dbg_print("KEY_9\n");
			rgb_ctl_set_color(PINK, 100);
			break;
		default:
			break;
	}
}

