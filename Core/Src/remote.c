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
			dbg_print("KEY_CH_MINUS");
			break;
		case KEY_CH:
			dbg_print("KEY_CH");
			break;
		case KEY_CH_PLUS:
			dbg_print("KEY_CH_PLUS");
			break;
		case KEY_PREV:
			dbg_print("KEY_PREV");
			break;
		case KEY_NEXT:
			dbg_print("KEY_NEXT");
			break;
		case KEY_PLAY_PAUSE:
			dbg_print("KEY_PLAY_PAUSE");
			break;
		case KEY_VOL_DOWN:
			dbg_print("KEY_VOL_DOWN");
			break;
		case KEY_VOL_UP:
			dbg_print("KEY_VOL_UP");
			break;
		case KEY_EQ:
			dbg_print("KEY_EQ");
			break;
		case KEY_0:
			dbg_print("KEY_0");
			break;
		case KEY_100:
			dbg_print("KEY_100");
			break;
		case KEY_200:
			dbg_print("KEY_200");
			break;
		case KEY_1:
			dbg_print("KEY_1");
			break;
		case KEY_2:
			dbg_print("KEY_2");
			break;
		case KEY_3:
			dbg_print("KEY_3");
			break;
		case KEY_4:
			dbg_print("KEY_4");
			break;
		case KEY_5:
			dbg_print("KEY_5");
			break;
		case KEY_6:
			dbg_print("KEY_6");
			break;
		case KEY_7:
			dbg_print("KEY_7");
			break;
		case KEY_8:
			dbg_print("KEY_8");
			break;
		case KEY_9:
			dbg_print("KEY_9");
			break;
		default:
//			dbg_printf("key 0x%.8lX\n", key);
			break;
	}
}

