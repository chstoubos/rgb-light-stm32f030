/*
 * remote.c
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#include "main.h"

void print_key(uint32_t key)
{
	switch (key)
	{
		case KEY_CH_MINUS:
			debug_print("KEY_CH_MINUS");
			break;
		case KEY_CH:
			debug_print("KEY_CH");
			break;
		case KEY_CH_PLUS:
			debug_print("KEY_CH_PLUS");
			break;
		case KEY_PREV:
			debug_print("KEY_PREV");
			break;
		case KEY_NEXT:
			debug_print("KEY_NEXT");
			break;
		case KEY_PLAY_PAUSE:
			debug_print("KEY_PLAY_PAUSE");
			break;
		case KEY_VOL_DOWN:
			debug_print("KEY_VOL_DOWN");
			break;
		case KEY_VOL_UP:
			debug_print("KEY_VOL_UP");
			break;
		case KEY_EQ:
			debug_print("KEY_EQ");
			break;
		case KEY_0:
			debug_print("KEY_0");
			break;
		case KEY_100:
			debug_print("KEY_100");
			break;
		case KEY_200:
			debug_print("KEY_200");
			break;
		case KEY_1:
			debug_print("KEY_1");
			break;
		case KEY_2:
			debug_print("KEY_2");
			break;
		case KEY_3:
			debug_print("KEY_3");
			break;
		case KEY_4:
			debug_print("KEY_4");
			break;
		case KEY_5:
			debug_print("KEY_5");
			break;
		case KEY_6:
			debug_print("KEY_6");
			break;
		case KEY_7:
			debug_print("KEY_7");
			break;
		case KEY_8:
			debug_print("KEY_8");
			break;
		case KEY_9:
			debug_print("KEY_9");
			break;
		default:
			debug_printf("key 0x%.8lX\n", key);
			break;
	}
}
