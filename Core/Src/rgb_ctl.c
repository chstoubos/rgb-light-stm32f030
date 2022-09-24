/*
 * rgb_ctl.c
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#include "rgb_ctl.h"
#include "main.h"

rgb_ctl_t rgb_ctl = {0};

/**
 * Initialize timer with 3-channel PWM 10KHz
 */
void rgb_ctl_init(void)
{
	LL_TIM_EnableCounter(RGB_CTL_TIMER);

	rgb_ctl_set_color(DEFAULT_COLOR, DEFAULT_BRIGHTNESS);

	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, RED_CHANNEL);
	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, GREEN_CHANNEL);
	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, BLUE_CHANNEL);
}

void rgb_ctl_set_color(fixed_colors_t color, uint8_t brightness)
{
	rgb_ctl.mode = FIXED;
	rgb_ctl.current_color = color;
	rgb_ctl.brightness_lvl_prcntg = brightness;

	switch (color) {
		case WHITE:
			RED_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			GREEN_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			BLUE_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			break;
		case RED:
			RED_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			GREEN_PWM(0);
			BLUE_PWM(0);
			break;
		case GREEN:
			RED_PWM(0);
			GREEN_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			BLUE_PWM(0);
			break;
		case BLUE:
			RED_PWM(0);
			GREEN_PWM(0);
			BLUE_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			break;
		case PURPLE:
			RED_PWM((PWM_MAX / 2) * rgb_ctl.brightness_lvl_prcntg / 100);
			GREEN_PWM(0);
			BLUE_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			break;
		case YELLOW:
			RED_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			GREEN_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			BLUE_PWM(0);
			break;
		case CYAN:
			RED_PWM(0);
			GREEN_PWM((PWM_MAX / 2) * rgb_ctl.brightness_lvl_prcntg / 100);
			BLUE_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			break;
		case ORANGE:
			RED_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			GREEN_PWM((PWM_MAX / 4) * rgb_ctl.brightness_lvl_prcntg / 100);
			BLUE_PWM(0);
			break;
		case PINK:
			RED_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			GREEN_PWM(0);
			BLUE_PWM((PWM_MAX / 4) * rgb_ctl.brightness_lvl_prcntg / 100);
			break;
		case TURQUISE:
			RED_PWM(0);
			GREEN_PWM(PWM_MAX * rgb_ctl.brightness_lvl_prcntg / 100);
			BLUE_PWM((PWM_MAX / 2) * rgb_ctl.brightness_lvl_prcntg / 100);
			break;
		default:
			break;
	}
}

void rgb_ctl_set_brightness(brightness_cmd_t cmd)
{
	if (cmd == STEP_UP) {
		rgb_ctl.brightness_lvl_prcntg += BRIGHTNESS_STEPS;

		if (rgb_ctl.brightness_lvl_prcntg > 100) {
			rgb_ctl.brightness_lvl_prcntg = 100;
		}
	}
	else {
		if (rgb_ctl.brightness_lvl_prcntg <= BRIGHTNESS_STEPS) {
			rgb_ctl.brightness_lvl_prcntg = 0;
		}
		else {
			rgb_ctl.brightness_lvl_prcntg -= BRIGHTNESS_STEPS;
		}
	}

	rgb_ctl_set_color(rgb_ctl.current_color, rgb_ctl.brightness_lvl_prcntg);
}
