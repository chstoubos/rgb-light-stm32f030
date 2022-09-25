/*
 * rgb_ctl.c
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#include "rgb_ctl.h"
#include "main.h"

rgb_ctl_t rgb_ctl = {0};
volatile uint8_t effect_flag;

rgb_t rgb[] = {
		{PWM_MAX, PWM_MAX, PWM_MAX},	// white
		{PWM_MAX, 0, 0},				// red
		{0, PWM_MAX, 0},				// green
		{0, PWM_MAX, 0},				// blue
		{PWM_MAX / 2, 0, PWM_MAX},		// purple
		{PWM_MAX, PWM_MAX, 0},			// yellow
		{0, PWM_MAX / 2, PWM_MAX},		// cyan
		{PWM_MAX, PWM_MAX / 4, 0},		// orange
		{PWM_MAX, 0, PWM_MAX / 4},		// pink
		{0, PWM_MAX, PWM_MAX / 2}		// turquise
};

/**
 *
 */
void rgb_ctl_init(void)
{
	LL_TIM_EnableCounter(RGB_CTL_TIMER);

	rgb_ctl_set_color(DEFAULT_COLOR, DEFAULT_BRIGHTNESS);

	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, RED_CHANNEL);
	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, GREEN_CHANNEL);
	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, BLUE_CHANNEL);
}

/**
 *
 * @param color: enum fixed_colors_t
 * @param brightness: 0-100
 */
void rgb_ctl_set_color(int color, uint8_t brightness)
{
	rgb_ctl.mode = FIXED;
	rgb_ctl.current_color = color;
	rgb_ctl.brightness_lvl_prcntg = brightness;

	RED_PWM(rgb[rgb_ctl.current_color].r * rgb_ctl.brightness_lvl_prcntg / 100);
	GREEN_PWM(rgb[rgb_ctl.current_color].g * rgb_ctl.brightness_lvl_prcntg / 100);
	BLUE_PWM(rgb[rgb_ctl.current_color].b * rgb_ctl.brightness_lvl_prcntg / 100);
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

void rgb_ctl_set_speed(void)
{

}
