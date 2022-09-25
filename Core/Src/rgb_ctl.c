/*
 * rgb_ctl.c
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#include "main.h"

rgb_ctl_t rgb_ctl = {0};
rainbow_effect_t rainbow;

rgb_t rgb[] = {
		{PWM_MAX, PWM_MAX, PWM_MAX},	// white
		{PWM_MAX, 0, 0},				// red
		{0, PWM_MAX, 0},				// green
		{0, 0, PWM_MAX},				// blue
		{PWM_MAX / 2, 0, PWM_MAX},		// purple
		{PWM_MAX, PWM_MAX, 0},			// yellow
		{0, PWM_MAX / 2, PWM_MAX},		// cyan
		{PWM_MAX, PWM_MAX / 4, 0},		// orange
		{PWM_MAX, 0, PWM_MAX / 4},		// pink
		{0, PWM_MAX, PWM_MAX / 2}		// turquise
};

uint8_t rainbow_channels[] = {1, 0, 2, 1, 0, 2};
int8_t rainbow_channels_direction[] = {1, -1, 1, -1, 1, -1};
volatile uint32_t *rgb_pwm_vals[] = {
		PWM_VAL_RED,
		PWM_VAL_GREEN,
		PWM_VAL_BLUE
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

void rgb_ctl_rainbow_start(void)
{
	rainbow.current_step = 0;

	*rgb_pwm_vals[0] = PWM_MAX;
	*rgb_pwm_vals[1] = 0;
	*rgb_pwm_vals[2] = 0;

	LL_TIM_EnableCounter(EFFECT_TIMER);
	LL_TIM_EnableIT_UPDATE(EFFECT_TIMER);
}

void rgb_ctl_rainbow_stop(void)
{
	LL_TIM_DisableCounter(EFFECT_TIMER);
}

inline void rgb_ctl_rainbow(void)
{
	int32_t new_pwm = *rgb_pwm_vals[rainbow_channels[rainbow.current_step]] +
			rainbow_channels_direction[rainbow.current_step] * EFFECT_PWM_STEP;

	unsigned int next_step = 0;
	if (new_pwm < 0)
	{
		new_pwm = 0;
		next_step = 1;

	}
	else if (new_pwm > PWM_MAX)
	{
		new_pwm = PWM_MAX;
		next_step = 1;
	}

	if (next_step)
	{
		if (rainbow.current_step >= ((sizeof(rainbow_channels) / sizeof(rainbow_channels[0])) -1)) {
			rainbow.current_step = 0;
		}
		else {
			rainbow.current_step++;
		}
	}

	*rgb_pwm_vals[rainbow_channels[rainbow.current_step]] = new_pwm;
}



















