/*
 * rgb_ctl.c
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#include "main.h"

rgb_ctl_t rgb_ctl = {0};

//default colors
rgb_t default_rgb[COLORS_NUM] = {
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

	rgb_ctl_set_fixed_color(DEFAULT_COLOR, DEFAULT_BRIGHTNESS);

	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, RED_CHANNEL);
	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, GREEN_CHANNEL);
	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, BLUE_CHANNEL);
}

static void rgb_ctl_set_color(int idx, int8_t brightness)
{
	*rgb_pwm_vals[0] = eeprom.cfg.colors[idx].r * brightness / 100;
	*rgb_pwm_vals[1] = eeprom.cfg.colors[idx].g * brightness / 100;
	*rgb_pwm_vals[2] = eeprom.cfg.colors[idx].b * brightness / 100;
}

/**
 *
 * @param color: enum fixed_colors_t
 * @param brightness: 0-100
 */
void rgb_ctl_set_fixed_color(int color, int8_t brightness)
{
	rgb_ctl_rainbow_stop();

	rgb_ctl.mode = MODE_FIXED_COLOR;
	rgb_ctl.current_color = color;
	rgb_ctl.brightness_lvl_prcntg = brightness;

	rgb_ctl_set_color(rgb_ctl.current_color, rgb_ctl.brightness_lvl_prcntg);
}

void rgb_ctl_set_brightness(cmd_t cmd)
{
	if (rgb_ctl.mode != MODE_FIXED_COLOR) {
		return;
	}

	if (cmd == STEP_UP) {
		// if the brightness is below 5%, step up 1% each time
		if (rgb_ctl.brightness_lvl_prcntg <= BRIGHTNESS_STEP) {
			rgb_ctl.brightness_lvl_prcntg += BRIGHTNESS_STEP_MIN;
		}
		else {
			rgb_ctl.brightness_lvl_prcntg += BRIGHTNESS_STEP;
		}

		if (rgb_ctl.brightness_lvl_prcntg > 100) {
			rgb_ctl.brightness_lvl_prcntg = 100;
		}
	}
	else {
		// if the brightness is below 5%, step down 1% each time
		if (rgb_ctl.brightness_lvl_prcntg <= BRIGHTNESS_STEP) {
			rgb_ctl.brightness_lvl_prcntg -= BRIGHTNESS_STEP_MIN;
		}
		else {
			rgb_ctl.brightness_lvl_prcntg -= BRIGHTNESS_STEP;
		}

		if (rgb_ctl.brightness_lvl_prcntg < 0) {
			rgb_ctl.brightness_lvl_prcntg = 0;
		}
	}

	rgb_ctl_set_fixed_color(rgb_ctl.current_color, rgb_ctl.brightness_lvl_prcntg);
}

void rgb_ctl_rainbow_start(void)
{
	rgb_ctl.mode = MODE_RAINBOW;
	rgb_ctl.rainbow_current_step = 0;
	rgb_ctl.rainbow_pwm_step = RAINBOW_PWM_STEP_MIN;

	rgb_ctl_set_color(RED, 100);

	LL_TIM_EnableCounter(RAINBOW_TIMER);
	LL_TIM_EnableIT_UPDATE(RAINBOW_TIMER);
}

void rgb_ctl_rainbow_stop(void)
{
	LL_TIM_DisableIT_UPDATE(RAINBOW_TIMER);
	LL_TIM_DisableCounter(RAINBOW_TIMER);
}

void rgb_ctl_rainbow_set_speed(cmd_t cmd)
{
	if (cmd == STEP_UP) {
		rgb_ctl.rainbow_pwm_step += RAINBOW_PWM_STEP_MIN;

		if (rgb_ctl.rainbow_pwm_step > RAINBOW_PWM_STEP_MAX) {
			rgb_ctl.rainbow_pwm_step = RAINBOW_PWM_STEP_MAX;
		}
	}
	else {
		if (rgb_ctl.rainbow_pwm_step <= RAINBOW_PWM_STEP_MIN) {
			rgb_ctl.rainbow_pwm_step = RAINBOW_PWM_STEP_MIN;
		}
		else {
			rgb_ctl.rainbow_pwm_step -= RAINBOW_PWM_STEP_MIN;
		}
	}
}

/**
 * Place inside timer interrupt
 */
inline void rgb_ctl_rainbow(void)
{
	int32_t new_pwm = *rgb_pwm_vals[rainbow_channels[rgb_ctl.rainbow_current_step]] +
			rainbow_channels_direction[rgb_ctl.rainbow_current_step] * rgb_ctl.rainbow_pwm_step;

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
		if (rgb_ctl.rainbow_current_step >= ((sizeof(rainbow_channels) / sizeof(rainbow_channels[0])) -1)) {
			rgb_ctl.rainbow_current_step = 0;
		}
		else {
			rgb_ctl.rainbow_current_step++;
		}
	}

	*rgb_pwm_vals[rainbow_channels[rgb_ctl.rainbow_current_step]] = new_pwm;
}

static void rgb_ctl_flash_color(uint16_t r, uint16_t g, uint16_t b)
{
	// Store previous rgb values
	uint16_t previous_r = *rgb_pwm_vals[0];
	uint16_t previous_g = *rgb_pwm_vals[1];
	uint16_t previous_b = *rgb_pwm_vals[2];

	*rgb_pwm_vals[0] = 0;
	*rgb_pwm_vals[1] = 0;
	*rgb_pwm_vals[2] = 0;

	delay_ms(FLASH_PERIOD_ms);

	*rgb_pwm_vals[0] = r;
	*rgb_pwm_vals[1] = g;
	*rgb_pwm_vals[2] = b;

	delay_ms(FLASH_PERIOD_ms);

	*rgb_pwm_vals[0] = 0;
	*rgb_pwm_vals[1] = 0;
	*rgb_pwm_vals[2] = 0;

	delay_ms(FLASH_PERIOD_ms);

	*rgb_pwm_vals[0] = previous_r;
	*rgb_pwm_vals[1] = previous_g;
	*rgb_pwm_vals[2] = previous_b;
}

/**
 * @brief
 *
 * 		TODO: add timer
 */
void rgb_ctl_custom_change_channel(void)
{
	// If this mode is not already running
	if (rgb_ctl.mode != MODE_CUSTOM_COLOR)
	{
		rgb_ctl.mode = MODE_CUSTOM_COLOR;
		rgb_ctl.custom_color_channel_idx = 0;

		rgb_ctl.last_custom_color[0] = *rgb_pwm_vals[0];
		rgb_ctl.last_custom_color[1] = *rgb_pwm_vals[1];
		rgb_ctl.last_custom_color[2] = *rgb_pwm_vals[2];

		rgb_ctl_flash_color(PWM_MAX, 0, 0);
		return;
	}

	rgb_ctl.custom_color_channel_idx++;
	if (rgb_ctl.custom_color_channel_idx == 1)
	{
		rgb_ctl_flash_color(0, PWM_MAX, 0);
	}
	else if (rgb_ctl.custom_color_channel_idx == 2) {
		rgb_ctl_flash_color(0, 0, PWM_MAX);
	}
	else {
		// Flash the current color to indicate apply of color
		rgb_ctl_flash_color(*rgb_pwm_vals[0], *rgb_pwm_vals[1], *rgb_pwm_vals[2]);

		// Return to normal mode
		rgb_ctl.mode = MODE_FIXED_COLOR;
	}
}

void rgb_ctl_custom_color_run(cmd_t cmd)
{
	if (rgb_ctl.mode != MODE_CUSTOM_COLOR) {
		return;
	}

	int32_t channel_val = *rgb_pwm_vals[rgb_ctl.custom_color_channel_idx];

	if (cmd == STEP_UP)
	{
		channel_val += CUSTOM_COLOR_STEP;
		if (channel_val > PWM_MAX) {
			channel_val = PWM_MAX;
		}
	}
	else if (cmd == STEP_DOWN)
	{
		channel_val -= CUSTOM_COLOR_STEP;
		if (channel_val < 0) {
			channel_val = 0;
		}
	}

	*rgb_pwm_vals[rgb_ctl.custom_color_channel_idx] = channel_val;
	rgb_ctl.last_custom_color[rgb_ctl.custom_color_channel_idx] = channel_val;
}

void rgb_ctl_custom_color_save(unsigned int idx)
{
	// Save the current color in the selected memory
	eeprom.cfg.colors[idx].r = rgb_ctl.last_custom_color[0];
	eeprom.cfg.colors[idx].g = rgb_ctl.last_custom_color[1];
	eeprom.cfg.colors[idx].b = rgb_ctl.last_custom_color[2];

	rgb_ctl_flash_color(
			rgb_ctl.last_custom_color[0],
			rgb_ctl.last_custom_color[1],
			rgb_ctl.last_custom_color[2]);

	rgb_ctl_set_fixed_color(idx, 100);

	save_cfg_flag = 1;
}

void rgb_ctl_restore_defaults(void)
{
	eeprom_set_defaults();
	rgb_ctl_flash_color(
			eeprom.cfg.colors[0].r,
			eeprom.cfg.colors[0].g,
			eeprom.cfg.colors[0].b);

	rgb_ctl_set_fixed_color(0, 100);

	save_cfg_flag = 1;
}
