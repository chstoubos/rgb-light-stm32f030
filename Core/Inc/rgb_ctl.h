/*
 * rgb_ctl.h
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#ifndef INC_RGB_CTL_H_
#define INC_RGB_CTL_H_

#include "main.h"

#define RGB_CTL_TIMER			TIM3
#define RAINBOW_TIMER			TIM16

#define COLORS_NUM				10

#define RED_CHANNEL 			LL_TIM_CHANNEL_CH4
#define GREEN_CHANNEL			LL_TIM_CHANNEL_CH2
#define BLUE_CHANNEL			LL_TIM_CHANNEL_CH1

#define PWM_VAL_RED				&RGB_CTL_TIMER->CCR4
#define PWM_VAL_GREEN			&RGB_CTL_TIMER->CCR2
#define PWM_VAL_BLUE			&RGB_CTL_TIMER->CCR1

#warning: This is hardcoded, otherwise cannot use the array definition
#define PWM_MAX					(4799 + 1)

#define DEFAULT_BRIGHTNESS		100U
#define DEFAULT_COLOR			WHITE

#define BRIGHTNESS_STEP			5
#define BRIGHTNESS_STEP_MIN		1

#define RAINBOW_PWM_STEP_MIN	1U
#define RAINBOW_PWM_STEP_MAX	1000U

typedef enum {
	FIXED = 0,
	RAINBOW,
	SLEEP_TIMER,
	SLEEP_NO_TIMER
}rgb_mode_t;

typedef enum {
	WHITE = 0,
	RED,
	GREEN,
	BLUE,
	PURPLE,
	YELLOW,
	CYAN,
	ORANGE,
	PINK,
	TURQUISE
}fixed_colors_t;

typedef enum {
	STEP_UP = 0,
	STEP_DOWN
}cmd_t;

// 0-4800 PWM ARR
typedef struct {
	uint16_t r;
	uint16_t g;
	uint16_t b;
}rgb_t;

typedef struct {
	rgb_mode_t mode;
	int8_t brightness_lvl_prcntg;
	fixed_colors_t current_color;

	uint8_t rainbow_current_step;
	uint16_t rainbow_pwm_step;
}rgb_ctl_t;

extern rgb_t default_rgb[COLORS_NUM];

void rgb_ctl_init(void);
void rgb_ctl_set_color(int color, int8_t brightness);
void rgb_ctl_set_brightness(cmd_t cmd);
void rgb_ctl_rainbow_start(void);
void rgb_ctl_rainbow_stop(void);
void rgb_ctl_rainbow_set_speed(cmd_t cmd);
void rgb_ctl_rainbow(void);

#endif /* INC_RGB_CTL_H_ */
