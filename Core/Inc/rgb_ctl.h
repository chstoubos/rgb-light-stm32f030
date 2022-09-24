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

#define RED_CHANNEL 			LL_TIM_CHANNEL_CH4
#define GREEN_CHANNEL			LL_TIM_CHANNEL_CH2
#define BLUE_CHANNEL			LL_TIM_CHANNEL_CH1

#define RED_PWM(x)				LL_TIM_OC_SetCompareCH4(RGB_CTL_TIMER,x)
#define GREEN_PWM(x)			LL_TIM_OC_SetCompareCH2(RGB_CTL_TIMER,x)
#define BLUE_PWM(x)				LL_TIM_OC_SetCompareCH1(RGB_CTL_TIMER,x)

#define PWM_MAX					(RGB_CTL_TIMER->ARR + 1)

#define BRIGHTNESS_STEPS		5
#define DEFAULT_BRIGHTNESS		100U
#define DEFAULT_COLOR			WHITE

typedef enum {
	FIXED = 0,
	EFFECT,
	SLEEP_TIMER_ON,
	SLEEP_TIMER_OFF
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
}brightness_cmd_t;

typedef struct {
	rgb_mode_t mode;
	uint8_t brightness_lvl_prcntg;
	fixed_colors_t current_color;
}rgb_ctl_t;

extern rgb_ctl_t rgb_ctl;

void rgb_ctl_init(void);
void rgb_ctl_set_color(fixed_colors_t color, uint8_t brightness);
void rgb_ctl_set_brightness(brightness_cmd_t cmd);

#endif /* INC_RGB_CTL_H_ */
