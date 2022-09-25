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
#define EFFECT_TIMER			TIM16

#define RED_CHANNEL 			LL_TIM_CHANNEL_CH4
#define GREEN_CHANNEL			LL_TIM_CHANNEL_CH2
#define BLUE_CHANNEL			LL_TIM_CHANNEL_CH1

#define PWM_VAL_RED				&RGB_CTL_TIMER->CCR4
#define PWM_VAL_GREEN			&RGB_CTL_TIMER->CCR2
#define PWM_VAL_BLUE			&RGB_CTL_TIMER->CCR1

#define RED_PWM(x)				LL_TIM_OC_SetCompareCH4(RGB_CTL_TIMER,x)
#define GREEN_PWM(x)			LL_TIM_OC_SetCompareCH2(RGB_CTL_TIMER,x)
#define BLUE_PWM(x)				LL_TIM_OC_SetCompareCH1(RGB_CTL_TIMER,x)

#warning: This is hardcoded, otherwise cannot use the array definition
#define PWM_MAX					(4799 + 1)

#define BRIGHTNESS_STEPS		5
#define DEFAULT_BRIGHTNESS		100U
#define DEFAULT_COLOR			WHITE

#define EFFECT_PWM_STEP			1U

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

// 0-4800 ARR
typedef struct {
	uint16_t r;
	uint16_t g;
	uint16_t b;
}rgb_t;

typedef struct {
	uint8_t current_step;
}rainbow_effect_t;

typedef struct {
	rgb_mode_t mode;
	uint8_t brightness_lvl_prcntg;
	fixed_colors_t current_color;
}rgb_ctl_t;

extern rgb_ctl_t rgb_ctl;
extern volatile uint8_t effect_flag;

void rgb_ctl_init(void);
void rgb_ctl_set_color(int color, uint8_t brightness);
void rgb_ctl_set_brightness(brightness_cmd_t cmd);
void rgb_ctl_rainbow_start(void);
void rgb_ctl_rainbow_stop(void);
void rgb_ctl_rainbow(void);

#endif /* INC_RGB_CTL_H_ */
