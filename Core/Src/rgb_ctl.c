/*
 * rgb_ctl.c
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#include "rgb_ctl.h"
#include "main.h"

/**
 * Initialize timer with 3-channel PWM 10KHz
 */
void rgb_ctl_init(void)
{
	LL_TIM_EnableCounter(RGB_CTL_TIMER);

	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, RED_CHANNEL);
	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, GREEN_CHANNEL);
	LL_TIM_CC_EnableChannel(RGB_CTL_TIMER, BLUE_CHANNEL);

	LL_TIM_OC_SetCompareCH1(RGB_CTL_TIMER, RGB_CTL_TIMER->ARR / 2);
	LL_TIM_OC_SetCompareCH2(RGB_CTL_TIMER, RGB_CTL_TIMER->ARR / 2);
	LL_TIM_OC_SetCompareCH4(RGB_CTL_TIMER, RGB_CTL_TIMER->ARR / 2);
}
