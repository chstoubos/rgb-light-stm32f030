/*
 * rgb_ctl.h
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#ifndef INC_RGB_CTL_H_
#define INC_RGB_CTL_H_

#define RGB_CTL_TIMER			TIM3

#define RED_CHANNEL 			LL_TIM_CHANNEL_CH4
#define GREEN_CHANNEL			LL_TIM_CHANNEL_CH2
#define BLUE_CHANNEL			LL_TIM_CHANNEL_CH1

void rgb_ctl_init(void);

#endif /* INC_RGB_CTL_H_ */
