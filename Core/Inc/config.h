/*
 * config.h
 *
 *  Created on: Sep 17, 2022
 *      Author: chris
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define delay_ms(x)						LL_mDelay(x)

#define DBG_USART						USART1

#define DBG_TOGGLE						LL_GPIO_TogglePin(DBG_GPIO_Port, DBG_Pin)
#define DBG_HIGH						LL_GPIO_SetOutputPin(DBG_GPIO_Port, DBG_Pin)
#define DBG_LOW							LL_GPIO_ResetOutputPin(DBG_GPIO_Port, DBG_Pin)

#define ADC_MEAS_NUM					16
#define PR_ADC							ADC1
#define ADC_PR_CHANNEL					LL_ADC_CHANNEL_1

#endif /* INC_CONFIG_H_ */
