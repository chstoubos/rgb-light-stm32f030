/*
 * eeprom.c
 *
 *  Created on: Oct 1, 2022
 *      Author: chris
 */

#include "main.h"

eeprom_t eeprom;

/**
 * Page write (32bytes for 24AA64)
 * @param addr
 * @param data
 * @param len
 * @return
 */
static int eeprom_write(uint16_t addr, uint8_t *data, uint16_t len)
{
	LL_I2C_HandleTransfer(
						EEPROM_I2C,
						EEPROM_I2C_ADDRESS,
						LL_I2C_ADDRSLAVE_7BIT,
						len + 2,  // +2 because of the address
						LL_I2C_MODE_AUTOEND,
						LL_I2C_GENERATE_START_WRITE);

	// Send address to write
	while (!LL_I2C_IsActiveFlag_TXE(EEPROM_I2C)) ;
	LL_I2C_TransmitData8(EEPROM_I2C, addr & 0xFF);

	while (!LL_I2C_IsActiveFlag_TXE(EEPROM_I2C)) ;
//	LL_I2C_TransmitData8(EEPROM_I2C, (addr >> 8) & 0xFF);
	LL_I2C_TransmitData8(EEPROM_I2C, 0x01);

	// Send data
	for (unsigned int i = 0; i < len; i++)
	{
		while (!LL_I2C_IsActiveFlag_TXE(EEPROM_I2C)) ;
		LL_I2C_TransmitData8(EEPROM_I2C, data[i]);
	}

	while(!LL_I2C_IsActiveFlag_STOP(EEPROM_I2C)) ;
	LL_I2C_ClearFlag_STOP(EEPROM_I2C);

	return 0;
}

/**
 * Random Read
 * @param addr
 * @param buf
 * @param len
 * @return
 */
static int eeprom_read(uint16_t addr, uint8_t *data, uint16_t len)
{
	unsigned int idx = 0;

	// Send Address to read
	LL_I2C_HandleTransfer(
						EEPROM_I2C,
						EEPROM_I2C_ADDRESS,
						LL_I2C_ADDRSLAVE_7BIT,
						len,
						LL_I2C_MODE_AUTOEND,
						LL_I2C_GENERATE_START_WRITE);

	// Send address to write
	while (!LL_I2C_IsActiveFlag_TXE(EEPROM_I2C)) ;
	LL_I2C_TransmitData8(EEPROM_I2C, addr & 0xFF);

	while (!LL_I2C_IsActiveFlag_TXE(EEPROM_I2C)) ;
	LL_I2C_TransmitData8(EEPROM_I2C, (addr >> 8) & 0xFF);

	while(!LL_I2C_IsActiveFlag_STOP(EEPROM_I2C)) ;
	LL_I2C_ClearFlag_STOP(EEPROM_I2C);

	// Repeated start
	LL_I2C_HandleTransfer(
						EEPROM_I2C,
						EEPROM_I2C_ADDRESS,
						LL_I2C_ADDRSLAVE_7BIT,
						len,
						LL_I2C_MODE_AUTOEND,
						LL_I2C_GENERATE_START_READ);

	while(!LL_I2C_IsActiveFlag_STOP(EEPROM_I2C))  // Loop until end of transfer received (STOP flag raised)
	{
		if(LL_I2C_IsActiveFlag_RXNE(EEPROM_I2C))
		{
			// RXNE flag is cleared by reading the data
			data[idx++] = LL_I2C_ReceiveData8(EEPROM_I2C);
		}
	}

	LL_I2C_ClearFlag_STOP(I2C1);

	return 1;
}

void eeprom_set_defaults(void)
{
	eeprom.cfg.version = 0x1010;
//	for (unsigned int i = 0; i < COLORS_NUM; i++)
//	{
//		eeprom.cfg.colors[i].r = default_rgb[i].r;
//		eeprom.cfg.colors[i].g = default_rgb[i].g;
//		eeprom.cfg.colors[i].b = default_rgb[i].b;
//	}
}

void eeprom_load(void)
{
	eeprom_read(0, eeprom.buf, sizeof(eeprom.buf) / sizeof(eeprom.buf[0]));
}

void eeprom_save(void)
{
	eeprom_write(0, eeprom.buf, sizeof(eeprom.buf) / sizeof(eeprom.buf[0]));
}
