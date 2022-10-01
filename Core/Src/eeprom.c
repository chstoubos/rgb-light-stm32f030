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
	if (len > EEPROM_PAGE_SIZE) {
		return 1;
	}

	// TODO: timeout
	// Start condition
	LL_I2C_GenerateStartCondition(EEPROM_I2C);
	while (!LL_I2C_IsActiveFlag_BUSY(EEPROM_I2C)) ;

	// Slave address WRITE
	LL_I2C_TransmitData8(EEPROM_I2C, EEPROM_I2C_ADDRESS);
	while (!LL_I2C_IsActiveFlag_ADDR(EEPROM_I2C)) ;
	LL_I2C_ClearFlag_ADDR(EEPROM_I2C);

	// EEPROM address HIGH byte
	LL_I2C_TransmitData8(EEPROM_I2C, addr & 0xFF);
	while (!LL_I2C_IsActiveFlag_TXE(EEPROM_I2C)) ;

	// EEPROM address LOW byte
	LL_I2C_TransmitData8(EEPROM_I2C, (addr >> 8) & 0xFF);
	while (!LL_I2C_IsActiveFlag_TXE(EEPROM_I2C)) ;

	// Send data
	for (unsigned int i = 0; i < len; i++)
	{
		LL_I2C_TransmitData8(EEPROM_I2C, data[i]);
		while (!LL_I2C_IsActiveFlag_TXE(EEPROM_I2C)) ;
	}

	// Stop condition
	LL_I2C_GenerateStopCondition(EEPROM_I2C);
	while (!LL_I2C_IsActiveFlag_STOP(EEPROM_I2C)) ;

	return 0;
}

/**
 * Sequential read
 * @param addr
 * @param buf
 * @param len
 * @return
 */
static int eeprom_read(uint16_t addr, uint8_t *data, uint16_t len)
{
	// Start condition
	LL_I2C_GenerateStartCondition(EEPROM_I2C);
	while (!LL_I2C_IsActiveFlag_BUSY(EEPROM_I2C)) ;

	//slave address + READ
	LL_I2C_TransmitData8(EEPROM_I2C, EEPROM_I2C_ADDRESS + 1);
	while (!LL_I2C_IsActiveFlag_ADDR(EEPROM_I2C)) ;
	LL_I2C_ClearFlag_ADDR(EEPROM_I2C);

    for(unsigned int i = 0; i < (len - 1); i++)
    {
		LL_I2C_AcknowledgeNextData(EEPROM_I2C, LL_I2C_ACK);
		while (!LL_I2C_IsActiveFlag_RXNE(EEPROM_I2C)) ;
		data[i] = LL_I2C_ReceiveData8(EEPROM_I2C);
    }

	LL_I2C_AcknowledgeNextData(EEPROM_I2C, LL_I2C_NACK);

	//NEEDS STOP CONDITION BEFORE POLLING FOR THE RXNE
	LL_I2C_GenerateStopCondition(EEPROM_I2C);

	while (!LL_I2C_IsActiveFlag_RXNE(EEPROM_I2C)) ;
	data[len - 1] = LL_I2C_ReceiveData8(EEPROM_I2C);

	return 1;
}

void eeprom_set_defaults(void)
{
	for (unsigned int i = 0; i < COLORS_NUM; i++)
	{
		eeprom.cfg.colors[i].r = default_rgb[i].r;
		eeprom.cfg.colors[i].g = default_rgb[i].g;
		eeprom.cfg.colors[i].b = default_rgb[i].b;
	}
}

void eeprom_load(void)
{
	eeprom_read(0, eeprom.buf, sizeof(eeprom.buf) / sizeof(eeprom.buf[0]));
}

void eeprom_save(void)
{
	eeprom_write(0, eeprom.buf, sizeof(eeprom.buf) / sizeof(eeprom.buf[0]));
}
