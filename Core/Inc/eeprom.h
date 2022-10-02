/*
 * eeprom.h
 *
 *  Created on: Oct 1, 2022
 *      Author: chris
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_I2C				I2C1
#define EEPROM_I2C_ADDRESS		(0x50<<1)
#define EEPROM_PAGE_SIZE		32U

#define EEPROM_VERSION			0x00

typedef struct {
	uint16_t version;
//	rgb_t colors[COLORS_NUM];
}eeprom_data_t;

typedef union {
	eeprom_data_t cfg;
	uint8_t buf[sizeof(eeprom_data_t)];
}eeprom_t;

extern eeprom_t eeprom;

void eeprom_set_defaults(void);
void eeprom_load(void);
void eeprom_save(void);

#endif /* INC_EEPROM_H_ */
