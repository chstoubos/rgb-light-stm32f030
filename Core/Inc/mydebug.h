/*
 * mydebug.h
 *
 *  Created on: Nov 7, 2021
 *      Author: chris
 */

#ifndef INC_MYDEBUG_H_
#define INC_MYDEBUG_H_

#include <stdio.h>

//#define MY_DEBUG	0
#define MY_DEBUG 	1

#if (MY_DEBUG == 1)
#warning building with debug prints
#endif

#define debug_printf(fmt, ...)	do { if (MY_DEBUG) printf(" *** %s(): " fmt, __func__, __VA_ARGS__); } while (0)
#define debug_print(s)			if (MY_DEBUG) printf(s);

#endif /* INC_MYDEBUG_H_ */
