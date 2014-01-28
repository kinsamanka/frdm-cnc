/*    Copyright (C) 2013 GP Orcullo
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include <MKL25Z4.h>

/*
	Pin Usage:

	output	- mega	- frdm
	----------------------

	STEP_X	- A0	- B0
	DIR_X	- A1	- B1
	EN_Y	- A2	- B2
	EN_E0	- D24	- B9
	STEP_E0	- D26	- B10
	DIR_E0	- D28	- B11

	LED	- D13	- D1
	PS_ON	- D12	- D3

	EN_E1	- D30	- E2
	DIR_E1	- D34	- E3
	STEP_E1	- D36	- E4
	EN_X	- D38	- E5
	STEP_Z	- D46	- E20
	DIR_Z	- D48	- E21
	STEP_Y	- A6	- E22
	DIR_Y	- A7	- E23
	EN_Z	- A8	- E29
	
	input	- mega	- frdm
	----------------------

	Y_MIN	- D4	- A4
	Y_MAX	- D5	- A5
	X_MIN	- D3	- A12
	Z_MAX	- D19	- C10
	Z_MIN	- D18	- C11
	X_MAX	- D2	- D4

	pwm	- mega	- frdm
	----------------------

	HTR_2	- D8	- A13
	HTR_0	- D10	- D0
	HTR_1	- D9	- D5

	i2c	- mega	- frdm
	----------------------

	SDA	- D7	- C9
	SCL	- D6	- C8

	analog	- mega	- frdm
	----------------------

	THM_0	- A3	- B3
	THM_2	- A5	- C1
	THM_1	- A4	- C2

*/

#define RED		(18)
#define RED_SHIFT	(1<<RED)

#define RED_OFF		(GPIOB_PSOR = RED_SHIFT)
#define RED_ON		(GPIOB_PCOR = RED_SHIFT)
#define RED_TOGGLE	(GPIOB_PTOR = RED_SHIFT)

#define GREEN		(19)
#define GREEN_SHIFT	(1<<GREEN)

#define GREEN_OFF	(GPIOB_PSOR = GREEN_SHIFT)
#define GREEN_ON	(GPIOB_PCOR = GREEN_SHIFT)
#define GREEN_TOGGLE	(GPIOB_PTOR = GREEN_SHIFT)

#define BLUE		(1)
#define BLUE_SHIFT	(1<<BLUE)

#define BLUE_OFF	(GPIOD_PSOR = BLUE_SHIFT)
#define BLUE_ON		(GPIOD_PCOR = BLUE_SHIFT)
#define BLUE_TOGGLE	(GPIOD_PTOR = BLUE_SHIFT)

#define REQ_IN		(GPIOC_PDIR & (1<<3))

#define RDY_LO		(GPIOC_PCOR = (1<<0))
#define RDY_HI		(GPIOC_PSOR = (1<<0))

#define X_MIN_IN	(GPIOA_PDIR & (1<<12))
#define X_MAX_IN	(GPIOD_PDIR & (1<<4))
#define Y_MIN_IN	(GPIOA_PDIR & (1<<4))
#define Y_MAX_IN	(GPIOA_PDIR & (1<<5))
#define Z_MIN_IN	(GPIOC_PDIR & (1<<11))
#define Z_MAX_IN	(GPIOC_PDIR & (1<<10))

#define EN_X_LO		(GPIOE_PCOR = (1<<5))
#define EN_X_HI		(GPIOE_PSOR = (1<<5))
#define EN_Y_LO		(GPIOB_PCOR = (1<<2))
#define EN_Y_HI		(GPIOB_PSOR = (1<<2))
#define EN_Z_LO		(GPIOE_PCOR = (1<<29))
#define EN_Z_HI		(GPIOE_PSOR = (1<<29))
#define EN_A_LO		(GPIOB_PCOR = (1<<9))
#define EN_A_HI		(GPIOB_PSOR = (1<<9))
#define EN_B_LO		(GPIOE_PCOR = (1<<2))
#define EN_B_HI		(GPIOE_PSOR = (1<<2))
#define LED_TOGGLE	(GPIOD_PTOR = (1<<1))

#define STEP_X_HI	(GPIOB_PSOR = (1<<0))
#define STEP_X_LO	(GPIOB_PCOR = (1<<0))
#define DIR_X_HI	(GPIOB_PSOR = (1<<1))
#define DIR_X_LO	(GPIOB_PCOR = (1<<1))

#define STEP_Y_HI	(GPIOE_PSOR = (1<<22))
#define STEP_Y_LO	(GPIOE_PCOR = (1<<22))
#define DIR_Y_HI	(GPIOE_PSOR = (1<<23))
#define DIR_Y_LO	(GPIOE_PCOR = (1<<23))

#define STEP_Z_HI	(GPIOE_PSOR = (1<<20))
#define STEP_Z_LO	(GPIOE_PCOR = (1<<20))
#define DIR_Z_HI	(GPIOE_PSOR = (1<<21))
#define DIR_Z_LO	(GPIOE_PCOR = (1<<21))

#define STEP_A_HI	(GPIOB_PSOR = (1<<10))
#define STEP_A_LO	(GPIOB_PCOR = (1<<10))
#define DIR_A_HI	(GPIOB_PSOR = (1<<11))
#define DIR_A_LO	(GPIOB_PCOR = (1<<11))

#define STEP_B_HI	(GPIOE_PSOR = (1<<4))
#define STEP_B_LO	(GPIOE_PCOR = (1<<4))
#define DIR_B_HI	(GPIOE_PSOR = (1<<3))
#define DIR_B_LO	(GPIOE_PCOR = (1<<3))

#endif /* __HARDWARE_H__ */
