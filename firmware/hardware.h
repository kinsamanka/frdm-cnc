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

	input	   	- Ard	- frdm
	------------------------------

	ABORT		- A0	- B0
	FEEDHOLD	- A1	- B1
	RESUME		- A2	- B2
	X_LIM		- D9	- D5
	Y_LIM		- D10	- D0
	Z_LIM		- D11	- D2

	output	   	- Ard	- frdm
	------------------------------

	COOLANT_EN	- A3	- B3
	STEP_X		- D2	- D4
	STEP_Y		- D3	- A12
	STEP_Z		- D4	- A4
	DIR_X		- D5	- A5
	DIR_Y		- D6	- C8
	DIR_Z		- D7	- C9
	MOTORS_EN	- D8	- A13
	SPINDLE_DIR	- D12	- D3
	SPINDLE_EN	- D13	- D1
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

#define X_LIM_IN	(GPIOD_PDIR & (1<<5))
#define Y_LIM_IN	(GPIOD_PDIR & (1<<0))
#define Z_LIM_IN	(GPIOD_PDIR & (1<<2))
#define ABORT_IN	(GPIOB_PDIR & (1<<0))
#define FHOLD_IN	(GPIOB_PDIR & (1<<1))
#define RSUME_IN	(GPIOB_PDIR & (1<<2))

#define ENABLE_LO	(GPIOA_PCOR = (1<<13))
#define ENABLE_HI	(GPIOA_PSOR = (1<<13))
#define COOLANT_LO	(GPIOB_PCOR = (1<<3))
#define COOLANT_HI	(GPIOB_PSOR = (1<<3))
#define SPINEN_LO	(GPIOD_PCOR = (1<<3))
#define SPINEN_HI	(GPIOD_PSOR = (1<<3))
#define SPINDIR_LO	(GPIOD_PCOR = (1<<1))
#define SPINDIR_HI	(GPIOD_PSOR = (1<<1))

#define STEP_X_HI	(GPIOD_PSOR = (1<<4))
#define STEP_X_LO	(GPIOD_PCOR = (1<<4))
#define DIR_X_HI	(GPIOA_PSOR = (1<<5))
#define DIR_X_LO	(GPIOA_PCOR = (1<<5))

#define STEP_Y_HI	(GPIOA_PSOR = (1<<12))
#define STEP_Y_LO	(GPIOA_PCOR = (1<<12))
#define DIR_Y_HI	(GPIOC_PSOR = (1<<8))
#define DIR_Y_LO	(GPIOC_PCOR = (1<<8))

#define STEP_Z_HI	(GPIOA_PSOR = (1<<4))
#define STEP_Z_LO	(GPIOA_PCOR = (1<<4))
#define DIR_Z_HI	(GPIOC_PSOR = (1<<9))
#define DIR_Z_LO	(GPIOC_PCOR = (1<<9))

#define STEP_A_HI	(GPIOD_PSOR = (1<<3))
#define STEP_A_LO	(GPIOD_PCOR = (1<<3))
#define DIR_A_HI	(GPIOD_PSOR = (1<<1))
#define DIR_A_LO	(GPIOD_PCOR = (1<<1))

#endif /* __HARDWARE_H__ */
