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

#define REQ_IO_IN	(GPIOE_PDIR & (1<<5))
#define ABORT_IO_IN	(GPIOB_PDIR & (1<<0))
#define HOLD_IO_IN	(GPIOB_PDIR & (1<<1))
#define RESUME_IO_IN	(GPIOB_PDIR & (1<<2))
#define LIMY_IO_IN	(GPIOD_PDIR & (1<<0))
#define LIMZ_IO_IN	(GPIOD_PDIR & (1<<2))
#define LIMX_IO_IN	(GPIOD_PDIR & (1<<5))

#define RDY_IO_LO	(GPIOB_PCOR = (1<<11))
#define RDY_IO_HI	(GPIOB_PSOR = (1<<11))

#define ENABLE_IO_LO	(GPIOA_PCOR = (1<<13))
#define ENABLE_IO_HI	(GPIOA_PSOR = (1<<13))

#define COOLANT_IO_LO	(GPIOB_PCOR = (1<<3))
#define COOLANT_IO_HI	(GPIOB_PSOR = (1<<3))

#define SPINEN_IO_LO	(GPIOD_PCOR = (1<<3))
#define SPINEN_IO_HI	(GPIOD_PSOR = (1<<3))

#define SPINDIR_IO_LO	(GPIOD_PCOR = (1<<1))
#define SPINDIR_IO_HI	(GPIOD_PSOR = (1<<1))

#define STEPHI_X	(GPIOD_PSOR = (1<<4))
#define STEPLO_X	(GPIOD_PCOR = (1<<4))
#define DIR_HI_X	(GPIOA_PSOR = (1<<5))
#define DIR_LO_X	(GPIOA_PCOR = (1<<5))

#define STEPHI_Y	(GPIOA_PSOR = (1<<12))
#define STEPLO_Y	(GPIOA_PCOR = (1<<12))
#define DIR_HI_Y	(GPIOC_PSOR = (1<<8))
#define DIR_LO_Y	(GPIOC_PCOR = (1<<8))

#define STEPHI_Z	(GPIOA_PSOR = (1<<4))
#define STEPLO_Z	(GPIOA_PCOR = (1<<4))
#define DIR_HI_Z	(GPIOC_PSOR = (1<<9))
#define DIR_LO_Z	(GPIOC_PCOR = (1<<9))

#define STEPHI_A	(GPIOD_PSOR = (1<<3))
#define STEPLO_A	(GPIOD_PCOR = (1<<3))
#define DIR_HI_A	(GPIOD_PSOR = (1<<1))
#define DIR_LO_A	(GPIOD_PCOR = (1<<1))

#endif /* __HARDWARE_H__ */




