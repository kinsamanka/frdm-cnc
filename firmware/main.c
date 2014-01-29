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

#include <string.h>

#include "hardware.h"
#include "stepgen.h"

#define SPIBUFSIZE	32		/* BCM2835 SPI buffer size */
#define BUFSIZE		(SPIBUFSIZE/4)

static volatile uint32_t txBuf[BUFSIZE] __attribute__((aligned(32))),
       rxBuf[BUFSIZE] __attribute__((aligned(32)));

static volatile int spi_data_ready;

static inline void enable_irq(int n)
{
	NVIC_ICPR |= 1 << ((n - 16)%32);
	NVIC_ISER |= 1 << ((n - 16)%32);
}

static inline void disable_irq(int n)
{
	NVIC_ICER = 1 << ((n - 16)%32);
}

static void initLED(void)
{
	/* enable port B & D clock*/
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;

	/* Set the PTB18 pin multiplexer to GPIO mode */
	PORTB_PCR18 = PORT_PCR_MUX(1);
	PORTB_PCR19 = PORT_PCR_MUX(1);
	PORTD_PCR1  = PORT_PCR_MUX(1);

	/* Set the pins direction to output */
	GPIOB_PDDR |= RED_SHIFT | GREEN_SHIFT;
	GPIOD_PDDR |= BLUE_SHIFT;

	GPIOB_PSOR = RED_SHIFT | GREEN_SHIFT;
	GPIOD_PSOR = BLUE_SHIFT;
}

static void initPIT(void)
{
	/* enable PIT clock*/
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; // enable PIT module
	
	enable_irq(INT_PIT);

	PIT_MCR = 0x00;  // MDIS = 0  enables timer
	PIT_TCTRL1 = 0x00; // disable PIT0
	PIT_LDVAL1 = 299; // 40 Khz base freq
	PIT_TCTRL1 = PIT_TCTRL_TIE_MASK; // enable PIT0 and interrupt
	PIT_TFLG1 = 0x01; // clear flag
	PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK;
}

static void initDMA(void)
{
	/* enable DMAMUX & DMA clock */
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;
	
	enable_irq(INT_DMA0);

	/* setup SPI rx DMA channel */
	DMA_SAR0 = (uint32_t)(&(SPI0_D));		/* SPI source */
	DMA_DAR0 = (uint32_t)&(rxBuf);
	DMA_DSR_BCR0 = DMA_DSR_BCR_DONE_MASK;
	DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(SPIBUFSIZE);
	DMA_DCR0 = DMA_DCR_ERQ_MASK |
	        DMA_DCR_CS_MASK |
	        DMA_DCR_DMOD(2) |
	        DMA_DCR_D_REQ_MASK |
	        DMA_DCR_DSIZE(1) |
	        DMA_DCR_SSIZE(1) | DMA_DCR_EINT_MASK |
	        DMA_DCR_DINC_MASK;

	/* route SPI rx request to channel 0 */
	DMAMUX0_CHCFG0 = 0;
	DMAMUX0_CHCFG0 = DMAMUX_CHCFG_SOURCE(16);
	DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK;

	/* setup SPI tx DMA channel */
	DMA_SAR1 = (uint32_t)&(txBuf);
	DMA_DAR1 = (uint32_t)(&(SPI0_D));		/* SPI dest */
	DMA_DSR_BCR1 = DMA_DSR_BCR_DONE_MASK;
	DMA_DSR_BCR1 = DMA_DSR_BCR_BCR(SPIBUFSIZE);
	DMA_DCR1 = DMA_DCR_ERQ_MASK |
	        DMA_DCR_CS_MASK |
	        DMA_DCR_SMOD(2) |
	        DMA_DCR_D_REQ_MASK |
	        DMA_DCR_DSIZE(1) |
	        DMA_DCR_SSIZE(1) |
	        DMA_DCR_SINC_MASK;

	/* route SPI tx request to channel 1 */
	DMAMUX0_CHCFG1 = 0;
	DMAMUX0_CHCFG1 = DMAMUX_CHCFG_SOURCE(17);
	DMAMUX0_CHCFG1 |= DMAMUX_CHCFG_ENBL_MASK;
}

static void initSPI(void)
{
	/* enable SPI0 clock */
	SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;
	/* enable ports C clock */
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

	/* Disable SPI */
	SPI0_C1 = 0x00U;

	/* configure IO pins */
	PORTC_PCR7 = PORT_PCR_MUX(0x02);	/* MISO */
	PORTC_PCR6 = PORT_PCR_MUX(0x02);	/* MOSI */
	PORTC_PCR5 = PORT_PCR_MUX(0x02);	/* SCLK */
	PORTC_PCR4 = PORT_PCR_MUX(0x02);	/* CS */

	/* SPI0_C0 register already setup for slave mode, CPHA = 0, CPOL = 0 */
	/* Enable DMA on box rx and tx */
	SPI0_C2 = SPI_C2_RXDMAE_MASK | SPI_C2_TXDMAE_MASK;
	/* Start SPI */
	SPI0_C1 |= SPI_C1_SPE_MASK;
}

static void initGPIO(void)
{
	/* enable port A - D clock */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK|
	             SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK; 

	/* inputs */
	PORTB_PCR0 = PORT_PCR_MUX(1);	/* abort */
	PORTB_PCR1 = PORT_PCR_MUX(1);	/* feed hold */
	PORTB_PCR2 = PORT_PCR_MUX(1);	/* resume */
	PORTC_PCR3 = PORT_PCR_MUX(1);	/* data request */
	PORTD_PCR0 = PORT_PCR_MUX(1);	/* y limit */
	PORTD_PCR2 = PORT_PCR_MUX(1);	/* z limit */
	PORTD_PCR5 = PORT_PCR_MUX(1);	/* x limit */

	/* Set the pins direction to input */
	GPIOB_PDDR &= ~((1<<0)|(1<<1)|(1<<2));
	GPIOC_PDDR &= ~(1<<3);
	GPIOD_PDDR &= ~((1<<0)|(1<<2)|(1<<5));

	/* outputs */
	PORTA_PCR4 = PORT_PCR_MUX(1);	/* step z */
	PORTA_PCR5 = PORT_PCR_MUX(1);	/* dir x */
	PORTA_PCR12 = PORT_PCR_MUX(1);	/* step y */
	PORTA_PCR13 = PORT_PCR_MUX(1);	/* enable */
	PORTB_PCR3 = PORT_PCR_MUX(1);	/* coolant */
	PORTC_PCR0 = PORT_PCR_MUX(1);	/* data ready */
	PORTC_PCR8 = PORT_PCR_MUX(1);	/* dir y */
	PORTC_PCR9 = PORT_PCR_MUX(1);	/* dir z */
	PORTD_PCR1 = PORT_PCR_MUX(1);	/* dir a */
	PORTD_PCR3 = PORT_PCR_MUX(1);	/* step a */
	PORTD_PCR4 = PORT_PCR_MUX(1);	/* step x */

	/* Set the initial output state to low */
	GPIOA_PCOR = (1<<4)|(1<<5)|(1<<12);
	GPIOB_PCOR = (1<<3);
	GPIOC_PCOR = (1<<8)|(1<<9);
	GPIOD_PCOR = (1<<1)|(1<<3)|(1<<4);

	/* data ready active low */
	GPIOC_PSOR = (1<<0);
	/* motor enable active low */
	GPIOA_PSOR = (1<<13);

	/* Set the pins direction to output */
	GPIOA_PDDR |= (1<<4)|(1<<5)|(1<<12)|(1<<13);
	GPIOB_PDDR |= (1<<3);
	GPIOC_PDDR |= (1<<0)|(1<<8)|(1<<9);
	GPIOD_PDDR |= (1<<1)|(1<<3)|(1<<4);
}

static uint32_t read_inputs(void)
{
	uint32_t x;

	x  = (X_LIM_IN ? 1 : 0) << 0;
	x |= (Y_LIM_IN ? 1 : 0) << 1;
	x |= (Z_LIM_IN ? 1 : 0) << 2;
	x |= (ABORT_IN ? 1 : 0) << 3;
	x |= (FHOLD_IN ? 1 : 0) << 4;
	x |= (RSUME_IN ? 1 : 0) << 5;

	return x;
}

static void update_outputs(uint32_t x)
{
	if (x & (1 << 0))
		ENABLE_LO;
	else
		ENABLE_HI;
	
	if (x & (1 << 1))
		COOLANT_HI;
	else
		COOLANT_LO;
	
	/* Spindle enable/dir signals are available
	   only when A axis is disabled */
	if (MAXGEN == 3) {
		if (x & (1 << 2))
			SPINEN_HI;
		else
			SPINEN_LO;
		
		if (x & (1 << 3))
			SPINDIR_HI;
		else
			SPINDIR_LO;
	}
}

static void reset_board(void)
{
	stepgen_reset();

	ENABLE_HI;	/* motors enable active low */
	COOLANT_LO;
	SPINEN_LO;
}

int main(void)
{
	unsigned long counter, i;
	int spi_timeout;

	initGPIO();
	initLED();
	initPIT();
	initDMA();
	initSPI();
	
	reset_board();
	spi_data_ready = 0;
	spi_timeout = 0;
	counter = 0;
	
	/* main loop */
	while (1) {
		
		/* process incoming data request,
		   transfer valid data to txBuf */
		if (!REQ_IN) {
			stepgen_get_position((void *)&txBuf[1]);

			/* read inputs */
			txBuf[1+MAXGEN] = read_inputs();

			RDY_LO;
		} else {
			RDY_HI;
		}

		/* process received data */
		if (spi_data_ready) {
			spi_data_ready = 0;

			/* reset spi_timeout */
			spi_timeout = 20000L;

			/* the first byte received is a command byte */
			switch (rxBuf[0]) {
			case 0x5453523E:	/* >RST */
				reset_board();
				break;
			case 0x444D433E:	/* >CMD */
				stepgen_update_input((const void *)&rxBuf[1]);
				update_outputs(rxBuf[1+MAXGEN]);
				break;
			case 0x4746433E:	/* >CFG */
				stepgen_update_stepwidth(rxBuf[1]);
				stepgen_reset();
				break;
			case 0x5453543E:	/* >TST */
				for (i=0; i<BUFSIZE; i++)
					txBuf[i] = rxBuf[i] ^ ~0;
				break;
			}
		}

		/* shutdown stepgen if no activity */
		if (spi_timeout)
			spi_timeout--;
		else
			reset_board();

		/* blink onboard led */
		if (!(counter++ % (spi_timeout ? 0xA000 : 0xF000))) {
			GREEN_TOGGLE;
		}
		
		/* reset watchdog sequence*/
		SIM_SRVCOP = SIM_SRVCOP_SRVCOP(0x55);
		SIM_SRVCOP = SIM_SRVCOP_SRVCOP(0xAA);
	}
	return 0;
}

__attribute__((interrupt("IRQ")))
void PIT_IRQHandler(void)
{
	PIT_TFLG1 |= PIT_TFLG_TIF_MASK;
	stepgen();
}

__attribute__((interrupt("IRQ")))
void DMA0_IRQHandler(void)
{

	/* disable SPI */
	SPI0_C1 &= ~SPI_C1_SPE_MASK;

	/* data integrity check */
	txBuf[0] = rxBuf[0] ^ ~0;

	/* reset src and dst pointers */
	DMA_DSR_BCR0 = DMA_DSR_BCR_DONE_MASK;
	DMA_DSR_BCR0 |= DMA_DSR_BCR_BCR(SPIBUFSIZE);
	DMA_DSR_BCR1 = DMA_DSR_BCR_DONE_MASK;
	DMA_DSR_BCR1 = DMA_DSR_BCR_BCR(SPIBUFSIZE);

	/* enable pheripheral request */
	DMA_DCR0 |= DMA_DCR_ERQ_MASK;
	DMA_DCR1 |= DMA_DCR_ERQ_MASK;

	/* enable SPI */
	SPI0_C1 |= SPI_C1_SPE_MASK;

	spi_data_ready = 1;
}