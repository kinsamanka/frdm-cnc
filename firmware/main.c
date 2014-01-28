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

static inline void enable_irq(int n) {
	NVIC_ICPR |= 1 << ((n - 16)%32);
	NVIC_ISER |= 1 << ((n - 16)%32);
}

static inline void disable_irq(int n) {
	NVIC_ICER = 1 << ((n - 16)%32);
}

static void initLED(void) {
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

static void initPIT(void) {
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

static void initDMA(void) {
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

static void initSPI(void) {
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

static void initGPIO(void) {
	/* enable port A - E clock */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
	        SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK |
	        SIM_SCGC5_PORTE_MASK;

	/* inputs */
	PORTA_PCR4 = PORT_PCR_MUX(1);	/* y min */
	PORTA_PCR5 = PORT_PCR_MUX(1);	/* y max */
	PORTA_PCR12 = PORT_PCR_MUX(1);	/* x min */
	PORTC_PCR3 = PORT_PCR_MUX(1);	/* data request */
	PORTC_PCR10 = PORT_PCR_MUX(1);	/* z max */
	PORTC_PCR11 = PORT_PCR_MUX(1);	/* z min */
	PORTD_PCR4 = PORT_PCR_MUX(1);	/* x max */

	/* Set the pins direction to input */
	GPIOA_PDDR &= ~((1<<4)|(1<<5)|(1<<12));
	GPIOC_PDDR &= ~((1<<3)|(1<<10)|(1<<11));
	GPIOD_PDDR &= ~((1<<4));

	/* outputs */
	PORTB_PCR0 = PORT_PCR_MUX(1);	/* step x */
	PORTB_PCR1 = PORT_PCR_MUX(1);	/* dir x */
	PORTB_PCR2 = PORT_PCR_MUX(1);	/* enable y */
	PORTB_PCR9 = PORT_PCR_MUX(1);	/* enable e0*/
	PORTB_PCR10 = PORT_PCR_MUX(1);	/* step e0*/
	PORTB_PCR11 = PORT_PCR_MUX(1);	/* dir e0 */

	PORTC_PCR0 = PORT_PCR_MUX(1);	/* data ready */

	PORTD_PCR1 = PORT_PCR_MUX(1);	/* led */
	PORTD_PCR3 = PORT_PCR_MUX(1);	/* ps_on */

	PORTE_PCR2 = PORT_PCR_MUX(1);	/* enable e1 */
	PORTE_PCR3 = PORT_PCR_MUX(1);	/* dir e1 */
	PORTE_PCR4 = PORT_PCR_MUX(1);	/* step e1 */
	PORTE_PCR5 = PORT_PCR_MUX(1);	/* enable x */
	PORTE_PCR20 = PORT_PCR_MUX(1);	/* step z */
	PORTE_PCR21 = PORT_PCR_MUX(1);	/* dir z */
	PORTE_PCR22 = PORT_PCR_MUX(1);	/* step y */
	PORTE_PCR23 = PORT_PCR_MUX(1);	/* dir y */
	PORTE_PCR29 = PORT_PCR_MUX(1);	/* enable z */

	/* Set the initial output state to low */
	GPIOB_PCOR = (1<<0)|(1<<1)|(1<<10)|(1<<11);
	GPIOD_PCOR = (1<<1)|(1<<3);
	GPIOE_PCOR = (1<<3)|(1<<4)|(1<<20)|(1<<21)|(1<<22)|(1<<23);

	/* enable lines and data ready active low */
	GPIOB_PSOR = (1<<2)|(1<<9);
	GPIOC_PSOR = (1<<0);
	GPIOE_PSOR = (1<<2)|(1<<5)|(1<<29);

	/* Set the pins direction to output */
	GPIOB_PDDR |= (1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<10)|(1<<11);
	GPIOC_PDDR |= (1<<0);
	GPIOD_PDDR |= (1<<1)|(1<<3);
	GPIOE_PDDR |= (1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<20)|
		(1<<21)|(1<<22)|(1<<23)|(1<<29);
}

static void initADC(void) {
	uint32_t x;
	
	/* enable ADC clock */
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;
	/* enable port B & C clock */
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK;

	/* set ports as analog */
	PORTB_PCR3 = PORT_PCR_MUX(0);	/* ADC0_SE13 - SC1n[ADCH] = 01101 */
	PORTC_PCR2 = PORT_PCR_MUX(0);	/* ADC0_SE11 - SC1n[ADCH] = 01011 */
	PORTC_PCR1 = PORT_PCR_MUX(0);	/* ADC0_SE15 - SC1n[ADCH] = 01111 */

	/* 16 bits, long sample time, using asynchronous clock */
	ADC0_CFG1 = 0x1F;

	/* enable asynchronous clock output and slowest adc sampling */
	ADC0_CFG2 = 0x08;
	
	/* software triggered and default voltage reference pins */
	ADC0_SC2 = 0x00;
	
	/* single conversion, 32 samples averaged */
	ADC0_SC3 = 0x07;

	/* start adc calibration */
	ADC0_SC3 |= 1<<7;
	
	/* wait until done */
	while (ADC0_SC3 & 1<<7);
	
	/* check if failed */
	if (ADC0_SC3 & 1<<6) {
		RED_ON;
	} else {
	
		x = ADC0_CLPS + ADC0_CLP0 + ADC0_CLP1 + ADC0_CLP2 + ADC0_CLP3 + ADC0_CLP4;
		x = (x >> 1) | 0x8000;
		ADC0_PG = x;
		
		x = ADC0_CLMS + ADC0_CLM0 + ADC0_CLM1 + ADC0_CLM2 + ADC0_CLM3 + ADC0_CLM4;
		x = (x >> 1) | 0x8000;
		ADC0_MG = x;
	}
}

static void initPWM(void) {
	/* TPM clock at 8 MHz */
	
	/* enable TPM0 and TPM1 clock */
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK;                                   
	/* enable port A & D clock */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTD_MASK;
	
	/* setup TPM0 */
	TPM0_SC = 0;
	TPM0_CNT = 0;
	TPM0_C0SC = 0;
	TPM0_C5SC = 0;
	/* Set up modulo register */
	TPM0_MOD = TPM_MOD_MOD(0x3E80);
	/* Edge-aligned PWM */
	TPM0_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	TPM0_C0V = 0;
	/* Edge-aligned PWM */
	TPM0_C5SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
	TPM0_C5V = 0;

	/* enable output pins */
	PORTD_PCR0 = PORT_PCR_MUX(0x04);
	PORTD_PCR5 = PORT_PCR_MUX(0x04);
	
	/* use TPM clock prescale by 1 */
	TPM0_SC = (TPM_SC_CMOD(0x01) | TPM_SC_PS(0x00)); 
  
	/* setup TPM1 */
	TPM1_SC = 0;
	TPM1_CNT = 0;
	TPM1_C1SC = 0;
	/* Set up modulo register */
	TPM1_MOD = TPM_MOD_MOD(0x3E80);
	/* Edge-aligned PWM */
#if 0
	TPM1_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
#else
	TPM1_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_ELSA_MASK; /* active low */
#endif
	TPM1_C1V = 0;

	/* enable output pin */
	PORTA_PCR13 = PORT_PCR_MUX(0x03);

	/* use TPM clock prescale by 1 */
	TPM1_SC = (TPM_SC_CMOD(0x01) | TPM_SC_PS(0x00));  
}

static void initI2C(void) {
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;                                   
	I2C0_C1 = 0x00U;                     /* Clear control register */
	I2C0_FLT = (I2C_FLT_STOPF_MASK | I2C_FLT_FLT(0x00)); /* Clear bus status interrupt flags */
	I2C0_S = I2C_S_IICIF_MASK;           /* Clear interrupt flag */
	PORTC_PCR9 = PORT_PCR_MUX(0x02);
	PORTC_PCR8 = PORT_PCR_MUX(0x02);
	I2C0_C2 = I2C_C2_AD(0x00);                                   
	I2C0_FLT = I2C_FLT_FLT(0x00);        /* Set glitch filter register */
	I2C0_SMB = I2C_SMB_SLTF_MASK;                                   
	I2C0_F = (I2C_F_MULT(0x00) | I2C_F_ICR(0x00)); /* Set prescaler bits */
}

static uint32_t read_inputs(void) {
	uint32_t x;

	x  = (X_MIN_IN ? 1 : 0) << 0;
	x |= (X_MAX_IN ? 1 : 0) << 1;
	x |= (Y_MIN_IN ? 1 : 0) << 2;
	x |= (Y_MAX_IN ? 1 : 0) << 3;
	x |= (Z_MIN_IN ? 1 : 0) << 4;
	x |= (Z_MAX_IN ? 1 : 0) << 5;

	return x;
}

static void update_outputs(uint32_t x) {
	if (x & (1 << 0))
		EN_X_LO;
	else
		EN_X_HI;
	if (x & (1 << 1))
		EN_Y_LO;
	else
		EN_Y_HI;
	if (x & (1 << 2))
		EN_Z_LO;
	else
		EN_Z_HI;
	if (x & (1 << 3))
		EN_A_LO;
	else
		EN_A_HI;
	if (x & (1 << 4))
		EN_B_LO;
	else
		EN_B_HI;
}

static inline void update_pwm_period(uint32_t val) {
	TPM0_MOD = TPM_MOD_MOD(val);
	TPM1_MOD = TPM_MOD_MOD(val);
}

static inline void update_pwm_duty(uint32_t *val) {
	TPM0_C0V = *(val) >> 16 ;
	TPM0_C5V = *(val) & 0xFFFF;
	TPM1_C1V = *(val+1) >> 16;
}

static void reset_board(void) {
	stepgen_reset();

	/* axis enable active low */
	EN_X_HI;
	EN_Y_HI;
	EN_Z_HI;
	EN_A_HI;
	EN_B_HI;
	
	/* turn off pwm */
	TPM0_C0V = 0;
	TPM0_C5V = 0;
	TPM1_C1V = 0;
}

int main(void) {
	unsigned long counter, i, chan = 0;
	int spi_timeout;
	uint32_t adc[4] = { 0 };

	initGPIO();
	initLED();
	initPIT();
	initDMA();
	initSPI();
	initADC();
	initPWM();
	initI2C();
	
	reset_board();
	spi_data_ready = 0;
	spi_timeout = 0;
	counter = 0;
	
	/* main loop */
	while (1) {

		/* start adc conversion */
		if (!(ADC0_SC2 & (1 << 7))) {
			/* read results */
			adc[chan++] = ADC0_RA;
			/* start another conversion */
			switch (chan) {
			case 1:
				ADC0_SC1A = 0x0B;
				break;
			case 2:
				ADC0_SC1A = 0x0F;
				break;
			case 3:
				ADC0_SC1A = 0x0D;
				chan = 0;
				break;
			}
		}
		
		/* process incoming data request,
		   transfer valid data to txBuf */
		if (!REQ_IN) {
			stepgen_get_position((void *)&txBuf[1]);

			/* read inputs */
			txBuf[1+MAXGEN] = read_inputs();
			/* adc readings */
			txBuf[2+MAXGEN] = adc[0] << 16 | adc[1];
			txBuf[3+MAXGEN] = adc[2] << 16;

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
				update_pwm_duty((uint32_t *)&rxBuf[2+MAXGEN]);
				break;
			case 0x4746433E:	/* >CFG */
				stepgen_update_stepwidth(rxBuf[1]);
				update_pwm_period(rxBuf[2]);
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
			LED_TOGGLE;
		}
	}
	return 0;
}

__attribute__((interrupt("IRQ")))
void PIT_IRQHandler(void) {
	PIT_TFLG1 |= PIT_TFLG_TIF_MASK;
	stepgen();
}

__attribute__((interrupt("IRQ")))
void DMA0_IRQHandler(void) {

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