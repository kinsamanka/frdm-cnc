/*
 *    kinetis_sysinit.c - Default init routines for Flycatcher
 *                     		Kinetis ARM systems
 *    Copyright © 2012 Freescale semiConductor Inc. All Rights Reserved.
 */

//#include "kinetis_sysinit.h"
#include "MKL25Z4.h"

/**
 **===========================================================================
 **  External declarations
 **===========================================================================
 */
#if __cplusplus
extern "C" {
#endif
	extern uint32_t __vector_table[];
	extern unsigned long _estack;
	extern void __thumb_startup(void);
#if __cplusplus
}
#endif

/**
 **===========================================================================
 **  Default interrupt handler
 **===========================================================================
 */
void Default_Handler(void) {
	__asm("bkpt");
}

/**
 **===========================================================================
 **  Reset handler
 **===========================================================================
 */
void __init_hardware(void) {
	SCB_VTOR = (uint32_t)__vector_table; /* Set the interrupt vector table position */

	// Disable the Watchdog because it may reset the core before entering main().
	SIM_COPC = 0;

	/* System clock initialization */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;   /* Enable clock gate for ports to enable pin routing */
	SIM_CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(0x00) | SIM_CLKDIV1_OUTDIV4(0x01)); /* Update system prescalers */
	SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK; /* Select PLL as a clock source for various peripherals */
	SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(0x03); /* LPO 1kHz oscillator drives 32 kHz clock for various peripherals */
	SIM_SOPT2 = (SIM_SOPT2 & ~SIM_SOPT2_TPMSRC(0x02)) | SIM_SOPT2_TPMSRC(0x01);                      /* Set the TPM clock */
	PORTA_PCR18 &= ~(PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07));
	PORTA_PCR19 &= ~(PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07));
	MCG_SC &= ~MCG_SC_FCRDIV(0x07);

	/* Switch to FBE Mode */
	MCG_C2 = MCG_C2_RANGE0(0x02) | MCG_C2_EREFS0_MASK | MCG_C2_IRCS_MASK;
	OSC0_CR = OSC_CR_ERCLKEN_MASK;
	MCG_C1 = MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK;
	MCG_C4 &= ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03));
	MCG_C5 = MCG_C5_PRDIV0(0x03);
	MCG_C6 = MCG_C6_VDIV0(0x00);
	while ((MCG_S & MCG_S_IREFST_MASK) != 0x00U) {
		/* Check that the source of the FLL reference clock is the external reference clock. */
	}
	while ((MCG_S & 0x0CU) != 0x08U) {
		/* Wait until external reference clock is selected as MCG output */
	}

	/* Switch to PBE Mode */
	MCG_C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0x00);
	while ((MCG_S & 0x0CU) != 0x08U) {
		/* Wait until external reference clock is selected as MCG output */
	}
	while ((MCG_S & MCG_S_LOCK0_MASK) == 0x00U) {
		/* Wait until locked */
	}

	/* Switch to PEE Mode */
	MCG_C1 = MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x03) | MCG_C1_IRCLKEN_MASK;
	while ((MCG_S & 0x0CU) != 0x0CU) {
		/* Wait until output of the PLL is selected */
	}
}

/* Weak definitions of handlers point to Default_Handler if not implemented */
void NMI_Handler(void)		__attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)	__attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)		__attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)	__attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)	__attribute__((weak, alias("Default_Handler")));

void DMA0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void DMA1_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void DMA2_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void DMA3_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void MCM_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void FTFL_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void PMC_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void LLW_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void I2C0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void SPI0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void UART0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void UART1_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void UART2_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void ADC0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void CMP0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void FTM0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void FTM1_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void FTM2_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_Seconds_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PIT_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void USBOTG_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void DAC0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void TSI0_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void MCG_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void LPTimer_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void PORTA_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));
void PORTD_IRQHandler(void)	__attribute__((weak, alias("Default_Handler")));

/* The Interrupt Vector Table */
void (* const InterruptVector[])(void) __attribute__((section(".vectortable"))) = {
	/* Processor exceptions */
	(void( *)(void)) &_estack,
	__thumb_startup,
	NMI_Handler,
	HardFault_Handler,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	SVC_Handler,
	0,
	0,
	PendSV_Handler,
	SysTick_Handler,

	/* Interrupts */
	DMA0_IRQHandler, /* DMA Channel 0 Transfer Complete and Error */
	DMA1_IRQHandler, /* DMA Channel 1 Transfer Complete and Error */
	DMA2_IRQHandler, /* DMA Channel 2 Transfer Complete and Error */
	DMA3_IRQHandler, /* DMA Channel 3 Transfer Complete and Error */
	MCM_IRQHandler, /* Normal Interrupt */
	FTFL_IRQHandler, /* FTFL Interrupt */
	PMC_IRQHandler, /* PMC Interrupt */
	LLW_IRQHandler, /* Low Leakage Wake-up */
	I2C0_IRQHandler, /* I2C0 interrupt */
	I2C1_IRQHandler, /* I2C1 interrupt */
	SPI0_IRQHandler, /* SPI0 Interrupt */
	SPI1_IRQHandler, /* SPI1 Interrupt */
	UART0_IRQHandler, /* UART0 Status and Error interrupt */
	UART1_IRQHandler, /* UART1 Status and Error interrupt */
	UART2_IRQHandler, /* UART2 Status and Error interrupt */
	ADC0_IRQHandler, /* ADC0 interrupt */
	CMP0_IRQHandler, /* CMP0 interrupt */
	FTM0_IRQHandler, /* FTM0 fault, overflow and channels interrupt */
	FTM1_IRQHandler, /* FTM1 fault, overflow and channels interrupt */
	FTM2_IRQHandler, /* FTM2 fault, overflow and channels interrupt */
	RTC_Alarm_IRQHandler, /* RTC Alarm interrupt */
	RTC_Seconds_IRQHandler, /* RTC Seconds interrupt */
	PIT_IRQHandler, /* PIT timer all channels interrupt */
	Default_Handler, /* Reserved interrupt 39/23 */
	USBOTG_IRQHandler, /* USB interrupt */
	DAC0_IRQHandler, /* DAC0 interrupt */
	TSI0_IRQHandler, /* TSI0 Interrupt */
	MCG_IRQHandler, /* MCG Interrupt */
	LPTimer_IRQHandler, /* LPTimer interrupt */
	Default_Handler, /* Reserved interrupt 45/29 */
	PORTA_IRQHandler, /* Port A interrupt */
	PORTD_IRQHandler /* Port D interrupt */
};

/* Flash configuration field */
__attribute__ ((section (".cfmconfig"))) const uint8_t _cfm[0x10] = {
	/* NV_BACKKEY3: KEY=0xFF */
	0xFFU,
	/* NV_BACKKEY2: KEY=0xFF */
	0xFFU,
	/* NV_BACKKEY1: KEY=0xFF */
	0xFFU,
	/* NV_BACKKEY0: KEY=0xFF */
	0xFFU,
	/* NV_BACKKEY7: KEY=0xFF */
	0xFFU,
	/* NV_BACKKEY6: KEY=0xFF */
	0xFFU,
	/* NV_BACKKEY5: KEY=0xFF */
	0xFFU,
	/* NV_BACKKEY4: KEY=0xFF */
	0xFFU,
	/* NV_FPROT3: PROT=0xFF */
	0xFFU,
	/* NV_FPROT2: PROT=0xFF */
	0xFFU,
	/* NV_FPROT1: PROT=0xFF */
	0xFFU,
	/* NV_FPROT0: PROT=0xFF */
	0xFFU,
	/* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 */
	0x7EU,
	/* Disable NMI on boot */
	/* NV_FOPT: ??=1,??=1,FAST_INIT=1,LPBOOT1=1,RESET_PIN_CFG=1,NMI_DIS=0,??=1,LPBOOT0=1 */
	0xFBU,
	0xFFU,
	0xFFU
  };
