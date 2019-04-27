/****************************************************************************
 *   $Id:: LPC8xx.h 6437 2012-10-31 11:06:06Z dep00694                     $
 *   Project: NXP LPC8xx software example  
 *
 *   Description:
 *     CMSIS Cortex-M0+ Core Peripheral Access Layer Header File for 
 *     NXP LPC800 Device Series 
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.

 * Permission to use, copy, modify, and distribute this software and its 
 * documentation is hereby granted, under NXP Semiconductors' 
 * relevant copyright in the software, without fee, provided that it 
 * is used in conjunction with NXP Semiconductors microcontrollers. This 
 * copyright, permission, and disclaimer notice must appear in all copies of 
 * this code.
****************************************************************************/
#ifndef __LPC8xx_H__
#define __LPC8xx_H__

#ifdef __cplusplus
 extern "C" {
#endif 

/** @addtogroup LPC8xx_Definitions LPC8xx Definitions
  This file defines all structures and symbols for LPC8xx:
    - Registers and bitfields
    - peripheral base address
    - PIO definitions
  @{
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup LPC8xx_CMSIS LPC8xx CMSIS Definitions
  Configuration of the Cortex-M0+ Processor and Core Peripherals
  @{
*/

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    Reset_IRQn                    = -15,	/*!< 1 Reset Vector, invoked on Power up and warm reset */
	NonMaskableInt_IRQn           = -14,	/*!< 2 Non Maskable Interrupt                           */
	HardFault_IRQn                = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt                   */
	SVCall_IRQn                   = -5,		/*!< 11 Cortex-M0 SV Call Interrupt                     */
	PendSV_IRQn                   = -2,		/*!< 14 Cortex-M0 Pend SV Interrupt                     */
	SysTick_IRQn                  = -1,		/*!< 15 Cortex-M0 System Tick Interrupt                 */

/******  LPC8xx Specific Interrupt Numbers ********************************************************/
    SPI0_IRQn                     = 0,		/*!< SPI0                                             */
	SPI1_IRQn                     = 1,		/*!< SPI1                                             */
	Reserved0_IRQn                = 2,		/*!< Reserved Interrupt                               */
	UART0_IRQn                    = 3,		/*!< USART0                                           */
	UART1_IRQn                    = 4,		/*!< USART1                                           */
	UART2_IRQn                    = 5,		/*!< USART2                                           */
	Reserved1_IRQn                = 6,		/*!< Reserved Interrupt                               */
	I2C1_IRQn                     = 7,		/*!< I2C1                                             */
	I2C0_IRQn                     = 8,		/*!< I2C0                                             */
	SCT_IRQn                      = 9,		/*!< SCT                                              */
	MRT_IRQn                      = 10,		/*!< MRT                                              */
	CMP_IRQn                      = 11,		/*!< CMP                                              */
	WDT_IRQn                      = 12,		/*!< WDT                                              */
	BOD_IRQn                      = 13,		/*!< BOD                                              */
	FLASH_IRQn                    = 14,		/*!< Flash interrupt                                  */
	WKT_IRQn                      = 15,		/*!< WKT Interrupt                                    */
	ADC_SEQA_IRQn                 = 16,		/*!< ADC sequence A completion                        */
	ADC_SEQB_IRQn                 = 17,		/*!< ADC sequence B completion                        */
	ADC_THCMP_IRQn                = 18,		/*!< ADC threshold compare                            */
	ADC_OVR_IRQn                  = 19,		/*!< ADC overrun                                      */
	DMA_IRQn                      = 20,		/*!< Reserved Interrupt                               */
	I2C2_IRQn                     = 21,		/*!< Reserved Interrupt                               */
	I2C3_IRQn                     = 22,		/*!< Reserved Interrupt                               */
	Reserved2_IRQn                = 23,		/*!< Reserved Interrupt                               */
	PIN_INT0_IRQn                 = 24,		/*!< External Interrupt 0                             */
	PIN_INT1_IRQn                 = 25,		/*!< External Interrupt 1                             */
	PIN_INT2_IRQn                 = 26,		/*!< External Interrupt 2                             */
	PIN_INT3_IRQn                 = 27,		/*!< External Interrupt 3                             */
	PIN_INT4_IRQn                 = 28,		/*!< External Interrupt 4                             */
	PIN_INT5_IRQn                 = 29,		/*!< External Interrupt 5                             */
	PIN_INT6_IRQn                 = 30,		/*!< External Interrupt 6                             */
	PIN_INT7_IRQn                 = 31,		/*!< External Interrupt 7                             */
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0+ Processor and Core Peripherals */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */

/*@}*/ /* end of group LPC8xx_CMSIS */


#include "core_cm0plus.h"                  /* Cortex-M0+ processor and core peripherals          */
#include "system_LPC8xx.h"                 /* System Header                                      */


/******************************************************************************/
/*                Device Specific Peripheral Registers structures             */
/******************************************************************************/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*------------- System Control (SYSCON) --------------------------------------*/
/** @addtogroup LPC8xx_SYSCON LPC8xx System Control Block 
  @{
*/
typedef struct {
	__IO uint32_t SYSMEMREMAP;			/*!< Offset: 0x000 System memory remap (R/W) */
	__IO uint32_t PRESETCTRL;			/*!< Offset: 0x004 Peripheral reset control (R/W) */
	__IO uint32_t SYSPLLCTRL;			/*!< Offset: 0x008 System PLL control (R/W) */
	__IO uint32_t SYSPLLSTAT;			/*!< Offset: 0x00C System PLL status (R/W ) */
	uint32_t RESERVED0[4];
	__IO uint32_t SYSOSCCTRL;			/*!< Offset: 0x020 System oscillator control (R/W) */
	__IO uint32_t WDTOSCCTRL;			/*!< Offset: 0x024 Watchdog oscillator control (R/W) */
	__IO uint32_t IRCCTRL;              /*!< Offset: 0x028 IRC Control Register (Available only in LPC82X) */
	uint32_t RESERVED1[1];
	__IO uint32_t SYSRSTSTAT;			/*!< Offset: 0x030 System reset status Register (R/W ) */
	uint32_t RESERVED2[3];
	__IO uint32_t SYSPLLCLKSEL;			/*!< Offset: 0x040 System PLL clock source select (R/W) */
	__IO uint32_t SYSPLLCLKUEN;			/*!< Offset: 0x044 System PLL clock source update enable (R/W) */
	uint32_t RESERVED3[10];
	__IO uint32_t MAINCLKSEL;			/*!< Offset: 0x070 Main clock source select (R/W) */
	__IO uint32_t MAINCLKUEN;			/*!< Offset: 0x074 Main clock source update enable (R/W) */
	__IO uint32_t SYSAHBCLKDIV;			/*!< Offset: 0x078 System AHB clock divider (R/W) */
	uint32_t RESERVED4[1];
	__IO uint32_t SYSAHBCLKCTRL;		/*!< Offset: 0x080 System AHB clock control (R/W) */
	uint32_t RESERVED5[4];
	__IO uint32_t UARTCLKDIV;			/*!< Offset: 0x094 UART clock divider (R/W) */
	uint32_t RESERVED6[18];
	__IO uint32_t CLKOUTSEL;			/*!< Offset: 0x0E0 CLKOUT clock source select (R/W) */
	__IO uint32_t CLKOUTUEN;			/*!< Offset: 0x0E4 CLKOUT clock source update enable (R/W) */
	__IO uint32_t CLKOUTDIV;			/*!< Offset: 0x0E8 CLKOUT clock divider (R/W) */
	uint32_t RESERVED7;
	__IO uint32_t UARTFRGDIV;			/*!< Offset: 0x0F0 UART fractional divider SUB(R/W) */
	__IO uint32_t UARTFRGMULT;			/*!< Offset: 0x0F4 UART fractional divider ADD(R/W) */
	uint32_t RESERVED8[1];
	__IO uint32_t EXTTRACECMD;			/*!< Offset: 0x0FC External trace buffer command register  */
	__IO uint32_t PIOPORCAP0;			/*!< Offset: 0x100 POR captured PIO status 0 (R/ ) */
	uint32_t RESERVED9[12];
	__IO uint32_t IOCONCLKDIV[7];		/*!< Offset: 0x134 Peripheral clock x to the IOCON block for programmable glitch filter */
	__IO uint32_t BODCTRL;				/*!< Offset: 0x150 BOD control (R/W) */
	__IO uint32_t SYSTCKCAL;			/*!< Offset: 0x154 System tick counter calibration (R/W) */
	uint32_t RESERVED10[6];
	__IO uint32_t IRQLATENCY;			/*!< Offset: 0x170 IRQ delay */
	__IO uint32_t NMISRC;				/*!< Offset: 0x174 NMI Source Control     */
	__IO uint32_t PINTSEL[8];			/*!< Offset: 0x178 GPIO Pin Interrupt Select register 0 */
	uint32_t RESERVED11[27];
	__IO uint32_t STARTERP0;			/*!< Offset: 0x204 Start logic signal enable Register 0 (R/W) */
	uint32_t RESERVED12[3];
	__IO uint32_t STARTERP1;			/*!< Offset: 0x214 Start logic signal enable Register 0 (R/W) */
	uint32_t RESERVED13[6];
	__IO uint32_t PDSLEEPCFG;			/*!< Offset: 0x230 Power-down states in Deep-sleep mode (R/W) */
	__IO uint32_t PDAWAKECFG;			/*!< Offset: 0x234 Power-down states after wake-up (R/W) */
	__IO uint32_t PDRUNCFG;				/*!< Offset: 0x238 Power-down configuration Register (R/W) */
	uint32_t RESERVED14[111];
	__I  uint32_t DEVICEID;				/*!< Offset: 0x3F8 Device ID (R/ ) */
} LPC_SYSCTL_T;
/*@}*/ /* end of group LPC8xx_SYSCON */


/**
  * @brief Product name title=UM10462 Chapter title=LPC8xx I/O configuration Modification date=3/16/2011 Major revision=0 Minor revision=3  (IOCONFIG)
  */

typedef enum CHIP_PINx {
	IOCON_PIO0  =  0x11,	/*!< PIN 0 */
	IOCON_PIO1  =  0x0B,	/*!< PIN 1 */
	IOCON_PIO2  =  0x06,	/*!< PIN 2 */
	IOCON_PIO3  =  0x05,	/*!< PIN 3 */
	IOCON_PIO4  =  0x04,	/*!< PIN 4 */
	IOCON_PIO5  =  0x03,	/*!< PIN 5 */
	/* The following pins are not present in DIP8 packages */
	IOCON_PIO6  =  0x10,	/*!< PIN 6 */
	IOCON_PIO7  =  0x0F,	/*!< PIN 7 */
	IOCON_PIO8  =  0x0E,	/*!< PIN 8 */
	IOCON_PIO9  =  0x0D,	/*!< PIN 9 */
	IOCON_PIO10 =  0x08,	/*!< PIN 10 */
	IOCON_PIO11 =  0x07,	/*!< PIN 11 */
	IOCON_PIO12 =  0x02,	/*!< PIN 12 */
	IOCON_PIO13 =  0x01,	/*!< PIN 13 */
	/* The following pins are not present in DIP8 & TSSOP16 packages */
	IOCON_PIO14 =  0x12,	/*!< PIN 14 */
	IOCON_PIO15 =  0x0A,	/*!< PIN 15 */
	IOCON_PIO16 =  0x09,	/*!< PIN 16 */
	IOCON_PIO17 =  0x00,	/*!< PIN 17 */
	IOCON_PIO_NUL0 = 0x0C,	/*!< PIN NULL */

	/* The following pins are not present in DIP8, TSSOP16 & TSSOP20 packages */
	IOCON_PIO18 =  0x1E,	/*!< PIN 18 */
	IOCON_PIO19 =  0x1D,	/*!< PIN 19 */
	IOCON_PIO20 =  0x1C,	/*!< PIN 20 */
	IOCON_PIO21 =  0x1B,	/*!< PIN 21 */
	IOCON_PIO22 =  0x1A,	/*!< PIN 22 */
	IOCON_PIO23 =  0x19,	/*!< PIN 23 */
	IOCON_PIO24 =  0x18,	/*!< PIN 24 */
	IOCON_PIO25 =  0x17,	/*!< PIN 25 */
	IOCON_PIO26 =  0x16,	/*!< PIN 26 */
	IOCON_PIO27 =  0x15,	/*!< PIN 27 */
	IOCON_PIO28 =  0x14,	/*!< PIN 28 */
	IOCON_PIO_NUL1 = 0x13,	/*!< PIN NULL */
} CHIP_PINx_T;

#define NUM_IOCON_PIO  (29)

typedef struct {		/*!< (@ 0x40044000) IOCONFIG Structure     */
	__IO uint32_t PIO0[NUM_IOCON_PIO + 2]; /* 2 added for reserved register */
} LPC_IOCON_T;

/*@}*/ /* end of group LPC8xx_IOCON */

/**
 * @brief FLASH Memory Controller Unit register block structure
 */
typedef struct {
	__I  uint32_t  RESERVED1[4];
	__IO uint32_t  FLASHCFG;		/*!< Flash Configuration register */
	__I  uint32_t  RESERVED2[3];
	__IO uint32_t  FMSSTART;		/*!< Signature start address register */
	__IO uint32_t  FMSSTOP;			/*!< Signature stop address register */
	__I  uint32_t  RESERVED3;
	__I  uint32_t  FMSW[1];			/*!< Signature word regsiter */
} LPC_FMC_T;
/*@}*/ /* end of group LPC8xx_FLASHCTRL */


/*------------- Power Management Unit (PMU) --------------------------*/
/** @addtogroup LPC8xx_PMU LPC8xx Power Management Unit 
  @{
*/
typedef struct {
	__IO uint32_t PCON;		/*!< Offset: 0x000 Power control Register (R/W) */
	__IO uint32_t GPREG[4];	/*!< Offset: 0x004 General purpose Registers 0..3 (R/W) */
	__IO uint32_t DPDCTRL;	/*!< Offset: 0x014 Deep power-down control register (R/W) */
} LPC_PMU_T;
/*@}*/ /* end of group LPC8xx_PMU */


/*------------- Switch Matrix Port --------------------------*/
/** @addtogroup LPC8xx_SWM LPC8xx Switch Matrix Port 
  @{
*/
typedef struct {
#if defined(CHIP_LPC82X)
	__IO uint32_t PINASSIGN[12];	/*!< Pin Assign register array */
	__I  uint32_t RESERVED0[100];
#else
	__IO uint32_t PINASSIGN[9];		/*!< Pin Assign register array */
	__I  uint32_t RESERVED0[103];
#endif
	__IO uint32_t PINENABLE0;		/*!< Pin Enable register */
} LPC_SWM_T;
/*@}*/ /* end of group LPC8xx_SWM */


// ------------------------------------------------------------------------------------------------
// -----                                       GPIO_PORT                                      -----
// ------------------------------------------------------------------------------------------------

/**
  * @brief Product name title=UM10462 Chapter title=LPC8xx GPIO Modification date=3/17/2011 Major revision=0 Minor revision=3  (GPIO_PORT)
  */

typedef struct {				/*!< GPIO_PORT Structure */
	__IO uint8_t B[128][32];	/*!< Offset 0x0000: Byte pin registers ports 0 to n; pins PIOn_0 to PIOn_31 */
	__IO uint32_t W[32][32];	/*!< Offset 0x1000: Word pin registers port 0 to n */
	__IO uint32_t DIR[32];		/*!< Offset 0x2000: Direction registers port n */
	__IO uint32_t MASK[32];		/*!< Offset 0x2080: Mask register port n */
	__IO uint32_t PIN[32];		/*!< Offset 0x2100: Portpin register port n */
	__IO uint32_t MPIN[32];		/*!< Offset 0x2180: Masked port register port n */
	__IO uint32_t SET[32];		/*!< Offset 0x2200: Write: Set register for port n Read: output bits for port n */
	__O  uint32_t CLR[32];		/*!< Offset 0x2280: Clear port n */
	__O  uint32_t NOT[32];		/*!< Offset 0x2300: Toggle port n */
	__O  uint32_t DIRSET[32];   /*!< Offset 0x2380: Set Direction */
	__O  uint32_t DIRCLR[32];   /*!< Offset 0x2400: Clear Direction */
	__O  uint32_t DIRNOT[32];   /*!< Offset 0x2480: Toggle Dirction */
} LPC_GPIO_T;


// ------------------------------------------------------------------------------------------------
// -----                                     PIN_INT                                     -----
// ------------------------------------------------------------------------------------------------

/**
 * @brief LPC8xx Pin Interrupt and Pattern Match register block structure
 */
typedef struct {			/*!< (@ 0xA0004000) PIN_INT Structure */
	__IO uint32_t ISEL;		/*!< (@ 0xA0004000) Pin Interrupt Mode register */
	__IO uint32_t IENR;		/*!< (@ 0xA0004004) Pin Interrupt Enable (Rising) register */
	__IO uint32_t SIENR;	/*!< (@ 0xA0004008) Set Pin Interrupt Enable (Rising) register */
	__IO uint32_t CIENR;	/*!< (@ 0xA000400C) Clear Pin Interrupt Enable (Rising) register */
	__IO uint32_t IENF;		/*!< (@ 0xA0004010) Pin Interrupt Enable Falling Edge / Active Level register */
	__IO uint32_t SIENF;	/*!< (@ 0xA0004014) Set Pin Interrupt Enable Falling Edge / Active Level register */
	__IO uint32_t CIENF;	/*!< (@ 0xA0004018) Clear Pin Interrupt Enable Falling Edge / Active Level address */
	__IO uint32_t RISE;		/*!< (@ 0xA000401C) Pin Interrupt Rising Edge register */
	__IO uint32_t FALL;		/*!< (@ 0xA0004020) Pin Interrupt Falling Edge register */
	__IO uint32_t IST;		/*!< (@ 0xA0004024) Pin Interrupt Status register */
	__IO uint32_t PMCTRL;	/*!< (@ 0xA0004028) GPIO pattern match interrupt control register          */
	__IO uint32_t PMSRC;	/*!< (@ 0xA000402C) GPIO pattern match interrupt bit-slice source register */
	__IO uint32_t PMCFG;	/*!< (@ 0xA0004030) GPIO pattern match interrupt bit slice configuration register */
} LPC_PIN_INT_T;

/**
 * @brief CRC register block structure
 */
typedef struct {					/*!< CRC Structure */
	__IO    uint32_t    MODE;		/*!< CRC Mode Register */
	__IO    uint32_t    SEED;		/*!< CRC SEED Register */
	union {
		__I     uint32_t    SUM;	/*!< CRC Checksum Register. */
		__O     uint32_t    WRDATA32;	/*!< CRC Data Register: write size 32-bit*/
		__O     uint16_t    WRDATA16;	/*!< CRC Data Register: write size 16-bit*/
		__O     uint8_t     WRDATA8;	/*!< CRC Data Register: write size 8-bit*/
	};

} LPC_CRC_T;
/*@}*/ /* end of group LPC8xx_CRC */

/*------------- Comparator (CMP) --------------------------------------------------*/
/** @addtogroup LPC8xx_CMP LPC8xx Comparator
  @{
*/
typedef struct {			/*!< ACMP Structure */
	__IO uint32_t  CTRL;	/*!< Comparator control register */
	__IO uint32_t  LAD;		/*!< Voltage ladder register */
} LPC_CMP_T;
/*@}*/ /* end of group LPC8xx_CMP */


/*------------- Wakeup Timer (WKT) --------------------------------------------------*/
/** @addtogroup LPC8xx_WKT 
  @{
*/
typedef struct {
	__IO uint32_t  CTRL;	/*!< Offset: 0x000 Alarm/Wakeup Timer Control register */
	uint32_t  Reserved[2];
	__IO uint32_t  COUNT;	/*!< Offset: 0x000C Alarm/Wakeup Timer Counter register */
} LPC_WKT_T;
/*@}*/ /* end of group LPC8xx_WKT */


/*------------- Multi-Rate Timer(MRT) --------------------------------------------------*/
typedef struct {
	__IO uint32_t INTVAL;	/*!< Timer interval register */
	__O  uint32_t TIMER;	/*!< Timer register */
	__IO uint32_t CTRL;		/*!< Timer control register */
	__IO uint32_t STAT;		/*!< Timer status register */
} LPC_MRT_CH_T;

typedef struct {
	LPC_MRT_CH_T CHANNEL[4];
	uint32_t unused[45];
	__O  uint32_t IDLE_CH;
	__IO uint32_t IRQ_FLAG;
} LPC_MRT_T;


/*------------- Universal Asynchronous Receiver Transmitter (USART) -----------*/
/** @addtogroup LPC8xx_UART LPC8xx Universal Asynchronous Receiver/Transmitter 
  @{
*/
/**
  * @brief Product name title=LPC8xx MCU Chapter title=USART Modification date=4/18/2012 Major revision=0 Minor revision=9  (USART)
  */
typedef struct {
	__IO uint32_t  CFG;				/*!< Configuration register */
	__IO uint32_t  CTRL;			/*!< Control register */
	__IO uint32_t  STAT;			/*!< Status register */
	__IO uint32_t  INTENSET;		/*!< Interrupt Enable read and set register */
	__O  uint32_t  INTENCLR;		/*!< Interrupt Enable clear register */
	__I  uint32_t  RXDATA;			/*!< Receive Data register */
	__I  uint32_t  RXDATA_STAT;		/*!< Receive Data with status register */
	__IO uint32_t  TXDATA;			/*!< Transmit data register */
	__IO uint32_t  BRG;				/*!< Baud Rate Generator register */
	__IO uint32_t  INTSTAT;			/*!< Interrupt status register */
	__IO uint32_t  OSR;             /*!< Oversampling Selection regiser */
	__IO uint32_t  ADDR;            /*!< Address register for automatic address matching */
} LPC_USART_T;

/*@}*/ /* end of group LPC8xx_USART */


/*------------- Synchronous Serial Interface Controller (SPI) -----------------------*/
/** @addtogroup LPC8xx_SPI LPC8xx Synchronous Serial Port 
  @{
*/
typedef struct {					/*!< SPI Structure */
	__IO uint32_t  CFG;				/*!< SPI Configuration register */
	__IO uint32_t  DLY;				/*!< SPI Delay register */
	__IO uint32_t  STAT;			/*!< SPI Status register */
	__IO uint32_t  INTENSET;		/*!< SPI Interrupt Enable Set register */
	__O  uint32_t  INTENCLR;		/*!< SPI Interrupt Enable Clear register */
	__I  uint32_t  RXDAT;			/*!< SPI Receive Data register */
	__IO uint32_t  TXDATCTL;		/*!< SPI Transmit Data with Control register */
	__IO uint32_t  TXDAT;			/*!< SPI Transmit Data register */
	__IO uint32_t  TXCTRL;			/*!< SPI Transmit Control register */
	__IO uint32_t  DIV;				/*!< SPI clock Divider register */
	__I  uint32_t  INTSTAT;			/*!< SPI Interrupt Status register */
} LPC_SPI_T;
/*@}*/ /* end of group LPC8xx_SPI */


/*------------- Inter-Integrated Circuit (I2C) -------------------------------*/
/** @addtogroup LPC8xx_I2C I2C-Bus Interface 
  @{
*/
typedef struct {					/* I2C0 Structure         */
	__IO uint32_t CFG;			/*!< I2C Configuration Register common for Master, Slave and Monitor */
	__IO uint32_t STAT;			/*!< I2C Status Register common for Master, Slave and Monitor */
	__IO uint32_t INTENSET;	/*!< I2C Interrupt Enable Set Register common for Master, Slave and Monitor */
	__O  uint32_t INTENCLR;	/*!< I2C Interrupt Enable Clear Register common for Master, Slave and Monitor */
	__IO uint32_t TIMEOUT;	/*!< I2C Timeout value Register */
	__IO uint32_t CLKDIV;		/*!< I2C Clock Divider Register */
	__I  uint32_t INTSTAT;	/*!< I2C Interrupt Status Register */
	__I  uint32_t RESERVED0;
	__IO uint32_t MSTCTL;		/*!< I2C Master Control Register */
	__IO uint32_t MSTTIME;	/*!< I2C Master Time Register for SCL */
	__IO uint32_t MSTDAT;		/*!< I2C Master Data Register */
	__I  uint32_t RESERVED1[5];
	__IO uint32_t SLVCTL;		/*!< I2C Slave Control Register */
	__IO uint32_t SLVDAT;		/*!< I2C Slave Data Register */
	__IO uint32_t SLVADR[4];	/*!< I2C Slave Address Registers */
	__IO uint32_t SLVQUAL0;	/*!< I2C Slave Address Qualifier 0 Register */
	__I  uint32_t RESERVED2[9];
	__I  uint32_t MONRXDAT;	/*!< I2C Monitor Data Register */
} LPC_I2C_T;

/*@}*/ /* end of group LPC8xx_I2C */

/**
  * @brief State Configurable Timer (SCT) (SCT)
  */

/**
  * @brief Product name title=UM10430 Chapter title=LPC8xx State Configurable Timer (SCT) Modification date=1/18/2011 Major revision=0 Minor revision=7  (SCT)
  */

#define CONFIG_SCT_nEV   (16)			/*!< Number of events */
#define CONFIG_SCT_nRG   (16)			/*!< Number of match/compare registers */
#define CONFIG_SCT_nOU   (16)			/*!< Number of outputs */

typedef struct {
	__IO  uint32_t CONFIG;				/*!< Configuration Register */
	union {
		__IO uint32_t CTRL_U;			/*!< Control Register */
		struct {
			__IO uint16_t CTRL_L;		/*!< Low control register */
			__IO uint16_t CTRL_H;		/*!< High control register */
		};
	};
	union {
		__IO uint32_t LIMIT;			/*!< limit Register */
		struct {
			__IO uint16_t LIMIT_L;		/*!< limit register for counter L */
			__IO uint16_t LIMIT_H;		/*!< limit register for counter H */
		};

	};

	union {
		__IO uint32_t HALT;				/*!< Halt Register */
		struct {
			__IO uint16_t HALT_L;		/*!< halt register for counter L */
			__IO uint16_t HALT_H;		/*!< halt register for counter H */
		};

	};

	union {
		__IO uint32_t STOP;				/*!< Stop Register */
		struct {
			__IO uint16_t STOP_L;		/*!< stop register for counter L */
			__IO uint16_t STOP_H;		/*!< stop register for counter H */
		};

	};

	union {
		__IO uint32_t START;			/*!< start Register */
		struct {
			__IO uint16_t START_L;		/*!< start register for counter L */
			__IO uint16_t START_H;		/*!< start register for counter H */
		};

	};

	uint32_t RESERVED1[10];				/*!< 0x03C reserved */
	union {
		__IO uint32_t COUNT_U;			/*!< counter register */
		struct {
			__IO uint16_t COUNT_L;		/*!< counter register for counter L */
			__IO uint16_t COUNT_H;		/*!< counter register for counter H */
		};

	};

	union {
		__IO uint32_t STATE;			/*!< State register */
		struct {
			__IO uint16_t STATE_L;		/*!< state register for counter L */
			__IO uint16_t STATE_H;		/*!< state register for counter H */
		};

	};

	__I  uint32_t INPUT;				/*!< input register */
	union {
		__IO uint32_t REGMODE;			/*!< RegMode register */
		struct {
			__IO uint16_t REGMODE_L;	/*!< match - capture registers mode register L */
			__IO uint16_t REGMODE_H;	/*!< match - capture registers mode register H */
		};

	};

	__IO uint32_t OUTPUT;				/*!< output register */
	__IO uint32_t OUTPUTDIRCTRL;		/*!< output counter direction Control Register */
	__IO uint32_t RES;					/*!< conflict resolution register */
	__IO uint32_t DMA0REQUEST;			/*!< DMA0 Request Register */
	__IO uint32_t DMA1REQUEST;			/*!< DMA1 Request Register */
	uint32_t RESERVED2[35];
	__IO uint32_t EVEN;					/*!< event enable register */
	__IO uint32_t EVFLAG;				/*!< event flag register */
	__IO uint32_t CONEN;				/*!< conflict enable register */
	__IO uint32_t CONFLAG;				/*!< conflict flag register */
	union {
		__IO union {					/*!< ... Match / Capture value */
			uint32_t U;					/*!<       SCTMATCH[i].U  Unified 32-bit register */
			struct {
				uint16_t L;				/*!<       SCTMATCH[i].L  Access to L value */
				uint16_t H;				/*!<       SCTMATCH[i].H  Access to H value */
			};

		} MATCH[CONFIG_SCT_nRG];

		__I union {
			uint32_t U;					/*!<       SCTCAP[i].U  Unified 32-bit register */
			struct {
				uint16_t L;				/*!<       SCTCAP[i].L  Access to L value */
				uint16_t H;				/*!<       SCTCAP[i].H  Access to H value */
			};

		} CAP[CONFIG_SCT_nRG];

	};

	uint32_t RESERVED3[32 - CONFIG_SCT_nRG];		/*!< ...-0x17C reserved */
	union {
		__IO uint16_t MATCH_L[CONFIG_SCT_nRG];		/*!< 0x180-... Match Value L counter */
		__I  uint16_t CAP_L[CONFIG_SCT_nRG];		/*!< 0x180-... Capture Value L counter */
	};

	uint16_t RESERVED4[32 - CONFIG_SCT_nRG];		/*!< ...-0x1BE reserved */
	union {
		__IO uint16_t MATCH_H[CONFIG_SCT_nRG];		/*!< 0x1C0-... Match Value H counter */
		__I  uint16_t CAP_H[CONFIG_SCT_nRG];		/*!< 0x1C0-... Capture Value H counter */
	};

	uint16_t RESERVED5[32 - CONFIG_SCT_nRG];		/*!< ...-0x1FE reserved */
	union {
		__IO union {					/*!< 0x200-... Match Reload / Capture Control value */
			uint32_t U;					/*!<       SCTMATCHREL[i].U  Unified 32-bit register */
			struct {
				uint16_t L;				/*!<       SCTMATCHREL[i].L  Access to L value */
				uint16_t H;				/*!<       SCTMATCHREL[i].H  Access to H value */
			};

		} MATCHREL[CONFIG_SCT_nRG];

		__IO union {
			uint32_t U;					/*!<       SCTCAPCTRL[i].U  Unified 32-bit register */
			struct {
				uint16_t L;				/*!<       SCTCAPCTRL[i].L  Access to L value */
				uint16_t H;				/*!<       SCTCAPCTRL[i].H  Access to H value */
			};

		} CAPCTRL[CONFIG_SCT_nRG];

	};

	uint32_t RESERVED6[32 - CONFIG_SCT_nRG];		/*!< ...-0x27C reserved */
	union {
		__IO uint16_t MATCHREL_L[CONFIG_SCT_nRG];	/*!< 0x280-... Match Reload value L counter */
		__IO uint16_t CAPCTRL_L[CONFIG_SCT_nRG];	/*!< 0x280-... Capture Control value L counter */
	};

	uint16_t RESERVED7[32 - CONFIG_SCT_nRG];		/*!< ...-0x2BE reserved */
	union {
		__IO uint16_t MATCHREL_H[CONFIG_SCT_nRG];	/*!< 0x2C0-... Match Reload value H counter */
		__IO uint16_t CAPCTRL_H[CONFIG_SCT_nRG];	/*!< 0x2C0-... Capture Control value H counter */
	};

	uint16_t RESERVED8[32 - CONFIG_SCT_nRG];		/*!< ...-0x2FE reserved */
	__IO struct {						/*!< 0x300-0x3FC  SCTEVENT[i].STATE / SCTEVENT[i].CTRL*/
		uint32_t STATE;					/*!< Event State Register */
		uint32_t CTRL;					/*!< Event Control Register */
	} EVENT[CONFIG_SCT_nEV];

	uint32_t RESERVED9[128 - 2 * CONFIG_SCT_nEV];	/*!< ...-0x4FC reserved */
	__IO struct {						/*!< 0x500-0x57C  SCTOUT[i].SET / SCTOUT[i].CLR */
		uint32_t SET;					/*!< Output n Set Register */
		uint32_t CLR;					/*!< Output n Clear Register */
	} OUT[CONFIG_SCT_nOU];

	uint32_t RESERVED10[191 - 2 * CONFIG_SCT_nOU];	/*!< ...-0x7F8 reserved */
	__I  uint32_t MODULECONTENT;		/*!< 0x7FC Module Content */
} LPC_SCT_T;
/*@}*/ /* end of group LPC8xx_SCT */


/*------------- Watchdog Timer (WWDT) -----------------------------------------*/
/** @addtogroup LPC8xx_WDT LPC8xx WatchDog Timer 
  @{
*/
typedef struct {				/*!< WWDT Structure         */
	__IO uint32_t  MOD;			/*!< Watchdog mode register. This register contains the basic mode and status of the Watchdog Timer. */
	__IO uint32_t  TC;			/*!< Watchdog timer constant register. This register determines the time-out value. */
	__O  uint32_t  FEED;		/*!< Watchdog feed sequence register. Writing 0xAA followed by 0x55 to this register reloads the Watchdog timer with the value contained in WDTC. */
	__I  uint32_t  TV;			/*!< Watchdog timer value register. This register reads out the current value of the Watchdog timer. */
	__I  uint32_t  RESERVED0;
	__IO uint32_t  WARNINT;		/*!< Watchdog warning interrupt register. This register contains the Watchdog warning interrupt compare value. */
	__IO uint32_t  WINDOW;		/*!< Watchdog timer window register. This register contains the Watchdog window value. */
} LPC_WWDT_T;
/*@}*/ /* end of group LPC8xx_WDT */


/**
 * @brief ADC register block structure
 */

typedef enum {
	ADC_SEQA_IDX,
	ADC_SEQB_IDX
} ADC_SEQ_IDX_T;

typedef struct {								/*!< ADCn Structure */
	__IO uint32_t CTRL;							/*!< A/D Control Register. The AD0CR register must be written to select the operating mode before A/D conversion can occur. */
	__I  uint32_t RESERVED0;
	__IO uint32_t SEQ_CTRL[ADC_SEQB_IDX + 1];	/*!< A/D Sequence A & B Control Register. Controls triggering and channel selection for sonversion sequence. */
	__IO uint32_t SEQ_GDAT[ADC_SEQB_IDX + 1];	/*!< A/D Sequence A & B Global Data Register. Contains the result of the most recent A/D conversion for sequence. */
	__I  uint32_t RESERVED1[2];
	__I  uint32_t DR[12];						/*!< A/D Channel Data Register. This register contains the result of the most recent conversion completed on channel n. */
	__IO uint32_t THR_LOW[2];					/*!< A/D Low Compare Threshold Register 0 & 1. Contains the lower threshold level for automatic threshold comparison. */
	__IO uint32_t THR_HIGH[2];					/*!< A/D High Compare Threshold Register 0 & 1. Contains the higher threshold level for automatic threshold comparison. */
	__IO uint32_t CHAN_THRSEL;					/*!< A/D Channel Threshold Select Register. Specifies which set of threshold compare registers to use. */
	__IO uint32_t INTEN;						/*!< A/D Interrupt Enable Register. This register contains enable bits that enable sequence-A, sequence-B, threshold compare and overrun interrupts. */
	__IO uint32_t FLAGS;						/*!< A/D Flags Register. This register contains interrupt flags. - To be checked */
	__IO uint32_t TRM;							/*!< A/D Trim Register. */
} LPC_ADC_T;


/**
 * @brief DMA Controller shared registers structure
 */

typedef struct {					/*!< DMA shared registers structure */
	__IO uint32_t  ENABLESET;		/*!< DMA Channel Enable read and Set for all DMA channels */
	__I  uint32_t  RESERVED0;
	__O  uint32_t  ENABLECLR;		/*!< DMA Channel Enable Clear for all DMA channels */
	__I  uint32_t  RESERVED1;
	__I  uint32_t  ACTIVE;			/*!< DMA Channel Active status for all DMA channels */
	__I  uint32_t  RESERVED2;
	__I  uint32_t  BUSY;			/*!< DMA Channel Busy status for all DMA channels */
	__I  uint32_t  RESERVED3;
	__IO uint32_t  ERRINT;			/*!< DMA Error Interrupt status for all DMA channels */
	__I  uint32_t  RESERVED4;
	__IO uint32_t  INTENSET;		/*!< DMA Interrupt Enable read and Set for all DMA channels */
	__I  uint32_t  RESERVED5;
	__O  uint32_t  INTENCLR;		/*!< DMA Interrupt Enable Clear for all DMA channels */
	__I  uint32_t  RESERVED6;
	__IO uint32_t  INTA;			/*!< DMA Interrupt A status for all DMA channels */
	__I  uint32_t  RESERVED7;
	__IO uint32_t  INTB;			/*!< DMA Interrupt B status for all DMA channels */
	__I  uint32_t  RESERVED8;
	__O  uint32_t  SETVALID;		/*!< DMA Set ValidPending control bits for all DMA channels */
	__I  uint32_t  RESERVED9;
	__O  uint32_t  SETTRIG;			/*!< DMA Set Trigger control bits for all DMA channels */
	__I  uint32_t  RESERVED10;
	__O  uint32_t  ABORT;			/*!< DMA Channel Abort control for all DMA channels */
} LPC_DMA_COMMON_T;

/**
 * @brief DMA Controller shared registers structure
 */
typedef struct {					/*!< DMA channel register structure */
	__IO uint32_t  CFG;				/*!< DMA Configuration register */
	__I  uint32_t  CTLSTAT;			/*!< DMA Control and status register */
	__IO uint32_t  XFERCFG;			/*!< DMA Transfer configuration register */
	__I  uint32_t  RESERVED;
} LPC_DMA_CHANNEL_T;

/* DMA channel mapping - each channel is mapped to an individual peripheral
   and direction or a DMA imput mux trigger */
typedef enum {
	DMAREQ_USART0_RX,					/*!< USART0 receive DMA channel */
	DMA_CH0 = DMAREQ_USART0_RX,
	DMAREQ_USART0_TX,					/*!< USART0 transmit DMA channel */
	DMA_CH1 = DMAREQ_USART0_TX,
	DMAREQ_USART1_RX,					/*!< USART1 receive DMA channel */
	DMA_CH2 = DMAREQ_USART1_RX,
	DMAREQ_USART1_TX,					/*!< USART1 transmit DMA channel */
	DMA_CH3 = DMAREQ_USART1_TX,
	DMAREQ_USART2_RX,					/*!< USART2 receive DMA channel */
	DMA_CH4 = DMAREQ_USART2_RX,
	DMAREQ_USART2_TX,					/*!< USART2 transmit DMA channel */
	DMA_CH5 = DMAREQ_USART2_TX,
	DMAREQ_SPI0_RX,
	DMA_CH6 = DMAREQ_SPI0_RX,           /*!< SPI0 receive DMA channel */
	DMAREQ_SPI0_TX,
	DMA_CH7 = DMAREQ_SPI0_TX,           /*!< SPI0 transmit DMA channel */
	DMAREQ_SPI1_RX,
	DMA_CH8 = DMAREQ_SPI1_RX,           /*!< SPI1 receive DMA channel */
	DMAREQ_SPI1_TX,
	DMA_CH9 = DMAREQ_SPI1_TX,           /*!< SPI1 transmit DMA channel */
	DMAREQ_I2C0_MST,
	DMA_CH10 = DMAREQ_I2C0_MST,         /*!< I2C0 Master DMA channel */
	DMAREQ_I2C0_SLV,
	DMA_CH11 = DMAREQ_I2C0_SLV,         /*!< I2C0 Slave DMA channel */
	DMAREQ_I2C1_MST,
	DMA_CH12 = DMAREQ_I2C1_MST,         /*!< I2C1 Master DMA channel */
	DMAREQ_I2C1_SLV,
	DMA_CH13 = DMAREQ_I2C1_SLV,         /*!< I2C1 Slave DMA channel */
	DMAREQ_I2C2_MST,
	DMA_CH14 = DMAREQ_I2C2_MST,         /*!< I2C2 Master DMA channel */
	DMAREQ_I2C2_SLV,
	DMA_CH15 = DMAREQ_I2C2_SLV,         /*!< I2C2 Slave DMA channel */
	DMAREQ_I2C3_MST,
	DMA_CH16 = DMAREQ_I2C3_MST,         /*!< I2C2 Master DMA channel */
	DMAREQ_I2C3_SLV,
	DMA_CH17 = DMAREQ_I2C3_SLV,         /*!< I2C2 Slave DMA channel */
} DMA_CHID_T;

#define MAX_DMA_CHANNEL			(DMA_CH17 + 1)

/**
 * @brief DMA Controller register block structure
 */
typedef struct {					/*!< DMA Structure */
	__IO uint32_t  CTRL;			/*!< DMA control register */
	__I  uint32_t  INTSTAT;			/*!< DMA Interrupt status register */
	__IO uint32_t  SRAMBASE;		/*!< DMA SRAM address of the channel configuration table */
	__I  uint32_t  RESERVED2[5];
	LPC_DMA_COMMON_T DMACOMMON[1];	/*!< DMA shared channel (common) registers */
	__I  uint32_t  RESERVED0[225];
	LPC_DMA_CHANNEL_T DMACH[MAX_DMA_CHANNEL];	/*!< DMA channel registers */
} LPC_DMA_T;


/**
 * @brief DMA trigger pin muxing structure
 */
typedef struct {					/*!< DMA trigger pin muxing register structure */
	__IO uint32_t  DMA_ITRIG_INMUX[MAX_DMA_CHANNEL];	/*!< Trigger input select register for DMA channels */
} LPC_DMATRIGMUX_T;



/** @defgroup INMUX_8XX CHIP: LPC8xx INPUT Mux Controller driver
 * @ingroup CHIP_8XX_Drivers
 * @{
 */
typedef struct {
	__IO uint32_t  DMA_INMUX_INMUX[2];    /*!< DMA Trigger Input 20 & 21 PINMUX 0-1 */
	__O  uint32_t  RESERVED[6];           /*!< Reserved; Should not be used */
	__IO uint32_t  SCT0_INMUX[4];         /*!< Input mux register for SCT0; INPUT0-3 */
} LPC_INMUX_T;


#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Base addresses */
#define LPC_FLASH_BASE        (0x00000000UL)
#define LPC_RAM_BASE          (0x10000000UL)
#define LPC_ROM_BASE          (0x1FFF0000UL)
#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_AHB_BASE          (0x50000000UL)

/* APB0 peripherals */
#define LPC_WWDT_BASE         (0x40000000UL)
#define LPC_MRT_BASE          (0x40004000UL)
#define LPC_WKT_BASE          (0x40008000UL)
#define LPC_SWM_BASE          (0x4000C000UL)
#define LPC_ADC_BASE          (0x4001C000UL)  /* Available only on LPC82x */
#define LPC_PMU_BASE          (0x40020000UL)
#define LPC_CMP_BASE          (0x40024000UL)
#define LPC_DMATIRGMUX_BASE   (0x40028000UL)  /* Available only on LPC82x */
#define LPC_INMUX_BASE        (0x4002C000UL)  /* Available only on LPC82x */

#define LPC_FMC_BASE          (0x40040000UL)
#define LPC_IOCON_BASE        (0x40044000UL)
#define LPC_SYSCTL_BASE       (0x40048000UL)
#define LPC_I2C0_BASE         (0x40050000UL)
#define LPC_I2C1_BASE         (0x40054000UL)  /* Available only on LPC82x */
#define LPC_SPI0_BASE         (0x40058000UL)
#define LPC_SPI1_BASE         (0x4005C000UL)
#define LPC_USART0_BASE       (0x40064000UL)
#define LPC_USART1_BASE       (0x40068000UL)
#define LPC_USART2_BASE       (0x4006C000UL)
#define LPC_I2C2_BASE         (0x40070000UL)  /* Available only on LPC82x */
#define LPC_I2C3_BASE         (0x40074000UL)  /* Available only on LPC82x */

/* AHB peripherals */
#define LPC_CRC_BASE          (0x50000000UL)
#define LPC_SCT_BASE          (0x50004000UL)
#define LPC_DMA_BASE          (0x50008000UL)  /* Available only on LPC82x */

#define LPC_GPIO_PORT_BASE    (0xA0000000UL)
#define LPC_PIN_INT_BASE      (0xA0004000UL)

#define LPC_WWDT            ((LPC_WWDT_T     *) LPC_WWDT_BASE)
#define LPC_SPI0            ((LPC_SPI_T      *) LPC_SPI0_BASE)
#define LPC_SPI1            ((LPC_SPI_T      *) LPC_SPI1_BASE)
#define LPC_USART0          ((LPC_USART_T    *) LPC_USART0_BASE)
#define LPC_USART1          ((LPC_USART_T    *) LPC_USART1_BASE)
#define LPC_USART2          ((LPC_USART_T    *) LPC_USART2_BASE)
#define LPC_WKT             ((LPC_WKT_T         *) LPC_WKT_BASE)
#define LPC_PMU             ((LPC_PMU_T         *) LPC_PMU_BASE)
#define LPC_CRC             ((LPC_CRC_T         *) LPC_CRC_BASE)
#define LPC_SCT             ((LPC_SCT_T         *) LPC_SCT_BASE)
#define LPC_GPIO_PORT       ((LPC_GPIO_T        *) LPC_GPIO_PORT_BASE)
#define LPC_PININT          ((LPC_PIN_INT_T     *) LPC_PIN_INT_BASE)
#define LPC_IOCON           ((LPC_IOCON_T       *) LPC_IOCON_BASE)
#define LPC_SWM             ((LPC_SWM_T         *) LPC_SWM_BASE)
#define LPC_SYSCTL          ((LPC_SYSCTL_T      *) LPC_SYSCTL_BASE)
#define LPC_CMP             ((LPC_CMP_T         *) LPC_CMP_BASE)
#define LPC_FMC             ((LPC_FMC_T         *) LPC_FMC_BASE)
#define LPC_MRT             ((LPC_MRT_T         *) LPC_MRT_BASE)
#define LPC_I2C0            ((LPC_I2C_T         *) LPC_I2C0_BASE)

/* Peripherals available only on LPC82x */
#define LPC_ADC             ((LPC_ADC_T         *) LPC_ADC_BASE)
#define LPC_I2C1            ((LPC_I2C_T         *) LPC_I2C1_BASE)
#define LPC_I2C2            ((LPC_I2C_T         *) LPC_I2C2_BASE)
#define LPC_I2C3            ((LPC_I2C_T         *) LPC_I2C3_BASE)
#define LPC_DMA             ((LPC_DMA_T         *) LPC_DMA_BASE)
#define LPC_DMATRIGMUX      ((LPC_DMATRIGMUX_T  *) LPC_DMATIRGMUX_BASE)
#define LPC_INMUX           ((LPC_INMUX_T       *) LPC_INMUX_BASE)

/* Base address Alias list */
#define LPC_I2C_BASE         LPC_I2C0_BASE
#define LPC_I2C              LPC_I2C0
#define LPC_SYSCON           LPC_SYSCTL

/* IRQ Handler alias list */
#define SCT0_IRQHandler      SCT_IRQHandler
#define CMP0_IRQHandler      CMP_IRQHandler
#define WWDT_IRQHandler      WDT_IRQHandler
#define I2C_IRQHandler       I2C0_IRQHandler

#ifdef __cplusplus
}
#endif

#endif  /* __LPC8xx_H__ */
