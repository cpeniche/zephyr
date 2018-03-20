/*
 * lpc176x.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Carlo Peniche
 */

#ifndef _LPC176X_H_
#define _LPC176X_H_

#include "core_cm3.h"            /* Cortex-M0 processor and core peripherals */
#include <stdint.h>

typedef struct
{
  __IO uint32_t pll0con;
  __IO uint32_t pll0cfg;
  __IO uint32_t pll0stat;
  __IO uint32_t pll0feed;
  __IO uint32_t pll1con;
  __IO uint32_t pll1cfg;
  __IO uint32_t pll1stat;
  __IO uint32_t pll1feed;
  __IO uint32_t rsv0[5];
  __IO uint32_t pcon;
  __IO uint32_t pconp;
  __IO uint32_t rsv1[16];
  __IO uint32_t cclkcfg;
  __IO uint32_t usbclkcfg;
  __IO uint32_t clksrcsel;
  __IO uint32_t rsv2[39];
  __IO uint32_t pclksel0;
  __IO uint32_t pclksel1;
  __IO uint32_t rsv3[7];
  __IO uint32_t clkoutcfg;
};

#define FLASH_BASE            ((uint32_t)0x00000000U)              /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END       ((uint32_t)0x00007FFFU)              /*!< FLASH END address of bank1 */
#define SRAM_BASE             ((uint32_t)0x10000000U)              /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000U)              /*!< Peripheral base address in the alias region */


/*!< Peripheral memory map */
#define APB0PERIPH_BASE        PERIPH_BASE
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x00080000U)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x10000000U)


/*!< APB0 peripherals */
#define WDT_BASE              (APB0PERIPH_BASE + 0x00000000)
#define TIM0_BASE             (APB0PERIPH_BASE + 0x00004000)
#define TIM1_BASE             (APB0PERIPH_BASE + 0x00008000)
#define UART0_BASE            (APB0PERIPH_BASE + 0x0000C000)
#define UART1_BASE            (APB0PERIPH_BASE + 0x00010000)
#define PWM1_BASE             (APB0PERIPH_BASE + 0x00018000)
#define I2C0_BASE             (APB0PERIPH_BASE + 0x0001C000)
#define SPI_BASE              (APB0PERIPH_BASE + 0x00020000)
#define RTC_BASE              (APB0PERIPH_BASE + 0x00024000)
#define GPIO_INT_BASE         (APB0PERIPH_BASE + 0x00028000)
#define PIN_CNT_BASE          (APB0PERIPH_BASE + 0x0002C000)
#define SSP1_BASE             (APB0PERIPH_BASE + 0x00030000)
#define ADC_BASE              (APB0PERIPH_BASE + 0x00034000)
#define CAN_AF_RAM_BASE       (APB0PERIPH_BASE + 0x00038000)
#define CAN_AF_BASE           (APB0PERIPH_BASE + 0x0003C000)
#define CAN_CMON_BASE         (APB0PERIPH_BASE + 0x00040000)
#define CAN1_BASE             (APB0PERIPH_BASE + 0x00044000)
#define CAN2_BASE             (APB0PERIPH_BASE + 0x00048000)
#define I2C1_BASE             (APB0PERIPH_BASE + 0x0005C000)


/*!< APB1 peripherals */

#define SSP0_BASE             (APB1PERIPH_BASE + 0x00008000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x0000C000)
#define TIM2_BASE             (APB1PERIPH_BASE + 0x00010000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x00014000)
#define UART2_BASE            (APB1PERIPH_BASE + 0x00018000)
#define UART3_BASE            (APB1PERIPH_BASE + 0x0001C000)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x00020000)
#define I2S_INT_BASE          (APB1PERIPH_BASE + 0x00028000)
#define RPT_INT_TMR_BASE      (APB1PERIPH_BASE + 0x00030000)
#define MTR_CNTRL_BASE        (APB1PERIPH_BASE + 0x00038000)
#define QEI_BASE              (APB1PERIPH_BASE + 0x0003C000)
#define SYS_CNTRL_BASE        (APB1PERIPH_BASE + 0x0007C000)





/*!< AHB peripherals */
#define ETH_BASE              (AHBPERIPH_BASE + 0x00000000)
#define DMA_BASE              (AHBPERIPH_BASE + 0x00004000)
#define USB_BASE              (AHBPERIPH_BASE + 0x0000C000)


#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000)
#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x00002000) /*!< FLASH registers base address */
#define OB_BASE               ((uint32_t)0x1FFFF800U)       /*!< FLASH Option Bytes base address */
#define FLASHSIZE_BASE        ((uint32_t)0x1FFFF7CCU)       /*!< FLASH Size register base address */
#define UID_BASE              ((uint32_t)0x1FFFF7ACU)       /*!< Unique device ID register base address */
#define CRC_BASE              (AHBPERIPH_BASE + 0x00003000)

/*!< AHB2 peripherals */
#define GPIOA_BASE            (AHB2PERIPH_BASE + 0x00000000)
#define GPIOB_BASE            (AHB2PERIPH_BASE + 0x00000400)
#define GPIOC_BASE            (AHB2PERIPH_BASE + 0x00000800)
#define GPIOF_BASE            (AHB2PERIPH_BASE + 0x00001400)


#endif /* _LPC176X_H_ */
