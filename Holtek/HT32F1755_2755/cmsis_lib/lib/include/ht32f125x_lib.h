/*********************************************************************************************************//**
 * @file    ht32f125x_lib.h
 * @version V1.0
 * @date    04/11/2011
 * @brief   The header file that includes all the header files of the libraries.
 *************************************************************************************************************
 *
 * <h2><center>Copyright (C) 2011 Holtek Semiconductor Inc. All rights reserved</center></h2>
 *
 ************************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------------------------------------*/
#ifndef __HT32F125x_LIB_H
#define __HT32F125x_LIB_H

/* Includes ------------------------------------------------------------------------------------------------*/
#include "ht32f125x_conf.h"
#include <stdio.h>

/* APB Peripherals                                                                                          */
#if _USART
  #include "ht32f125x_usart.h"
#endif

#if _SPI
  #include "ht32f125x_spi.h"
#endif

#if _ADC
  #include "ht32f125x_adc.h"
#endif

#if _CMP_OP
  #include "ht32f125x_cmp_op.h"
#endif

#if _GPIO
  #include "ht32f125x_gpio.h"
#endif

#if _EXTI
  #include "ht32f125x_exti.h"
#endif

#if _I2C
  #include "ht32f125x_i2c.h"
#endif

#if _WDT
  #include "ht32f125x_wdt.h"
#endif

#if _RTC
  #include "ht32f125x_rtc.h"
#endif

#if _PWRCU
  #include "ht32f125x_pwrcu.h"
#endif

#if _GPTM
  #include "ht32f125x_gptm.h"
#endif

/* AHB Peripherals                                                                                          */
#if _CKCU
  #include "ht32f125x_ckcu.h"
#endif

#if _RSTCU
  #include "ht32f125x_rstcu.h"
#endif

#if _FLASH
  #include "ht32f125x_flash.h"
#endif

/* Cortex-M3                                                                                                */
#if _MISC
  #include "ht32f125x_misc.h"
#endif

#if (_RETARGET)
  #if defined ( __GNUC__ )
    #undef getchar
    #define getchar SERIAL_GetChar
  #endif
  #include "ht32f125x_serial.h"
#endif

/* Exported types ------------------------------------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------------------------------------*/
void debug(void);

#endif /* __HT32F125x_LIB_H --------------------------------------------------------------------------------*/
