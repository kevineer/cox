/**************************************************************************//**
 * @file    ht32f175x_275x.h
 * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer Header File
 * @version V1.00
 * @date:   09/08/2011
 *
 * @note
 * Copyright (C) 2011 Holtek Semiconductor Inc. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) supplies this software for Cortex-M processor-based
 * microcontrollers.  This file can be freely distributed within
 * development tools that are supporting such ARM-based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup HT32F175x_275x
  * @{
  */

#ifndef __HT32F175x_275x_H__
#define __HT32F175x_275x_H__

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup Library_configuration_section
  * @{
  */

#define HSI_VALUE         8000000UL  /*!< Value of the Internal oscillator in Hz                            */

/**
 * @brief Adjust the value of External High Speed oscillator (HSE)

   Tip: To avoid from modifying every time for different HSE, please define
        the HSE value in your own toolchain compiler preprocessor.
  */
#if !defined  HSE_VALUE
  /* Avaiable HSE_VALUE: 4MHz ~ 16MHz                                                                       */
  #define HSE_VALUE       8000000UL   /*!< Value of the External oscillator in Hz                           */
#endif


/**
 * @brief Adjust the External High Speed oscillator (HSE) Startup Timeout value
   */
#define HSE_READY_TIME    ((uint16_t)0xFFFF)    /*!< Time out for HSE start up                              */




/**
  * @}
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */
#define __MPU_PRESENT             0    /*!< MPU present or not                                              */
#define __NVIC_PRIO_BITS          4    /*!< Number of Bits used for Priority Levels                         */
#define __Vendor_SysTickConfig    0    /*!< Set to 1 if different SysTick Config is used                    */

/**
  * @}
  */


/** @addtogroup Configuration_for_Inetrrupt_Number
  * @{
  */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ******************************                              */
  NonMaskableInt_IRQn     = -14,    /*!< 2 Non Maskable Interrupt                                           */
  HardFault_IRQn          = -13,    /*!< 3 Cortex-M3 Hard Fault Interrupt                                   */
  MemoryManagement_IRQn   = -12,    /*!< 4 Cortex-M3 Memory Management Int                                  */
  BusFault_IRQn           = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                                    */
  UsageFault_IRQn         = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                                  */
  SVCall_IRQn             = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                                     */
  DebugMonitor_IRQn       = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                               */
  PendSV_IRQn             = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                                     */
  SysTick_IRQn            = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                                 */

/******  HT32 Specific Interrupt Numbers ************************************                               */
  CKRDY_IRQn              = 0,      /*!< Clock ready interrupt                                              */
  LVD_IRQn                = 1,      /*!< Low voltage detection interrupt                                    */
  BOD_IRQn                = 2,      /*!< Brown-Out detection interrupt                                      */
  WDT_IRQn                = 3,      /*!< WDT global interrupt                                               */
  RTC_IRQn                = 4,      /*!< RTC Wake-up Interrupt                                              */
  FLASH_IRQn              = 5,      /*!< FLASH global Interrupt                                             */
  EVWUP_IRQn              = 6,      /*!< Event Wake-up Interrupt                                            */
  LPWUP_IRQn              = 7,      /*!< WAKEUP pin Interrupt                                               */
  EXTI0_IRQn              = 8,      /*!< EXTI0 Line detection Interrupt                                     */
  EXTI1_IRQn              = 9,      /*!< EXTI1 Line detection Interrupt                                     */
  EXTI2_IRQn              = 10,     /*!< EXTI2 Line detection Interrupt                                     */
  EXTI3_IRQn              = 11,     /*!< EXTI3 Line detection Interrupt                                     */
  EXTI4_IRQn              = 12,     /*!< EXTI4 Line detection Interrupt                                     */
  EXTI5_IRQn              = 13,     /*!< EXTI5 Line detection Interrupt                                     */
  EXTI6_IRQn              = 14,     /*!< EXTI6 Line detection Interrupt                                     */
  EXTI7_IRQn              = 15,     /*!< EXTI7 Line detection Interrupt                                     */
  EXTI8_IRQn              = 16,     /*!< EXTI8 Line detection Interrupt                                     */
  EXTI9_IRQn              = 17,     /*!< EXTI9 Line detection Interrupt                                     */
  EXTI10_IRQn             = 18,     /*!< EXTI10 Line detection Interrupt                                    */
  EXTI11_IRQn             = 19,     /*!< EXTI11 Line detection Interrupt                                    */
  EXTI12_IRQn             = 20,     /*!< EXTI12 Line detection Interrupt                                    */
  EXTI13_IRQn             = 21,     /*!< EXTI13 Line detection Interrupt                                    */
  EXTI14_IRQn             = 22,     /*!< EXTI14 Line detection Interrupt                                    */
  EXTI15_IRQn             = 23,     /*!< EXTI15 Line detection Interrupt                                    */
  COMP_IRQn               = 24,     /*!< Comparator global Interrupt                                        */
  ADC_IRQn                = 25,     /*!< ADC Interrupt                                                      */
  MCTMBRK_IRQn            = 27,     /*!< MCTM BRK interrupt                                                 */
  MCTMUP_IRQn             = 28,     /*!< MCTM UP interrupt                                                  */
  MCTMTR_IRQn             = 29,     /*!< MCTM TR interrupt                                                  */
  MCTMCC_IRQn             = 30,     /*!< MCTM CC interrupt                                                  */
  GPTM0_IRQn              = 35,     /*!< General-Purpose Timer0 Interrupt                                   */
  GPTM1_IRQn              = 36,     /*!< General-Purpose Timer1 Interrupt                                   */
  BFTM0_IRQn              = 41,     /*!< Basic Function Timer0 interrupt                                    */
  BFTM1_IRQn              = 42,     /*!< Basic Function Timer1 interrupt                                    */
  I2C0_IRQn               = 43,     /*!< I2C0 global Interrupt                                              */
  I2C1_IRQn               = 44,     /*!< I2C1 global Interrupt                                              */
  SPI0_IRQn               = 45,     /*!< SPI0 global Interrupt                                              */
  SPI1_IRQn               = 46,     /*!< SPI1 global Interrupt                                              */
  USART0_IRQn             = 47,     /*!< USART0 global Interrupt                                            */
  USART1_IRQn             = 48,     /*!< USART1 global Interrupt                                            */
  SCI_IRQn                = 51,     /*!< Smart Card interface interrupt                                     */
  USB_IRQn                = 53,     /*!< USB interrupt                                                      */
  PDMACH0_IRQn            = 55,     /*!< PDMA channel 0 global interrupt                                    */
  PDMACH1_IRQn            = 56,     /*!< PDMA channel 1 global interrupt                                    */
  PDMACH2_IRQn            = 57,     /*!< PDMA channel 2 global interrupt                                    */
  PDMACH3_IRQn            = 58,     /*!< PDMA channel 3 global interrupt                                    */
  PDMACH4_IRQn            = 59,     /*!< PDMA channel 4 global interrupt                                    */
  PDMACH5_IRQn            = 60,     /*!< PDMA channel 5 global interrupt                                    */
  PDMACH6_IRQn            = 61,     /*!< PDMA channel 6 global interrupt                                    */
  PDMACH7_IRQn            = 62,     /*!< PDMA channel 7 global interrupt                                    */
  PDMACH8_IRQn            = 63,     /*!< PDMA channel 8 global interrupt                                    */
  PDMACH9_IRQn            = 64,     /*!< PDMA channel 9 global interrupt                                    */
  PDMACH10_IRQn           = 65,     /*!< PDMA channel 10 global interrupt                                   */
  PDMACH11_IRQn           = 66,     /*!< PDMA channel 11 global interrupt                                   */
  CSIF_IRQn               = 67      /*!< CMOS sensor interface interrupt                                    */
} IRQn_Type;


/**
  * @}
  */

#include "core_cm3.h"                  /* Cortex-M3 processor and core peripherals                          */
#include "system_ht32f175x_275x.h"     /* HT32 system                                                       */


/** @addtogroup Exported_Types
  * @{
  */

typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;

typedef const int32_t sc32;           /*!< Read Only                                                        */
typedef const int16_t sc16;           /*!< Read Only                                                        */
typedef const int8_t  sc8;            /*!< Read Only                                                        */

typedef __IO int32_t vs32;
typedef __IO int16_t vs16;
typedef __IO int8_t  vs8;

typedef __I int32_t vsc32;            /*!< Read Only                                                        */
typedef __I int16_t vsc16;            /*!< Read Only                                                        */
typedef __I int8_t  vsc8;             /*!< Read Only                                                        */

typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long  const uc32;    /*!< Read Only                                                        */
typedef unsigned short const uc16;    /*!< Read Only                                                        */
typedef unsigned char  const uc8;     /*!< Read Only                                                        */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t  vu16;
typedef __IO uint8_t   vu8;

typedef __I uint32_t vuc32;           /*!< Read Only                                                        */
typedef __I uint16_t vuc16;           /*!< Read Only                                                        */
typedef __I uint8_t  vuc8;            /*!< Read Only                                                        */


typedef enum {DISABLE = 0, ENABLE = !DISABLE} EventStatus, ControlStatus;
#define IS_CONTROL_STATUS(STATUS) ((STATUS == DISABLE) || (STATUS == ENABLE))

typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus;

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrStatus;

/**
  * @}
  */

#if defined (__CC_ARM)
  #define __ALIGN4 __align(4)
#elif defined (__ICCARM__)
  #define __ALIGN4 _Pragma("data_alignment = 4")
#elif defined (__GNUC__)
  #define __ALIGN4  __attribute__((aligned(4)))
#endif

#if defined (__GNUC__)
  #define __PACKED_H
  #define __PACKED_F __attribute__ ((packed))
#elif defined (__ICCARM__) || (__CC_ARM)
  #define __PACKED_H __packed
  #define __PACKED_F
#endif


#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)


/**
 * @brief Exported constants and macro
 */
#define wb(addr, value)     (*((u8  volatile *) (addr)) = value)
#define rb(addr)            (*((u8  volatile *) (addr)))
#define whw(addr, value)    (*((u16 volatile *) (addr)) = value)
#define rhw(addr)           (*((u16 volatile *) (addr)))
#define ww(addr, value)     (*((u32 volatile *) (addr)) = value)
#define rw(addr)            (*((u32 volatile *) (addr)))


#define ResetBit_BB(Addr, BitNumber) (*(vu32 *) ((Addr & 0xF0000000) + 0x02000000 + \
                                      ((Addr & 0xFFFFF) << 5) + (BitNumber << 2)) = 0)
#define SetBit_BB(Addr, BitNumber)   (*(vu32 *) ((Addr & 0xF0000000) + 0x02000000 + \
                                      ((Addr & 0xFFFFF) << 5) + (BitNumber << 2)) = 1)
#define GetBit_BB(Addr, BitNumber)   (*(vu32 *) ((Addr & 0xF0000000) + 0x02000000 + \
                                      ((Addr & 0xFFFFF) << 5) + (BitNumber << 2)))
#define BitBand(Addr, BitNumber)     (*(vu32 *) ((Addr & 0xF0000000) + 0x02000000 + \
                                     ((Addr & 0xFFFFF) << 5) + (BitNumber << 2)))


#ifndef EXT
  #define EXT extern
#endif

/** @addtogroup Peripheral_Registers_Structures
  * @{
  */


/**
 * @brief Universal Synchronous Asynchronous Receiver Transmitter
 */
typedef struct
{
                                 /* USART0: 0x40000000                                                      */
                                 /* USART1: 0x40040000                                                      */
  __IO uint32_t RBR;             /*!< 0x000         Receive Buffer Register/Transmit Holding Register       */
  __IO uint32_t IER;             /*!< 0x004         Interrupt Enable Register                               */
  __IO uint32_t IIR;             /*!< 0x008         Interrupt Identification Register/FIFO Control Register */
  __IO uint32_t FCR;             /*!< 0x00C         FIFO Control Register                                   */
  __IO uint32_t LCR;             /*!< 0x010         Line Control Register                                   */
  __IO uint32_t MCR;             /*!< 0x014         Modem Control Register                                  */
  __IO uint32_t LSR;             /*!< 0x018         Line Status Register                                    */
  __IO uint32_t MSR;             /*!< 0x01C         Modem Status Register                                   */
  __IO uint32_t TPR;             /*!< 0x020         Timing Parameter Register                               */
  __IO uint32_t MDR;             /*!< 0x024         Mode Register                                           */
  __IO uint32_t ICR;             /*!< 0x028         IrDA Register                                           */
  __IO uint32_t RCR;             /*!< 0x02C         RS485 Control Register                                  */
  __IO uint32_t SCR;             /*!< 0x030         Synchronous Control Register                            */
  __IO uint32_t FSR;             /*!< 0x034         FIFO Status Register                                    */
  __IO uint32_t DLR;             /*!< 0x038         Divisor Latch Register                                  */
  __IO uint32_t DTR;             /*!< 0x040         Debug/Test Register                                     */
} USART_TypeDef;


/**
 * @brief SPI
 */
typedef struct
{
                                 /* SPI0: 0x40004000                                                        */
                                 /* SPI1: 0x40044000                                                        */
  __IO uint32_t CR0;             /*!< 0x000         Control Register 0                                      */
  __IO uint32_t CR1;             /*!< 0x004         Control Register 1                                      */
  __IO uint32_t IER;             /*!< 0x008         Interrupt Enable Register                               */
  __IO uint32_t CPR;             /*!< 0x00C         Clock Prescale Register                                 */
  __IO uint32_t DR;              /*!< 0x010         Data Register                                           */
  __IO uint32_t SR;              /*!< 0x014         Status Register                                         */
  __IO uint32_t FCR;             /*!< 0x018         FIFO Control Register                                   */
  __IO uint32_t FSR;             /*!< 0x01C         FIFO Status Register                                    */
  __IO uint32_t FTOCR;           /*!< 0x020         FIFO Time Out Conuter Register                          */
} SPI_TypeDef;


/**
 * @brief Analog to Digital Converter
 */
typedef struct
{
       uint32_t RESERVE0[1];     /*!< 0x000         Reserved                                                */
  __IO uint32_t RST;             /*!< 0x004         ADC Reset Register                                      */
  __IO uint32_t CONV;            /*!< 0x008         ADC (Regular) Conversion Mode Register                  */
       uint32_t RESERVE1[1];     /*!< 0x00C         Reserved                                                */
  __IO uint32_t LST[4];          /*!< 0x010 - 0x01C ADC (Regular) Conversion List Register 0-3              */
       uint32_t RESERVE2[4];     /*!< 0x020 - 0x02C Reserved                                                */
  __IO uint32_t OFR[8];          /*!< 0x030 - 0x04C ADC Input Offset Register 0-7                           */
       uint32_t RESERVE3[8];     /*!< 0x050 - 0x06C Reserved                                                */
  __IO uint32_t STR[8];          /*!< 0x070 - 0x08C ADC Input Sampling Time Register 0-7                    */
       uint32_t RESERVE4[8];     /*!< 0x090 - 0x0AC Reserved                                                */
  __IO uint32_t DR[16];          /*!< 0x0B0 - 0x0EC ADC (Regular) Conversion Data Register 0-15             */
       uint32_t RESERVE5[4];     /*!< 0x0F0 - 0x0FC Reserved                                                */
  __IO uint32_t TCR;             /*!< 0x100         ADC (Regular) Trigger Control Register                  */
  __IO uint32_t TSR;             /*!< 0x104         ADC (Regular) Trigger Source Register                   */
       uint32_t RESERVE6[6];     /*!< 0x108 - 0x11C Reserved                                                */
  __IO uint32_t WCR;             /*!< 0x120         ADC Watchdog Control Register                           */
  __IO uint32_t LTR;             /*!< 0x124         ADC Lower Threshold Register                            */
  __IO uint32_t UTR;             /*!< 0x128         ADC Upper Threshold Register                            */
       uint32_t RESERVE7[1];     /*!< 0x12C         Reserved                                                */
  __IO uint32_t IM;              /*!< 0x130         ADC Interrupt Mask Enable Register                      */
  __IO uint32_t IRAW;            /*!< 0x134         ADC Interrupt Raw Status Register                       */
  __IO uint32_t IMASK;           /*!< 0x138         ADC Interrupt Masked Status Register                    */
  __IO uint32_t ICLR;            /*!< 0x13C         ADC Interrupt Clear Register                            */
  __IO uint32_t PDMAR;           /*!< 0x140         ADC PDMA Request Register                               */
} ADC_TypeDef;


/**
 * @brief Op Amp/Comparator
 */
typedef struct
{
                                /* CMP_OP0: 0x40018000                                                      */
                                /* CMP_OP1: 0x40018100                                                      */
  __IO uint32_t OPAC;           /*!< 0x000         Operational Amplifier control register                   */
  __IO uint32_t OFVC;           /*!< 0x004         Comparator control register                              */
  __IO uint32_t CMPIM;          /*!< 0x008         Comparator interrupt mask enable                         */
  __IO uint32_t CMPIRAW;        /*!< 0x00C         Comparator raw interrupt status                          */
  __IO uint32_t CMPIMASK;       /*!< 0x010         Comparator masked interrupt status                       */
  __IO uint32_t CMPICLR;        /*!< 0x014         Comparator interrupt clear register                      */
} CMP_OP_TypeDef;


/**
 * @brief General Purpose I/O
 */
typedef struct
{
                                /* GPIOA: 0x4001A000                                                        */
                                /* GPIOB: 0x4001B000                                                        */
                                /* GPIOC: 0x4001C000                                                        */
                                /* GPIOD: 0x4001D000                                                        */
                                /* GPIOE: 0x4001E000                                                        */
  __IO uint32_t DIRCR;          /*!< 0x000         Data Direction Control Register                          */
  __IO uint32_t INER;           /*!< 0x004         Input function enable register                           */
  __IO uint32_t PUR;            /*!< 0x008         Pull-Up Selection Register                               */
  __IO uint32_t PDR;            /*!< 0x00C         Pull-Down Selection Register                             */
  __IO uint32_t ODR;            /*!< 0x010         Open Drain Selection Register                            */
  __IO uint32_t DRVR;           /*!< 0x014         Drive Current Selection Register                         */
  __IO uint32_t LOCKR;          /*!< 0x018         Lock Register                                            */
  __IO uint32_t DINR;           /*!< 0x01c         Data Input Register                                      */
  __IO uint32_t DOUTR;          /*!< 0x020         Data Output Register                                     */
  __IO uint32_t SRR;            /*!< 0x024         Output Set and Reset Control Register                    */
  __IO uint32_t RR;             /*!< 0x028         Output Reset Control Register                            */
} GPIO_TypeDef;


/**
 * @brief AFIO
 */
typedef struct
{
                                /* AFIO: 0x40022000                                                         */
  __IO uint32_t ESSR[2];        /*!< 0x000         EXTI Source Selection Register 0 ~ 1                     */
  __IO uint32_t GPACFGR;        /*!< 0x008         GPIO A Configuration Register                            */
  __IO uint32_t GPBCFGR;        /*!< 0x00C         GPIO B Configuration Register                            */
  __IO uint32_t GPCCFGR;        /*!< 0x010         GPIO C Configuration Register                            */
  __IO uint32_t GPDCFGR;        /*!< 0x014         GPIO D Configuration Register                            */
  __IO uint32_t GPECFGR;        /*!< 0x018         GPIO E Configuration Register                            */
} AFIO_TypeDef;


/**
 * @brief External Interrupt/Event Controller
 */
typedef struct
{
                                 /* EXTI: 0x40024000                                                        */
  __IO uint32_t CFGR0;           /*!< 0x000         EXTI Interrupt 0 Configuration Register                 */
  __IO uint32_t CFGR1;           /*!< 0x000         EXTI Interrupt 1 Configuration Register                 */
  __IO uint32_t CFGR2;           /*!< 0x000         EXTI Interrupt 2 Configuration Register                 */
  __IO uint32_t CFGR3;           /*!< 0x000         EXTI Interrupt 3 Configuration Register                 */
  __IO uint32_t CFGR4;           /*!< 0x000         EXTI Interrupt 4 Configuration Register                 */
  __IO uint32_t CFGR5;           /*!< 0x000         EXTI Interrupt 5 Configuration Register                 */
  __IO uint32_t CFGR6;           /*!< 0x000         EXTI Interrupt 6 Configuration Register                 */
  __IO uint32_t CFGR7;           /*!< 0x000         EXTI Interrupt 7 Configuration Register                 */
  __IO uint32_t CFGR8;           /*!< 0x000         EXTI Interrupt 8 Configuration Register                 */
  __IO uint32_t CFGR9;           /*!< 0x000         EXTI Interrupt 9 Configuration Register                 */
  __IO uint32_t CFGR10;          /*!< 0x000         EXTI Interrupt 10 Configuration Register                */
  __IO uint32_t CFGR11;          /*!< 0x000         EXTI Interrupt 11 Configuration Register                */
  __IO uint32_t CFGR12;          /*!< 0x000         EXTI Interrupt 12 Configuration Register                */
  __IO uint32_t CFGR13;          /*!< 0x000         EXTI Interrupt 13 Configuration Register                */
  __IO uint32_t CFGR14;          /*!< 0x000         EXTI Interrupt 14 Configuration Register                */
  __IO uint32_t CFGR15;          /*!< 0x000         EXTI Interrupt 15 Configuration Register                */
  __IO uint32_t CR;              /*!< 0x040         EXTI Interrupt Control Register                         */
  __IO uint32_t EDGEFLGR;        /*!< 0x044         EXTI Interrupt Edge Flag Register                       */
  __IO uint32_t EDGESR;          /*!< 0x048         EXTI Interrupt Edge Status Register                     */
  __IO uint32_t SCR;             /*!< 0x04C         EXTI Interrupt Software Command Register                */
  __IO uint32_t WAKUPCR;         /*!< 0x050         EXTI Interrupt Wakeup Control Register                  */
  __IO uint32_t WAKUPPOLR;       /*!< 0x054         EXTI Interrupt Wakeup Polarity Register                 */
  __IO uint32_t WAKUPFLG;        /*!< 0x058         EXTI Interrupt Wakeup Flag Register                     */
} EXTI_TypeDef;


/**
 * @brief Inter-integrated Circuit Interface
 */
typedef struct
{
                                 /* I2C0: 0x40048000                                                        */
                                 /* I2C1: 0x40049000                                                        */
  __IO uint32_t CR;              /*!< 0x000         Control Register                                        */
  __IO uint32_t IER;             /*!< 0x004         Interrupt Enable Register                               */
  __IO uint32_t ADDR;            /*!< 0x008         Address Register                                        */
  __IO uint32_t SR;              /*!< 0x00C         Status Register                                         */
  __IO uint32_t SHPGR;           /*!< 0x010         SCL High Period Generation Register                     */
  __IO uint32_t SLPGR;           /*!< 0x014         SCL Low Period Generation Register                      */
  __IO uint32_t DR;              /*!< 0x018         Data Register                                           */
  __IO uint32_t TAR;             /*!< 0x01C         Target Register                                         */
  __IO uint32_t ADDMR;           /*!< 0x020         Address Mask Register                                   */
  __IO uint32_t ADDBR;           /*!< 0x024         Address Buffer Register                                 */
  __IO uint32_t TOUT;            /*!< 0x028         Timeout Register                                        */
} I2C_TypeDef;


/**
 * @brief WATCHDOG
 */
typedef struct
{
                                 /* WDT: 0x40068000                                                         */
  __IO uint32_t CR;              /*!< 0x000         Control Register                                        */
  __IO uint32_t MR0;             /*!< 0x004         Mode 0 Register                                         */
  __IO uint32_t MR1;             /*!< 0x008         Mode 1 Register                                         */
  __IO uint32_t SR;              /*!< 0x00C         Status Register                                         */
  __IO uint32_t PR;              /*!< 0x010         WDT Write Protect Register                              */
} WDT_TypeDef;


/**
 * @brief Real-Time Clock
 */
typedef struct
{
                                 /* RTC: 0x4006A000                                                         */
  __IO uint32_t CNT;             /*!< 0x000         RTC Counter Rgister                                     */
  __IO uint32_t CMP;             /*!< 0x004         RTC Compare Rgister                                     */
  __IO uint32_t CR;              /*!< 0x008         RTC Control Rgister                                     */
  __IO uint32_t SR;              /*!< 0x00C         RTC Status Register                                     */
  __IO uint32_t IWEN;            /*!< 0x010         RTC Interrup/Wake-up Enable Register                    */
} RTC_TypeDef;


/**
 * @brief Power Control Unit
 */
typedef struct
{
                                 /* PWRCU: 0x4006A100                                                       */
  __IO uint32_t BAKSR;           /*!< 0x000         Backup Domain Status Register                           */
  __IO uint32_t BAKCR;           /*!< 0x004         Backup Domain Control Register                          */
  __IO uint32_t BAKTEST;         /*!< 0x008         Backup Domain Test Register                             */
       uint32_t HSIRCR;          /*!< 0x00C         HSI Ready Counter Control Register                      */
  __IO uint32_t LVDCSR;          /*!< 0x010         Low Voltage/Brown Out Detect Control and Status Register*/
       uint32_t RESERVE2[59];    /*!< 0x014 ~ 0x0FC Reserved                                                */
  __IO uint32_t BAKREG[10];      /*!< 0x100 ~ 0x124 Backup Register 0 ~ 9                                   */
} PWRCU_TypeDef;


/**
 * @brief General-Purpose Timer
 */
typedef struct
{
                                 /* GPTM0: 0x4006E000                                                       */
                                 /* GPTM1: 0x4006F000                                                       */
  __IO uint32_t CNTCFR;          /*!< 0x000          Counter Configuration Register                         */
  __IO uint32_t MDCFR;           /*!< 0x004          Mode Configuration Register                            */
  __IO uint32_t TRCFR;           /*!< 0x008          Trigger Configuration Register                         */
       uint32_t REV0;            /*!< 0x00C          Reserved                                               */
  __IO uint32_t CTR;             /*!< 0x010          Control Register                                       */
       uint32_t RESERVED1[3];    /*!< 0x014 - 0x01C  Reserved                                               */
  __IO uint32_t CH0ICFR;         /*!< 0x020          Channel-0 Input Configuration Register                 */
  __IO uint32_t CH1ICFR;         /*!< 0x024          Channel-1 Input Configuration Register                 */
  __IO uint32_t CH2ICFR;         /*!< 0x028          Channel-2 Input Configuration Register                 */
  __IO uint32_t CH3ICFR;         /*!< 0x02C          Channel-3 Input Configuration Register                 */
       uint32_t RESERVED2[4];    /*!< 0x030 - 0x03C  Reserved                                               */
  __IO uint32_t CH0OCFR;         /*!< 0x040          Channel-0 Output Configuration Register                */
  __IO uint32_t CH1OCFR;         /*!< 0x044          Channel-1 Output Configuration Register                */
  __IO uint32_t CH2OCFR;         /*!< 0x048          Channel-2 Output Configuration Register                */
  __IO uint32_t CH3OCFR;         /*!< 0x04C          Channel-3 Output Configuration Register                */
  __IO uint32_t CHCTR;           /*!< 0x050          Channel Control Register                               */
  __IO uint32_t CHPOLR;          /*!< 0x054          Channel Polarity Configuration Register                */
       uint32_t RESERVED3[7];    /*!< 0x058 - 0x073  Reserved                                               */
  __IO uint32_t DICTR;           /*!< 0x074          DMA / Interrupt Control Register                       */
  __IO uint32_t EVGR;            /*!< 0x078          Software Interrupt Control Register                    */
  __IO uint32_t INTSR;           /*!< 0x07C          Interrupt Status Register                              */
  __IO uint32_t CNTR;            /*!< 0x080          Counter Register                                       */
  __IO uint32_t PSCR;            /*!< 0x084          Prescaler Register                                     */
  __IO uint32_t CRR;             /*!< 0x088          Counter Reload Register                                */
       uint32_t RESERVED4;       /*!< 0x08C          Reserved                                               */
  __IO uint32_t CH0CCR;          /*!< 0x090          Channel 0 Capture/Compare Register                     */
  __IO uint32_t CH1CCR;          /*!< 0x094          Channel 1 Capture/Compare Register                     */
  __IO uint32_t CH2CCR;          /*!< 0x098          Channel 2 Capture/Compare Register                     */
  __IO uint32_t CH3CCR;          /*!< 0x09C          Channel 3 Capture/Compare Register                     */
} GPTM_TypeDef;


/**
 * @brief Flash Memory Controller
 */
typedef struct
{
                                 /* FLASH: 0x40080000                                                       */
  __IO uint32_t TADR;            /*!< 0x000         Flash Target Address Register                           */
  __IO uint32_t WRDR;            /*!< 0x004         Flash Write Data Register                               */
  __IO uint32_t RDDR;            /*!< 0x008         Flash Read Data Register                                */
  __IO uint32_t OCMR;            /*!< 0x00C         Flash Operation Command Register                        */
  __IO uint32_t OPCR;            /*!< 0x010         Flash Operation Control Register                        */
  __IO uint32_t OIER;            /*!< 0x014         Flash Operation Interrupt Enable Register               */
  __IO uint32_t OISR;            /*!< 0x018         Flash Operation Interrupt and Status Register           */
       uint32_t RESERVED0;       /*!< 0x01C         Reserved                                                */
  __IO uint32_t PPSR[4];         /*!< 0x020 ~ 0x02C Flash Page Erase/Program Protection Status Register     */
  __IO uint32_t CPSR;            /*!< 0x030         Flash Security Protection Status Register               */
       uint32_t RESERVED1[51];   /*!< 0x034 ~ 0x0FF Reserved                                                */
  __IO uint32_t VMCR;            /*!< 0x100         Flash Vector Mapping Control Register                   */
       uint32_t RESERVED2[31];   /*!< 0x104 ~ 0x17F Reserved                                                */
  __IO uint32_t MDID;            /*!< 0x180         Flash Manufacturer and Device ID Register               */
  __IO uint32_t PNSR;            /*!< 0x184         Flash Page Number Status Register                       */
  __IO uint32_t PSSR;            /*!< 0x188         Flash Page Size StatusRegister                          */
       uint32_t RESERVED3[29];   /*!< 0x18C ~ 0x1FF Reserved                                                */
  __IO uint32_t CFCR;            /*!< 0x200         Flash Pre-fetch Control Register                        */
  __IO uint32_t CNT8M;           /*!< 0x204         Counter 8MHz Counting Value Register                    */
       uint32_t RESERVED4[62];   /*!< 0x208 ~ 0x2FF Reserved                                                */
  __IO uint32_t SBVT[4];         /*!< 0x300 ~ 0x30C SRAM Booting Vector (4x32Bit)                           */
} FLASH_TypeDef;


/**
 * @brief Clock Control Unit
 */
typedef struct
{
                                 /* CKCU: 0x40088000                                                        */
  __IO uint32_t GCFGR;           /*!< 0x000         Global Clock Configuration Register                     */
  __IO uint32_t GCCR;            /*!< 0x004         Global Clock Control Register                           */
  __IO uint32_t GCSR;            /*!< 0x008         Global Clock Status Register                            */
  __IO uint32_t GCIR;            /*!< 0x00C         Global Clock Interrupt Register                         */
  __IO uint32_t STCALIR;         /*!< 0x010         System Tick Calibration Register                        */
  __IO uint32_t CKWKUPCR;        /*!< 0x014         Clock Wakeup Control Register                           */
  __IO uint32_t PLLCFGR;         /*!< 0x018         PLL Configuration Register                              */
  __IO uint32_t PLLCR;           /*!< 0x01C         PLL Control Register                                    */
  __IO uint32_t AHBCFGR;         /*!< 0x020         AHB Configuration Register                              */
  __IO uint32_t AHBCCR;          /*!< 0x024         AHB Clock Control Register                              */
  __IO uint32_t APBCFGR;         /*!< 0x028         APB Configuration Register                              */
  __IO uint32_t APBCCR0;         /*!< 0x02C         APB Clock Control Register 0                            */
  __IO uint32_t APBCCR1;         /*!< 0x030         APB Clock Control Register 1                            */
  __IO uint32_t CKST;            /*!< 0x034         Clock source status Register                            */
       uint32_t RESERVED0[49];   /*!< 0x038 ~ 0x0FB Reserved                                                */
  __IO uint32_t DBGCLK;          /*!< 0x0FC         Debug clock Register                                    */
       uint32_t RESERVED1[128];  /*!< 0x100 ~ 0x2FF Reserved                                                */
  __IO uint32_t LPCR;            /*!< 0x300         Low Power Control Register                              */
  __IO uint32_t MCUDBGCR;        /*!< 0x304         MCU Debug Control Register                              */
} CKCU_TypeDef;


/**
 * @brief Reset Control Unit
 */
typedef struct
{
                                 /* RSTCU: 0x40088100                                                       */
  __IO uint32_t GRSR;            /*!< 0x000         Global Reset Status Register                            */
  __IO uint32_t AHBPRST;         /*!< 0x004         AHB Peripheral Reset Register                           */
  __IO uint32_t APBPRST0;        /*!< 0x008         APB Peripheral Reset Register 0                         */
  __IO uint32_t APBPRST1;        /*!< 0x00C         APB Peripheral Reset Register 1                         */
} RSTCU_TypeDef;


/**
 * @brief CMOS Sensor Interface
 */
typedef struct
{
                                 /* CSIF: 0x400CC000                                                        */
  __IO uint32_t ENR;             /*!< 0x000         Enable Register                                         */
  __IO uint32_t CR;              /*!< 0x004         Control Register                                        */
  __IO uint32_t IMGWH;           /*!< 0x008         Image Width/Height Register                             */
  __IO uint32_t WCR0;            /*!< 0x00C         Window Capture 0 Register                               */
  __IO uint32_t WCR1;            /*!< 0x010         Window Capture 1 Register                               */
  __IO uint32_t SMP;             /*!< 0x014         Row & Column Sub-Sample Register                        */
  __IO uint32_t SMPCOL;          /*!< 0x018         Column Sub-Sample Register                              */
  __IO uint32_t SMPROW;          /*!< 0x01C         Row Sub-Sample Register                                 */
  __IO uint32_t FIFO0;           /*!< 0x020         FIFO Register 0                                         */
  __IO uint32_t FIFO1;           /*!< 0x024         FIFO Register 1                                         */
  __IO uint32_t FIFO2;           /*!< 0x028         FIFO Register 2                                         */
  __IO uint32_t FIFO3;           /*!< 0x02C         FIFO Register 3                                         */
  __IO uint32_t FIFO4;           /*!< 0x030         FIFO Register 4                                         */
  __IO uint32_t FIFO5;           /*!< 0x034         FIFO Register 5                                         */
  __IO uint32_t FIFO6;           /*!< 0x038         FIFO Register 6                                         */
  __IO uint32_t FIFO7;           /*!< 0x03C         FIFO Register 7                                         */
  __IO uint32_t IER;             /*!< 0x040         Interrupt Enable Register                               */
  __IO uint32_t SR;              /*!< 0x044         Status Register                                         */
} CSIF_TypeDef;


/**
 * @brief Smart Card Interface
 */
typedef struct
{
                                 /* SCI: 0x40043000                                                         */
  __IO uint32_t CR;              /*!< 0x000         Controle Register                                       */
  __IO uint32_t SR;              /*!< 0x004         Status Register                                         */
  __IO uint32_t CCR;             /*!< 0x008         Contact Control Register                                */
  __IO uint32_t ETU;             /*!< 0x00C         Elementary Time Unit Register                           */
  __IO uint32_t GT;              /*!< 0x010         Guardtime Register                                      */
  __IO uint32_t WT;              /*!< 0x014         Waiting Time Register                                   */
  __IO uint32_t IER;             /*!< 0x018         Interrupt Enable Register                               */
  __IO uint32_t IPR;             /*!< 0x01C         Interrupt Pending Register                              */
  __IO uint32_t TXB;             /*!< 0x020         Transmit Buffer Register                                */
  __IO uint32_t RXB;             /*!< 0x024         Receive Buffer Register                                 */
  __IO uint32_t PSC;             /*!< 0x028         Prescaler Register                                      */
} SCI_TypeDef;


/**
 * @brief Basic Function Timer
 */
typedef struct
{
                                /* BFTM0: 0x40076000                                                        */
                                /* BFTM1: 0x40077000                                                        */
  __IO uint32_t CR;             /*!< 0x000          Control Register                                        */
  __IO uint32_t SR;             /*!< 0x004          Status Register                                         */
  __IO uint32_t CNTR;           /*!< 0x008          Counter Value Register                                  */
  __IO uint32_t CMP;            /*!< 0x00C          Compare Value Register                                  */
} BFTM_TypeDef;


/**
 * @brief Motor Control Timer
 */
typedef struct
{
                                 /* MCTM: 0x4002C000                                                        */
  __IO uint32_t CNTCFR;          /*!< 0x000          Counter Configuration Register                         */
  __IO uint32_t MDCFR;           /*!< 0x004          Mode Configuration Register                            */
  __IO uint32_t TRCFR;           /*!< 0x008          Trigger Configuration Register                         */
       uint32_t REV0;            /*!< 0x00C          Reserved                                               */
  __IO uint32_t CTR;             /*!< 0x010          Control Register                                       */
       uint32_t RESERVED1[3];    /*!< 0x014 - 0x01F  Reserved                                               */
  __IO uint32_t CH0ICFR;         /*!< 0x020          Channel-0 Input Configuration Register                 */
  __IO uint32_t CH1ICFR;         /*!< 0x024          Channel-1 Input Configuration Register                 */
  __IO uint32_t CH2ICFR;         /*!< 0x028          Channel-2 Input Configuration Register                 */
  __IO uint32_t CH3ICFR;         /*!< 0x02C          Channel-3 Input Configuration Register                 */
       uint32_t RESERVED2[4];    /*!< 0x030 - 0x03F  Reserved                                               */
  __IO uint32_t CH0OCFR;         /*!< 0x040          Channel-0 Output Configuration Register                */
  __IO uint32_t CH1OCFR;         /*!< 0x044          Channel-1 Output Configuration Register                */
  __IO uint32_t CH2OCFR;         /*!< 0x048          Channel-2 Output Configuration Register                */
  __IO uint32_t CH3OCFR;         /*!< 0x04C          Channel-3 Output Configuration Register                */
  __IO uint32_t CHCTR;           /*!< 0x050          Channel Control Register                               */
  __IO uint32_t CHPOLR;          /*!< 0x054          Channel Polarity Configuration Register                */
       uint32_t RESERVED3[5];    /*!< 0x058 - 0x06B  Reserved                                               */
  __IO uint32_t CHBRKCFR;        /*!< 0x06C          Channel Break Configuration Register                   */
  __IO uint32_t CHBRKCTR;        /*!< 0x070          Channel Break Control Register                         */
  __IO uint32_t DICTR;           /*!< 0x074          DMA / Interrupt Control Register                       */
  __IO uint32_t EVGR;            /*!< 0x078          Event Generator Register                               */
  __IO uint32_t INTSR;           /*!< 0x07C          Interrupt Status Register                              */
  __IO uint32_t CNTR;            /*!< 0x080          Counter Register                                       */
  __IO uint32_t PSCR;            /*!< 0x084          Prescaler Register                                     */
  __IO uint32_t CRR;             /*!< 0x088          Counter Reload Register                                */
  __IO uint32_t REPR;            /*!< 0x08C          Repetition Register                                    */
  __IO uint32_t CH0CCR;          /*!< 0x090          Channel 0 Capture/Compare Register                     */
  __IO uint32_t CH1CCR;          /*!< 0x094          Channel 1 Capture/Compare Register                     */
  __IO uint32_t CH2CCR;          /*!< 0x098          Channel 2 Capture/Compare Register                     */
  __IO uint32_t CH3CCR;          /*!< 0x09C          Channel 3 Capture/Compare Register                     */
} MCTM_TypeDef;


/**
 * @brief Peripheral Direct Memory Access Channel
 */
typedef struct
{
  __IO uint32_t CR;              /*!< 0x000    PDMA Channel Control Register                                */
  __IO uint32_t SADR;            /*!< 0x004    PDMA Channel Source Address Register                         */
  __IO uint32_t DADR;            /*!< 0x008    PDMA Channel Destination Address Register                    */
  __IO uint32_t CADR;            /*!< 0x00C    PDMA Channel Current Address Register                        */
  __IO uint32_t TSR;             /*!< 0x010    PDMA Channel Transfer Size Register                          */
  __IO uint32_t CTSR;            /*!< 0x014    PDMA Channel Current Transfer Size Register                  */
} PDMACH_TypeDef;


/**
 * @brief Peripheral Direct Memory Access Global
 */
typedef struct
{
                                  /* PDMA: 0x40090000                                                       */
  PDMACH_TypeDef  PDMACH0;        /*!< 0x000    PDMA channel  0 registers                                   */
  PDMACH_TypeDef  PDMACH1;        /*!< 0x018    PDMA channel  1 registers                                   */
  PDMACH_TypeDef  PDMACH2;        /*!< 0x030    PDMA channel  2 registers                                   */
  PDMACH_TypeDef  PDMACH3;        /*!< 0x048    PDMA channel  3 registers                                   */
  PDMACH_TypeDef  PDMACH4;        /*!< 0x060    PDMA channel  4 registers                                   */
  PDMACH_TypeDef  PDMACH5;        /*!< 0x078    PDMA channel  5 registers                                   */
  PDMACH_TypeDef  PDMACH6;        /*!< 0x090    PDMA channel  6 registers                                   */
  PDMACH_TypeDef  PDMACH7;        /*!< 0x0A8    PDMA channel  7 registers                                   */
  PDMACH_TypeDef  PDMACH8;        /*!< 0x0C0    PDMA channel  8 registers                                   */
  PDMACH_TypeDef  PDMACH9;        /*!< 0x0D8    PDMA channel  9 registers                                   */
  PDMACH_TypeDef  PDMACH10;       /*!< 0x0F0    PDMA channel 10 registers                                   */
  PDMACH_TypeDef  PDMACH11;       /*!< 0x108    PDMA channel 11 registers                                   */
  __IO uint32_t   ISR0;           /*!< 0x120    PDMA Interrupt Status Register 0                            */
  __IO uint32_t   ISR1;           /*!< 0x124    PDMA Interrupt Status Register 1                            */
  __IO uint32_t   ISCR0;          /*!< 0x128    PDMA Interrupt Status Clear Register 0                      */
  __IO uint32_t   ISCR1;          /*!< 0x12C    PDMA Interrupt Status Clear Register 1                      */
  __IO uint32_t   IER0;           /*!< 0x130    PDMA Interrupt Enable Register 0                            */
  __IO uint32_t   IER1;           /*!< 0x134    PDMA Interrupt Enable Register 1                            */
  __IO uint32_t   STTR;           /*!< 0x138    PDMA Software Trigger Test Register                         */
} PDMA_TypeDef;


/**
 * @brief Peripheral Universal Serial Bus Global
 */
typedef struct
{
                                 /* USB: 0x4004E000                                                         */
  __IO uint32_t CSR;             /*!< 0x000 USB Control and Status Register                                 */
  __IO uint32_t IER;             /*!< 0x004 USB Interrupt Enable Register                                   */
  __IO uint32_t ISR;             /*!< 0x008 USB Interrupt Status Register                                   */
  __IO uint32_t FCR;             /*!< 0x00C USB Frame Count Register                                        */
  __IO uint32_t DEVAR;           /*!< 0x010 USB Device Address Register                                     */
  __IO uint32_t EP0CSR;          /*!< 0x014 USB Endpoint 0 Control and Status Register                      */
  __IO uint32_t EP0IER;          /*!< 0x018 USB Endpoint 0 Interrupt Enable Register                        */
  __IO uint32_t EP0ISR;          /*!< 0x01C USB Endpoint 0 Interrupt Status Register                        */
  __IO uint32_t EP0TCR;          /*!< 0x020 USB Endpoint 0 Transfer Count Register                          */
  __IO uint32_t EP0CFGR;         /*!< 0x024 USB Endpoint 0 Configuration Register                           */
  __IO uint32_t EP1CSR;          /*!< 0x028 USB Endpoint 1 Control and Status Register                      */
  __IO uint32_t EP1IER;          /*!< 0x02C USB Endpoint 1 Interrupt Enable Register                        */
  __IO uint32_t EP1ISR;          /*!< 0x030 USB Endpoint 1 Interrupt Status Register                        */
  __IO uint32_t EP1TCR;          /*!< 0x034 USB Endpoint 1 Transfer Count Register                          */
  __IO uint32_t EP1CFGR;         /*!< 0x038 USB Endpoint 1 Configuration Register                           */
  __IO uint32_t EP2CSR;          /*!< 0x03C USB Endpoint 2 Control and Status Register                      */
  __IO uint32_t EP2IER;          /*!< 0x040 USB Endpoint 2 Interrupt Enable Register                        */
  __IO uint32_t EP2ISR;          /*!< 0x044 USB Endpoint 2 Interrupt Status Register                        */
  __IO uint32_t EP2TCR;          /*!< 0x048 USB Endpoint 2 Transfer Count Register                          */
  __IO uint32_t EP2CFGR;         /*!< 0x04C USB Endpoint 2 Configuration Register                           */
  __IO uint32_t EP3CSR;          /*!< 0x050 USB Endpoint 3 Control and Status Register                      */
  __IO uint32_t EP3IER;          /*!< 0x054 USB Endpoint 3 Interrupt Enable Register                        */
  __IO uint32_t EP3ISR;          /*!< 0x058 USB Endpoint 3 Interrupt Status Register                        */
  __IO uint32_t EP3TCR;          /*!< 0x05C USB Endpoint 3 Transfer Count Register                          */
  __IO uint32_t EP3CFGR;         /*!< 0x060 USB Endpoint 3 Configuration Register                           */
  __IO uint32_t EP4CSR;          /*!< 0x064 USB Endpoint 4 Control and Status Register                      */
  __IO uint32_t EP4IER;          /*!< 0x068 USB Endpoint 4 Interrupt Enable Register                        */
  __IO uint32_t EP4ISR;          /*!< 0x06C USB Endpoint 4 Interrupt Status Register                        */
  __IO uint32_t EP4TCR;          /*!< 0x070 USB Endpoint 4 Transfer Count Register                          */
  __IO uint32_t EP4CFGR;         /*!< 0x074 USB Endpoint 4 Configuration Register                           */
  __IO uint32_t EP5CSR;          /*!< 0x078 USB Endpoint 5 Control and Status Register                      */
  __IO uint32_t EP5IER;          /*!< 0x07C USB Endpoint 5 Interrupt Enable Register                        */
  __IO uint32_t EP5ISR;          /*!< 0x080 USB Endpoint 5 Interrupt Status Register                        */
  __IO uint32_t EP5TCR;          /*!< 0x084 USB Endpoint 5 Transfer Count Register                          */
  __IO uint32_t EP5CFGR;         /*!< 0x088 USB Endpoint 5 Configuration Register                           */
  __IO uint32_t EP6CSR;          /*!< 0x08C USB Endpoint 6 Control and Status Register                      */
  __IO uint32_t EP6IER;          /*!< 0x090 USB Endpoint 6 Interrupt Enable Register                        */
  __IO uint32_t EP6ISR;          /*!< 0x094 USB Endpoint 6 Interrupt Status Register                        */
  __IO uint32_t EP6TCR;          /*!< 0x098 USB Endpoint 6 Transfer Count Register                          */
  __IO uint32_t EP6CFGR;         /*!< 0x09C USB Endpoint 6 Configuration Register                           */
  __IO uint32_t EP7CSR;          /*!< 0x0A0 USB Endpoint 7 Control and Status Register                      */
  __IO uint32_t EP7IER;          /*!< 0x0A4 USB Endpoint 7 Interrupt Enable Register                        */
  __IO uint32_t EP7ISR;          /*!< 0x0A8 USB Endpoint 7 Interrupt Status Register                        */
  __IO uint32_t EP7TCR;          /*!< 0x0AC USB Endpoint 7 Transfer Count Register                          */
  __IO uint32_t EP7CFGR;         /*!< 0x0B0 USB Endpoint 7 Configuration Register                           */
} USB_TypeDef;

/**
 * @brief Peripheral Universal Serial Bus Endpoint
 */
typedef struct
{
                                 /* USB Endpoint0: 0x4004E014                                               */
                                 /* USB Endpoint1: 0x4004E028                                               */
                                 /* USB Endpoint2: 0x4004E03C                                               */
                                 /* USB Endpoint3: 0x4004E050                                               */
                                 /* USB Endpoint4: 0x4004E064                                               */
                                 /* USB Endpoint5: 0x4004E078                                               */
                                 /* USB Endpoint6: 0x4004E08C                                               */
                                 /* USB Endpoint7: 0x4004E0A0                                               */
  __IO uint32_t CSR;             /*!< 0x000 USB Endpoint n Control and Status Register                      */
  __IO uint32_t IER;             /*!< 0x004 USB Endpoint n Interrupt Enable Register                        */
  __IO uint32_t ISR;             /*!< 0x008 USB Endpoint n Interrupt Status Register                        */
  __IO uint32_t TCR;             /*!< 0x00C USB Endpoint n Transfer Count Register                          */
  __IO uint32_t CFGR;            /*!< 0x010 USB Endpoint n Configuration Register                           */
} USBEP_TypeDef;


/**
  * @}
  */


/** @addtogroup Peripheral_Memory_Map
  * @{
  */

#define SRAM_BASE             ((u32)0x20000000)
#define SRAM_BB_BASE          ((u32)0x22000000)

#define PERIPH_BASE           ((u32)0x40000000)
#define PERIPH_BB_BASE        ((u32)0x42000000)

#define APB0PERIPH_BASE       PERIPH_BASE                   /* 0x40000000                                   */
#define APB1PERIPH_BASE       (PERIPH_BASE + 0x40000)       /* 0x40040000                                   */
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x80000)       /* 0x40080000                                   */

/* APB0                                                                                                     */
#define USART0_BASE           (APB0PERIPH_BASE + 0x0000)    /* 0x40000000                                   */
#define SPI0_BASE             (APB0PERIPH_BASE + 0x4000)    /* 0x40004000                                   */
#define ADC_BASE              (APB0PERIPH_BASE + 0x10000)   /* 0x40010000                                   */
#define CMP_OP0_BASE          (APB0PERIPH_BASE + 0x18000)   /* 0x40018000                                   */
#define CMP_OP1_BASE          (APB0PERIPH_BASE + 0x18100)   /* 0x40018100                                   */
#define GPIOA_BASE            (APB0PERIPH_BASE + 0x1A000)   /* 0x4001A000                                   */
#define GPIOB_BASE            (APB0PERIPH_BASE + 0x1B000)   /* 0x4001B000                                   */
#define GPIOC_BASE            (APB0PERIPH_BASE + 0x1C000)   /* 0x4001C000                                   */
#define GPIOD_BASE            (APB0PERIPH_BASE + 0x1D000)   /* 0x4001D000                                   */
#define GPIOE_BASE            (APB0PERIPH_BASE + 0x1E000)   /* 0x4001E000                                   */
#define AFIO_BASE             (APB0PERIPH_BASE + 0x22000)   /* 0x40022000                                   */
#define EXTI_BASE             (APB0PERIPH_BASE + 0x24000)   /* 0x40024000                                   */
#define MCTM_BASE             (APB0PERIPH_BASE + 0x2C000)   /* 0x4002C000                                   */

/* APB1                                                                                                     */
#define USART1_BASE           (APB1PERIPH_BASE + 0x0000)    /* 0x40040000                                   */
#define SCI_BASE              (APB1PERIPH_BASE + 0x3000)    /* 0x40043000                                   */
#define SPI1_BASE             (APB1PERIPH_BASE + 0x4000)    /* 0x40044000                                   */
#define I2C0_BASE             (APB1PERIPH_BASE + 0x8000)    /* 0x40048000                                   */
#define I2C1_BASE             (APB1PERIPH_BASE + 0x9000)    /* 0x40049000                                   */
#define USB_BASE              (APB1PERIPH_BASE + 0xE000)    /* 0x4004E000                                   */
#define USB_EP0_BASE          (USB_BASE        + 0x0014)    /* 0x4004E014                                   */
#define USB_EP1_BASE          (USB_BASE        + 0x0028)    /* 0x4004E028                                   */
#define USB_EP2_BASE          (USB_BASE        + 0x003C)    /* 0x4004E03C                                   */
#define USB_EP3_BASE          (USB_BASE        + 0x0050)    /* 0x4004E050                                   */
#define USB_EP4_BASE          (USB_BASE        + 0x0064)    /* 0x4004E064                                   */
#define USB_EP5_BASE          (USB_BASE        + 0x0078)    /* 0x4004E078                                   */
#define USB_EP6_BASE          (USB_BASE        + 0x008C)    /* 0x4004E08C                                   */
#define USB_EP7_BASE          (USB_BASE        + 0x00A0)    /* 0x4004E0A0                                   */
#define USB_SRAM_BASE         (APB1PERIPH_BASE + 0xE400)    /* 0x4004E400                                   */
#define WDT_BASE              (APB1PERIPH_BASE + 0x28000)   /* 0x40068000                                   */
#define RTC_BASE              (APB1PERIPH_BASE + 0x2A000)   /* 0x4006A000                                   */
#define PWRCU_BASE            (APB1PERIPH_BASE + 0x2A100)   /* 0x4006A100                                   */
#define GPTM0_BASE            (APB1PERIPH_BASE + 0x2E000)   /* 0x4006E000                                   */
#define GPTM1_BASE            (APB1PERIPH_BASE + 0x2F000)   /* 0x4006F000                                   */
#define BFTM0_BASE            (APB1PERIPH_BASE + 0x36000)   /* 0x40076000                                   */
#define BFTM1_BASE            (APB1PERIPH_BASE + 0x37000)   /* 0x40077000                                   */

/* AHB                                                                                                      */
#define FLASH_BASE            (AHBPERIPH_BASE + 0x0000)     /* 0x40080000                                   */
#define CKCU_BASE             (AHBPERIPH_BASE + 0x8000)     /* 0x40088000                                   */
#define RSTCU_BASE            (AHBPERIPH_BASE + 0x8100)     /* 0x40088100                                   */
#define PDMA_BASE             (AHBPERIPH_BASE + 0x10000)    /* 0x40090000                                   */
#define CSIF_BASE             (AHBPERIPH_BASE + 0x4C000)    /* 0x400CC000                                   */



/**
  * @}
  */

/* Peripheral declaration                                                                                   */
#define USART0              ((USART_TypeDef *) USART0_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define SPI0                ((SPI_TypeDef *) SPI0_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define ADC                 ((ADC_TypeDef *) ADC_BASE)
#define CMP_OP0             ((CMP_OP_TypeDef *) CMP_OP0_BASE)
#define CMP_OP1             ((CMP_OP_TypeDef *) CMP_OP1_BASE)
#define AFIO                ((AFIO_TypeDef *) AFIO_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define MCTM                ((MCTM_TypeDef *) MCTM_BASE)
#define SCI                 ((SCI_TypeDef *) SCI_BASE)
#define I2C0                ((I2C_TypeDef *) I2C0_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define USB                 ((USB_TypeDef *) USB_BASE)
#define USBEP0              ((USBEP_TypeDef *) USB_EP0_BASE)
#define USBEP1              ((USBEP_TypeDef *) USB_EP1_BASE)
#define USBEP2              ((USBEP_TypeDef *) USB_EP2_BASE)
#define USBEP3              ((USBEP_TypeDef *) USB_EP3_BASE)
#define USBEP4              ((USBEP_TypeDef *) USB_EP4_BASE)
#define USBEP5              ((USBEP_TypeDef *) USB_EP5_BASE)
#define USBEP6              ((USBEP_TypeDef *) USB_EP6_BASE)
#define USBEP7              ((USBEP_TypeDef *) USB_EP7_BASE)
#define WDT                 ((WDT_TypeDef *) WDT_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define PWRCU               ((PWRCU_TypeDef *) PWRCU_BASE)
#define GPTM0               ((GPTM_TypeDef *) GPTM0_BASE)
#define GPTM1               ((GPTM_TypeDef *) GPTM1_BASE)
#define BFTM0               ((BFTM_TypeDef *) BFTM0_BASE)
#define BFTM1               ((BFTM_TypeDef *) BFTM1_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_BASE)
#define CKCU                ((CKCU_TypeDef *) CKCU_BASE)
#define RSTCU               ((RSTCU_TypeDef *) RSTCU_BASE)
#define PDMA                ((PDMA_TypeDef *) PDMA_BASE)
#define CSIF                ((CSIF_TypeDef *) CSIF_BASE)


#if defined  USE_HT32_DRIVER
  #include "ht32f175x_275x_lib.h"
#endif


/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __HT32F175x_275x_H__                                                                                  */

/**
  * @}
  */

  /**
  * @}
  */

/******************* (C) COPYRIGHT 2011 Holtek Semiconductor Inc. *****END OF FILE***                       */
