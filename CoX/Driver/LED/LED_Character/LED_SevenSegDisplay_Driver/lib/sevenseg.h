//*****************************************************************************
//
//! \file sevenseg.h
//! \brief Prototypes for Seven Segment Driver.
//! \version 1.0.0
//! \date 10/18/2011
//! \author CooCox
//! \copy
//!
//! Copyright (c)  2011, CooCox 
//! All rights reserved.
//! 
//! Redistribution and use in source and binary forms, with or without 
//! modification, are permitted provided that the following conditions 
//! are met: 
//! 
//!     * Redistributions of source code must retain the above copyright 
//! notice, this list of conditions and the following disclaimer. 
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution. 
//!     * Neither the name of the <ORGANIZATION> nor the names of its 
//! contributors may be used to endorse or promote products derived 
//! from this software without specific prior written permission. 
//! 
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#ifndef __SEVENSEG_H__
#define __SEVENSEG_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup COX_Driver_Lib
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Led
//! @{
//
//*****************************************************************************
    
//*****************************************************************************
//
//! \addtogroup Led_Character
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Seven_Segment
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Seven_Segment_Config_Pins Seven Segment Pin Configurtion
//!
//! \brief GPIO Pins Configurtions that communication with the Seven Segment  
//!  should be set before using this driver. Here, we define a virtual seven 
//!  segment which is portable. If you want to use it, you must config each 
//!  segment to the suitable pin for displaying rightly. Each segment mark on 
//!  Led show as:
//!                        A
//!                    +-------+
//!                  F |       | B
//!                    |   G   |
//!                    +-------+
//!                  E |       | C
//!                    |       | 
//!                    +-------+ . H
//!                        D
//!
//! @{
//
//*****************************************************************************

#define LED_SEG_A                  PE3
#define LED_SEG_B                  PE4
#define LED_SEG_C                  PE0
#define LED_SEG_D                  PE5
#define LED_SEG_E                  PE6
#define LED_SEG_F                  PE2
#define LED_SEG_G                  PE7
#define LED_SEG_H                  PE1

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Seven Segment_Led_Select_Config_Pins Seven Segment Led Select  
//!  Pin Configuration
//!
//! \brief GPIO Pins Configurtions that communication with the selected Seven   
//!  Segment Led should be set before using this driver.
//!
//! @{
//
//*****************************************************************************

#define LED1_PIN             PC4
#define LED2_PIN             PC5
#define LED3_PIN             PC6
#define LED4_PIN             PC7

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Seven_Segment_Led_Config_Moudle Seven Segment Led Moudle 
//!  Configuration
//!
//! \brief Seven Segment Led Moudle Configurtions that communication with the    
//!  selected Seven Segment should be set before using this driver.
//!
//! @{
//
//*****************************************************************************

//
//! The number of Seven Segment Leds, can be 8, 4, 2 or 1.
//
#define LED_NUM               4

//
//! The type of Seven Segment Led' Compolar, can be anode or cathode.
//
#define LED_COM_POLAR         LED_COM_ANODE

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup Seven_Segment_Led_Config_Function Seven Segment Led Function 
//!  Configuration
//!
//! \brief Configure Seven Segment common polar
//!
//! @{
//
//*****************************************************************************

//
//! The type of Seven Segment Led's Common Polar.
//!
//! Can be set to:
//! - \ref LED_COM_ANODE - Led's common polar is anode
//! - \ref LED_COM_CATHODE - Led's common polar is cathode
//! .
//! More info reference \ref LED_COM_POLAR
//
#define LED_COM_ANODE         1
#define LED_COM_CATHODE       0

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! Configuration about timer used to scan seven segment leds.
//
//*****************************************************************************
//
//! Set timer's timeout frequency. 
//
#define TIMER_FREQ    500
//
//! Select timer which you want to use.
//
#define TIMERX        0
#define TIMERX_BASE  xTIMER0_BASE

//*****************************************************************************
//
//! \addtogroup SevenSeg_Exported_APIs  Seven Segment Driver APIs
//! \brief API Refrence of Seven Segment Driver.
//! @{
//
//*****************************************************************************
extern void LedInit(void);
extern void LedOn(unsigned long ulLed);
extern void LedOff(unsigned long ulLed);
extern void LedBlinkEnable(unsigned long ulLed);
extern void LedBlinkDisable(unsigned long ulLed);
extern void LedBlinkFreqSet(unsigned long ulFreq);
extern void LedCharSet(unsigned long ulLed, char ch);
extern void LedStringSet(unsigned long ulStart, unsigned long ulLen,
                                                        const char* pcString);

extern void LedBlinkSet(unsigned long ulLed, unsigned long ulFreq);
extern void LedDisplay(unsigned long ulLed, const char* pcString);
extern void LedNumDisplay(unsigned long ulLed, long Val);
extern void LedFloatDisplay(unsigned long ulLed, double Val);
extern void LedConfig(unsigned long ulLed, unsigned long ulBlinkLed,
                              unsigned long ulBlinkFreq, const char* pcString);

extern void CharTabAdd(char ch, unsigned char ucCode);

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
//! @}
//
//*****************************************************************************

#endif //__SEVENLED_SEG_H__
