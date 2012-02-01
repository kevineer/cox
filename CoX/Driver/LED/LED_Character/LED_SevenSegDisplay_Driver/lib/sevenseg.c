//*****************************************************************************
//
//! \file sevenseg.c
//! \brief Driver for Character LED Seven Segment.
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

#include "xhw_types.h"
#include "xhw_memmap.h"
#include "xhw_ints.h"
#include "xcore.h"
#include "xdebug.h"
#include "xsysctl.h"
#include "xgpio.h"
#include "xtimer.h"
#include "sevenseg.h"
#include "hw_sevenseg.h"

//*****************************************************************************
//
//! Define a few macros and global variables. They may be used in some APIs,  
//! sunch as StringDisplay(), LedNumDisplay(), LedConfig() and so on.
//
//*****************************************************************************
//
// Set string buffer size
//
#define STRING_SIZE  50
//
// Set char segment code table size
//
#define CHAR_NUM   80

unsigned long guLed = 0;
unsigned long guBlinkLed = 0;
unsigned long guBlinkFreq = 0;

//
// Save string you want to display
//
char gpcString[STRING_SIZE];

//
// Define char segment code table for seven segment led character displaying.
//
typedef struct
{
    unsigned char ch;
    unsigned char ucCode;
} SegCode;

#if (LED_COM_POLAR == LED_COM_ANODE)
SegCode SegCodeTab[CHAR_NUM] = {{'0',0xC0}, {'1',0xF9}, {'2',0xA4}, {'3',0xB0},
        {'4',0x99}, {'5',0x92}, {'6',0x82}, {'7',0xF8}, {'8',0x80}, {'9',0x98},
        {'A',0x88}, {'B',0x83}, {'b',0x83}, {'C',0xC6}, {'c',0xA7}, {'D',0xC0},
        {'d',0xA1}, {'E',0x86}, {'e',0x84}, {'F',0x8E}, {'G',0xC2}, {'g',0x90},
        {'H',0x89}, {'h',0x8B}, {'I',0xCF}, {'i',0xE6}, {'J',0xE1}, {'j',0xF2},
        {'N',0xC8}, {'n',0xAB}, {'P',0x8C}, {'p',0x8C}, {'T',0xCE}, {'t',0x8F},
        {'U',0xC1}, {'u',0xE3}, {'X',0x89}, {'y',0x91}, {'-',0xBF}, {'.',0x7F}};
#elif (LED_COM_POLAR == LED_COM_CATHODE)
SegCode SegCodeTab[CHAR_NUM] = {{'0',0x3F}, {'1',0x06}, {'2',0x5B}, {'3',0x4F},
        {'4',0x66}, {'5',0x6D}, {'6',0x7D}, {'7',0x07}, {'8',0x7F}, {'9',0x67},
        {'A',0x77}, {'B',0x7F}, {'b',0x7C}, {'C',0x39}, {'c',0x58}, {'D',0x3F},
        {'d',0x5E}, {'E',0x79}, {'e',0x7B}, {'F',0x71}, {'G',0x3D}, {'g',0x6F},
        {'H',0x76}, {'h',0x74}, {'I',0x30}, {'i',0x19}, {'J',0x1E}, {'j',0x0D},
        {'N',0x37}, {'n',0x54}, {'P',0x73}, {'p',0x73}, {'T',0x31}, {'t',0x70},
        {'U',0x3E}, {'u',0x1C}, {'X',0x76}, {'y',0x6E}, {'-',0x40}, {'.',0x80}};
#endif

//
// Indicate the current position of scaned character in string 
//
static unsigned long guptStrPos = 0;

//
// Indicate the current position of scaned seven segment led
//
static unsigned long guptLedPos = 0;

//
// It is used to control each led's status(on or off) change time. Each element 
// saves the blink led's delay count value. When it gradually increases from 0
// to BlinkFreq, current scanned led's status will chang from off to on. And  
// when it increases from BlinkFreq to 2 * BlinkFreq, current scanned led's  
// status will chang from on to off.
//
static unsigned long guDelayCount[LED_NUM] = {0,0,0,0};

//
// Save character displayed on each led on.
//
static char a[LED_NUM] = {0,0,0,0};

//
// The length of string to be displayed
//
unsigned long guStrLen = 0;

//
// The number of seven segment led on
//
unsigned long guLedOnNum = 0;

//
// Used when dot is included in string to be displayed
//
static xtBoolean bDotUse = 0;

//*****************************************************************************
//
//! Static function protype.
//
//*****************************************************************************
static unsigned long Open(unsigned long ulLed);
static void Close(unsigned long ulLed);
static void CharDisplay(unsigned long ulLed, unsigned char ch);
static void StringDisplay(void);

static unsigned long xtTimerCallback(void *pvCBData, unsigned long ulEvent,
                                     unsigned long ulMsgParam, void *pvMsgData);

//
// Timer interrpt callback function.
//
static unsigned long
xtTimerCallback(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam,
                void *pvMsgData )
{
    //
    // Display string when timer interrupt occurs.
    //
    StringDisplay();

    return 0;
}

//*****************************************************************************
//
//! \brief Init Seven Segment Led device.
//!
//! \param None
//!
//! This function is used to init seven segment led device. It must be called 
//! before any other seven segment function is used. 
//!
//! It opens the pins's GPIO peripheral port, and configs the pins type to GPIO 
//! output. Then configs the seven segment led into the default state, clear the
//! seven segment leds. Default seven segment led's compolar is anode, number
//! is 4. At last, it configs timer and use it to scan seven segment leds.
//! .
//!
//! \return None
//
//*****************************************************************************
void
LedInit()
{
    xtBoolean bSegOff;
    xtBoolean bLedOff;
    
#if (LED_COM_POLAR == LED_COM_ANODE)
    bSegOff = 1;
    bLedOff = 0;
#elif (LED_COM_POLAR == LED_COM_CATHODE)
    bSegOff = 0;
    bLedOff = 1;
#endif
    
    //
    // Set pins type to GPIO output. 
    //
#if (LED_NUM == 4)
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED1_PIN));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED2_PIN));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED3_PIN));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED4_PIN));
    xGPIOSPinTypeGPIOOutput(LED1_PIN);
    xGPIOSPinTypeGPIOOutput(LED2_PIN);
    xGPIOSPinTypeGPIOOutput(LED3_PIN);
    xGPIOSPinTypeGPIOOutput(LED4_PIN);
    xGPIOSPinWrite(LED1_PIN, bLedOff);
    xGPIOSPinWrite(LED2_PIN, bLedOff);
    xGPIOSPinWrite(LED3_PIN, bLedOff);
    xGPIOSPinWrite(LED4_PIN, bLedOff);
#elif (LED_NUM == 2)
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED1_PIN));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED2_PIN));
    xGPIOSPinTypeGPIOOutput(LED1_PIN);
    xGPIOSPinTypeGPIOOutput(LED2_PIN);
    xGPIOSPinWrite(LED1_PIN, bLedOff);
    xGPIOSPinWrite(LED2_PIN, bLedOff);
#endif
    
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED_SEG_A));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED_SEG_B));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED_SEG_C));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED_SEG_D));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED_SEG_E));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED_SEG_F));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED_SEG_G));
    xSysCtlPeripheralEnable(xGPIOSPinToPeripheralId(LED_SEG_H));
    xGPIOSPinTypeGPIOOutput(LED_SEG_A);
    xGPIOSPinTypeGPIOOutput(LED_SEG_B);
    xGPIOSPinTypeGPIOOutput(LED_SEG_C);
    xGPIOSPinTypeGPIOOutput(LED_SEG_D);
    xGPIOSPinTypeGPIOOutput(LED_SEG_E);
    xGPIOSPinTypeGPIOOutput(LED_SEG_F);
    xGPIOSPinTypeGPIOOutput(LED_SEG_G);
    xGPIOSPinTypeGPIOOutput(LED_SEG_H);
    
    xGPIOSPinWrite(LED_SEG_A, bSegOff);
    xGPIOSPinWrite(LED_SEG_B, bSegOff);
    xGPIOSPinWrite(LED_SEG_C, bSegOff);
    xGPIOSPinWrite(LED_SEG_D, bSegOff);
    xGPIOSPinWrite(LED_SEG_E, bSegOff);
    xGPIOSPinWrite(LED_SEG_F, bSegOff);
    xGPIOSPinWrite(LED_SEG_G, bSegOff);
    xGPIOSPinWrite(LED_SEG_H, bSegOff);
    
    //
    // Set system clock
    //
    xSysCtlClockSet(12000000, xSYSCTL_XTAL_12MHZ | xSYSCTL_OSC_MAIN);

    //
    // Config and start timer to scan seven segment leds
    //
    switch (TIMERX)
    {
    case 0:      
        xSysCtlPeripheralClockSourceSet(xSYSCTL_TIMER0_MAIN, 1);
        xSysCtlPeripheralEnable(xSYSCTL_PERIPH_TIMER0);
        xSPinTypeTimer(TIMCCP0, PB8);
        xTimerInitConfig(xTIMER0_BASE, 0, TIMER_MODE_PERIODIC, TIMER_FREQ);
        xTimerIntCallbackInit(xTIMER0_BASE, xtTimerCallback);
        xTimerIntEnable(xTIMER0_BASE, 0, TIMER_INT_MATCH);
        xIntEnable(xINT_TIMER0);
        xTimerStart(xTIMER0_BASE, 0);
        break;
    case 1:
        xSysCtlPeripheralClockSourceSet(xSYSCTL_TIMER1_MAIN, 1);
        xSysCtlPeripheralEnable(xSYSCTL_PERIPH_TIMER1);
        xSPinTypeTimer(TIMCCP1, PB9);
        xTimerInitConfig(xTIMER1_BASE, 0, TIMER_MODE_PERIODIC, TIMER_FREQ);
        xTimerIntCallbackInit(xTIMER1_BASE, xtTimerCallback);
        xTimerIntEnable(xTIMER1_BASE, 0, TIMER_INT_MATCH);
        xIntEnable(xINT_TIMER1);
        xTimerStart(xTIMER1_BASE, 0);
        break;
    case 2:
        xSysCtlPeripheralClockSourceSet(xSYSCTL_TIMER2_MAIN, 1);
        xSysCtlPeripheralEnable(xSYSCTL_PERIPH_TIMER2);
        xSPinTypeTimer(TIMCCP2, PB10);
        xTimerInitConfig(xTIMER2_BASE, 0, TIMER_MODE_PERIODIC, TIMER_FREQ);
        xTimerIntCallbackInit(xTIMER2_BASE, xtTimerCallback);
        xTimerIntEnable(xTIMER2_BASE, 0, TIMER_INT_MATCH);
        xIntEnable(xINT_TIMER2);
        xTimerStart(xTIMER2_BASE, 0);
        break;
    case 3:      
        xSysCtlPeripheralClockSourceSet(xSYSCTL_TIMER3_MAIN, 1);
        xSysCtlPeripheralEnable(xSYSCTL_PERIPH_TIMER3);
        xSPinTypeTimer(TIMCCP3, PB11);
        xTimerInitConfig(xTIMER3_BASE, 0, TIMER_MODE_PERIODIC, TIMER_FREQ);
        xTimerIntCallbackInit(xTIMER3_BASE, xtTimerCallback);
        xTimerIntEnable(xTIMER3_BASE, 0, TIMER_INT_MATCH);
        xIntEnable(xINT_TIMER3);
        xTimerStart(xTIMER3_BASE, 0);
        break;
    }
}

//*****************************************************************************
//
//! \brief Open the specified seven segment leds.
//!
//! \param ulLed The seven segment leds to be opened. It is specified using a
//!  bit-packed byte, where each bit which is set identifies the led to be 
//!  opened, and where bit 0 of the byte represents LED1, bit 1 represents 
//!  LED2, and so on. 
//!
//! \return The number of leds opened. It should be less than LED_NUM.
//
//!**************************************************************************** 
unsigned long 
Open(unsigned long ulLed)
{
    xtBoolean bLedOn;
    unsigned long ulCount = 0;
    
#if (LED_COM_POLAR == LED_COM_ANODE)
    bLedOn = 1;
#elif (LED_COM_POLAR == LED_COM_CATHODE)
    bLedOn = 0;
#endif
    
    //
    // Open the specified leds
    //
    if (ulLed & LED1)
    {
        xGPIOSPinWrite(LED1_PIN, bLedOn);
        ulCount++;
    }
    
    if (ulLed & LED2)
    {
        xGPIOSPinWrite(LED2_PIN, bLedOn);
        ulCount++;
    }
    
    if (ulLed & LED3)
    {
        xGPIOSPinWrite(LED3_PIN, bLedOn);
        ulCount++;
    }
    
    if (ulLed & LED4)
    {
        xGPIOSPinWrite(LED4_PIN, bLedOn);
        ulCount++;
    }
    
    return ulCount;
}

//*****************************************************************************
//
//! \brief Close the specified seven segment leds.
//!
//! \param ulLed The seven segment leds to be closed. It is specified using a
//!  bit-packed byte, where each bit which is set identifies the leds to be
//!  opened, and where bit 0 of the byte represents LED1, bit 1 represents LED2,
//!  and so on. 
//!
//! \return None
//
//!****************************************************************************
void
Close(unsigned long ulLed)
{
    xtBoolean bLedOff;
    
#if (LED_COM_POLAR == LED_COM_ANODE)
    bLedOff = 0;
#elif (LED_COM_POLAR == LED_COM_CATHODE)
    bLedOff = 1;
#endif
    
    //
    // Close the specified leds
    //
    if (ulLed & LED1)
    {
        xGPIOSPinWrite(LED1_PIN, bLedOff);
    }
    
    if (ulLed & LED2)
    {
        xGPIOSPinWrite(LED2_PIN, bLedOff);
    }
    
    if (ulLed & LED3)
    {
        xGPIOSPinWrite(LED3_PIN, bLedOff);
    }
    
    if (ulLed & LED4)
    {
        xGPIOSPinWrite(LED4_PIN, bLedOff);
    }
}

//*****************************************************************************
//
//! \breif Display char in the specified seven segment leds.
//!
//! \param ulLed The specified seven segment leds
//!
//! \param ch The character to be displayed
//!
//! \return None
//
//*****************************************************************************
void 
CharDisplay(unsigned long ulLed, unsigned char ch)
{
    int i = 0; 

    Open(ulLed);
    
    //
    // Find char table and set gpio pin according to segment code
    //
    for (i = 0; i < CHAR_NUM; i++)
    {
        if (ch == SegCodeTab[i].ch)
        {
            xGPIOSPinWrite(LED_SEG_A, (SegCodeTab[i].ucCode >> 0) & 0x01);
            xGPIOSPinWrite(LED_SEG_B, (SegCodeTab[i].ucCode >> 1) & 0x01);
            xGPIOSPinWrite(LED_SEG_C, (SegCodeTab[i].ucCode >> 2) & 0x01);
            xGPIOSPinWrite(LED_SEG_D, (SegCodeTab[i].ucCode >> 3) & 0x01);
            xGPIOSPinWrite(LED_SEG_E, (SegCodeTab[i].ucCode >> 4) & 0x01);
            xGPIOSPinWrite(LED_SEG_F, (SegCodeTab[i].ucCode >> 5) & 0x01);
            xGPIOSPinWrite(LED_SEG_G, (SegCodeTab[i].ucCode >> 6) & 0x01);
            xGPIOSPinWrite(LED_SEG_H, (SegCodeTab[i].ucCode >> 7) & 0x01);
            break;
        }
    }

    xASSERT(i < CHAR_NUM);
}

//*****************************************************************************
//
//! \brief Display string in the specified seven segment leds and blink a few
//!  opened leds at a rate. 
//! 
//! \param None
//!
//! \return None
//
//*****************************************************************************
void 
StringDisplay()
{  
    int uBitScan;
    int uBlinkDelay;

    //
    // Set blink delay time according to blink level
    //
    switch(guBlinkFreq)
    {
    case 0:
        uBlinkDelay = 5;
        break;
    case 1:
        uBlinkDelay = 10;
        break;
    case 2:
        uBlinkDelay = 40;
        break;
    }
    
    guptLedPos %= LED_NUM;
    uBitScan = guLed & (0x01 << (LED_NUM - guptLedPos - 1));
    
    if (uBitScan != 0)
    {
        guptStrPos %= guLedOnNum;
        if (guptStrPos >= guStrLen)
        {
            //a[guptLedPos] = 0;
            Close(guLed);
        }
        else
        {
            if (guBlinkLed & (0x01 << (LED_NUM - guptLedPos - 1)))
            {   
                if (guDelayCount[guptLedPos] < uBlinkDelay)
                {
                    Close(guLed);
                    guDelayCount[guptLedPos]++;
                }
                else
                {
                    guDelayCount[guptLedPos]++;
                    guDelayCount[guptLedPos] %= (2 * uBlinkDelay);
                    Close(guLed);
                    CharDisplay(uBitScan, gpcString[guptStrPos]);
                }
            }
            else
            {
                Close(guLed);
                CharDisplay(uBitScan, gpcString[guptStrPos]);
            }

            if (gpcString[guptStrPos] == '.' && bDotUse == 0)
            {
                bDotUse = 1;
                guLedOnNum++;
            }

            a[guptLedPos] = gpcString[guptStrPos];
        }

        guptStrPos++;
    }
    else 
    {
        a[guptLedPos] = 0;
        Close(guLed);
    }

    guptLedPos++;
    if (gpcString[guptStrPos] == '.')
    {
        guptLedPos--;
    }
}

//*****************************************************************************
//
//! \brief Open the specified seven segment leds.
//!
//! \param ulLed Leds to be opened.
//!
//! This function is used to open the specified seven segment leds.
//! It must be called after LedInit(). 
//! .
//!
//! \return None
//!
//
//*****************************************************************************
void 
LedOn(unsigned long ulLed)
{  
    xASSERT(ulLed && (LED1 | LED2 | LED3 | LED4));

    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);

    //
    // Clear old configuration
    //
    Close(guLed);
    guptStrPos = 0;
    guptLedPos = 0;

    //
    // Specified new Leds to be opened
    //
    guLed |= ulLed;

    //
    // Restart to calculate the number of opened seven segment leds
    //
    guLedOnNum = Open(guLed);
    Close(guLed);

    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Close specified seven segment leds.
//!
//! \param ulLed Leds to be closed.
//!
//! This function is used to close the specified seven segment leds.
//! It must be called after LedInit(). 
//! .
//!
//! \return None
//!
//
//*****************************************************************************
void 
LedOff(unsigned long ulLed)
{   
    xASSERT(ulLed && (LED1 | LED2 | LED3 | LED4));

    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);

    //
    // Clear old configuration
    //
    Close(guLed);
    guptStrPos = 0;
    guptLedPos = 0;

    //
    // Specified new Leds to be opened
    //
    guLed &= ~ulLed;

    //
    // Restart to calculate the number of opened seven segment leds
    //
    guLedOnNum = Open(guLed);
    Close(guLed);
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Enable Leds blink.
//!
//! \param ulLed Leds to blink.
//!  
//! This function is used to blink character with default blink frequency level 
//! 0 on the specified seven segment leds. It must be called after LedInit().
//!
//! \return None
//!
//
//*****************************************************************************
void 
LedBlinkEnable(unsigned long ulLed)
{
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    
    //
    // Choose specified Led to blink
    //
    guBlinkLed |= ulLed;
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Disable Leds blink.
//!
//! \param ulLed Leds to stop blinking.
//!  
//! This function is used to stop blinking string on the specified seven segment 
//! leds. It must be called after LedInit() and LedDisplay().
//! .
//!
//! \return None
//!
//
//*****************************************************************************
void 
LedBlinkDisable(unsigned long ulLed)
{
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    //
    // Choose specified Led not to blink
    //
    guBlinkLed &= ~ulLed;
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Set seven segment leds blink frequency.
//!
//! \param ulFreq Blink frequency for Leds.
//!  
//! This function is used to set blink frequency on the specified seven segment
//! leds. It must be called after LedInit().
//!
//! \return None
//!
//
//*****************************************************************************
void 
LedBlinkFreqSet(unsigned long ulFreq)
{
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    //
    // Choose specified Led not to blink
    //
    guBlinkFreq = ulFreq;
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Modify a character on a seven segment led.
//!
//! \param ulLed Leds to be modified.
//!
//! \param ch character to be displayed on a Led.
//!
//! This function is used to modify character on the specified seven segment 
//! leds. It must be called after LedInit().
//! .
//!
//! \return None
//!
//
//*****************************************************************************
void 
LedCharSet(unsigned long ulLed, char ch)
{
    int i = 0;
    int j = 0;
    
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    
    while (((ulLed >> i) & 0x1) == 0)
    {
        i++;
    }
    
    a[LED_NUM - i - 1] = ch;
    for (i = 0, j = 0; i < LED_NUM; i++,j++)
    {
        if (a[i] != 0)
        {
            gpcString[j] = a[i];
        }
    }
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);       
}

//*****************************************************************************
//
//! \brief Modify characters on seven segment leds.
//!
//! \param ulStart The start palce of Leds to be modified.
//!
//! \param ulLen The length of Leds to be modified.
//!
//! \param pcString characters to be displayed on Leds.
//!
//! This function is used to modify characters on the specified seven segment
//! leds. It must be called after InitLed().
//! .
//!
//! \return None
//!
//
//*****************************************************************************
void 
LedStringSet(unsigned long ulStart, unsigned long ulLen, const char* pcString)
{
    int i = 0;
    int j = 0;
    
    xASSERT((ulStart + ulLen) <= LED_NUM);

    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    
    while (i < ulLen && pcString != 0)
    {
        a[ulStart + i] = *pcString;
        pcString++;
        i++;
    }
    
    for (i = 0, j = 0; i < LED_NUM; i++,j++)
    {
        if (a[i] != 0)
        {
            gpcString[j] = a[i];
        }
    }
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Blink string on seven segment leds.
//!
//! \param ulLed Leds to blink
//!
//! \param ulFreq Blink level. It can be 0, 1, 2. Generally level 0 is fast, 
//!  level 1 is medium, level 2 is slow.
//!  
//! This function is used to blink string on the specified seven segment leds.
//! It must be called after LedInit() and LedDisplay().
//! .
//!
//! \return None
//!
//
//*****************************************************************************
void 
LedBlinkSet(unsigned long ulLed, unsigned long ulFreq)
{
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    
    //
    // Reset blinking leds and frequency
    //
    guBlinkLed = ulLed;
    guBlinkFreq = ulFreq;
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Display string on seven segment leds.
//!
//! \param ulLed Leds used to display string.
//!
//! \param pcString String to be displayed on the seven segment leds. 
//!
//! This function is used to display string on the specified seven segment leds.
//! It must be called after LedInit(). 
//! .
//!
//! \return None
//!
//! \note We advise that the number of Leds opened should larger than or equal 
//!  to the length of string. And when the number of Leds opened is less than
//!  string's length, the exceeded part will be ignored.
//
//*****************************************************************************
void 
LedDisplay(unsigned long ulLed, const char* pcString)
{
    int i = 0;
    
    xASSERT(ulLed && (LED1 | LED2 | LED3 | LED4));
    xASSERT(pcString != 0);
    
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    
    //
    // Clear old configuration
    //
    Close(guLed);
    guStrLen = 0;
    guptStrPos = 0;
    guptLedPos = 0;
    bDotUse = 0;
    for (i = 0; i < LED_NUM; i++)
    {
        guDelayCount[i] = 0;
    }

    //
    // Specified new Leds to be opened
    //
    guLed = ulLed;

    //
    // Restart to calculate the number of opened seven segment leds
    //
    guLedOnNum = Open(guLed);
    Close(guLed);
    
    //
    // Input string to be displayed and get the length of string
    //
    i = 0;
    while (*pcString)
    {
        gpcString[i++] = *pcString++;      
    }
    guStrLen = i;
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Display integer numeric on seven segment leds.
//!
//! \param ulLed Leds used to display integer numeric.
//!
//! \param Val Integer numeric to be displayed on the seven segment leds.
//!
//! This function is used to display integer numeric on the specified seven 
//! segment leds. It must be called after LedInit(). 
//! .
//!
//! \return None
//!
//! \note We advise that the number of Leds opened should larger than or equal 
//!  to the bits of integer numeric. And when the number of Leds opened is less  
//!  than numeric's bits, the exceeded part will be ignored.
//
//*****************************************************************************
void 
LedNumDisplay(unsigned long ulLed, long Val)
{
    //
    // Check if Val is positive or negative
    //
    xtBoolean bSign = 0;
    int i = 0;
    int j = 0;
    //
    // The number of leds used to display numeric(not include sign '+' or '-').
    //
    int LedOnNum = 0;
    int tmp = 1;
    char IntBuf[LED_NUM];
    
    xASSERT(ulLed && (LED1 | LED2 | LED3 | LED4));
    
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    
    //
    // Clear old configuration
    //
    Close(guLed);
    guStrLen = 0;
    guptStrPos = 0;
    guptLedPos = 0;
    bDotUse = 0;
    for (i = 0; i < LED_NUM; i++)
    {
        guDelayCount[i] = 0;
    }
    
    //
    // Specified new Leds to be opened
    //
    guLed = ulLed;

    //
    // Restart to calculate the number of opened seven segment leds
    //
    guLedOnNum = Open(guLed);
    Close(guLed);
    
    //
    // Get current number of leds used to display numeric(not include sign '+'
    //  or '-'). And if it is negative, convert it to positive.
    //
    if (Val < 0)
    {
        LedOnNum = guLedOnNum - 1;
        Val = 0 - Val;
        bSign = 1;
    }
    else
    {
        LedOnNum = guLedOnNum;
    }
    
    //
    // Get the max value which leds can display
    //
    i = 0;
    while (i < LedOnNum)
    {
        tmp *= 10;
        i++;
    }
    
    //
    // Check if the value to be displayed is valid
    //
    xASSERT(Val < tmp);
    
    //
    // Convert the value to string.
    //
    i = LED_NUM - 1;
    if (Val == 0) 
    {
        IntBuf[i--] = '0';
    }
    else
    {
        while (Val != 0)
        {
            tmp = Val % 10;
            IntBuf[i--] = tmp + '0';
            Val /= 10;
        }
    }
    
    if (bSign == 1)
    {
        gpcString[j++] = '-';
    }
    i = i + 1;
    while (i < LED_NUM)
    {
        gpcString[j++] = IntBuf[i++];
    }
    guStrLen = j;
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Display float numeric on seven segment leds.
//!
//! \param ulLed Leds used to display float numeric.
//!
//! \param Val Float numeric to be displayed on the seven segment leds.
//!
//! This function is used to display float numeric on the specified seven 
//! segment leds. It must be called after LedInit(). 
//! .
//!
//! \return None
//!
//! \note We advise that the number of Leds opened should larger than or equal 
//!  to the bits of float numeric. And when the number of Leds opened is less  
//!  than numeric's bits, the exceeded part will be ignored. At last, it may 
//!  lose precision when display float numeric.
//
//*****************************************************************************
void 
LedFloatDisplay(unsigned long ulLed, double Val)
{
    xtBoolean bSign = 0;
    int i = 0;
    int j = 0;
    int k = 0;    
    int tmp = 1;
    int var = 1;
    int LedOnNum = 0;
    int IntPart = 0;
    double FloatPart = 0;
    char IntBuf[LED_NUM];
    char FloatBuf[LED_NUM];
    unsigned long ulLen = 0;
    xASSERT(ulLed && (LED1 | LED2 | LED3 | LED4));
    
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    
    //
    // Clear old configuration
    //
    Close(guLed);
    guStrLen = 0;
    guptStrPos = 0;
    guptLedPos = 0;
    bDotUse = 0;
    for (i = 0; i < LED_NUM; i++)
    {
        guDelayCount[i] = 0;
    }

    //
    // Specified new Leds to be opened
    //
    guLed = ulLed;

    //
    // Restart to calculate the number of opened seven segment leds
    //
    guLedOnNum = Open(guLed);
    Close(guLed);
   
    //
    // Current number of leds on(not include sign '+' or '-'). And if it is 
    // negative, convert it to positive.
    //
    if (Val < 0)
    {
        LedOnNum = guLedOnNum - 1;
        Val = 0 - Val;
        bSign = 1;
    }
    else
    {
        LedOnNum = guLedOnNum;
    }
    
    IntPart = (int)Val;
    FloatPart = Val - IntPart;
    
    //
    // Get the max value which leds can display
    //
    i = 0;
    while (i < LedOnNum)
    {
        tmp *= 10;
        i++;
    }

    //
    // Check if the value to be displayed is valid
    //
    xASSERT(IntPart < tmp);
    
    //
    // Convert the integer part to string.
    //
    i = LED_NUM - 1;
    if (IntPart == 0) 
    {
        IntBuf[i--] = '0';
    }
    else
    {

        while (IntPart != 0)
        {
            tmp = IntPart % 10;
            IntBuf[i--] = tmp + '0';
            IntPart /= 10;
        }
    }
    
    if (bSign == 1)
    {
        ulLen = LED_NUM - i ;
        gpcString[j++] = '-';
    }
    else 
    {
        ulLen = LED_NUM - i - 1; 
    }
    
    //
    // Check if the value to be displayed is valid
    //
    xASSERT(ulLen < guLedOnNum);
    
    //
    // Convert the float part to string.
    //
    tmp = 1;
    while (k < guLedOnNum - ulLen)
    {
        var *= 10;
        tmp = (int)(FloatPart * var) % 10;
        FloatBuf[k++] = tmp + '0';
    }

    i = i + 1;
    while (i < LED_NUM)
    {
        gpcString[j++] = IntBuf[i++];
    }
    gpcString[j++] = '.';

    i = 0;
    while (i < k)
    {
        gpcString[j++] = FloatBuf[i++];
    }
    guStrLen = j;
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Configure seven segment leds.
//!
//! \param ulLed The seven segment leds to be opened. It is specified using a
//!  bit-packed byte, where each bit which is set identifies the led to be 
//!  opened, and where bit 0 of the byte represents DIGITTUBE_1, bit 1
//!  represents DIGITTUBE_2, and so on.
//!
//! \param ulBlinkLed The seven segment leds to blink. It must be a subset of 
//!  ulLed.
//!
//! \param ulBlinkFreq The leds blink frequence. The interval time of blink is:
//!  2 * LED_NUM * BlinkFreq / TIMER_FREQ, the unit is second. When BlinkFreq 
//!  equals 0, it means no leds blink.
//!  
//! \param pcString The string to be displayed. And the character included must
//!  be one of the character table: [0~9], [A~Z], '-' and '.'. For example, you
//!  can set pcString to be "2a7h".
//!
//! This function is used to config seven segment leds. It must be called after
//! LedInit(). 
//!
//! It clears old configures for seven segment leds and writes new configure.
//! .
//!
//! \return None
//
//*****************************************************************************
void 
LedConfig(unsigned long ulLed, unsigned long ulBlinkLed,
                               unsigned long ulBlinkFreq, const char* pcString)
{
    int i = 0;
    
    xASSERT(ulLed && (LED1 | LED2 | LED3 | LED4));
    xASSERT(ulBlinkFreq == 0 || ulBlinkFreq == 1 || ulBlinkFreq == 2);
    xASSERT(pcString != 0);
    
    //
    // Disable timer interrupt to mutually access global variants
    //
    xTimerIntDisable(TIMERX_BASE, 0, TIMER_INT_MATCH);
    
    //
    // Clear old configuration
    //
    Close(guLed);
    guStrLen = 0;
    guptStrPos = 0;
    guptLedPos = 0;
    bDotUse = 0;
    for (i = 0; i < LED_NUM; i++)
    {
        a[i] = 0;
    }
    
    for (i = 0; i < STRING_SIZE; i++)
    {
        if (gpcString[i] == 0)
        {
            break;
        }
        gpcString[i] = 0;
    }

    //
    // Specified new Leds to be opened
    //
    guLed = ulLed;
    guBlinkLed = ulBlinkLed;
    guBlinkFreq = ulBlinkFreq;

    //
    // Restart to calculate the number of opened seven segment leds
    //
    guLedOnNum = Open(guLed);
    Close(guLed);
    
    //
    // Get the length of string to be displayed
    //
    i = 0;
    while (i < STRING_SIZE && pcString[i] != 0)
    {
        gpcString[i] = pcString[i];
        i++;
    }
    guStrLen = i;
    
    //
    // Enable timer interrupt again
    //
    xTimerIntEnable(TIMERX_BASE, 0, TIMER_INT_MATCH);
}

//*****************************************************************************
//
//! \brief Add display characters into character segment code table for seven 
//!  segment leds.
//!
//! \param ch The character to be added into character table. 
//!
//! \param ucCode The character segment code. The way to get segment code of 
//!  character is as follow:
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
//!    (the segment code for part of characters to be listed)
//!    -------------------------------------------------------
//!    |       Seven Segment Led Type: Common Anode          |
//!    +-------------------------------+-----------+---------+
//!    |           LED   SEG           |  SegCode  |  Char   |
//!    +---------------+---------------+-----------+---------+
//!    | H | G | F | E | D | C | B | A |           |         |
//!    +---------------+---------------+-----------+---------+
//!    | 1   1   0   0 | 0   0   0   0 |   0xC0    |    0    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   1   1   1 | 1   0   0   1 |   0xF9    |    1    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   1   0 | 0   1   0   0 |   0xA4    |    2    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   1   1 | 0   0   0   0 |   0xB0    |    3    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   1 | 1   0   0   1 |   0x99    |    4    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   1 | 0   0   1   0 |   0x92    |    5    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   0 | 0   0   1   0 |   0x82    |    6    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   1   1   1 | 1   0   0   0 |   0xF8    |    7    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   0 | 0   0   0   0 |   0x80    |    8    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   1 | 1   0   0   0 |   0x98    |    9    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   0 | 1   0   0   0 |   0x88    |    A    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   0 | 0   0   1   1 |   0x83    |    b    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   1   0   0 | 0   1   1   0 |   0xC6    |    C    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   1   0 | 0   0   0   1 |   0xA1    |    d    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   0 | 0   1   1   0 |   0x86    |    E    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   0 | 1   1   1   0 |   0x8E    |    F    |
//!    +---------------+---------------+-----------+---------+
//!    | 1   0   0   0 | 0   0   0   0 |   0x80    |    .    |
//!    +---------------+---------------+-----------+---------+
//!    | 0   1   0   0 | 0   0   0   0 |   0x40    |    -    |
//!    -------------------------------------------------------
//!
//! This function is used to add character leds can display. They can use this 
//! API to make leds display character they want when user can not find suitable
//! character in character table. It must be called afterLedInit(). 
//!
//! You can not change the segment code of character which has been in character
//! table.
//! .
//!
//! \return None
//
//*****************************************************************************
void 
CharTabAdd(char ch, unsigned char ucCode)
{
    int i = 0;
    //
    // Find the first null character from character table and replace it with  
    // specified character.
    //
    while (i < CHAR_NUM)
    {
        if (SegCodeTab[i].ch == 0)
        {
            SegCodeTab[i].ch = ch;
            SegCodeTab[i].ucCode = ucCode;
            break;
        }
        i++;
    }
}
