//*****************************************************************************
//
//! @page xgpio_testcase xgpio register test
//!
//! File: @ref xgpiotest.c
//!
//! <h2>Description</h2>
//! This module implements the test sequence for the seven segment sub component
//! .<br><br>
//! - xx
//! .
//!
//! <h2>Preconditions</h2>
//! The module requires the following options:<br><br>
//! - \p Option-define:
//! <br>(1)None.<br><br>
//! - \p Option-hardware:
//! <br>(1)Connect an USB cable to the development board.<br><br>
//! - \p Option-OtherModule:
//! <br>Connect an COM cable to the development board.<br>
//! .
//! In case some of the required options are not enabled then some or all tests
//! may be skipped or result FAILED.<br>
//!
//! <h2>Test Cases</h2>
//! The module contain those sub tests:<br><br>
//! - \subpage Test001
//! .
//
//*****************************************************************************

#include "test.h"
#include "sevenseg.h"

//*****************************************************************************
//
//!\page Test001 Test001
//!
//!<h2>Description</h2>
//!Test 001. <br>
//!
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Get the Test description of the test.
//!
//! \return the desccription of the test.
//
//*****************************************************************************
static char* Test001GetTest(void)
{
    return "Test [001]: ";
}

//*****************************************************************************
//
//! \brief something should do before the test execute of the test.
//!
//! \return None.
//
//*****************************************************************************
static void Test001Setup(void)
{
    LedInit();
}

//*****************************************************************************
//
//! \brief something should do after the test execute of the test.
//!
//! \return None.
//
//*****************************************************************************
static void Test001TearDown(void)
{

}

//*****************************************************************************
//
//! \brief 001 test execute main body.
//!
//! \return None.
//
//*****************************************************************************
static void Test001Execute(void)
{
    int i = 0;
    
    //
    // Leds configure test
    //
    LedConfig(0x0F, 0, 0, "CDEF");
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedConfig(0x0F, 0x03, 0, "CDEF");
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedConfig(0x0F, 0x03, 2, "CDEF");
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedConfig(0x0F, 0x03, 2, "2689");
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedConfig(0x03, 0x03, 2, "26");
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedConfig(0x0F, 0, 0, "CDEF");
    xSysCtlDelay(xSysCtlClockGet() / 2);

    //
    // Leds display string and numeric test
    //
    LedDisplay(0x0F, "ABCd");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedNumDisplay(0x0F, 1230);
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedNumDisplay(0x0F, -456);
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedFloatDisplay(0x0F, 0.789);
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedFloatDisplay(0x0F, -0.34);
    xSysCtlDelay(xSysCtlClockGet() / 4);

    //
    // Leds characters set test
    //
    LedConfig(0x0F, 0, 0, "2468");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedCharSet(0x01, '7');
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedCharSet(0x02, '5');
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedCharSet(0x04, '3');
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedCharSet(0x08, '1');
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedStringSet(0, 4, "7531");
    xSysCtlDelay(xSysCtlClockGet() / 4);

    //
    // Leds on\off test
    //
    LedConfig(0x0F, 0, 0, "21EF");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    for (i = 0; i < LED_NUM; i++)
    {
        LedOff(0x01 << i);
        xSysCtlDelay(xSysCtlClockGet() / 4);
    }
    i = i - 1;
    for (; i >= 0; i--)
    {
        LedOn(0x01 << i);
        xSysCtlDelay(xSysCtlClockGet() / 4);
    }

    //
    // Leds blink on\off test
    //
    LedConfig(0x0F, 0, 0, "0123");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    for (i = 0; i < LED_NUM; i++)
    {
        LedBlinkEnable(0x01 << i);
        xSysCtlDelay(xSysCtlClockGet() / 4);
    }
    i = i - 1;
    for (; i >= 0; i--)
    {
        LedBlinkDisable(0x01 << i);
        xSysCtlDelay(xSysCtlClockGet() / 4);
    }

    //
    // Leds blink frequence test
    //
    LedConfig(0x0F, 0x0F, 0, "5678");
    for (i = 2; i >= 0; i--)
    {
        LedBlinkFreqSet(i);
        xSysCtlDelay(xSysCtlClockGet() / 2);
    }

    //
    // Leds blink test
    //
    LedConfig(0x0F, 0x0F, 0, "5678");
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedBlinkSet(0x07, 2);
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedBlinkSet(0x03, 1);
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedBlinkSet(0x01, 0);
    xSysCtlDelay(xSysCtlClockGet() / 2);
    LedConfig(0x0F, 0x00, 0, "5678");
    xSysCtlDelay(xSysCtlClockGet() / 2);

    //
    // Character add to character segment code table test
    //
    CharTabAdd('@', 0xA0);
    LedDisplay(0x03, "@@");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    CharTabAdd('v', 0xD5);
    LedDisplay(0x03, "vv");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    CharTabAdd('[', 0xC6);
    LedDisplay(0x03, "[[");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    CharTabAdd(']', 0xF0);
    LedDisplay(0x03, "]]");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    LedDisplay(0x0F, "[@v]");
    xSysCtlDelay(xSysCtlClockGet() / 4);
    
    LedOff(0x0F);
}

//
// test case 001 struct.
//
const tTestCase sTest001 = {
		Test001GetTest,
		Test001Setup,
		Test001TearDown,
		Test001Execute
};

//
// Xgpio test suits.
//
const tTestCase * const psPattern001[] =
{
    &sTest001,
    0
};
