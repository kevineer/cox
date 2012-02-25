/*********************************************************************************************************//**
 * @file    ht32f125x_pwrcu.c
 * @version V1.0
 * @date    04/11/2011
 * @brief   The files provides all the Power Control Unit firmware functions.
 *************************************************************************************************************
 *
 * <h2><center>Copyright (C) 2011 Holtek Semiconductor Inc. All rights reserved</center></h2>
 *
 ************************************************************************************************************/

/* Includes ------------------------------------------------------------------------------------------------*/
#include "ht32f125x_pwrcu.h"

/** @addtogroup HT32F125x_Peripheral_Driver HT32F125x Peripheral Driver
  * @{
  */

/** @defgroup PWRCU PWRCU
  * @brief PWRCU driver modules
  * @{
  */

/** @defgroup PWRCU_Private_TypesDefinitions PWRCU private types definitions
  * @{
  */
/**
  * @} End of PWRCU_Private_TypesDefinitions
  */

/** @defgroup PWRCU_Private_Define PWRCU private definitions
  * @{
  */

#define BAKCR_ADDR        (PWRCU_BASE + 0x04)
#define LVDCSR_ADDR       (PWRCU_BASE + 0x10)

#define BB_LDOOFF         BitBand(BAKCR_ADDR, 3)
#define BB_DMOSON         BitBand(BAKCR_ADDR, 7)
#define BB_WUPEN          BitBand(BAKCR_ADDR, 8)
#define BB_WUPIEN         BitBand(BAKCR_ADDR, 9)
#define BB_V18RDYSC       BitBand(BAKCR_ADDR, 12)
#define BB_DMOSSTS        BitBand(BAKCR_ADDR, 15)

#define BB_BODEN          BitBand(LVDCSR_ADDR, 0)
#define BB_BODRIS         BitBand(LVDCSR_ADDR, 1)
#define BB_BODF           BitBand(LVDCSR_ADDR, 3)
#define BB_LVDEN          BitBand(LVDCSR_ADDR, 16)
#define BB_LVDF           BitBand(LVDCSR_ADDR, 19)
#define BB_LVDWEN         BitBand(LVDCSR_ADDR, 20)

#define LVDS_MASK         0xFFF9FFFF
#define BAKRST_SET        0x1
#define BAKTEST_READY     0x27
#define TIME_OUT          36000000

#define SLEEPDEEP_SET     0x04      /*!< Cortex SLEEPDEEP bit */

/**
  * @} End of PWRCU_Private_Define group
  */

/** @defgroup PWRCU_Private_Macro PWRCU private macros
  * @{
  */
/**
  * @} End of PWRCU_Private_Macro group
  */

/** @defgroup PWRCU_Private_Variable PWRCU private variables
  * @{
  */
/**
  * @} End of PWRCU_Private_Variable group
  */

/** @defgroup PWRCU_Private_Function_Prototype PWRCU private function prototypes
  * @{
  */
/**
  * @} End of PWRCU_Private_Function_Prototype group
  */

/** @defgroup PWRCU_Private_Function PWRCU private functions
  * @{
  */

/*********************************************************************************************************//**
 * @brief Deinitializes backup domain which contains PWRCU and RTC units.
 * @retval None
 ************************************************************************************************************/
void PWRCU_DeInit(void)
{
  PWRCU->BAKCR = BAKRST_SET ;
  while(PWRCU->BAKCR);
  while(PWRCU->BAKSR);    /* Waits until the BAKPORF be cleared by read */
}

/*********************************************************************************************************//**
 * @brief Waits, until the RTC & PWRCU can be accessed.
 * @return PWRCU_TIMEOUT or PWRCU_OK
 ************************************************************************************************************/
PWRCU_Status PWRCU_CheckReadyAccessed(void)
{
  u32 wTimeOutCnt = TIME_OUT ;

  /* Sets the BK_ISO bit */
  CKCU->LPCR = 0x1 ;

  while(--wTimeOutCnt)
  {
    if(PWRCU->BAKTEST == BAKTEST_READY)
    {
      u32 write = ~PWRCU->BAKREG[9];
      u32 backup = PWRCU->BAKREG[9];
      while(1)
      {
        PWRCU->BAKREG[9] = write;
        if(PWRCU->BAKREG[9] == write) break;
      }
      PWRCU->BAKREG[9] = backup;
      return PWRCU_OK ;
    }
  }
  return PWRCU_TIMEOUT ;
}

/*********************************************************************************************************//**
 * @brief Returns the flags of PWRCU.
 * @return This function will return one of following:
 *   - 0x0000                                   : There is no flag is set.
 *   - 0x0001 (PWRCU_FLAG_BAKPOR)               : This is backup domain power on reset.
 *   - 0x0002 (PWRCU_FLAG_PD)                   : Power-Down flag has been set.
 *   - 0x0100 (PWRCU_FLAG_WUP)                  : External WAKEUP pin flag has been set.
 *   - 0x0102 (PWRCU_FLAG_PD | PWRCU_FLAG_WUP)  : Both PDF and WUPF flags have been set.
 ************************************************************************************************************/
u16 PWRCU_GetFlagStatus(void)
{
  return PWRCU->BAKSR;
}

/*********************************************************************************************************//**
 * @brief Returns the value of specified backup register.
 * @param BAKREGx: Number of backup register. Where x can be 0 ~ 9.
 * @return Between 0x0 ~ 0xFFFFFFFF.
 ************************************************************************************************************/
u32 PWRCU_ReadBackupRegister(PWRCU_BAKREG_Enum BAKREGx)
{
  Assert_Param(IS_PWRCU_BAKREG(BAKREGx));

  return PWRCU->BAKREG[BAKREGx];
}

/*********************************************************************************************************//**
 * @brief Writes the DATA to specified backup register.
 * @param BAKREGx : Number of backup registers. Where x can be 0 ~ 9.
 * @param DATA : Must between 0x0 ~ 0xFFFFFFFF.
 * @retval None
 ************************************************************************************************************/
void PWRCU_WriteBackupRegister(PWRCU_BAKREG_Enum BAKREGx, u32 DATA)
{
  Assert_Param(IS_PWRCU_BAKREG(BAKREGx));

  PWRCU->BAKREG[BAKREGx] = DATA;
}

/*********************************************************************************************************//**
 * @brief Enters SLEEP mode.
 * @param SleepEntry : Enters sleep mode instruction that is used to WFI or WFE.
 *   This parameter can be one of the following values:
 *   @arg PWRCU_SLEEP_ENTRY_WFE: Enters SLEEP mode via WFE instruction.
 *   @arg PWRCU_SLEEP_ENTRY_WFI: Enters SLEEP mode via WFI instruction.
 * @retval None
 ************************************************************************************************************/
void PWRCU_Sleep(PWRCU_SLEEP_ENTRY_Enum SleepEntry)
{
  Assert_Param(IS_PWRCU_SLEEP_ENTRY(SleepEntry));

  /* Clear SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR &= ~(u32)SLEEPDEEP_SET;

  if(SleepEntry == PWRCU_SLEEP_ENTRY_WFE)
  {
    /* Wait for event */
    __WFE();
  }
  else
  {
    /* Wait for interrupt */
    __WFI();
  }
}

/*********************************************************************************************************//**
 * @brief Enters DEEP-SLEEP Mode 1.
 * @param SleepEntry : Enters sleep mode instruction that is used to WFI or WFE.
 *   This parameter can be one of the following values:
 *   @arg PWRCU_SLEEP_ENTRY_WFE: Enters SLEEP mode via WFE instruction.
 *   @arg PWRCU_SLEEP_ENTRY_WFI: Enters SLEEP mode via WFI instruction.
 * @retval None
 ************************************************************************************************************/
void PWRCU_DeepSleep1(PWRCU_SLEEP_ENTRY_Enum SleepEntry)
{
  Assert_Param(IS_PWRCU_SLEEP_ENTRY(SleepEntry));

  BB_DMOSON = 0x0 ;
  BB_LDOOFF = 0x0 ;

  /* Sets SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SLEEPDEEP_SET;

  if(SleepEntry == PWRCU_SLEEP_ENTRY_WFE)
  {
    /* Wait for event */
    __WFE();
  }
  else
  {
    /* Wait for interrupt */
    __WFI();
  }
}

/*********************************************************************************************************//**
 * @brief Enters DEEP-SLEEP Mode 2.
 * @param SleepEntry : Enters sleep mode instruction that is used to WFI or WFE.
 *   This parameter can be one of the following values:
 *   @arg PWRCU_SLEEP_ENTRY_WFE: Enters SLEEP mode via WFE instruction.
 *   @arg PWRCU_SLEEP_ENTRY_WFI: Enters SLEEP mode via WFI instruction.
 * @retval None
 ************************************************************************************************************/
void PWRCU_DeepSleep2(PWRCU_SLEEP_ENTRY_Enum SleepEntry)
{
  Assert_Param(IS_PWRCU_SLEEP_ENTRY(SleepEntry));

  if(BB_DMOSSTS == 0)
  {
    BB_DMOSON = 0x0 ;
    BB_DMOSON = 0x1 ;
  }
  BB_LDOOFF = 0x0 ;

  /* Sets SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SLEEPDEEP_SET;

  if(SleepEntry == PWRCU_SLEEP_ENTRY_WFE)
  {
    /* Wait for event */
    __WFE();
  }
  else
  {
    /* Wait for interrupt */
    __WFI();
  }
}

/*********************************************************************************************************//**
 * @brief Enters POWER-DOWN Mode.
 * @retval None
 ************************************************************************************************************/
void PWRCU_PowerDown(void)
{
  BB_DMOSON = 0x0 ;
  BB_LDOOFF = 0x1 ;

  /* Sets SLEEPDEEP bit of Cortex System Control Register */
  SCB->SCR |= SLEEPDEEP_SET;

  /* Enters power-down mode. */
  __WFE();
}

/*********************************************************************************************************//**
 * @brief Configures LVD voltage level.
 * @param Level: Low voltage detect level.
 *   This parameter can be one of following:
 *   @arg PWRCU_LVDS_2V6: 2.6 V
 *   @arg PWRCU_LVDS_2V7: 2.7 V
 *   @arg PWRCU_LVDS_2V8: 2.8 V
 *   @arg PWRCU_LVDS_2V9: 2.9 V
 * @retval None
 ************************************************************************************************************/
void PWRCU_SetLVDS(PWRCU_LVDS_Enum Level)
{
  Assert_Param(IS_PWRCU_LVDS(Level));

  PWRCU->LVDCSR = (PWRCU->LVDCSR & LVDS_MASK) | Level ;
}

/*********************************************************************************************************//**
 * @brief Enables or Disables LVD function.
 * @param NewState: ENABLE or DISABLE.
 * @retval None
 ************************************************************************************************************/
void PWRCU_LVDCmd(ControlStatus NewState)
{
  Assert_Param(IS_CONTROL_STATUS(NewState));

  BB_LVDEN = NewState;
}

/*********************************************************************************************************//**
 * @brief Enables or Disables BOD reset function.
 * @param NewState: ENABLE or DISABLE.
 * @retval None
 ************************************************************************************************************/
void PWRCU_BODCmd(ControlStatus NewState)
{
  Assert_Param(IS_CONTROL_STATUS(NewState));

  BB_BODEN = NewState;
}

/*********************************************************************************************************//**
 * @brief Select when the BOD occurs, the action for the cause Reset or Interrupt.
 * @param Selection: BOD reset or interrupt selection.
 *   This parameter can be one of following:
 *   @arg PWRCU_BODRIS_RESET: Reset the whole chip
 *   @arg PWRCU_BODRIS_INT: Assert interrupt
 * @retval None
 ************************************************************************************************************/
void PWRCU_BODRISConfig(PWRCU_BODRIS_Enum Selection)
{
  Assert_Param(IS_PWRCU_BODRIS(Selection));

  BB_BODRIS = Selection;
}

/*********************************************************************************************************//**
 * @brief Return the flag status of LVD .
 * @return SET or RESET.
 ************************************************************************************************************/
FlagStatus PWRCU_GetLVDFlagStatus(void)
{
  return (FlagStatus)BB_LVDF;
}

/*********************************************************************************************************//**
 * @brief Return the flag status of BOD .
 * @return SET or RESET.
 ************************************************************************************************************/
FlagStatus PWRCU_GetBODFlagStatus(void)
{
  return (FlagStatus)BB_BODF;
}

/*********************************************************************************************************//**
 * @brief Returns the DMOS status.
 * @return This function will return one of following:
 *    - PWRCU_DMOS_STS_ON              : DMOS on
 *    - PWRCU_DMOS_STS_OFF             : DMOS off
 *    - PWRCU_DMOS_STS_OFF_BY_BODRESET : DMOS off caused by brow out reset
 ************************************************************************************************************/
PWRCU_DMOSStatus PWRCU_GetDMOSStatus(void)
{
  u32 wDmosStatus = PWRCU->BAKCR & 0x8080 ;

  if(wDmosStatus == 0x0 )
  {
    return PWRCU_DMOS_STS_OFF;
  }
  else if(wDmosStatus == 0x8080)
  {
    return PWRCU_DMOS_STS_ON;
  }
  else
  {
    return PWRCU_DMOS_STS_OFF_BY_BODRESET;
  }
}

/*********************************************************************************************************//**
 * @brief Enables or Disables DMOS function.
 * @param NewState: ENABLE or DISABLE.
 * @retval None
 ************************************************************************************************************/
void PWRCU_DMOSCmd(ControlStatus NewState)
{
  Assert_Param(IS_CONTROL_STATUS(NewState));

  if(BB_DMOSSTS == 0)
  {
    BB_DMOSON = 0;
  }

  BB_DMOSON = NewState;
}

/*********************************************************************************************************//**
 * @brief Configures VDD18 power good source.
 * @param Sel: Specifies VDD18 power good source.
 *    @arg PWRCU_V18RDYSC_BKISO  : Vdd18 power good source come from BK_ISO bit in CKCU unit
 *    @arg PWRCU_V18RDYSC_V18POR : Vdd18 power good source come from Vdd18 power on reset
 * @retval None
 ************************************************************************************************************/
void PWRCU_V18RDYSourceConfig(PWRCU_V18RDYSC_Enum Sel)
{
  Assert_Param(IS_PWRCU_V18RDYSC(Sel));

  BB_V18RDYSC = Sel;
}

/*********************************************************************************************************//**
 * @brief Enables or Disables LVD wakeup and interrupt function.
 * @param NewState: ENABLE or DISABLE.
 * @retval None
 ************************************************************************************************************/
void PWRCU_LVDWakeupIntConfig(ControlStatus NewState)
{
  Assert_Param(IS_CONTROL_STATUS(NewState));

  BB_LVDWEN = NewState;
}

/*********************************************************************************************************//**
 * @brief Enables or Disables external WAKEUP pin function.
 * @param NewState: ENABLE or DISABLE.
 * @retval None
 ************************************************************************************************************/
void PWRCU_WakeupPinCmd(ControlStatus NewState)
{
  Assert_Param(IS_CONTROL_STATUS(NewState));

  BB_WUPEN = NewState;
}

/*********************************************************************************************************//**
 * @brief Enables or Disables external WAKEUP pin interrupt function.
 * @param NewState: ENABLE or DISABLE.
 * @retval None
 ************************************************************************************************************/
void PWRCU_WakeupPinIntConfig(ControlStatus NewState)
{
  Assert_Param(IS_CONTROL_STATUS(NewState));

  BB_WUPIEN = NewState;
}

/*********************************************************************************************************//**
 * @brief Configures HSI ready counter bit length.
 * @param BitLength: HSI ready counter bit length.
 *   This parameter can be one of following:
 *   @arg PWRCU_HSIRCBL_7: 7 bits
 *   @arg PWRCU_HSIRCBL_8: 8 bits
 *   @arg PWRCU_HSIRCBL_9: 9 bits
 *   @arg PWRCU_HSIRCBL_10: 10 bits (Default)
 * @retval None
 ************************************************************************************************************/
void PWRCU_HSIReadyCounterBitLengthConfig(PWRCU_HSIRCBL_Enum BitLength)
{
  Assert_Param(IS_PWRCU_HSIRCBL(BitLength));

  PWRCU->HSIRCR = BitLength;
}

/**
  * @} End of PWRCU_Private_Function
  */

/**
  * @} End of PWRCU group
  */

/**
  * @} End of HT32F125x_Peripheral_Driver group
  */

