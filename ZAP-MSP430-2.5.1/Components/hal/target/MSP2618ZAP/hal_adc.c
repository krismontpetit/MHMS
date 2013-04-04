/**************************************************************************************************
  Filename:       hal_adc.c
  Revised:        $Date: 2010-02-25 12:33:58 -0800 (Thu, 25 Feb 2010) $
  Revision:       $Revision: 21797 $

  Description:    This file contains the interface to the HAL ADC.


  Copyright 2007-2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/

#include "hal_types.h"
#include "hal_adc.h"
#include "hal_mcu.h"

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* POT defines */
#define HAL_ADC_POT_CHANNEL             HAL_ADC_CHANNEL_0
#define HAL_ADC_POT_CH_BV               BV(0)
#define HAL_ADC_POT_CONV_STARTADDR      CSTARTADD_0
#define HAL_ADC_POT_RESULT              ADC12MEM0
#define HAL_ADC_POT_MEM_CONTROL         ADC12MCTL0

/* TEMP SENSOR defines */
#define HAL_ADC_TS_CHANNEL              10
#define HAL_ADC_TS_CH_BV                BV(10)
#define HAL_ADC_TS_CONV_STARTADDR       CSTARTADD_10
#define HAL_ADC_TS_RESULT               ADC12MEM10
#define HAL_ADC_TS_MEM_CONTROL          ADC12MCTL10

/* JOYSTICK Level defines */
#define HAL_ADC_JOY_CHANNEL             6
#define HAL_ADC_JOY_CH_BV               BV(6)
#define HAL_ADC_JOY_CONV_STARTADDR      CSTARTADD_6
#define HAL_ADC_JOY_RESULT              ADC12MEM6
#define HAL_ADC_JOY_MEM_CONTROL         ADC12MCTL6

/* ADC port */
#define HAL_ADC_PORT_SELECT             P6SEL

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/
/* Enables/disables the ADC macros */
#define HAL_ADC_ENABLE_12()   { ADC12CTL0 |= ENC; }
#define HAL_ADC_DISABLE_12()  { ADC12CTL0 &= (ENC ^ 0xFFFF); }

/**************************************************************************************************
 *                                      LOCAL FUNCTIONS
 **************************************************************************************************/
uint16 HalAdcReadPot(void);
uint16 HalAdcReadJoyLevel(void);
uint16 HalAdcReadTS(void);

/**************************************************************************************************
 * @fn      HalAdcInit
 *
 * @brief   Initialize ADC Service
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalAdcInit (void)
{
  /* Initialize ADC */
  ADC12CTL0 = 0;
  ADC12CTL1 = 0;
  ADC12MCTL0 = 0;

  /* Enable ADC for POT and KEY */
  HAL_ADC_PORT_SELECT |= HAL_ADC_POT_CH_BV | HAL_ADC_JOY_CH_BV;

}

/**************************************************************************************************
 * @fn      HalAdcRead
 *
 * @brief   Read the ADC based on given channel and resolution
 *
 * @param   channel - channel where ADC will be read
 *
 *          resolution - the resolution of the value
 *
 * @return  16 bit value of the ADC
 **************************************************************************************************/
uint16 HalAdcRead (uint8 channel, uint8 resolution)
{
  uint16 reading = 0;

  switch (channel)
  {
    case HAL_ADC_POT_CHANNEL:
      reading = HalAdcReadPot();
      break;

    case HAL_ADC_JOY_CHANNEL:
      reading = HalAdcReadJoyLevel();
      break;

    case HAL_ADC_TS_CHANNEL:
      reading = HalAdcReadTS();
      break;

    default:
      break;
  }

  /* Get Sample */
  switch (resolution)
  {
    case HAL_ADC_RESOLUTION_12:
      break;
    case HAL_ADC_RESOLUTION_8:
      reading >>= 4;
      break;
    case HAL_ADC_RESOLUTION_10:
      reading >>= 2;
      break;
    case HAL_ADC_RESOLUTION_14:
      reading <<= 2;
    default:
      break;
  }

  return (reading);
}

/*********************************************************************
 * @fn       HalAdcCheckVdd
 *
 * @brief    Check for minimum Vdd specified.
 *
 * @param   vdd - The board-specific Vdd reading to check for.
 *
 * @return  TRUE if the Vdd measured is greater than the 'vdd' minimum parameter;
 *          FALSE if not.
 *
 *********************************************************************/
bool HalAdcCheckVdd(uint8 vdd)
{
  ADC12CTL0 = REFON | ADC12ON;
  ADC12CTL1 = SHP;
  ADC12MCTL0 = SREF_1 | EOS | INCH_11;

  __delay_cycles(6000);                    // Settling time for internal reference on SmartRF05EB.

  ADC12CTL0 |= ENC + ADC12SC;              // Sampling and conversion start.
  while (ADC12IFG == 0);
  ADC12CTL0 = 0;                           // Disable the ADC
  ADC12CTL0 = 0;                           // Turn off reference (must be done AFTER clearing ENC).
 
  return ((uint8)(ADC12MEM0 >> 4) > vdd);
}

/**************************************************************************************************
 * @fn      HalAdcReadPot
 *
 * @brief   Read the POT value
 *
 * @param   void
 *
 * @return  16 bit value of the ADC
 **************************************************************************************************/
uint16 HalAdcReadPot()
{
  /* Setup sampling */
  ADC12CTL0 = ADC12ON + SHT0_2;
  HAL_ADC_POT_MEM_CONTROL = SREF_0 | HAL_ADC_POT_CHANNEL;

  /* Disable conversions */
  HAL_ADC_DISABLE_12()
  /* Use sampling timer */
  ADC12CTL1 = HAL_ADC_POT_CONV_STARTADDR | SHP;
  /* Enable conversions */
  HAL_ADC_ENABLE_12();
  /* Start Conversion */
  ADC12CTL0 |= ADC12SC;
  /* Wait for conversion to be done */
  while (!(ADC12IFG & HAL_ADC_POT_CH_BV));

  /* Get Result */
  return (HAL_ADC_POT_RESULT);
}

/**************************************************************************************************
 * @fn      HalAdcReadJoyLevel
 *
 * @brief   Read the Joystick level
 *
 * @param   NONE
 *
 * @return  16 bit value of the ADC
 **************************************************************************************************/
uint16 HalAdcReadJoyLevel()
{
  /* Setup sampling */
  ADC12CTL0 = ADC12ON + SHT0_2;
  HAL_ADC_JOY_MEM_CONTROL = SREF_0 | HAL_ADC_JOY_CHANNEL;

  /* Disable conversions */
  HAL_ADC_DISABLE_12()
  /* Use sampling timer */
  ADC12CTL1 = HAL_ADC_JOY_CONV_STARTADDR | SHP;
  /* Enable conversions */
  HAL_ADC_ENABLE_12();
  /* Start Conversion */
  ADC12CTL0 |= ADC12SC;
  /* Wait for conversion to be done */
  while (!(ADC12IFG & HAL_ADC_JOY_CH_BV));

  /* Get Result */
  return (HAL_ADC_JOY_RESULT);
}

/**************************************************************************************************
 * @fn      HalAdcReadTS()
 *
 * @brief   Read the Temperature sensor value
 *
 * @param   NONE
 *
 * @return  16 bit value of the ADC
 **************************************************************************************************/
uint16 HalAdcReadTS()
{
  /* Setup sampling */
  ADC12CTL0 = ADC12ON + SHT0_2;
  HAL_ADC_TS_MEM_CONTROL = SREF_0 | HAL_ADC_TS_CHANNEL;

  /* Disable conversions */
  HAL_ADC_DISABLE_12()
  /* Use sampling timer */
  ADC12CTL1 = HAL_ADC_TS_CONV_STARTADDR | SHP;
  /* Enable conversions */
  HAL_ADC_ENABLE_12();
  /* Start Conversion */
  ADC12CTL0 |= ADC12SC;
  /* Wait for conversion to be done */
  while (!(ADC12IFG & HAL_ADC_TS_CH_BV));

  /* Get Result */
  return (HAL_ADC_TS_RESULT);
}

/**************************************************************************************************
**************************************************************************************************/
