/**************************************************************************************************
    Filename:       tvsa_cc2530znp.c
    Revised:        $Date: 2011-07-25 20:23:05 -0700 (Mon, 25 Jul 2011) $
    Revision:       $Revision: 26899 $

    Description:

    This file implements the CC2530ZNP board-specific Temperature/Voltage calculations.

    Copyright 2009 Texas Instruments Incorporated. All rights reserved.

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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_adc.h"
#include "mt.h"
#include "tvsa.h"
#include "zap_app.h"
#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/*
 * These parameters are typical values and need to be calibrated
 * See the datasheet for the appropriate chip for more details
 * also, the math below may not be very accurate
 */
/* Assume ADC = 1480 at 25C and ADC = 4/C */
//#define VOLTAGE_AT_TEMP_25        1480
#define VOLTAGE_AT_TEMP_22        1468
#define TEMP_COEFFICIENT          4

#define HAL_ADC_REF_125V    0x00    /* Internal 1.25V Reference */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_CHN_TEMP    0x0e    /* Temperature sensor */

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define degC_0       (voltageAtTemp22 - (22 * TEMP_COEFFICIENT))
#define degC_100     (voltageAtTemp22 + (78 * TEMP_COEFFICIENT))

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint16 voltageAtTemp22;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void HalInitTV(void);
static void HalCalcTV(uint8 *tvDat);

/**************************************************************************************************
 * @fn          HalInitTV
 *
 * @brief       This function is called by tvsaAppInit() to calibrate to the assumed room temp
 *              of 22 degC.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void HalInitTV(void)
{
  uint8 calDat[TVSA_DAT_LEN];

  voltageAtTemp22 = VOLTAGE_AT_TEMP_22;
  HalCalcTV(calDat);

  if (22 > calDat[TVSA_TEM_IDX])
  {
    voltageAtTemp22 -= (22 - calDat[TVSA_TEM_IDX]) * TEMP_COEFFICIENT;
  }
  else if (22 < calDat[TVSA_TEM_IDX])
  {
    voltageAtTemp22 += (calDat[TVSA_TEM_IDX] - 22) * TEMP_COEFFICIENT;
  }
}

/**************************************************************************************************
 * @fn          HalCalcTV
 *
 * @brief       This function is called by tvsaDataCalc() to calculate the board-specific data
 *              for a TVSA report.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void HalCalcTV(uint8 *tvDat)
{
  uint16 tmp;
  uint8 args[2];

  args[0] = HAL_ADC_CHANNEL_TEMP;
  args[1] = HAL_ADC_RESOLUTION_14;

  zapSysReq(MT_SYS_ADC_READ, (uint8 *)&tmp, args);
  tmp >>= 4;  // Use the 12 MSB of adcValue.

  if (tmp < degC_0)
  {
    tmp = degC_0;
  }
  else if (tmp > degC_100)
  {
    tmp = degC_100;
  }
  
  tvDat[TVSA_TEM_IDX] = (uint8)((tmp - degC_0 + TEMP_COEFFICIENT-1) / TEMP_COEFFICIENT);

  args[0] = HAL_ADC_CHANNEL_VDD;

  zapSysReq(MT_SYS_ADC_READ, (uint8 *)&tmp, args);
  tmp >>= 4;  // Use the 12 MSB of adcValue.

  // Calculate the Vdd measured in 10ths of V.
  // ADC * 3.75 / 2047 * 10
  tmp *= 5;
  tmp /= 273;

  tvDat[TVSA_BUS_IDX] = (uint8)tmp;
}

/**************************************************************************************************
*/
