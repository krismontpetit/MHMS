/**************************************************************************************************
    Filename:       tvsa.c
    Revised:        $Date: 2011-07-25 20:23:05 -0700 (Mon, 25 Jul 2011) $
    Revision:       $Revision: 26899 $

    Description:

    This file implements the Temperature/Voltage Sample Application.




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

#include "af.h"
#if defined LCD_SUPPORTED
#include "hal_lcd.h"
#endif
#include "hal_uart.h"
#include "OnBoard.h"
#include "OSAL.h"
#include "tvsa.h"
#include "ZComDef.h"
#include "ZDApp.h"

#if !TVSA_DONGLE
#include "tvsa_cc2530znp.c"
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

//#define TVSA_DEMO  // TODO - define this constant for "Web Demo" TVSA behavior.

static const cId_t TVSA_ClusterList[TVSA_CLUSTER_CNT] =
{
  TVSA_CLUSTER_ID
};

static const SimpleDescriptionFormat_t TVSA_SimpleDesc =
{
  TVSA_ENDPOINT,
  TVSA_PROFILE_ID,
  TVSA_DEVICE_ID,
  TVSA_DEVICE_VERSION,
  TVSA_FLAGS,
  TVSA_CLUSTER_CNT,
  (cId_t *)TVSA_ClusterList,
  TVSA_CLUSTER_CNT,
  (cId_t *)TVSA_ClusterList
};

static const endPointDesc_t TVSA_epDesc=
{
  TVSA_ENDPOINT,
  &tvsaTaskId,
  (SimpleDescriptionFormat_t *)&TVSA_SimpleDesc,
  noLatencyReqs,
};

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

#if TVSA_DATA_CNF
uint8 tvsaCnfErrCnt;
#endif
uint8 tvsaTaskId;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

// Network address of the TVSA Dongle.
static uint16 tvsaAddr;
// Report counter.
static uint16 tvsaCnt;
// ZigBee-required packet transaction sequence number in calls to AF_DataRequest().
static uint8 tvsaTSN;

#if TVSA_DONGLE
static uint8 tvsaBuf[TVSA_BUF_LEN];
#if defined TVSA_DEMO
static uint8 tvsaCmd, tvsaState;
#endif
#else
static uint8 tvsaDat[TVSA_DAT_LEN];
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void tvsaAfMsgRx(afIncomingMSGPacket_t *msg);
static void tvsaSysEvtMsg(void);
#if !TVSA_DONGLE
static void tvsaDataCalc(void);
static void tvsaDataReq(void);
static void tvsaZdoStateChange(void);
#else //if TVSA_DONGLE
static void tvsaAnnce(void);
static void tvsaDataRx(afIncomingMSGPacket_t *msg);
static void tvsaUartRx(uint8 port, uint8 event);
static void tvsaZdoStateChange(void);
#ifndef TVSA_DEMO
static uint8 calcFCS(uint8 *pBuf, uint8 len);
static void sysPingRsp(void);
#endif

#endif

/**************************************************************************************************
 * @fn          tvsaAppInit
 *
 * @brief       This function is the application's task initialization.
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
void tvsaAppInit(uint8 id)
{
#if TVSA_DONGLE
  halUARTCfg_t uartConfig;

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  
#ifdef TVSA_DEMO
  uartConfig.baudRate             = HAL_UART_BR_115200;
#else
  uartConfig.baudRate             = HAL_UART_BR_38400;
#endif
  
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 16;                // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 32;                // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 254;               // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6;                 // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = tvsaUartRx;
  HalUARTOpen(TVSA_PORT, &uartConfig);
#else
  tvsaDat[TVSA_TYP_IDX] = (uint8)TVSA_DEVICE_ID;
#if defined TVSA_SRC_RTG
  tvsaDat[TVSA_OPT_IDX] = TVSA_OPT_SRC_RTG;
#endif
#endif

  tvsaTaskId = id;
  tvsaAddr = INVALID_NODE_ADDR;
  (void)afRegister((endPointDesc_t *)&TVSA_epDesc);
}

/**************************************************************************************************
 * @fn          tvsaAppEvt
 *
 * @brief       This function is called to process the OSAL events for the task.
 *
 * input parameters
 *
 * @param       id - OSAL task Id.
 * @param       evts - OSAL events bit mask of pending events.
 *
 * output parameters
 *
 * None.
 *
 * @return      evts - OSAL events bit mask of unprocessed events.
 **************************************************************************************************
 */
uint16 tvsaAppEvt(uint8 id, uint16 evts)
{
  uint16 mask = 0;
  (void)id;
  
  if (evts & SYS_EVENT_MSG)
  {
    mask = SYS_EVENT_MSG;
    tvsaSysEvtMsg();
  }
#if TVSA_DONGLE
  else if (evts & TVSA_EVT_ANN)
  {
    mask = TVSA_EVT_ANN;
    tvsaAnnce();
  }
#else
  else if (evts & TVSA_EVT_DAT)
  {
    mask = TVSA_EVT_DAT;
    tvsaDataCalc();
  }
  else if (evts & TVSA_EVT_REQ)
  {
    mask = TVSA_EVT_REQ;
    tvsaDataReq();
  }
#endif
  else
  {
    mask = evts;  // Discard unknown events - should never happen.
  }

  return (evts ^ mask);  // Return unprocessed events.
}

/**************************************************************************************************
 * @fn          tvsaAfMsgRx
 *
 * @brief       This function is called by tvsaSysEvtMsg() to process an incoming AF message.
 *
 * input parameters
 *
 * @param       msg - A pointer to the afIncomingMSGPacket_t packet.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void tvsaAfMsgRx(afIncomingMSGPacket_t *msg)
{
  uint8 *buf = msg->cmd.Data;

  switch (buf[TVSA_CMD_IDX])
  {
#if TVSA_DONGLE
  case TVSA_CMD_DAT:
    tvsaDataRx(msg);
    break;
#else

  case TVSA_CMD_BEG:
    if (INVALID_NODE_ADDR == tvsaAddr)
    {
      NLME_SetPollRate(0);
      (void)osal_set_event(tvsaTaskId, TVSA_EVT_DAT);
    }
    tvsaAddr = BUILD_UINT16(buf[TVSA_ADR_LSB], buf[TVSA_ADR_MSB]);
    break;

  case TVSA_CMD_END:
    NLME_SetPollRate(POLL_RATE);
    tvsaAddr = INVALID_NODE_ADDR;
    break;
#endif

  default:
    break;
  }
}

/**************************************************************************************************
 * @fn          tvsaSysEvtMsg
 *
 * @brief       This function is called by tvsaAppEvt() to process all of the pending OSAL messages.
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
static void tvsaSysEvtMsg(void)
{
  uint8 *msg;

  while ((msg = osal_msg_receive(tvsaTaskId)))
  {
    switch (*msg)
    {
#if TVSA_DATA_CNF
    case AF_DATA_CONFIRM_CMD:
      if (ZSuccess != ((afDataConfirm_t *)msg)->hdr.status)
      {
        if (0 == ++tvsaCnfErrCnt)
        {
          tvsaCnfErrCnt = 255;
        }
      }
      break;
#endif

    case AF_INCOMING_MSG_CMD:
      tvsaAfMsgRx((afIncomingMSGPacket_t *)msg);
      break;

    case ZDO_STATE_CHANGE:
      tvsaZdoStateChange();
      break;

    default:
      break;
    }

    (void)osal_msg_deallocate(msg);  // Receiving task is responsible for releasing the memory.
  }
}

#if !TVSA_DONGLE
/**************************************************************************************************
 * @fn          tvsaDataCalc
 *
 * @brief       This function is called by tvsaAppEvt() to calculate the data for a TVSA report.
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
static void tvsaDataCalc(void)
{
  if (INVALID_NODE_ADDR == tvsaAddr)
  {
    return;
  }

  if (ZSuccess != osal_start_timerEx(tvsaTaskId, TVSA_EVT_DAT, TVSA_DLY_DAT))
  {
    (void)osal_set_event(tvsaTaskId, TVSA_EVT_DAT);
  }

  HalCalcTV(tvsaDat);
#if TVSA_DATA_CNF
  tvsaDat[TVSA_RTG_IDX] = tvsaCnfErrCnt;
#else
  tvsaDat[TVSA_RTG_IDX] = 0;
#endif
  osal_set_event(tvsaTaskId, TVSA_EVT_REQ);
}

/**************************************************************************************************
 * @fn          tvsaDataReq
 *
 * @brief       This function is called by tvsaAppEvt() to send a TVSA data report.
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
static void tvsaDataReq(void)
{
  afAddrType_t addr;
  
  addr.addr.shortAddr = tvsaAddr;
  addr.addrMode = afAddr16Bit;
  addr.endPoint = TVSA_ENDPOINT;

  if (afStatus_SUCCESS != AF_DataRequest(&addr, (endPointDesc_t *)&TVSA_epDesc, TVSA_CLUSTER_ID,
                                          TVSA_DAT_LEN, tvsaDat, &tvsaTSN,
                                          AF_DISCV_ROUTE
#if TVSA_DATA_CNF
                                        | AF_ACK_REQUEST
#endif
                                         ,AF_DEFAULT_RADIUS))
  {
    osal_set_event(tvsaTaskId, TVSA_EVT_REQ);
  }
  else
  {
    tvsaCnt++;
  }
}

/**************************************************************************************************
 * @fn          tvsaZdoStateChange
 *
 * @brief       This function is called by tvsaSysEvtMsg() for a ZDO_STATE_CHANGE message.
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
static void tvsaZdoStateChange(void)
{
  (void)osal_stop_timerEx(tvsaTaskId, TVSA_EVT_DAT);

  if ((DEV_ZB_COORD == devState) || (DEV_ROUTER == devState) || (DEV_END_DEVICE == devState))
  {
    uint16 tmp = NLME_GetCoordShortAddr();
    uint8 dly = TVSA_STG_DAT;

    tvsaDat[TVSA_PAR_LSB] = LO_UINT16(tmp);
    tvsaDat[TVSA_PAR_MSB] = HI_UINT16(tmp);
    if ((DEV_ROUTER == devState) || (DEV_ZB_COORD == devState))
    {
      tvsaDat[TVSA_TYP_IDX] |= 0x80;
    }
    else
    {
      tvsaDat[TVSA_TYP_IDX] &= (0xFF ^ 0x80);
    }

#if TVSA_DONGLE_IS_ZC
    if (INVALID_NODE_ADDR == tvsaAddr)
    {
      // Assume ZC is the TVSA Dongle until a TVSA_CMD_BEG gives a different address.
      tvsaAddr = NWK_PAN_COORD_ADDR;
    }
#endif

    if (INVALID_NODE_ADDR != tvsaAddr)
    {
      if (ZSuccess != osal_start_timerEx(tvsaTaskId, TVSA_EVT_DAT, (dly + TVSA_DLY_MIN)))
      {
        (void)osal_set_event(tvsaTaskId, TVSA_EVT_DAT);
      }
    }

#if !TVSA_DONGLE
    if (0 == voltageAtTemp22)
    {
      HalInitTV();
      (void)osal_cpyExtAddr(tvsaDat+TVSA_IEE_IDX, &aExtendedAddress);
    }
#endif
  }
#if defined LCD_SUPPORTED
  HalLcdWriteValue(devState, 10, HAL_LCD_LINE_3);
#endif
}

#else // if TVSA_DONGLE
/**************************************************************************************************
 * @fn          tvsaAnnce
 *
 * @brief       This function is called by tvsaAppEvt() to send a TVSA announce to start or stop.
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
static void tvsaAnnce(void)
{
  uint8 msg[3];
  afAddrType_t addr;
  
  addr.addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVALL;
  addr.addrMode = afAddrBroadcast;
  addr.endPoint = TVSA_ENDPOINT;

  if (INVALID_NODE_ADDR != tvsaAddr)
  {
    msg[TVSA_CMD_IDX] = TVSA_CMD_BEG;
    if (ZSuccess != osal_start_timerEx(tvsaTaskId, TVSA_EVT_ANN, TVSA_DLY_ANN))
    {
      (void)osal_set_event(tvsaTaskId, TVSA_EVT_ANN);
    }
  }
  else
  {
    msg[TVSA_CMD_IDX] = TVSA_CMD_END;
  }

  msg[TVSA_ADR_LSB] = LO_UINT16(tvsaAddr);
  msg[TVSA_ADR_MSB] = HI_UINT16(tvsaAddr);

  if (afStatus_SUCCESS != AF_DataRequest(&addr, (endPointDesc_t *)&TVSA_epDesc, TVSA_CLUSTER_ID,
                                          3, msg, &tvsaTSN, AF_TX_OPTIONS_NONE, AF_DEFAULT_RADIUS))
  {
    osal_set_event(tvsaTaskId, TVSA_EVT_REQ);
  }
  else
  {
    tvsaCnt++;
  }
}

/**************************************************************************************************
 * @fn          tvsaDataRx
 *
 * @brief       This function is called by tvsaAfMsgRx() to process incoming TVSA data.
 *
 * input parameters
 *
 * @param       msg - A pointer to the afIncomingMSGPacket_t packet.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void tvsaDataRx(afIncomingMSGPacket_t *msg)
{
  uint8 fcs = 0, idx;

  // Last announce broadcast to stop must have expired before a parent could forward to a ZED.
  if (INVALID_NODE_ADDR == tvsaAddr)
  {
    (void)osal_set_event(tvsaTaskId, TVSA_EVT_ANN);
  }

  tvsaBuf[TVSA_SOP_IDX] = TVSA_SOP_VAL;
  tvsaBuf[TVSA_ADR_LSB] = LO_UINT16(msg->srcAddr.addr.shortAddr);
  tvsaBuf[TVSA_ADR_MSB] = HI_UINT16(msg->srcAddr.addr.shortAddr);

  // 1st byte of message is skipped - CMD is always 0 for data.
  (void)osal_memcpy(tvsaBuf+TVSA_DAT_OFF, msg->cmd.Data+1, TVSA_DAT_LEN-1);

  for (idx = TVSA_ADR_LSB; idx < TVSA_FCS_IDX; idx++)
  {
    fcs ^= tvsaBuf[idx];
  }
  tvsaBuf[idx] = fcs;
  
#ifdef TVSA_DEMO

  HalUARTWrite(TVSA_PORT, tvsaBuf, TVSA_BUF_LEN);

#else
  
  
  uint8 deviceTemp;
  uint8 deviceVolt;
  uint8 parentAddrLSB;
  uint8 parentAddrMSB;
  uint8 zsensorBuf[15];
  
  parentAddrLSB= tvsaBuf[11];
  parentAddrMSB= tvsaBuf[12];  
  deviceTemp = tvsaBuf[13];
  deviceVolt = tvsaBuf[14];
  
  //Start of Frame Delimiter
  zsensorBuf[0]=0xFE;
  
  
  zsensorBuf[1]=10;
  zsensorBuf[2]=LO_UINT16(0x8746);
  zsensorBuf[3]=HI_UINT16(0x8746);
  
  //Source Address
  zsensorBuf[4] = LO_UINT16(msg->srcAddr.addr.shortAddr);
  zsensorBuf[5] = HI_UINT16(msg->srcAddr.addr.shortAddr);
  
  zsensorBuf[6]=LO_UINT16(2);
  zsensorBuf[7]=HI_UINT16(2);
  zsensorBuf[8]=LO_UINT16(4);
  zsensorBuf[9]=HI_UINT16(4);
  
  //Temperature and Voltage Data
  zsensorBuf[10]= deviceTemp;
  zsensorBuf[11]=deviceVolt;
  
  //Parent Address
  zsensorBuf[12]=parentAddrLSB;
  zsensorBuf[13]=parentAddrMSB;


  //FCS Check on the middle 13 bytes
  zsensorBuf[14] = calcFCS(&zsensorBuf[1], 13 );


  HalUARTWrite(TVSA_PORT, zsensorBuf, 15);
  
  
#endif  
  
}

/**************************************************************************************************
 * @fn          tvsaUartRx
 *
 * @brief       This function is the Uart callback for Rx data.
 *
 * input parameters
 *
 * @param       port - Don't care.
 * @param       event - Don't care.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void tvsaUartRx(uint8 port, uint8 event)
{
#ifdef TVSA_DEMO
  uint8 ch;

  while (HalUARTRead(TVSA_PORT, &ch, 1))
  {
    switch (tvsaState)
    {
    case SOP_STATE:
      if (TVSA_SOP_VAL == ch)
      {
        tvsaState = CMD_STATE;
      }
      break;

    case CMD_STATE:
      tvsaCmd = ch;
      tvsaState = FCS_STATE;
      break;

    case FCS_STATE:
      if (tvsaCmd == ch)
      {
        if (tvsaCmd == TVSA_CMD_BEG)
        {
          tvsaAddr = NLME_GetShortAddr();
        }
        else if (tvsaCmd == TVSA_CMD_END)
        {
          tvsaAddr = INVALID_NODE_ADDR;
        }
        (void)osal_set_event(tvsaTaskId, TVSA_EVT_ANN);
      }

      tvsaState = SOP_STATE;
      break;

    default:
     break;
    }
  }
#else
  uint8 ch[5];
  
  HalUARTRead(TVSA_PORT, ch, 5);
  if (ch[2]==0x21)   //if statement to check for command from Z-Sensor Monitor
  {
    sysPingRsp();
  }
#endif
}

/**************************************************************************************************
 * @fn          tvsaZdoStateChange
 *
 * @brief       This function is called by tvsaSysEvtMsg() for a ZDO_STATE_CHANGE message.
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
static void tvsaZdoStateChange(void)
{
  (void)osal_stop_timerEx(tvsaTaskId, TVSA_EVT_ANN);

  if ((DEV_ZB_COORD == devState) || (DEV_ROUTER == devState) || (DEV_END_DEVICE == devState))
  {
#if TVSA_DONGLE_IS_ZC
    if (INVALID_NODE_ADDR == tvsaAddr)
    {
      // Assume ZC is the TVSA Dongle until a TVSA_CMD_BEG gives a different address.
      tvsaAddr = NWK_PAN_COORD_ADDR;
    }
#endif

    if (INVALID_NODE_ADDR != tvsaAddr)
    {
      if (ZSuccess != osal_start_timerEx(tvsaTaskId, TVSA_EVT_ANN, TVSA_DLY_ANN))
      {
        (void)osal_set_event(tvsaTaskId, TVSA_EVT_ANN);
      }
    }
  }
}

#ifndef TVSA_DEMO
/******************************************************************************
 * @fn          calcFCS
 *
 * @brief       This function calculates the FCS checksum for the serial message 
 *
 * @param       pBuf - Pointer to the end of a buffer to calculate the FCS.
 *              len - Length of the pBuf.
 *
 * @return      The calculated FCS.
 ******************************************************************************
 */
static uint8 calcFCS(uint8 *pBuf, uint8 len)
{
  uint8 rtrn = 0;

  while (len--)
  {
    rtrn ^= *pBuf++;
  }

  return rtrn;
}

/*************************************************************************************************
 * @fn          sysPingRsp
 *
 * @brief       Build and send Ping response
 *
 * @param       none
 *              
 * @return      none
**************************************************************************************************
 */
static void sysPingRsp(void)
{
  uint8 pingBuff[7];
  
  // Start of Frame Delimiter
  pingBuff[0] = 0xFE;
  
  // Length
  pingBuff[1] = 0x02; 
  
  // Command type
  pingBuff[2] = LO_UINT16(0x0161); 
  pingBuff[3] = HI_UINT16(0x0161);
  
  // Stack profile
  pingBuff[4] = LO_UINT16(0x0041);
  pingBuff[5] = HI_UINT16(0x0041);
  
  // Frame Check Sequence
  pingBuff[6] = calcFCS(&pingBuff[1], 5);
  
  
  HalUARTWrite(TVSA_PORT,pingBuff, 7);

}


#endif





#endif

/**************************************************************************************************
*/
