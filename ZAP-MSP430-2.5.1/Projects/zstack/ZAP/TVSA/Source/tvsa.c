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
#include "hal_led.h"  //MHMS for indicating if pulse is found
#include "hal_adc.h"  //MHMS used for capturing signal from pulse sensor

/*#if !TVSA_DONGLE  //MHMS dont' need this
#include "tvsa_cc2530znp.c"
#endif
*/

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */



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

// Constants for Pulse Sensor

static const cId_t PULSE_ClusterList[PULSE_CLUSTER_CNT] =
{
  PULSE_CLUSTER_ID
};

static const SimpleDescriptionFormat_t PULSE_SimpleDesc =
{
  PULSE_ENDPOINT,
  PULSE_PROFILE_ID,
  PULSE_DEVICE_ID,
  PULSE_DEVICE_VERSION,
  PULSE_FLAGS,
  PULSE_CLUSTER_CNT,
  (cId_t *)PULSE_ClusterList,
  PULSE_CLUSTER_CNT,
  (cId_t *)PULSE_ClusterList
};

static const endPointDesc_t PULSE_epDesc=
{
  PULSE_ENDPOINT,
  &pulseTaskId,
  (SimpleDescriptionFormat_t *)&PULSE_SimpleDesc,
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

//MHMS  Global Variables
uint8 pulseTaskId;


/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

// Network address of the TVSA Dongle.
static uint16 tvsaAddr;
static uint16 pulseAddr;
// Report counter.
static uint16 tvsaCnt;
// ZigBee-required packet transaction sequence number in calls to AF_DataRequest().
static uint8 tvsaTSN;
static uint8 pulseTSN;           //MHMS

#if TVSA_DONGLE
static uint8 pulseBuf[PULSE_BUF_LEN];
#if defined TVSA_DEMO
static uint8 tvsaCmd, tvsaState;
#endif
#else
static uint8 tvsaDat[TVSA_DAT_LEN];
static uint8 pulseDat[PULSE_DAT_LEN];  //MHMS define data array length for Pulse sensor
#endif

//MHMS From arduino interrupt
volatile int rate[10];                    // used to hold last ten IBI values
volatile uint32 sampleCounter = 0;          // used to determine pulse timing
volatile uint32 lastBeatTime = 0;           // used to find the inter beat interval
volatile int P = 512;                      // used to find peak in pulse wave
volatile int T = 512;                     // used to find trough in pulse wave
volatile int thresh = 512;                // used to find instant moment of heart beat
volatile int amp = 100;                   // used to hold amplitude of pulse waveform
volatile bool firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile bool secondBeat = true;       // used to seed rate array so we startup with reasonable BPM

// these variables are volatile because they are used during the interrupt service routine!
//MHMS From Arduino 1.1
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile bool Pulse = false;     // true when pulse wave is high, false when it's low
volatile bool QS = false;        // becomes true when Arduoino finds a beat.  



/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void tvsaAfMsgRx(afIncomingMSGPacket_t *msg);
static void tvsaSysEvtMsg(void);

#if !TVSA_DONGLE
static void pulseBPM(uint8 *pulsedata);  //MHMS Pulse calculation function
static void pulseDataCalc(void);
static void pulseDataReq(void);

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

  switch (buf[PULSE_CMD_IDX])
  {
#if TVSA_DONGLE
  case PULSE_CMD_DAT:
    tvsaDataRx(msg);
    break;
#else

  case PULSE_CMD_BEG:
    if (INVALID_NODE_ADDR == pulseAddr)
    {
      NLME_SetPollRate(0);
      (void)osal_set_event(pulseTaskId, PULSE_EVT_DAT);
    }
    pulseAddr = BUILD_UINT16(buf[TVSA_ADR_LSB], buf[TVSA_ADR_MSB]);
    break;

  case PULSE_CMD_END:
    NLME_SetPollRate(POLL_RATE);
    pulseAddr = INVALID_NODE_ADDR;
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

  while ((msg = osal_msg_receive(pulseTaskId)))
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
  (void)osal_stop_timerEx(pulseTaskId, PULSE_EVT_DAT);

  if ((DEV_ZB_COORD == devState) || (DEV_ROUTER == devState) || (DEV_END_DEVICE == devState))
  {
    uint16 tmp = NLME_GetCoordShortAddr();
    uint8 dly = TVSA_STG_DAT;

    pulseDat[TVSA_PAR_LSB] = LO_UINT16(tmp);
    pulseDat[TVSA_PAR_MSB] = HI_UINT16(tmp);
    if ((DEV_ROUTER == devState) || (DEV_ZB_COORD == devState))
    {
      pulseDat[TVSA_TYP_IDX] |= 0x80;
    }
    else
    {
      pulseDat[TVSA_TYP_IDX] &= (0xFF ^ 0x80);
    }

#if TVSA_DONGLE_IS_ZC  //MHMS do we need this?
    if (INVALID_NODE_ADDR == tvsaAddr)
    {
      // Assume ZC is the TVSA Dongle until a TVSA_CMD_BEG gives a different address.
      pulseAddr = NWK_PAN_COORD_ADDR;
    }
#endif

    if (INVALID_NODE_ADDR != tvsaAddr)
    {
      if (ZSuccess != osal_start_timerEx(pulseTaskId, PULSE_EVT_DAT, (dly + TVSA_DLY_MIN)))
      {
        (void)osal_set_event(pulseTaskId, PULSE_EVT_DAT);
      }
    }

#if !TVSA_DONGLE
    if (0 == 0)//voltageAtTemp22)
    {
     // HalInitTV();
      (void)osal_cpyExtAddr(pulseDat+PULSE_IEE_IDX, &aExtendedAddress);
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

  pulseBuf[PULSE_SOP_IDX] = PULSE_SOP_VAL;
  pulseBuf[PULSE_ADR_LSB] = LO_UINT16(msg->srcAddr.addr.shortAddr);
  pulseBuf[PULSE_ADR_MSB] = HI_UINT16(msg->srcAddr.addr.shortAddr);

  // 1st byte of message is skipped - CMD is always 0 for data.
  (void)osal_memcpy(pulseBuf+PULSE_DAT_OFF, msg->cmd.Data+1, PULSE_DAT_LEN-1);  //MHMS copies one buffer to another

  for (idx = PULSE_ADR_LSB; idx < PULSE_FCS_IDX; idx++)
  {
    fcs ^= pulseBuf[idx];
  }
  pulseBuf[idx] = fcs;
  
#ifdef TVSA_DEMO

  HalUARTWrite(TVSA_PORT, pulseBuf, TVSA_BUF_LEN);

#else
  
  
  uint8 deviceBPM;
  uint8 deviceVolt;
  uint8 parentAddrLSB;
  uint8 parentAddrMSB;
  uint8 zsensorBuf[15];
  
  parentAddrLSB= pulseBuf[11];
  parentAddrMSB= pulseBuf[12];  
  deviceBPM = pulseBuf[14];
  deviceVolt = 0xFF;
  
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
  zsensorBuf[10]= deviceBPM;
  zsensorBuf[11]= deviceBPM; //deviceVolt;
  
  //Parent Address
  zsensorBuf[12]=parentAddrLSB;
  zsensorBuf[13]=parentAddrMSB;


  //FCS Check on the middle 13 bytes
  zsensorBuf[14] = calcFCS(&zsensorBuf[1], 13 );


  HalUARTWrite(TVSA_PORT, zsensorBuf, 15);
  
  //MHMS USB communication with Pulse sensor Processor application
  /*int16 S
  int16
  int16  
  HalUARTWrite(TVSA_PORT, (pulseBuf+, 15);*/
  
#endif  
 /* sendDataToProcessing('S', Signal)
            sendDataToProcessing('B',BPM);   // send heart rate with a 'B' prefix
        sendDataToProcessing('Q',IBI);  */
        
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
      if (ZSuccess != osal_start_timerEx(pulseTaskId, TVSA_EVT_ANN, TVSA_DLY_ANN))
      {
        (void)osal_set_event(pulseTaskId, TVSA_EVT_ANN);
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



/*  //MHMS Pulse Sensor Functions */ 

/**************************************************************************************************
 * @fn          pulseAppInit
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
void pulseAppInit(uint8 id)
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
//  tvsaDat[TVSA_TYP_IDX] = (uint8)TVSA_DEVICE_ID;
    pulseDat[PULSE_TYP_IDX] = (uint8)PULSE_DEVICE_ID;
#if defined PULSE_SRC_RTG
//  tvsaDat[TVSA_OPT_IDX] = TVSA_OPT_SRC_RTG;
    pulseDat[PULSE_OPT_IDX] = PULSE_OPT_SRC_RTG;
#endif
#endif

  pulseTaskId = id;
  pulseAddr = INVALID_NODE_ADDR;
  (void)afRegister((endPointDesc_t *)&PULSE_epDesc);  //MHMS registers endpoint object
  
  //Initialize Px.y (5.0) to power Pulse sensor
  P5DIR = 0x1; 
  P5OUT = 0x1;
 
}

/**************************************************************************************************
 * @fn          pulseAppEvt
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
uint16 pulseAppEvt(uint8 id, uint16 evts)
{
  uint16 mask = 0;
  (void)id;  //MHMS casts a void to ignore warning for not using variable
  
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
  else if (evts & PULSE_EVT_DAT)
  {
    mask = PULSE_EVT_DAT;
    pulseDataCalc();
  }
  else if (evts & PULSE_EVT_REQ)
  {
    mask = PULSE_EVT_REQ;
    pulseDataReq();
  }
#endif
  else
  {
    mask = evts;  // Discard unknown events - should never happen.
  }

  return (evts ^ mask);  // Return unprocessed events.
}


//MHMS put coord stuff here, recieve func and sys


#if !TVSA_DONGLE  //Group 1
/**************************************************************************************************
 * @fn          pulseDataCalc
 *
 * @brief       This function is called by tvsaAppEvt() to calculate the data for a PULSE report.
 *              The function will called on a 2ms interval and detect whether a pulse is being measured.
 *              If a pulse is determined it will invoke the PulsedataReq interrupt timer (20ms intervals)
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
static void pulseDataCalc(void)
{
  if (INVALID_NODE_ADDR == pulseAddr)
  {
    return;
  }

  if (ZSuccess != osal_start_timerEx(pulseTaskId, PULSE_EVT_DAT, PULSE_DLY_DAT))  //If the timer can't be started set event flag again for it to be service again
  {
    (void)osal_set_event(pulseTaskId, PULSE_EVT_DAT);
  }
  pulseBPM(pulseDat);
  //  HalCalcTV(tvsaDat);
#if TVSA_DATA_CNF
  tvsaDat[TVSA_RTG_IDX] = tvsaCnfErrCnt;
#else
  tvsaDat[TVSA_RTG_IDX] = 0;
#endif
  //osal_set_event(tvsaTaskId, TVSA_EVT_REQ);
  if(QS == true && SUCCESS == osal_set_event(pulseTaskId, PULSE_EVT_REQ)){}  //If pulse is being measured synchronize pulsedatareq event
  
 
}
/**************************************************************************************************
 * @fn          pulseBPM
 *
 * @brief       This function is called by tvsaAppEvt() to calculate the data for a BPM report.
 *
 * input parameters
 *
* uint8 *pulsedata is a pointer to the PULSE over the air payload structure
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void pulseBPM(uint8 *pulsedata)
{
  


int BPM = pulsedata[PULSE_BPM];                         // used to hold the pulse rate
int Signal;                                             // holds the incoming raw data
int IBI = pulsedata[PULSE_IBI];                         // holds the time between beats, the Inter-Beat Interval

//    cli();                                    // disable interrupts while we do this
//    Signal = analogRead(pulsePin);              // read the Pulse Sensor  //MHMS orginal arduino code
//MHMS we will use uint16 HalAdcRead() function and set channel to read and 10 Bit resolution
  Signal = HalAdcRead(HAL_ADC_CHANNEL_7, HAL_ADC_RESOLUTION_10);
  sampleCounter += 2;                         // keep track of the time in mS with this variable
    int Number = (sampleCounter - lastBeatTime);     // monitor the time since the last beat to avoid noise

//  find the peak and trough of the pulse wave
    if(Signal < thresh && Number > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
        if (Signal < T){                        // T is the trough
            T = Signal;                         // keep track of lowest point in pulse wave 
         }
       }
      
    if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
        P = Signal;                             // P is the peak
       }                                        // keep track of highest point in pulse wave
    
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
if (Number > 500){                                   // avoid high frequency noise //increased from 250 to reduce high freq noise
  if ((Signal > thresh) && (Pulse == false) && (Number > (int)(IBI/5)*3) ){        
    Pulse = true;                               // set the Pulse flag when we think there is a pulse
    
    //digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
    //MHMS  could define some external LED or just write to LCD screen "Pulse found"
    // Use void HalLedSet (uint8 led, uint8 mode);
    HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);    //MHMS beat found
    HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);     //MHMS LED on upbeat
    
    IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
    lastBeatTime = sampleCounter;               // keep track of time for next pulse
         
         if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
             firstBeat = false;                 // clear firstBeat flag
             return;                            // IBI value is unreliable so discard it
            }   
         if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
            secondBeat = false;                 // clear secondBeat flag
               for(int i=0; i<=9; i++){         // seed the running total to get a realisitic BPM at startup
                    rate[i] = IBI;                      
                    }
            }
          
    // keep a running total of the last 10 IBI values
    int16 runningTotal = 0; //word runningTotal = 0;                   // clear the runningTotal variable    

    for(int i=0; i<=8; i++){                // shift data in the rate array
          rate[i] = rate[i+1];              // and drop the oldest IBI value 
          runningTotal += rate[i];          // add up the 9 oldest IBI values
        }
        
    rate[9] = IBI;                          // add the latest IBI to the rate array
    runningTotal += rate[9];                // add the latest IBI to runningTotal
    runningTotal /= 10;                     // average the last 10 IBI values 
    BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
    QS = true;                              // set Quantified Self flag //MHMS we will use this to flag other event to transmit data over network
    
    HalLcdWriteStringValue("BPM:",BPM, 10, 6); //MHMS display BPM on LCD screen
    // QS FLAG IS NOT CLEARED INSIDE THIS ISR
    }                       
}

  if (Signal < thresh && Pulse == true){     // when the values are going down, the beat is over
      //digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
     //MHMS  could define some external LED or just write to LCD screen "Pulse Not found"
      HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);
      
      Pulse = false;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
      P = thresh;                            // reset these for next time
      T = thresh;
     }
  
  if (Number > 2500){                             // if 2.5 seconds go by without a beat
      HalLedSet (HAL_LED_2, HAL_LED_MODE_ON);  //MHMS No beat found
      thresh = 512;                          // set thresh default
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = true;                      // set these to avoid noise
      secondBeat = true;                     // when we get the heartbeat back
     }

//MHMS Loading 16 bit results into 8 bit blocks for pulsedata array              
pulsedata[PULSE_BPM] = (uint8)((BPM & 0x00FF));
pulsedata[PULSE_RAW_MSB] = (uint8)((Signal >> 8));
pulsedata[PULSE_RAW_LSB] = (uint8)((Signal & 0x00FF));
pulsedata[PULSE_IBI] = (uint8)((IBI & 0x00FF));

pulsedata[PULSE_BPM_CHAR] = 'B';
pulsedata[PULSE_RAW_CHAR] = 'Q';
pulsedata[PULSE_IBI_CHAR] = 'S';


  //sei();                                     // enable interrupts when youre done!
}// end isr

/**************************************************************************************************
 * @fn          pulseDataReq
 *
 * @brief       This function is called by tvsaAppEvt() to send a PULSE data report. When it is detected that
 *              a pulse is found (QS flag is set) this function will start to transfer BPM, IBI, and raw Signal
 *              data over the air to the coordinator at 20ms intervals. When there is no BPM detected
 *              this function will stop sending information over the air to the coordinator.
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
static void pulseDataReq(void)
{
  afAddrType_t addr;                    //AF address stucture defined for info on the destination Endpoint object that data will be sent to
  
  addr.addr.shortAddr = pulseAddr;      //loading short address (16-bit) with pulse address
  addr.addrMode = afAddr16Bit;          //Set to directly sent to a node
  addr.endPoint = PULSE_ENDPOINT;       //Sets the endpoint of the final destination (coordinator?)

  if (afStatus_SUCCESS != AF_DataRequest(&addr, (endPointDesc_t *)&PULSE_epDesc, PULSE_CLUSTER_ID,
                                          PULSE_DAT_LEN, pulseDat, &pulseTSN,
                                          AF_DISCV_ROUTE
#if TVSA_DATA_CNF
                                        | AF_ACK_REQUEST
#endif
                                         ,AF_DEFAULT_RADIUS))  //MHMS
  { //if data transfer is unsuccessful place event immediately back into queue to attempt to send again
        osal_set_event(pulseTaskId, PULSE_EVT_REQ);
  }
  else
  {
    tvsaCnt++;
  }
  if(QS == true){
    osal_start_timerEx(pulseTaskId, PULSE_EVT_REQ, PULSE_DLY_DATAREQ);  //send next Pulse data report in 20ms
    QS = false;     //Clears Pulse measurement quantifier flag 
  }
}
#endif //Group 1