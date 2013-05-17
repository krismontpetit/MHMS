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

#include "zap_app.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

// Constants for Pulse Sensor

static const cId_t MHMS_ClusterList[MHMS_CLUSTER_CNT] =
{
  MHMS_CLUSTER_ID
};

static const SimpleDescriptionFormat_t MHMS_SimpleDesc =
{
  MHMS_ENDPOINT,
  MHMS_PROFILE_ID,
  MHMS_DEVICE_ID,
  MHMS_DEVICE_VERSION,
  MHMS_FLAGS,
  MHMS_CLUSTER_CNT,
  (cId_t *)MHMS_ClusterList,
  MHMS_CLUSTER_CNT,
  (cId_t *)MHMS_ClusterList
};

static const endPointDesc_t MHMS_epDesc=
{
  MHMS_ENDPOINT,
  &MHMSTaskId,
  (SimpleDescriptionFormat_t *)&MHMS_SimpleDesc,
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
uint8 MHMSCnfErrCnt;
#endif

//MHMS  Global Variables
uint8 MHMSTaskId;

//Flags to check if a zapsync() was detected
bool syncAttempted=FALSE;
bool PanEstablishedwithRouter = FALSE;

//Flag for enabling and disabling direct USB link for communicating with Pulse sketch program. 
uint8 EnableUSBPulseSketchTxFlag = 0;   


/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

// Network address of the TVSA Dongle.

static uint16 MHMSAddr;
// Report counter.
static uint16 MHMSCnt;  //MHMS Question what is this for?
// ZigBee-required packet transaction sequence number in calls to AF_DataRequest().

static uint8 MHMSTSN;

//Data arrays for Over the air data tx and RX
static uint8 MHMSBuf[MHMS_BUF_LEN];  //MHMS buffer used for recived over the air data
static uint8 MHMSDat[MHMS_DAT_LEN];  //MHMS define data array length for Pulse sensor
static uint8 TestDatTx[MHMS_TEST_PAYLOAD_LEN];
static uint8 TestRxBuffer[MHMS_TEST_BUFF_LEN];

//Syncronization Flags and other flags
static bool MHMSEvtDat_sync = FALSE;  
static bool MHMSEvtReq_sync;
static bool MHMSEvtCheckin_sync;  

static bool dev_gateway = FALSE;


volatile bool QS = FALSE;        // becomes TRUE when Arduino finds a beat.  



/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void MHMSAfMsgRx(afIncomingMSGPacket_t *msg);
static void MHMSSysEvtMsg(void);


static void pulseBPM(uint8 *pulsedata);  //MHMS Pulse calculation function
static void pulseDataCalc(void);
static void MHMSDataReq(void);
static void MHMSZdoStateChange(void);

static void MHMSNodeCheckIn(void);
static void TestPayloadTx(void);

static void MHMSAnnce(void);
static void MHMSDataRx(afIncomingMSGPacket_t *msg);
static void MHMSTestingDataRx(afIncomingMSGPacket_t *msg);

static void MHMSUartRx(uint8 port, uint8 event);


#ifndef TVSA_DEMO  //MHMS Question do we need this?
static uint8 calcFCS(uint8 *pBuf, uint8 len);
static void sysPingRsp(void);
#endif

/*  //MHMS Pulse Sensor Functions */ 

/**************************************************************************************************
 * @fn          MHMSAppInit
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
void MHMSAppInit(uint8 id)
{

  halUARTCfg_t uartConfig;

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  

  //uartConfig.baudRate             = HAL_UART_BR_38400;        //MHMS This baud rate is required to communicate with Zigbee Sensor Monitor
  uartConfig.baudRate             = HAL_UART_BR_115200;         //MHMS This baud rate is required to communicate with the Pulse processing program on PC

  
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 16;                // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 32;                // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 254;               // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6;                 // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = MHMSUartRx;
  HalUARTOpen(MHMS_PORT, &uartConfig);

    MHMSDat[MHMS_TYP_IDX] = (uint8)MHMS_DEVICE_ID;
    TestDatTx[MHMS_TEST_PAYLOAD_LEN - 3] = (uint8)MHMS_DEVICE_ID;
#if defined MHMS_SRC_RTG

    MHMSDat[MHMS_OPT_IDX] = MHMS_OPT_SRC_RTG;
    TestDatTx[MHMS_TEST_PAYLOAD_LEN - 1] = MHMS_OPT_SRC_RTG;
#endif

  MHMSTaskId = id;                                    
  MHMSAddr = INVALID_NODE_ADDR;
  (void)afRegister((endPointDesc_t *)&MHMS_epDesc);  //MHMS registers endpoint object
  
  //Initialize Px.y (5.0) to power Pulse sensor
  P5DIR = 0x1;  //Set IO direction as output
  P5OUT = 0x1;  //Set output to high
 
  
  //Test
P4DIR =  0x1; 
P4OUT = 0x1;
  
  //Setup ADC reference 
  REFCTL0 = REFVSEL_2;  // REF Reference Voltage Level Select 2.5V
}

/**************************************************************************************************
 * @fn          MHMSAppEvt
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
uint16 MHMSAppEvt(uint8 id, uint16 evts)
{
  uint16 mask = 0;
  (void)id;  //MHMS casts a void to ignore warning for not using variable
  
  if (evts & SYS_EVENT_MSG)
  {
    mask = SYS_EVENT_MSG;
    MHMSSysEvtMsg();
  }

  else if (evts & MHMS_EVT_ANN)
  {
    mask = MHMS_EVT_ANN;
   MHMSAnnce();
  }

  else if (evts & MHMS_EVT_DAT)
  {
    mask = MHMS_EVT_DAT;
    pulseDataCalc();
  }
  else if (evts & MHMS_EVT_REQ)
  {
    mask = MHMS_EVT_REQ;
    MHMSDataReq();
  }
    else if (evts & MHMS_EVT_CHECKIN)
  {
    mask = MHMS_EVT_CHECKIN;
    MHMSNodeCheckIn();
  }

else if (evts & TEST_EVT_PAYLOAD_TX)
  {
    mask = TEST_EVT_PAYLOAD_TX;
    TestPayloadTx();
  }
  else
  {
    mask = evts;  // Discard unknown events - should never happen.
  }

  return (evts ^ mask);  // Return unprocessed events.
}


//MHMS put coord stuff here, recieve func and sys



/**************************************************************************************************
 * @fn          MHMSSysEvtMsg
 *
 * @brief       This function is called by MHMSAppEvt() to process all of the pending OSAL messages.
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
static void MHMSSysEvtMsg(void)
{
  uint8 *msg;

  while ((msg = osal_msg_receive(MHMSTaskId)))
  {
    switch (*msg)
    {
#if TVSA_DATA_CNF  //MHMS Question what is this for?
    case AF_DATA_CONFIRM_CMD:
      if (ZSuccess != ((afDataConfirm_t *)msg)->hdr.status)
      {
        if (0 == ++MHMSCnfErrCnt)
        {
          MHMSCnfErrCnt = 255;
        }
      }
      break;
#endif

    case AF_INCOMING_MSG_CMD:  //MHMS this a router processing the incomming command from the coordinator
      MHMSAfMsgRx((afIncomingMSGPacket_t *)msg);
      break;

    case ZDO_STATE_CHANGE:
      MHMSZdoStateChange();
      break;

    default:
      break;
    }

    (void)osal_msg_deallocate(msg);  // Receiving task is responsible for releasing the memory.
  }
}

/**************************************************************************************************
 * @fn          MHMSAfMsgRx
 *
 * @brief       This function is called by MHMSSysEvtMsg() to process an incoming AF message.
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
static void MHMSAfMsgRx(afIncomingMSGPacket_t *msg)
{
  uint8 *buf = msg->cmd.Data;
  
  if(PanEstablishedwithRouter == FALSE & devState == DEV_ZB_COORD){
    if(zap_set_logicalType(ZG_DEVICETYPE_ROUTER)){
      HalLcdWriteString("Ready",HAL_LCD_LINE_7);
      }
    PanEstablishedwithRouter = TRUE;
        
    if (ZSuccess != osal_start_timerEx(MHMSTaskId, MHMS_EVT_ANN, MHMS_DLY_ANN))
      {
        (void)osal_set_event(MHMSTaskId, MHMS_EVT_ANN);
      }
  }
  else if(PanEstablishedwithRouter == FALSE & devState == DEV_ROUTER){
    HalLcdWriteString("Ready",HAL_LCD_LINE_7);
    PanEstablishedwithRouter = TRUE;
  }
  else {
  switch (buf[MHMS_CMD_IDX])
  {

  case MHMS_CMD_DAT:  //Nodes will send this by default
    MHMSDataRx(msg);
    break;

  case MHMS_CMD_BEG:
    if (INVALID_NODE_ADDR == MHMSAddr)
    {
      if(dev_gateway == FALSE){
      NLME_SetPollRate(0);
      if(MHMSEvtDat_sync == FALSE){
      (void)osal_set_event(MHMSTaskId, MHMS_EVT_DAT);           //Sync Pulsedat event operation
       }
      }
    }
    MHMSAddr = msg->srcAddr.addr.shortAddr; // BUILD_UINT16(buf[MHMS_ADR_LSB], buf[MHMS_ADR_MSB]);
    break;

  case MHMS_CMD_END:
    NLME_SetPollRate(POLL_RATE);
    MHMSAddr = INVALID_NODE_ADDR;
    break;

  case MHMS_CMD_DAT_TEST:  //This is used for testing different payload sizes. Not used in normal operation
    MHMSTestingDataRx(msg);
    break;

  default:
    break;
  }
} //end else
}
/**************************************************************************************************
 * @fn          MHMSDataRx
 *
 * @brief       This function is called by MHMSAfMsgRx() to process incoming MHMS data.
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
static void MHMSDataRx(afIncomingMSGPacket_t *msg)
{
  uint8 fcs = 0, idx;
    
  // Last announce broadcast to stop must have expired before a parent could forward to a ZED.
  if (INVALID_NODE_ADDR == MHMSAddr)
  {
    (void)osal_set_event(MHMSTaskId, MHMS_EVT_ANN);
  }

  MHMSBuf[MHMS_SOP_IDX] = MHMS_SOP_VAL;
  MHMSBuf[MHMS_ADR_LSB] = LO_UINT16(msg->srcAddr.addr.shortAddr);
  MHMSBuf[MHMS_ADR_MSB] = HI_UINT16(msg->srcAddr.addr.shortAddr);

  // 1st byte of message is skipped - CMD is always 0 for data.
  (void)osal_memcpy(MHMSBuf+MHMS_DAT_OFF, msg->cmd.Data+1, MHMS_DAT_LEN-1);  //MHMS copies one buffer to another

  for (idx = MHMS_ADR_LSB; idx < MHMS_FCS_IDX; idx++)
  {
    fcs ^= MHMSBuf[idx];
  }
  MHMSBuf[idx] = fcs;
  
  uint8 deviceBPM;
  uint8 parentAddrLSB;
  uint8 parentAddrMSB;
  uint8 zsensorBuf[15];
  
  parentAddrLSB= MHMSBuf[11];
  parentAddrMSB= MHMSBuf[12];  
  
  if(MHMSBuf[13] == CHECK_IN_INACTIVE){
  deviceBPM = MHMSBuf[15];
  }
  else{
  deviceBPM = 5;
  }
  
  //Start of Frame Delimiter
  zsensorBuf[0]=0xFE;
  zsensorBuf[1]=10;
  zsensorBuf[2]=LO_UINT16(0x8746);
  zsensorBuf[3]=HI_UINT16(0x8746);
  
  //Source Address
  zsensorBuf[4] = LO_UINT16(msg->srcAddr.addr.shortAddr);
  zsensorBuf[5] = HI_UINT16(msg->srcAddr.addr.shortAddr);
  
  zsensorBuf[6]=LO_UINT16(2);  //MHMS are 6 - 9 being used for anything?
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

  HalUARTWrite(MHMS_PORT, zsensorBuf, 15);  //For communicating with the Zigbee sensor Monitor
  
/*  //MHMS USB communication with Pulse sensor Processor application
   
  
  uint8 BPMBuf[7] = {'B',0,0,0,10,13};
  uint8 IBIBuf[7] = {'Q',0,0,0,10,13};
  uint8 SignalBuf[7] = {'S',0,0,0,10,13};
  
  //conversion Signal Dec to ASCII
  uint16 temp = (BUILD_UINT16(MHMSBuf[16], MHMSBuf[17])) - 400;
  if(temp > 999){
    SignalBuf[1] = '9';
    SignalBuf[2] = '9';
    SignalBuf[3] = '9';
  }
  else { 
    SignalBuf[1] = (uint8)((temp/100)+ 48);
    SignalBuf[2] = (uint8)((((temp%100) - (temp % 100)%10)/10) + 48);
    SignalBuf[3] = (uint8)(((temp % 100)%10)+ 48);
  }
  
  //conversion BPM Dec to ASCII
  temp = (uint16)MHMSBuf[14];
  BPMBuf[1] = (uint8)((temp/100)+ 48);
  BPMBuf[2] = (uint8)((((temp%100) - (temp % 100)%10)/10) + 48);
  BPMBuf[3] = (uint8)(((temp % 100)%10)+ 48);
  
  //conversion IBI Dec to ASCII
  temp = (uint16)MHMSBuf[19];
  IBIBuf[1] = (uint8)((temp/100)+ 48);
  IBIBuf[2] = (uint8)((((temp%100) - (temp % 100)%10)/10) + 48);
  IBIBuf[3] = (uint8)(((temp % 100)%10)+ 48);
   

  //HalUARTWrite(MHMS_PORT, SignalBuf, 6);
 // HalUARTWrite(MHMS_PORT, BPMBuf, 6);
 // HalUARTWrite(MHMS_PORT, IBIBuf, 6);
*/

}

/**************************************************************************************************
 * @fn          MHMSTestingDataRx
 *
 * @brief       This function is called by MHMSAfMsgRx() to process incoming data  This is not used in 
 *              normal operation.  This is used for testing system with differnt payload sizes.
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
static void MHMSTestingDataRx(afIncomingMSGPacket_t *msg)
{
   uint8 fcs = 0, idx;
   int8 Rssi;
   //Convert LQI to RSSI
   Rssi =(int8)(((msg->LinkQuality * 97)/0xFF) -87); 
   //uint8 *TestRxBuffer;
   //TestRxBuffer = (uint8*)osal_mem_alloc(MHMS_TEST_BUFF_LEN);
    
  

  // Last announce broadcast to stop must have expired before a parent could forward to a ZED.
  if (INVALID_NODE_ADDR == MHMSAddr)
  {
    (void)osal_set_event(MHMSTaskId, MHMS_EVT_ANN);
  }

  TestRxBuffer[MHMS_SOP_IDX] = MHMS_SOP_VAL;
  TestRxBuffer[MHMS_ADR_LSB] = LO_UINT16(msg->srcAddr.addr.shortAddr);
  TestRxBuffer[MHMS_ADR_MSB] = HI_UINT16(msg->srcAddr.addr.shortAddr);

  // 1st byte of message is skipped - CMD is always 0 for data.
  (void)osal_memcpy(TestRxBuffer+MHMS_DAT_OFF, msg->cmd.Data+1, MHMS_TEST_PAYLOAD_LEN-1);  //MHMS copies one buffer to another

  for (idx = MHMS_ADR_LSB; idx < MHMS_FCS_IDX; idx++)
  {
    fcs ^= TestRxBuffer[idx];
  }
  TestRxBuffer[idx] = fcs;
  
  uint8 PktSeqNum; 
  
  uint8 parentAddrLSB;
  uint8 parentAddrMSB;
  uint8 tsensorBuf[15];
  
  parentAddrLSB= TestRxBuffer[11];
  parentAddrMSB= TestRxBuffer[12];  
  
  PktSeqNum = TestRxBuffer[13];
  
  //Start of Frame Delimiter
  tsensorBuf[0]=0xFE;
  tsensorBuf[1]=10;
  tsensorBuf[2]=LO_UINT16(0x8746);
  tsensorBuf[3]=HI_UINT16(0x8746);
  
  //Source Address
  tsensorBuf[4] = LO_UINT16(msg->srcAddr.addr.shortAddr);
  tsensorBuf[5] = HI_UINT16(msg->srcAddr.addr.shortAddr);
  
  tsensorBuf[6]=LO_UINT16(2);  //MHMS are 6 - 9 being used for anything?
  tsensorBuf[7]=HI_UINT16(2);
  tsensorBuf[8]=LO_UINT16(4);
  tsensorBuf[9]=HI_UINT16(4);
  
  //Temperature and Voltage Data
  tsensorBuf[10]= Rssi;
  tsensorBuf[11]= PktSeqNum; 
  
  //Parent Address
  tsensorBuf[12]= parentAddrLSB;
  tsensorBuf[13]= parentAddrMSB;


  //FCS Check on the middle 13 bytes
  tsensorBuf[14] = calcFCS(&tsensorBuf[1], 13 );

  HalUARTWrite(MHMS_PORT, tsensorBuf, 15);  //For communicating with the Zigbee sensor Monitor
  
  //osal_mem_free(TestRxBuffer);
}


/**************************************************************************************************
 * @fn          MHMSZdoStateChange 
 *
 * @brief       This function is called by MHMSSysEvtMsg() for a ZDO_STATE_CHANGE message.
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
static void MHMSZdoStateChange(void)
{
  if(DEV_ZB_COORD == devState) 
  {
    (void)osal_stop_timerEx(MHMSTaskId, MHMS_EVT_ANN);

    if ((DEV_ZB_COORD == devState) || (DEV_ROUTER == devState) || (DEV_END_DEVICE == devState))
    {

      if (INVALID_NODE_ADDR == MHMSAddr)
      {
      MHMSAddr = NWK_PAN_COORD_ADDR;
      }

      if (INVALID_NODE_ADDR != MHMSAddr)
      {
        if (ZSuccess != osal_start_timerEx(MHMSTaskId, MHMS_EVT_ANN, MHMS_DLY_ANN))
        {
          (void)osal_set_event(MHMSTaskId, MHMS_EVT_ANN);
        }
      }
    }
  }
  else if ((DEV_ROUTER == devState) || (DEV_END_DEVICE == devState))
  {
    (void)osal_stop_timerEx(MHMSTaskId, MHMS_EVT_DAT);
    MHMSEvtDat_sync = FALSE; //allow node to respond to Anounce commands to begin pulse collection

        if ((DEV_ROUTER == devState) || (DEV_END_DEVICE == devState)) //
        {
          uint16 tmp = NLME_GetCoordShortAddr();
          uint8 dly = MHMS_STG_DAT;

          MHMSDat[MHMS_PAR_LSB] = LO_UINT16(tmp);
          MHMSDat[MHMS_PAR_MSB] = HI_UINT16(tmp);
          
          TestDatTx[MHMS_PAR_LSB] = LO_UINT16(tmp);
          TestDatTx[MHMS_PAR_MSB] = HI_UINT16(tmp);
          if ((DEV_ROUTER == devState) || (DEV_ZB_COORD == devState))
          {
            MHMSDat[MHMS_TYP_IDX] |= 0x80;
            TestDatTx[MHMS_TEST_PAYLOAD_LEN-3] |= 0x80;

          }
          else
          {
            MHMSDat[MHMS_TYP_IDX] &= (0xFF ^ 0x80);
            TestDatTx[MHMS_TEST_PAYLOAD_LEN-3] &= (0xFF ^ 0x80);

          }

      #if TVSA_DONGLE_IS_ZC
          if (INVALID_NODE_ADDR == MHMSAddr)
          {

            MHMSAddr = NWK_PAN_COORD_ADDR;
          }
      #endif

          if (INVALID_NODE_ADDR != MHMSAddr && dev_gateway == FALSE)
          {
            if (ZSuccess != osal_start_timerEx(MHMSTaskId, MHMS_EVT_DAT, (dly + MHMS_DLY_MIN)))
            {
              (void)osal_set_event(MHMSTaskId, MHMS_EVT_DAT);
            }
          }


          if (0 == 0)
          {
            (void)osal_cpyExtAddr(MHMSDat+MHMS_IEE_IDX, &aExtendedAddress);
            (void)osal_cpyExtAddr(TestDatTx+MHMS_IEE_IDX, &aExtendedAddress);
          }
        }
  }

#if defined LCD_SUPPORTED
  HalLcdWriteValue(devState, 10, HAL_LCD_LINE_4);
#endif
}


/**************************************************************************************************
 * @fn          MHMSAnnce
 *
 * @brief       This function is called by MHMSAppEvt() to send a TVSA announce to start or stop.
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
static void MHMSAnnce(void)
{
  uint8 msg[3];
  afAddrType_t addr;
  
  addr.addr.shortAddr = NWK_BROADCAST_SHORTADDR_DEVALL;
  addr.addrMode = afAddrBroadcast;
  addr.endPoint = MHMS_ENDPOINT;

  if (INVALID_NODE_ADDR != MHMSAddr)
  {
    msg[MHMS_CMD_IDX] = MHMS_CMD_BEG;
    if (ZSuccess != osal_start_timerEx(MHMSTaskId, MHMS_EVT_ANN, MHMS_DLY_ANN))
    {
      (void)osal_set_event(MHMSTaskId, MHMS_EVT_ANN);
    }
  }
  else
  {
    msg[MHMS_CMD_IDX] = MHMS_CMD_END;
  }

  msg[MHMS_ADR_LSB] = LO_UINT16(MHMSAddr);
  msg[MHMS_ADR_MSB] = HI_UINT16(MHMSAddr);

  if (afStatus_SUCCESS != AF_DataRequest(&addr, (endPointDesc_t *)&MHMS_epDesc, MHMS_CLUSTER_ID,
                                          3, msg, &MHMSTSN, AF_TX_OPTIONS_NONE, AF_DEFAULT_RADIUS))
  {
    osal_set_event(MHMSTaskId, MHMS_EVT_ANN);
  }
  else
  {
    MHMSCnt++;
  }
}


/**************************************************************************************************
 * @fn          pulseDataCalc
 *
 * @brief       This function is called by MHMSAppEvt() to calculate the data for a MHMS report.
 *              The function will called on a 2ms interval and detect whether a pulse is being measured.
 *              If a pulse is determined it will invoke the MHMSdataReq interrupt timer (20ms intervals)
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
  MHMSEvtDat_sync = TRUE;      //Pulse Data collection has been synced, no need to respond to annc commands to set EVT
  static int Pulse_sketch_count = 0;
  
  if (INVALID_NODE_ADDR == MHMSAddr)
  {
    return;
  }
  
  if (ZSuccess != osal_start_timerEx(MHMSTaskId, MHMS_EVT_DAT, MHMS_DLY_DAT))  //If the timer can't be started set event flag again for it to be service again
  {
    (void)osal_set_event(MHMSTaskId, MHMS_EVT_DAT);
  }
  pulseBPM(MHMSDat);  //Function to collect/calculate Pulse

#if TVSA_DATA_CNF
  MHMSDat[MHMS_RTG_IDX] = MHMSCnfErrCnt;
#else
  MHMSDat[MHMS_RTG_IDX] = 0;
#endif
  
if(QS == TRUE && MHMSEvtReq_sync == FALSE){//If pulse is being measured synchronize MHMSdatareq event
  osal_set_event(MHMSTaskId, MHMS_EVT_REQ);
  MHMSEvtReq_sync = TRUE;
    }  
 else if(QS == FALSE && MHMSEvtCheckin_sync == FALSE)
  {
  osal_set_event(MHMSTaskId, MHMS_EVT_CHECKIN);  //Since no pulse data is being collected, just send check in data to gateway
  MHMSEvtCheckin_sync = TRUE;
  }
 
  Pulse_sketch_count++;


  //MHMS USB communication with Pulse sensor Processor application
  
  if(EnableUSBPulseSketchTxFlag == TRUE && Pulse_sketch_count >= 10){
  
  uint8 BPMBuf[7] = {'B',0,0,0,10,13};
  uint8 IBIBuf[7] = {'Q',0,0,0,10,13};
  uint8 SignalBuf[7] = {'S',0,0,0,10,13};
  
  Pulse_sketch_count = 0;  //reset count
 
  //conversion Signal Dec to ASCII
  uint16 temp = (BUILD_UINT16(MHMSDat[15], MHMSDat[16])) - 400;
  if(temp > 999){
    SignalBuf[1] = '9';
    SignalBuf[2] = '9';
    SignalBuf[3] = '9';
  }
  else { 
    SignalBuf[1] = (uint8)((temp/100)+ 48);
    SignalBuf[2] = (uint8)((((temp%100) - (temp % 100)%10)/10) + 48);
    SignalBuf[3] = (uint8)(((temp % 100)%10)+ 48);
  }
  
  //conversion BPM Dec to ASCII
  temp = (uint16)MHMSDat[13];
  BPMBuf[1] = (uint8)((temp/100)+ 48);
  BPMBuf[2] = (uint8)((((temp%100) - (temp % 100)%10)/10) + 48);
  BPMBuf[3] = (uint8)(((temp % 100)%10)+ 48);
  
  //conversion IBI Dec to ASCII
  temp = (uint16)MHMSDat[18];
  IBIBuf[1] = (uint8)((temp/100)+ 48);
  IBIBuf[2] = (uint8)((((temp%100) - (temp % 100)%10)/10) + 48);
  IBIBuf[3] = (uint8)(((temp % 100)%10)+ 48);
   
  
  HalUARTWrite(0, SignalBuf, 6);
  HalUARTWrite(0, BPMBuf, 6);
  HalUARTWrite(0, IBIBuf, 6);
}


}

/**************************************************************************************************
 * @fn          pulseBPM
 *
 * @brief       This function is called by pulseDataCalc().  This function contains the main algorithm for 
 *              pulse calculation that originally develop.
 *              
 *
 * input parameters
 *
 * Pointer to the MHMSdata array that will be sent over the air.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************/
static void pulseBPM(uint8 *pulsedata)
{

//MHMS 
//MHMS From arduino interrupt
// these variables are volatile because they are used during the interrupt service routine!
//MHMS From Arduino 1.1
static  int BPM;                        // used to hold the pulse rate
static  int Signal;                     // holds the incoming raw data
static  int IBI = 600;                  // holds the time between beats, the Inter-Beat Interval
static  bool Pulse = FALSE;             // TRUE when pulse wave is high, FALSE when it's low  
  
static  int rate[10];                    // used to hold last ten IBI values
static  uint32 sampleCounter = 0;          // used to determine pulse timing
static  uint32 lastBeatTime = 0;           // used to find the inter beat interval
static  int P = 512;                      // used to find peak in pulse wave
static  int T = 512;                     // used to find trough in pulse wave
static  int thresh = 512;                // used to find instant moment of heart beat
static  int amp = 100;                   // used to hold amplitude of pulse waveform
static  bool firstBeat = TRUE;        // used to seed rate array so we startup with reasonable BPM
static  bool secondBeat = TRUE;       // used to seed rate array so we startup with reasonable BPM
BPM = pulsedata[MHMS_BPM];                         // used to hold the pulse rate
IBI = pulsedata[MHMS_IBI];                         // holds the time between beats, the Inter-Beat Interval

     P4OUT = 0x0;

//    cli();                                            // disable interrupts while we do this
//    Signal = analogRead(pulsePin);                    // read the Pulse Sensor  //MHMS orginal arduino code

//MHMS using HAL layer API to set channel to read and 10 Bit resolution
  Signal = HalAdcRead(HAL_ADC_CHANNEL_7, HAL_ADC_RESOLUTION_10);
  
  sampleCounter +=2;                                   // keep track of the time in mS with this variable
  int Number = (sampleCounter - lastBeatTime);          // monitor the time since the last beat to avoid noise

//  find the peak and trough of the pulse wave
    if(Signal < thresh && Number > (IBI/5)*3){          // avoid dichrotic noise by waiting 3/5 of last IBI
        if (Signal < T){                                // T is the trough
            T = Signal;                                 // keep track of lowest point in pulse wave 
         }
       }
      
    if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
        P = Signal;                             // P is the peak
       }                                        // keep track of highest point in pulse wave
    
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
if (Number > 500){                                   // avoid high frequency noise //MHMS increased from 250 to 500 to reduce high freq noise
  if ((Signal > thresh) && (Pulse == FALSE) && (Number > (int)(IBI/5)*3) ){        
    Pulse = TRUE;                               // set the Pulse flag when we think there is a pulse
    
    //MHMS  could define some external LED or just write to LCD screen "Pulse found"
    HalLedSet (HAL_LED_2, HAL_LED_MODE_OFF);    //MHMS beat found
    HalLedSet (HAL_LED_1, HAL_LED_MODE_ON);     //MHMS LED on during upbeat
    
    IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
    lastBeatTime = sampleCounter;               // keep track of time for next pulse
         
         if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
             firstBeat = FALSE;                 // clear firstBeat flag
             return;                            // IBI value is unreliable so discard it
            }   
         if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
            secondBeat = FALSE;                 // clear secondBeat flag
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
    QS = TRUE;                              // set Quantified Self flag //MHMS we will use this to flag other event to transmit data over network
    

    HalLcdWriteStringValue("BPM:",BPM, 10, HAL_LCD_LINE_5); //MHMS display BPM on LCD screen

    }                       
}

  if (Signal < thresh && Pulse == TRUE){     // when the values are going down, the beat is over
      //digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
     //MHMS  could define some external LED or just write to LCD screen "Pulse Not found"
      HalLedSet (HAL_LED_1, HAL_LED_MODE_OFF);
      
      Pulse = FALSE;                         // reset the Pulse flag so we can do it again
      amp = P - T;                           // get amplitude of the pulse wave
      thresh = amp/2 + T + 100;              // set thresh at 50% of the amplitude  //MHMS offset up by 100 to ignore small flucuations due to noise
      P = thresh;                            // reset these for next time
      T = thresh;
     }
  
  if (Number > 2500){                        // if 2.5 seconds go by without a beat
      HalLedSet (HAL_LED_2, HAL_LED_MODE_ON);// MHMS Indicate that no beat found
      thresh = 512 + 100;                    // set thresh default //MHMS offset up by 100 to ignore small flucuations due to noise
      P = 512;                               // set P default
      T = 512;                               // set T default
      lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
      firstBeat = TRUE;                      // set these to avoid noise
      secondBeat = TRUE;                     // when we get the heartbeat back
      QS = FALSE;                            // Clears Pulse measurement quantifier flag so no data  is sent over the air
     }

//MHMS Loading 16 bit results into 8 bit blocks for pulsedata array              
pulsedata[MHMS_BPM] = (uint8)((BPM & 0x00FF));
pulsedata[MHMS_RAW_MSB] = (uint8)((Signal >> 8));
pulsedata[MHMS_RAW_LSB] = (uint8)((Signal & 0x00FF));
pulsedata[MHMS_IBI] = (uint8)((IBI & 0x00FF));

pulsedata[MHMS_BPM_CHAR] = 'B';
pulsedata[MHMS_RAW_CHAR] = 'S';
pulsedata[MHMS_IBI_CHAR] = 'Q';

     P4OUT = 0x1;
  //sei();                                     // enable interrupts when youre done!
}// end isr

/**************************************************************************************************
 * @fn          MHMSDataReq
 *
 * @brief       This function is called by MHMSAppEvt() to send a MHMS data report. When it is detected that
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
static void MHMSDataReq(void)
{
  static bool MHMSDataReqFlag;
  MHMSDataReqFlag = FALSE;
  afAddrType_t addr;                    //AF address stucture defined for info on the destination Endpoint object that data will be sent to
  
  osal_stop_timerEx(MHMSTaskId, MHMS_EVT_CHECKIN);  //Node is collecting data and sending it to coordinator so turn off check in event
  MHMSEvtCheckin_sync = FALSE;  
  
  MHMSDat[MHMS_CHECK_IN] = CHECK_IN_INACTIVE;   //Flag is off and will notify coordinator that node is not sending check in data
 
  addr.addr.shortAddr = MHMSAddr;      //loading short address (16-bit) with pulse address
  addr.addrMode = afAddr16Bit;          //Set to directly sent to a node
  addr.endPoint = MHMS_ENDPOINT;       //Sets the endpoint of the final destination (coordinator?)

  if (afStatus_SUCCESS != AF_DataRequest(&addr, (endPointDesc_t *)&MHMS_epDesc, MHMS_CLUSTER_ID,
                                          MHMS_DAT_LEN, MHMSDat, &MHMSTSN,
                                          AF_DISCV_ROUTE,AF_DEFAULT_RADIUS)) 
  { //if data transfer is unsuccessful place event immediately back into queue to attempt to send again
        osal_set_event(MHMSTaskId, MHMS_EVT_REQ);
  }
  
  if((QS == TRUE) && (MHMSDataReqFlag == FALSE)){
    osal_start_timerEx(MHMSTaskId, MHMS_EVT_REQ, MHMS_DLY_DATAREQ);  //send next Pulse data report in 500ms
    MHMSDataReqFlag = TRUE;  //to prevent restarting of timer if existing already running
     
  }
}


/**************************************************************************************************
 * @fn          MHMSNodeCheckIn
 *
 * @brief       This function is called by the MHMSAppEvt() function.  
 *
 * input parameters
 *
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */

static void MHMSNodeCheckIn(void)
{

  static bool Flag;
  afAddrType_t addr;                    //AF address stucture defined for info on the destination Endpoint object that data will be sent to
  
  osal_stop_timerEx(MHMSTaskId, MHMS_EVT_REQ); //Stop MHMSDataReq() task since no pulse data is being measured
  MHMSEvtReq_sync = FALSE;
  Flag = FALSE;
  addr.addr.shortAddr = MHMSAddr;      //loading short address (16-bit) with pulse address
  addr.addrMode = afAddr16Bit;          //Set to directly sent to a node
  addr.endPoint = MHMS_ENDPOINT;       //Sets the endpoint of the final destination (coordinator?)

  MHMSDat[MHMS_CHECK_IN] = CHECK_IN_ACTIVE;   //Flag is set and will notify coordinator that node is currently sending check in data
  HalLcdWriteString("BPMsensor Inacti",HAL_LCD_LINE_5);

  if (afStatus_SUCCESS != AF_DataRequest(&addr, (endPointDesc_t *)&MHMS_epDesc, MHMS_CLUSTER_ID,
                                          MHMS_DAT_LEN, MHMSDat, &MHMSTSN, AF_DISCV_ROUTE,AF_DEFAULT_RADIUS))
    { //if data transfer is unsuccessful place event immediately back into queue to attempt to send again
        osal_set_event(MHMSTaskId, MHMS_EVT_CHECKIN);
  }
  else
  {
    MHMSCnt++;
  }

  if((QS == FALSE) && (Flag == FALSE)){
    osal_start_timerEx(MHMSTaskId, MHMS_EVT_CHECKIN, MHMS_DLY_CHECKIN);  //send check in dummy packet every 10 seconds
    Flag = TRUE;  //to prevent restarting of timer if existing already running
  }
  
}

/**************************************************************************************************
 * @fn          TestPayloadTx
 *
 * @brief       This function is called by the MHMSAppEvt() function. This function is used to test 
 *              network loading by sending test payloads that have a packet size and Tx frequency that
 *              are user defined.  This is operated on a timer and is initiated by SW1.
 * input parameters
 *
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void TestPayloadTx(void){
  static uint8 SeqNum = 0;              //MHMS Note that there is actually a separate Seq number appended to the over the air msg by the ZNP (ex. msg.nwkSeqNum)
  static bool flag = FALSE;
  afAddrType_t addr;                    //AF address stucture defined for info on the destination Endpoint object that data will be sent to
  
  // Turn off collecting data and regular Pulse report generation for Payload testing
  osal_stop_timerEx(MHMSTaskId, MHMS_EVT_CHECKIN);
  osal_stop_timerEx(MHMSTaskId, MHMS_EVT_DAT);
  osal_stop_timerEx(MHMSTaskId, MHMS_EVT_REQ);
  
  TestDatTx[11] = SeqNum;
  TestDatTx[MHMS_CMD_IDX] = MHMS_CMD_DAT_TEST;
  if( flag == FALSE){
  SeqNum++;
  }
  flag = FALSE;
  addr.addr.shortAddr = MHMSAddr;      //loading short address (16-bit) with pulse address
  addr.addrMode = afAddr16Bit;          //Set to directly sent to a node
  addr.endPoint = MHMS_ENDPOINT;       //Sets the endpoint of the final destination (coordinator?)

  
  HalLcdWriteString("TestPayload TX",HAL_LCD_LINE_5);

  if (afStatus_SUCCESS != AF_DataRequest(&addr, (endPointDesc_t *)&MHMS_epDesc, MHMS_CLUSTER_ID,
                                          MHMS_TEST_PAYLOAD_LEN, TestDatTx, &MHMSTSN, AF_DISCV_ROUTE,AF_DEFAULT_RADIUS))
    { //if data transfer is unsuccessful place event immediately back into queue to attempt to send again
        osal_set_event(MHMSTaskId, TEST_EVT_PAYLOAD_TX);
        flag = TRUE;
  }
     osal_start_timerEx(MHMSTaskId, TEST_EVT_PAYLOAD_TX, TEST_DLY_PAYLOAD_TX);  //send check in dummy packet every 10 seconds

}

//MHMS Question do we need this ? tvsaUartRx  this is for recieving command messages from UART
/**************************************************************************************************
 * @fn          MHMSUartRx
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

static void MHMSUartRx(uint8 port, uint8 event)
{

  uint8 ch[5];
  
  HalUARTRead(MHMS_PORT, ch, 5);
  
  dev_gateway = TRUE;   //flag that device is acting as device
  if (ch[2]==0x21)   //if statement to check for command from Z-Sensor Monitor
  {
    sysPingRsp();
    //syncAttempted=TRUE;
    zapSync();      
    if(znpPanId == INVALID_PAN_ID){
      //if still invalid id, there is no PAN so set one up by becomeing the coordinator
     if(zap_set_logicalType(ZG_DEVICETYPE_COORDINATOR)){
      HalLcdWriteString("Initilizing NWK...",HAL_LCD_LINE_7);
     }    
     else{
      HalLcdWriteString("Problem",HAL_LCD_LINE_7);
     }
    }
    else{
      MHMSAddr = znpAddr;
    if (ZSuccess != osal_start_timerEx(MHMSTaskId, MHMS_EVT_ANN, MHMS_DLY_ANN))
      {
        (void)osal_set_event(MHMSTaskId, MHMS_EVT_ANN);
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
  
  
  HalUARTWrite(MHMS_PORT,pingBuff, 7);

}
#endif
/**************************************************************************************************
*/

