/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * radiolink.c - Radio link layer
 */
#define DEBUG_MODULE "RADIOLINK"
#include <string.h>
#include <stdint.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"
#include "radiolink.h"
#include "syslink.h"
#include "crtp.h"
#include "configblock.h"
#include "log.h"
#include "led.h"
#include "ledseq.h"
#include "queuemonitor.h"
#include "static_mem.h"
#include "debug.h"
#include "usb.h"

#define RADIOLINK_TX_QUEUE_SIZE (1)
#define RADIOLINK_CRTP_QUEUE_SIZE (5)
#define RADIO_ACTIVITY_TIMEOUT_MS (1000)

#define RADIOLINK_P2P_QUEUE_SIZE (5)

static xQueueHandle txQueue;
STATIC_MEM_QUEUE_ALLOC(txQueue, RADIOLINK_TX_QUEUE_SIZE, sizeof(SyslinkPacket));

static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, RADIOLINK_CRTP_QUEUE_SIZE, sizeof(CRTPPacket));

static bool isInit;

static uint8_t sendBuffer[1];

static int radiolinkSendCRTPPacket(CRTPPacket *p);
static int radiolinkSetEnable(bool enable);
static int radiolinkReceiveCRTPPacket(CRTPPacket *p);

//Local RSSI variable used to enable logging of RSSI values from Radio
static uint8_t rssi;
static bool isConnected;
static uint32_t lastPacketTick;

static volatile P2PCallback p2p_callback;

static bool radiolinkIsConnected(void) {
  return (xTaskGetTickCount() - lastPacketTick) < M2T(RADIO_ACTIVITY_TIMEOUT_MS);
}

static struct crtpLinkOperations radiolinkOp = {
  .setEnable         = radiolinkSetEnable,
  .sendPacket        = radiolinkSendCRTPPacket,
  .receivePacket     = radiolinkReceiveCRTPPacket,
  .isConnected       = radiolinkIsConnected
};

void radiolinkInit(void) {
  if (isInit)
    return;

  //create queues for transmitting data and "delivering packets"
  txQueue = STATIC_MEM_QUEUE_CREATE(txQueue);
  DEBUG_QUEUE_MONITOR_REGISTER(txQueue);
  crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);
  DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

  ASSERT(crtpPacketDelivery);

  //init the syslink task to read over UART packets from radio chip
  syslinkInit();

  radiolinkSetChannel(configblockGetRadioChannel());
  radiolinkSetDatarate(configblockGetRadioSpeed());
  radiolinkSetAddress(configblockGetRadioAddress());

  isInit = true;
}

bool radiolinkTest(void) {
  return syslinkTest();
}

void radiolinkSetChannel(uint8_t channel) {
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_CHANNEL;
  slp.length = 1;
  slp.data[0] = channel;
  syslinkSendPacket(&slp);
}

void radiolinkSetDatarate(uint8_t datarate) {
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_DATARATE;
  slp.length = 1;
  slp.data[0] = datarate;
  syslinkSendPacket(&slp);
}

void radiolinkSetAddress(uint64_t address) {
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_ADDRESS;
  slp.length = 5;
  memcpy(&slp.data[0], &address, 5);
  syslinkSendPacket(&slp);
}

void radiolinkSetPowerDbm(int8_t powerDbm) {
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_POWER;
  slp.length = 1;
  slp.data[0] = powerDbm;
  syslinkSendPacket(&slp);
}

//this is called by syslinkTask->sysLinkRouteIncommingPacket
void radiolinkSyslinkDispatch(SyslinkPacket *slp) {
	//the ack to be sent back to the controller
  static SyslinkPacket txPacket;

  /*
  //raw radio packets
  if (slp->type == SYSLINK_RADIO_RAW || slp->type == SYSLINK_RADIO_RAW_BROADCAST) {
    lastPacketTick = xTaskGetTickCount();
  }*/

  if (slp->type == SYSLINK_RADIO_RAW) {
	 //DEBUG_PRINT("radiolinkSyslinkDispatch(): type of incoming pkt does = SYSLINK_RADIO_RAW\n");

    slp->length--; //Decrease to get CRTP size.

    //put this incoming SyslinkPacket (from the radio chip) on the crtp packet queue, passing memory address where length starts
    xQueueSend(crtpPacketDelivery, (&slp->length - 1), 0); //ADJUST

    ledseqRun(&seq_linkUp);



    //ACK SENDING CODE

    /*
    //If a radio packet is received, get packet off the txQueue to be sent
    if (xQueueReceive(txQueue, &txPacket, 0) == pdTRUE) {
    	//run some indicator LEDs
      ledseqRun(&seq_linkDown);

      //DEBUG_PRINT("radiolinkSyslinkDispatch(): sending ACK\n");

      //send the ack packet to the controller (PC or phone)
      syslinkSendPacket(&txPacket);
    }*/

    //NOAH CHANGE: SEND TEMPORARY ACK 0x09
    sendBuffer[0] = 9;
    usbSendData(1, sendBuffer);
  }

  //if it's a NULL packet
  else if (slp->type == 0xff) {
	  //DEBUG_PRINT("radiolinkSyslinkDispatch(): type of incoming pkt is NULL PKT\n");

	  if (xQueueReceive(txQueue, &txPacket, 0) == pdTRUE) {
	      	//run some indicator LEDs
	        ledseqRun(&seq_linkDown);

	        //DEBUG_PRINT("radiolinkSyslinkDispatch(): sending ACK\n");

	        //send the ack packet to the controller (PC or phone)
	        syslinkSendPacket(&txPacket);
	      }
	  else {
		  //DEBUG_PRINT("radiolinkSyslinkDispatch(): no ack packet waiting on txPacket queue\n");
	  }
  }


  else if (slp->type == SYSLINK_RADIO_RAW_BROADCAST) {
    slp->length--; // Decrease to get CRTP size.
    xQueueSend(crtpPacketDelivery, &slp->length, 0);
    ledseqRun(&seq_linkUp);

    //NOTE: NO ACK SENT BACK FOR BROADCASTS
  }

  else if (slp->type == SYSLINK_RADIO_RSSI) {
    //Extract RSSI sample sent from radio
    memcpy(&rssi, slp->data, sizeof(uint8_t)); //rssi will not change on disconnect
  }


  else if (slp->type == SYSLINK_RADIO_P2P_BROADCAST) {
    ledseqRun(&seq_linkUp);
    P2PPacket p2pp;
    p2pp.port=slp->data[0];
    p2pp.rssi = slp->data[1];
    memcpy(&p2pp.data[0], &slp->data[2], slp->length - 2);
    p2pp.size=slp->length;
    if (p2p_callback)
        p2p_callback(&p2pp);
  }

  //check if link still connected
  //isConnected = radiolinkIsConnected();
}

//receive CRTP packet into p from the crtpPacketDelivery queue. NOT BLOCKING apparently
static int radiolinkReceiveCRTPPacket(CRTPPacket *p) {


	//get packet off crtpPacketDelivery queue into p
  if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE) {
	//DEBUG_PRINT("radiolinkReceiveCRTPPacket(): packet received successfully\n");
    return 0;
  }

  //DEBUG_PRINT("radiolinkReceiveCRTPPacket(): no packet found\n");
  return -1;
}

void p2pRegisterCB(P2PCallback cb) {
    p2p_callback = cb;
}

//send the passed packet by putting it on the txQueue
static int radiolinkSendCRTPPacket(CRTPPacket *p) {
  static SyslinkPacket slp;

  ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  slp.type = SYSLINK_RADIO_RAW;
  slp.length = p->size + 1;
  memcpy(slp.data, &p->header, p->size + 1);

  if (xQueueSend(txQueue, &slp, M2T(100)) == pdTRUE) {
    return true;
  }

  return false;
}

bool radiolinkSendP2PPacketBroadcast(P2PPacket *p) {
  static SyslinkPacket slp;

  ASSERT(p->size <= P2P_MAX_DATA_SIZE);

  slp.type = SYSLINK_RADIO_P2P_BROADCAST;
  slp.length = p->size + 1;
  memcpy(slp.data, p->raw, p->size + 1);

  syslinkSendPacket(&slp);
  ledseqRun(&seq_linkDown);

  return true;
}


struct crtpLinkOperations * radiolinkGetLink() {
  return &radiolinkOp;
}

static int radiolinkSetEnable(bool enable) {
  return 0;
}

LOG_GROUP_START(radio)
LOG_ADD(LOG_UINT8, rssi, &rssi)
LOG_ADD(LOG_UINT8, isConnected, &isConnected)
LOG_GROUP_STOP(radio)
