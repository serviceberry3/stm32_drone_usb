/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * syslink.c: nRF24L01 implementation of the CRTP link
 */
#define DEBUG_MODULE "USBLINK"
#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "usblink.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"
#include "pm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"
#include "static_mem.h"

#include "usb.h"
#include "debug.h"

static bool isInit = false;
static xQueueHandle crtpPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDelivery, 16, sizeof(CRTPPacket));
static uint8_t sendBuffer[64];

static int usblinkSendPacket(CRTPPacket *p);
static int usblinkSetEnable(bool enable);
static int usblinkReceiveCRTPPacket(CRTPPacket *p);

//STATIC_MEM_TASK_ALLOC(usblinkTask, USBLINK_TASK_STACKSIZE);

static struct crtpLinkOperations usblinkOp =
{
  .setEnable         = usblinkSetEnable,
  .sendPacket        = usblinkSendPacket,
  .receivePacket     = usblinkReceiveCRTPPacket,
};

/* Radio task handles the CRTP packet transfers as well as the radio link
 * specific communications (eg. Scann and ID ports, communication error handling
 * and so much other cool things that I don't have time for it ...)
 */
//static USBPacket usbIn;
//static CRTPPacket p;



/*
static void usblinkTask(void *param) {
  while (1) {

    //Fetch a USB packet off the queue. BLOCKING fxn so it blocks other work till complete
	  //(foreground run)
    usbGetDataBlocking(&usbIn);


    DEBUG_PRINT("USB Data packet received from usbGetDataBlocking: ");


    //print out the packet received
    DEBUG_PRINT("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X "
    		"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X "
    		"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X "
    		"0x%02X 0x%02X 0x%02X 0x%02X", usbIn.data[0], usbIn.data[1], usbIn.data[2],
			usbIn.data[3], usbIn.data[4], usbIn.data[5], usbIn.data[6], usbIn.data[7],
			usbIn.data[8], usbIn.data[9], usbIn.data[10], usbIn.data[11], usbIn.data[12],
			usbIn.data[13], usbIn.data[14], usbIn.data[15], usbIn.data[16], usbIn.data[17],
			usbIn.data[18], usbIn.data[19], usbIn.data[20], usbIn.data[21], usbIn.data[22],
			usbIn.data[23], usbIn.data[24], usbIn.data[25], usbIn.data[26], usbIn.data[27],
			usbIn.data[28], usbIn.data[29], usbIn.data[30], usbIn.data[31], usbIn.data[32]);

    DEBUG_PRINT("\n");


    //set size of the CRTP Packet
    p.size = usbIn.size - 1;

	//copy the raw bytes from USB packet to CRTP packet
	memcpy(&p.raw, usbIn.data, usbIn.size);

	//Send the packet onto the packet queue I guess, treating it just like a CRTP radio packet
	//This queuing will copy a CRTP packet size from usbIn
	ASSERT(xQueueSend(crtpPacketDelivery, &p, 0) == pdTRUE); //sometimes failing assert
  }

}*/


static int usblinkReceiveCRTPPacket(CRTPPacket *p) {
	//get packet from queue that was placed there by usblinkTask at some pt
  if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE) {
    ledseqRun(&seq_linkUp);
    return 0;
  }

  return -1;
}

static int usblinkSendPacket(CRTPPacket *p) {
  int dataSize;

  ASSERT(p->size < SYSLINK_MTU);

  sendBuffer[0] = p->header;

  if (p->size <= CRTP_MAX_DATA_SIZE) {
    memcpy(&sendBuffer[1], p->data, p->size);
  }
  dataSize = p->size + 1;


  ledseqRun(&seq_linkDown);

  return usbSendData(dataSize, sendBuffer);
}

static int usblinkSetEnable(bool enable) {
  return 0;
}

/*
 * Public functions
 */

void usblinkInit() {
  if (isInit)
    return;

  //Initialize the USB peripheral for the MCU
  usbInit();

  crtpPacketDelivery = STATIC_MEM_QUEUE_CREATE(crtpPacketDelivery);

  DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

  //create usblinkTask
  //STATIC_MEM_TASK_CREATE(usblinkTask, usblinkTask, USBLINK_TASK_NAME, NULL, USBLINK_TASK_PRI);

  isInit = true;
}

bool usblinkTest() {
  return isInit;
}

struct crtpLinkOperations* usblinkGetLink() {
  return &usblinkOp;
}
