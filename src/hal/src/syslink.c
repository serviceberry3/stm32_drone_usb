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
 * syslink.c: Communication between NRF51 and STM32
 */
#define DEBUG_MODULE "SL"

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "config.h"
#include "debug.h"
#include "syslink.h"
#include "radiolink.h"
#include "uart_syslink.h"
#include "configblock.h"
#include "pm.h"
#include "ow.h"
#include "static_mem.h"
#include "usb.h"

#ifdef UART2_LINK_COMM
#include "uart2.h"
#endif

static bool isInit = false;
static uint8_t sendBuffer[SYSLINK_MTU + 6];

static void syslinkRouteIncommingPacket(SyslinkPacket *slp);

static xSemaphoreHandle syslinkAccess;

static USBPacket usbIn;
static volatile SyslinkRxState rxState = waitForFirstStart;
//static volatile uint8_t dataIndex = 0;
static volatile uint8_t cksum[2] = {0};

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(syslinkTask, SYSLINK_TASK_STACKSIZE);



//parse a USB packet from syslink
int usbslkParseIncomingPacket(SyslinkPacket* slp) {
	//DEBUG_PRINT("USB Data packet of len %d received from usbGetDataBlocking: ", usbIn.size);


	    //print out the packet received
	   // DEBUG_PRINT("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X "
	    		//"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n"
	    		/*
	    		0x%02X "
	    		"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X "
	    		"0x%02X 0x%02X 0x%02X 0x%02X\n"*/,


				usbIn.data[0], usbIn.data[1], usbIn.data[2],
				usbIn.data[3], usbIn.data[4], usbIn.data[5], usbIn.data[6], usbIn.data[7],
				usbIn.data[8], usbIn.data[9], usbIn.data[10], usbIn.data[11], usbIn.data[12],
				usbIn.data[13], usbIn.data[14], usbIn.data[15], usbIn.data[16]);

																		   /*usbIn.data[17],
				usbIn.data[18], usbIn.data[19], usbIn.data[20], usbIn.data[21], usbIn.data[22],
				usbIn.data[23], usbIn.data[24], usbIn.data[25], usbIn.data[26], usbIn.data[27],
				usbIn.data[28], usbIn.data[29], usbIn.data[30], usbIn.data[31], usbIn.data[32]);*/


	int ctr = 0;
	int dataIndex=0;

	uint8_t c = 0x00;

	while (1) {
		if (ctr <= 16) {
			c = usbIn.data[ctr];

			ctr++;
		}

		  switch (rxState) {
			  //first start: should be 0xBC
			  case waitForFirstStart:
				  if (c == SYSLINK_START_BYTE1) {
					  rxState = waitForSecondStart;
				  }
				  //might be a NULL packet
				  else if (c == 0xff) {
					  //DEBUG_PRINT("Parser found NULL pkt, setting 0xff as type of slp\n");
					  slp->type = c;
					  rxState = waitForFirstStart;
					  dataIndex=0;
					  return 0;
				  }
				  else {
					  rxState = waitForFirstStart;
					  //DEBUG_PRINT("USB packet parse fail on first start byte\n");
					  dataIndex=0;
					  return 1;
				  }
				  break;


				//second start: should be 0xCF
			  case waitForSecondStart:
				  if (c == SYSLINK_START_BYTE2) {
					  //CHANGED: WE ASSUME TYPE IS RADIO RAW, JUMP OVER THAT STATE
					  slp->type = 0x00;
					  //rxState = waitForType;
					  rxState = waitForLength;
				  }
				  else {
					  rxState = waitForFirstStart;
					  //DEBUG_PRINT("USB packet parse fail on second start byte\n");
					  dataIndex=0;
					  return 1;
				  }
				  break;


				//type: could be various of RADIO (0x00), PM (0x10), or OW (0x20)
			  case waitForType:
				cksum[0] = c;
				cksum[1] = c;
				slp->type = c;
				rxState = waitForLength;
				break;

				//length: defines the data length
			  case waitForLength:
				  /*
				if (c <= SYSLINK_MTU) {
				  slp->length = c;
				  cksum[0] += c;
				  cksum[1] += cksum[0];
				  dataIndex = 0;
				  rxState = (c > 0) ? waitForData : waitForChksum1;
				}

				else {
				  rxState = waitForFirstStart;
				  DEBUG_PRINT("USB packet parse fail on length less than or equal to 64 (0x40)\n");
				  return 1;
				}*/

				  //let's just assume length of the data is 14: 1 port specifier, 3 floats (4 bytes each), char (2 bytes) for
				  //thrust, and 2 fake radio headers
				  rxState = waitForData;
				  slp->length = 14;
				  dataIndex=0;
				break;

				//data:
			  case waitForData:
				  //add this byte into the SyslinkPacket's data field


				slp->data[dataIndex] = c;
				cksum[0] += c;
				cksum[1] += cksum[0];
				dataIndex++;

				//after adding our 14th data byte
				if (dataIndex == 14) {
					dataIndex=0;
					rxState=waitForFirstStart;
					return 0;
				  //rxState = waitForChksum1;
				}
				rxState = waitForData;
				break;

				//2-byte Fletcher 8-bit checksum, calculated using TYPE, LENGTH, DATA
			  case waitForChksum1: //first byte of checksum
				/*if (cksum[0] == c) {
				  rxState = waitForChksum2;
				}

				else {
				  rxState = waitForFirstStart; //Checksum error, return to initial state
				  //IF_DEBUG_ASSERT(0);
				  DEBUG_PRINT("USB packet parse fail on checksum 1\n");
				  return 1;
				}*/

				  //CHANGED: BYPASS CHCKSUM 1
				  rxState = waitForChksum2;
				break;

				//second byte of checksum
			  case waitForChksum2:
				  rxState = waitForFirstStart; //Checksum error, go back to initial state

				  //CHANGED: BYPASS CHKSUM 2

				  /*
				if (cksum[1] == c) {

					//DONE PARSING, SUCESSFUL SyslinkPacket found!
					return 0;
				}

				else {
					DEBUG_PRINT("USB packet parse fail on checksum 2\n");
				  return 1;
				}*/

				  return 0;
				  break;

			  default:
				ASSERT(0);
				break;
		  }

  }
}



/* Syslink task, handles communication between nrf and stm and dispatch messages
 */
static void syslinkTask(void *param) {
  SyslinkPacket slp;

  while (1) {
	//get an incoming radio packet (blocking fxn), read it into slp
    //uartslkGetPacketBlocking(&slp); //TODO: MOD SO THAT IT'S USING USBLINK INSTEAD
    //syslinkRouteIncommingPacket(&slp);
	  //usbslkGetPacketBlocking(&slp);

	  //DEBUG_PRINT("Getting usb data blocking\n");
	  usbGetDataBlocking(&usbIn);
	  //DEBUG_PRINT("USB data read in successfully\n");

	  //if the packet looks valid
	  if (!usbslkParseIncomingPacket(&slp)) {
			//incoming packet is now in slp

			//route the packet to the appropriate location
			syslinkRouteIncommingPacket(&slp);
	  }
	  else {
		  DEBUG_PRINT("syslinkTask: parse failed, not routing the packet\n");
	  }
	  //otherwise the parsing found invalidity, get the next incoming USB packet
  }
}




#ifdef UART2_LINK_COMM

STATIC_MEM_TASK_ALLOC(uart2Task, UART2_TASK_STACKSIZE);

static void uart2Task(void *param) {
  SyslinkPacket slp;


  while (1) {
    uart2GetPacketBlocking(&slp);
    syslinkRouteIncommingPacket(&slp);
  }
}

#endif

static void syslinkRouteIncommingPacket(SyslinkPacket *slp) {
  //uint8_t groupType;

  //groupType = slp->type & SYSLINK_GROUP_MASK;

  radiolinkSyslinkDispatch(slp);

  /*
  switch (groupType) {
    case SYSLINK_RADIO_GROUP:
    //this is a radio packet, dispatch it
      radiolinkSyslinkDispatch(slp);
      break;
    case SYSLINK_PM_GROUP:
      pmSyslinkUpdate(slp);
      break;
    case SYSLINK_OW_GROUP:
      owSyslinkRecieve(slp);
      break;
    default:
      DEBUG_PRINT("Unknown packet: %X.\n", slp->type);
      break;
  }

  //check to see if it's a NULL packet
  if (slp->type == 0xff) {
	  DEBUG_PRINT("routed NULL\n");
	  radiolinkSyslinkDispatch(slp);
  }*/
}

/*
 * Public functions
 */
//create the infinite syslink task to run on a 'thread'
void syslinkInit() {
  if (isInit) {
    return;
  }

  vSemaphoreCreateBinary(syslinkAccess);

  STATIC_MEM_TASK_CREATE(syslinkTask, syslinkTask, SYSLINK_TASK_NAME, NULL, SYSLINK_TASK_PRI);

  #ifdef UART2_LINK_COMM
  	  DEBUG_PRINT("syslinkInit(): Creating uart2Task...\n");
	  uart2Init(512000);
	  STATIC_MEM_TASK_CREATE(uart2Task, uart2Task, UART2_TASK_NAME, NULL, UART2_TASK_PRI);
  #endif

  isInit = true;
}

bool syslinkTest() {
  return isInit;
}

int syslinkSendPacket(SyslinkPacket *slp) {
	/*DEBUG_PRINT("Ack to be sent back to controller: ");

	//print out the ack to be sent
	DEBUG_PRINT("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X "
			"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X "
			"0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X "
			"0x%02X 0x%02X 0x%02X 0x%02X\n", slp->type, slp->length, slp->data[0],
			slp->data[1], slp->data[2], slp->data[3], slp->data[4], slp->data[5],
			slp->data[6], slp->data[7], slp->data[8], slp->data[9], slp->data[10],
			slp->data[11], slp->data[12], slp->data[13], slp->data[14], slp->data[15],
			slp->data[16], slp->data[17], slp->data[18], slp->data[19], slp->data[20],
			slp->data[21], slp->data[22], slp->data[23], slp->data[24], slp->data[25],
			slp->data[26], slp->data[27], slp->data[28], slp->data[29], slp->data[30]);*/


  int i = 0;
  int dataSize;
  uint8_t cksum[2] = {0};

  xSemaphoreTake(syslinkAccess, portMAX_DELAY);

  ASSERT(slp->length <= SYSLINK_MTU);

  sendBuffer[0] = SYSLINK_START_BYTE1;
  sendBuffer[1] = SYSLINK_START_BYTE2;
  sendBuffer[2] = slp->type;
  sendBuffer[3] = slp->length;

  memcpy(&sendBuffer[4], slp->data, slp->length);
  dataSize = slp->length + 6;
  // Calculate checksum delux
  for (i = 2; i < dataSize - 2; i++) {
    cksum[0] += sendBuffer[i];
    cksum[1] += cksum[0];
  }
  sendBuffer[dataSize-2] = cksum[0];
  sendBuffer[dataSize-1] = cksum[1];

  #ifdef UART2_LINK_COMM
  uint8_t groupType;
  groupType = slp->type & SYSLINK_GROUP_MASK;


  //send the packet over dma/radio
  switch (groupType) {
    case SYSLINK_RADIO_GROUP:
      uart2SendDataDmaBlocking(dataSize, sendBuffer);
      break;
    case SYSLINK_PM_GROUP:
      uartslkSendDataDmaBlocking(dataSize, sendBuffer);
      break;
    case SYSLINK_OW_GROUP:
      uartslkSendDataDmaBlocking(dataSize, sendBuffer);
      break;
    default:
      DEBUG_PRINT("Unknown ack packet:%X.\n", slp->type);
      break;
  }


  #else
  uartslkSendDataDmaBlocking(dataSize, sendBuffer);
  #endif

  xSemaphoreGive(syslinkAccess);

  return 0;
}
