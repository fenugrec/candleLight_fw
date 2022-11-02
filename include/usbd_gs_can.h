/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "usbd_def.h"
#include "queue.h"
#include "led.h"
#include "gs_usb.h"


/* Define these here so they can be referenced in other files */

#define CAN_DATA_MAX_PACKET_SIZE 32    /* Endpoint IN & OUT Packet size */
#if defined(CANFD_SUPPORT)
	#define CAN_CMD_PACKET_SIZE			72    /* Control Endpoint Packet size */
#else
	#define CAN_CMD_PACKET_SIZE			64    /* Control Endpoint Packet size */
#endif
#define USB_CAN_CONFIG_DESC_SIZ	 50
#define NUM_CAN_CHANNEL			 1
#define USBD_GS_CAN_VENDOR_CODE	 0x20
#define DFU_INTERFACE_NUM		 1
#define DFU_INTERFACE_STR_INDEX	 0xE0

/* Create a compile time define to reduce RAM use for non-CANFD codebases */
#if defined(CANFD_SUPPORT)
#define GS_HOST_FRAME gs_host_frame
#define GS_HOST_FRAME_CLASSIC_CAN gs_host_frame_classic_can
#else
#define GS_HOST_FRAME gs_host_frame_classic_can
#endif

/* this exists here to prevent circular dependency with can.h */
#if defined (STM32F0) || defined (STM32F4)
#define CAN_HANDLE_TYPEDEF				CAN_HandleTypeDef
#define CAN_TYPEDEF					CAN_TypeDef
#elif defined (STM32G0)
#define CAN_HANDLE_TYPEDEF				FDCAN_HandleTypeDef
#define CAN_TYPEDEF					FDCAN_GlobalTypeDef
#endif

extern USBD_ClassTypeDef USBD_GS_CAN;

#if defined(STM32F0)
# define USB_INTERFACE USB
# define USB_INTERRUPT USB_IRQn
#elif defined(STM32F4)
# define USB_INTERFACE USB_OTG_FS
# define USB_INTERRUPT OTG_FS_IRQn

// RX FIFO is defined in words, so divide bytes by 4
// RX FIFO size chosen according to reference manual RM0368 which suggests
// using (largest packet size / 4) + 1
# define USB_RX_FIFO_SIZE ((256U / 4U) + 1U)
#elif defined(STM32G0)
#define USB_INTERFACE   USB_DRD_FS
#define USB_INTERRUPT   USB_UCPD1_2_IRQn
#elif defined(STM32G4)
#define USB_INTERFACE   USB
#define USB_INTERRUPT   USB_LP_IRQn
#endif

uint8_t USBD_GS_CAN_Init(USBD_HandleTypeDef *pdev, queue_t *q_frame_pool, queue_t *q_from_host, led_data_t *leds);
void USBD_GS_CAN_SetChannel(USBD_HandleTypeDef *pdev, uint8_t channel, CAN_HANDLE_TYPEDEF* handle);
CAN_HANDLE_TYPEDEF* USBD_GS_CAN_GetChannelHandle(USBD_HandleTypeDef *pdev, uint8_t channel);
uint8_t USBD_GS_CAN_GetChannelNumber(USBD_HandleTypeDef *pdev, CAN_HANDLE_TYPEDEF* handle);
void USBD_GS_CAN_SuspendCallback(USBD_HandleTypeDef  *pdev);
void USBD_GS_CAN_ResumeCallback(USBD_HandleTypeDef  *pdev);
bool USBD_GS_CAN_TxReady(USBD_HandleTypeDef *pdev);
uint8_t USBD_GS_CAN_PrepareReceive(USBD_HandleTypeDef *pdev);
bool USBD_GS_CAN_CustomDeviceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
bool USBD_GS_CAN_CustomInterfaceRequest(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

bool USBD_GS_CAN_DfuDetachRequested(USBD_HandleTypeDef *pdev);
uint8_t USBD_GS_CAN_SendFrame(USBD_HandleTypeDef *pdev, struct GS_HOST_FRAME *frame);
uint8_t USBD_GS_CAN_Transmit(USBD_HandleTypeDef *pdev, uint8_t *buf, uint16_t len);
