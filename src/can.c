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

#include <string.h>
#include "usbd_def.h"
#include "can.h"
#include "config.h"
#include "gs_usb.h"
#include "usbd_gs_can.h"
#include "hal_include.h"

extern USBD_HandleTypeDef hUSB;

// data read/write buffers to support CAN
#if defined(STM32G0)
	uint8_t can_rx_data_buff[64];
	uint8_t can_tx_data_buff[64];
#else
	uint8_t can_rx_data_buff[8];
	uint8_t can_tx_data_buff[8];
	uint32_t TxMailbox;
#endif



// The STM32F0 only has one CAN interface, define it as CAN1 as
// well, so it doesn't need to be handled separately.
#if !defined(CAN1) && defined(CAN)
#define CAN1 CAN
#endif

// Completely reset the CAN pheriperal, including bus-state and error counters
static void rcc_reset(CAN_TYPEDEF *instance)
{
#ifdef CAN1
	if (instance == CAN1) {
		__HAL_RCC_CAN1_FORCE_RESET();
		__HAL_RCC_CAN1_RELEASE_RESET();
	}
#endif

#ifdef CAN2
	if (instance == CAN2) {
		__HAL_RCC_CAN2_FORCE_RESET();
		__HAL_RCC_CAN2_RELEASE_RESET();
	}
#endif

#if defined(FDCAN1)
		UNUSED(instance);
		__HAL_RCC_FDCAN_FORCE_RESET();
		__HAL_RCC_FDCAN_RELEASE_RESET();
#endif
}

void can_init(CAN_HANDLE_TYPEDEF *hcan, CAN_TYPEDEF *instance)
{
	GPIO_InitTypeDef itd = {0};
#if defined(STM32F0)
	__HAL_RCC_CAN1_CLK_ENABLE();
	itd.Pin = GPIO_PIN_8|GPIO_PIN_9;
	itd.Mode = GPIO_MODE_AF_PP;
	itd.Pull = GPIO_NOPULL;
	itd.Speed = GPIO_SPEED_FREQ_HIGH;
	itd.Alternate = GPIO_AF4_CAN;
	HAL_GPIO_Init(GPIOB, &itd);
#elif defined(STM32F4)
	__HAL_RCC_CAN1_CLK_ENABLE();
	itd.Pin = GPIO_PIN_0|GPIO_PIN_1;
	itd.Mode = GPIO_MODE_AF_PP;
	itd.Pull = GPIO_NOPULL;
	itd.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	itd.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOD, &itd);
#elif defined(STM32G0)
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
	PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	__HAL_RCC_FDCAN_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	itd.Pin = GPIO_PIN_9|GPIO_PIN_8;
	itd.Mode = GPIO_MODE_AF_PP;
	itd.Pull = GPIO_NOPULL;
	itd.Speed = GPIO_SPEED_FREQ_LOW;
	itd.Alternate = GPIO_AF3_FDCAN1;
    HAL_GPIO_Init(GPIOB, &itd);
#if defined(FDCAN2)
	itd.Pin = GPIO_PIN_0|GPIO_PIN_1;
	itd.Mode = GPIO_MODE_AF_PP;
	itd.Pull = GPIO_NOPULL;
	itd.Speed = GPIO_SPEED_FREQ_LOW;
	itd.Alternate = GPIO_AF3_FDCAN2;
	HAL_GPIO_Init(GPIOB, &itd);
#endif
#endif


#if defined(STM32F0) || defined(STM32F4)
	hcan->Instance = instance;
	hcan->Init.TimeTriggeredMode = DISABLE;
	hcan->Init.AutoBusOff = ENABLE;
	hcan->Init.AutoWakeUp = DISABLE;
	hcan->Init.AutoRetransmission = ENABLE;
	hcan->Init.ReceiveFifoLocked = DISABLE;
	hcan->Init.TransmitFifoPriority = ENABLE;
	hcan->Init.Mode = CAN_MODE_NORMAL;

	/* all values for the bxCAN init are -1 and shifted */
	hcan->Init.SyncJumpWidth = ((1)-1) << CAN_BTR_SJW_Pos;
	hcan->Init.Prescaler = ((6)-1);
#if defined(STM32F4)
	hcan->Init.TimeSeg1 = ((12)-1) << CAN_BTR_TS1_Pos;
	hcan->Init.TimeSeg2 = ((1)-1) << CAN_BTR_TS2_Pos;
#else
	hcan->Init.TimeSeg1 = ((13)-1) << CAN_BTR_TS1_Pos;
	hcan->Init.TimeSeg2 = ((2)-1) << CAN_BTR_TS2_Pos;
#endif
	HAL_CAN_Init(hcan);

#elif defined(STM32G0)
	hcan->Instance = instance;
	hcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	hcan->Init.Mode = FDCAN_MODE_NORMAL;
	hcan->Init.AutoRetransmission = DISABLE;
	hcan->Init.TransmitPause = DISABLE;
	hcan->Init.ProtocolException = ENABLE;
	hcan->Init.NominalPrescaler = 8;
	hcan->Init.NominalSyncJumpWidth = 1;
	hcan->Init.NominalTimeSeg1 = 13;
	hcan->Init.NominalTimeSeg2 = 2;
	hcan->Init.DataPrescaler = 2;
	hcan->Init.DataSyncJumpWidth = 4;
	hcan->Init.DataTimeSeg1 = 15;
	hcan->Init.DataTimeSeg2 = 4;
	hcan->Init.StdFiltersNbr = 0;
	hcan->Init.ExtFiltersNbr = 0;
	hcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	HAL_FDCAN_Init(hcan);
#endif
}

bool can_set_bittiming(CAN_HANDLE_TYPEDEF *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{
	if ((brp>0) && (brp<=1024)
		&& (phase_seg1>0) && (phase_seg1<=16)
		&& (phase_seg2>0) && (phase_seg2<=8)
		&& (sjw>0) && (sjw<=4)
	) {
#if defined(STM32G0)
		hcan->Init.NominalPrescaler = brp;
		hcan->Init.NominalTimeSeg1 = phase_seg1;
		hcan->Init.NominalTimeSeg2 = phase_seg2;
		hcan->Init.NominalSyncJumpWidth = sjw;
#else
		hcan->Init.SyncJumpWidth = (sjw-1) << CAN_BTR_SJW_Pos;
		hcan->Init.TimeSeg1 = (phase_seg1-1) << CAN_BTR_TS1_Pos;
		hcan->Init.TimeSeg2 = (phase_seg2-1) << CAN_BTR_TS2_Pos;;
		hcan->Init.Prescaler = brp;
#endif
		return true;
	}

	return false;
}

bool can_set_data_bittiming(CAN_HANDLE_TYPEDEF *hcan, uint16_t brp, uint8_t phase_seg1, uint8_t phase_seg2, uint8_t sjw)
{

	if ((brp>0) && (brp<=1024)
		&& (phase_seg1>0) && (phase_seg1<=16)
		&& (phase_seg2>0) && (phase_seg2<=8)
		&& (sjw>0) && (sjw<=4)
	) {
#if defined(STM32G0)
		hcan->Init.DataPrescaler = brp;
		hcan->Init.DataTimeSeg1 = phase_seg1;
		hcan->Init.DataTimeSeg2 = phase_seg2;
		hcan->Init.DataSyncJumpWidth = sjw;
		return true;
#else
		UNUSED(hcan);
		UNUSED(brp);
		UNUSED(phase_seg1);
		UNUSED(phase_seg2);
		UNUSED(sjw);
#endif
	}
	return false;
}

void can_enable(CAN_HANDLE_TYPEDEF *hcan, bool loop_back, bool listen_only, bool one_shot, bool can_mode_fd)
{
	// Completely reset before reinitializing the bus
	rcc_reset(hcan->Instance);

#if defined(STM32F0) || defined (STM32F4)
	UNUSED(can_mode_fd);
	hcan->Init.AutoRetransmission = one_shot ? DISABLE : ENABLE;
	hcan->Init.Mode = CAN_MODE_NORMAL;
	if (listen_only) hcan->Init.Mode |= CAN_MODE_SILENT;
	if (loop_back) hcan->Init.Mode |= CAN_MODE_LOOPBACK;
	
	HAL_CAN_Init(hcan);

	// Configure reception filter to Rx FIFO 0
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
	
	// Start CAN using HAL
	HAL_CAN_Start(hcan);

#elif defined(STM32G0)
	hcan->Init.AutoRetransmission = one_shot ? DISABLE : ENABLE;
	if (loop_back && listen_only) hcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
	else if (loop_back) hcan->Init.Mode = FDCAN_MODE_EXTERNAL_LOOPBACK;
	else if (listen_only) hcan->Init.Mode = FDCAN_MODE_BUS_MONITORING;
	else hcan->Init.Mode = FDCAN_MODE_NORMAL;
	hcan->Init.FrameFormat = can_mode_fd ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;

	HAL_FDCAN_Init(hcan);

	/* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x7FF;

	HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig);

	/* Configure global filter on both FDCAN instances:
		 Filter all remote frames with STD and EXT ID
		 Reject non matching frames with STD ID and EXT ID */
	HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

	// Completely reset while being off the bus
	rcc_reset(hcan->Instance);
	// Start CAN using HAL
	HAL_FDCAN_Start(hcan);
#endif
#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, nCANSTBY_Active_High == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}

void can_disable(CAN_HANDLE_TYPEDEF *hcan)
{
#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, nCANSTBY_Active_High == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
#if defined(STM32F0) || defined (STM32F4)
	HAL_CAN_Stop(hcan);
#elif defined(STM32G0)
	HAL_FDCAN_Stop(hcan);
#endif
}

bool can_is_enabled(CAN_HANDLE_TYPEDEF *hcan)
{
#if defined(STM32F0) || defined (STM32F4)
	return hcan->State == HAL_CAN_STATE_LISTENING;
#elif defined(STM32G0)
	return hcan->State == HAL_FDCAN_STATE_BUSY;
#endif
}

bool can_is_rx_pending(CAN_HANDLE_TYPEDEF *hcan)
{
#if defined(STM32F0) || defined (STM32F4)
	return (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) >= 1);
#elif defined(STM32G0)
	return (HAL_FDCAN_GetRxFifoFillLevel(hcan, FDCAN_RX_FIFO0) >= 1);
#endif
}

bool can_receive(CAN_HANDLE_TYPEDEF *hcan, struct GS_HOST_FRAME *rx_frame)
{
#if defined(STM32F0) || defined (STM32F4)
  CAN_RxHeaderTypeDef RxHeader;

	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, can_rx_data_buff) != HAL_OK)
	{
		return false;
	}

	rx_frame->channel = USBD_GS_CAN_GetChannelNumber(&hUSB, hcan);
	rx_frame->can_dlc = RxHeader.DLC;
	if (RxHeader.IDE == CAN_ID_EXT) {
		rx_frame->can_id = RxHeader.ExtId | CAN_EFF_FLAG;
	}
	else {
		rx_frame->can_id = RxHeader.StdId;
	}

	if (RxHeader.RTR == CAN_RTR_REMOTE) {
		rx_frame->can_id |= CAN_RTR_FLAG;
	}
	memcpy(rx_frame->classic_can.data, can_rx_data_buff, 8);

	return true;

#elif defined(STM32G0)
	FDCAN_RxHeaderTypeDef RxHeader;

	if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &RxHeader, can_rx_data_buff) != HAL_OK) {
		 return false;
	}

	rx_frame->channel = USBD_GS_CAN_GetChannelNumber(&hUSB, hcan);
	rx_frame->can_id = RxHeader.Identifier;

	if (RxHeader.IdType == FDCAN_EXTENDED_ID) {
		rx_frame->can_id |= CAN_EFF_FLAG;
	}

	if (RxHeader.RxFrameType == FDCAN_REMOTE_FRAME) {
		rx_frame->can_id |= CAN_RTR_FLAG;
	}

	rx_frame->can_dlc = (RxHeader.DataLength & 0x000F0000) >> 16;

	if (RxHeader.FDFormat == FDCAN_FD_CAN) {
		/* this is a CAN-FD frame */
		rx_frame->flags = GS_CAN_FLAG_FD;
		if (RxHeader.BitRateSwitch == FDCAN_BRS_ON) {
			rx_frame->flags |= GS_CAN_FLAG_BRS;
		}
		memcpy(rx_frame->canfd.data, can_rx_data_buff, 64);
	}
	else {
		memcpy(rx_frame->classic_can.data, can_rx_data_buff, 8);
	}

	return true;
#endif
}

bool can_send(CAN_HANDLE_TYPEDEF *hcan, struct GS_HOST_FRAME *frame)
{
#if defined(STM32F0) || defined (STM32F4)
  CAN_TxHeaderTypeDef TxHeader;

	TxHeader.StdId = frame->can_id & 0x7FF;
	TxHeader.ExtId = frame->can_id & 0x1FFFFFFF;
	TxHeader.RTR = frame->can_id & CAN_RTR_FLAG ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	TxHeader.IDE = frame->can_id & CAN_EFF_FLAG ? CAN_ID_EXT : CAN_ID_STD;
	TxHeader.DLC = frame->can_dlc;
	TxHeader.TransmitGlobalTime = DISABLE;

	memcpy(can_tx_data_buff, frame->classic_can.data, 8);

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, can_tx_data_buff, &TxMailbox) != HAL_OK) {
		return false;
	}
	else {
		return true;
	}
#elif defined(STM32G0)
	FDCAN_TxHeaderTypeDef TxHeader;

	TxHeader.DataLength = frame->can_dlc << 16;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	TxHeader.TxFrameType = frame->can_id & CAN_RTR_FLAG ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;


	if (frame->can_id & CAN_EFF_FLAG) {
		TxHeader.IdType = FDCAN_EXTENDED_ID;
		TxHeader.Identifier = frame->can_id & 0x1FFFFFFF;
	}
	else {
		TxHeader.IdType = FDCAN_STANDARD_ID;
		TxHeader.Identifier = frame->can_id & 0x7FF;
	}

	if (frame->flags & GS_CAN_FLAG_FD) {
		TxHeader.FDFormat = FDCAN_FD_CAN;
		if (frame->flags & GS_CAN_FLAG_BRS) {
			TxHeader.BitRateSwitch = FDCAN_BRS_ON;
		}
		else {
			TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
		}
		memcpy(can_tx_data_buff, frame->canfd.data, 64);
	}
	else {
		TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
		TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
		memcpy(can_tx_data_buff, frame->classic_can.data, 8);
	}

	if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, can_tx_data_buff) != HAL_OK) {
			return false;
	}
	else {
			return true;
	}
#endif
}

uint32_t can_get_error_status(CAN_HANDLE_TYPEDEF *hcan)
{
#if defined(STM32F0) || defined (STM32F4)
	uint32_t err = hcan->Instance->ESR;
	/* Write 7 to LEC so we know if it gets set to the same thing again */
	hcan->Instance->ESR = 7<<4;
	return err;
#elif defined(STM32G0)
	uint32_t err = hcan->Instance->PSR;
  /* Write 7 to LEC so we know if it gets set to the same thing again */
  hcan->Instance->PSR = 7;
	return err;
#endif
}

static bool status_is_active(uint32_t err)
{
#if defined(STM32F0) || defined (STM32F4)
	return !(err & (CAN_ESR_BOFF | CAN_ESR_EPVF));
#elif defined(STM32G0)
	return !(err & (FDCAN_PSR_BO | FDCAN_PSR_EP));
#endif
}

bool can_parse_error_status(uint32_t err, uint32_t last_err, CAN_HANDLE_TYPEDEF *hcan, struct GS_HOST_FRAME *frame)
{
	/* We build up the detailed error information at the same time as we decide
	 * whether there's anything worth sending. This variable tracks that final
	 * result. */
	bool should_send = false;

	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG;
	frame->can_dlc = CAN_ERR_DLC;
	frame->classic_can.data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->classic_can.data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->classic_can.data[2] = CAN_ERR_PROT_UNSPEC;
	frame->classic_can.data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->classic_can.data[4] = CAN_ERR_TRX_UNSPEC;
	frame->classic_can.data[5] = 0;
	frame->classic_can.data[6] = 0;
	frame->classic_can.data[7] = 0;

	/* We transitioned from passive/bus-off to active, so report the edge. */
	if (!status_is_active(last_err) && status_is_active(err)) {
		frame->can_id |= CAN_ERR_CRTL;
		frame->classic_can.data[1] |= CAN_ERR_CRTL_ACTIVE;
		should_send = true;
	}

#if defined(STM32F0) || defined(STM32F4)
  UNUSED(hcan);
	if (err & CAN_ESR_BOFF) {
		if (!(last_err & CAN_ESR_BOFF)) {
			/* We transitioned to bus-off. */
			frame->can_id |= CAN_ERR_BUSOFF;
			should_send = true;
		}
		// - tec (overflowed) / rec (looping, likely used for recessive counting)
		//   are not valid in the bus-off state.
		// - The warning flags remains set, error passive will cleared.
		// - LEC errors will be reported, while the device isn't even allowed to send.
		//
		// Hence only report bus-off, ignore everything else.
		return should_send;
	}

	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	 * place as any. */
	frame->classic_can.data[6] = (err>>16) & 0xFF;
	frame->classic_can.data[7] = (err>>24) & 0xFF;

	if (err & CAN_ESR_EPVF) {
		if (!(last_err & CAN_ESR_EPVF)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
			should_send = true;
		}
	} else if (err & CAN_ESR_EWGF) {
		if (!(last_err & CAN_ESR_EWGF)) {
			frame->can_id |= CAN_ERR_CRTL;
			frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
			should_send = true;
		}
	}

	uint8_t lec = (err>>4) & 0x07;
#elif defined(STM32G0)
	if (err & FDCAN_PSR_BO) {
		if (!(last_err & FDCAN_PSR_BO)) {
			/* We transitioned to bus-off. */
			frame->can_id |= CAN_ERR_BUSOFF;
			should_send = true;
		}
	}

	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	* place as any. */
	// TX error count
	frame->classic_can.data[6] = ((hcan->Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);
	// RX error count
	frame->classic_can.data[7] = ((hcan->Instance->ECR & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);

	if (err & FDCAN_PSR_EP) {
	if (!(last_err & FDCAN_PSR_EP)) {
		frame->can_id |= CAN_ERR_CRTL;
		frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
		should_send = true;
	}
	} else if (err & FDCAN_PSR_EW) {
	if (!(last_err & FDCAN_PSR_EW)) {
		frame->can_id |= CAN_ERR_CRTL;
		frame->classic_can.data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
		should_send = true;
	}
	}

  uint8_t lec = err & FDCAN_PSR_LEC;
#endif

	switch (lec) {
		case 0x01: /* stuff error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can.data[2] |= CAN_ERR_PROT_STUFF;
			should_send = true;
			break;
		case 0x02: /* form error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can.data[2] |= CAN_ERR_PROT_FORM;
			should_send = true;
			break;
		case 0x03: /* ack error */
			frame->can_id |= CAN_ERR_ACK;
			should_send = true;
			break;
		case 0x04: /* bit recessive error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can.data[2] |= CAN_ERR_PROT_BIT1;
			should_send = true;
			break;
		case 0x05: /* bit dominant error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can.data[2] |= CAN_ERR_PROT_BIT0;
			should_send = true;
			break;
		case 0x06: /* CRC error */
			frame->can_id |= CAN_ERR_PROT;
			frame->classic_can.data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			should_send = true;
			break;
		default: /* 0=no error, 7=no change */
			break;
	}
	return should_send;
}
