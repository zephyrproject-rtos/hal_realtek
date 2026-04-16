/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file can_stubs.c
 * @brief Stub implementations for CAN (rtl_can.h) library functions.
 *        Used when building without binary blobs (CONFIG_BUILD_ONLY_NO_BLOBS).
 */

#include <stdbool.h>
#include <stdint.h>
#include <rtl_can.h>

/* Stub implementations for CAN functions */
void CAN_DeInit(void)
{
}

void CAN_Init(CAN_InitTypeDef *CAN_InitStruct)
{
	(void)CAN_InitStruct;
}

void CAN_StructInit(CAN_InitTypeDef *CAN_InitStruct)
{
	(void)CAN_InitStruct;
}

void CAN_Cmd(FunctionalState NewState)
{
	(void)NewState;
}

void CAN_INTConfig(uint32_t CAN_INT, FunctionalState newState)
{
	(void)CAN_INT;
	(void)newState;
}

#if (CAN_SUPPORT_INT_MSK_STS == 1)
void CAN_MaskINTConfig(uint32_t CAN_INT_FLAG, FunctionalState NewState)
{
	(void)CAN_INT_FLAG;
	(void)NewState;
}
#endif

ITStatus CAN_GetINTRawStatus(uint32_t CAN_INT_FLAG)
{
	(void)CAN_INT_FLAG;
	return 0;
}

ITStatus CAN_GetINTStatus(uint32_t CAN_INT_FLAG)
{
	(void)CAN_INT_FLAG;
	return 0;
}

void CAN_ClearINTPendingBit(uint32_t CAN_INT_FLAG)
{
	(void)CAN_INT_FLAG;
}

FlagStatus CAN_GetErrorStatus(uint32_t CAN_ERR_STAT)
{
	(void)CAN_ERR_STAT;
	return 0;
}

void CAN_CLearErrorStatus(uint32_t CAN_ERR_STAT)
{
	(void)CAN_ERR_STAT;
}

CANError_TypeDef CAN_SetMsgBufTxMode(CANTxFrame_TypeDef *p_tx_frame_params,
				     const uint8_t *p_frame_data,
				     uint8_t data_len)
{
	(void)p_tx_frame_params;
	(void)p_frame_data;
	(void)data_len;
	return CAN_RAM_STATE_ERR;
}

CANError_TypeDef CAN_SetMsgBufRxMode(CANRxFrame_TypeDef *p_rx_frame_params)
{
	(void)p_rx_frame_params;
	return CAN_RAM_STATE_ERR;
}

CANError_TypeDef CAN_GetMsgBufInfo(uint8_t msg_buf_id,
				   CANMsgBufInfo_TypeDef *p_mb_info)
{
	(void)msg_buf_id;
	(void)p_mb_info;
	return CAN_RAM_STATE_ERR;
}

CANError_TypeDef CAN_GetRamData(uint8_t data_len, uint8_t *p_data)
{
	(void)data_len;
	(void)p_data;
	return CAN_RAM_STATE_ERR;
}

CANDataFrameSel_TypeDef CAN_CheckFrameType(uint8_t rtr_bit, uint8_t ide_bit)
{
	(void)rtr_bit;
	(void)ide_bit;
	return CAN_INVALID_DATA_FRAME;
}

void CAN_MBTxINTConfig(uint8_t message_buffer_index, FunctionalState newState)
{
	(void)message_buffer_index;
	(void)newState;
}

void CAN_MBRxINTConfig(uint8_t message_buffer_index, FunctionalState newState)
{
	(void)message_buffer_index;
	(void)newState;
}

void CAN_GetFifoStatus(CANFifoStatus_TypeDef *CAN_FifoStatus)
{
	(void)CAN_FifoStatus;
}

void CAN_TxTriggerConfig(FunctionalState newState,
			 uint16_t trigger_timestamp_begin,
			 uint16_t close_offset)
{
	(void)newState;
	(void)trigger_timestamp_begin;
	(void)close_offset;
}

uint32_t CAN_GetBusState(void)
{
	return 0;
}

uint32_t CAN_GetRamState(void)
{
	return 0;
}

FlagStatus CAN_GetMBnTxDoneFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
	return 0;
}

void CAN_ClearMBnTxDoneFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
}

FlagStatus CAN_GetMBnTxErrorFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
	return 0;
}

void CAN_ClearMBnTxErrorFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
}

FlagStatus CAN_GetMBnStatusTxFinishFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
	return 0;
}

FlagStatus CAN_GetMBnStatusTxReqFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
	return 0;
}

FlagStatus CAN_GetMBnRxDoneFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
	return 0;
}

void CAN_ClearMBnRxDoneFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
}

FlagStatus CAN_GetMBnStatusRxValidFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
	return 0;
}

FlagStatus CAN_GetMBnStatusRxReadyFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
	return 0;
}

void CAN_TimeStampConfig(FunctionalState newState)
{
	(void)newState;
}

uint16_t CAN_GetTimeStampCount(void)
{
	return 0;
}

uint32_t CAN_GetRxDmaMsize(void)
{
	return 0;
}

FlagStatus CAN_GetMBnRxDmaEnFlag(uint8_t message_buffer_index)
{
	(void)message_buffer_index;
	return 0;
}

void CAN_SetMBnRxDmaEnFlag(uint8_t message_buffer_index,
			   FunctionalState newState)
{
	(void)message_buffer_index;
	(void)newState;
}

void CAN_SetTestMode(uint8_t CAN_TestModeSel)
{
	(void)CAN_TestModeSel;
}

void CAN_AutoReTxCmd(FunctionalState NewState)
{
	(void)NewState;
}

void CAN_SetTiming(CAN_BIT_TIMING_TYPE_TypeDef *CAN_BitTiming)
{
	(void)CAN_BitTiming;
}

FlagStatus CAN_GetErrorPassiveStatus(void)
{
	return 0;
}

FlagStatus CAN_GetErrorWarningStatus(void)
{
	return 0;
}

int CAN_GetTxErrorCnt(void)
{
	return 0;
}

int CAN_GetRxErrorCnt(void)
{
	return 0;
}

void CAN_ClkDivConfig(CANClockDiv_TypeDef div)
{
	(void)div;
}

bool CAN_ClkGet(CANClockSrc_TypeDef *ClockSrc, CANClockDiv_TypeDef *ClockDiv)
{
	(void)ClockSrc;
	(void)ClockDiv;
	return false;
}

#if (CAN_SUPPORT_SLEEP_MODE == 1)
FlagStatus CAN_CheckSleepStatus(void)
{
	return 0;
}

void CAN_LowPowerClkCmd(FunctionalState newState, CANLowPowerClkDIV_TypeDef div)
{
	(void)newState;
	(void)div;
}

void CAN_SetWakeUpPinFltFunction(FunctionalState newState, uint8_t flt_length)
{
	(void)newState;
	(void)flt_length;
}

void CAN_RequestToSleepMode(void)
{
}

void CAN_ManualWakeup(void)
{
}
#endif /* CAN_SUPPORT_SLEEP_MODE */