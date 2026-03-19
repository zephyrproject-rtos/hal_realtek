/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

static const char *const TAG = "GDMA";

void GDMA_Init(u8 GDMA_Index, u8 GDMA_ChNum, PGDMA_InitTypeDef GDMA_InitStruct)
{
	u32 CtlxLow = 0;
	u32 CtlxUp = 0;
	u32 CfgxLow = 0;
	u32 CfgxUp = 0;
	u32 ChEn = BIT(GDMA_ChNum) | BIT((GDMA_ChNum + 8));
	u32 BlockSize = GDMA_InitStruct->GDMA_BlockSize;
	GDMA_TypeDef *GDMA = ((GDMA_TypeDef *) GDMA_BASE);

	if (TrustZone_IsSecure()) {
		GDMA = ((GDMA_TypeDef *) GDMA0S_REG_BASE);
	}

	/* Check the parameters */

	assert_param(IS_GDMA_Index(GDMA_Index));
	assert_param(IS_GDMA_ChannelNum(GDMA_ChNum));

	if (IS_GDMA_DIR(GDMA_InitStruct->GDMA_DIR) == 0) {
		RTK_LOGE(TAG, "AmebaD does not support this transmission direction setting\n");
		return;
	}
	/* first open secure transfer, then set other registers */
	if (GDMA_InitStruct->SecureTransfer) {
		GDMA->CH[GDMA_ChNum].CFG_HIGH &= ~BIT_CFGX_UP_SEC_DISABLE;
	} else {
		GDMA->CH[GDMA_ChNum].CFG_HIGH |= BIT_CFGX_UP_SEC_DISABLE;
	}

	/* Enable GDMA in DmaCfgReg*/
	GDMA->DmaCfgReg = 1;

	/* Check chanel is avaliable */
	if (GDMA->ChEnReg & ChEn) {
		/* Disable Channel */
		RTK_LOGW(TAG, "Channel had used; Disable Channel!!!!\n");

		GDMA_Cmd(GDMA_Index, GDMA_ChNum, DISABLE);

	}

	/* Check if there are the pending isr; TFR, Block, Src Tran, Dst Tran, Error */
	GDMA_ClearINT(GDMA_Index, GDMA_ChNum);

	/* Fill in SARx register */
	GDMA->CH[GDMA_ChNum].SAR = GDMA_InitStruct->GDMA_SrcAddr;
	/* Fill in DARx register */
	GDMA->CH[GDMA_ChNum].DAR = GDMA_InitStruct->GDMA_DstAddr;

	/* Process CTLx */
	CtlxLow = BIT_CTLX_LO_INT_EN |
			  (GDMA_InitStruct->GDMA_DstDataWidth << 1) |
			  (GDMA_InitStruct->GDMA_SrcDataWidth << 4) |
			  (GDMA_InitStruct->GDMA_DstInc << 7) |
			  (GDMA_InitStruct->GDMA_SrcInc << 9) |
			  (GDMA_InitStruct->GDMA_DstMsize << 11) |
			  (GDMA_InitStruct->GDMA_SrcMsize << 14) |
			  (GDMA_InitStruct->GDMA_DIR << 20) |
			  (GDMA_InitStruct->GDMA_LlpDstEn << 27) |
			  (GDMA_InitStruct->GDMA_LlpSrcEn << 28);


	CtlxUp = (BlockSize & BIT_CTLX_UP_BLOCK_BS) /*|
		BIT_CTLX_UP_DONE*/;


	/* Fill in CTLx register */
	GDMA->CH[GDMA_ChNum].CTL_LOW = CtlxLow;
	GDMA->CH[GDMA_ChNum].CTL_HIGH = CtlxUp;

	/* Program CFGx */
	CfgxLow = (GDMA_InitStruct->GDMA_ReloadSrc << 30) |
			  (GDMA_InitStruct->GDMA_ReloadDst << 31);

	CfgxUp = GDMA->CH[GDMA_ChNum].CFG_HIGH;
	CfgxUp &= ~(BIT_CFGX_UP_SRC_PER | BIT_CFGX_UP_DEST_PER);
	CfgxUp |= (GDMA_InitStruct->GDMA_SrcHandshakeInterface << 7) |
			  (GDMA_InitStruct->GDMA_DstHandshakeInterface << 11);

	GDMA->CH[GDMA_ChNum].CFG_LOW = CfgxLow;
	GDMA->CH[GDMA_ChNum].CFG_HIGH = CfgxUp;

	GDMA_INTConfig(GDMA_Index, GDMA_ChNum, GDMA_InitStruct->GDMA_IsrType, ENABLE);
}

/**
  * @brief  Set LLP mode when use GDMA LLP (AmebaZ not support this mode).
  * @param  GDMA_Index: 0.
  * @param  GDMA_ChNum: 0 ~ 7.
  * @param  MultiBlockCount: block counts.
  * @param  pGdmaChLli: LLP node list, point to a Linked List Item.
  * @param  round: 0: Linear list, last node id last one 1:last node is not the end.
  * @retval   None
  */
void GDMA_SetLLP(u8 GDMA_Index, u8 GDMA_ChNum, u32 MultiBlockCount, struct GDMA_CH_LLI *pGdmaChLli, u32 round)
{
	u32 CtlxLow, CtlxUp;
	PGDMA_CH_LLI_ELE pLliEle = &pGdmaChLli->LliEle;
	GDMA_TypeDef *GDMA = ((GDMA_TypeDef *) GDMA_BASE);

	if (TrustZone_IsSecure()) {
		GDMA = ((GDMA_TypeDef *) GDMA0S_REG_BASE);
	}

	/* Check the parameters */
	assert_param(IS_GDMA_Index(GDMA_Index));
	assert_param(IS_GDMA_ChannelNum(GDMA_ChNum));

	//DBG_GDMA_INFO("Block Count %lu\n", MultiBlockCount);

	CtlxLow = GDMA->CH[GDMA_ChNum].CTL_LOW;
	CtlxUp = GDMA->CH[GDMA_ChNum].CTL_HIGH;

	CtlxUp = (pGdmaChLli->BlockSize & BIT_CTLX_UP_BLOCK_BS) | /*BIT_CTLX_UP_DONE | */CtlxUp;
	GDMA->CH[GDMA_ChNum].CTL_HIGH = CtlxUp;

	/* LLP register */
	GDMA->CH[GDMA_ChNum].LLP = (u32) & (pGdmaChLli->LliEle);

	while (MultiBlockCount > 0) {
		pLliEle = &pGdmaChLli->LliEle;

		/* Clear the last element llp enable bit */
		if ((1 == MultiBlockCount) & (~round)) {
			CtlxLow &= ~(BIT_CTLX_LO_LLP_DST_EN |
						 BIT_CTLX_LO_LLP_SRC_EN);
		}
		/* Update block size for transfer */
		CtlxUp &= ~(BIT_CTLX_UP_BLOCK_BS);
		CtlxUp |= (pGdmaChLli->BlockSize & BIT_CTLX_UP_BLOCK_BS);

		/* Updatethe Llpx context */
		pLliEle->CtlxLow = CtlxLow;
		pLliEle->CtlxUp = CtlxUp;
		pLliEle->Llpx = (u32)&pGdmaChLli->pNextLli->LliEle;

		DCache_CleanInvalidate((u32)pGdmaChLli, sizeof(struct GDMA_CH_LLI));

		/* Update the Lli and Block size list point to next llp */
		pGdmaChLli = pGdmaChLli->pNextLli;
		MultiBlockCount--;
	}
}
/**
  * @brief  Set channel priority.
  * @param  GDMA_Index: 0.
  * @param  GDMA_ChNum: 0 ~ 7.
  * @retval   IrqNum
  */
void GDMA_SetChnlPriority(u8 GDMA_Index, u8 GDMA_ChNum, u32 ChnlPriority)
{
	(void)GDMA_Index;
	(void)GDMA_ChNum;
	(void)ChnlPriority;
	RTK_LOGW(TAG, "AmebaD does not support priority setting\n");
}

/**
 * @brief Get whether the fifo is empty
 *
 * @param  GDMA_Index: 0.
 * @param  GDMA_ChNum: 0 ~ 7.
 * @retval TRUE/FALSE
 */
__weak u8
GDMA_ChnlFIFOIsEmpty(u8 GDMA_Index, u8 GDMA_ChNum)
{
	GDMA_TypeDef *GDMA = ((GDMA_TypeDef *) GDMA_BASE);
	u8 ret = FALSE;
	/* Check the parameters */
	assert_param(IS_GDMA_Index(GDMA_Index));
	assert_param(IS_GDMA_ChannelNum(GDMA_ChNum));

	if (TrustZone_IsSecure()) {
		GDMA = ((GDMA_TypeDef *) GDMA0S_REG_BASE);
	}

	if (GDMA->CH[GDMA_ChNum].CFG_LOW & BIT_CFGx_L_FIFO_EMPTY) {
		ret = TRUE;
	}

	return  ret;
}
/**
  * @brief  Suspend a channel.
  * @param  GDMA_Index: 0.
  * @param  GDMA_ChNum: 0 ~ 7.
  */
__weak
void GDMA_Suspend(u8 GDMA_Index, u8 GDMA_ChNum)
{
	GDMA_TypeDef *GDMA = ((GDMA_TypeDef *) GDMA_BASE);
	if (TrustZone_IsSecure()) {
		GDMA = ((GDMA_TypeDef *) GDMA0S_REG_BASE);
	}
	/* Check the parameters */
	assert_param(IS_GDMA_Index(GDMA_Index));
	assert_param(IS_GDMA_ChannelNum(GDMA_ChNum));

	GDMA->CH[GDMA_ChNum].CFG_LOW |= BIT_CFGx_L_CH_SUSP;
}

/**
  * @brief  Resume a channel.
  * @param  GDMA_Index: 0.
  * @param  GDMA_ChNum: 0 ~ 7.
  */
__weak
void GDMA_Resume(u8 GDMA_Index, u8 GDMA_ChNum)
{
	GDMA_TypeDef *GDMA = ((GDMA_TypeDef *) GDMA_BASE);
	if (TrustZone_IsSecure()) {
		GDMA = ((GDMA_TypeDef *) GDMA0S_REG_BASE);
	}
	/* Check the parameters */
	assert_param(IS_GDMA_Index(GDMA_Index));
	assert_param(IS_GDMA_ChannelNum(GDMA_ChNum));

	GDMA->CH[GDMA_ChNum].CFG_LOW &= ~BIT_CFGx_L_CH_SUSP;
}

/**
  * @brief  Abort a channel.
  * @param  GDMA_Index: 0.
  * @param  GDMA_ChNum: 0 ~ 7.
  * @retval TRUE/FALSE
  */
__weak
u8 GDMA_Abort(u8 GDMA_Index, u8 GDMA_ChNum)
{
	u32 timeout = 500;
	GDMA_TypeDef *GDMA = ((GDMA_TypeDef *) GDMA_BASE);
	if (TrustZone_IsSecure()) {
		GDMA = ((GDMA_TypeDef *) GDMA0S_REG_BASE);
	}
	/* Check the parameters */
	assert_param(IS_GDMA_Index(GDMA_Index));
	assert_param(IS_GDMA_ChannelNum(GDMA_ChNum));

	GDMA_Suspend(GDMA_Index, GDMA_ChNum);
	/*If ChEnReg[GDMA_ChNum] is not equal to 0, it means that
	  the channel is working and the Suspend status must be checked.*/
	while (timeout--) {
		if ((GDMA->ChEnReg & BIT(GDMA_ChNum)) == 0 || \
			(GDMA->CH[GDMA_ChNum].CFG_LOW & BIT_CFGx_L_INACTIVE)) {
			break;
		}
	}
	/*If the channel is still active after the timeout period, resume is required*/
	if (timeout == 0) {
		GDMA_Resume(GDMA_Index, GDMA_ChNum);
		return FALSE;
	}

	GDMA_Cmd(GDMA_Index, GDMA_ChNum, DISABLE);
	return TRUE;
}

/**
 * @brief Check if the channel is active
 *
 * @param GDMA_Index 0
 * @param GDMA_ChNum 0 ~ 7
 * @retval TRUE/FALSE
 */
__weak
u8 GDMA_ChannelIsActive(u8 GDMA_Index, u8 GDMA_ChNum)
{
	GDMA_TypeDef *GDMA = ((GDMA_TypeDef *) GDMA_BASE);
	if (TrustZone_IsSecure()) {
		GDMA = ((GDMA_TypeDef *) GDMA0S_REG_BASE);
	}
	/* Check the parameters */
	assert_param(IS_GDMA_Index(GDMA_Index));
	assert_param(IS_GDMA_ChannelNum(GDMA_ChNum));

	if ((GDMA->ChEnReg & BIT(GDMA_ChNum)) && \
		((GDMA->CH[GDMA_ChNum].CFG_LOW & BIT_CFGx_L_INACTIVE) == 0)) {
		return TRUE;
	}
	return FALSE;
}
