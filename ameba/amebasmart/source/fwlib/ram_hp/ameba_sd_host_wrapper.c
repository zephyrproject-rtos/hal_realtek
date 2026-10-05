/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

static const char *const TAG = "SDHOST";

/** @addtogroup Ameba_Periph_Driver
  * @{
  */

/** @addtogroup SDHOST_ABSTRACT
  * @{
  */

/* Private defines -----------------------------------------------------------*/

/* Highest bus clock still served by the card identification mode (390.625 kHz). */
#define SD_HOST_ID_MODE_MAX_KHZ		400U
/* Bus clock ceilings of the SD 2.0 divider taps, base clock is 100 MHz. */
#define SD_HOST_CLK_DIV8_MAX_KHZ	12500U
#define SD_HOST_CLK_DIV4_MAX_KHZ	25000U
/* The only block length the DMA engine transfers through its wide window. */
#define SD_HOST_BLOCK_SIZE		512U

/* Private variables ---------------------------------------------------------*/

/* One transfer's parameters, carried across the split ConfigData/ConfigDMA/
 * SendCommand/WaitResp calls that the SDIOH primitives take at once. */
static struct {
	u8 initialised;    /* SDIO_HostInit() has run, so the host FSM is alive */
	u8 in_id_mode;     /* Host is in card identification mode (390 kHz, 1-bit) */
	u8 bus_width;      /* Bus width last requested, restored on leaving id mode */
	u8 data_present;   /* Current command has a data phase */
	u8 trans_dir;      /* SDIO_TRANS_CARD_TO_HOST or SDIO_TRANS_HOST_TO_CARD */
	u8 long_resp;      /* Current command's response arrives over DMA */
	u8 bounce;         /* Data phase goes through sd_host_buf */
	u16 block_size;
	u16 block_cnt;
	u32 cmd_error;     /* Error raised while issuing the current command */
	u32 user_addr;     /* Final destination of a bounced transfer */
	u32 xfer_len;      /* Requested byte count of a bounced transfer */
} sd_host;

/* Staging buffer for the narrow DMA window (long R2 responses and sub-block reads):
 * SDIOH_DMAConfig() always fills the full 64-byte window, overrunning a shorter
 * caller buffer. Aligned to one window so cache ops stay off neighbouring data. */
static u8 sd_host_buf[SDIOH_C6R2_BUF_LEN] __attribute__((aligned(SDIOH_C6R2_BUF_LEN)));

/* DMA-completion sync hooks: take on the poll path, give in SD_IRQHandler.
 * NULL means poll-only. Install with SD_SetSema(). */
int (*sd_sema_take_fn)(u32);
int (*sd_sema_give_isr_fn)(u32);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Check that a host pointer refers to the single AmebaSmart SD host.
  * @param  SDIOx Pointer to the SD host register block.
  * @return HAL operation result:
  *           - HAL_OK: Pointer refers to the SD host.
  *           - HAL_ERR_PARA: Pointer refers to something else.
  */
static u32 SD_HostVerify(SDIOHOST_TypeDef *SDIOx)
{
	if (SDIOx != (SDIOHOST_TypeDef *)SDIOH_BASE) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "invalid SD host instance 0x%08x\n", (u32)SDIOx);
		return HAL_ERR_PARA;
	}

	return HAL_OK;
}

/**
  * @brief  Map a response type onto the response length the host has to latch.
  * @param  RespType Response type, a value of @ref SDHOST_ABSTRACT_Response_Type.
  * @return One of SDIOH_NO_RESP, SDIOH_RSP_6B or SDIOH_RSP_17B.
  */
static u8 SD_HostRespLen(u8 RespType)
{
	if (RespType == SDMMC_RSP_NONE) {
		return SDIOH_NO_RESP;
	} else if (RespType == SDMMC_RSP_R2) {
		return SDIOH_RSP_17B;
	} else {
		return SDIOH_RSP_6B;
	}
}

/**
  * @brief  Check whether the host can pick a work mode for a data command.
  * @param  CmdIndex Command index.
  * @return 1 if the command is supported with a data phase, 0 otherwise.
  */
static u8 SD_HostDataCmdSupported(u8 CmdIndex)
{
	switch (CmdIndex) {
	case SD_CMD_SwitchFunc:     /* CMD6  */
	case SD_CMD_SendSts:        /* ACMD13 */
	case SD_CMD_RdSingleBlk:    /* CMD17 */
	case SD_CMD_RdMulBlk:       /* CMD18 */
	case SD_CMD_WrBlk:          /* CMD24 */
	case SD_CMD_WrMulBlk:       /* CMD25 */
	case SD_CMD_SendScr:        /* ACMD51 */
	case EMMC_CMD_SendExtCsd:   /* CMD8 on eMMC; SD's CMD8 is response-only, never reaches here */
		return 1;
	default:
		return 0;
	}
}

/**
  * @brief  Translate a host error status word into an SD error code.
  * @param  ErrStatus Error status as reported by SDIOH_CheckTxError().
  * @return One of @ref SDHOST_ABSTRACT_Error_Code.
  */
static u32 SD_HostErrorCode(u16 ErrStatus)
{
	if (ErrStatus & SDIOH_SD_CMD_RSP_TO_ERR) {
		return SD_ERROR_CMD_RSP_TIMEOUT;
	} else if (ErrStatus & SDIOH_CRC7_ERR) {
		return SD_ERROR_CMD_CRC_FAIL;
	} else if (ErrStatus & (SDIOH_CRC16_ERR | SDIOH_WR_CRC_ERR)) {
		return SD_ERROR_DATA_CRC_FAIL;
	} else if (ErrStatus & SDIOH_GET_WRCRC_STA_TO_ERR) {
		return SD_ERROR_DATA_TIMEOUT;
	} else {
		return SD_ERROR_GENERAL_UNKNOWN_ERR;
	}
}

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SDHOST_ABSTRACT_Exported_Functions
  * @{
  */

/**
  * @brief  Route the SD host to its clock source.
  * @note   The AmebaSmart SD host is hard-wired to the HS AHB clock at 100 MHz,
  *         so there is no source to select. The divider taps applied later by
  *         SDIO_ConfigClock() are inside the host itself.
  */
void SDIO_ClockSourceInit(void)
{
}

/**
  * @brief  Initialize the SD host and enter card identification mode.
  * @param  SDIOx Pointer to the SD host register block.
  * @return HAL operation result:
  *           - HAL_OK: SD host initialized successfully.
  *           - Others: SD host failed to initialize.
  */
u32 SDIO_HostInit(SDIOHOST_TypeDef *SDIOx)
{
	u32 ret;

	ret = SD_HostVerify(SDIOx);
	if (ret != HAL_OK) {
		return ret;
	}

	/* The width here only seeds SDIOH_CheckBusState()'s idle-level mask; pass the
	 * widest bus so it covers every data line whatever width is negotiated later. */
	ret = SDIOH_Init(SDIOH_BUS_WIDTH_4BIT);
	if (ret != HAL_OK) {
		return ret;
	}

	/* SDIOH_Init() leaves the host in card identification mode. */
	sd_host.in_id_mode = 1;
	sd_host.bus_width = SDIOH_BUS_WIDTH_1BIT;
	sd_host.initialised = 1;

	return HAL_OK;
}

/**
  * @brief  Reset the SD host back to its post-initialization state.
  * @param  SDIOx Pointer to the SD host register block.
  * @return HAL operation result:
  *           - HAL_OK: SD host reset successfully.
  *           - Others: SD host failed to reset.
  */
u32 SDIO_ResetAll(SDIOHOST_TypeDef *SDIOx)
{
	u32 ret;

	ret = SD_HostVerify(SDIOx);
	if (ret != HAL_OK) {
		return ret;
	}

	/* Runs before SDIO_HostInit() on the probe path: bring the peripheral clock
	 * up before touching registers. Idempotent. */
	RCC_PeriphClockCmd(APBPeriph_SDH, APBPeriph_SDH_CLOCK, ENABLE);

	SDIOH_DMAReset();
	SDIOH_INTClearPendingBit(SDIOH_SD_ISR_ALL);

	if (sd_host.initialised) {
		/* Back to identification mode (390 kHz, 1-bit), like a standard host after
		 * reset. Skipped on the probe path where the FSM is not running yet. */
		ret = SDIOH_InitialModeCmd(ENABLE, SDIOH_SIG_VOL_33);
		if (ret != HAL_OK) {
			return ret;
		}

		sd_host.in_id_mode = 1;
		sd_host.bus_width = SDIOH_BUS_WIDTH_1BIT;
	}

	sd_host.bounce = 0;
	sd_host.long_resp = 0;
	sd_host.cmd_error = SD_ERROR_NONE;
	sd_host.data_present = SDIO_TRANS_NO_DATA;

	return HAL_OK;
}

/**
  * @brief  Check whether the SD host is idle.
  * @param  SDIOx Pointer to the SD host register block.
  * @return HAL operation result:
  *           - HAL_OK: SD host is idle.
  *           - Others: SD host is busy.
  */
u32 SDIO_CheckState(SDIOHOST_TypeDef *SDIOx)
{
	u32 ret;

	ret = SD_HostVerify(SDIOx);
	if (ret != HAL_OK) {
		return ret;
	}

	/* The command and data state machines only report idle once the host runs. */
	if (!sd_host.initialised) {
		return HAL_OK;
	}

	return SDIOH_Busy();
}

/**
  * @brief  Turn the card interface on.
  * @param  SDIOx Pointer to the SD host register block.
  * @note   AmebaSmart has no card power switch. The closest equivalent is the SD
  *         module clock: gating it stops SDCLK, which is what a standard host
  *         controller achieves by removing bus power.
  */
void SDIO_PowerState_ON(SDIOHOST_TypeDef *SDIOx)
{
	if (SD_HostVerify(SDIOx) != HAL_OK) {
		return;
	}

	SDIOx->CARD_CLK_EN_CTL = SDIOH_SD_CARD_MOUDLE_EN;
}

/**
  * @brief  Turn the card interface off.
  * @param  SDIOx Pointer to the SD host register block.
  * @note   See SDIO_PowerState_ON().
  */
void SDIO_PowerState_OFF(SDIOHOST_TypeDef *SDIOx)
{
	if (SD_HostVerify(SDIOx) != HAL_OK) {
		return;
	}

	SDIOx->CARD_CLK_EN_CTL = 0;
}

/**
  * @brief  Set the bus clock.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  ClkKHz Requested bus clock in kHz.
  * @return HAL operation result:
  *           - HAL_OK: Bus clock set successfully.
  *           - Others: Failed to set the bus clock.
  */
u32 SDIO_ConfigClock(SDIOHOST_TypeDef *SDIOx, u32 ClkKHz)
{
	u32 ret;
	u8 clk_div;

	ret = SD_HostVerify(SDIOx);
	if (ret != HAL_OK) {
		return ret;
	}

	if (ClkKHz <= SD_HOST_ID_MODE_MAX_KHZ) {
		if (!sd_host.in_id_mode) {
			ret = SDIOH_InitialModeCmd(ENABLE, SDIOH_SIG_VOL_33);
			if (ret != HAL_OK) {
				return ret;
			}

			sd_host.in_id_mode = 1;
		}

		return HAL_OK;
	}

	if (sd_host.in_id_mode) {
		ret = SDIOH_InitialModeCmd(DISABLE, SDIOH_SIG_VOL_33);
		if (ret != HAL_OK) {
			return ret;
		}

		sd_host.in_id_mode = 0;

		/* SDIOH_InitialModeCmd() rewrites SD_CONFIG1 and forces 1-bit, so the
		 * negotiated width must be re-applied. */
		SDIOH_SetBusWidth(sd_host.bus_width);
	}

	if (ClkKHz <= SD_HOST_CLK_DIV8_MAX_KHZ) {
		clk_div = SDIOH_CLK_DIV8;
	} else if (ClkKHz <= SD_HOST_CLK_DIV4_MAX_KHZ) {
		clk_div = SDIOH_CLK_DIV4;
	} else {
		/* 50 MHz, the fastest tap SD 2.0 high speed allows. */
		clk_div = SDIOH_CLK_DIV2;
	}

	SDIOH_SwitchSpeed(clk_div, SDIOH_SD20_MODE);

	return HAL_OK;
}

/**
  * @brief  Set the bus width.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  BusWidth Bus width, SDIOH_BUS_WIDTH_1BIT or SDIOH_BUS_WIDTH_4BIT.
  * @return HAL operation result:
  *           - HAL_OK: Bus width set successfully.
  *           - Others: Failed to set the bus width.
  */
u32 SDIO_ConfigBusWidth(SDIOHOST_TypeDef *SDIOx, u8 BusWidth)
{
	u32 ret;

	ret = SD_HostVerify(SDIOx);
	if (ret != HAL_OK) {
		return ret;
	}

	sd_host.bus_width = BusWidth;

	/* Identification mode pins the bus to 1-bit; record the request and let
	 * SDIO_ConfigClock() apply it on leaving that mode. */
	if (!sd_host.in_id_mode) {
		SDIOH_SetBusWidth(BusWidth);
	}

	return HAL_OK;
}

/**
  * @brief  Describe the data phase of the next command.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  Data Pointer to the data control structure.
  * @note   AmebaSmart programs block length and block count from inside
  *         SDIOH_DMAConfig(), so this only records the parameters.
  */
void SDIO_ConfigData(SDIOHOST_TypeDef *SDIOx, SDIO_DataInitTypeDef *Data)
{
	if (SD_HostVerify(SDIOx) != HAL_OK) {
		return;
	}

	sd_host.block_size = Data->BlockSize;
	sd_host.block_cnt = Data->BlockCnt;
	sd_host.trans_dir = Data->TransDir;
}

/**
  * @brief  Arm the DMA engine for the data phase described by SDIO_ConfigData().
  * @param  SDIOx Pointer to the SD host register block.
  * @param  DmaMode DMA mode, only SDIO_SDMA_MODE is supported.
  * @param  DmaAddr Address of the caller's transfer buffer.
  * @return HAL operation result:
  *           - HAL_OK: DMA armed successfully.
  *           - Others: The requested transfer cannot be performed.
  */
u32 SDIO_ConfigDMA(SDIOHOST_TypeDef *SDIOx, u8 DmaMode, u32 DmaAddr)
{
	SDIOH_DmaCtl dma_cfg;
	u32 ret;
	u32 len;

	ret = SD_HostVerify(SDIOx);
	if (ret != HAL_OK) {
		return ret;
	}

	/* There is a single scatter-gather-less DMA engine, no descriptor modes. */
	if (DmaMode != SDIO_SDMA_MODE) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "unsupported DMA mode %u\n", DmaMode);
		return HAL_ERR_PARA;
	}

	len = (u32)sd_host.block_size * (u32)sd_host.block_cnt;
	sd_host.bounce = 0;
	sd_host.user_addr = DmaAddr;
	sd_host.xfer_len = len;

	if (sd_host.block_size == SD_HOST_BLOCK_SIZE) {
		dma_cfg.type = SDIOH_DMA_NORMAL;
		dma_cfg.blk_cnt = sd_host.block_cnt;
	} else if ((sd_host.trans_dir == SDIO_TRANS_CARD_TO_HOST) && (len <= SDIOH_C6R2_BUF_LEN)) {
		/* Short read, served by the narrow window; see sd_host_buf. */
		dma_cfg.type = SDIOH_DMA_64B;
		dma_cfg.blk_cnt = 1;
		sd_host.bounce = 1;
	} else {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "unsupported transfer: %u x %u bytes, dir %u\n",
				 sd_host.block_cnt, sd_host.block_size, sd_host.trans_dir);
		return HAL_ERR_PARA;
	}

	if (sd_host.bounce) {
		_memset((void *)sd_host_buf, 0, sizeof(sd_host_buf));
		DCache_CleanInvalidate((u32)sd_host_buf, sizeof(sd_host_buf));
		dma_cfg.start_addr = (u32)sd_host_buf / SDIOH_DMA_ALIGN_SZ;
	} else {
		/* The engine takes the address pre-divided, so it has to be aligned. */
		if ((DmaAddr % SDIOH_DMA_ALIGN_SZ) != 0) {
			RTK_LOGS(TAG, RTK_LOG_ERROR, "buffer 0x%08x is not %u-byte aligned\n",
					 DmaAddr, SDIOH_DMA_ALIGN_SZ);
			return HAL_ERR_PARA;
		}

		dma_cfg.start_addr = DmaAddr / SDIOH_DMA_ALIGN_SZ;
	}

	dma_cfg.op = (sd_host.trans_dir == SDIO_TRANS_CARD_TO_HOST) ? SDIOH_DMA_READ : SDIOH_DMA_WRITE;
	SDIOH_DMAConfig(&dma_cfg);

	return HAL_OK;
}

/**
  * @brief  Issue a command on the bus.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  Command Pointer to the command control structure.
  * @note   The command is started but not waited for; use SDIO_WaitResp().
  */
void SDIO_SendCommand(SDIOHOST_TypeDef *SDIOx, SDIO_CmdInitTypeDef *Command)
{
	SDIOH_CmdTypeDef cmd_attr;
	SDIOH_DmaCtl dma_cfg;

	if (SD_HostVerify(SDIOx) != HAL_OK) {
		return;
	}

	sd_host.data_present = Command->DataPresent;
	sd_host.long_resp = 0;
	sd_host.cmd_error = SD_ERROR_NONE;

	if ((Command->DataPresent == SDIO_TRANS_WITH_DATA) &&
		(SD_HostDataCmdSupported(Command->CmdIndex) == 0)) {
		/* The IP picks a work mode from the index and drops the data phase for
		 * unknown ones, so fail loudly instead. */
		RTK_LOGS(TAG, RTK_LOG_ERROR, "CMD%u cannot carry data on this host\n", Command->CmdIndex);
		sd_host.cmd_error = SD_ERROR_UNSUPPORTED_FEATURE;
		return;
	}

	cmd_attr.arg = Command->Argument;
	cmd_attr.idx = Command->CmdIndex;
	cmd_attr.rsp_type = SD_HostRespLen(Command->RespType);
	/* R3 and R4 carry the OCR, which the specification sends without a CRC7. */
	cmd_attr.rsp_crc_chk = ((Command->RespType == SDMMC_RSP_NONE) ||
							(Command->RespType == SDMMC_RSP_R3) ||
							(Command->RespType == SDMMC_RSP_R4)) ? DISABLE : ENABLE;
	cmd_attr.data_present = (Command->DataPresent == SDIO_TRANS_WITH_DATA) ?
							SDIOH_DATA_EXIST : SDIOH_NO_DATA;

	if (cmd_attr.rsp_type == SDIOH_RSP_17B) {
		/*
		 * A long response does not land in registers: the host DMAs all 17 bytes
		 * into memory, so a buffer has to be armed before the command goes out.
		 */
		_memset((void *)sd_host_buf, 0, sizeof(sd_host_buf));
		DCache_CleanInvalidate((u32)sd_host_buf, sizeof(sd_host_buf));

		dma_cfg.op = SDIOH_DMA_READ;
		dma_cfg.start_addr = (u32)sd_host_buf / SDIOH_DMA_ALIGN_SZ;
		dma_cfg.blk_cnt = 1;
		dma_cfg.type = SDIOH_DMA_R2;
		SDIOH_DMAConfig(&dma_cfg);

		sd_host.long_resp = 1;
	}

	/*
	 * Pass a zero timeout so the primitive only starts the transfer: waiting is
	 * split between SDIO_WaitResp() and SD_WaitTransDone(), the latter being able
	 * to block on a semaphore instead of polling.
	 */
	if (SDIOH_SendCommand(&cmd_attr, 0) != HAL_OK) {
		sd_host.cmd_error = SD_ERROR_BUSY;
	}
}

/**
  * @brief  Wait for the response of the command last sent.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  RespType Response type, a value of @ref SDHOST_ABSTRACT_Response_Type.
  * @param  TimeOutUs Timeout value in microseconds.
  * @return One of @ref SDHOST_ABSTRACT_Error_Code, SD_ERROR_NONE on success.
  */
u32 SDIO_WaitResp(SDIOHOST_TypeDef *SDIOx, u8 RespType, u32 TimeOutUs)
{
	u16 err_status;
	u32 ret;

	if (SD_HostVerify(SDIOx) != HAL_OK) {
		return SD_ERROR_UNSUPPORTED_FEATURE;
	}

	if (sd_host.cmd_error != SD_ERROR_NONE) {
		return sd_host.cmd_error;
	}

	if (sd_host.long_resp) {
		/* The response itself arrives over DMA. */
		ret = SDIOH_WaitDMADone(TimeOutUs);
		if (ret != HAL_OK) {
			return SD_ERROR_TIMEOUT;
		}

		DCache_Invalidate((u32)sd_host_buf, sizeof(sd_host_buf));
	} else if (sd_host.data_present == SDIO_TRANS_WITH_DATA) {
		/* Command and data are one indivisible op here; completion is picked up
		 * by SD_WaitTransDone(), which can yield the CPU. */
		return SD_ERROR_NONE;
	} else {
		ret = SDIOH_WaitTxDone(TimeOutUs);
		if (ret != HAL_OK) {
			return SD_ERROR_TIMEOUT;
		}
	}

	/* Host-level errors only (timeout/CRC); R1 card-status bits are decoded by
	 * the protocol layer. */
	if (SDIOH_CheckTxError(&err_status) != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "command error, status 0x%04x\n", err_status);
		return SD_HostErrorCode(err_status);
	}

	return SD_ERROR_NONE;
}

/**
  * @brief  Read one 32-bit word of the response last received.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  Response Word index, SDIO_RESP0 to SDIO_RESP3.
  * @return The requested response word.
  * @note   SDIO_RESP0..SDIO_RESP3 are interpreted as 32-bit word indices here,
  *         whereas SDIOH_GetResponse() takes the same constants as byte indices.
  */
u32 SDIO_GetResponse(SDIOHOST_TypeDef *SDIOx, u8 Response)
{
	const u8 *p;

	if ((SD_HostVerify(SDIOx) != HAL_OK) || (Response > SDIO_RESP3)) {
		return 0;
	}

	if (sd_host.long_resp) {
		/* 17-byte R2: 1 reserved header byte then the 128-bit CID/CSD. Returns the
		 * four words from the header on; the driver's standard <<8 on CMD2/CMD9
		 * drops that header byte and yields the register. */
		p = &sd_host_buf[(SDIO_RESP3 - Response) * 4U];
	} else {
		/* 6-byte response: index, 32-bit payload in bytes 1..4, CRC7. One word. */
		if (Response != SDIO_RESP0) {
			return 0;
		}

		return ((u32)SDIOH_GetResponse(SDIO_RESP1) << 24) |
			   ((u32)SDIOH_GetResponse(SDIO_RESP2) << 16) |
			   ((u32)SDIOH_GetResponse(SDIO_RESP3) << 8) |
			   ((u32)SDIOH_GetResponse(SDIO_RESP4));
	}

	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | (u32)p[3];
}

/**
  * @brief  Send CMD5 (IO_SEND_OP_COND) and report whether it was answered.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  Ocr Operating condition register value to offer.
  * @return One of @ref SDHOST_ABSTRACT_Error_Code, SD_ERROR_NONE on success.
  */
u32 SDIO_CmdSendOpCond(SDIOHOST_TypeDef *SDIOx, u32 Ocr)
{
	SDIO_CmdInitTypeDef cmd;

	cmd.Argument = Ocr;
	cmd.CmdIndex = SDMMC_IO_SEND_OP_COND;
	cmd.CmdType = SDMMC_CMD_NORMAL;
	cmd.RespType = SDMMC_RSP_R4;
	cmd.DataPresent = SDIO_TRANS_NO_DATA;
	SDIO_SendCommand(SDIOx, &cmd);

	return SDIO_WaitResp(SDIOx, cmd.RespType, SDIOH_CMD_CPLT_TIMEOUT);
}

/**
  * @brief  Enable or disable normal interrupt signals.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  SDIO_IT Interrupt mask; bits outside the host's set are ignored.
  * @param  NewState New state of the interrupts, ENABLE or DISABLE.
  */
void SDIO_ConfigNormIntSig(SDIOHOST_TypeDef *SDIOx, u32 SDIO_IT, u32 NewState)
{
	if (SD_HostVerify(SDIOx) != HAL_OK) {
		return;
	}

	SDIOH_INTConfig((u8)(SDIO_IT & SDIOH_SD_ISR_ALL), NewState);
}

/**
  * @brief  Read the normal interrupt status.
  * @param  SDIOx Pointer to the SD host register block.
  * @return Pending normal interrupt status bits.
  */
u32 SDIO_GetNormSts(SDIOHOST_TypeDef *SDIOx)
{
	if (SD_HostVerify(SDIOx) != HAL_OK) {
		return 0;
	}

	return SDIOH_GetISR();
}

/**
  * @brief  Clear normal interrupt status bits.
  * @param  SDIOx Pointer to the SD host register block.
  * @param  SDIO_IT Status bits to clear.
  */
void SDIO_ClearNormSts(SDIOHOST_TypeDef *SDIOx, u32 SDIO_IT)
{
	if (SD_HostVerify(SDIOx) != HAL_OK) {
		return;
	}

	SDIOH_INTClearPendingBit((u8)(SDIO_IT & SDIOH_SD_ISR_ALL));
}

/**
  * @brief  Arm the completion signalling used by SD_WaitTransDone().
  * @param  hsd Pointer to the SD handle.
  */
void SD_PreDMATrans(SD_HdlTypeDef *hsd)
{
	UNUSED(hsd);

	SDIOH_PreDMATrans();
}

/**
  * @brief  Wait for the data phase of the command last sent to complete.
  * @param  hsd Pointer to the SD handle.
  * @param  TimeOutUs Timeout value in microseconds.
  * @return HAL operation result:
  *           - HAL_OK: Transfer completed within the specified timeout.
  *           - Others: Transfer failed or timed out.
  */
u32 SD_WaitTransDone(SD_HdlTypeDef *hsd, u32 TimeOutUs)
{
	u16 err_status;
	u32 ret;

	/*
	 * No explicit stop transmission here: the multiple-block work modes of this
	 * host issue CMD12 on their own, and the response left in SD_CMD is CMD12's.
	 */
	ret = SDIOH_WaitDMADone(TimeOutUs);
	if (ret != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "transfer timeout\n");
	} else if (SDIOH_CheckTxError(&err_status) != HAL_OK) {
		RTK_LOGS(TAG, RTK_LOG_ERROR, "transfer error, status 0x%04x\n", err_status);
		hsd->ErrorCode |= SD_HostErrorCode(err_status);
		ret = HAL_ERR_UNKNOWN;
	} else if (sd_host.bounce) {
		DCache_Invalidate((u32)sd_host_buf, sizeof(sd_host_buf));
		_memcpy((void *)sd_host.user_addr, (void *)sd_host_buf, sd_host.xfer_len);
		/* Push the copy to memory; the caller invalidates this range on return. */
		DCache_CleanInvalidate(sd_host.user_addr, sd_host.xfer_len);
	}

	if (ret != HAL_OK) {
		hsd->ErrorCode |= SD_ERROR_TIMEOUT;
	}

	sd_host.bounce = 0;
	hsd->State = SD_STATE_READY;
	hsd->Context = SD_CONTEXT_NONE;

	return ret;
}

/**
  * @brief  Register semaphore functions for SD DMA transfer synchronization. If not called, polling mode is used
  *         while waiting for DMA transfer completion.
  * @param  sema_take_fn Pointer to the semaphore take function provided by the caller.
  * @param  sema_give_isr_fn Pointer to the semaphore give function provided by the caller, invoked from ISR context.
  */
void SD_SetSema(int (*sema_take_fn)(u32), int (*sema_give_isr_fn)(u32))
{
	sd_sema_take_fn = sema_take_fn;
	sd_sema_give_isr_fn = sema_give_isr_fn;
}

/**
  * @brief  SD host interrupt handler.
  * @param  pData Pointer to the SD handle (unused).
  * @return 0.
  */
u32 SD_IRQHandler(void *pData)
{
	u32 sts;

	UNUSED(pData);

	__DSB();

	sts = SDIOH_GetISR();
	if (sts != 0) {
		SDIOH_INTClearPendingBit((u8)sts);

		if ((sts & SDIOH_DMA_TRANSFER_DONE) && (sd_sema_give_isr_fn != NULL)) {
			sd_sema_give_isr_fn(SD_SEMA_MAX_DELAY);
		}
	}

	return 0;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
