/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _AMEBA_SD_HOST_WRAPPER_H_
#define _AMEBA_SD_HOST_WRAPPER_H_

#include "ameba_sdioh.h"

/** @addtogroup Ameba_Periph_Driver
  * @{
  */

/** @defgroup SDHOST_ABSTRACT SDHOST abstract layer
  * @brief Presents the AmebaGreen2 SDIO_/SD_ host contract on top of the
  *        AmebaSmart SDIOH_ primitives, so the generic SDHC driver builds
  *        unmodified against both SoCs.
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SDHOST_ABSTRACT_Exported_Types SDHOST abstract layer Exported Types
  * @{
  */

/**
  * @brief  SD host register block.
  * @note   AmebaSmart has a single SD host instance at @ref SDIOH_BASE. The
  *         SDIOH_* primitives address it implicitly; the pointer carried through
  *         this layer is validated against SDIOH_BASE and otherwise unused.
  */
typedef SDIOH_TypeDef SDIOHOST_TypeDef;

/**
  * @brief  SD locking object.
  */
typedef enum {
	SD_UNLOCKED = 0x00U,
	SD_LOCKED   = 0x01U
} HAL_LockTypeDef;

/**
  * @brief  SD state enumeration.
  */
typedef enum {
	SD_STATE_RESET                  = 0x00000000U,  /*!< SD not yet initialized or disabled  */
	SD_STATE_READY                  = 0x00000001U,  /*!< SD initialized and ready for use    */
	SD_STATE_TIMEOUT                = 0x00000002U,  /*!< SD Timeout state                    */
	SD_STATE_BUSY                   = 0x00000003U,  /*!< SD process ongoing                  */
	SD_STATE_PROGRAMMING            = 0x00000004U,  /*!< SD Programming State                */
	SD_STATE_RECEIVING              = 0x00000005U,  /*!< SD Receiving State                  */
	SD_STATE_TRANSFER               = 0x00000006U,  /*!< SD Transfer State                   */
	SD_STATE_ERROR                  = 0x0000000FU   /*!< SD is in error state                */
} SD_StateTypeDef;

/**
  * @brief  SD card information structure.
  */
typedef struct {
	u32 CardType;                     /*!< Specifies the card type                         */
	u32 CardVersion;                  /*!< Specifies the card version                      */
	u32 Class;                        /*!< Specifies the class of the card class           */
	u32 RelCardAdd;                   /*!< Specifies the Relative Card Address             */
	u32 BlockNbr;                     /*!< Specifies the Card Capacity in blocks           */
	u32 BlockSize;                    /*!< Specifies one block size in bytes               */
	u32 LogBlockNbr;                  /*!< Specifies the Card logical Capacity in blocks   */
	u32 LogBlockSize;                 /*!< Specifies logical block size in bytes           */
} SDIO_CardInfoTypeDef;

/**
  * @brief  SD handle structure.
  */
typedef struct {
	SDIOHOST_TypeDef         *Instance;   /*!< SD registers base address           */

	SDIO_CardInfoTypeDef     Card;        /*!< Card information                    */
	HAL_LockTypeDef          Lock;        /*!< SD locking object                   */
	u32                      CSD[4];      /*!< SD card specific data table         */
	u32                      CID[4];      /*!< SD card identification number table */
	u32                      SCR[2];      /*!< SD configuration register           */

	__IO SD_StateTypeDef     State;       /*!< SD card State                       */
	__IO u32                 Context;     /*!< SD transfer context                 */
	__IO u32                 ErrorCode;   /*!< SD Card Error codes                 */

	u8                       *pTxBuffPtr; /*!< Pointer to SD Tx transfer Buffer    */
	u8                       *pRxBuffPtr; /*!< Pointer to SD Rx transfer Buffer    */
	u32                      TxXferSize;  /*!< SD Tx Transfer size                 */
	u32                      RxXferSize;  /*!< SD Rx Transfer size                 */
} SD_HdlTypeDef;

/**
  * @brief  SDMMC command control structure.
  */
typedef struct {
	u32 Argument;    /*!< Specifies the SDMMC command argument which is sent to a card as
						  part of a command message. */
	u8 CmdIndex;     /*!< Specifies the SDMMC command index. It must be
						  Min_Data = 0 and Max_Data = 64 */
	u8 CmdType;      /*!< Specifies command type */
	u8 RespType;     /*!< Specifies the SDMMC response type.
						  This parameter can be a value of @ref SDHOST_ABSTRACT_Response_Type */
	u8 DataPresent;  /*!< Specifies whether data is present */
} SDIO_CmdInitTypeDef;

/**
  * @brief  SDMMC data control structure.
  */
typedef struct {
	u8 TransType; /*!< data transfer type, single/infinite/multiple/stop multiple transfer */
	u8 TransDir;
	u8 AutoCmdEn;
	u8 DmaEn;
	u16 BlockSize;
	u16 BlockCnt;
} SDIO_DataInitTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SDHOST_ABSTRACT_Exported_Constants SDHOST abstract layer Exported Constants
  * @{
  */

/** @defgroup SDHOST_ABSTRACT_Error_Code SD error codes
  * @{
  */
#define SD_ERROR_NONE                     0x00000000U
#define SD_ERROR_CMD_CRC_FAIL             0x00000001U
#define SD_ERROR_DATA_CRC_FAIL            0x00000002U
#define SD_ERROR_CMD_RSP_TIMEOUT          0x00000004U
#define SD_ERROR_DATA_TIMEOUT             0x00000008U
#define SD_ERROR_GENERAL_UNKNOWN_ERR      0x00010000U
#define SD_ERROR_UNSUPPORTED_FEATURE      0x10000000U
#define SD_ERROR_BUSY                     0x20000000U
#define SD_ERROR_DMA                      0x40000000U
#define SD_ERROR_TIMEOUT                  0x80000000U
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Transfer_Context SD transfer context
  * @{
  */
#define SD_CONTEXT_NONE                   0x00000000U /* None */
#define SD_CONTEXT_READ_SINGLE_BLOCK      0x00000001U /* Read single block operation    */
#define SD_CONTEXT_READ_MULTIPLE_BLOCK    0x00000002U /* Read multiple blocks operation */
#define SD_CONTEXT_WRITE_SINGLE_BLOCK     0x00000010U /* Write single block operation    */
#define SD_CONTEXT_WRITE_MULTIPLE_BLOCK   0x00000020U /* Write multiple blocks operation */
#define SD_CONTEXT_IT                     0x00000008U /* Process in Interrupt mode */
#define SD_CONTEXT_DMA                    0x00000080U /* Process in DMA mode       */
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Command_Type SDMMC command type
  * @{
  */
#define SDMMC_CMD_NORMAL	((u8)0x0)
#define SDMMC_CMD_SUSPEND	((u8)0x1)
#define SDMMC_CMD_RESUME	((u8)0x2)
#define SDMMC_CMD_ABORT		((u8)0x3)
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Response_Type SDMMC response type
  * @note  The numbering matches the SD specification response classes, so a
  *        generic host driver can pass its own native response type through.
  * @{
  */
#define SDMMC_RSP_NONE		((u8)0x0)
#define SDMMC_RSP_R1		((u8)0x1)
#define SDMMC_RSP_R1B		((u8)0x2)
#define SDMMC_RSP_R2		((u8)0x3)
#define SDMMC_RSP_R3		((u8)0x4)
#define SDMMC_RSP_R4		((u8)0x5)
#define SDMMC_RSP_R5		((u8)0x6)
#define SDMMC_RSP_R5B		((u8)0x7)
#define SDMMC_RSP_R6		((u8)0x8)
#define SDMMC_RSP_R7		((u8)0x9)
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Command_Index SDMMC command index
  * @note  Only the commands this layer issues by itself are listed; every other
  *        index is supplied by the caller. @ref SDHOST_Card_Command_Index holds
  *        the indices the register level cares about.
  * @{
  */
#define SDMMC_IO_SEND_OP_COND	((u8)0x5)   /*!< CMD5: Send operation condition (SDIO only). */
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Data_Present SDMMC data presence
  * @{
  */
#define SDIO_TRANS_NO_DATA		((u8)0x00U)
#define SDIO_TRANS_WITH_DATA	((u8)0x01U)
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Transfer_Type SDMMC data transfer type
  * @{
  */
#define SDIO_TRANS_SINGLE_BLK	((u8)0x00U)
#define SDIO_TRANS_INFIN_BLK	((u8)0x01U)
#define SDIO_TRANS_MULTI_BLK	((u8)0x02U)
#define SDIO_TRANS_MULTI_STOP	((u8)0x03U)
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Transfer_Dir SDMMC data transfer direction
  * @{
  */
#define SDIO_TRANS_HOST_TO_CARD ((u8)0x00U)
#define SDIO_TRANS_CARD_TO_HOST ((u8)0x01U)
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Auto_Cmd SDMMC auto command
  * @note  AmebaSmart issues CMD12 from hardware in its multiple-block work
  *        modes, so only SDIO_TRANS_AUTO_DIS is meaningful here.
  * @{
  */
#define SDIO_TRANS_AUTO_DIS			((u8)0x00U)
#define SDIO_TRANS_AUTO_CMD12_EN	((u8)0x01U)
#define SDIO_TRANS_AUTO_CMD23_EN	((u8)0x02U)
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Dma_Enable SDMMC DMA enable
  * @{
  */
#define SDIO_TRANS_DMA_DIS	((u8)0x00U)
#define SDIO_TRANS_DMA_EN	((u8)0x01U)
/**
  * @}
  */

/** @defgroup SDHOST_ABSTRACT_Dma_Mode SDMMC DMA mode
  * @note  AmebaSmart only has a single-buffer DMA engine, so only
  *        SDIO_SDMA_MODE is supported.
  * @{
  */
#define SDIO_SDMA_MODE		((u8)0x00)
/**
  * @}
  */

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup SDHOST_ABSTRACT_Exported_Functions SDHOST abstract layer Exported Functions
  * @{
  */

/* Initialization and de-initialization functions */
void SDIO_ClockSourceInit(void);
u32 SDIO_HostInit(SDIOHOST_TypeDef *SDIOx);
u32 SDIO_ResetAll(SDIOHOST_TypeDef *SDIOx);
u32 SDIO_CheckState(SDIOHOST_TypeDef *SDIOx);

/* Host configuration functions */
void SDIO_PowerState_ON(SDIOHOST_TypeDef *SDIOx);
void SDIO_PowerState_OFF(SDIOHOST_TypeDef *SDIOx);
u32 SDIO_ConfigClock(SDIOHOST_TypeDef *SDIOx, u32 ClkKHz);
u32 SDIO_ConfigBusWidth(SDIOHOST_TypeDef *SDIOx, u8 BusWidth);

/* Data transfer functions */
void SDIO_ConfigData(SDIOHOST_TypeDef *SDIOx, SDIO_DataInitTypeDef *Data);
u32 SDIO_ConfigDMA(SDIOHOST_TypeDef *SDIOx, u8 DmaMode, u32 DmaAddr);

/* Command and response functions */
void SDIO_SendCommand(SDIOHOST_TypeDef *SDIOx, SDIO_CmdInitTypeDef *Command);
u32 SDIO_WaitResp(SDIOHOST_TypeDef *SDIOx, u8 RespType, u32 TimeOutUs);
u32 SDIO_GetResponse(SDIOHOST_TypeDef *SDIOx, u8 Response);
u32 SDIO_CmdSendOpCond(SDIOHOST_TypeDef *SDIOx, u32 Ocr);

/* Interrupt functions */
void SDIO_ConfigNormIntSig(SDIOHOST_TypeDef *SDIOx, u32 SDIO_IT, u32 NewState);
u32 SDIO_GetNormSts(SDIOHOST_TypeDef *SDIOx);
void SDIO_ClearNormSts(SDIOHOST_TypeDef *SDIOx, u32 SDIO_IT);

/* Handle level functions */
void SD_PreDMATrans(SD_HdlTypeDef *hsd);
u32 SD_WaitTransDone(SD_HdlTypeDef *hsd, u32 TimeOutUs);
u32 SD_IRQHandler(void *pData);
void SD_SetSema(int (*sema_take_fn)(u32), int (*sema_give_isr_fn)(u32));

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* _AMEBA_SD_HOST_WRAPPER_H_ */
