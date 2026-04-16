/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef _RTL_CAN_H_
#define _RTL_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "utils/rtl_utils.h"
#include "rtl_can_def.h"

/*============================================================================*
 *                         Register Defines
 *============================================================================*/


/** \defgroup CAN         CAN
  * \brief
  * \{
  */
/*============================================================================*
 *                         Constants
 *============================================================================*/
/** \defgroup CAN_Exported_Constants CAN Exported Constants
  * \brief
  * \{
  */

/**
 * \defgroup    CAN_Private_Macros CAN Private Macros
 * \{
 * \ingroup     CAN_Exported_Constants
 */
#define CAN_MESSAGE_BUFFER_MAX_CNT      16              //!< The max count of message buffer.
#define CAN_MESSAGE_BUFFER_MAX_INDEX    (CAN_MESSAGE_BUFFER_MAX_CNT - 1)     //!< The max index of message buffer.
#define CAN_MESSAGE_BUFFER_DEFAULT_LEN  20              //!< The default length of message buffer.
#define CAN_MESSAGE_FIFO_START_ID       12              //!< The start index of message buffer for CAN FIFO.
#define CAN_DEFAULT_ERR_WARN_TH         96              //!< The threshold of CAN error count.
#define CAN_STAND_DATA_MAX_LEN          8               //!< The max data length of standard CAN.
#define CAN_RAM_DATA_MAX_INDEX          1               //!< The max index of RAM data.
#define CAN_STD_FRAME_ID_POS            18              //!< The pos for CAN standard frame.
#define CAN_EXT_FRAME_ID_POS            0               //!< The pos for CAN extend frame.
#define CAN_RAM_ACC_DATA_POS            11              //!< The pos for CAN RAM ACC.
#define CAN_STAND_FRAME_ID_MAX_VALUE    0x7FFUL         //!< The max id of standard frame.
#define CAN_EXTEND_FRAME_ID_MAX_VALUE   0x3FFFFUL       //!< The max id of extend frame.
#define CAN_FRAME_ID_MASK_MAX_VALUE     0x1FFFFFFFUL    //!< The mask of CAN frame ID.

/** End of CAN_Private_Macros
  * \}
  */

/**
 * \defgroup    CAN_DLC_Byte CAN DLC Byte
 * \{
 * \ingroup     CAN_Exported_Constants
 */
#define CAN_DLC_BYTES_0                         (0x0UL)     //!< Data length = 0 byte.
#define CAN_DLC_BYTES_1                         (0x1UL)     //!< Data length = 1 byte.
#define CAN_DLC_BYTES_2                         (0x2UL)     //!< Data length = 2 byte.
#define CAN_DLC_BYTES_3                         (0x3UL)     //!< Data length = 3 byte.
#define CAN_DLC_BYTES_4                         (0x4UL)     //!< Data length = 4 byte.
#define CAN_DLC_BYTES_5                         (0x5UL)     //!< Data length = 5 byte.
#define CAN_DLC_BYTES_6                         (0x6UL)     //!< Data length = 6 byte.
#define CAN_DLC_BYTES_7                         (0x7UL)     //!< Data length = 7 byte.
#define CAN_DLC_BYTES_8                         (0x8UL)     //!< Data length = 8 byte.

/** End of CAN_DLC_Byte
  * \}
  */

/**
 * \defgroup    CAN_Interrupt_Definition CAN Interrupt Definition
 * \{
 * \ingroup     CAN_Exported_Constants
 */
#define CAN_RAM_MOVE_DONE_INT                       BIT5        //!< Triggered when data move from register to CAN IP internal RAM finished.
#define CAN_BUS_OFF_INT                             BIT4        //!< Triggered when the bus is off.
#define CAN_WAKE_UP_INT                             BIT3        //!< Triggered by wake-up interrupt.
#define CAN_ERROR_INT                               BIT2        //!< Triggered by an error during transmission.
#define CAN_RX_INT                                  BIT1        //!< Triggered when data is received.
#define CAN_TX_INT                                  BIT0        //!< Triggered when sending is completed.

#define IS_CAN_INT_CONFIG(CONFIG)     (((CONFIG) == CAN_RAM_MOVE_DONE_INT)  || \
                                       ((CONFIG) == CAN_BUS_OFF_INT)        || \
                                       ((CONFIG) == CAN_WAKE_UP_INT)        || \
                                       ((CONFIG) == CAN_ERROR_INT)          || \
                                       ((CONFIG) == CAN_RX_INT)             || \
                                       ((CONFIG) == CAN_TX_INT)) //!< Check if the input parameter is valid.

/** End of CAN_Interrupt_Definition
  * \}
  */

/**
 * \defgroup    CAN_Interrupt_Flag CAN Interrupt Flag
 * \{
 * \ingroup     CAN_Exported_Constants
 */
#define CAN_RAM_MOVE_DONE_INT_FLAG                  BIT5    //!< Triggered when data move from register to CAN IP internal RAM finished.
#define CAN_BUS_OFF_INT_FLAG                        BIT4    //!< Triggered when the bus is off.
#define CAN_WAKE_UP_INT_FLAG                        BIT3    //!< Triggered by wake-up interrupt.
#define CAN_ERROR_INT_FLAG                          BIT2    //!< Triggered by an error during transmission.
#define CAN_RX_INT_FLAG                             BIT1    //!< Triggered when data is received.
#define CAN_TX_INT_FLAG                             BIT0    //!< Triggered when sending is completed.

#define IS_CAN_INT_FLAG(FLAG)         (((CONFIG) == CAN_RAM_MOVE_DONE_INT_FLAG) || \
                                       ((CONFIG) == CAN_BUS_OFF_INT_FLAG)       || \
                                       ((CONFIG) == CAN_WAKE_UP_INT_FLAG)       || \
                                       ((CONFIG) == CAN_ERROR_INT_FLAG)         || \
                                       ((CONFIG) == CAN_RX_INT_FLAG)            || \
                                       ((CONFIG) == CAN_TX_INT_FLAG)) //!< Check if the input parameter is valid.

/** End of CAN_Interrupt_Flag
  * \}
  */

/**
 * \defgroup    CAN_Error_Mask CAN Error Mask
 * \{
 * \ingroup     CAN_Exported_Constants
 */
#define CAN_ERROR_RX                                BIT9    //!< Receiving error occurred.
#define CAN_ERROR_TX                                BIT8    //!< Sending error occurred.
#define CAN_ERROR_ACK                               BIT5    //!< Ack error occurred.
#define CAN_ERROR_STUFF                             BIT4    //!< Stuff error occurred.
#define CAN_ERROR_CRC                               BIT3    //!< CRC error occurred.
#define CAN_ERROR_FORM                              BIT2    //!< Form error occurred.
#define CAN_ERROR_BIT1                              BIT1    //!< BIT1 error occurred.
#define CAN_ERROR_BIT0                              BIT0    //!< BIT0 error occurred.

#define IS_CAN_ERROR_STATUS(STATUS)   (((STATUS) == CAN_ERROR_RX)   || \
                                       ((STATUS) == CAN_ERROR_TX)   || \
                                       ((STATUS) == CAN_ERROR_ACK)  || \
                                       ((STATUS) == CAN_ERROR_CRC)  || \
                                       ((STATUS) == CAN_ERROR_FORM) || \
                                       ((STATUS) == CAN_ERROR_BIT1) || \
                                       ((STATUS) == CAN_ERROR_BIT0)) //!< Check if the input parameter is valid.

/** End of CAN_Error_Mask
  * \}
  */

/** End of CAN_Exported_Constants
  * \}
  */

/*============================================================================*
 *                         Types
 *============================================================================*/
/** \defgroup CAN_Exported_Types CAN Exported Types
  * \brief
  * \{
  */

/**
 * \brief       CAN init structure definition.
 *
 * \ingroup     CAN_Exported_Types
 */

typedef union
{
    uint32_t d32;                   //!< The word member in the union.
    uint8_t d8[4];                  //!< The four byte array members in the union.
    struct
    {
        uint32_t can_tseg1: 8;      //!< The TSEG1 of CAN.
        uint32_t can_tseg2: 8;      //!< The TSEG2 of CAN.
        uint32_t can_sjw: 3;        //!< The SJW of CAN.
        uint32_t RESERVED_0:  5;    //!< Reserved.
        uint32_t can_brp: 8;        //!< The BRP of CAN.
    } b;
} CAN_BIT_TIMING_TYPE_TypeDef;

typedef union
{
    uint32_t d32;                   //!< The word member in the union.
    uint8_t d8[4];                  //!< The four byte array members in the union.
    struct
    {
        const uint32_t can_time_stamp: 16;  //!< The time stamp of CAN.
        uint32_t can_time_stamp_div: 8;     //!< The div of time stamp.
        uint32_t RESERVED_0:  7;            //!< Reserved.
        uint32_t can_time_stamp_en: 1;      //!< The enable bit of time stamp.
    } b;
} CAN_TIME_STAMP_TYPE_TypeDef;

typedef union
{
    uint32_t d32;                   //!< The word member in the union.
    uint8_t d8[4];                  //!< The four byte array members in the union.
    struct
    {
        uint32_t can_ram_id: 29;    //!< The ID of CAN.
        uint32_t can_ram_ide: 1;    //!< The IDE of CAN.
        uint32_t can_ram_rtr: 1;    //!< The RTR of CAN.
    } b;
} CAN_RAM_ARB_TYPE_TypeDef;

typedef union
{
    uint32_t d32;                   //!< The word member in the union.
    uint8_t d8[4];                  //!< The four byte array members in the union.
    struct
    {
        uint32_t can_ram_id_mask: 29;   //!< The mask of CAN ID.
        uint32_t can_ram_ide_mask: 1;   //!< The mask of CAN IDE.
        uint32_t can_ram_rtr_mask: 1;   //!< The mask of CAN RTR.
    } b;
} CAN_RAM_MASK_TYPE_TypeDef;

typedef union
{
    uint32_t d32;                   //!< The word member in the union.
    uint8_t d8[4];                  //!< The four byte array members in the union.
    struct
    {
        uint32_t can_ram_dlc: 4;        //!< The data length.
uint32_t can_ram_lost:
        1;       //!< Value 1 indicates that data have lost in the read buffer, means more than one frame have beed received since last read.
        uint32_t can_ram_rxtx: 1;       //!< Can frame RX or TX, 0 is for RX, 1 is for TX.
        uint32_t can_ram_autoreply: 1;  //!< Set the message buffer with remote frame auto reply function.
        uint32_t RESERVED_1:  1;        //!< Reserved.
        uint32_t reset: 1;              //!< Reserved.
        uint32_t RESERVED_2: 1;         //!< Reserved.
        uint32_t can_ram_esi: 1;        //!< The ESI of CAN.
        uint32_t RESERVED_0:  5;        //!< Reserved.
        uint32_t can_ram_timestamp: 16; //!< The received message time stamp.
    } b;
} CAN_RAM_CS_TYPE_TypeDef;

typedef union
{
    uint32_t d32;                       //!< The word member in the union.
    uint8_t d8[4];                      //!< The four byte array members in the union.
    struct
    {
        uint32_t can_ram_acc_num: 8;    //!< Access message buffer number.
uint32_t can_ram_acc_mask:
        1;   //!< Access CAN_RAM_MASK field, the field will exchange with RAM spaces.
uint32_t can_ram_acc_cs:
        1;     //!< Access CAN_RAM_CS field, the field will exchange with RAM space.
uint32_t can_ram_acc_arb:
        1;    //!< Access CAN_RAM_ARB field, the field will exchange with RAM space.
uint32_t can_ram_acc_data:
        16;  //!< Access CAN_RAM_DATA field, the field will exchange with RAM space.
        uint32_t RESERVED_0:  2;        //!< Reserved.
        uint32_t can_ram_buffer_en: 1;  //!< Enable the can_ram_acc_num specified message buffer for rx/tx.
uint32_t can_ram_dir:
        1;        //!< 0 is for read from RAM to CAN_RAM_* registers, 1 is for write from CAN_RAM_* to RAM.
uint32_t can_ram_start:
        1;      //!< start to exchange data with RAM, the bit will clear automatically after the data change is finshed.
    } b;
} CAN_RAM_CMD_TYPE_TypeDef;

typedef union
{
    uint32_t d32;                        //!< The word member in the union.
    uint8_t d8[4];                       //!< The four byte array members in the union.
    struct
    {
        uint32_t can_ram_data0: 8;       //!< The data0 of CAN.
        uint32_t can_ram_data1: 8;       //!< The data1 of CAN.
        uint32_t can_ram_data2: 8;       //!< The data2 of CAN.
        uint32_t can_ram_data3: 8;       //!< The data3 of CAN.
    } b;
} CAN_RAM_DATA_TYPE_TypeDef;

/**
 * \brief       CAN init structure definition.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef struct
{
    uint8_t CAN_AutoReTxEn;                         /*!< Specify Whether the auto re-transmission function is enable.
                                                         This parameter can be a value of ENABLE or DISABLE. */
    uint8_t CAN_RxFifoEn;                           /*!< Specify Whether the RX FIFO function is enable.
                                                         This parameter can be a value of ENABLE or DISABLE. */
    uint8_t CAN_RxDmaEn;                            /*!< Specify Whether the RX DMA function is enable.
                                                         This parameter can be a value of ENABLE or DISABLE. */
    uint8_t CAN_TestModeSel;                        /*!< Specify the test mode of CAN.
                                                         This parameter can be a value of @ref CANTestModeSel_TypeDef */
    uint16_t CAN_ErrorWarnThd;                      /*!< Specify error counter warning threshold.
                                                         This parameter can be set from 0x1 to 0x1ff. */
    CAN_BIT_TIMING_TYPE_TypeDef
    CAN_BitTiming;      /*!< Specify the bit timing of CAN. @ref CAN_BIT_TIMING_TYPE_TypeDef */
    CAN_TIME_STAMP_TYPE_TypeDef
    CAN_TimeStamp;      /*!< Specify the timestamp function of CAN. @ref CAN_TIME_STAMP_TYPE_TypeDef */
} CAN_InitTypeDef;

/**
 * \brief       The state of CAN.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef enum
{
    CAN_BUS_STATE_OFF,          //!< CAN state on.
    CAN_BUS_STATE_ON,           //!< CAN state off.
} CANBusStateSel_TypeDef;

/**
 * \brief       The state of CAN RAM.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef enum
{
    CAN_RAM_STATE_IDLE,         //!< CAN RAM is idle.
    CAN_RAM_STATE_EXCHANGING,   //!< CAN RAM is exchanging.
} CANRamStateSel_TypeDef;

/**
 * \brief       The test mode of CAN.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef enum
{
    CAN_TEST_MODE_SILENCE,          //!< Silence mode.
#if CAN_SUPPORT_EXT_LOOPBACK
    CAN_TEST_MODE_EXT_LOOPBACK,     //!< Internal loopback mode.
#endif
    CAN_TEST_MODE_INT_LOOPBACK,     //!< Internal loopback mode.
    CAN_TEST_MODE_NONE,             //!< No test mode.
} CANTestModeSel_TypeDef;

/**
 * \brief       The data type of CAN frame.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef enum
{
    CAN_RTR_DATA_FRAME = 0,         //!< Data frame.
    CAN_RTR_REMOTE_FRAME = 1,       //!< Remote frame.
} CANRtrSel_TypeDef;

/**
 * \brief       The type of CAN ID.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef enum
{
    CAN_IDE_STANDARD_FORMAT = 0,    //!< Standard ID.
    CAN_IDE_EXTEND_FORMAT = 1,      //!< Extend ID.
} CANIdeSel_TypeDef;

/**
 * \brief       The type of CAN frame ID.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef enum
{
    CAN_EDL_STARDARD_FRAME = 0,     //!< Standard frame.
} CANEdlSel_TypeDef;

/**
 * \brief       The type of CAN frame.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef enum
{
    CAN_INVALID_DATA_FRAME,      //!< invalide data frame.
    CAN_STD_DATA_FRAME,          //!< standard data frame.
    CAN_EXT_DATA_FRAME,          //!< extend data frame.
    CAN_STD_REMOTE_FRAME,        //!< standard remote frame.
    CAN_EXT_REMOTE_FRAME,        //!< extend remote frame.
} CANDataFrameSel_TypeDef;

/**
 * \brief       The error of CAN.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef enum
{
    CAN_NO_ERR = 0,             //!< no error.
    CAN_MSG_ID_ERR = 1,         //!< can message id error.
    CAN_ID_ERR = 2,             //!< can frame id error.
    CAN_DATA_LEN_ERR = 3,       //!< can frame data length error.
    CAN_TYPE_ERR = 4,           //!< can frame type error.
    CAN_RAM_STATE_ERR = 5,      //!< can frame ram status error.
    CAN_TIMEOUT_ERR = 6,        //!< can timeout error.
} CANError_TypeDef;

/**
 * \brief       The members of CAN frame.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef struct
{
    uint8_t msg_buf_id;                 //!< Message buffer ID.
    uint8_t auto_reply_bit;             //!< The auto reply bit of frame.
    CANDataFrameSel_TypeDef frame_type; //!< The type of frame.
    uint16_t standard_frame_id;         //!< The standard frame ID.
    uint32_t extend_frame_id;           //!< The extend frame ID.
} CANTxFrame_TypeDef;

/**
 * \brief       The members of CAN RX frame.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef struct
{
    uint8_t msg_buf_id;            //!< Message buffer ID.
    uint8_t rx_dma_en;             //!< The DMA enable bit of RX frame.
    uint8_t frame_rtr_mask;        //!< can frame RTR mask, 1 means don't care, 0 means the bit should match.
    uint8_t frame_ide_mask;        //!< can frame IDE mask, 1 means don't care, 0 means the bit should match.
    uint32_t frame_id_mask;        //!< can frame ID mask, 1 means the ID bit in CAN_RAM_ARB don't care, 0 means the bit should match.
    uint8_t frame_rtr_bit;         //!< can frame RTR bit, determine DATA or REMOTE frame.
    uint8_t frame_ide_bit;         //!< can frame IDE bit, determine standard or extend format.
    uint8_t auto_reply_bit;        //!< The auto reply bit of RX frame.
    uint16_t standard_frame_id;    //!< Standard frame ID.
    uint32_t extend_frame_id;      //!< Extend frame ID.
    bool rx_msg_buf_enable;        //!< Message buffer enable.
} CANRxFrame_TypeDef;

/**
 * \brief       The members of message buffer.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef struct
{
    uint8_t rtr_mask;                   //!< The mask of rtr.
    uint8_t ide_mask;                   //!< The mask of ide.
    uint8_t id_mask;                    //!< The mask of id.
    uint8_t esi_bit;                    //!< The mask of esi.
    uint8_t auto_reply_bit;             //!< The auto reply bit.
    uint8_t rxtx_bit;                   //!< The rxtx bit.
    uint8_t rx_lost_bit;                //!< The RX lost bit.
    uint8_t data_length;                //!< The data length.
    uint8_t rx_dma_en;                  //!< The DMA enable bit.
    CANRtrSel_TypeDef rtr_bit;          //!< The rtr bit \ref CANRtrSel_TypeDef.
    CANIdeSel_TypeDef ide_bit;          //!< The ide bit \ref CANIdeSel_TypeDef.
    uint16_t rx_timestamp;              //!< The time stamp.
    uint16_t standard_frame_id;         //!< Standard frame ID.
    uint32_t extend_frame_id;           //!< Extend frame ID.
} CANMsgBufInfo_TypeDef;

/**
 * \brief       The status of CAN FIFO.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef struct
{
    uint8_t fifo_msg_lvl;               //!< Indicates the number of message in FIFO.
    uint8_t fifo_msg_overflow;          //!< Indicates whether the FIFO is overflow.
    uint8_t fifo_msg_empty;             //!< Indicates whether the FIFO is empty.
    uint8_t fifo_msg_full;              //!< Indicates whether the FIFO is full.
} CANFifoStatus_TypeDef;

/**
 * \brief       The members of RX DMA data.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef struct
{
    CAN_RAM_ARB_TYPE_TypeDef can_ram_arb;           //!< ARB
    CAN_RAM_CS_TYPE_TypeDef can_ram_cs;             //!< CS
    uint8_t rx_dma_data[CAN_STAND_DATA_MAX_LEN];    //!< Data
} CANRxDmaData_TypeDef;

/**
 * \brief       The members of TX DMA data.
 *
 * \ingroup     CAN_Exported_Types
 */
typedef struct
{
    __IO uint32_t CAN_RAM_DATA[2];      //!< The data of CAN frame.

    union
    {
        __IO uint32_t CAN_RAM_ARB;                          //!< The word member in the union.
        __IO CAN_RAM_ARB_TYPE_TypeDef
        CAN_RAM_ARB_BITS;     //!< The ARB of CAN. \ref CAN_RAM_ARB_TYPE_TypeDef
    } u_CAN_RAM_ARB;

    union
    {
        __IO uint32_t CAN_RAM_MASK;                         //!< The word member in the union.
        __IO CAN_RAM_MASK_TYPE_TypeDef
        CAN_RAM_MASK_BITS;   //!< The mask of CAN. \ref CAN_RAM_MASK_TYPE_TypeDef
    } u_CAN_RAM_MASK;

    union
    {
        __IO uint32_t CAN_RAM_CS;                           //!< The word member in the union.
        __IO CAN_RAM_CS_TYPE_TypeDef CAN_RAM_CS_BITS;       //!< The CS of CAN. \ref CAN_RAM_CS_TYPE_TypeDef
    } u_CAN_RAM_CS;

    union
    {
        __IO uint32_t CAN_RAM_CMD;                          //!< The word member in the union.
        __IO CAN_RAM_CMD_TYPE_TypeDef
        CAN_RAM_CMD_BITS;     //!< The CMD of CAN. \ref CAN_RAM_CMD_TYPE_TypeDef
    } u_CAN_RAM_CMD;
} CAN_RAM_TypeDef;

/** End of CAN_Exported_Types
  * \}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** \defgroup CAN_Exported_Functions CAN Exported Functions
  * \brief
  * \{
  */

/**
 * \brief   Deinitialize the CAN peripheral registers to their default values.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void can_driver_init(void)
 * {
 *     CAN_DeInit();
 * }
 * \endcode
 */
void CAN_DeInit();

/**
 * \brief   Initialize the CAN peripheral according to the specified
 *          parameters in the CAN_InitStruct
 *
 * \param[in] CAN_InitStruct  Pointer to a CAN_InitTypeDef structure which will be initialized.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void can_driver_init(void)
 * {
 *     RCC_PeriphClockCmd(APBPeriph_CAN0, APBPeriph_CAN0_CLOCK, ENABLE);
 *
 *     CAN_InitTypeDef init_struct;
 *
 *     CAN_StructInit(&init_struct);
 *     init_struct.CAN_AutoReTxEn = DISABLE;
 *     init_struct.CAN_BitTiming.can_brp = 7;
 *     init_struct.CAN_BitTiming.can_sjw = 3;
 *     init_struct.CAN_BitTiming.can_tseg1 = 4;
 *     init_struct.CAN_BitTiming.can_tseg2 = 3;
 *
 *     CAN_Init(&init_struct);
 *     CAN_Cmd(ENABLE);
 * }
 * \endcode
 */
void CAN_Init(CAN_InitTypeDef *CAN_InitStruct);

/**
 * \brief   Fill each CAN_InitStruct member with its default value.
 *
 * \note   The default settings for the CAN_InitStruct member are shown in the following table:
 *         | CAN_InitStruct member              | Default value                |
 *         |:----------------------------------:|:----------------------------:|
 *         | CAN_TriSampleEn                    | DISABLE                      |
 *         | CAN_AutoReTxEn                     | ENABLE                       |
 *         | CAN_RxFifoEn                       | DISABLE                      |
 *         | CAN_RxDmaEn                        | DISABLE                      |
 *         | CAN_TimeStamp.b.can_time_stamp_en  | DISABLE                      |
 *         | CAN_TimeStamp.b.can_time_stamp_div | 0                            |
 *         | CAN_ErrorWarnThd                   | \ref CAN_DEFAULT_ERR_WARN_TH |
 *         | CAN_BitTiming.b.can_brp            | 7                            |
 *         | CAN_BitTiming.b.can_sjw            | 3                            |
 *         | CAN_BitTiming.b.can_tseg2          | 3                            |
 *         | CAN_BitTiming.b.can_tseg1          | 4                            |
 *         | CAN_TestModeSel                    | \ref CAN_TEST_MODE_NONE      |
 *
 * \param[in] CAN_InitStruct  Pointer to an CAN_InitTypeDef structure which will be initialized.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void can_driver_init(void)
 * {
 *     RCC_PeriphClockCmd(APBPeriph_CAN, APBPeriph_CAN_CLOCK, ENABLE);
 *
 *     CAN_InitTypeDef init_struct;
 *
 *     CAN_StructInit(&init_struct);
 *     init_struct.CAN_AutoReTxEn = DISABLE;
 *     init_struct.CAN_BitTiming.can_brp = 7;
 *     init_struct.CAN_BitTiming.can_sjw = 3;
 *     init_struct.CAN_BitTiming.can_tseg1 = 4;
 *     init_struct.CAN_BitTiming.can_tseg2 = 3;
 *
 *     CAN_Init(&init_struct);
 *     CAN_Cmd(ENABLE);
 * }
 * \endcode
 */
void CAN_StructInit(CAN_InitTypeDef *CAN_InitStruct);

/**
 * \brief   Enable or disable the selected CAN mode.
 *
 * \param[in] NewState  New state of the operation mode.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the CAN.
 *                      - DISABLE: Disable the CAN.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void can_driver_init(void)
 * {
 *     RCC_PeriphClockCmd(APBPeriph_CAN0, APBPeriph_CAN0_CLOCK, ENABLE);
 *
 *     CAN_InitTypeDef init_struct;
 *
 *     CAN_StructInit(&init_struct);
 *     init_struct.CAN_AutoReTxEn = DISABLE;
 *     init_struct.CAN_BitTiming.can_brp = 7;
 *     init_struct.CAN_BitTiming.can_sjw = 3;
 *     init_struct.CAN_BitTiming.can_tseg1 = 4;
 *     init_struct.CAN_BitTiming.can_tseg2 = 3;
 *
 *     CAN_Init(&init_struct);
 *     CAN_Cmd(ENABLE);
 * }
 * \endcode
 */
void CAN_Cmd(FunctionalState NewState);

/**
 * \brief   Enable or disable the specified CAN interrupt source.
 *
 * \param[in] CAN_INT   Specify the CAN interrupt source to be enable or disable.
 *                      This parameter can be the following values:
 *                      \arg CAN_RAM_MOVE_DONE_INT: data move from register to CAN IP internal RAM finished interupt.
 *                      \arg CAN_BUS_OFF_INT: bus off interrupt.
 *                      \arg CAN_WAKE_UP_INT: wakeup interrupt.
 *                      \arg CAN_ERROR_INT: error interrupt.
 *                      \arg CAN_RX_INT: RX interrupt.
 *                      \arg CAN_TX_INT: TX interrupt.
 * \param[in] NewState  New state of the specified CAN interrupt.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the specified CAN interrupt.
 *                      - DISABLE: Disable the specified CAN interrupt.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void can_driver_init(void)
 * {
 *     CAN_INTConfig((CAN_RAM_MOVE_DONE_INT | CAN_BUS_OFF_INT | CAN_WAKE_UP_INT |
 *                    CAN_ERROR_INT | CAN_RX_INT | CAN_TX_INT), ENABLE);
 * }
 * \endcode
 */
void CAN_INTConfig(uint32_t CAN_INT, FunctionalState newState);

#if (CAN_SUPPORT_INT_MSK_STS == 1)
/**
 * \brief   Mask or unmask the specified CAN interrupt source.
 *
 * \param[in] CAN_INT   Specify the CAN interrupt source to be enable or disable.
 *                      This parameter can be the following values:
 *                      \arg CAN_RAM_MOVE_DONE_INT: data move from register to CAN IP internal RAM finished interupt.
 *                      \arg CAN_BUS_OFF_INT: bus off interrupt.
 *                      \arg CAN_WAKE_UP_INT: wakeup interrupt.
 *                      \arg CAN_ERROR_INT: error interrupt.
 *                      \arg CAN_RX_INT: RX interrupt.
 *                      \arg CAN_TX_INT: TX interrupt.
 * \param[in] NewState  New state of the specified CAN interrupt.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Mask the specified CAN interrupt.
 *                      - DISABLE: Unmask the specified CAN interrupt.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void can_driver_init(void)
 * {
 *     CAN_MaskINTConfig((CAN_RAM_MOVE_DONE_INT | CAN_BUS_OFF_INT | CAN_WAKE_UP_INT |
 *                      CAN_ERROR_INT | CAN_RX_INT | CAN_TX_INT), ENABLE);
 * }
 * \endcode
 */

void CAN_MaskINTConfig(uint32_t CAN_INT_FLAG, FunctionalState NewState);
#endif

/**
 * \brief   Get the specified CAN interrupt raw status.
 *
 * \param[in] CAN_INT  The specified CAN interrupt.
 *                     This parameter can be one of the following values:
 *                     \arg CAN_RAM_MOVE_DONE_INT_FLAG: ram move done interrupt flag.
 *                     \arg CAN_BUS_OFF_INT_FLAG: bus off interrupt flag.
 *                     \arg CAN_WAKE_UP_INT_FLAG: wakeup interrupt flag.
 *                     \arg CAN_ERROR_INT_FLAG: error interrupt flag.
 *                     \arg CAN_RX_INT_FLAG: RX interrupt flag.
 *                     \arg CAN_TX_INT_FLAG: Clear TX interrupt flag.
 *
 * \return  The new state of CAN_INT.
 *         - SET: The interrupt raw status is set.
 *         - RESET: The interrupt raw status is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void CAN_Handler(void)
 * {
 *     if (SET == CAN_GetINTRawStatus(CAN_RAM_MOVE_DONE_INT_FLAG))
 *  {
 *      DBG_DIRECT("[CAN] CAN_Handler CAN_RAM_MOVE_DONE_INT_FLAG");
 *      CAN_ClearINTPendingBit(CAN_RAM_MOVE_DONE_INT_FLAG);
 *  }
 * }
 * \endcode
 */
ITStatus CAN_GetINTRawStatus(uint32_t CAN_INT_FLAG);

/**
 * \brief   Get the specified CAN interrupt status.
 *
 * \param[in] CAN_INT  The specified CAN interrupt.
 *                     This parameter can be one of the following values:
 *                     \arg CAN_RAM_MOVE_DONE_INT_FLAG: ram move done interrupt flag.
 *                     \arg CAN_BUS_OFF_INT_FLAG: bus off interrupt flag.
 *                     \arg CAN_WAKE_UP_INT_FLAG: wakeup interrupt flag.
 *                     \arg CAN_ERROR_INT_FLAG: error interrupt flag.
 *                     \arg CAN_RX_INT_FLAG: RX interrupt flag.
 *                     \arg CAN_TX_INT_FLAG: Clear TX interrupt flag.
 *
 * \return The new state of CAN_INT.
 *         - SET: The interrupt is set.
 *         - RESET: The interrupt is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void CAN_Handler(void)
 * {
 *     if (SET == CAN_GetINTStatus(CAN_RAM_MOVE_DONE_INT_FLAG))
 *  {
 *      DBG_DIRECT("[CAN] CAN_Handler CAN_RAM_MOVE_DONE_INT_FLAG");
 *      CAN_ClearINTPendingBit(CAN_RAM_MOVE_DONE_INT_FLAG);
 *  }
 * }
 * \endcode
 */
ITStatus CAN_GetINTStatus(uint32_t CAN_INT_FLAG);

/**
 * \brief  Clear the CAN interrupt pending bit.
 *
 * \param[in] CAN_INT_FLAG  Specify the interrupt pending bit to clear.
 *                          This parameter can be any combination of the following values:
 *                          - CAN_RAM_MOVE_DONE_INT_FLAG: ram move done interrupt flag.
 *                          - CAN_BUS_OFF_INT_FLAG: bus off interrupt flag.
 *                          - CAN_WAKE_UP_INT_FLAG: wakeup interrupt flag.
 *                          - CAN_ERROR_INT_FLAG: error interrupt flag.
 *                          - CAN_RX_INT_FLAG: RX interrupt flag.
 *                          - CAN_TX_INT_FLAG: Clear TX interrupt flag.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void CAN_Handler(void)
 * {
 *     if (SET == CAN_GetINTStatus(CAN_RAM_MOVE_DONE_INT_FLAG))
 *     {
 *      DBG_DIRECT("[CAN] CAN_Handler CAN_RAM_MOVE_DONE_INT_FLAG");
 *      CAN_ClearINTPendingBit(CAN_RAM_MOVE_DONE_INT_FLAG);
 *     }
 * }
 * \endcode
 */
void CAN_ClearINTPendingBit(uint32_t CAN_INT_FLAG);

/**
 * \brief  Get the specified CAN error status.
 *
 * \param[in]  CAN_ERR_STAT  The specified CAN error.
 *                           This parameter can be one of the following values:
 *                           \arg CAN_ERROR_RX: can rx error flag
 *                           \arg CAN_ERROR_TX: can tx error flag
 *                           \arg CAN_ERROR_ACK: latest error is ack error
 *                           \arg CAN_ERROR_STUFF: latest error is stuff error
 *                           \arg CAN_ERROR_CRC: latest error is crc error
 *                           \arg CAN_ERROR_FORM: latest error is form error
 *                           \arg CAN_ERROR_BIT1: latest error is bit1 error, tx=1 but rx=0
 *                           \arg CAN_ERROR_BIT0: latest error is bit0 error, tx=0 but rx=1
 *
 * \return The new state of CAN_INT.
 *         - SET: The error state is set.
 *         - RESET: The error state is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void CAN_Handler(void)
 * {
 *     if (SET == CAN_GetErrorStatus(CAN_ERROR_TX))
 *     {
 *          DBG_DIRECT("[CAN] CAN_ERROR_TX");
 *          CAN_CLearErrorStatus(CAN_ERROR_TX);
 *     }
 * }
 * \endcode
 */
FlagStatus CAN_GetErrorStatus(uint32_t CAN_ERR_STAT);

/**
 * \brief  Clear the specified CAN error status.
 *
 * \param[in]  CAN_ERR_STAT  The specified CAN error.
 *                           This parameter can be one of the following values:
 *                           - CAN_ERROR_RX: can rx error flag
 *                           - CAN_ERROR_TX: can tx error flag
 *                           - CAN_ERROR_ACK: latest error is ack error
 *                           - CAN_ERROR_STUFF: latest error is stuff error
 *                           - CAN_ERROR_CRC: latest error is crc error
 *                           - CAN_ERROR_FORM: latest error is form error
 *                           - CAN_ERROR_BIT1: latest error is bit1 error, tx=1 but rx=0
 *                           - CAN_ERROR_BIT0: latest error is bit0 error, tx=0 but rx=1
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void CAN_Handler(void)
 * {
 *     if (SET == CAN_GetErrorStatus(CAN_ERROR_TX))
 *     {
 *          DBG_DIRECT("[CAN] CAN_ERROR_TX");
 *          CAN_CLearErrorStatus(CAN_ERROR_TX);
 *     }
 * }
 * \endcode
 */
void CAN_CLearErrorStatus(uint32_t CAN_ERR_STAT);

/**
 * \brief  Set the CAN message buffer TX mode.
 *
 * \param[in]  p_tx_frame_params  The CAN frame parameter. \ref CANTxFrame_TypeDef
 * \param[in]  p_frame_data       The specified CAN data.
 * \param[in]  data_len           The length of CAN data to be sent. The range is 0 to 8.
 *
 * \return The state of set buffer \ref CANError_TypeDef.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_basic_tx(uint32_t buf_id, CANDataFrameSel_TypeDef frame_type, \
 *                        uint16_t frame_id, uint32_t ext_id, uint8_t *tx_data, uint8_t data_len)
 * {
 *
 *   CANError_TypeDef tx_error;
 *
 *   CANTxFrame_TypeDef tx_frame_type;
 *
 *   tx_frame_type.msg_buf_id = buf_id;
 *   tx_frame_type.frame_type = frame_type;
 *   tx_frame_type.standard_frame_id = frame_id;
 *   tx_frame_type.auto_reply_bit = DISABLE;
 *   tx_frame_type.extend_frame_id = 0;
 *
 *   switch (frame_type)
 *   {
 *   case CAN_EXT_DATA_FRAME:
 *   case CAN_EXT_REMOTE_FRAME:
 *       tx_frame_type.extend_frame_id = ext_id;
 *   case CAN_STD_DATA_FRAME:
 *   case CAN_STD_REMOTE_FRAME:
 *       break;
 *   }
 *
 *   CAN_MBTxINTConfig(tx_frame_type.msg_buf_id, ENABLE);
 *   tx_error = CAN_SetMsgBufTxMode(&tx_frame_type, tx_data, data_len);
 *
 *   while (CAN_GetRamState() != CAN_RAM_STATE_IDLE);
 *
 *   if (tx_error != CAN_NO_ERR)
 *   {
 *       IO_PRINT_INFO1("can_basic_tx: TX error %d", tx_error);
 *   }
 *  }
 * \endcode
 */
CANError_TypeDef CAN_SetMsgBufTxMode(CANTxFrame_TypeDef *p_tx_frame_params,
                                     const uint8_t *p_frame_data,
                                     uint8_t data_len);

/**
 * \brief  Set the CAN message buffer RX mode.
 *
 * \param[in]  p_rx_frame_params  The CAN frame parameter. \ref CANRxFrame_TypeDef
 *
 * \return The state of set buffer \ref CANError_TypeDef.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_basic_rx(void)
 * {
 *     CANError_TypeDef rx_error;
 *     CANRxFrame_TypeDef rx_frame_type;
 *     rx_frame_type.msg_buf_id = 7;
 *
 *     rx_frame_type.extend_frame_id = 0;
 *     rx_frame_type.standard_frame_id = 0;
 *     rx_frame_type.frame_rtr_mask = SET;
 *     rx_frame_type.frame_ide_mask = SET;
 *     rx_frame_type.frame_id_mask = CAN_FRAME_ID_MASK_MAX_VALUE;
 *     rx_frame_type.rx_dma_en = RESET;
 *     rx_frame_type.auto_reply_bit = RESET;
 *     rx_error = CAN_SetMsgBufRxMode(&rx_frame_type);
 *
 *     CAN_MBRxINTConfig(rx_frame_type.msg_buf_id, ENABLE);
 *
 *     while (CAN_GetRamState() != CAN_RAM_STATE_IDLE);
 *
 *     if (rx_error != CAN_NO_ERR)
 *     {
 *         IO_PRINT_INFO1("can_basic_rx: RX error %d", rx_error);
 *     }
 *
 *     IO_PRINT_INFO0("can_basic_rx: waiting for RX...");
 * }
 * \endcode
 */
CANError_TypeDef CAN_SetMsgBufRxMode(CANRxFrame_TypeDef *p_rx_frame_params);

/**
 * \brief  Get message buffer information.
 *
 * \param[in]  msg_buf_id  Message buffer id.
 * \param[out] p_mb_info   Message buffer information. \ref CANMsgBufInfo_TypeDef
 *
 * \return The state of get buffer information \ref CANError_TypeDef.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_msg_info(void)
 * {
 *     uint8_t index = 0
 *     uint8_t rx_data[64];
 *     CANMsgBufInfo_TypeDef mb_info;
 *     CAN_GetMsgBufInfo(index, &mb_info);
 *     CAN_GetRamData(mb_info.data_length, rx_data);
 * }
 * \endcode
 */
CANError_TypeDef CAN_GetMsgBufInfo(uint8_t msg_buf_id,
                                   CANMsgBufInfo_TypeDef *p_mb_info);

/**
 * \brief  Get the RAM data of message buffer.
 *
 * \param[in]  data_len  The length of RAM data.
 * \param[out] p_data    Data buffer to be received.
 *
 * \return The state of get RAM data \ref CANError_TypeDef.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_ram_data(void)
 * {
 *     uint8_t index = 0
 *     uint8_t rx_data[8];
 *     CANMsgBufInfo_TypeDef mb_info;
 *     CAN_GetMsgBufInfo(index, &mb_info);
 *     CAN_GetRamData(mb_info.data_length, rx_data);
 * }
 * \endcode
 */
CANError_TypeDef CAN_GetRamData(uint8_t data_len, uint8_t *p_data);

/**
 * \brief  Check the type of frame.
 *
 * \param[in]  rtr_bit  Refer to \ref CANRtrSel_TypeDef.
 * \param[in]  ide_bit  Refer to \ref CANIdeSel_TypeDef.
 *
 * \return The type of frame \ref CANDataFrameSel_TypeDef.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_ram_data(void)
 * {
 *     uint8_t index = 0
 *     uint8_t rx_data[8];
 *     CANMsgBufInfo_TypeDef mb_info;
 *     CAN_GetMsgBufInfo(index, &mb_info);
 *     CANDataFrameSel_TypeDef frame_type = CAN_CheckFrameType(mb_info.rtr_bit, mb_info.ide_bit);
 * }
 * \endcode
 */
CANDataFrameSel_TypeDef CAN_CheckFrameType(uint8_t rtr_bit, uint8_t ide_bit);

/**
 * \brief  Config message buffer TX interrupt.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 * \param[in]  newState              New state of the TX interrupt of message buffer.
 *                                   This parameter can be one of the following values:
 *                                   - ENABLE: Enable the TX interrupt of message buffer.
 *                                   - DISABLE: Disable the TX interrupt of message buffer.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_basic_tx(uint32_t buf_id, CANDataFrameSel_TypeDef frame_type, \
 *                        uint16_t frame_id, uint32_t ext_id, uint8_t *tx_data, uint8_t data_len)
 * {
 *
 *   CANError_TypeDef tx_error;
 *
 *   CANTxFrame_TypeDef tx_frame_type;
 *
 *   tx_frame_type.msg_buf_id = buf_id;
 *   tx_frame_type.frame_type = frame_type;
 *   tx_frame_type.standard_frame_id = frame_id;
 *   tx_frame_type.auto_reply_bit = DISABLE;
 *   tx_frame_type.extend_frame_id = 0;
 *
 *   switch (frame_type)
 *   {
 *   case CAN_EXT_DATA_FRAME:
 *   case CAN_EXT_REMOTE_FRAME:
 *       tx_frame_type.extend_frame_id = ext_id;
 *   case CAN_STD_DATA_FRAME:
 *   case CAN_STD_REMOTE_FRAME:
 *       break;
 *   }
 *
 *   CAN_MBTxINTConfig(tx_frame_type.msg_buf_id, ENABLE);
 *   tx_error = CAN_SetMsgBufTxMode(&tx_frame_type, tx_data, data_len);
 *
 *   while (CAN_GetRamState() != CAN_RAM_STATE_IDLE);
 *
 *   if (tx_error != CAN_NO_ERR)
 *   {
 *       IO_PRINT_INFO1("can_basic_tx: TX error %d", tx_error);
 *   }
 *  }
 * \endcode
 */
void CAN_MBTxINTConfig(uint8_t message_buffer_index, FunctionalState newState);

/**
 * \brief  Config message buffer RX interrupt.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 * \param[in]  newState              New state of the RX interrupt of message buffer.
 *                                   This parameter can be one of the following values:
 *                                   - ENABLE: Enable the RX interrupt of message buffer.
 *                                   - DISABLE: Disable the RX interrupt of message buffer.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_basic_rx(void)
 * {
 *     CANError_TypeDef rx_error;
 *     CANRxFrame_TypeDef rx_frame_type;
 *     rx_frame_type.msg_buf_id = 7;
 *
 *     rx_frame_type.extend_frame_id = 0;
 *     rx_frame_type.standard_frame_id = 0;
 *     rx_frame_type.frame_rtr_mask = SET;
 *     rx_frame_type.frame_ide_mask = SET;
 *     rx_frame_type.frame_id_mask = CAN_FRAME_ID_MASK_MAX_VALUE;
 *     rx_frame_type.rx_dma_en = RESET;
 *     rx_frame_type.auto_reply_bit = RESET;
 *     rx_error = CAN_SetMsgBufRxMode(&rx_frame_type);
 *
 *     CAN_MBRxINTConfig(rx_frame_type.msg_buf_id, ENABLE);
 *
 *     while (CAN_GetRamState() != CAN_RAM_STATE_IDLE);
 *
 *     if (rx_error != CAN_NO_ERR)
 *     {
 *         IO_PRINT_INFO1("can_basic_rx: RX error %d", rx_error);
 *     }
 *
 *     IO_PRINT_INFO0("can_basic_rx: waiting for RX...");
 * }
 * \endcode
 */
void CAN_MBRxINTConfig(uint8_t message_buffer_index, FunctionalState newState);

/**
 * \brief  Get CAN FIFO status.
 *
 * \param[in] CAN_FifoStatus  The status of CAN FIFO. \ref CANFifoStatus_TypeDef
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_fifo_status(void)
 * {
 *     CANFifoStatus_TypeDef fifo_status;
 *     CAN_GetFifoStatus(&fifo_status);
 * }
 * \endcode
 */
void CAN_GetFifoStatus(CANFifoStatus_TypeDef *CAN_FifoStatus);

/**
 * \brief  Set TX message trigger by timestamp timer.
 *
 * \param[in]  newState                 New state of the TX trigger function.
 *                                      This parameter can be one of the following values:
 *                                      - ENABLE: Enable the TX trigger function.
 *                                      - DISABLE: Disable the TX trigger function.
 * \param[in]  trigger_timestamp_begin  End of trigger time.
 * \param[in]  close_offset             Start of trigger time.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_tx_trigger(void)
 * {
 *     uint16_t begin_ts = CAN_GetTimeStampCount();
 *     begin_ts += 1000;
 *     CAN_TxTriggerConfig(ENABLE, begin_ts, 100);
 * }
 * \endcode
 */
void CAN_TxTriggerConfig(FunctionalState newState,
                         uint16_t trigger_timestamp_begin,
                         uint16_t close_offset);

/**
 * \brief  Get can bus state.
 *
 * \return The state of bus.
 *         - CAN_BUS_STATE_ON: The bus is on.
 *         - CAN_BUS_STATE_OFF: The bus is off.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_bus_state(void)
 * {
 *     while (CAN_GetBusState() != CAN_BUS_STATE_ON);
 * }
 * \endcode
 */
uint32_t CAN_GetBusState();

/**
 * \brief  Get message buffer RAM state.
 *
 * \return The state of message buffer RAM.
 *         - CAN_RAM_STATE_EXCHANGING: CAN RAM is exchanging.
 *         - CAN_RAM_STATE_IDLE: CAN RAM is idle.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_ram_state(void)
 * {
 *     while (CAN_GetRamState() != CAN_RAM_STATE_IDLE);
 * }
 * \endcode
 */
uint32_t CAN_GetRamState();

/**
 * \brief  Get message buffer TX done flag.
 *
 * \param[in] message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * \return The flag of message buffer TX done.
 *         - SET: The TX done of message buffer is set.
 *         - RESET: The TX done of message buffer is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_tx_done_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnTxDoneFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_trx_handler: MB_%d TX done", index);
 *              CAN_ClearMBnTxDoneFlag(index);
 *          }
 *      }
 * }
 * \endcode
 */
FlagStatus CAN_GetMBnTxDoneFlag(uint8_t message_buffer_index);

/**
 * \brief  Clear message buffer TX done flag.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_tx_done_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnTxDoneFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_trx_handler: MB_%d TX done", index);
 *              CAN_ClearMBnTxDoneFlag(index);
 *          }
 *      }
 * }
 * \endcode
 */
void CAN_ClearMBnTxDoneFlag(uint8_t message_buffer_index);

/**
 * \brief  Get message buffer TX error flag.
 *
 * \param[in] message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * \return The flag of message buffer TX error.
 *         - SET: The TX error of message buffer is set.
 *         - RESET: The TX error of message buffer is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_tx_error_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnTxErrorFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_trx_handler: CAN ERROR TX MB_%d", index);
 *              CAN_ClearMBnTxErrorFlag(index);
 *          }
 *      }
 * }
 * \endcode
 */
FlagStatus CAN_GetMBnTxErrorFlag(uint8_t message_buffer_index);

/**
 * \brief  Clear message buffer TX error flag.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_tx_error_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnTxErrorFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_trx_handler: CAN ERROR TX MB_%d", index);
 *              CAN_ClearMBnTxErrorFlag(index);
 *          }
 *      }
 * }
 * \endcode
 */
void CAN_ClearMBnTxErrorFlag(uint8_t message_buffer_index);

/**
 * \brief  Get message buffer TX finish flag, it indicates the TX message in the message buffer finish sending.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * \return The flag of message buffer TX finish.
 *         - SET: The TX finish of message buffer is set.
 *         - RESET: The TX finish of message buffer is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_tx_finish_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnStatusTxFinishFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_get_tx_finish_flag: TX finish MB_%d", index);
 *          }
 *      }
 * }
 * \endcode
 */
FlagStatus CAN_GetMBnStatusTxFinishFlag(uint8_t message_buffer_index);

/**
 * \brief  Get message buffer TX request flag, it indicates the TX message in buffer is pending for transmit.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * \return The flag of message buffer TX request.
 *         - SET: The TX request of message buffer is set.
 *         - RESET: The TX request of message buffer is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_tx_req_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnStatusTxReqFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_get_tx_req_flag: TX pending MB_%d", index);
 *          }
 *      }
 * }
 * \endcode
 */
FlagStatus CAN_GetMBnStatusTxReqFlag(uint8_t message_buffer_index);

/**
 * \brief  Get message buffer RX done flag.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * \return The flag of message buffer RX done.
 *         - SET: The RX done of message buffer is set.
 *         - RESET: The RX done of message buffer is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_rx_done_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnRxDoneFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_trx_handler: MB_%d RX done", index);
 *              CAN_ClearMBnRxDoneFlag(index);
 *          }
 *      }
 * }
 * \endcode
 */
FlagStatus CAN_GetMBnRxDoneFlag(uint8_t message_buffer_index);

/**
 * \brief  Clear message buffer RX done flag.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_clear_rx_done_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnRxDoneFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_trx_handler: MB_%d RX done", index);
 *              CAN_ClearMBnRxDoneFlag(index);
 *          }
 *      }
 * }
 * \endcode
 */
void CAN_ClearMBnRxDoneFlag(uint8_t message_buffer_index);

/**
 * \brief  Get message buffer RX valid flag, it indicates new message has been received in the message buffer.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * \return The flag of message buffer RX valid.
 *         - SET: The RX valid of message buffer is set.
 *         - RESET: The RX valid of message buffer is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_rx_valid_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnStatusRxValidFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_get_rx_valid_flag: MB_%d RX valid", index);
 *          }
 *      }
 * }
 * \endcode
 */
FlagStatus CAN_GetMBnStatusRxValidFlag(uint8_t message_buffer_index);

/**
 * \brief  Get message buffer RX ready flag, it indicates the message buffer is ready for receiving a new message.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * \return The flag of message buffer RX ready.
 *         - SET: The RX ready of message buffer is set.
 *         - RESET: The RX ready of message buffer is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_rx_ready_flag(void)
 * {
 *     for (uint8_t index = 0; index < CAN_MESSAGE_BUFFER_MAX_CNT; index++)
 *     {
 *          if (SET == CAN_GetMBnStatusRxReadyFlag(index))
 *          {
 *              IO_PRINT_INFO1("can_get_rx_ready_flag: MB_%d RX ready", index);
 *          }
 *      }
 * }
 * \endcode
 */
FlagStatus CAN_GetMBnStatusRxReadyFlag(uint8_t message_buffer_index);

/**
 * \brief  Enable or disable can time stamp.
 *
 * \param[in]  newState  New state of the time stamp.
 *                       This parameter can be one of the following values:
 *                       - ENABLE: Enable the time stamp.
 *                       - DISABLE: Disable the time stamp.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_enable_timestamp(void)
 * {
 *     CAN_TimeStampConfig(ENABLE);
 * }
 * \endcode
 */
void CAN_TimeStampConfig(FunctionalState newState);

/**
 * \brief  Get time stamp count.
 *
 * \return Time stamp count.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_timestamp(void)
 * {
 *     uint16_t begin_ts = CAN_GetTimeStampCount();
 * }
 * \endcode
 */
uint16_t CAN_GetTimeStampCount();

/**
 * \brief  Get RX DMA block size(word).
 *
 * \return RX DMA block size.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_dma_msize(void)
 * {
 *     uint32_t dma_buffer_size = CAN_GetRxDmaMsize();
 * }
 * \endcode
 */
uint32_t CAN_GetRxDmaMsize();

/**
 * \brief  Get message buffer RX DMA enable flag.
 *
 * \param[in]  message_buffer_index  CAN message buffer index. The range is 0 to 15.
 *
 * \return The flag of message buffer RX DMA enable.
 *         - SET: The RX dma enable of message buffer is set.
 *         - RESET: The RX dma enable of message buffer is unset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_get_rx_dma_enable_flag(void)
 * {
 *     FlagStatus dma_en_flag = CAN_GetMBnRxDmaEnFlag(index);
 * }
 * \endcode
 */
FlagStatus CAN_GetMBnRxDmaEnFlag(uint8_t message_buffer_index);

/**
 * \brief  Set message buffer RX DMA enable.
 *
 * \param[in] message_buffer_index  CAN message buffer index. The range is 0 to 15.
 * \param[in] newState              New state of the message buffer RX DMA enable.
 *                                  This parameter can be one of the following values:
 *                                  - ENABLE: Enable the RX DMA of message buffer.
 *                                  - DISABLE: Disable the RX DMA of message buffer.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_set_rx_dma_enable(void)
 * {
 *     CAN_SetMBnRxDmaEnFlag(RX_DMA_BUF_ID, ENABLE);
 * }
 * \endcode
 */
void CAN_SetMBnRxDmaEnFlag(uint8_t message_buffer_index,
                           FunctionalState newState);

/**
 * \brief  Set test mode for CAN.
 * \param[in] CANx: selected CAN peripheral.
 * \param[in] CAN_TestModeSel: Test mode to set.
 *      This parameter can the following values:
 *      \arg CAN_TEST_MODE_NONE: Normal tx/rx mode.
 *      \arg CAN_TEST_MODE_INT_LOOPBACK: Loopback mode.
 *      \arg CAN_TEST_MODE_SILENCE: Silence mode.
 * \return none.
 */
void CAN_SetTestMode(uint8_t CAN_TestModeSel);

/**
 * \brief  Enable or disable CAN auto-re-tx function.
 * \param[in] CANx: selected CAN peripheral.
 * \param[in] newState: the state of CAN auto-re-tx function.
 * \return none.
 */
void CAN_AutoReTxCmd(FunctionalState NewState);

/**
 * \brief  Set can bit timing.
 * \param[in] CANx: selected CAN peripheral.
 * \param[in] CAN_BitTiming:  Pointer to a CAN_BIT_TIMING_TYPE_TypeDef structure that
 *            contains the bit timing information for the specified CAN peripheral
 * \return none.
 */
void CAN_SetTiming(CAN_BIT_TIMING_TYPE_TypeDef *CAN_BitTiming);

/**
 * \brief  Get error passive status.
 * \param[in] CANx: selected CAN peripheral.
 * \return The flag of error passive status.
 */
FlagStatus CAN_GetErrorPassiveStatus();

/**
 * \brief  Get error warning status.
 * \param[in] CANx: selected CAN peripheral.
 * \return The flag of error warning status.
 */
FlagStatus CAN_GetErrorWarningStatus();

/**
 * \brief  Get tx error counter.
 * \param[in] CANx: selected CAN peripheral.
 * \return The counter of tx error.
 */
int CAN_GetTxErrorCnt();

/**
 * \brief  Get rx error counter.
 * \param[in] CANx: selected CAN peripheral.
 * \return The counter of rx error.
 */
int CAN_GetRxErrorCnt();

/**
 * \brief  Config CAN clock source div.
 *
 * \param[in] div  CAN clock div. Refer to \ref CANClkDIV_TypeDef.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * static void can_config_clock(void)
 * {
 *     CAN_ClkDivConfig(CAN_CLK_DIV_1);
 * }
 * \endcode
 */
void CAN_ClkDivConfig(CANClockDiv_TypeDef div);

/**
 * \brief  Get CAN clock divider.
 *
 * \param[out] ClockSrc  CAN clock src.
 * \param[out] ClockDiv  CAN clock div.
 *
 * \return The status of get clock.
 */
bool CAN_ClkGet(CANClockSrc_TypeDef *ClockSrc, CANClockDiv_TypeDef *ClockDiv);

#if (CAN_SUPPORT_SLEEP_MODE == 1)
/**
 * \brief  Check CAN sleep state.
 *
 * \return The flag of CAN sleep state.
 */
FlagStatus CAN_CheckSleepStatus();

/**
 * \brief  Enable or disable CAN lower power clock.
 *
 * \param[in] newState  Enable or disable CAN lower power clock.
 * \param[in] div       Low power clock div.
 */
void CAN_LowPowerClkCmd(FunctionalState newState, CANLowPowerClkDIV_TypeDef div);

/**
 * \brief  Config wake up pin fit function.
 *
 * \param[in] newState    Enable or disable CAN wake up pin fit function.
 * \param[in] flt_length  Config wake up pin fit length.
 *                        minimu value is 1. The unit is low power clock period.
 */
void CAN_SetWakeUpPinFltFunction(FunctionalState newState, uint8_t flt_length);
/**
 * \brief  Request CAN to sleep mode.
 *
 */
void CAN_RequestToSleepMode();

/**
 * \brief  Manual wake up CAN.
 *
 * \return The flag of message buffer rx dma enable.
 */
void CAN_ManualWakeup();
#endif

/** End of CAN_Exported_Functions
  * \}
  */
/** End of CAN
  * \}
  */
#ifdef __cplusplus
}
#endif
#endif /* _RTL_CAN_H_ */

