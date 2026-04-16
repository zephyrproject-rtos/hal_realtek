/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef RTL_KEYSCAN_H
#define RTL_KEYSCAN_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "utils/rtl_utils.h"
#include "rtl_keyscan_def.h"

/** \defgroup KEYSCAN     KEYSCAN
  * \brief
  * \{
  */

/*============================================================================*
 *                         Constants
 *============================================================================*/
/** \defgroup KEYSCAN_Exported_Constants KEYSCAN Exported Constants
  * \brief
  * \{
  */

/**
 * \defgroup    KEYSCAN_FIFO_Depth KEYSCAN FIFO Depth
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define KEYSCAN_FIFO_DEPTH              26   //!< The FIFO depth of the KEYSCAN is 26.

/** End of KEYSCAN_FIFO_Depth
  * \}
  */

/**
 * \defgroup    KEYSCAN_Row_Number KEYSCAN Row Number
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define IS_KEYSCAN_ROW_NUM(ROW) ((ROW) <= 12)   //!< The row number of the KEYSCAN need less than or equal to 12.

/** End of KEYSCAN_Row_Number
  * \}
  */

/**
 * \defgroup    KEYSCAN_Column_Number KEYSCAN Column Number
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define IS_KEYSCAN_COL_NUM(COL) ((COL) <= 20)   //!< The column number of the KEYSCAN need less than or equal to 20.

/** End of KEYSCAN_Column_Number
  * \}
  */

/**
 * \defgroup    KEYSCAN_Debounce_Config KEYSCAN Debounce Config
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define IS_KEYSCAN_DEBOUNCE_EN(EN) (((EN) == ENABLE) || ((EN) == DISABLE))   //!< The debounce config of the KEYSCAN can be ENABLE or DISABLE  

/** End of KEYSCAN_Debounce_Config
  * \}
  */

/**
 * \defgroup    KEYSCAN_Scan_Interval_Enable KEYSCAN Scan Interval Enable
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define IS_KEYSCAN_SCANINTERVAL_EN(EN) (((EN) == ENABLE) || ((EN) == DISABLE))   //!< The scan interval of the KEYSCAN can be ENABLE or DISABLE

/** End of KEYSCAN_Scan_Interval_Enable
  * \}
  */

/**
 * \defgroup    KEYSCAN_Release_Detect_Timer_Enable KEYSCAN Release Detect Timer Enable
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define IS_KEYSCAN_RELEASE_DETECT_EN(EN) (((EN) == ENABLE) || ((EN) == DISABLE))   //!< The release detect timer of the KEYSCAN can be ENABLE or DISABLE

/** End of KEYSCAN_Release_Detect_Timer_Enable
  * \}
  */

/**
 * \defgroup    KEYSCAN_Scan_Mode KEYSCAN Scan Mode
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
typedef enum
{
    KeyScan_Manual_Scan_Mode = 0x00,   //!< The KEYSCAN manual scan mode.
    KeyScan_Auto_Scan_Mode = 0x01,     //!< The KEYSCAN auto scan mode.
} KEYSCANScanMode_TypeDef;

#define IS_KEYSCAN_SCAN_MODE(MODE)  (((MODE) == KeyScan_Manual_Scan_Mode) || ((MODE) == KeyScan_Auto_Scan_Mode)) //!< Check if the input parameter is valid.

/** End of KEYSCAN_Scan_Mode
  * \}
  */

/**
 * \defgroup    KEYSCAN_Press_Detect_Mode KEYSCAN Press Detect Mode
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
typedef enum
{
    KeyScan_Detect_Mode_Edge = 0x00,    //!< The key detection mode is edge-triggered.
    KeyScan_Detect_Mode_Level = 0x01,   //!< The key detection mode is level-triggered.
} KEYSCANPressDetectMode_TypeDef;

#define IS_KEYSCAN_DETECT_MODE(MODE)    (((MODE) == KeyScan_Detect_Mode_Edge) || ((MODE) == KeyScan_Detect_Mode_Level)) //!< Check if the input parameter is valid.

/** End of KEYSCAN_Press_Detect_Mode
  * \}
  */

/**
 * \defgroup    KEYSCAN_FIFO_Overflow_Control KEYSCAN FIFO Overflow Control
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
typedef enum
{
    KeyScan_FIFO_OVR_CTRL_DIS_ALL = 0x00,     //!< Discard the new scan data when FIFO is full.
    KeyScan_FIFO_OVR_CTRL_DIS_LAST = 0x01,    //!< Discard the oldest scan data when FIFO is full.
} KEYSCANFifoOverflowControl_TypeDef;

#define IS_KEYSCAN_FIFO_OVR_CTRL(CTRL)  (((CTRL) == KeyScan_FIFO_OVR_CTRL_DIS_ALL) || ((CTRL) == KeyScan_FIFO_OVR_CTRL_DIS_LAST)) //!< Check if the input parameter is valid.

/** End of KEYSCAN_FIFO_Overflow_Control
  * \}
  */

/**
 * \defgroup    KEYSCAN_Manual_Sel_Mode KEYSCAN Manual Sel Mode
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
typedef enum
{
    KeyScan_Manual_Sel_Bit = 0x00,     //!< KEYSCAN manual trigger register (Call API Keyscan_Cmd)
    KeyScan_Manual_Sel_Key = 0x01,     //!< KEYSCAN manual trigger by key.
} KEYSCANManualMode_TypeDef;

/** End of KEYSCAN_Manual_Sel_Mode
  * \}
  */

#if (KEYSCAN_SUPPORT_RAP_FUNCTION == 1)
/**
 * \defgroup    KEYSCAN_Qactive_Force KEYSCAN Qactive Force
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
typedef enum
{
    KEYSCAN_QACTIVE_FW_FORCE = 0x0,
    KEYSCAN_QACTIVE_FW_FORCE_PCLK = 0x1,
} KeyScanQactiveForce_TypeDef;

/** End of KEYSCAN_Qactive_Force
  * \}
  */

/**
 * \defgroup    KEYSCAN_Task KEYSCAN Task
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
typedef enum
{
    KEYSCAN_TASK_MANUAL = 0,
} KEYSCANTask_TypeDef;

/** End of KEYSCAN_Task
  * \}
  */
#endif

/**
 * \defgroup    KEYSCAN_Key_Limit KEYSCAN Key Limit
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define IS_KEYSCAN_KEY_LIMIT(DATA_NUM) ((DATA_NUM) <= KEYSCAN_FIFO_DEPTH) //!< The key limit of the KEYSCAN need less than or equal to KEYSCAN_FIFO_DEPTH.

/** End of KEYSCAN_Key_Limit
  * \}
  */

/**
 * \defgroup    KEYSCAN_Interrupt KEYSCAN Interrupt
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define KEYSCAN_INT_THRESHOLD                    BIT4     //!< KEYSCAN FIFO data over threshold interrupt.
#define KEYSCAN_INT_OVER_READ                    BIT3     //!< KEYSCAN over read interrupt.
#define KEYSCAN_INT_SCAN_END                     BIT2     //!< KEYSCAN scan end interrupt.
#define KEYSCAN_INT_FIFO_NOT_EMPTY               BIT1     //!< KEYSCAN FIFO not empty interrupt.
#define KEYSCAN_INT_ALL_RELEASE                  BIT0     //!< KEYSCAN all key release interrupt.
#define IS_KEYSCAN_CONFIG_IT(IT) ((((IT) & (uint32_t)0xFFF8) == 0x00) && ((IT) != 0x00))     //!< KEYSCAN all interrupt configurations.

/** End of KEYSCAN_Interrupt
  * \}
  */

/**
 * \defgroup    KEYSCAN_Flag KEYSCAN Flag
 * \{
 * \ingroup     KEYSCAN_Exported_Constants
 */
#define KEYSCAN_FLAG_FIFOLIMIT                       BIT20     //!< When data filtering occurs, this bit will be set to 1.
#define KEYSCAN_INT_FLAG_THRESHOLD                   BIT19     //!< FIFO threshold interrupt status.
#define KEYSCAN_INT_FLAG_OVER_READ                   BIT18     //!< FIFO over read interrupt status.
#define KEYSCAN_INT_FLAG_SCAN_END                    BIT17     //!< Scan finish interrupt status.
#define KEYSCAN_INT_FLAG_FIFO_NOT_EMPTY              BIT16     //!< FIFO not empty interrupt status.
#define KEYSCAN_INT_FLAG_ALL_RELEASE                 BIT15     //!< All release interrupt status.
#define KEYSCAN_FLAG_DATAFILTER                      BIT3      //!< FIFO data filter status.
#define KEYSCAN_FLAG_OVR                             BIT2      //!< FIFO overflow status.
#define KEYSCAN_FLAG_FULL                            BIT1      //!< FIFO full status.
#define KEYSCAN_FLAG_EMPTY                           BIT0      //!< FIFO empty status.
#define IS_KEYSCAN_FLAG(FLAG)       ((((FLAG) & (uint32_t)0x01FF) == 0x00) && ((FLAG) != (uint32_t)0x00))     //!< KEYSCAN all interrupt flags.
#define IS_KEYSCAN_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0x00C0) == 0x00) && ((FLAG) != (uint32_t)0x00))     //!< KEYSCAN all clear flags.

/** End of KEYSCAN_Flag
  * \}
  */

/** End of KEYSCAN_Exported_Constants
  * \}
  */

/*============================================================================*
 *                         Types
 *============================================================================*/
/** \defgroup KEYSCAN_Exported_Types KEYSCAN Exported Types
  * \brief
  * \{
  */

/**
 * \brief       KEYSCAN initialize parameters
 *
 * \ingroup     KEYSCAN_Exported_Types
 */
typedef struct
{
    uint16_t rowSize;                               /*!< Specify KEYSCAN Row Size.
                                                         This parameter can be set to a value of 12 or less. */

    uint16_t colSize;                               /*!< Specify KEYSCAN Column Size.
                                                         This parameter can be set to a value of 20 or less. */

    uint16_t clockdiv;                              /*!< Specify KEYSCAN clock divider.
                                                         scan clock = system clock/(SCAN_DIV+1). */

    uint8_t delayclk;                               /*!< Specify KEYSCAN delay clock divider.
                                                         delay clock = scan clock/(delayclk+1). */

    FunctionalState debounceEn;                     /*!< Enable or disable debounce.
                                                         This parameter can be a value of DISABLE or ENABLE. */

    FunctionalState scantimerEn;                    /*!< Enable or disable scan timer.
                                                         This parameter can be a value of DISABLE or ENABLE. */

    FunctionalState detecttimerEn;                  /*!< Enable or disable detect timer.
                                                         This parameter can be a value of DISABLE or ENABLE. */

    uint16_t debouncecnt;                           /*!< Specify KEYSCAN debounce timer.
                                                         debounce time = delay clock * debouncecnt.
                                                         This parameter can be a value of 0 ~ 0x1FF. */

    uint16_t scanInterval;                          /*!< Specify KEYSCAN scan interval.
                                                         scan interval time = delay clock * scanInterval.
                                                         This parameter can be a value of 0 ~ 0x1FF. */

    uint16_t releasecnt;                            /*!< Specify KEYSCAN release time.
                                                         release time = delay clock * releasecnt.
                                                         This parameter can be a value of 0 ~ 0x1FF. */

    KEYSCANScanMode_TypeDef scanmode;               /*!< Specify KEYSCAN scan mode.
                                                         This parameter can be a value of \ref KEYSCAN_Scan_Mode. */

    KEYSCANPressDetectMode_TypeDef detectMode;      /*!< Specify key detection mode of KEYSCAN.
                                                         This parameter can be a value of \ref KEYSCAN_Press_Detect_Mode. */

    uint16_t fifotriggerlevel;                      /*!< Specify KEYSCAN FIFO threshold to trigger interrupt.
                                                         This parameter can be a value of 0 ~ 26. */

    KEYSCANFifoOverflowControl_TypeDef
    fifoOvrCtrl;                                    /*!< Specify KEYSCAN fifo over flow control.
                                                         This parameter can be a value of \ref KEYSCAN_FIFO_Overflow_Control. */

    uint8_t keylimit;                               /*!< Specify the maximum allowable scan data for each scan.
                                                         This parameter can be a value of 0 ~ 26. */

    KEYSCANManualMode_TypeDef
    manual_sel;                                     /*!< Specify trigger mode in manual scan mode.
                                                         This parameter can be a value of \ref KEYSCAN_Manual_Sel_Mode. */

#if KEYSCAN_SUPPORT_ROW_LEVEL_CONFIGURE
    FunctionalState
    rowpullhighEn;                  /*!< Enable or disable the function of row pull high.
                                                         This parameter can be a value of DISABLE or ENABLE. */
#endif
#if KEYSCAN_SUPPORT_COLUNM_LEVEL_CONFIGURE
    FunctionalState colunmoutputhighEn;
#endif
} KEYSCAN_InitTypeDef;

/** End of KEYSCAN_Exported_Types
  * \}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** \defgroup KEYSCAN_Exported_Functions KEYSCAN Exported Functions
  * \brief
  * \{
  */

/**
 * \brief  Deinitialize the KEYSCAN peripheral registers to their default reset values(turn off keyscan clock).
 *
 * \param[in]  KeyScan  Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_keyscan_init(void)
 * {
 *     KeyScan_DeInit(KEYSCAN);
 * }
 * \endcode
 */
void KeyScan_DeInit(KEYSCAN_TypeDef *KeyScan);

/**
 * \brief   Initialize the KeyScan peripheral according to the specified
 *          parameters in the KeyScan_InitStruct
 *
 * \param[in]  KeyScan             Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in]  KeyScan_InitStruct  Pointer to a KEYSCAN_InitTypeDef structure which will be initialized.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_keyscan_init(void)
 * {
 *     RCC_PeriphClockCmd(APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK, ENABLE);

 *     KEYSCAN_InitTypeDef KEYSCAN_InitStruct;
 *     KeyScan_StructInit(&KEYSCAN_InitStruct);

 *     KEYSCAN_InitStruct.rowSize  = KEYBOARD_ROW_SIZE;
 *     KEYSCAN_InitStruct.colSize  = KEYBOARD_COLUMN_SIZE;
 *     KEYSCAN_InitStruct.scanmode     = KeyScan_Manual_Scan_Mode;
 *     KEYSCAN_InitStruct.debounceEn   = KeyScan_Debounce_Enable;

 *     KeyScan_Init(KEYSCAN, &KEYSCAN_InitStruct);

 *     KeyScan_INTConfig(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);
 *     KeyScan_ClearINTPendingBit(KEYSCAN, KEYSCAN_INT_SCAN_END);
 *     KeyScan_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, DISABLE);  // Unmask keyscan interrupt
 *     KeyScan_Cmd(KEYSCAN, ENABLE);
 * }
 * \endcode
 */
void KeyScan_Init(KEYSCAN_TypeDef *KeyScan, KEYSCAN_InitTypeDef *KeyScan_InitStruct);

/**
 * \brief  Fill each KEYSCAN_InitStruct member with its default value.
 *
 * \note The default settings for the KeyScan_InitStruct member are shown in the following table:
 *       | KeyScan_InitStruct member    | Default value                        |
 *       |:----------------------------:|:------------------------------------:|
 *       | colSize                      | 2                                    |
 *       | rowSize                      | 2                                    |
 *       | clockdiv                     | 0x1f8                                |
 *       | delayclk                     | 0x01                                 |
 *       | debounceEn                   | ENABLE                               |
 *       | scantimerEn                  | ENABLE                               |
 *       | detecttimerEn                | ENABLE                               |
 *       | debouncecnt                  | 0x10                                 |
 *       | scanInterval                 | 0x10                                 |
 *       | releasecnt                   | 0x1                                  |
 *       | scanmode                     | \ref KeyScan_Auto_Scan_Mode          |
 *       | detectMode                   | \ref KeyScan_Detect_Mode_Level       |
 *       | manual_sel                   | \ref KeyScan_Manual_Sel_Key          |
 *       | fifotriggerlevel             | 1                                    |
 *       | fifoOvrCtrl                  | \ref KeyScan_FIFO_OVR_CTRL_DIS_LAST  |
 *       | keylimit                     | 0x03                                 |
 *       | rowpullhighEn                | ENABLE                               |
 *
 * \param[in] KeyScan_InitStruct  Pointer to a KEYSCAN_InitTypeDef structure which will be initialized.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_keyscan_init(void)
 * {
 *     RCC_PeriphClockCmd(APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK, ENABLE);

 *     KEYSCAN_InitTypeDef KEYSCAN_InitStruct;
 *     KeyScan_StructInit(&KEYSCAN_InitStruct);

 *     KEYSCAN_InitStruct.rowSize  = KEYBOARD_ROW_SIZE;
 *     KEYSCAN_InitStruct.colSize  = KEYBOARD_COLUMN_SIZE;
 *     KEYSCAN_InitStruct.scanmode     = KeyScan_Manual_Scan_Mode;
 *     KEYSCAN_InitStruct.debounceEn   = KeyScan_Debounce_Enable;

 *     KeyScan_Init(KEYSCAN, &KEYSCAN_InitStruct);

 *     KeyScan_INTConfig(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);
 *     KeyScan_ClearINTPendingBit(KEYSCAN, KEYSCAN_INT_SCAN_END);
 *     KeyScan_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, DISABLE);  // Unmask keyscan interrupt
 *     KeyScan_Cmd(KEYSCAN, ENABLE);
 * }
 * \endcode
 */
void KeyScan_StructInit(KEYSCAN_InitTypeDef *KeyScan_InitStruct);

/**
 * \brief  Enable or disable the specified KeyScan interrupt.
 *
 * \param[in] KeyScan     Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in] KeyScan_IT  Specify the KeyScan interrupts sources to be enabled or disabled. Refer to \ref KEYSCAN_Interrupt.
 *                        This parameter can be any combination of the following values:
 *                        \arg KEYSCAN_INT_THRESHOLD: Keyscan FIFO data over threshold interrupt.
 *                        \arg KEYSCAN_INT_OVER_READ: KeyScan over read interrupt.
 *                        \arg KEYSCAN_INT_SCAN_END: KeyScan scan end interrupt.
 *                        \arg KEYSCAN_INT_FIFO_NOT_EMPTY: KeyScan FIFO not empty interrupt.
 *                        \arg KEYSCAN_INT_ALL_RELEASE: KeyScan all key release interrupt.
 * \param[in] NewState    New state of the specified KeyScan interrupts.
 *                        This parameter can be one of the following values:
 *                        - ENABLE: Enable the interrupt of KeyScan.
 *                        - DISABLE: Disable the interrupt of KeyScan.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     KeyScan_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);
 *     KeyScan_INTConfig(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);
 *     KeyScan_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, DISABLE);
 * }
 * \endcode
 */
void KeyScan_INTConfig(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_IT,
                       FunctionalState NewState);

/**
 * \brief  Mask the specified KeyScan interrupt.
 *
 * \param[in] KeyScan     Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in] KeyScan_IT  Specify the KeyScan interrupts sources to be enabled or disabled. Refer to \ref KEYSCAN_Interrupt.
 *                        This parameter can be any combination of the following values:
 *                        \arg KEYSCAN_INT_THRESHOLD: Keyscan FIFO data over threshold interrupt.
 *                        \arg KEYSCAN_INT_OVER_READ: KeyScan over read interrupt.
 *                        \arg KEYSCAN_INT_SCAN_END: KeyScan scan end interrupt.
 *                        \arg KEYSCAN_INT_FIFO_NOT_EMPTY: KeyScan FIFO not empty interrupt.
 *                        \arg KEYSCAN_INT_ALL_RELEASE: KeyScan all key release interrupt.
 * \param[in] NewState    New state of the specified KeyScan interrupt mask.
 *                        This parameter can be one of the following values:
 *                        - ENABLE: Enable the interrupt mask of KeyScan.
 *                        - DISABLE: Disable the interrupt mask of KeyScan.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     KeyScan_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);
 * }
 * \endcode
 */
void KeyScan_INTMask(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_IT,
                     FunctionalState NewState);

/**
 * \brief  Read data from keyscan FIFO.
 *
 * \param[in]  KeyScan  Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[out] outBuf   Buffer to save data read from KeyScan FIFO.
 * \param[in]  count    Data length to be read.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     uint16_t data[3] = {0};
 *     KeyScan_Read(KEYSCAN, data, 3);
 * }
 * \endcode
 */
void KeyScan_Read(KEYSCAN_TypeDef *KeyScan, uint16_t *outBuf, uint16_t count);

/**
 * \brief   Enable or disable the KeyScan peripheral.
 *
 * \param[in] KeyScan   Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in] NewState  New state of the KeyScan peripheral.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable KEYSCAN scanning. When an external key is pressed,
 *                        it will trigger the keyscan scanning. When configured in manual scan mode and bit trigger,
 *                        calling this API will immediately execute the keyscan scanning.
 *                      - DISABLE: Disable KEYSCAN scanning.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     KeyScan_Cmd(KEYSCAN, ENABLE);
 * }
 * \endcode
 */
void KeyScan_Cmd(KEYSCAN_TypeDef *KeyScan, FunctionalState NewState);

/**
 * \brief   Set filter data.
 *
 * \param[in] KeyScan   Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in] data      Config the data to be filtered.
 *                      This parameter should not be more than 9 bits.
 * \param[in] NewState  New state of the KeyScan filtering.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable data filtering.
 *                      - DISABLE: Disable data filtering.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     KeyScan_FilterDataConfig(KEYSCAN, 0x01, ENABLE);
 *
 * }
 * \endcode
 */
void KeyScan_FilterDataConfig(KEYSCAN_TypeDef *KeyScan, uint16_t data,
                              FunctionalState NewState);

/**
 * \brief   KeyScan debounce time config.
 *
 * \param[in] KeyScan   Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in] time      KEYSCAN hardware debounce time. debounce time = delay clock * time.
 * \param[in] NewState  New state of the KeyScan debounce function.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable KeyScan debounce function.
 *                      - DISABLE: Disable KeyScan debounce function.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     KeyScan_debounceConfig(KEYSCAN, 10, ENABLE);
 *
 * }
 * \endcode
 */
void KeyScan_debounceConfig(KEYSCAN_TypeDef *KeyScan, uint16_t time,
                            FunctionalState NewState);

/**
 * \brief   Get KeyScan FIFO data num.
 *
 * \param[in]  KeyScan  Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 *
 * \return  Data length in FIFO.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     uint16_t data_len = KeyScan_GetFifoDataNum(KEYSCAN);
 * }
 * \endcode
 */
uint16_t KeyScan_GetFifoDataNum(KEYSCAN_TypeDef *KeyScan);

/**
 * \brief  Clear the KeyScan interrupt pending bit.
 *
 * \param[in] KeyScan     Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in] KeyScan_IT  Specify the interrupt pending bit to clear. Refer to \ref KEYSCAN_Interrupt.
 *                        This parameter can be any combination of the following values:
 *                        - KEYSCAN_INT_THRESHOLD: Keyscan FIFO data over threshold interrupt.
 *                        - KEYSCAN_INT_OVER_READ: KeyScan over read interrupt.
 *                        - KEYSCAN_INT_SCAN_END: KeyScan scan end interrupt.
 *                        - KEYSCAN_INT_FIFO_NOT_EMPTY: KeyScan FIFO not empty interrupt.
 *                        - KEYSCAN_INT_ALL_RELEASE: KeyScan all key release interrupt.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     KeyScan_ClearINTPendingBit(KEYSCAN, KEYSCAN_INT_SCAN_END);
 * }
 * \endcode
 */
void KeyScan_ClearINTPendingBit(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_IT);

/**
 * \brief   Clear the specified KeyScan flag.
 *
 * \param[in] KeyScan       Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in] KeyScan_FLAG  Specify the flag to clear.
 *                          This parameter can be one of the following values, refer to \ref KEYSCAN_Flag.
 *                          \arg KEYSCAN_FLAG_FIFOLIMIT: When data filtering occurs, this bit will be set to 1.
 *                          \arg KEYSCAN_FLAG_DATAFILTER: FIFO data filter status.
 *                          \arg KEYSCAN_FLAG_OVR: FIFO overflow status.
 *
 * \note    KEYSCAN_FLAG_FULL and KEYSCAN_FLAG_EMPTY can't be cleared manually.
 *          They are cleared by hardware automatically.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     KeyScan_ClearFlags(KEYSCAN, KEYSCAN_FLAG_FIFOLIMIT);
 * }
 * \endcode
 */
void KeyScan_ClearFlags(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_FLAG);

/**
 * \brief   Get the specified KeyScan flag status.
 *
 * \param[in] KeyScan       Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 * \param[in] KeyScan_FLAG  Specify the flag to check.
 *                          This parameter can be one of the following values, refer to \ref KEYSCAN_Flag.
 *                          \arg KEYSCAN_FLAG_FIFOLIMIT: When data filtering occurs, this bit will be set to 1.
 *                          \arg KEYSCAN_FLAG_THRESHOLD: FIFO threshold interrupt status.
 *                          \arg KEYSCAN_FLAG_OVER_READ: FIFO over read interrupt status.
 *                          \arg KEYSCAN_FLAG_SCAN_END: Scan finish interrupt status.
 *                          \arg KEYSCAN_FLAG_FIFO_NOT_EMPTY: FIFO not empty interrupt status.
 *                          \arg KEYSCAN_FLAG_ALL_RELEASE: All release interrupt status.
 *                          \arg KEYSCAN_FLAG_DATAFILTER: FIFO data filter status.
 *                          \arg KEYSCAN_FLAG_OVR: FIFO overflow status.
 *                          \arg KEYSCAN_FLAG_FULL: FIFO full status.
 *                          \arg KEYSCAN_FLAG_EMPTY: FIFO empty status.
 *
 * \return  The status of KeyScan flag.
 *          - SET: The specified KeyScan flag has been set.
 *          - RESET: The specified KeyScan flag bit has not been set.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     FlagStatus flag_status = KeyScan_GetFlagState(KEYSCAN, KEYSCAN_FLAG_OVR);
 *
 * }
 * \endcode
 */
FlagStatus KeyScan_GetFlagState(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_FLAG);

/**
 * \brief  Read keyscan FIFO data.
 *
 * \param[in] KeyScan  Select KeyScan peripheral. Refer to \ref KEYSCAN_Declaration.
 *
 * \return KEYSCAN FIFO data.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void keyscan_demo(void)
 * {
 *     uint16_t data = KeyScan_ReadFifoData(KEYSCAN);
 * }
 * \endcode
 */
uint16_t KeyScan_ReadFifoData(KEYSCAN_TypeDef *KeyScan);


#if (KEYSCAN_SUPPORT_RAP_FUNCTION == 1)

void KeyScan_RAPModeCmd(KEYSCAN_TypeDef *KeyScan, FunctionalState NewState);

void KEYSCAN_TaskTrigger(KEYSCAN_TypeDef *KeyScan, uint32_t Task);

void KeyScan_RAPQactiveCtrl(KEYSCAN_TypeDef *KeyScan, uint32_t Qactive, FunctionalState NewState);

#endif

/** End of KEYSCAN_Exported_Functions
  * \}
  */

/** End of KEYSCAN
  * \}
  */

#ifdef __cplusplus
}
#endif

#endif /* RTL_KEYSCAN_H */


