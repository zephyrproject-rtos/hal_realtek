/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef RTL_ADC_H
#define RTL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "utils/rtl_utils.h"
#include "rtl_adc_def.h"
#include "adc_lib.h"

/** \defgroup ADC         ADC
  * \brief
  * \{
  */

/*============================================================================*
 *                         Constants
 *============================================================================*/
/** \defgroup ADC_Exported_Constants ADC Exported Constants
  * \brief
  * \{
  */

/**
 * \defgroup    ADC_Channel_Index ADC Channel Index
 * \{
 * \ingroup     ADC_Exported_Constants
 */
#define ADC_Channel_Index_0           0   //!< ADC channel 0.
#define ADC_Channel_Index_1           1   //!< ADC channel 1.
#define ADC_Channel_Index_2           2   //!< ADC channel 2.
#define ADC_Channel_Index_3           3   //!< ADC channel 3.
#define ADC_Channel_Index_4           4   //!< ADC channel 4.
#if (CHIP_ADC_CHANNEL_NUM > 4)
#define ADC_Channel_Index_5           5   //!< ADC channel 5.
#define ADC_Channel_Index_6           6   //!< ADC channel 6.
#define ADC_Channel_Index_7           7   //!< ADC channel 7.
#endif
#if (CHIP_ADC_CHANNEL_NUM > 8)
#define ADC_Channel_Index_8           8   //!< ADC channel 8.
#define ADC_Channel_Index_9           9   //!< ADC channel 9.
#define ADC_Channel_Index_10          10  //!< ADC channel 10.
#define ADC_Channel_Index_11          11  //!< ADC channel 11.
#define ADC_Channel_Index_12          12  //!< ADC channel 12.
#define ADC_Channel_Index_13          13  //!< ADC channel 13.
#define ADC_Channel_Index_14          14  //!< ADC channel 14.
#define ADC_Channel_Index_15          15  //!< ADC channel 15.
#endif
#define IS_ADC_CHANNEL(ch)      ((ch) < CHIP_ADC_CHANNEL_NUM) //!< Specify the maximum ADC channel number.

/** End of ADC_Channel_Index
  * \}
  */

/**
 * \defgroup    ADC_Schedule_Index ADC Schedule Index
 * \{
 * \ingroup     ADC_Exported_Constants
 */
#define ADC_Schedule_Index_0          0    //!< ADC schedule index 0.
#define ADC_Schedule_Index_1          1    //!< ADC schedule index 1.
#define ADC_Schedule_Index_2          2    //!< ADC schedule index 2.
#define ADC_Schedule_Index_3          3    //!< ADC schedule index 3.
#define ADC_Schedule_Index_4          4    //!< ADC schedule index 4.
#define ADC_Schedule_Index_5          5    //!< ADC schedule index 5.
#define ADC_Schedule_Index_6          6    //!< ADC schedule index 6.
#define ADC_Schedule_Index_7          7    //!< ADC schedule index 7.
#define ADC_Schedule_Index_8          8    //!< ADC schedule index 8.
#define ADC_Schedule_Index_9          9    //!< ADC schedule index 9.
#define ADC_Schedule_Index_10         10   //!< ADC schedule index 10.
#define ADC_Schedule_Index_11         11   //!< ADC schedule index 11.
#define ADC_Schedule_Index_12         12   //!< ADC schedule index 12.
#define ADC_Schedule_Index_13         13   //!< ADC schedule index 13.
#define ADC_Schedule_Index_14         14   //!< ADC schedule index 14.
#define ADC_Schedule_Index_15         15   //!< ADC schedule index 15.
#if (CHIP_ADC_SCHEDULE_NUM > 16)
#define ADC_Schedule_Index_16         16   //!< ADC schedule index 16.
#define ADC_Schedule_Index_17         17   //!< ADC schedule index 17.
#define ADC_Schedule_Index_18         18   //!< ADC schedule index 18.
#define ADC_Schedule_Index_19         19   //!< ADC schedule index 19.
#endif
#define IS_ADC_SCH_INDEX(IDEX) ((IDEX) < CHIP_ADC_SCHEDULE_NUM) //!< Specify the maximum ADC schedule number.

/** End of ADC_Schedule_Index
  * \}
  */

/**
 * \defgroup    ADC_Schedule_Table ADC Channel and Mode
 * \{
 * \ingroup     ADC_Exported_Constants
 */
#define SCHEDULE_TABLE(Index)         (Index) //!< ADC schedule table index.
#define EXT_SINGLE_ENDED(Index)       ((uint16_t)((ADC_MODE_SINGLE_ENDED_VALUE << CHIP_ADC_MODE_OFFSET) | (Index))) //!< External single-ended mode. Index can be  refered to \ref ADC_Channel_Index. 
#define EXT_DIFFERENTIAL(Index)       ((uint16_t)((ADC_MODE_DIFFERENTIAL_VALUE << CHIP_ADC_MODE_OFFSET) | (Index))) //!< External differential mode. Index can be  refered to \ref ADC_Channel_Index.

#define INTERNAL_VBAT_MODE            ((uint16_t)((ADC_MODE_INTERNAL_VALUE << CHIP_ADC_MODE_OFFSET) | 0x00)) //!< Internal VBAT mode.
#if ADC_SUPPORT_VADPIN_MODE
#define INTERNAL_VADPIN_MODE          ((uint16_t)((ADC_MODE_INTERNAL_VALUE << CHIP_ADC_MODE_OFFSET) | 0x01)) //!< Internal VADPIN mode.
#endif


#if ADC_SUPPORT_VADPIN_MODE
#define IS_ADC_SCHEDULE_INDEX_CONFIG(CONFIG) (((CONFIG) & (0xffff << 2 << CHIP_ADC_MODE_OFFSET)) == 0 && \
                                              ((IS_ADC_SCH_INDEX((CONFIG) & (~(0xffff << CHIP_ADC_MODE_OFFSET))) && \
                                                (CONFIG & BIT(CHIP_ADC_MODE_OFFSET + 1) == 0)) || \
                                               (CONFIG) == INTERNAL_VBAT_MODE || \
                                               (CONFIG) == INTERNAL_VADPIN_MODE)) //!< Check if the input parameter is valid.
#else
#define IS_ADC_SCHEDULE_INDEX_CONFIG(CONFIG) (((CONFIG) & (0xffff << 2 << CHIP_ADC_MODE_OFFSET)) == 0 && \
                                              ((IS_ADC_SCH_INDEX((CONFIG) & (~(0xffff << CHIP_ADC_MODE_OFFSET))) && \
                                                (CONFIG & BIT(CHIP_ADC_MODE_OFFSET + 1) == 0)) || \
                                               (CONFIG) == INTERNAL_VBAT_MODE)) //!< Check if the input parameter is valid.
#endif

/** End of ADC_Schedule_Table
  * \}
  */

/**
 * \defgroup    ADC_Convert_Time ADC Convert Time
 * \{
 * \ingroup     ADC_Exported_Constants
 */
typedef enum
{
    ADC_CONVERT_TIME_500NS,  //!< ADC Convert time is 500ns.
    ADC_CONVERT_TIME_700NS,  //!< ADC Convert time is 700ns.
    ADC_CONVERT_TIME_900NS,  //!< ADC Convert time is 900ns.
    ADC_CONVERT_TIME_1100NS, //!< ADC Convert time is 1100ns.
} ADCConvertTim_TypeDef;

#define IS_ADC_CONVERT_TIME(TIME) (((TIME) == ADC_CONVERT_TIME_500NS) || \
                                   ((TIME) == ADC_CONVERT_TIME_700NS) || \
                                   ((TIME) == ADC_CONVERT_TIME_900NS) || \
                                   ((TIME) == ADC_CONVERT_TIME_1100NS)) //!< Check if the input parameter is valid.

/** End of ADC_Convert_Time
  * \}
  */

/**
 * \defgroup    ADC_Latch_Data_Edge ADC Latch Data Edge
 * \{
 * \ingroup     ADC_Exported_Constants
 */
typedef enum
{
    ADC_LATCH_DATA_Positive, //!< ADC latch data at positive clock edge.
    ADC_LATCH_DATA_Negative, //!< ADC latch data at negative clock edge.
} ADCDataLatchEdge_TypeDef;

#define IS_ADC_LATCH_MODE(MODE) (((MODE) == ADC_LATCH_DATA_Positive) || ((MODE) == ADC_LATCH_DATA_Negative)) //!< Check if the input parameter is valid.

/** End of ADC_Latch_Data_Edge
  * \}
  */

/**
 * \defgroup    ADC_Data_Align ADC Data Align
 * \{
 * \ingroup     ADC_Exported_Constants
 */
typedef enum
{
    ADC_DATA_ALIGN_LSB, //!< ADC data storage format is LSB.
    ADC_DATA_ALIGN_MSB, //!< ADC data storage format is MSB.
} ADCAlign_TypeDef;

#define IS_ADC_DATA_ALIGN(DATA_ALIGN) (((DATA_ALIGN) == ADC_DATA_ALIGN_LSB) || ((DATA_ALIGN) == ADC_DATA_ALIGN_MSB)) //!< Check if the input parameter is valid.

/** End of ADC_Data_Align
  * \}
  */

/**
 * \defgroup    ADC_Raw_Data_Average ADC Raw Data Average
 * \{
 * \ingroup     ADC_Exported_Constants
 */
typedef enum
{
    ADC_DATA_AVERAGE_OF_2,    //!< 2 data used to calculate average.
    ADC_DATA_AVERAGE_OF_4,    //!< 4 data used to calculate average.
    ADC_DATA_AVERAGE_OF_8,    //!< 8 data used to calculate average.
    ADC_DATA_AVERAGE_OF_16,   //!< 16 data used to calculate average.
    ADC_DATA_AVERAGE_OF_32,   //!< 32 data used to calculate average.
    ADC_DATA_AVERAGE_OF_64,   //!< 64 data used to calculate average.
    ADC_DATA_AVERAGE_OF_128,  //!< 128 data used to calculate average.
    ADC_DATA_AVERAGE_OF_256,  //!< 256 data used to calculate average.
    ADC_DATA_AVERAGE_MAX,     //!< The largest enumeration value.
} ADCDataAvgSel_TypeDef;

#define IS_ADC_DATA_AVG_NUM(NUM) (((NUM) == ADC_DATA_AVERAGE_OF_2) || \
                                  ((NUM) == ADC_DATA_AVERAGE_OF_4) || \
                                  ((NUM) == ADC_DATA_AVERAGE_OF_8) || \
                                  ((NUM) == ADC_DATA_AVERAGE_OF_16) || \
                                  ((NUM) == ADC_DATA_AVERAGE_OF_32) || \
                                  ((NUM) == ADC_DATA_AVERAGE_OF_64) || \
                                  ((NUM) == ADC_DATA_AVERAGE_OF_128) || \
                                  ((NUM) == ADC_DATA_AVERAGE_OF_256)) //!< Check if the input parameter is valid.

/** End of ADC_Data_Average
  * \}
  */


/**
 * \defgroup    ADC_FIFO_Threshold ADC FIFO Threshold
 * \{
 * \ingroup     ADC_Exported_Constants
 */
#define IS_ADC_FIFO_THRESHOLD(THD) ((THD) <= 0x3F) //!< ADC FIFO threshold is between 0 and 0x3F.

/** End of ADC_FIFO_Threshold
  * \}
  */

/**
 * \defgroup    ADC_Burst_Size ADC Burst Size
 * \{
 * \ingroup     ADC_Exported_Constants
 */
#define IS_ADC_BURST_SIZE_CONFIG(CONFIG) ((CONFIG) <= 0x3F) //!< ADC burst size is between 0 and 0x3F.

/** End of ADC_Burst_Size
  * \}
  */

/**
 * \defgroup    ADC_Operation_Mode ADC Operation Mode
 * \{
 * \ingroup     ADC_Exported_Constants
 */
typedef enum
{
    ADC_CONTINUOUS_MODE, //!< ADC continuous mode.
    ADC_ONE_SHOT_MODE,   //!< ADC one shot mode.
} ADCOperationMode_TypeDef;

#define IS_ADC_MODE(MODE) (((MODE) == ADC_CONTINUOUS_MODE) || ((MODE) == ADC_ONE_SHOT_MODE)) //!< Check if the input parameter is valid.

/** End of ADC_Operation_Mode
  * \}
  */

/**
 * \defgroup    ADC_Power_Mode  ADC Power Mode
 * \{
 * \ingroup     ADC_Exported_Constants
 */
typedef enum
{
    ADC_POWER_ON_AUTO,     //!< The power mode of ADC is auto mode.
    ADC_POWER_ON_MANUAL,   //!< The power mode of ADC is manual mode.
} ADCPowerMode_TypeDef;

#define IS_ADC_POWER_MODE(MODE) (((MODE) == ADC_POWER_ON_AUTO) || ((MODE) == ADC_POWER_ON_MANUAL)) //!< Check if the input parameter is valid.

/** End of ADC_Power_Mode
  * \}
  */

/**
 * \defgroup    ADC_Interrupts_Definition ADC Interrupts Definition
 * \{
 * \ingroup     ADC_Exported_Constants
 */
#define ADC_INT_FIFO_RD_REQ           ((uint32_t)(1 << 0)) //!< ADC GDMA request interrupt. When FIFO data level achieve the GDMA threshold level (ADC_WaterLevel), this interrupt is triggered.
#define ADC_INT_FIFO_RD_ERR           ((uint32_t)(1 << 1)) //!< ADC FIFO read error interrupt. When read the empty FIFO, this interrupt is triggered.
#define ADC_INT_FIFO_THD              ((uint32_t)(1 << 2)) //!< ADC FIFO threshold interrupt. When FIFO data number is more than or equal to the threshold level (ADC_FifoThdLevel), this interrupt is triggered.
#if ADC_SUPPORT_INT_FIFO_FULL
#define ADC_INT_FIFO_FULL             ((uint32_t)(1 << 3)) //!< ADC FIFO full interrupt. When FIFO full, this interrupt is triggered.
#else
#define ADC_INT_FIFO_OVERFLOW         ((uint32_t)(1 << 3)) //!< ADC FIFO overflow interrupt. When FIFO overflow, this interrupt is triggered.
#endif
#define ADC_INT_ONE_SHOT_DONE         ((uint32_t)(1 << 4)) //!< ADC one shot mode done interrupt. When ADC conversion done, this interrupt is triggered.
#if ADC_SUPPORT_INT_FIFO_FULL
#define ADC_INT_FIFO_OVERFLOW         ((uint32_t)(1 << 5)) //!< ADC FIFO overflow interrupt. When FIFO overflow, this interrupt is triggered.
#endif

#if ADC_SUPPORT_INT_FIFO_FULL
#define IS_ADC_INT(INT) (((INT) == ADC_INT_FIFO_RD_REQ) || \
                         ((INT) == ADC_INT_FIFO_RD_ERR) || \
                         ((INT) == ADC_INT_FIFO_THD) || \
                         ((INT) == ADC_INT_FIFO_OVERFLOW) || \
                         ((INT) == ADC_INT_ONE_SHOT_DONE) || \
                         ((INT) == ADC_INT_FIFO_FULL))     //!< Check if the input parameter is valid.
#else
#define IS_ADC_INT(INT) (((INT) == ADC_INT_FIFO_RD_REQ) || \
                         ((INT) == ADC_INT_FIFO_RD_ERR) || \
                         ((INT) == ADC_INT_FIFO_THD) || \
                         ((INT) == ADC_INT_FIFO_OVERFLOW) || \
                         ((INT) == ADC_INT_ONE_SHOT_DONE)) //!< Check if the input parameter is valid.
#endif

/** End of ADC_Interrupts_Definition
  * \}
  */

#if (ADC_SUPPORT_RAP_FUNCTION == 1)

/**
 * \defgroup    ADC_Qactive_Force TIM Qactive Force
 * \{
 * \ingroup     ADC_Exported_Constants
 */
typedef enum
{
    ADC_QACTIVE_FW_SCLK_FORCE = 0x0,
    ADC_QACTIVE_FW_PCLK_FORCE = 0x1,
    ADC_QACTIVE_FW_PCLK_ICG = 0x1,
} ADCQactiveForce_TypeDef;

/** End of ADC_Qactive_Force
  * \}
  */

/**
 * \defgroup    ADC_Task TIM Task
 * \{
 * \ingroup     ADC_Exported_Constants
 */
typedef enum
{
    ADC_TASK_ONE_SHOT_SAMPLE = 0,
} ADCTask_TypeDef;

/** End of ADC_Task
  * \}
  */

#endif


/** End of ADC_Exported_Constants
  * \}
  */

/*============================================================================*
 *                         Types
 *============================================================================*/
/** \defgroup ADC_Exported_Types ADC Exported Types
  * \brief
  * \{
  */

/**
 * \brief       ADC init structure definition.
 *
 * \ingroup     ADC_Exported_Types
 */
typedef struct
{
    uint16_t ADC_SampleTime;                /*!< Specify the ADC Sample clock. (ADC_SampleTime+1) cycles of 10MHz.
                                                 This parameter can be a value of 19 to 16383. */

    ADCConvertTim_TypeDef ADC_ConvertTime;  /**< Specify the ADC Sample convert time.
                                                 This parameter can be a value of \ref ADC_Convert_Time. */

    FunctionalState ADC_DataWriteToFifo;    /*!< Write ADC one shot mode data into FIFO.
                                                 This parameter can be a value of DISABLE or ENABLE. */

    uint8_t ADC_FifoThdLevel;               /*!< Specify the ADC FIFO threshold to trigger interrupt ADC_INT_FIFO_TH.
                                                 This parameter can be a value of 0 to 31. */

    uint8_t ADC_WaterLevel;                 /*!< Specify the ADC FIFO Burst Size to trigger GDMA.
                                                 This parameter can be a value of 0 to 31. */

    FunctionalState ADC_FifoOverWriteEn;    /*!< Specify if over write FIFO when FIFO overflow.
                                                 This parameter can be a value of DISABLE or ENABLE. */

#if ADC_SUPPORT_DMA_EN
    FunctionalState ADC_DmaEn;              /*!< Enable or disable the DMA function of ADC.
                                                 This parameter can be a value of DISABLE or ENABLE. */
#endif

    uint16_t ADC_SchIndex[CHIP_ADC_SCHEDULE_NUM]; /*!< Specify ADC mode and channel for schedule table.*/

    uint32_t ADC_Bitmap;                    /*!< Specify the schedule table channel map.
                                                 This parameter can be a value of 16Bits map. */

    FunctionalState ADC_TimerTriggerEn;     /*!< Enable or disable ADC one-shot mode when tim7 toggles.
                                                 This parameter can be a value of DISABLE or ENABLE. */

    ADCAlign_TypeDef ADC_DataAlign;         /*!< ADC Data MSB or LSB aligned.
                                                 This parameter can be a value of \ref ADC_Data_Align. */

    FunctionalState ADC_DataMinusEn;        /**< Enable or disable function that adc data latched
                                                 minus the given offset before writes to reg/FIFO. */

    uint16_t ADC_DataMinusOffset;           /**< Offset to be minused from adc data latched.
                                                 This parameter can be a value of 0 to 4095. */

    ADCDataAvgSel_TypeDef ADC_DataAvgSel;   /**< Number of data for calculate average.
                                                 This parameter can be a value of \ref ADC_Raw_Data_Average. */

    uint8_t ADC_DataAvgEn;                  /**< Enable or disable the calculation for average result of the one shot data.
                                                 This parameter can be a value of DISABLE or ENABLE. */

    uint8_t ADC_DataLatchDly;               /*!< Specify specify the number of the beginning data from ADC output to be discarded. */

    FunctionalState ADC_FifoStopWriteEn;    /*!< Stop fifo from writing data. This bit will be asserted
                                                 automatically as fifo overflow, (not automatically when
                                                 ADC_FIFO_OVER_WRITE_ENABLE), need to be cleared in order
                                                 to write data again. This will not stop overwrite mode. */

    ADCPowerMode_TypeDef ADC_PowerOnMode;   /**< Specify ADC power on mode.
                                                 This parameter can be a value of \ref ADC_Power_Mode. */

    FunctionalState ADC_PowerAlwaysOnEn;    /*!< Enable or disable the power always on.
                                                 This parameter can be a value of DISABLE or ENABLE. */

#if ADC_SUPPORT_POWER_ON_DELAY
    FunctionalState ADC_PowerOnDlyEn;       /*!< Enable or disable ADC 8ms delay after adc power on.
                                                 This parameter can be a value of DISABLE or ENABLE. */
#endif
} ADC_InitTypeDef;

/** End of ADC_Exported_Types
  * \}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** \defgroup ADC_Exported_Functions ADC Exported Functions
  * \brief
  * \{
  */

/**
 * \brief   Deinitialize the ADC peripheral registers to their
 *          default reset values(turn off ADC clock).
 *
 * \param[in] ADCx  Specify ADC peripheral, can only be ADC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_adc_init(void)
 * {
 *     //Turn off the clock.
 *     ADC_DeInit(ADC);
 * }
 * \endcode
 */
void ADC_DeInit(ADC_TypeDef *ADCx);

/**
 * \brief Initialize the ADC peripheral according to the specified parameters in the ADC_InitStruct
 *
 * \param[in]  ADCx            Select ADC peripheral.
 * \param[in]  ADC_InitStruct  Pointer to a ADC_InitTypeDef structure which will be initialized.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_adc_init(void)
 * {
 *     //Turn on the clock.
 *     RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

 *     ADC_InitTypeDef ADC_InitStruct;
 *     ADC_StructInit(&ADC_InitStruct);
 *     ADC_InitStruct.ADC_SchIndex[0] = EXT_SINGLE_ENDED(0);
 *     ADC_InitStruct.ADC_SchIndex[1] = EXT_SINGLE_ENDED(1);
 *     ADC_InitStruct.ADC_Bitmap = 0x03;
 *     //Add other initialization parameters that need to be configured here.
 *     ADC_Init(ADC, &ADC_InitStruct);
 * }
 * \endcode
 */
void ADC_Init(ADC_TypeDef *ADCx, ADC_InitTypeDef *ADC_InitStruct);

/**
 * \brief   Fill each ADC_InitStruct member with its default value.
 *
 * \note   The default settings for the ADC_InitStruct member are shown in the following table:
 *         | ADC_InitStruct member   | Default value                   |
 *         |:-----------------------:|:-------------------------------:|
 *         | ADC_SampleTime          | 0x3E7                           |
 *         | ADC_ConvertTime         | \ref ADC_CONVERT_TIME_500NS     |
 *         | ADC_DataWriteToFifo     | DISABLE                         |
 *         | ADC_FifoThdLevel        | 0x06                            |
 *         | ADC_WaterLevel          | 0x1                             |
 *         | ADC_FifoOverWriteEn     | ENABLE                          |
 *         | ADC_SchIndex[16]        | 0                               |
 *         | ADC_Bitmap              | 0x0                             |
 *         | ADC_TimerTriggerEn      | DISABLE                         |
 *         | ADC_DataAlign           | \ref ADC_DATA_ALIGN_LSB         |
 *         | ADC_DataMinusEn         | DISABLE                         |
 *         | ADC_DataMinusOffset     | 0                               |
 *         | ADC_FifoStopWriteEn     | DISABLE                         |
 *         | ADC_DataAvgEn           | DISABLE                         |
 *         | ADC_DataAvgSel          | \ref ADC_DATA_AVERAGE_OF_2      |
 *         | ADC_PowerAlwaysOnEn     | DISABLE                         |
 *         | ADC_DataLatchDly        | 0x1                             |
 *         | ADC_PowerOnDlyEn        | DISABLE                         |
 *
 * \param[in] ADC_InitStruct  Pointer to an ADC_InitTypeDef structure which will be initialized.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_adc_init(void)
 * {
 *     //Turn on the clock.
 *     RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

 *     ADC_InitTypeDef ADC_InitStruct;
 *     ADC_StructInit(&ADC_InitStruct);
 *     ADC_InitStruct.ADC_SchIndex[0] = EXT_SINGLE_ENDED(0);
 *     ADC_InitStruct.ADC_SchIndex[1] = EXT_SINGLE_ENDED(1);
 *     ADC_InitStruct.ADC_Bitmap = 0x03;
 *     //Add other initialization parameters that need to be configured here.
 *     ADC_Init(ADC, &ADC_InitStruct);
 * }
 * \endcode
 * \callgraph
 *
 */
void ADC_StructInit(ADC_InitTypeDef *ADC_InitStruct);

/**
 * \brief   Enable or disable the ADC peripheral.
 *
 * \param[in] ADCx      Specify ADC peripheral.
 * \param[in] AdcMode   ADC operation mode selection.
 *                      This parameter can be one of the following values:
 *                      \arg ADC_ONE_SHOT_MODE: One shot mode.
 *                      \arg ADC_CONTINUOUS_MODE: Continuous sampling mode.
 * \param[in] NewState  New state of the ADC peripheral.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the specified ADC peripheral.
 *                      - DISABLE: Disable the specified ADC peripheral.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void board_adc_init(void)
 * {
 *     Pad_Config(P2_0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
 *                PAD_OUT_LOW);
 *
 *     Pad_Config(P2_1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
 *                PAD_OUT_LOW);
 * }
 *
 * void driver_adc_init(void)
 * {
 *     //open clock
 *     RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);
 *
 *     ADC_InitTypeDef ADC_InitStruct;
 *     ADC_StructInit(&ADC_InitStruct);
 *     ADC_InitStruct.ADC_SchIndex[0] = EXT_SINGLE_ENDED(0);
 *     ADC_InitStruct.ADC_SchIndex[1] = EXT_SINGLE_ENDED(1);
 *     ADC_InitStruct.ADC_Bitmap = 0x03;
 *     //Add other initialization parameters here.
 *     ADC_Init(ADC, &ADC_InitStruct);
 *
 *     ADC_INTConfig(ADC, ADC_INT_ONE_SHOT_DONE, ENABLE);
 * }
 *
 * void adc_demo(void)
 * {
 *     board_adc_init();
 *     driver_adc_init();
 *     ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);
 * }
 * \endcode
 */
void ADC_Cmd(ADC_TypeDef *ADCx, ADCOperationMode_TypeDef AdcMode, FunctionalState NewState);

/**
 * \brief   Enable or disable the specified ADC interrupts.
 *
 * \param[in] ADCx      Specify ADC peripheral.
 * \param[in] ADC_IT    Specify the ADC interrupts sources to be enabled or disabled.
 *                      This parameter can be any combination of the following values.
 *                      Refer to \ref ADC_Interrupts_Definition.
 *                      \arg ADC_INT_FIFO_RD_REQ : FIFO read request.
 *                      \arg ADC_INT_FIFO_RD_ERR : FIFO read error.
 *                      \arg ADC_INT_FIFO_THD : ADC FIFO size > thd.
 *                      \arg ADC_INT_FIFO_OVERFLOW : ADC FIFO overflow.
 *                      \arg ADC_INT_ONE_SHOT_DONE : ADC one shot mode done.
 * \param[in] NewState  New state of the specified ADC interrupt.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the specified ADC interrupts.
 *                      - DISABLE: Disable the specified ADC interrupts.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_adc_init(void)
 * {
 *     //open clock
 *     RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);
 *
 *     ADC_InitTypeDef ADC_InitStruct;
 *     ADC_StructInit(&ADC_InitStruct);
 *     ADC_InitStruct.ADC_SchIndex[0] = EXT_SINGLE_ENDED(0);
 *     ADC_InitStruct.ADC_SchIndex[1] = EXT_SINGLE_ENDED(1);
 *     ADC_InitStruct.ADC_Bitmap = 0x03;
 *     //Add other initialization parameters here.
 *     ADC_Init(ADC, &ADC_InitStruct);
 *
 *     ADC_INTConfig(ADC, ADC_INT_FIFO_RD_ERR, ENABLE);
 *     ADC_INTConfig(ADC, ADC_INT_ONE_SHOT_DONE, ENABLE);
 * }
 * \endcode
 *
 */
void ADC_INTConfig(ADC_TypeDef *ADCx, uint32_t ADC_INT, FunctionalState NewState);

/**
 * \brief      Read ADC data according to specific channel.
 *
 * \param[in]  ADCx   Specify ADC peripheral.
 * \param[in]  Index  Can be 0 to 15.
 *
 * \return     The 12-bit converted ADC raw data.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     board_adc_init();
 *     driver_adc_init();
 *     ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);
 *     while(ADC_GetINTStatus(ADC, ADC_INT_ONE_SHOT_DONE) == RESET);
 *     uint16_t raw_data_0 = ADC_ReadRawData(ADC, 0);
 *     uint16_t raw_data_1 = ADC_ReadRawData(ADC, 1);
 * }
 * \endcode
 */
uint16_t ADC_ReadRawData(ADC_TypeDef *ADCx, uint8_t Index);

/**
 * \brief   Get ADC average data from ADC schedule table0.
 *
 * \param[in]  ADCx  Specify ADC peripheral.
 *
 * \return  The 12-bit converted ADC raw data.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     board_adc_init();
 *     driver_adc_init();
 *     ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);
 *     while(ADC_GetINTStatus(ADC, ADC_INT_ONE_SHOT_DONE) == RESET);
 *     uint16_t raw_data = 0;
 *     raw_data = ADC_ReadAvgRawData(ADC);
 * }
 * \endcode
 *
 */
uint16_t ADC_ReadAvgRawData(ADC_TypeDef *ADCx);

/**
 * \brief  Read one byte data from ADC FIFO.
 *
 * \param[in]  ADCx  Select ADC peripheral.
 *
 * \return ADC FIFO data.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     board_adc_init();
 *     driver_adc_init();
 *     ADC_Cmd(ADC, ADC_ONE_SHOT_MODE, ENABLE);
 *     while(ADC_GetINTStatus(ADC, ADC_INT_ONE_SHOT_DONE) == RESET);
 *     uint16_t raw_data = 0;
 *     raw_data = ADC_ReadFIFO(ADC);
 * }
 * \endcode
 */
uint16_t ADC_ReadFIFO(ADC_TypeDef *ADCx);

/**
 * \brief   Get data from ADC FIFO.
 *
 * \param[in]  ADCx    Specify ADC peripheral.
 * \param[out] outBuf  Buffer to save data read from ADC FIFO.
 * \param[in]  Num     Number of data to be read.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     //ADC already start
 *     uint16_t raw_data[32] = {0};
 *     uint8_t data_len = ADC_GetFIFODataLen(ADC);
 *     ADC_ReadFIFOData(ADC,raw_data,data_len);
 * }
 * \endcode
 *
 */
void ADC_ReadFIFOData(ADC_TypeDef *ADCx, uint16_t *outBuf, uint16_t Num);

/**
 * \brief   Get ADC FIFO data number.
 *
 * \param[in] ADCx  Select ADC peripheral.
 *
 * \return  Current data number in ADC FIFO.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     //ADC already start
 *     uint16_t raw_data[32] = {0};
 *     uint8_t data_len = ADC_GetFIFODataLen(ADC);
 *     ADC_ReadFIFOData(ADC,raw_data,data_len);
 * }
 * \endcode
 *
 */
uint8_t ADC_GetFIFODataLen(ADC_TypeDef *ADCx);

/**
 * \brief   Config ADC schedule table.
 *
 * \param[in] ADCx     Specify ADC peripheral.
 * \param[in] AdcMode  ADC operation mode.
 *                     This parameter can be one of the following values:
 *                     \arg EXT_SINGLE_ENDED(index): Single-ended mode, the input is external channel index.
 *                     \arg EXT_DIFFERENTIAL(index): Differential mode, the positive input is external channel index,
 *                     the negative input external channel index1.
 *                     \arg INTERNAL_VBAT_MODE: The input is internal battery voltage detection channel.
 * \param[in] Index:   Schedule table index.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     ADC_SchIndexConfig(ADC,INTERNAL_VBAT_MODE,0);
 * }
 * \endcode
 *
 */
void ADC_SchIndexConfig(ADC_TypeDef *ADCx, uint8_t AdcMode, uint16_t Index);

/**
 * \brief   Config ADC bit map.
 *
 * \param[in]  ADCx      Specify ADC peripheral.
 * \param[in]  BitMap    ADC bit map.
 * \param[in]  NewState  New state of the ADC peripheral.
 *                       This parameter can be one of the following values:
 *                       - ENABLE: Enable ADC bit map.
 *                       - DISABLE: Disable ADC bit map.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     uint16_t bit_map = 0x03;
 *     ADC_BitMapConfig(ADC,bit_map);
 * }
 * \endcode
 */
void ADC_BitMapConfig(ADC_TypeDef *ADCx, uint16_t BitMap);

/**
 * \brief   Enbale or disable write data to FIFO.
 *
 * \param[in] ADCx      Specify ADC peripheral.
 * \param[in] NewState  New state of the ADC FIFO write.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the stop of FIFO writing data.
 *                      - DISABLE: Disable the stop of FIFO writing data.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     ADC_WriteFIFOCmd(ADC, ENABLE);
 * }
 * \endcode
 */
void ADC_WriteFIFOCmd(ADC_TypeDef *ADCx, FunctionalState NewState);

/**
 * \brief     Config ADC bypass resistor.
 *
 * \param[in] channelNum  External channel number, can be 0~7.
 * \param[in] NewState    Specify whether the channel enables bypass mode.
 *                        This parameter can be one of the following values:
 *                        - ENABLE: Enable ADC bypass function.
 *                        - DISABLE: Disable ADC bypass function.
 *
 * \attention The input voltage of channel pin using bypass mode cannot exceed 0.9V!
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     ADC_BypassCmd(0,ENABLE);
 * }
 * \endcode
 */
void ADC_BypassCmd(uint8_t ChannelNum, FunctionalState NewState);

/**
 * \brief  Get the specified ADC interrupt flag status.
 *
 * \param[in]  ADCx          Select ADC peripheral.
 * \param[in]  ADC_INT_FLAG  Specify the interrupt flag to check.
 *                           This parameter can be one of the following values.
 *                           Refer to \ref ADC_Interrupts_Definition.
 *                           \arg ADC_INT_ONE_SHOT_DONE: ADC once convert end interrupt.
 *                           \arg ADC_INT_FIFO_OVERFLOW: ADC FIFO overflow interrupt.
 *                           \arg ADC_INT_FIFO_THD: FIFO larger than threshold interrupt.
 *                           \arg ADC_INT_FIFO_RD_ERR: ADC read FIFO error interrupt.
 *                           \arg ADC_INT_FIFO_RD_REQ: ADC read FIFO request interrupt.
 *
 * \return The status of ADC interrupt.
 *         - SET: The ADC interrupt status is set.
 *         - RESET: The ADC interrupt status is reset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     ITStatus int_status = RESET;
 *     int_status = ADC_GetINTStatus(ADC,ADC_INT_FIFO_OVERFLOW);
 * }
 * \endcode
 */
ITStatus ADC_GetINTStatus(ADC_TypeDef *ADCx, uint32_t ADC_INT);

/**
 * \brief  Clear the ADC interrupt pending bit.
 *
 * \param[in] ADCx     Specify ADC peripheral.
 * \param[in] ADC_INT  Specify the interrupt pending bit to clear.
 *                     This parameter can be any combination of the following values.
 *                     Refer to \ref ADC_Interrupts_Definition.
 *                     - ADC_INT_ONE_SHOT_DONE: ADC once convert end interrupt.
 *                     - ADC_INT_FIFO_OVERFLOW: ADC FIFO overflow interrupt.
 *                     - ADC_INT_FIFO_THD: FIFO larger than threshold interrupt.
 *                     - ADC_INT_FIFO_RD_ERR: ADC read FIFO error interrupt.
 *                     - ADC_INT_FIFO_RD_REQ: ADC read FIFO request interrupt.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     ADC_ClearINTPendingBit(ADC,ADC_INT_FIFO_OVERFLOW);
 * }
 * \endcode
 */
void ADC_ClearINTPendingBit(ADC_TypeDef *ADCx, uint32_t ADC_INT);

/**
 * \brief   Clear ADC FIFO.
 *
 * \param[in] ADCx  Specify ADC peripheral.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     ADC_ClearFIFO(ADC);
 * }
 * \endcode
 */
void ADC_ClearFIFO(ADC_TypeDef *ADCx);

/**
 * \brief   Get all adc interrupt flag status.
 *
 * \param[in] ADCx  Specify ADC peripheral.
 *
 * \return  All ADC interrupt status.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void adc_demo(void)
 * {
 *     uint8_t all_flag_status = 0;
 *     all_flag_status = ADC_GetAllFlagStatus(ADC);
 * }
 * \endcode
 *
 */
uint8_t ADC_GetAllFlagStatus(ADC_TypeDef *ADCx);

/**
 * \brief  Get the index state of ADC controller.
 *
 * \param[in]  ADCx  Specify ADC peripheral.
 *
 * \return BIT[15:0] stores the data in the FIFO, BIT[31:28] stores the index of the data.
 */
uint32_t ADC_ReadScheduleIndexandFifoData(ADC_TypeDef *ADCx);

#if (ADC_SUPPORT_RAP_FUNCTION == 1)

void ADC_RAPModeCmd(ADC_TypeDef *ADCx, FunctionalState NewState);

void ADC_RAPQactiveCtrl(ADC_TypeDef *ADCx, uint32_t Qactive, FunctionalState NewState);

void ADC_TaskTrigger(ADC_TypeDef *ADCx, uint32_t Task);

#endif

/** End of ADC_Exported_Functions
  * \}
  */

/** End of ADC
  * \}
  */

#ifdef __cplusplus
}
#endif

#endif /* RTL_ADC_H */
