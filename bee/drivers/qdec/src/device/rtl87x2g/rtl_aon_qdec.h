/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef RTL_AON_QDEC_H
#define RTL_AON_QDEC_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "utils/rtl_utils.h"
#include "rtl_aon_qdec_def.h"

/** \defgroup AON_QDEC    AON_QDEC
  * \brief
  * \{
  */

/*============================================================================*
 *                         Constants
 *============================================================================*/
/** \defgroup AON_QDEC_Exported_Constants AON_QDEC Exported Constants
  * \brief
  * \{
  */

/**
 * \defgroup    AON_QDEC_Counter_Scale AON_QDEC Counter Scale
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
typedef enum
{
    CounterScale_1_Phase = 0x00,     //!< AON_QDEC update counter when 1 phase change.
    CounterScale_2_Phase = 0x01,     //!< AON_QDEC update counter when 2 phases change.
} AONQDECAxisCntScale_TypeDef;

#define IS_CNT_SCALE_PHASE_TYPE(TYPE) ((TYPE) <= 0x01) //!< Check if the input parameter is valid.

/** End of AON_QDEC_Counter_Scale
  * \}
  */

/**
 * \defgroup    AON_QDEC_Init_Phase AON_QDEC Init Phase
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
typedef enum
{
    phaseMode0 = 0x00,     //!< AON_QDEC initial phase 00.
    phaseMode1 = 0x01,     //!< AON_QDEC initial phase 01.
    phaseMode2 = 0x02,     //!< AON_QDEC initial phase 10.
    phaseMode3 = 0x03,     //!< AON_QDEC initial phase 11.
} AONQDECInitPhase_TypeDef;

#define IS_PHSAE_MODE_TYPE(TYPE) ((TYPE) <= 0x03) //!< Check if the input parameter is valid.

/** End of AON_QDEC_Init_Phase
  * \}
  */

/**
 * \defgroup    AON_QDEC_Axis_Direction AON_QDEC Axis Direction
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
typedef enum
{
    AON_QDEC_AXIS_DIR_DOWN = 0x00,     //!< The direction for x-axis is decreasing.
    AON_QDEC_AXIS_DIR_UP = 0x01,       //!< The direction for x-axis is increasing.
} AONQDECAxisDir_TypeDef;

#define IS_AON_QDEC_AXIS_DIR_TYPE(TYPE) ((TYPE) <= 0x01) //!< Check if the input parameter is valid.

/** End of AON_QDEC_Axis_Direction
  * \}
  */

/**
 * \defgroup    AON_QDEC_Interrupts AON_QDEC Interrupts
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
#define AON_QDEC_X_INT_NEW_DATA                     BIT0     //!< The counter interrupt triggered by new data on x-axis.
#define AON_QDEC_X_INT_ILLEAGE                      BIT1     //!< The illegal interrupt on x-axis.

#define IS_AON_QDEC_INT_CONFIG(CONFIG) (((CONFIG) == AON_QDEC_X_INT_NEW_DATA) || ((CONFIG) == AON_QDEC_X_INT_ILLEAGE)) //!< Check if the input parameter is valid.

/** End of AON_QDEC_Interrupts
  * \}
  */

/**
 * \defgroup    AON_QDEC_Interrupts_Mask AON_QDEC Interrupts Mask
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
#define AON_QDEC_X_INT_MASK                         BIT24     //!< AON_QDEC interrupt mask.
#define AON_QDEC_X_WAKE_AON_MASK                    BIT23     //!< AON_QDEC wakeup interrupt mask.
#define AON_QDEC_X_CT_INT_MASK                      BIT8      //!< AON_QDEC counter interrupt mask triggered by new data.
#define AON_QDEC_X_ILLEAGE_INT_MASK                 BIT7      //!< AON_QDEC illegal interrupt mask.

#define IS_AON_QDEC_INT_MASK_CONFIG(CONFIG) (((CONFIG) == AON_QDEC_X_CT_INT_MASK) || ((CONFIG) == AON_QDEC_X_ILLEAGE_INT_MASK)) //!< Check if the input parameter is valid.

/** End of AON_QDEC_Interrupts_Mask
  * \}
  */

/**
 * \defgroup    AON_QDEC_Clr_Flag AON_QDEC Clr Flag
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
#define AON_QDEC_CLR_ILLEGAL_CT_X                   BIT5    //!< Clear the counter of illegal interrupts.
#define AON_QDEC_CLR_ILLEGAL_INT_X                  BIT4    //!< Clear the illegal interrupt flag.
#define AON_QDEC_CLR_UNDERFLOW_X                    BIT3    //!< Clear the underflow flag.
#define AON_QDEC_CLR_OVERFLOW_X                     BIT2    //!< Clear the overflow flag.
#define AON_QDEC_CLR_NEW_CT_X                       BIT1    //!< Clear the new data flag.
#define AON_QDEC_CLR_ACC_CT_X                       BIT0    //!< Clear the ACC counter flag.

#define IS_AON_QDEC_INT_CLR_CONFIG(CONFIG) (((CONFIG) == AON_QDEC_CLR_ACC_CT_X) ||  ((CONFIG) == AON_QDEC_CLR_ILLEGAL_INT_X)\
                                            || ((CONFIG) == AON_QDEC_CLR_UNDERFLOW_X) ||((CONFIG) == AON_QDEC_CLR_OVERFLOW_X)\
                                            || ((CONFIG) == AON_QDEC_CLR_NEW_CT_X)) //!< Check if the input parameter is valid.

/** End of AON_QDEC_Clr_Flag
  * \}
  */

/**
 * \defgroup    AON_QDEC_Flag AON_QDEC Flag
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
#define AON_QDEC_FLAG_NEW_CT_STATUS_X               BIT0    //!< The new data flag.
#define AON_QDEC_FLAG_OVERFLOW_X                    BIT1    //!< The overflow flag.
#define AON_QDEC_FLAG_UNDERFLOW_X                   BIT2    //!< The underflow flag.
#define AON_QDEC_FLAG_ILLEGAL_STATUS_X              BIT4    //!< The illegal counting flag.

#define IS_AON_QDEC_CLR_INT_STATUS(INT) (((INT) == AON_QDEC_FLAG_ILLEGAL_STATUS_X) || ((INT) == AON_QDEC_FLAG_NEW_CT_STATUS_X)\
                                         || ((INT) == AON_QDEC_FLAG_OVERFLOW_X) || ((INT) == AON_QDEC_FLAG_UNDERFLOW_X)\
                                         || ((INT) == AON_QDEC_FLAG_AUTO_STATUS_X)) //!< Check if the input parameter is valid.

/** End of AON_QDEC_Flag
  * \}
  */

/**
 * \defgroup    AON_QDEC_Axis AON_QDEC Axis
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
#define AON_QDEC_AXIS_X                             BIT0    //!< The AON_QDEC X axis.

/** End of AON_QDEC_Axis
  * \}
  */

/**
 * \defgroup    AON_QDEC_Immediate_Number AON_QDEC Immediate Number
 * \{
 * \ingroup     AON_QDEC_Exported_Constants
 */
#define AON_QDEC_0X04_CNT_DIR           BIT20    //!< The AON_QDEC x-axis direction.
#define AON_QDEC_0X00_AXIS_EN           BIT31    //!< The AON_QDEC x-axis enable bit.

/** End of AON_QDEC_Immediate_Number
  * \}
  */

/** End of AON_QDEC_Exported_Constants
  * \}
  */

/*============================================================================*
 *                         Types
 *============================================================================*/
/** \defgroup AON_QDEC_Exported_Types AON_QDEC Exported Types
  * \brief
  * \{
  */

/**
 * \brief       AON_QDEC init structure definition.
 *
 * \ingroup     AON_QDEC_Exported_Types
 */
typedef struct
{
    FunctionalState axisConfigX;               /*!< Specify the x-axis function.
                                                    This parameter can be a value of ENABLE or DISABLE. */

    FunctionalState manualLoadInitPhase;       /*!< Specify manual-load initial phase function.
                                                    This parameter can be a value of ENABLE or DISABLE. */

    AONQDECAxisCntScale_TypeDef counterScaleX; /*!< Specify the x-axis counter scale.
                                                    This parameter can be a value of \ref AON_QDEC_Counter_Scale */

    FunctionalState debounceEnableX;           /*!< Specify the x-axis debounce function.
                                                    This parameter can be a value of ENABLE or DISABLE. */

    uint16_t debounceTimeX;                    /*!< Specify the x-axis debounce time.
                                                    This parameter can be a value of 0 - 8191 */

    AONQDECInitPhase_TypeDef initPhaseX;       /*!< Specify the x-axis initial phase.
                                                    This parameter can be a value of \ref AON_QDEC_Init_Phase */
} AON_QDEC_InitTypeDef;

/** End of AON_QDEC_Exported_Types
  * \}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** \defgroup AON_QDEC_Exported_Functions AON_QDEC Exported Functions
  * \brief
  * \{
  */

/**
  * \brief  Reset AON QDEC.
  *
  * \param[in]  None
  */
void AON_QDEC_DeInit(void);

/**
 * \brief   Initialize the AON_QDEC peripheral according to the specified
 *          parameters in the AON_QDEC_InitStruct
 *
 * \param[in]  AON_QDECx            Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in]  AON_QDEC_InitStruct  Pointer to a AON_QDEC_InitStruct structure which will be initialized.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_aon_qdec_init(void)
 * {
 *
 *     AON_QDEC_InitTypeDef AON_QDEC_InitStruct;
 *     AON_QDEC_StructInit(&AON_QDEC_InitStruct);
 *     AON_QDEC_InitStruct.axisConfigX       = ENABLE;
 *     AON_QDEC_InitStruct.debounceEnableX   = Debounce_Enable;
 *     AON_QDEC_Init(AON_QDEC, &AON_QDEC_InitStruct);
 *
 *     AON_QDEC_Cmd(AON_QDEC, AON_QDEC_AXIS_X, ENABLE);
 * }
 * \endcode
 */
void AON_QDEC_Init(AON_QDEC_TypeDef *AON_QDECx, AON_QDEC_InitTypeDef *AON_QDEC_InitStruct);

/**
 * \brief  Fill each AON_QDEC_InitStruct member with its default value.
 *
 * \note The default settings for the AON_QDEC_InitStruct member are shown in the following table:
 *       | AON_QDEC_InitStruct member   | Default value                        |
 *       |:----------------------------:|:------------------------------------:|
 *       | debounceTimeX                | 32 * 5                               |
 *       | counterScaleX                | \ref CounterScale_1_Phase            |
 *       | debounceEnableX              | DISABLE                              |
 *       | manualLoadInitPhase          | DISABLE                              |
 *       | initPhaseX                   | \ref phaseMode0                      |
 *
 * \param[in]  AON_QDEC_InitStruct  Pointer to an AON_QDEC_InitStruct structure which will be initialized.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_aon_qdec_init(void)
 * {
 *
 *     AON_QDEC_InitTypeDef AON_QDEC_InitStruct;
 *     AON_QDEC_StructInit(&AON_QDEC_InitStruct);
 *     AON_QDEC_InitStruct.axisConfigX       = ENABLE;
 *     AON_QDEC_InitStruct.debounceEnableX   = Debounce_Enable;
 *     AON_QDEC_Init(AON_QDEC, &AON_QDEC_InitStruct);
 *
 *     AON_QDEC_Cmd(AON_QDEC, AON_QDEC_AXIS_X, ENABLE);
 * }
 * \endcode
 */
void AON_QDEC_StructInit(AON_QDEC_InitTypeDef *AON_QDEC_InitStruct);

/**
 * \brief  Enable or disable the specified AON_QDEC interrupt source.
 *
 * \param[in] AON_QDECx    Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in] AON_QDEC_IT  Specify the AON_QDEC interrupts sources to be enabled or disabled.
 *                         This parameter can be one of the following values:
 *                         \arg AON_QDEC_X_INT_NEW_DATA: The counter interrupt for X axis.
 *                         \arg AON_QDEC_X_INT_ILLEAGE: The illegal interrupt for X axis.
 * \param[in] NewState     New state of the specified AON_QDEC interrupt.
 *                         This parameter can be one of the following values:
 *                         - ENABLE: Enable the interrupt of AON_QDEC.
 *                         - DISABLE: Disable the interrupt of AON_QDEC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_aon_qdec_init(void)
 * {
 *     AON_QDEC_INTConfig(AON_QDEC, AON_QDEC_X_INT_NEW_DATA, ENABLE);
 * }
 * \endcode
 */
void AON_QDEC_INTConfig(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_IT,
                        FunctionalState NewState);

/**
 * \brief  Get the specified AON_QDEC flag status.
 *
 * \param[in] AON_QDECx      Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in] AON_QDEC_FLAG  Specify the flag to check.
 *                           This parameter can be one of the following values:
 *                           \arg AON_QDEC_FLAG_NEW_CT_STATUS_X: Status of the counter interrupt for X axis.
 *                           \arg AON_QDEC_FLAG_ILLEGAL_STATUS_X: Status of the illegal interrupt for X axis.
 *                           \arg AON_QDEC_FLAG_OVERFLOW_X: The overflow flag for x-axis accumulation counter.
 *                           \arg AON_QDEC_FLAG_UNDERFLOW_X: The underflow flag for x-axis accumulation counter.
 *
 * \return  The new state of AON_QDEC_FLAG.
 *          - SET: The AON_QDEC flag has been set.
 *          - RESET: The AON_QDEC flag has been reset.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void aon_qdec_demo(void)
 * {
 *     FlagStatus flag_status = AON_QDEC_GetFlagState(AON_QDEC, AON_QDEC_FLAG_NEW_CT_STATUS_X);
 * }
 * \endcode
 */
FlagStatus AON_QDEC_GetFlagState(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_FLAG);

/**
 * \brief  Mask or unmask the specified AON_QDEC axis interrupts.
 *
 * \param[in] AON_QDECx      Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in] AON_QDEC_AXIS  Specify the AON_QDEC interrupt mask.
 *                           This parameter can be one or logical OR of the following values:
 *                           \arg AON_QDEC_X_CT_INT_MASK: The x-axis counter interrupt mask.
 *                           \arg AON_QDEC_X_ILLEAGE_INT_MASK: The x-axis illegal interrupt mask.
 * \param[in] NewState       New state of the specified AON_QDEC interrupts mask.
 *                           This parameter can be one of the following values:
 *                           - ENABLE: Enable the interrupt mask of AON_QDEC.
 *                           - DISABLE: Disable the interrupt mask of AON_QDEC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void aon_qdec_demo(void)
 * {
 *
 *     AON_QDEC_INTMask(AON_QDEC, AON_QDEC_X_CT_INT_MASK, ENABLE);
 *
 * }
 * \endcode
 */
void AON_QDEC_INTMask(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_AXIS,
                      FunctionalState NewState);

/**
 * \brief  Enable or disable the selected AON_QDEC axis x.
 *
 * \param[in] AON_QDECx      Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in] AON_QDEC_AXIS  Specify the AON_QDEC axis.
 *                           This parameter can be one of the following values:
 *                           \arg AON_QDEC_AXIS_X: The AON_QDEC X axis.
 * \param[in] NewState       New state of the selected AON_QDEC axis.
 *                           This parameter can be one of the following values:
 *                           - ENABLE: Count value changes upon detecting a phase change.
 *                           - DISABLE: AON_QDEC stops working.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void aon_qdec_demo(void)
 * {
 *     AON_QDEC_Cmd(AON_QDEC, AON_QDEC_AXIS_X, ENABLE);
 * }
 * \endcode
 */
void AON_QDEC_Cmd(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_AXIS,
                  FunctionalState NewState);

/**
 * \brief   Clear AON_QDEC interrupt pending bit.
 *
 * \param[in] AON_QDECx      Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in] AON_QDEC_FLAG  Specify the flag to clear.
 *                           This parameter can be one of the following values.
 *                           Refer to \ref AON_QDEC_Clr_Flag.
 *                           - AON_QDEC_CLR_OVERFLOW_X: The overflow flag for x-axis accumulation counter.
 *                           - AON_QDEC_CLR_ILLEGAL_INT_X: The illegal interrupt for X axis.
 *                           - AON_QDEC_CLR_UNDERFLOW_X: The underflow flag for x-axis accumulation counter.
 *                           - AON_QDEC_CLR_NEW_CT_X: The counter interrupt for X axis.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void aon_qdec_demo(void)
 * {
 *     AON_QDEC_ClearINTPendingBit(AON_QDEC, AON_QDEC_CLR_OVERFLOW_X);
 * }
 * \endcode
 */

void AON_QDEC_ClearINTPendingBit(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_CLR_INT);
/**
 * \brief  Get AON_QDEC Axis direction.
 *
 * \param[in] AON_QDECx      Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in] AON_QDEC_AXIS  Specify the AON_QDEC axis.
 *                            This parameter can be one of the following values:
 *                            \arg  AON_QDEC_AXIS_X: The AON_QDEC X axis.
 *
 * \return The direction of the axis.
 *         - AON_QDEC_AXIS_DIR_UP: The axis is rolling up.
 *         - AON_QDEC_AXIS_DIR_DOWN: The axis is rolling down.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void aon_qdec_demo(void)
 * {
 *     uint16_t dir = AON_QDEC_GetAxisDirection(AON_QDEC, AON_QDEC_AXIS_X);
 * }
 * \endcode
 */
uint16_t AON_QDEC_GetAxisDirection(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_AXIS);

/**
 * \brief  Get AON_QDEC Axis(x) count.
 *
 * \param[in] AON_QDECx      Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in] AON_QDEC_AXIS  Specify the AON_QDEC axis.
 *                           This parameter can be one of the following values:
 *                           - AON_QDEC_AXIS_X: The AON_QDEC X axis.
 *
 * \return The count of the axis.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void aon_qdec_demo(void)
 * {
 *     uint16_t counter = AON_QDEC_GetAxisCount(AON_QDEC, AON_QDEC_AXIS_X);
 * }
 * \endcode
 */
uint16_t AON_QDEC_GetAxisCount(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_AXIS);

/**
 * \brief  Pause or resume AON_QDEC Axis x counter.
 *
 * \param[in] AON_QDECx      Selected AON_QDEC peripheral. Refer to \ref AON_QDEC_Declaration.
 * \param[in] AON_QDEC_AXIS  Specify the AON_QDEC axis.
 *                           This parameter can be one of the following values:
 *                           \arg AON_QDEC_AXIS_X: The AON_QDEC X axis.
 * \param[in] NewState       New state of the specified AON_QDEC Axis.
 *                           This parameter can be one of the following values:
 *                           - ENABLE: Pause the X-axis of the AON_QDEC counter.
 *                           - DISABLE: Resume the X-axis of the AON_QDEC counter.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void aon_qdec_demo(void)
 * {
 *     AON_QDEC_CounterPauseCmd(AON_QDEC, AON_QDEC_AXIS_X, ENABLE);
 * }
 * \endcode
 */
void AON_QDEC_CounterPauseCmd(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_AXIS,
                              FunctionalState NewState);

/** End of AON_QDEC_Exported_Functions
  * \}
  */

/** End of AON_QDEC
  * \}
  */

#ifdef __cplusplus
}
#endif

#endif /* RTL_AON_QDEC_H */



