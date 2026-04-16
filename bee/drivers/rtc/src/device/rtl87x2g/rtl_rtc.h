/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef RTL_RTC_H
#define RTL_RTC_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "utils/rtl_utils.h"
#include "rtl_rtc_def.h"

/** \defgroup RTC         RTC
  * \brief
  * \{
  */

/*============================================================================*
 *                         Constants
 *============================================================================*/
/** \defgroup RTC_Exported_Constants RTC Exported Constants
  * \brief
  * \{
  */

/**
 * \defgroup    RTC_Comparator_Index RTC Comparator Index
 * \{
 * \ingroup     RTC_Exported_Constants
 */

typedef enum
{
    RTC_COMP0 = 0x00,     //!< RTC comparator index0.
    RTC_COMP1 = 0x01,     //!< RTC comparator index1.
    RTC_COMP2 = 0x02,     //!< RTC comparator index2.
    RTC_COMP3 = 0x03,     //!< RTC comparator index3.
} RTCComIndex_TypeDef;

#define IS_RTC_COMP(COMP) (((COMP) == RTC_COMP0) || \
                           ((COMP) == RTC_COMP1) || \
                           ((COMP) == RTC_COMP2) || \
                           ((COMP) == RTC_COMP3)) //!< Check if the input parameter is valid.

/** End of RTC_Comparator_Index
  * \}
  */

#if (RTC_SUPPORT_COMPARE_GUARDTIME == 1)
/**
 * \defgroup    RTC_ComparatorGT_Index RTC ComparatorGT Index
 * \{
 * \ingroup     RTC_Exported_Constants
 */

typedef enum
{
    RTC_COMP0GT = 0x0,     //!< RTC comparator GT index0.
    RTC_COMP1GT = 0x1,     //!< RTC comparator GT index1.
    RTC_COMP2GT = 0x2,     //!< RTC comparator GT index2.
    RTC_COMP3GT = 0x3,     //!< RTC comparator GT index3.
} RTCCmopGTIndex_TypeDef;

#define IS_RTC_COMPGT(COMP) (((COMP) == RTC_COMP0GT) || \
                             ((COMP) == RTC_COMP1GT) || \
                             ((COMP) == RTC_COMP2GT) || \
                             ((COMP) == RTC_COMP3GT)) //!< Check if the input parameter is valid.

/** End of RTC_ComparatorGT_Index
  * \}
  */
#endif

/**
 * \defgroup    RTC_Interrupts RTC Interrupts
 * \{
 * \ingroup     RTC_Exported_Constants
 */
#define RTC_INT_TICK           BIT8     //!< RTC tick interrupt.
#define RTC_INT_OVF            BIT9     //!< RTC overflow interrupt.
#define RTC_INT_PRE_COMP       BIT10    //!< RTC PRECOMP interrupt.
#define RTC_INT_PRE_COMP3      BIT11    //!< RTC PRECOMP&CMP3 interrupt.
#define RTC_INT_COMP0          BIT16    //!< RTC CMP0 interrupt.
#define RTC_INT_COMP1          BIT17    //!< RTC CMP1 interrupt.
#define RTC_INT_COMP2          BIT18    //!< RTC CMP2 interrupt.
#define RTC_INT_COMP3          BIT19    //!< RTC CMP3 interrupt.

#define IS_RTC_INT(INT) (((INT) == RTC_INT_TICK) || \
                         ((INT) == RTC_INT_OVF) || \
                         ((INT) == RTC_INT_COMP0) || \
                         ((INT) == RTC_INT_COMP1) || \
                         ((INT) == RTC_INT_COMP2) || \
                         ((INT) == RTC_INT_COMP3) || \
                         ((INT) == RTC_INT_PRE_COMP) || \
                         ((INT) == RTC_INT_PRE_COMP3)) //!< Check if the input parameter is valid.

/** End of RTC_Interrupts
  * \}
  */

/**
 * \defgroup    RTC_Wakeup_Interrupts RTC Wakeup Interrupts
 * \{
 * \ingroup     RTC_Exported_Constants
 */
#if (RTC_SUPPORT_PRE_COMP_OVF_TICK_WAKE_UP == 1)
#define RTC_WK_TICK            BIT8     //!< RTC tick wakeup interrupt.
#define RTC_WK_OVF             BIT9     //!< RTC overflow wakeup interrupt. 
#define RTC_WK_PRE_COMP        BIT10    //!< RTC PRECOMP wakeup interrupt.
#define RTC_WK_PRE_COMP3       BIT11    //!< RTC PRECOMP&CMP3 wakeup interrupt.

#define IS_RTC_WK_PRECOMP_OVF_TICK(WK) (((WK) == RTC_WK_TICK) || \
                                        ((WK) == RTC_WK_OVF) || \
                                        ((WK) == RTC_WK_PRE_COMP) || \
                                        ((WK) == RTC_WK_PRE_COMP3)) //!< Check if the input parameter is valid.
#else
#define IS_RTC_WK_PRECOMP_OVF_TICK(WK) (0) //!< Check if the input parameter is valid.
#endif

#if (RTC_SUPPORT_COMPARE_GUARDTIME == 1)
#define RTC_WK_COMP0GT         BIT12     //!< RTC CMP0 GT wakeup interrupt.
#define RTC_WK_COMP1GT         BIT13     //!< RTC CMP1 GT wakeup interrupt.
#define RTC_WK_COMP2GT         BIT14     //!< RTC CMP2 GT wakeup interrupt.
#define RTC_WK_COMP3GT         BIT15     //!< RTC CMP3 GT wakeup interrupt.

#define IS_RTC_WK_CMPGT(WK) (((WK) == RTC_WK_COMP0GT) || \
                             ((WK) == RTC_WK_COMP1GT) || \
                             ((WK) == RTC_WK_COMP2GT) || \
                             ((WK) == RTC_WK_COMP3GT)) //!< Check if the input parameter is valid.
#else
#define IS_RTC_WK_CMPGT(WK) (0) //!< Check if the input parameter is valid.
#endif

#define RTC_WK_COMP0           BIT20     //!< RTC CMP0 wakeup interrupt.
#define RTC_WK_COMP1           BIT21     //!< RTC CMP1 wakeup interrupt.
#define RTC_WK_COMP2           BIT22     //!< RTC CMP2 wakeup interrupt.
#define RTC_WK_COMP3           BIT23     //!< RTC CMP3 wakeup interrupt.

#define IS_RTC_WK(WK)         (((WK) == RTC_WK_COMP0) || \
                               ((WK) == RTC_WK_COMP1) || \
                               ((WK) == RTC_WK_COMP2) || \
                               ((WK) == RTC_WK_COMP3) || \
                               (IS_RTC_WK_CMPGT(WK)) || \
                               (IS_RTC_WK_PRECOMP_OVF_TICK(WK))) //!< Check if the input parameter is valid.

/** End of RTC_Wakeup_Interrupts
  * \}
  */

/**
 * \defgroup    RTC_Interrupt_Clear RTC Interrupt Clear
 * \{
 * \ingroup     RTC_Exported_Constants
 */
#define RTC_COMP3_CLR               (RTC_INT_COMP3     >> 8)    //!< Clear interrupt status of CMP3.
#define RTC_COMP2_CLR               (RTC_INT_COMP2     >> 8)    //!< Clear interrupt status of CMP2.
#define RTC_COMP1_CLR               (RTC_INT_COMP1     >> 8)    //!< Clear interrupt status of CMP1.
#define RTC_COMP0_CLR               (RTC_INT_COMP0     >> 8)    //!< Clear interrupt status of CMP0.
#define RTC_PRE_COMP3_CLR           (RTC_INT_PRE_COMP3 >> 8)    //!< Clear interrupt status of PRECOMP&CMP3.
#define RTC_PRE_COMP_CLR            (RTC_INT_PRE_COMP  >> 8)    //!< Clear interrupt status of PRECOMP.
#define RTC_OVERFLOW_CLR            (RTC_INT_OVF       >> 8)    //!< Clear interrupt status of overflow.
#define RTC_TICK_CLR                (RTC_INT_TICK      >> 8)    //!< Clear interrupt status of tick.

#define RTC_ALL_INT_CLR             (RTC_PRE_COMP3_CLR | \
                                     RTC_PRE_COMP_CLR | \
                                     RTC_COMP3_CLR | \
                                     RTC_COMP2_CLR | \
                                     RTC_COMP1_CLR | \
                                     RTC_COMP0_CLR | \
                                     RTC_OVERFLOW_CLR | \
                                     RTC_TICK_CLR) //!< Check if the input parameter is valid.

/** End of RTC_Interrupt_Clear
  * \}
  */

/**
 * \defgroup    RTC_Wakeup_Clear RTC Wakeup Clear
 * \{
 * \ingroup     RTC_Exported_Constants
 */
#if (RTC_SUPPORT_COMPARE_GUARDTIME == 1)
#define RTC_COMP3GT_CLR             (RTC_WK_COMP3GT >> 8)    //!< Clear interrupt status of CMP3GT.
#define RTC_COMP2GT_CLR             (RTC_WK_COMP2GT >> 8)    //!< Clear interrupt status of CMP2GT.
#define RTC_COMP1GT_CLR             (RTC_WK_COMP1GT >> 8)    //!< Clear interrupt status of CMP1GT.
#define RTC_COMP0GT_CLR             (RTC_WK_COMP0GT >> 8)    //!< Clear interrupt status of CMP0GT.

#define RTC_COMPGT_WAKEUP_CLR       (RTC_COMP3GT_CLR | \
                                     RTC_COMP2GT_CLR | \
                                     RTC_COMP1GT_CLR | \
                                     RTC_COMP0GT_CLR) //!< Check if the input parameter is valid.
#else
#define RTC_COMPGT_WAKEUP_CLR       (0) //!< Check if the input parameter is valid.
#endif

#define RTC_COMP3_WK_CLR            (RTC_WK_COMP3   >> 8)    //!< Clear interrupt status of CMP3 wakeup.
#define RTC_COMP2_WK_CLR            (RTC_WK_COMP2   >> 8)    //!< Clear interrupt status of CMP2 wakeup.
#define RTC_COMP1_WK_CLR            (RTC_WK_COMP1   >> 8)    //!< Clear interrupt status of CMP1 wakeup.
#define RTC_COMP0_WK_CLR            (RTC_WK_COMP0   >> 8)    //!< Clear interrupt status of CMP0 wakeup.

#define RTC_ALL_WAKEUP_CLR          (RTC_COMP3_WK_CLR | \
                                     RTC_COMP2_WK_CLR | \
                                     RTC_COMP1_WK_CLR | \
                                     RTC_COMP0_WK_CLR | \
                                     RTC_COMPGT_WAKEUP_CLR) //!< Check if the input parameter is valid.

/** End of RTC_Wakeup_Clear
  * \}
  */

#if (RTC_SUPPORT_RAP_FUNCTION == 1)
/**
 * \defgroup    RTC_Task RTC Task
 * \{
 * \ingroup     RTC_Exported_Constants
 */
typedef enum
{
    RTC_TASK_START = 0,
    RTC_TASK_STOP  = 1,
    RTC_TASK_CLR = 2,
} RTCTask_TypeDef;

/** End of RTC_Task
  * \}
  */

/**
 * \defgroup    RTC_Shortcut_Task RTC Shortcut Task
 * \{
 * \ingroup     RTC_Exported_Constants
 */
typedef enum
{
    RTC_SHORTCUT_TASK_STOP = 1,
    RTC_SHORTCUT_TASK_CLEAR = 2,
} RTCShortcutTask_TypeDef;

/** End of RTC_Shortcut_Task
  * \}
  */

/**
 * \defgroup    RTC_Shortcut_Event RTC Shortcut Event
 * \{
 * \ingroup     RTC_Exported_Constants
 */
typedef enum
{
    RTC_SHORTCUT_EVENT_COM0 = 0,
    RTC_SHORTCUT_EVENT_COM1 = 1,
    RTC_SHORTCUT_EVENT_COM2 = 2,
    RTC_SHORTCUT_EVENT_COM3 = 3,
} RTCShortcutEvent_TypeDef;

/** End of RTC_Shortcut_Event
  * \}
  */
#endif

/** End of RTC_Exported_Constants
  * \}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** \defgroup RTC_Exported_Functions RTC Exported Functions
  * \brief
  * \{
  */

/**
 * \brief     Deinitialize the RTC peripheral registers to their default reset values(turn off clock).
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 * }
 * \endcode
 */
void RTC_DeInit(void);

/**
 * \brief     Set RTC prescaler value.
 *
 * \param[in] value  The prescaler value to be set. Should be no more than 12 bits!
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * #define RTC_PRESCALER_VALUE     49
 * #define RTC_COMP_INDEX          RTC_COMP3
 * #define RTC_COMP_INDEX_INT      RTC_INT_COMP3
 * #define RTC_COMP_VALUE          (1000)
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 *
 *     RTC_SetPrescaler(RTC_PRESCALER_VALUE);
 *     RTC_SetCompValue(RTC_COMP_INDEX, RTC_COMP_VALUE);
 *
 *     RTC_MaskINTConfig(RTC_COMP_INDEX_INT, DISABLE);
 *     RTC_INTConfig(RTC_COMP_INDEX_INT, ENABLE);
 *
 *     RTC_NvCmd(ENABLE);
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_SetPrescaler(uint16_t value);

/**
 * \brief     Start or stop RTC peripheral.
 *
 * \param[in] NewState  New state of RTC peripheral.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: RTC count start.
 *                      - DISABLE: RTC count stop and clear count value.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * #define RTC_PRESCALER_VALUE     49
 * #define RTC_COMP_INDEX          RTC_COMP3
 * #define RTC_COMP_INDEX_INT      RTC_INT_COMP3
 * #define RTC_COMP_VALUE          (1000)
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 *
 *     RTC_SetPrescaler(RTC_PRESCALER_VALUE);
 *     RTC_SetCompValue(RTC_COMP_INDEX, RTC_COMP_VALUE);
 *
 *     RTC_MaskINTConfig(RTC_COMP_INDEX_INT, DISABLE);
 *     RTC_INTConfig(RTC_COMP_INDEX_INT, ENABLE);
 *
 *     RTC_NvCmd(ENABLE);
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_Cmd(FunctionalState NewState);

/**
 * \brief     Enable or disable the specified RTC interrupt source.
 *
 * \param[in] RTC_INT   Specify the RTC interrupt sources.
 *                      This parameter can be any combination of the following values, refer to \ref RTC_Interrupts.
 *                      \arg RTC_INT_TICK: tick interrupt.
 *                      \arg RTC_INT_OVF: counter overflow interrupt
 *                      \arg RTC_INT_COMP0: Compare 0 interrupt.
 *                      \arg RTC_INT_COMP1: Compare 1 interrupt.
 *                      \arg RTC_INT_COMP2: Compare 2 interrupt.
 *                      \arg RTC_INT_COMP3: Compare 3 interrupt.
 *                      \arg RTC_INT_PRE_COMP: Prescale compare interrupt.
 *                      \arg RTC_INT_PRE_COMP3: Prescale & compare 3 interrupt.
 * \param[in] NewState  New state of the specified RTC interrupt.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the specified interrupt of RTC.
 *                      - DISABLE: Disable the specified interrupt of RTC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * #define RTC_PRESCALER_VALUE     49
 * #define RTC_COMP_INDEX          RTC_COMP3
 * #define RTC_COMP_INDEX_INT      RTC_INT_COMP3
 * #define RTC_COMP_VALUE          (1000)
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 *
 *     RTC_SetPrescaler(RTC_PRESCALER_VALUE);
 *     RTC_SetCompValue(RTC_COMP_INDEX, RTC_COMP_VALUE);
 *
 *     RTC_MaskINTConfig(RTC_COMP_INDEX_INT, DISABLE);
 *     RTC_INTConfig(RTC_COMP_INDEX_INT, ENABLE);
 *
 *     RTC_NvCmd(ENABLE);
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_INTConfig(uint32_t RTC_INT, FunctionalState NewState);

/**
 * \brief  Enable or disable the specified RTC wakeup function.
 *
 * \param[in] RTC_WK    Specify the RTC wakeup sources.
 *                      This parameter can be any combination of the following values.
 *                      Refer to \ref RTC_Wakeup_Interrupts.
 *                      \arg RTC_WK_TICK: tick wakeup function
 *                      \arg RTC_WK_OVF: tick wakeup function
 *                      \arg RTC_WK_PRE_COMP: prescale compare wakeup function
 *                      \arg RTC_WK_PRE_COMP3: prescale & compare 3 wakeup function
 *                      \arg RTC_WK_COMP0GT: compare 0 gt wakeup function
 *                      \arg RTC_WK_COMP1GT: compare 1 gt wakeup function
 *                      \arg RTC_WK_COMP2GT: compare 2 gt wakeup function
 *                      \arg RTC_WK_COMP3GT: compare 3 gt wakeup function
 *                      \arg RTC_WK_COMP0: compare 0 wakeup function
 *                      \arg RTC_WK_COMP1: compare 1 wakeup function
 *                      \arg RTC_WK_COMP2: compare 2 wakeup function
 *                      \arg RTC_WK_COMP3: compare 3 wakeup function
 * \param[in] NewState  New state of the specified RTC wakeup function.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the specified interrupt of RTC wakeup.
 *                      - DISABLE: Disable the specified interrupt of RTC wakeup.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * #define RTC_PRESCALER_VALUE     49
 * #define RTC_COMP_INDEX          RTC_COMP3
 * #define RTC_COMP_INDEX_INT      RTC_INT_COMP3
 * #define RTC_COMP_VALUE          (1000)
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 *
 *     RTC_SetPrescaler(RTC_PRESCALER_VALUE);
 *     RTC_SetCompValue(RTC_COMP_INDEX, RTC_COMP_VALUE);
 *
 *     RTC_MaskINTConfig(RTC_COMP_INDEX_INT, DISABLE);
 *     RTC_INTConfig(RTC_COMP_INDEX_INT, ENABLE);
 *     RTC_WKConfig(RTC_WK_TICK, ENABLE);
 *     RTC_NvCmd(ENABLE);
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_WKConfig(uint32_t RTC_WK, FunctionalState NewState);

/**
 * \brief     Enable or disable RTC interrupt signal to CPU NVIC.
 *
 * \param[in] NewState  Enable or disable RTC interrupt signal to CPU.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the interrupt of CPU NVIC.
 *                      - DISABLE: Disable the interrupt of CPU NVIC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * #define RTC_PRESCALER_VALUE     49
 * #define RTC_COMP_INDEX          RTC_COMP3
 * #define RTC_COMP_INDEX_INT      RTC_INT_COMP3
 * #define RTC_COMP_VALUE          (1000)
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 *
 *     RTC_SetPrescaler(RTC_PRESCALER_VALUE);
 *     RTC_SetCompValue(RTC_COMP_INDEX, RTC_COMP_VALUE);
 *
 *     RTC_MaskINTConfig(RTC_COMP_INDEX_INT, DISABLE);
 *     RTC_INTConfig(RTC_COMP_INDEX_INT, ENABLE);
 *
 *     RTC_NvCmd(ENABLE);
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_NvCmd(FunctionalState NewState);

/**
 * \brief     Enable or disable system wake up function of RTC.
 *
 * \param[in] NewState  New state of the wake up function.
 *                      This parameter can be one of the following values:
 *                      - ENABLE: Enable the system wake up function.
 *                      - DISABLE: Disable the system wake up function.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_SystemWakeupConfig(ENABLE);
 * }
 * \endcode
 */
void RTC_SystemWakeupConfig(FunctionalState NewState);

/**
 * \brief     Reset counter value of RTC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_ResetCounter();
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_ResetCounter(void);

/**
 * \brief     Reset prescaler counter value of RTC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_ResetPrescalerCounter();
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_ResetPrescalerCounter(void);

/**
 * \brief  Get the specified RTC interrupt status.
 *
 * \param[in] RTC_INT  Specify the RTC interrupt sources.
 *                     This parameter can be any combination of the following values.
 *                     Refer to \ref RTC_Interrupts.
 *                     - RTC_INT_TICK: RTC tick interrupt source.
 *                     - RTC_INT_COMP0: Compare 0 interrupt source.
 *                     - RTC_INT_COMP1: Compare 1 interrupt source.
 *                     - RTC_INT_COMP2: Compare 2 interrupt source.
 *                     - RTC_INT_COMP3: Compare 3 interrupt source.
 *                     - RTC_INT_PRE_COMP: Prescale compare interrupt source.
 *                     - RTC_INT_PRE_COMP3: Prescale & compare 3 interrupt source.
 *
 * \return  The status of \ref RTC_Interrupts.
 *          - SET: The RTC interrupt has occurred.
 *          - RESET: The RTC interrupt has not occurred.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     ITStatus int_status = RTC_GetINTStatus(RTC_INT_COMP0);
 * }
 * \endcode
 */
ITStatus RTC_GetINTStatus(uint32_t RTC_INT);

/**
 * \brief     Clear the interrupt pending bits of RTC.
 *
 * \param[in] RTC_INT  Specify the RTC interrupt flag to clear.
 *                     This parameter can be any combination of the following values.
 *                     Refer to \ref RTC_Interrupts.
 *                     - RTC_INT_TICK: RTC tick interrupt source.
 *                     - RTC_INT_OVF: RTC counter overflow interrupt source.
 *                     - RTC_INT_COMP0: Compare 0 interrupt source.
 *                     - RTC_INT_COMP1: Compare 1 interrupt source.
 *                     - RTC_INT_COMP2: Compare 2 interrupt source.
 *                     - RTC_INT_COMP3: Compare 3 interrupt source.
 *                     - RTC_INT_PRE_COMP: Prescale compare interrupt source.
 *                     - RTC_INT_PRE_COMP3: Prescale & compare 3 interrupt source.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_ClearINTPendingBit(RTC_INT_COMP0);
 * }
 * \endcode
 */
void RTC_ClearINTPendingBit(uint32_t RTC_INT);

/**
 * \brief  Get the specified RTC wakeup interrupt status.
 *
 * \param[in] RTC_WK  Specify the RTC wakeup interrupt sources.
 *                    This parameter can be any combination of the following values.
 *                    Refer to \ref RTC_Wakeup_Interrupts.
 *                    \arg RTC_WK_TICK: tick wakeup function
 *                    \arg RTC_WK_OVF: tick wakeup function
 *                    \arg RTC_WK_PRE_COMP: prescale compare wakeup function
 *                    \arg RTC_WK_PRE_COMP3: prescale & compare 3 wakeup function
 *                    \arg RTC_WK_COMP0GT: compare 0 gt wakeup function
 *                    \arg RTC_WK_COMP1GT: compare 1 gt wakeup function
 *                    \arg RTC_WK_COMP2GT: compare 2 gt wakeup function
 *                    \arg RTC_WK_COMP3GT: compare 3 gt wakeup function
 *                    \arg RTC_WK_COMP0: compare 0 wakeup function
 *                    \arg RTC_WK_COMP1: compare 1 wakeup function
 *                    \arg RTC_WK_COMP2: compare 2 wakeup function
 *                    \arg RTC_WK_COMP3: compare 3 wakeup function
 *
 * \return  The status of \ref RTC_Wakeup.
 *          - SET: The RTC wakeup interrupt has occurred.
 *          - RESET: The RTC wakeup interrupt has not occurred.
 */
ITStatus RTC_GetWakeupStatus(uint32_t RTC_WK);

/**
 * \brief  Clear the wakeup status bits of RTC.
 *
 * \param[in] RTC_WK  Specify the RTC wakeup flag to clear.
 *                    This parameter can be any combination of the following values.
 *                    Refer to \ref RTC_Wakeup_Interrupts.
 *                    \arg RTC_WK_TICK: tick wakeup function
 *                    \arg RTC_WK_OVF: tick wakeup function
 *                    \arg RTC_WK_PRE_COMP: prescale compare wakeup function
 *                    \arg RTC_WK_PRE_COMP3: prescale & compare 3 wakeup function
 *                    \arg RTC_WK_COMP0GT: compare 0 gt wakeup function
 *                    \arg RTC_WK_COMP1GT: compare 1 gt wakeup function
 *                    \arg RTC_WK_COMP2GT: compare 2 gt wakeup function
 *                    \arg RTC_WK_COMP3GT: compare 3 gt wakeup function
 *                    \arg RTC_WK_COMP0: compare 0 wakeup function
 *                    \arg RTC_WK_COMP1: compare 1 wakeup function
 *                    \arg RTC_WK_COMP2: compare 2 wakeup function
 *                    \arg RTC_WK_COMP3: compare 3 wakeup function
 */
void RTC_ClearWakeupStatusBit(uint32_t RTC_WK);

/**
 * \brief     Clear the interrupt pending bit of the select comparator of RTC.
 *
 * \param[in] index  The comparator number, Refer to \ref RTC_Comparator_Index.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_ClearCompINT(RTC_COMP0);
 * }
 * \endcode
 */
void RTC_ClearCompINT(RTCComIndex_TypeDef index);

/**
 * \brief     Clear the overflow interrupt pending bit of RTC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_ClearOverFlowINT();
 * }
 * \endcode
 */
void RTC_ClearOverFlowINT(void);

/**
 * \brief     Clear the tick interrupt pending bit of RTC.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_ClearTickINT();
 * }
 * \endcode
 */
void RTC_ClearTickINT(void);

#if (RTC_SUPPORT_CLK_INPUT_FROM_PAD_SEL == 1)
/**
 * \brief  Use the GPIO input signal as the clock source for the RTC.
 *
 * \param[in] rtc_in  The selected gpio pin.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_SelectSrcToGpioInput(PAD_P2_4_RTC_IN);
 * }
 * \endcode
 */
void RTC_SelectSrcToGpioInput(RTCInSel_TypeDef rtc_in);
#endif

/**
 * \brief     Set RTC comparator value.
 *
 * \param[in] index  The comparator number can range from 0 to 3, Refer to \ref RTC_Comparator_Index.
 * \param[in] value  The comparator value to be set. Should be no more than 24 bits!
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * #define RTC_PRESCALER_VALUE     49
 * #define RTC_COMP_INDEX          RTC_COMP3
 * #define RTC_COMP_INDEX_INT      RTC_INT_COMP3
 * #define RTC_COMP_VALUE          (1000)
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 *
 *     RTC_SetPrescaler(RTC_PRESCALER_VALUE);
 *     RTC_SetCompValue(RTC_COMP_INDEX, RTC_COMP_VALUE);
 *
 *     RTC_MaskINTConfig(RTC_COMP_INDEX_INT, DISABLE);
 *     RTC_INTConfig(RTC_COMP_INDEX_INT, ENABLE);
 *
 *     RTC_NvCmd(ENABLE);
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_SetCompValue(RTCComIndex_TypeDef index, uint32_t value);

#if (RTC_SUPPORT_COMPARE_GUARDTIME == 1)
/**
 * \brief     Set RTC comparator GT value.
 *
 * \param[in] index  The comparator gt number can range from 0 ~ 3, Refer to \ref RTC_ComparatorGT_Index.
 * \param[in] value  The comparator value to be set.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * #define RTC_PRESCALER_VALUE     49
 * #define RTC_COMP_INDEX          RTC_COMP3
 * #define RTC_COMP_INDEX_INT      RTC_INT_COMP3
 * #define RTC_COMP_VALUE          (1000)
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 *
 *     RTC_SetPrescaler(RTC_PRESCALER_VALUE);
 *     RTC_SetCompValue(RTC_COMP_INDEX, RTC_COMP_VALUE);
 *
 *     RTC_MaskINTConfig(RTC_COMP_INDEX_INT, DISABLE);
 *     RTC_INTConfig(RTC_COMP_INDEX_INT, ENABLE);
 *
 *     RTC_NvCmd(ENABLE);
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_SetCompGTValue(RTCCmopGTIndex_TypeDef index, uint32_t value);
#endif

/**
 * \brief     Set RTC prescaler comparator value.
 *
 * \param[in] value  The prescaler comparator value to be set. Should be no more than 12 bits!
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * #define RTC_PRESCALER_VALUE     (3200 - 1)//max 4095
 * #define RTC_PRECOMP_VALUE       (320)//max 4095
 * #define RTC_COMP3_VALUE         (10)
 *
 * void driver_rtc_init(void)
 * {
 *     RTC_DeInit();
 *
 *     RTC_SetPrescaler(RTC_PRESCALER_VALUE);
 *     RTC_SetPreCompValue(RTC_PRECOMP_VALUE);
 *     RTC_SetCompValue(RTC_COMP3, RTC_COMP3_VALUE);
 *
 *     RTC_MaskINTConfig(RTC_INT_PRE_COMP3, DISABLE);
 *     RTC_INTConfig(RTC_INT_PRE_COMP3, ENABLE);
 *
 *     RTC_NvCmd(ENABLE);
 *     RTC_Cmd(ENABLE);
 * }
 * \endcode
 */
void RTC_SetPreCompValue(uint32_t value);

/**
 * \brief     Get counter value of RTC.
 *
 * \return    The counter value.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     uitn32_t counter = RTC_GetCounter();
 * }
 * \endcode
 */
uint32_t RTC_GetCounter(void);

/**
 * \brief     Get prescaler counter value of RTC.
 *
 * \return    The prescaler counter value.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     uitn32_t pre_counter = RTC_GetPreCounter();
 * }
 * \endcode
 */
uint32_t RTC_GetPreCounter(void);

/**
 * \brief     Get RTC comparator value.
 *
 * \param[in] index  The comparator number,  Refer to \ref RTC_Comparator_Index
 *
 * \return    The comparator value.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     uint32_t data = RTC_GetCompValue(RTC_COMP0);
 * }
 * \endcode
 */
uint32_t RTC_GetCompValue(RTCComIndex_TypeDef index);

#if (RTC_SUPPORT_COMPARE_GUARDTIME == 1)
/**
 * \brief     Get RTC comparator gt value.
 *
 * \param[in] index  The comparator number 0~3, Refer to \ref RTC_ComparatorGT_Index.
 *
 * \return    The comparator value.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     uitn32_t data = RTC_GetCompGTValue(RTC_COMP0);
 * }
 * \endcode
 */
uint32_t RTC_GetCompGTValue(RTCCmopGTIndex_TypeDef index);
#endif

/**
 * \brief     Get RTC prescaler comparator value.
 *
 * \return    The prescaler comparator value.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     uitn32_t data = RTC_GetPreCompValue();
 * }
 * \endcode
 */
uint32_t RTC_GetPreCompValue(void);

/**
 * \brief     Write backup register for store time information.
 *
 * \param[in] value  The data to be written to the backup register.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     RTC_WriteBackupReg(0x01020304);
 * }
 * \endcode
 */
void RTC_WriteBackupReg(uint32_t value);

/**
 * \brief     Read backup register.
 *
 * \return    Register value.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void rtc_demo(void)
 * {
 *     uint32_t reg_data = RTC_ReadBackupReg();
 * }
 * \endcode
 */
uint32_t RTC_ReadBackupReg(void);

#if (RTC_SUPPORT_RAP_FUNCTION == 1)

void RTC_RAPQactiveCtrl(uint32_t Qactive, FunctionalState NewState);

void RTC_RAPModeCmd(FunctionalState NewState);

void RTC_TaskTrigger(uint32_t Task);

void RTC_ShortcutCmd(uint32_t Task, uint32_t Event, FunctionalState NewState);

#endif

/** End of RTC_Exported_Functions
  * \}
  */

/** End of RTC
  * \}
  */

#ifdef __cplusplus
}
#endif

#endif /* RTL_RTC_H */



