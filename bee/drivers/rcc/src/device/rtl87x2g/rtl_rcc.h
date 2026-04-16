/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef RTL_RCC_H
#define RTL_RCC_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "utils/rtl_utils.h"
#include "rtl_rcc_def.h"

/** \defgroup RCC         RCC
  * \brief
  * \{
  */

/*============================================================================*
 *                         Constants
 *============================================================================*/
/** \defgroup RCC_Exported_Constants RCC Exported Constants
  * \brief
  * \{
  */

/**
 * \defgroup    Clock_Divider Clock Divider
 * \{
 * \ingroup     RCC_Exported_Constants
 */
typedef enum
{
    CLOCK_DIV_1,    //!< The RCC clock divider is set to 1.
    CLOCK_DIV_2,    //!< The RCC clock divider is set to 2.
    CLOCK_DIV_4,    //!< The RCC clock divider is set to 4.
    CLOCK_DIV_8,    //!< The RCC clock divider is set to 8.
    CLOCK_DIV_16,   //!< The RCC clock divider is set to 16.
    CLOCK_DIV_32,   //!< The RCC clock divider is set to 32.
    CLOCK_DIV_40,   //!< The RCC clock divider is set to 40.
    CLOCK_DIV_64,   //!< The RCC clock divider is set to 64.
} RCCClockDiv_Typedef;

#define IS_DIV(DIV)   (((DIV) == CLOCK_DIV_1) || \
                       ((DIV) == CLOCK_DIV_2) || \
                       ((DIV) == CLOCK_DIV_4) || \
                       ((DIV) == CLOCK_DIV_8) || \
                       ((DIV) == CLOCK_DIV_16) || \
                       ((DIV) == CLOCK_DIV_32) || \
                       ((DIV) == CLOCK_DIV_40) || \
                       ((DIV) == CLOCK_DIV_64)) //!< Check if the input parameter is valid.

/** End of Clock_Divider
  * \}
  */

/** End of RCC_Exported_Constants
  * \}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** \defgroup RCC_Exported_Functions RCC Exported Functions
  * \brief
  * \{
  */

/**
 * \brief  Enable or disable the APB peripheral clock.
 *
 * \param[in] APBPeriph        Specify the APB peripheral to gates its clock.
 *                             This parameter can refer to \ref APB_Peripheral_Func.
 * \param[in] APBPeriph_Clock  Specify the APB peripheral clock config.
 *                             This parameter can refer to \ref APB_Peripheral_Clock (must be the same with APBPeriph).
 * \param[in] NewState         New state of the specified peripheral clock.
 *                             This parameter can be one of the following values:
 *                             - ENABLE: Enable the specified peripheral clock.
 *                             - DISABLE: Disable the specified peripheral clock.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_spi_init(void)
 * {
 *     RCC_PeriphClockCmd(APBPeriph_SPI0, APBPeriph_SPI0_CLOCK, ENABLE);
 * }
 * \endcode
 */
void RCC_PeriphClockCmd(uint32_t APBPeriph, uint32_t APBPeriph_Clock, FunctionalState NewState);

/**
 * \brief  Enable or disable the APB peripheral function.
 *
 * \param[in] APBPeriph        Specify the APB peripheral.
 *                             This parameter can refer APB Peripheral.
 * #if RCC_SUPPORT_PERIPHCLOCKCONFIG_API_MORE_PARAMS
 * \param[in] APBPeriph_Clock  Specify the APB peripheral clock config.
 *                             This parameter can refer to APB Peripheral Clock (must be the same with APBPeriph).
 * #endif
 * \param[in] NewState         New state of the specified peripheral function.
 *                             This parameter can be one of the following values:
 *                             \arg ENABLE: Enable the specified peripheral function.
 *                             \arg DISABLE: Disable the specified peripheral function.
 */
#if RCC_SUPPORT_PERIPHCLOCKCONFIG_API_MORE_PARAMS
void RCC_PeriFunctionConfig(uint32_t APBPeriph, uint32_t APBPeriph_Clock, FunctionalState NewState);
#else
void RCC_PeriFunctionConfig(uint32_t APBPeriph, FunctionalState NewState);
#endif

/**
 * \brief  Enable or disable the specified APB peripheral clock.
 *
 * \param[in] APBPeriph        Specify the APB peripheral to gates its clock.
 *                             This parameter can refer APB Peripheral.
 * #if RCC_SUPPORT_PERIPHCLOCKCONFIG_API_MORE_PARAMS
 * \param[in] APBPeriph_Clock  Specify the APB peripheral clock config.
 *                             This parameter can refer to APB Peripheral Clock (must be the same with APBPeriph).
 * #endif
 * \param[in] NewState         New state of the specified APB peripheral clock.
 *                             This parameter can be one of the following values:
 *                             \arg ENABLE: Enable the specified APB peripheral clock.
 *                             \arg DISABLE: Disable the specified APB peripheral clock.
 */
#if RCC_SUPPORT_PERIPHCLOCKCONFIG_API_MORE_PARAMS
void RCC_PeriClockConfig(uint32_t APBPeriph, uint32_t APBPeriph_Clock, FunctionalState NewState);
#else
void RCC_PeriClockConfig(uint32_t APBPeriph_Clock, FunctionalState NewState);
#endif

#if RCC_SUPPORT_CLOCKGATECMD_API
/**
 * \brief  Enable or disable the specified RCC Clock Source Gating.
 *
 * \param[in] ClockGate  Specify the clock source to gate.
 *                       This parameter can be one of the following values:
 *                       \arg CLOCK_GATE_5M: 5M clock source for adc and keyscan.
 *                       \arg CLOCK_GATE_10M: 10M clock source for bluewiz.
 *                       \arg CLOCK_GATE_20M: 20M clock source for 2wssi and qdec.
 * \param[in] NewState   New state of the specified RCC Clock Source.
 *                       This parameter can be one of the following values:
 *                       - ENABLE: Enable the specified RCC Clock Source.
 *                       - DISABLE: Disable the specified RCC Clock Source.
 *
 * <b>Example usage</b>
 * \code{.c}
 *
 * void driver_xx_init(void)
 * {
 *     RCC_ClockGateCmd(CLOCK_GATE_5M, ENABLE);
 * }
 * \endcode
 */
extern void RCC_ClockGateCmd(uint32_t ClockGate, FunctionalState NewState);
#endif

/** End of RCC_Exported_Functions
  * \}
  */

/** End of RCC
  * \}
  */

#ifdef __cplusplus
}
#endif

#endif /* RTL_RCC_H */


