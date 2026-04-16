/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ADC_K_LIB_H
#define __ADC_K_LIB_H

#include <stdbool.h>
#include <stdint.h>

/** \defgroup ADC_LIB        ADC LIB
  * \brief
  * \{
  */
/*============================================================================*
 *                         Constants
 *============================================================================*/
/** \defgroup ADC_LIB_ADC_Exported_Constants ADC Exported Constants
  * \brief
  * \{
  */

/**
 * \defgroup    ADC_Mode ADC Mode
 * \{
 * \ingroup     ADC_LIB_ADC_Exported_Constants
 */
#define USE_ADC_DIVIDE_SINGLE_MODE      1
#define USE_ADC_BYPASS_SINGLE_MODE      1
#define USE_ADC_DIVIDE_DIFF_MODE        1
#define USE_ADC_BYPASS_DIFF_MODE        1
/** End of ADC_Mode
  * \}
  */

/**
 * \defgroup   ADC_SampleMode ADC SampleMode
 * \{
 * \ingroup    ADC_LIB_ADC_Exported_Constants
 */
typedef enum
{
    DIVIDE_SINGLE_MODE = 1,
    BYPASS_SINGLE_MODE = 2,
    DIVIDE_DIFFERENTIAL_MODE = 3,
    BYPASS_DIFFERENTIAL_MODE = 4,
} ADC_SampleMode;
/** End of ADC_SampleMode
  * \}
  */

/**
 * \defgroup    ADC_ErrorStatus ADC ErrorStatus
 * \{
 * \ingroup     ADC_LIB_ADC_Exported_Constants
 */
typedef enum
{
    NO_ERROR = 0,
    PARAMETER_ERROR = -1,
    RAM_DATA_ERROR = -2,
    NO_CALIBRATION = -3,
    VERSION_ERROR = -4,
} ADC_ErrorStatus;
/** End of ADC_ErrorStatus
  * \}
  */

/** End of ADC_LIB_ADC_Exported_Constants
  * \}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** \defgroup ADC_LIB_ADC_Exported_Functions ADC Exported Functions
  * \brief
  * \{
  */

/**
  * @brief  Initialize the ADC calibration process.
  * @return A boolean value indicating the success of the calibration operation.
  */
bool ADC_CalibrationInit(void);

/**
  * @brief  Retrieve the voltage value from an ADC conversion.
  * @param[in]  vSampleMode  Specifies the mode of ADC operation.
  * @param[in]  vSampleData  Represents the raw data obtained from the ADC.
  * @param[in]  pErrorStatus Pointer to a variable where ADC error status will be stored.
  * @return The voltage value calculated from the ADC data.
  */
float ADC_GetVoltage(const ADC_SampleMode vSampleMode, int32_t vSampleData,
                     ADC_ErrorStatus *pErrorStatus);

/**
  * @brief  Retrieve the resistance value measured by the ADC.
  * @return The ADC resistance value.
  */
uint16_t ADC_GetResistance(void);

/** End of ADC_LIB_ADC_Exported_Functions
  * \}
  */

/** End of ADC_LIB
  * \}
  */

#endif /* ADC_LIB_H */
