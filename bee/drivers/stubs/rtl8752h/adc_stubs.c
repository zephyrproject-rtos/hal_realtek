/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file adc_stubs.c
 * @brief Stub implementations for ADC library functions.
 *        Used when building without binary blobs (CONFIG_BUILD_ONLY_NO_BLOBS).
 */

#include <stdbool.h>
#include <stdint.h>
#include <adc_lib.h>

/* Stub implementations for ADC functions */
bool ADC_CalibrationInit(void)
{
	return true;
}

float ADC_GetVoltage(const ADC_SampleMode vSampleMode, int32_t vSampleData,
		     ADC_ErrorStatus *pErrorStatus)
{
	(void)vSampleMode;
	(void)vSampleData;
	(void)pErrorStatus;
	return 0.0f;
}

uint16_t ADC_GetResistance(void)
{
	return 0;
}