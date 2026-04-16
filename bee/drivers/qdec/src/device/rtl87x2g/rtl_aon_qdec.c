/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "rtl_aon_qdec.h"

/*============================================================================*
 *                           Public Functions
 *============================================================================*/
/**
  * \brief  Reset AON QDEC.
  * \param  None
  * \return None
  */
void AON_QDEC_DeInit(void)
{
    AON_QDEC_CONFIG_TypeDef aon_qdec_0x00 = {.d32 = AON_QDEC->AON_QDEC_CONFIG};
    /* Disable the AON QDEC. */
    aon_qdec_0x00.b.x_axis_en = 0;

    /* Disable the interrupt of AON QDEC. */
    aon_qdec_0x00.b.x_cnt_int_en = 0;
    aon_qdec_0x00.b.x_illegal_int_en = 0;

    /* Mask the interrupt of AON QDEC. */
    aon_qdec_0x00.b.cnt_int_mask = 1;
    aon_qdec_0x00.b.illegal_int_mask = 1;
    aon_qdec_0x00.b.x_int_mask = 1;
    aon_qdec_0x00.b.x_wake_aon_mask = 1;

    /* Disable the debounce function and clear the debounce time. */
    aon_qdec_0x00.b.x_debounce_en = 0;
    aon_qdec_0x00.b.x_debounce_cnt = 0;

    /* Reset counter scale. */
    aon_qdec_0x00.b.x_cnt_scale = 0;
    AON_QDEC->AON_QDEC_CONFIG = aon_qdec_0x00.d32;
    for (volatile uint32_t i = 0; i < 150; i++);

    /* Set initial phase to 00. */
    aon_qdec_0x00.b.x_set_initial_phase = 1;
    aon_qdec_0x00.b.x_initial_phase = phaseMode0;
    AON_QDEC->AON_QDEC_CONFIG = aon_qdec_0x00.d32;
    for (volatile uint32_t i = 0; i < 150; i++);

    aon_qdec_0x00.b.x_set_initial_phase = 0;
    AON_QDEC->AON_QDEC_CONFIG = aon_qdec_0x00.d32;
    for (volatile uint32_t i = 0; i < 150; i++);

    /* Clear ACC counter and illegal counter. */
    AON_QDEC_INT_CLR_TypeDef aon_qdec_0x08 = {.d32 = AON_QDEC->AON_QDEC_INT_CLR};
    aon_qdec_0x08.b.acc_cnt_clr = 1;
    aon_qdec_0x08.b.illegal_cnt_clr = 1;
    AON_QDEC->AON_QDEC_INT_CLR = aon_qdec_0x08.d32;
    for (volatile uint32_t i = 0; i < 150; i++);
}


/**
  * \brief  Initializes the AON Qdecoder peripheral according to the specified
  *         parameters in the AON_QDEC_InitStruct
  * \param  AON_QDECx: selected AON Qdecoder peripheral.
  * \param  AON_QDEC_InitStruct: pointer to an AON_QDEC_InitStruct structure that
  *         contains the configuration information for the specified AON Qdecoder peripheral
  * \return None
  */
void AON_QDEC_Init(AON_QDEC_TypeDef *AON_QDECx, AON_QDEC_InitTypeDef *AON_QDEC_InitStruct)
{
    assert_param(IS_QDEC_PERIPH(AON_QDECx));

    AON_QDEC_CONFIG_TypeDef aon_qdec_0x00 = {.d32 = AON_QDECx->AON_QDEC_CONFIG};

    /* Set Manual Initial Phase*/
    if (AON_QDEC_InitStruct->manualLoadInitPhase == ENABLE)
    {
        aon_qdec_0x00.b.x_axis_en = 0x0;
        aon_qdec_0x00.b.x_set_initial_phase = 0x1;
        aon_qdec_0x00.b.x_initial_phase = AON_QDEC_InitStruct->initPhaseX;
        AON_QDECx->AON_QDEC_CONFIG = aon_qdec_0x00.d32;
        for (volatile uint32_t i = 0; i < 150; i++);
    }

    aon_qdec_0x00.b.x_set_initial_phase = 0x0;
    AON_QDECx->AON_QDEC_CONFIG = aon_qdec_0x00.d32;
    for (volatile uint32_t i = 0; i < 150; i++);

    /* The max debounce cnt is 0x1FFF*/
    if (AON_QDEC_InitStruct->debounceTimeX >=  0x1FFF)
    {
        AON_QDEC_InitStruct->debounceTimeX = 0x1FFF;

    }
    /* QDEC configure */
    aon_qdec_0x00.b.x_debounce_cnt = AON_QDEC_InitStruct->debounceTimeX;
    aon_qdec_0x00.b.x_cnt_scale = AON_QDEC_InitStruct->counterScaleX;
    aon_qdec_0x00.b.x_debounce_en = AON_QDEC_InitStruct->debounceEnableX;
    AON_QDECx->AON_QDEC_CONFIG = aon_qdec_0x00.d32;

    AON_QDEC_INT_CLR_TypeDef aon_qdec_0x08 = {.d32 = AON_QDECx->AON_QDEC_INT_CLR};
    aon_qdec_0x08.b.acc_cnt_clr = 0x1; // write 1 to clear acc cnt
    AON_QDECx->AON_QDEC_INT_CLR = aon_qdec_0x08.d32;
    return;
}


/**
  * \brief  Fills each AON_QDEC_InitStruct member with its default value.
  * \param  AON_QDEC_InitStruct: pointer to an AON_QDEC_InitStruct structure which will be initialized.
  * \return None
  */
void AON_QDEC_StructInit(AON_QDEC_InitTypeDef *AON_QDEC_InitStruct)
{
    AON_QDEC_InitStruct->debounceTimeX       = 32 * 5; /*!< 5ms */
    AON_QDEC_InitStruct->counterScaleX       = CounterScale_1_Phase;
    AON_QDEC_InitStruct->debounceEnableX     = DISABLE;
    AON_QDEC_InitStruct->manualLoadInitPhase = DISABLE;
    AON_QDEC_InitStruct->initPhaseX          = phaseMode0;

    return;
}


/**
  * \brief  Enable or disable the specified AON Qdecoder interrupts.
  * \param  AON_QDECx: selected AON Qdecoder peripheral.
  * \param  AON_QDEC_IT: specifies the AON QDECODER interrupts sources to be enabled or disabled.
  *         This parameter can be one of the following values:
  *         \arg AON_QDEC_X_INT_NEW_DATA:
  *         \arg AON_QDEC_X_INT_ILLEAGE:
  * \param  NewState: new state of the specified AON QDECODER interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * \return None
  */
void AON_QDEC_INTConfig(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_IT,
                        FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_AON_QDEC_PERIPH(AON_QDECx));
    assert_param(IS_FUNCTIONAL_STATE(NewState));
    assert_param(IS_AON_QDEC_INT_CONFIG(AON_QDEC_IT));

    AON_QDEC_CONFIG_TypeDef aon_qdec_0x00 = {.d32 = AON_QDECx->AON_QDEC_CONFIG};

    /* Enable or disable the selected AON QDECODER interrupts */
    if (AON_QDEC_IT & AON_QDEC_X_INT_NEW_DATA)
    {
        aon_qdec_0x00.b.x_cnt_int_en = NewState;
    }
    if (AON_QDEC_IT & AON_QDEC_X_INT_ILLEAGE)
    {
        aon_qdec_0x00.b.x_illegal_int_en = NewState;
    }
    AON_QDECx->AON_QDEC_CONFIG = aon_qdec_0x00.d32;
    return;
}

/**
  * \brief  Checks whether the specified AON Qdecoder flag is set or not.
  * \param  AON_QDECx: selected AON Qdecoder peripheral.
  * \param  AON_QDEC_FLAG: specifies the flag to check.
  *         This parameter can be one of the following values:
  *         \arg AON_QDEC_FLAG_NEW_CT_STATUS_X:
  *         \arg AON_QDEC_FLAG_ILLEGAL_STATUS_X:
  *         \arg AON_QDEC_FLAG_OVERFLOW_X:
  *         \arg AON_QDEC_FLAG_UNDERFLOW_X:
  * \return The new state of AON_QDEC_FLAG (SET or RESET).
  */
FlagStatus AON_QDEC_GetFlagState(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_FLAG)
{
    /* Check the parameters */
    assert_param(IS_AON_QDEC_PERIPH(QDECx));
    assert_param(IS_AON_QDEC_CLR_INT_STATUS(QDEC_FLAG));

    FlagStatus bitstatus = RESET;

    if (AON_QDEC_FLAG == AON_QDEC_FLAG_NEW_CT_STATUS_X)
    {
        if (AON_QDECx->AON_QDEC_SR_X & (BIT18))
        {
            bitstatus = SET;
        }
    }
    else if (AON_QDEC_FLAG == AON_QDEC_FLAG_ILLEGAL_STATUS_X)
    {
        if (AON_QDECx->AON_QDEC_SR_X & (BIT19))
        {
            bitstatus = SET;
        }
    }
    else if (AON_QDEC_FLAG == AON_QDEC_FLAG_OVERFLOW_X)
    {
        if (AON_QDECx->AON_QDEC_SR_X & (BIT16))
        {
            bitstatus = SET;
        }
    }
    else if (AON_QDEC_FLAG == AON_QDEC_FLAG_UNDERFLOW_X)
    {
        if (AON_QDECx->AON_QDEC_SR_X & (BIT17))
        {
            bitstatus = SET;
        }
    }

    return bitstatus;
}

/**
  * \brief  Enable or disable mask the specified AON Qdecoder axis interrupts.
  * \param  AON_QDECx: selected AON Qdecoder peripheral.
  * \param  AON_QDEC_AXIS: specifies the AON Qdecoder axis.
  *         This parameter can be one or logical OR of the following values:
  *         \arg AON_QDEC_X_CT_INT_MASK: The AON qdecoder CNT interrupt mask.
  *         \arg AON_QDEC_X_ILLEAGE_INT_MASK: The AON qdecoder illeage interrupt mask.
  *         \arg AON_QDEC_X_WAKE_AON_MASK: The AON qdecoder wake up AON mask.
  *         \arg AON_QDEC_X_INT_MASK: The AON qdecoder interrupt mask.
  * \param  NewState: new state of the specified AON Qdecoder interrupts mask.
  *         This parameter can be: ENABLE or DISABLE.
  * \return None
  */
void AON_QDEC_INTMask(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_AXIS,
                      FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_AON_QDEC_PERIPH(AON_QDECx));
    assert_param(IS_AON_QDEC_INT_MASK_CONFIG(AON_QDEC_AXIS));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState == ENABLE)
    {
        /* mask the selected AON_QDEC interrupts */
        AON_QDECx->AON_QDEC_CONFIG |= (AON_QDEC_AXIS);
    }
    else
    {
        /* unmask the selected AON_QDEC interrupts */
        AON_QDECx->AON_QDEC_CONFIG &= (~(AON_QDEC_AXIS));
    }

    return;
}

/**
  * \brief  Enable or disable AON Qdecoder Axis x direction.
  * \param  AON_QDECx: selected AON Qdecoder peripheral.
  * \param  AON_QDEC_AXIS: specifies the AON Qdecoder axis.
  *         This parameter can be one of the following values:
  *         \arg QDEC_AXIS_X: The qdecoder X axis.
  * \param  NewState: This parameter can be one of the following values:
  *         \arg ENABLE: Pause.
  *         \arg DISABLE: Resume.
  */
void AON_QDEC_Cmd(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_AXIS,
                  FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_AON_QDEC_PERIPH(AON_QDECx));
    assert_param(IS_AON_QDEC_AXIS_DIR(AON_QDEC_AXIS));

    if (NewState == ENABLE)
    {
        *((volatile uint32_t *)(&AON_QDECx->AON_QDEC_CONFIG) + AON_QDEC_AXIS / 2) |= AON_QDEC_0X00_AXIS_EN;

    }
    else
    {
        *((volatile uint32_t *)(&AON_QDECx->AON_QDEC_CONFIG) + AON_QDEC_AXIS / 2) &= ~
                                                                                     (AON_QDEC_0X00_AXIS_EN);
    }
}

/**
  * \brief  Clear AON Qdecoder interrupt pending bit.
  * \param  AON_QDECx: Selected AON Qdecoder peripheral.
  * \param  AON_QDEC_FLAG: Specifies the flag to clear.
  *         This parameter parameter can be one of the following values:
  *         \arg AON_QDEC_CLR_OVERFLOW_X: The overflow flag for x-axis accumulation counter.
  *         \arg AON_QDEC_CLR_ILLEGAL_INT_X: The illegal interrupt for X axis.
  *         \arg AON_QDEC_CLR_UNDERFLOW_X: The underflow flag for x-axis accumulation counter.
  *         \arg AON_QDEC_CLR_NEW_CT_X: The counter interrupt for X axis.
  * \return None.
  */
void AON_QDEC_ClearINTPendingBit(AON_QDEC_TypeDef *AON_QDECx, uint32_t AON_QDEC_CLR_INT)
{
    /* Check the parameters */
    assert_param(IS_AON_QDEC_PERIPH(AON_QDECx));
    assert_param(IS_AON_QDEC_CLR_INT_STATUS(AON_QDEC_CLR_INT));

    AON_QDECx->AON_QDEC_INT_CLR |= AON_QDEC_CLR_INT;

    return;
}

/**
  * \brief  Get AON Qdecoder Axis(x/y/z) direction.
  *
  * \param  AON_QDECx: Selected AON Qdecoder peripheral.
  * \param  AON_QDEC_AXIS: Specifies the AON Qdecoder axis.
  *         This parameter parameter can be one of the following values:
  *         \arg  AON_QDEC_AXIS_X: The AON qdecoder X axis.
  * \return The direction of the axis. This parameter parameter can be one of the following values:
  *         \retval AON_QDEC_AXIS_DIR_UP: The axis is rolling up.
  *         \retval AON_QDEC_AXIS_DIR_DOWN: The axis is rolling down.
  */
uint16_t AON_QDEC_GetAxisDirection(AON_QDEC_TypeDef *AON_QDECx,
                                   uint32_t AON_QDEC_AXIS)
{
    /* Check the parameters */
    assert_param(IS_AON_QDEC_PERIPH(AON_QDECx));
    assert_param(IS_AON_QDEC_AXIS_DIR(AON_QDEC_AXIS));

    return ((*((volatile uint32_t *)(&AON_QDECx->AON_QDEC_SR_X)) & (1 << 20)) == AON_QDEC_0X04_CNT_DIR);
}

/**
  * \brief  Get AON Qdecoder Axis(x/y/z) count.
  *
  * \param  AON_QDECx: Selected AON Qdecoder peripheral.
  * \param  AON_QDEC_AXIS: Specifies the AON Qdecoder axis.
  *         This parameter parameter can be one of the following values:
  *         \arg AON_QDEC_AXIS_X: The AON qdecoder X axis.
  * \return The count of the axis.
  */
uint16_t AON_QDEC_GetAxisCount(AON_QDEC_TypeDef *AON_QDECx,
                               uint32_t AON_QDEC_AXIS)
{
    /* Check the parameters */
    assert_param(IS_AON_QDEC_PERIPH(AON_QDECx));
    assert_param(IS_AON_QDEC_AXIS_DIR(AON_QDEC_AXIS));

    return ((uint16_t)(*((volatile uint32_t *)(&AON_QDECx->AON_QDEC_SR_X))));
}

/**
  * \brief  Pause or resume AON Qdecoder Axis x.
  *
  * \param  AON_QDECx: Selected AON Qdecoder peripheral.
  * \param  AON_QDEC_AXIS: Specifies the AON Qdecoder axis.
  *         This parameter parameter can be one of the following values:
  *         \arg AON_QDEC_AXIS_X: The AON qdecoder X axis.
  * \param  NewState: New state of the specified AON Qdecoder Axis.
  *         This parameter parameter can be one of the following values:
  *         \arg ENABLE: Pause.
  *         \arg DISABLE: Resume.
  * \return None.
  */
void AON_QDEC_CounterPauseCmd(AON_QDEC_TypeDef *AON_QDECx,
                              uint32_t AON_QDEC_AXIS,
                              FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_AON_QDEC_PERIPH(AON_QDECx));
    assert_param(IS_AON_QDEC_AXIS_DIR(AON_QDEC_AXIS));

    AON_QDEC_CONFIG_TypeDef aon_qdec_0x00 = {.d32 = AON_QDECx->AON_QDEC_CONFIG};
    aon_qdec_0x00.b.x_cnt_pause = NewState;
    AON_QDECx->AON_QDEC_CONFIG = aon_qdec_0x00.d32;
}



