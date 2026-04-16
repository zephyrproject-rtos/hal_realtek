/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "rtl_keyscan.h"
#include "rtl_rcc.h"

/*============================================================================*
 *                           Public Functions
 *============================================================================*/
/**
  * \brief  Deinitializes the Keyscan peripheral registers to their default reset values(turn off keyscan clock).
  * \param  KeyScan: Selected KeyScan peripheral.
  * \return None
  */
void KeyScan_DeInit(KEYSCAN_TypeDef *KeyScan)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));

    RCC_PeriphClockCmd(APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK, DISABLE);
}

/**
  * \brief  Initializes the KeyScan peripheral according to the specified
  *         parameters in the KeyScan_InitStruct
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  KeyScan_InitStruct: Pointer to a KEYSCAN_InitTypeDef structure that
  *         contains the configuration information for the specified KeyScan peripheral
  * \return None
  */
void KeyScan_Init(KEYSCAN_TypeDef *KeyScan, KEYSCAN_InitTypeDef *KeyScan_InitStruct)
{
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    assert_param(IS_KEYSCAN_ROW_NUM(KeyScan_InitStruct->rowSize));
    assert_param(IS_KEYSCAN_COL_NUM(KeyScan_InitStruct->colSize));
    assert_param(IS_KEYSCAN_DEBOUNCE_EN(KeyScan_InitStruct->debounceEn));
    assert_param(IS_KEYSCAN_SCANINTERVAL_EN(KeyScan_InitStruct->scantimerEn));
    assert_param(IS_KEYSCAN_RELEASE_DETECT_EN(KeyScan_InitStruct->detecttimerEn));
    assert_param(IS_KEYSCAN_DETECT_MODE(KeyScan_InitStruct->detectMode));
    assert_param(IS_KEYSCAN_FIFO_OVR_CTRL(KeyScan_InitStruct->fifoOvrCtrl));
    assert_param(IS_KEYSCAN_SCAN_MODE(KeyScan_InitStruct->scanmode));
    assert_param(IS_KEYSCAN_KEY_LIMIT(KeyScan_InitStruct->keylimit));

    /* Mask all keyscan interrupt */
    KeyScan->KEYSCAN_INT_MASK |= 0x1f;
    /* Set FSM to idle state */
    KEYSCAN_CONFIG2_TypeDef keyscan_0x08 = {.d32 = KeyScan->KEYSCAN_CONFIG2};
    keyscan_0x08.b.keyscan_run_enable = 0;
    keyscan_0x08.b.keyscan_manual_sel = KeyScan_InitStruct->manual_sel;
    KeyScan->KEYSCAN_CONFIG2 = keyscan_0x08.d32;

    KEYSCAN_CLK_DIV_TypeDef keyscan_0x00 = {.d32 = KeyScan->KEYSCAN_CLK_DIV};
    KEYSCAN_CONFIG1_TypeDef keyscan_0x04 = {.d32 = KeyScan->KEYSCAN_CONFIG1};
    KEYSCAN_COLUMN_CONFIG_TypeDef keyscan_0x0c = {.d32 = KeyScan->KEYSCAN_COLUMN_CONFIG};
    KEYSCAN_ROW_CONFIG_TypeDef keyscan_0x10 = {.d32 = KeyScan->KEYSCAN_ROW_CONFIG};
    /* Clock divider config */
    keyscan_0x00.b.keyscan_clk_div = KeyScan_InitStruct->clockdiv;
    keyscan_0x00.b.keyscan_delay_div = KeyScan_InitStruct->delayclk;
    /* Config scan mode and detect mode*/
    keyscan_0x08.b.keyscan_work_mode = KeyScan_InitStruct->scanmode;
    keyscan_0x08.b.keyscan_trig_sel = KeyScan_InitStruct->detectMode;
    keyscan_0x08.b.keyscan_fifo_ov_ctrl = KeyScan_InitStruct->fifoOvrCtrl;
    /* fifo threshol setting */
    keyscan_0x08.b.keyscan_fifo_th_level = KeyScan_InitStruct->fifotriggerlevel;
    /* Key limit setting */
    keyscan_0x08.b.keyscan_fifo_limit = KeyScan_InitStruct->keylimit;

    /* Time config */
    keyscan_0x04.b.keyscan_deb_timer_en = KeyScan_InitStruct->debounceEn;
    keyscan_0x04.b.keyscan_interval_timer_en = KeyScan_InitStruct->scantimerEn;
    keyscan_0x04.b.keyscan_release_timer_en = KeyScan_InitStruct->detecttimerEn;
    /* Time count config */
    keyscan_0x04.b.keyscan_deb_timer_cnt = KeyScan_InitStruct->debouncecnt;
    keyscan_0x04.b.keyscan_interval_timer_cnt = KeyScan_InitStruct->scanInterval;
    keyscan_0x04.b.keyscan_release_timer_cnt = KeyScan_InitStruct->releasecnt;
#if KEYSCAN_SUPPORT_ROW_LEVEL_CONFIGURE
    keyscan_0x04.b.keyscan_row_pull_high_en = KeyScan_InitStruct->rowpullhighEn;
#endif
#if KEYSCAN_SUPPORT_COLUNM_LEVEL_CONFIGURE
    keyscan_0x04.b.keyscan_column_output_high_en = KeyScan_InitStruct->colunmoutputhighEn;
#endif
    /* Set col map, config which col to work */
    keyscan_0x0c.b.keyscan_col_num = KeyScan_InitStruct->colSize - 1;
    keyscan_0x0c.b.keyscan_col_sel = (1 << KeyScan_InitStruct->colSize) - 1;
    /* Set col map, config which col to work */
    keyscan_0x10.b.keyscan_row_num = KeyScan_InitStruct->rowSize - 1;
    keyscan_0x10.b.keyscan_row_sel = (1 << KeyScan_InitStruct->rowSize) - 1;

    KeyScan->KEYSCAN_CLK_DIV = keyscan_0x00.d32;
    KeyScan->KEYSCAN_CONFIG2 = keyscan_0x08.d32;
    KeyScan->KEYSCAN_CONFIG1 = keyscan_0x04.d32;
    KeyScan->KEYSCAN_COLUMN_CONFIG = keyscan_0x0c.d32;
    KeyScan->KEYSCAN_ROW_CONFIG = keyscan_0x10.d32;
    /* Clear all interrupt status and status flag */
    KeyScan->KEYSCAN_INT_CLR |= 0x1ff;
    /* Unmask all keyscan interrupt */
    KeyScan->KEYSCAN_INT_MASK &= ~0x1f;
    return;
}

/**
  * \brief  Fills each I2C_InitStruct member with its default value.
  * \param  KeyScan_InitStruct: Pointer to a KEYSCAN_InitTypeDef structure which will be initialized.
  * \return None
  */
void KeyScan_StructInit(KEYSCAN_InitTypeDef *KeyScan_InitStruct)
{
    KeyScan_InitStruct->colSize          = 2;
    KeyScan_InitStruct->rowSize          = 2;

    KeyScan_InitStruct->clockdiv         = 0x1f8;
    KeyScan_InitStruct->delayclk         = 0x01;

    KeyScan_InitStruct->debounceEn       = ENABLE;
    KeyScan_InitStruct->scantimerEn      = ENABLE;
    KeyScan_InitStruct->detecttimerEn    = ENABLE;

    KeyScan_InitStruct->debouncecnt      = 0x10;
    KeyScan_InitStruct->scanInterval     = 0x10;
    KeyScan_InitStruct->releasecnt       = 0x1;

    KeyScan_InitStruct->scanmode         = KeyScan_Auto_Scan_Mode;
    KeyScan_InitStruct->detectMode       = KeyScan_Detect_Mode_Level;
    KeyScan_InitStruct->manual_sel       = KeyScan_Manual_Sel_Key;

    KeyScan_InitStruct->fifotriggerlevel = 1;
    KeyScan_InitStruct->fifoOvrCtrl      = KeyScan_FIFO_OVR_CTRL_DIS_LAST;
    KeyScan_InitStruct->keylimit         = 0x03;

#if KEYSCAN_SUPPORT_ROW_LEVEL_CONFIGURE
    KeyScan_InitStruct->rowpullhighEn    = ENABLE;
#endif
#if KEYSCAN_SUPPORT_COLUNM_LEVEL_CONFIGURE
    KeyScan_InitStruct->colunmoutputhighEn  = DISABLE;
#endif
    return;
}

/**
  * \brief  Enable or disable the specified KeyScan interrupt.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  KeyScan_IT: Specifies the KeyScan interrupts sources to be enabled or disabled.
  *         This parameter can be any combination of the following values:
  *         \arg KEYSCAN_INT_THRESHOLD: Kescan FIFO data over threshold interrupt.
  *         \arg KEYSCAN_INT_OVER_READ: KeyScan over read interrupt.
  *         \arg KEYSCAN_INT_SCAN_END: KeyScan scan end interrupt.
  *         \arg KEYSCAN_INT_FIFO_NOT_EMPTY: KeyScan FIFO not empty interrupt.
  *         \arg KEYSCAN_INT_ALL_RELEASE: KeyScan all key release interrupt.
  * \param  NewState: New state of the specified KeyScan interrupts.
  *         This parameter can be: ENABLE or DISABLE.
  * \return None
  */
void KeyScan_INTConfig(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_IT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    assert_param(IS_FUNCTIONAL_STATE(NewState));
    assert_param(IS_KEYSCAN_CONFIG_IT(KeyScan_IT));

    if (NewState == ENABLE)
    {
        /* Enable the selected KeyScan interrupts */
        KeyScan->KEYSCAN_CONFIG2 |= KeyScan_IT;
    }
    else
    {
        /* Disable the selected KeyScan interrupts */
        KeyScan->KEYSCAN_CONFIG2 &= (uint32_t)~KeyScan_IT;
    }
}

/**
  * \brief  Enable or disable the specified KeyScan interrupts mask.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  NewState: New state of the specified KeyScan interrupts mask.
  *         This parameter can be: ENABLE or DISABLE.
  * \return None
  */
void KeyScan_INTMask(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_IT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState == ENABLE)
    {
        /* Mask KeyScan interrupts */
        KeyScan->KEYSCAN_INT_MASK |= KeyScan_IT;
    }
    else
    {
        /* Enable KeyScan interrupts */
        KeyScan->KEYSCAN_INT_MASK &= (~KeyScan_IT);
    }
}

/**
  * \brief  Read data from keyscan FIFO.
  * \param  KeyScan: selected KeyScan peripheral.
  * \param  outBuf: buffer to save data read from KeyScan FIFO.
  * \param  count: number of data to be read.
  * \return None
  */
void KeyScan_Read(KEYSCAN_TypeDef *KeyScan, uint16_t *outBuf, uint16_t count)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));

    uint16_t i = 0;

    for (i = 0; i < count; i++)
    {
        *outBuf++ = (uint16_t)KeyScan->KEYSCAN_FIFO_ENTRY;
    }

    return;
}

/**
  * \brief  Enable or disable the KeyScan peripheral.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  NewState: New state of the KeyScan peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * \return None
  */
void KeyScan_Cmd(KEYSCAN_TypeDef *KeyScan, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    assert_param(IS_FUNCTIONAL_STATE(NewState));
    KEYSCAN_CONFIG2_TypeDef keyscan_0x08 = {.d32 = KeyScan->KEYSCAN_CONFIG2};
    keyscan_0x08.b.keyscan_run_enable = NewState;
    keyscan_0x08.b.keyscan_manual_trigger = NewState && (!keyscan_0x08.b.keyscan_work_mode)
                                            && (!keyscan_0x08.b.keyscan_manual_sel);
    KeyScan->KEYSCAN_CONFIG2 = keyscan_0x08.d32;
}

/**
  * \brief  Set filter data.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  data: Config the data to be filtered.
  *         This parameter should not be more than 9 bits
  * \param  NewState: New state of the KeyScan peripheral.
  * \return none.
  */
void KeyScan_FilterDataConfig(KEYSCAN_TypeDef *KeyScan, uint16_t data, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));

    if (NewState == ENABLE)
    {
        KeyScan->KEYSCAN_CONFIG2 &= ~BIT21;

        KeyScan->KEYSCAN_CONFIG2 &= ~(0x1ff << 12);
        KeyScan->KEYSCAN_CONFIG2 |= ((data & 0x1ff) << 12);

        KeyScan->KEYSCAN_CONFIG2 |= BIT21;
    }
    else
    {
        KeyScan->KEYSCAN_CONFIG2 &= ~BIT21;

        KeyScan->KEYSCAN_CONFIG2 &= ~(0x1ff << 12);
        KeyScan->KEYSCAN_CONFIG2 |= ((data & 0x1ff) << 12);
    }

    return;
}

/**
  * \brief  KeyScan debounce time config.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  time: Keyscan hardware debounce time.
  * \param  NewState: New state of the KeyScan debounce function.
  *         This parameter can be: ENABLE or DISABLE.
  * \return none.
  */
void KeyScan_debounceConfig(KEYSCAN_TypeDef *KeyScan, uint16_t time,
                            FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    assert_param(IS_FUNCTIONAL_STATE(NewState));
    KEYSCAN_CONFIG1_TypeDef keyscan_0x04 = {.d32 = KeyScan->KEYSCAN_CONFIG1};
    keyscan_0x04.b.keyscan_deb_timer_en = NewState;
    keyscan_0x04.b.keyscan_deb_timer_cnt = time;
    KeyScan->KEYSCAN_CONFIG1 = keyscan_0x04.d32;
}

/**
  * \brief  Get KeyScan FIFO data num.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \return Data length in FIFO.
  */
uint16_t KeyScan_GetFifoDataNum(KEYSCAN_TypeDef *KeyScan)
{
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    KEYSCAN_INT_STS_TypeDef keyscan_0x20 = {.d32 = KeyScan->KEYSCAN_INT_STS};
    return (uint16_t)(keyscan_0x20.b.keyscan_fifo_data_level);
}

/**
  * \brief  Clear the KeyScan interrupt pending bit.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  KeyScan_IT: Specifies the KeyScan interrupts sources to be enabled or disabled.
  *         This parameter can be any combination of the following values:
  *         \arg KEYSCAN_INT_THRESHOLD: Kescan FIFO data over threshold interrupt.
  *         \arg KEYSCAN_INT_OVER_READ: KeyScan over read interrupt.
  *         \arg KEYSCAN_INT_SCAN_END: KeyScan scan end interrupt.
  *         \arg KEYSCAN_INT_FIFO_NOT_EMPTY: KeyScan FIFO not empty interrupt.
  *         \arg KEYSCAN_INT_ALL_RELEASE: KeyScan all key release interrupt.
  * \return none.
  */
void KeyScan_ClearINTPendingBit(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_IT)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    assert_param(IS_KEYSCAN_CONFIG_IT(KeyScan_IT));

    KeyScan->KEYSCAN_INT_CLR |= KeyScan_IT;

    return;
}

/**
  * \brief  Clear the specified KeyScan flag.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  KeyScan_FLAG: Specifies the flag to clear.
  *         This parameter can be any combination of the following values:
  *         \arg KEYSCAN_FLAG_FIFOLIMIT
  *         \arg KEYSCAN_FLAG_DATAFILTER
  *         \arg KEYSCAN_FLAG_OVR
  * \return none.
  */
void KeyScan_ClearFlags(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_FLAG)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    assert_param(IS_KEYSCAN_CLEAR_FLAG(KeyScan_FLAG));
    KEYSCAN_INT_CLR_TypeDef keyscan_0x1c = {.d32 = KeyScan->KEYSCAN_INT_CLR};
    if (KeyScan_FLAG & KEYSCAN_FLAG_FIFOLIMIT)
    {
        keyscan_0x1c.b.keyscan_fifo_limit_st_clr = 0x01;
    }
    if (KeyScan_FLAG & KEYSCAN_FLAG_DATAFILTER)
    {
        keyscan_0x1c.b.keyscan_fifo_datafilter_st_clr = 0x01;
    }
    if (KeyScan_FLAG & KEYSCAN_FLAG_OVR)
    {
        keyscan_0x1c.b.keyscan_fifo_ov_st_clr = 0x01;
    }
    KeyScan->KEYSCAN_INT_CLR = keyscan_0x1c.d32;
    return;
}

/**
  * \brief  Check whether the specified KeyScan flag is set.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \param  KeyScan_FLAG: Specifies the flag to clear.
  *         This parameter can be any combination of the following values:
  *         \arg KEYSCAN_FLAG_FIFOLIMIT:
  *         \arg KEYSCAN_FLAG_THRESHOLD:
  *         \arg KEYSCAN_FLAG_OVER_READ:
  *         \arg KEYSCAN_FLAG_SCAN_END:
  *         \arg KEYSCAN_FLAG_FIFO_NOT_EMPTY:
  *         \arg KEYSCAN_FLAG_ALL_RELEASE:
  *         \arg KEYSCAN_FLAG_DATAFILTER:
  *         \arg KEYSCAN_FLAG_OVR:
  *         \arg KEYSCAN_FLAG_FULL:
  *         \arg KEYSCAN_FLAG_EMPTY:
  * \return none.
  */
FlagStatus KeyScan_GetFlagState(KEYSCAN_TypeDef *KeyScan, uint32_t KeyScan_FLAG)
{
    FlagStatus bitstatus = RESET;

    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));
    assert_param(IS_KEYSCAN_FLAG(KeyScan_FLAG));

    if ((KeyScan->KEYSCAN_INT_STS & KeyScan_FLAG) != 0)
    {
        bitstatus = SET;
    }

    return bitstatus;
}

/**
  * \brief  Read FIFO data.
  * \param  KeyScan: Selected KeyScan peripheral.
  * \return Keyscan FIFO data.
  */
uint16_t KeyScan_ReadFifoData(KEYSCAN_TypeDef *KeyScan)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));

    return (uint16_t)(KeyScan->KEYSCAN_FIFO_ENTRY);
}
#if (KEYSCAN_SUPPORT_RAP_FUNCTION == 1)
void KeyScan_RAPModeCmd(KEYSCAN_TypeDef *KeyScan, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_KeyScan_PERIPH(KeyScan));

    KEYSCAN_TASK_CTRL_TypeDef keyscan_0x38 = {.d32 = KeyScan->KEYSCAN_TASK_CTRL};
    keyscan_0x38.b.rap_mode = NewState;
    KeyScan->KEYSCAN_TASK_CTRL = keyscan_0x38.d32;

    return;
}

void KEYSCAN_TaskTrigger(KEYSCAN_TypeDef *KeyScan, uint32_t Task)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(KeyScan));

    KEYSCAN_TASK_CTRL_TypeDef keyscan_0x38 = {.d32 = KeyScan->KEYSCAN_TASK_CTRL};
    if (Task == KEYSCAN_TASK_MANUAL)
    {
        keyscan_0x38.b.task_manual = 0x1;
    }
    KeyScan->KEYSCAN_TASK_CTRL = keyscan_0x38.d32;

    return;
}

void KeyScan_RAPQactiveCtrl(KEYSCAN_TypeDef *KeyScan, uint32_t Qactive, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_ADC_ALL_PERIPH(KeyScan));

    KEYSCAN_QACTIVE_CTRL_TypeDef keyscan_0x3C = {.d32 = KeyScan->KEYSCAN_QACTIVE_CTRL};
    if (Qactive == KEYSCAN_QACTIVE_FW_FORCE)
    {
        keyscan_0x3C.b.qact_clk_force_en = NewState;
    }
    if (Qactive == KEYSCAN_QACTIVE_FW_FORCE_PCLK)
    {
        keyscan_0x3C.b.qact_pclk_force_en = NewState;
    }
    KeyScan->KEYSCAN_QACTIVE_CTRL = keyscan_0x3C.d32;

    return;
}
#endif
