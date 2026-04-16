/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "rtl_adc.h"
#include "rtl_rcc.h"
#include "stdbool.h"
#include "app_section.h"

/*============================================================================*
 *                          Private Macros
 *============================================================================*/
#define AUXADC_AON_REG0X1B94                ((AON_NS_REG0X_AUX_V09_TYPE *)0x40001B94UL)
#define AUXADC_AON_REG0X1B90                ((AON_NS_REG0X_LDO_TYPE *)0x40001B90UL)
#define LPC_AON_AUXADC                      ((AON_NS_REG0X_SD_TYPE *)0x40001B98UL)
#define PERI_ON_AUDIO_CLOCK_CTRL            *((volatile uint32_t *)PERIBLKCTRL_AUDIO_REG_BASE)

typedef enum
{
    AUTO_SWITCH_TABLE0_IDLE_MODE            = 0,
    AUTO_SWITCH_TABLE1_XTAL_MODE_TRANSITION = 1,
    AUTO_SWITCH_TABLE2_TXAFE1_TRX_MODE      = 2,
    AUTO_SWITCH_TABLE3_TXAFE2_TRX_MODE      = 3,
    AUTO_SWITCH_TABLE4_TXAFE1_TX_4DBM_MODE  = 4,

    AUTO_SWITCH_TABLE_MAX                   = 5,
}
AUTO_SWITCH_TABLE_TYPE;

extern void (*pmu_auto_switch_map_force_base_level)(AUTO_SWITCH_TABLE_TYPE type, bool enable);

void force_vddcore_1v2_for_auxadc(bool enable)
{
    static bool current_state = false;
    if (current_state != enable)
    {
        pmu_auto_switch_map_force_base_level(AUTO_SWITCH_TABLE3_TXAFE2_TRX_MODE, enable);
        current_state = enable;
    }
}
/*============================================================================*
 *                           Public Functions
 *============================================================================*/
void ADC_SISet(void)
{
    /*Added to stabilize the power supply!*/
    /* set to LDO mode */
    force_vddcore_1v2_for_auxadc(true);
    AUXADC_AON_REG0X1B90->hw_pd = 0x0;
    AUXADC_AON_REG0X1B90->SELLDO = 0x1;
}

void ADC_PowerOff(void)
{
    /*Power off control flow!*/

    AUXADC_AON_REG0X1B94->pow = 0x0;
    AUXADC_AON_REG0X1B94->pow_ref = 0x0;

    uint32_t codec_clk_en = (PERI_ON_AUDIO_CLOCK_CTRL & BIT9) >> 9;

    if (LPC_AON_AUXADC-> POW_SD_H != 0x1 && codec_clk_en != 1)
    {
        AUXADC_AON_REG0X1B90->anapar_pow_ad_2 = 0x0;
        AUXADC_AON_REG0X1B90->anapar_pow_ad_1 = 0x0;
        AUXADC_AON_REG0X1B90->hw_pd = 0x1;
    }
    force_vddcore_1v2_for_auxadc(false);

}

typedef enum
{
    ADC_POW_AD1_DELAY_10_US,
    ADC_POW_AD1_DELAY_20_US,
    ADC_POW_AD1_DELAY_40_US,
    ADC_POW_AD1_DELAY_80_US,
} ADCPowAD1Delay_TypeDef;

typedef enum
{
    ADC_POW_AD2_DELAY_20_US,
    ADC_POW_AD2_DELAY_40_US,
    ADC_POW_AD2_DELAY_80_US,
    ADC_POW_AD2_DELAY_160_US,
} ADCPowAD2Delay_TypeDef;

typedef enum
{
    ADC_POW_REF_DELAY_30_US,
    ADC_POW_REF_DELAY_60_US,
    ADC_POW_REF_DELAY_120_US,
    ADC_POW_REF_DELAY_240_US,
} ADCPowRefDelay_TypeDef;


void ADC_DelayConfig(ADC_TypeDef *ADCx)
{
    ADC_POW_DATA_DLY_CTRL_TypeDef adc_0x50 = {.d32 = ADCx->ADC_POW_DATA_DLY_CTRL};

    adc_0x50.b.adc_reg0x_ldo_1_delay_sel = ADC_POW_AD1_DELAY_10_US;
    adc_0x50.b.adc_reg0x_ldo_2_delay_sel = ADC_POW_AD2_DELAY_20_US;
    adc_0x50.b.adc_reg0x_aux_v09_1_delay_sel = ADC_POW_REF_DELAY_30_US;

    ADCx->ADC_POW_DATA_DLY_CTRL = adc_0x50.d32;
}

void ADC_ManualModeConfig(void)
{
}

void ADC_BypassRegConfig(uint8_t ChannelNum, FunctionalState NewState)
{
    if (NewState != DISABLE)
    {
        AUXADC_AON_REG0X1B94->bypassen_7_0 |= BIT(ChannelNum);
    }
    else
    {
        AUXADC_AON_REG0X1B94->bypassen_7_0 &= ~BIT(ChannelNum);
    }
}

/**
  * \brief  Store ADC register values when system enter DLPS.
  * \param  PeriReg: Specifies to select the ADC peripheral.
  * \param  StoreBuf: Store buffer to store ADC register data.
  * \return None.
  */
RAM_FUNCTION
void ADC_DLPSEnter(void *PeriReg, void *StoreBuf)
{
    ADCStoreReg_TypeDef *store_buf = (ADCStoreReg_TypeDef *)StoreBuf;

    /*Open 10M clock source*/
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

    store_buf->adc_reg[0]         =  ADC->ADC_DIG_CTRL;         //0x04
    store_buf->adc_reg[1]         =  ADC->ADC_SCHED_CTRL;       //0x08
    store_buf->adc_reg[2]         =  ADC->ADC_CTRL_INT;         //0x0C

    store_buf->adc_schtab_reg[0]  =  ADC->ADC_SCHTAB0;          //0x10
    store_buf->adc_schtab_reg[1]  =  ADC->ADC_SCHTAB1;
    store_buf->adc_schtab_reg[2]  =  ADC->ADC_SCHTAB2;          //0x14
    store_buf->adc_schtab_reg[3]  =  ADC->ADC_SCHTAB3;
    store_buf->adc_schtab_reg[4]  =  ADC->ADC_SCHTAB4;          //0x18
    store_buf->adc_schtab_reg[5]  =  ADC->ADC_SCHTAB5;
    store_buf->adc_schtab_reg[6]  =  ADC->ADC_SCHTAB6;          //0x1C
    store_buf->adc_schtab_reg[7]  =  ADC->ADC_SCHTAB7;
    store_buf->adc_schtab_reg[8]  =  ADC->ADC_SCHTAB8;          //0x20
    store_buf->adc_schtab_reg[9]  =  ADC->ADC_SCHTAB9;
    store_buf->adc_schtab_reg[10] =  ADC->ADC_SCHTAB10;         //0x24
    store_buf->adc_schtab_reg[11] =  ADC->ADC_SCHTAB11;
    store_buf->adc_schtab_reg[12] =  ADC->ADC_SCHTAB12;         //0x28
    store_buf->adc_schtab_reg[13] =  ADC->ADC_SCHTAB13;
    store_buf->adc_schtab_reg[14] =  ADC->ADC_SCHTAB14;         //0x2C
    store_buf->adc_schtab_reg[15] =  ADC->ADC_SCHTAB15;

    store_buf->adc_reg[3]         =  ADC->ADC_POW_DATA_DLY_CTRL;//0x50
    store_buf->adc_reg[4]         =  ADC->ADC_DATA_CLK_CTRL;    //0x54
    store_buf->adc_reg[5]         =  ADC->ADC_TIME_PERIOD;      //0x5C

    return;
}

/**
  * \brief  Restore ADC register values when system enter DLPS.
  * \param  PeriReg: Specifies to select the ADC peripheral.
  * \param  StoreBuf: Restore buffer to restore ADC register data.
  * \return None
  */
RAM_FUNCTION
void ADC_DLPSExit(void *PeriReg, void *StoreBuf)
{
    ADCStoreReg_TypeDef *store_buf = (ADCStoreReg_TypeDef *)StoreBuf;

    /*Open 10M clock source*/
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

    /*Disable all interrupt.*/
    ADC->ADC_CTRL_INT &= (~0x1f);

    /* Set power mode first */
    ADC->ADC_POW_DATA_DLY_CTRL = store_buf->adc_reg[3];

    /* Disable schedule table */
    ADC->ADC_SCHED_CTRL &= (~0xffff);

    *(__IO uint32_t *)((uint32_t *)(&ADC->ADC_SCHTAB0))  = store_buf->adc_schtab_reg[0] |
                                                           (store_buf->adc_schtab_reg[1] << 16);
    *(__IO uint32_t *)((uint32_t *)(&ADC->ADC_SCHTAB2))  = store_buf->adc_schtab_reg[2] |
                                                           (store_buf->adc_schtab_reg[3] << 16);
    *(__IO uint32_t *)((uint32_t *)(&ADC->ADC_SCHTAB4))  = store_buf->adc_schtab_reg[4] |
                                                           (store_buf->adc_schtab_reg[5] << 16);
    *(__IO uint32_t *)((uint32_t *)(&ADC->ADC_SCHTAB6))  = store_buf->adc_schtab_reg[6] |
                                                           (store_buf->adc_schtab_reg[7] << 16);
    *(__IO uint32_t *)((uint32_t *)(&ADC->ADC_SCHTAB8))  = store_buf->adc_schtab_reg[8] |
                                                           (store_buf->adc_schtab_reg[9] << 16);
    *(__IO uint32_t *)((uint32_t *)(&ADC->ADC_SCHTAB10))  = store_buf->adc_schtab_reg[10] |
                                                            (store_buf->adc_schtab_reg[11] << 16);
    *(__IO uint32_t *)((uint32_t *)(&ADC->ADC_SCHTAB12))  = store_buf->adc_schtab_reg[12] |
                                                            (store_buf->adc_schtab_reg[13] << 16);
    *(__IO uint32_t *)((uint32_t *)(&ADC->ADC_SCHTAB14))  = store_buf->adc_schtab_reg[14] |
                                                            (store_buf->adc_schtab_reg[15] << 16);

    ADC->ADC_SCHED_CTRL     = store_buf->adc_reg[1];
    ADC->ADC_DIG_CTRL       = (store_buf->adc_reg[0] & (~((uint32_t)0x03)));
    ADC->ADC_DATA_CLK_CTRL  = store_buf->adc_reg[4];
    ADC->ADC_TIME_PERIOD    = store_buf->adc_reg[5];

    /*Clear ADC FIFO */
    ADC->ADC_DIG_CTRL |= BIT26;
    /* Clear all interrupt */
    ADC->ADC_CTRL_INT |= (0x1f << 8);

    /* Restore specify interrupt */
    ADC->ADC_CTRL_INT = store_buf->adc_reg[2];

    return;
}

