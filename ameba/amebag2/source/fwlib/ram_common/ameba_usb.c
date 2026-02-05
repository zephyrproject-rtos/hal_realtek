/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/

#include "ameba_soc.h"
#include "usb_hal.h"

/* Private defines -----------------------------------------------------------*/

#define USB_CAL_DATA_LEN									28U

/* USB OTG addon control register */
#define USB_ADDON_REG_CTRL									(USB_ADDON_REG_BASE + 0x04UL)

#define USB_ADDON_REG_CTRL_BIT_DIS_SUSPEND					BIT(1)	/* 1: Disable suspend signal to USBPHY */
#define USB_ADDON_REG_CTRL_BIT_UPLL_CKRDY					BIT(5)  /* 1: USB PHY clock ready */
#define USB_ADDON_REG_CTRL_BIT_USB_OTG_RST					BIT(8)  /* 1: Enable USB OTG */

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static int usb_chip_init(u8 mode);
static int usb_chip_deinit(void);
static usb_cal_data_t *usb_chip_get_cal_data(u8 mode);
static void usb_chip_enable_interrupt(u8 priority);
static void usb_chip_disable_interrupt(void);
static void usb_chip_register_irq_handler(void *handler, u8 priority);
static void usb_chip_unregister_irq_handler(void);

/* Private variables ---------------------------------------------------------*/

static usb_cal_data_t usb_cal_data[USB_CAL_DATA_LEN];

/* Exported variables --------------------------------------------------------*/

usb_hal_driver_t usb_hal_driver = {
	.init = usb_chip_init,
	.deinit = usb_chip_deinit,
	.get_cal_data = usb_chip_get_cal_data,
	.enable_interrupt = usb_chip_enable_interrupt,
	.disable_interrupt = usb_chip_disable_interrupt,
	.register_irq_handler = usb_chip_register_irq_handler,
	.unregister_irq_handler = usb_chip_unregister_irq_handler,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Get USB chip specific calibration data
  * @param  mode: 0 - device; 1 - host
  * @retval Pointer to calibration data buffer
  */
static usb_cal_data_t *usb_chip_get_cal_data(u8 mode)
{
	usb_cal_data_t *data = NULL;

	UNUSED(mode);

	u32 reg;

	reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_CTRL, 0U);
	if (reg & USB_ADDON_REG_AUTOLOAD_CTRL_BIT_AUTOLOAD_UPHY_EN) {
		reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_UPHY_P0E0, 0U);

		data = &usb_cal_data[0];
		data->page = 0x00;
		data->addr = 0xE0;
		data->val = (u8)(reg & 0xFF);

		data = &usb_cal_data[1];
		data->page = 0x00;
		data->addr = 0xE1;
		data->val = (u8)((reg >> 8) & 0xFF);

		data = &usb_cal_data[2];
		data->page = 0x00;
		data->addr = 0xE2;
		data->val = (u8)(reg >> 16 & 0xFF);

		data = &usb_cal_data[3];
		data->page = 0x00;
		data->addr = 0xE3;
		data->val = (u8)((reg >> 24) & 0xFF);

		reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_UPHY_P0E4, 0U);

		data = &usb_cal_data[4];
		data->page = 0x00;
		data->addr = 0xE4;
		data->val = (u8)(reg & 0xFF);

		data = &usb_cal_data[5];
		data->page = 0x00;
		data->addr = 0xE5;
		data->val = (u8)((reg >> 8) & 0xFF);

		data = &usb_cal_data[6];
		data->page = 0x00;
		data->addr = 0xE6;
		data->val = (u8)((reg >> 16) & 0xFF);

		data = &usb_cal_data[7];
		data->page = 0x00;
		data->addr = 0xE7;
		data->val = (u8)((reg >> 24) & 0xFF);

		reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_UPHY_P1E0, 0U);

		data = &usb_cal_data[8];
		data->page = 0x01;
		data->addr = 0xE0;
		data->val = (u8)(reg & 0xFF);

		data = &usb_cal_data[9];
		data->page = 0x01;
		data->addr = 0xE1;
		data->val = (u8)((reg >> 8) & 0xFF);

		reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_UPHY_P1E4, 0U);
		data = &usb_cal_data[10];
		data->page = 0x01;
		data->addr = 0xE5;
		data->val = (u8)((reg >> 8) & 0xFF);

		data = &usb_cal_data[11];
		data->page = 0x01;
		data->addr = 0xE6;
		data->val = (u8)((reg >> 16) & 0xFF);

		data = &usb_cal_data[12];
		data->page = 0x01;
		data->addr = 0xE7;
		data->val = (u8)((reg >> 24) & 0xFF);

		reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_UPHY_P0F0, 0U);

		data = &usb_cal_data[13];
		data->page = 0x00;
		data->addr = 0xF0;
		data->val = (u8)(reg & 0xFF);

		data = &usb_cal_data[14];
		data->page = 0x00;
		data->addr = 0xF1;
		data->val = (u8)((reg >> 8) & 0xFF);

		data = &usb_cal_data[15];
		data->page = 0x00;
		data->addr = 0xF2;
		data->val = (u8)((reg >> 16) & 0xFF);

		data = &usb_cal_data[16];
		data->page = 0x00;
		data->addr = 0xF3;
		data->val = (u8)((reg >> 24) & 0xFF);

		reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_UPHY_P0F4, 0U);

		data = &usb_cal_data[17];
		data->page = 0x00;
		data->addr = 0xF4;
		data->val = (u8)(reg & 0xFF);

		data = &usb_cal_data[18];
		data->page = 0x00;
		data->addr = 0xF5;
		data->val = (u8)((reg >> 8) & 0xFF);

		data = &usb_cal_data[19];
		data->page = 0x00;
		data->addr = 0xF6;
		data->val = (u8)((reg >> 16) & 0xFF);

		data = &usb_cal_data[20];
		data->page = 0x00;
		data->addr = 0xF7;
		data->val = (u8)((reg >> 24) & 0xFF);

		reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_UPHY_P1F4, 0U);

		data = &usb_cal_data[21];
		data->page = 0x01;
		data->addr = 0xF6;
		data->val = (u8)((reg >> 16) & 0xFF);

		data = &usb_cal_data[22];
		data->page = 0x01;
		data->addr = 0xF7;
		data->val = (u8)((reg >> 24) & 0xFF);

		reg = HAL_READ32(USB_ADDON_REG_AUTOLOAD_UPHY_P2E4, 0U);

		data = &usb_cal_data[23];
		data->page = 0x02;
		data->addr = 0xE4;
		data->val = (u8)(reg & 0xFF);

		data = &usb_cal_data[24];
		data->page = 0x02;
		data->addr = 0xE5;
		data->val = (u8)((reg >> 8) & 0xFF);

		data = &usb_cal_data[25];
		data->page = 0x02;
		data->addr = 0xE6;
		data->val = (u8)((reg >> 16) & 0xFF);

		data = &usb_cal_data[26];
		data->page = 0x02;
		data->addr = 0xE7;
		data->val = (u8)((reg >> 24) & 0xFF);

		data = &usb_cal_data[27];
		data->page = 0xFF;
		data->addr = 0x00;
		data->val = 0x00;

		data = &usb_cal_data[0];
	}

	return data;
}

/**
  * @brief  USB chip specific initialization
  * @param  void
  * @retval Status
  */
static int usb_chip_init(u8 mode)
{
	UNUSED(mode);

	u32 reg = 0;
	PLL_TypeDef *pll = (PLL_TypeDef *)PLL_REG_BASE;

	/* USB pinmux config */
	Pinmux_Config(USB_PIN_DM, PINMUX_FUNCTION_USB);
	Pinmux_Config(USB_PIN_DP, PINMUX_FUNCTION_USB);
	PAD_PullCtrl(USB_PIN_DM, GPIO_PuPd_NOPULL);
	PAD_PullCtrl(USB_PIN_DP, GPIO_PuPd_NOPULL);
	PAD_SleepPullCtrl(USB_PIN_DM, GPIO_PuPd_NOPULL);
	PAD_SleepPullCtrl(USB_PIN_DP, GPIO_PuPd_NOPULL);
	PAD_InputCtrl(USB_PIN_DM, DISABLE);
	PAD_InputCtrl(USB_PIN_DP, DISABLE);

	RCC_PeriphClockCmd(APBPeriph_USB, APBPeriph_USB_CLOCK, ENABLE);

	pll->PLL_UPLL_CTRL0 &= ~(PLL_BIT_USB2_DIGPADEN | PLL_BIT_USB2_DIGOTGPADEN);
	pll->PLL_UPLL_CTRL0 |= PLL_BIT_PWC_UAHV_ALIVE;
	pll->PLL_UPLL_CTRL0 |= PLL_BIT_USB_DPHY_EN;
	DelayUs(34);

	reg = HAL_READ32(USB_ADDON_REG_CTRL, 0U);
	reg |= USB_ADDON_REG_CTRL_BIT_USB_OTG_RST;
	HAL_WRITE32(USB_ADDON_REG_CTRL, 0U, reg);

	return HAL_OK;
}

/**
  * @brief  USB chip specific deinitialization
  * @param  void
  * @retval Status
  */
static int usb_chip_deinit(void)
{
	u32 reg = 0;
	PLL_TypeDef *pll = (PLL_TypeDef *)PLL_REG_BASE;

	reg = HAL_READ32(USB_ADDON_REG_CTRL, 0U);
	reg &= ~USB_ADDON_REG_CTRL_BIT_USB_OTG_RST;
	HAL_WRITE32(USB_ADDON_REG_CTRL, 0U, reg);

	pll->PLL_UPLL_CTRL0 &= ~PLL_BIT_USB_DPHY_EN;
	pll->PLL_UPLL_CTRL0 &= ~PLL_BIT_PWC_UAHV_ALIVE;
	pll->PLL_UPLL_CTRL0 |= PLL_BIT_USB2_DIGPADEN | PLL_BIT_USB2_DIGOTGPADEN;

	RCC_PeriphClockCmd(APBPeriph_USB, APBPeriph_USB_CLOCK, DISABLE);

	Pinmux_Config(USB_PIN_DM, PINMUX_FUNCTION_GPIO);
	Pinmux_Config(USB_PIN_DP, PINMUX_FUNCTION_GPIO);

	return HAL_OK;
}

/**
  * @brief  Enable USB interrupt
  * @param  priority: IRQ priority
  * @retval void
  */
static void usb_chip_enable_interrupt(u8 priority)
{
	UNUSED(priority);
	InterruptEn(USB_IRQ, priority);
}

/**
  * @brief  Disable USB interrupt
  * @retval void
  */
static void usb_chip_disable_interrupt(void)
{
	InterruptDis(USB_IRQ);
}

/**
  * @brief  Register USB IRQ handler
  * @param  handler: IRQ handler
  * @param  priority: IRQ priority
  * @retval void
  */
static void usb_chip_register_irq_handler(void *handler, u8 priority)
{
	if (handler != NULL) {
		InterruptRegister((IRQ_FUN)handler, USB_IRQ, (u32)NULL, priority);
	}
}

/**
  * @brief  Unregister USB IRQ handler
  * @retval void
  */
static void usb_chip_unregister_irq_handler(void)
{
	InterruptUnRegister(USB_IRQ);
}
/* Exported functions --------------------------------------------------------*/
