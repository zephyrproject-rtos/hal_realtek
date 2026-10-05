/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

static GPIO_TypeDef *GPIO_PortAddrGet(u32 GPIO_Port)
{
	assert_param(IS_GPIO_PORT_NUM(GPIO_Port));

	return GPIO_PORTx[GPIO_Port];
}

u32 GPIO_INTStatusGet(u32 GPIO_Port)
{
	GPIO_TypeDef *GPIO = GPIO_PortAddrGet(GPIO_Port);

	return GPIO->GPIO_INT_STATUS;
}

void GPIO_INTStatusClearEdge(u32 GPIO_Port)
{
	GPIO_TypeDef *GPIO = NULL;
	u32 IntStatus;

	GPIO = GPIO_PortAddrGet(GPIO_Port);

	IntStatus = GPIO->GPIO_INT_STATUS;

	/* Clear pending edge interrupt */
	GPIO->GPIO_INT_EOI = IntStatus;
}

u32 GPIO_DirectionGet(u32 port, u32 pin_mask)
{
	GPIO_TypeDef *GPIO = GPIO_PortAddrGet(port);

	/* pin_mask must encode exactly one pin. */
	assert_param(pin_mask != 0 && (pin_mask & (pin_mask - 1)) == 0);

	return (GPIO->PORT[0].GPIO_DDR & pin_mask) ? GPIO_Mode_OUT : GPIO_Mode_IN;
}

u8 PAD_PullCtrlGet(u8 PinName)
{
	/* PADCTR[] has 72 entries; PinName must be a valid GPIO pin index. */
	assert_param(PinName < ARRAY_SIZE(PINMUX->PADCTR));

	u32 Temp = PINMUX->PADCTR[PinName];

	if ((Temp & PAD_BIT_GPIOx_PU) && !(Temp & PAD_BIT_GPIOx_PD)) {
		return GPIO_PuPd_UP;
	} else if (!(Temp & PAD_BIT_GPIOx_PU) && (Temp & PAD_BIT_GPIOx_PD)) {
		return GPIO_PuPd_DOWN;
	}

	return GPIO_PuPd_NOPULL;
}
