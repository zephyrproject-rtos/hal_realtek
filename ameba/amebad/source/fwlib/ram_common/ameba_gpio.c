/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

static GPIO_TypeDef *GPIO_PortAddrGet(u32 GPIO_Port)
{
	GPIO_TypeDef *GPIO = NULL;

	GPIO = PORT_AB[GPIO_Port];

	return GPIO;
}

u32 GPIO_INTStatusGet(u32 GPIO_Port)
{
	GPIO_TypeDef *GPIO = GPIO_PortAddrGet(GPIO_Port);

	return GPIO->INT_STATUS;
}

void GPIO_INTStatusClearEdge(u32 GPIO_Port)
{
	GPIO_TypeDef *GPIO = NULL;
	u32 IntStatus;

	GPIO = GPIO_PortAddrGet(GPIO_Port);

	IntStatus = GPIO->INT_STATUS;

	/* Clear pending edge interrupt */
	GPIO->PORTA_EOI = IntStatus;
}
