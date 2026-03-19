/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"

/**
  * @brief  Reads the specified GPIO pins.
  * @param  GPIO_Port: Specifies port number
  *          This parameter can be one of the following values:
  *		 		@arg GPIO_PORT_A: port number is A
  *		 		@arg GPIO_PORT_B: port number is B
  * @param  GPIO_Mask: One bit one GPIO pin, PortA/B:0x00000000 ~0xFFFFFFFF.
  * @note   It can config multiple pins of one port by parameter GPIO_Mask
  * @retval The value of the specified port pins
  */
u32 GPIO_PortRead(u32 GPIO_Port, u32 GPIO_Mask)
{
	GPIO_TypeDef *GPIO = NULL;
	u32 RegValue;

	GPIO = PORT_AB[GPIO_Port];

	RegValue = GPIO->EXT_PORT[0];

	RegValue &= GPIO_Mask;

	return RegValue;
}

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
