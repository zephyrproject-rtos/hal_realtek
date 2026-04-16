/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "rtl_i2c.h"
#include "rtl_rcc.h"

/*============================================================================*
 *                           Private Functions
 *============================================================================*/
/* I2C Data Cmd Mask */
#define I2C_CMD_READ_BIT                             (BIT8)
#define I2C_CMD_STOP_BIT                             (BIT9)
#define I2C_CMD_RESTART_BIT                          (BIT10)

/* SDA Setup Time */
#define I2C_SDA_SETUP_TIME_VALUE                     (2)

/* SCL Low/High Period Time */
#define I2C_SCL_HIGH_PERIOD_COMPENSATE                          (7)
#define I2C_SCL_LOW_PERIOD_COMPENSATE                           (1)
#define NS_PER_SECOND                                           (1000000000)
/* I2C SCL High/Low min period according to spec */
#define I2C_FAST_MODE_PLUS_SCL_LOW_PERIOD_MIN_NS                (500)
#define I2C_FAST_MODE_PLUS_SCL_HIGH_PERIOD_MIN_NS               (260)
#define I2C_FAST_MODE_SCL_LOW_PERIOD_MIN_NS                     (1300)
#define I2C_FAST_MODE_SCL_HIGH_PERIOD_MIN_NS                    (600)
#define I2C_STANDARD_MODE_SCL_LOW_PERIOD_MIN_NS                 (4700)
#define I2C_STANDARD_MODE_SCL_HIGH_PERIOD_MIN_NS                (4000)
/* SDA Hold Time */
#define I2C_STANDARD_FAST_MODE_SDA_HOLD_TIME_NS                 (600)
#define I2C_FAST_MODE_PLUS_SDA_HOLD_TIME_NS                     (300)

#define ROUNDUP(a, b)                                (((a + b - 1) / b))

uint32_t I2C_TimeOut = 0xFFF;
static uint32_t I2C_RisingTimeNs;
extern uint8_t I2C_GetCompIndex(I2C_TypeDef *I2Cx);

/*============================================================================*
 *                           Public Functions
 *============================================================================*/
/**
  * \brief  Deinitializes the I2Cx peripheral registers to their default reset values.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \return None
  */
void I2C_DeInit(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    /*Disable I2C IP*/
    switch ((uint32_t)I2Cx)
    {
#ifdef I2C0
    case (uint32_t)I2C0:
        RCC_PeriphClockCmd(APBPeriph_I2C0, APBPeriph_I2C0_CLOCK, DISABLE);
        break;
#endif
#ifdef I2C1
    case (uint32_t)I2C1:
        RCC_PeriphClockCmd(APBPeriph_I2C1, APBPeriph_I2C1_CLOCK, DISABLE);
        break;
#endif
#ifdef I2C2
    case (uint32_t)I2C2:
        RCC_PeriphClockCmd(APBPeriph_I2C2, APBPeriph_I2C2_CLOCK, DISABLE);
        break;
#endif
#ifdef I2C3
    case (uint32_t)I2C3:
        RCC_PeriphClockCmd(APBPeriph_I2C3, APBPeriph_I2C3_CLOCK, DISABLE);
        break;
#endif

    default:
        break;
    }
}

/**
  * \brief  Initializes the I2Cx peripheral according to the specified
  *         parameters in the I2C_InitStruct.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  I2C_InitStruct: Pointer to a I2C_InitTypeDef structure that
  *         contains the configuration information for the specified I2C peripheral.
  * \return None
  */
void I2C_Init(I2C_TypeDef *I2Cx, I2C_InitTypeDef *I2C_InitStruct)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(IS_I2C_CLOCK_SPEED(I2C_InitStruct->I2C_ClockSpeed));

    /* Disable I2C device before change configuration */
    IC_ENABLE_TypeDef i2c_0x6c = {.d32 = I2Cx->IC_ENABLE};
    i2c_0x6c.b.ic_enable = 0x0;
    I2Cx->IC_ENABLE = i2c_0x6c.d32;

    /* Config sda tx hold time in different clock speed */
    uint8_t sda_tx_hold = 0;
    uint8_t compensate_index = I2C_GetCompIndex(I2Cx);
    uint32_t I2CSrcClk = 0;

    if (compensate_index == 6)
    {
        I2CSrcClk = 1000000;
    }
    else if (compensate_index == 7)
    {
        I2CSrcClk = 625000;
    }
    else
    {
        I2CSrcClk = 40000000 / BIT(compensate_index);
    }
    uint32_t i2c_source_clock_ns = NS_PER_SECOND / I2CSrcClk;

    if ((I2CSrcClk == 40000000) && (I2C_InitStruct->I2C_ClockSpeed > 400000))
    {
        sda_tx_hold = I2C_FAST_MODE_PLUS_SDA_HOLD_TIME_NS / i2c_source_clock_ns;
    }
    else
    {
        sda_tx_hold = (I2C_STANDARD_FAST_MODE_SDA_HOLD_TIME_NS / i2c_source_clock_ns == 0) ? 1 :
                      (I2C_STANDARD_FAST_MODE_SDA_HOLD_TIME_NS / i2c_source_clock_ns);
    }

    /* ------------------------------ Initialize I2C device ------------------------------*/
    if (I2C_DeviveMode_Master == I2C_InitStruct->I2C_DeviveMode)
    {
        /* Configure I2C device mode which can be selected for master or slave */
        IC_CON_TypeDef i2c_0x00 = {.d32 = I2Cx->IC_CON};
        i2c_0x00.b.master_mode = 0x1;
        i2c_0x00.b.ic_slave_disable = 0x1;
        i2c_0x00.b.ic_restart_en = 0x1;
        I2Cx->IC_CON = i2c_0x00.d32;

        /* Set target address */
        IC_TAR_TypeDef i2c_0x04 = {.d32 = I2Cx->IC_TAR};
        i2c_0x04.b.ic_10bitaddr_master = I2C_InitStruct->I2C_AddressMode;
        i2c_0x04.b.ic_tar = I2C_InitStruct->I2C_SlaveAddress;
        I2Cx->IC_TAR = i2c_0x04.d32;

        /* Set SDA hold time in master mode */
        IC_SDA_HOLD_TypeDef i2c_0x7c = {.d32 = I2Cx->IC_SDA_HOLD};
        i2c_0x7c.b.ic_sda_tx_hold = sda_tx_hold;
        i2c_0x7c.b.ic_sda_rx_hold = 0x1;
        I2Cx->IC_SDA_HOLD = i2c_0x7c.d32;

        I2C_RisingTimeNs = I2C_InitStruct->I2C_RisingTimeNs;
        /* Configure I2C speed */
        I2C_SetClockSpeed(I2Cx, I2C_InitStruct->I2C_ClockSpeed);
    }
    else
    {
        /* Set to slave mode */
        IC_CON_TypeDef i2c_0x00 = {.d32 = I2Cx->IC_CON};
        i2c_0x00.b.master_mode = 0x0;
        i2c_0x00.b.ic_10bitaddr_slave = I2C_InitStruct->I2C_AddressMode;
        i2c_0x00.b.ic_slave_disable = 0x0;
        I2Cx->IC_CON = i2c_0x00.d32;

        /* Set slave address */
        IC_SAR_TypeDef i2c_0x08 = {.d32 = I2Cx->IC_SAR};
        i2c_0x08.b.ic_sar = I2C_InitStruct->I2C_SlaveAddress;
        I2Cx->IC_SAR = i2c_0x08.d32;

        /* Set Ack in slave mode */
        IC_ACK_GENERAL_CALL_TypeDef i2c_0x98 = {.d32 = I2Cx->IC_ACK_GENERAL_CALL};
        i2c_0x98.b.ack_gen_call &= I2C_InitStruct->I2C_Ack;
        I2Cx->IC_ACK_GENERAL_CALL = i2c_0x98.d32;

        /* Set SDA hold time in slave mode */
        IC_SDA_HOLD_TypeDef i2c_0x7c = {.d32 = I2Cx->IC_SDA_HOLD};
        i2c_0x7c.b.ic_sda_tx_hold = sda_tx_hold;
        i2c_0x7c.b.ic_sda_rx_hold = 0x08;
        I2Cx->IC_SDA_HOLD = i2c_0x7c.d32;

        /* Set SDA setup time delay only in slave transmitter mode(greater than 2) ,delay time:[(IC_SDA_SETUP - 1) * (ic_clk_period)]*/
        IC_SDA_SETUP_TypeDef i2c_0x94 = {.d32 = I2Cx->IC_SDA_SETUP};
        i2c_0x94.b.sda_setup = I2C_SDA_SETUP_TIME_VALUE;
        I2Cx->IC_SDA_SETUP = i2c_0x94.d32;
    }

    /* Set Tx empty level */
    IC_TX_TL_TypeDef i2c_0x3c = {.d32 = I2Cx->IC_TX_TL};
    i2c_0x3c.b.tx_tl = I2C_InitStruct->I2C_TxThresholdLevel;
    I2Cx->IC_TX_TL = i2c_0x3c.d32;

    /*set Rx full level*/
    IC_RX_TL_TypeDef i2c_0x40 = {.d32 = I2Cx->IC_RX_TL};
    i2c_0x40.b.rx_tl = I2C_InitStruct->I2C_RxThresholdLevel;
    I2Cx->IC_RX_TL = i2c_0x40.d32;

    /* Config I2C dma mode */
    IC_DMA_CR_TypeDef i2c_0x88 = {.d32 = I2Cx->IC_DMA_CR};
    i2c_0x88.b.rdmae = I2C_InitStruct->I2C_RxDmaEn;
    i2c_0x88.b.tdmae = I2C_InitStruct->I2C_TxDmaEn;
    I2Cx->IC_DMA_CR = i2c_0x88.d32;

    /* Config I2C waterlevel */
    IC_DMA_TDLR_TypeDef i2c_0x8c = {.d32 = I2Cx->IC_DMA_TDLR};
    IC_DMA_RDLR_TypeDef i2c_0x90 = {.d32 = I2Cx->IC_DMA_RDLR};
    i2c_0x8c.b.dmatdl = I2C_InitStruct->I2C_TxWaterlevel;
    i2c_0x90.b.dmardl = I2C_InitStruct->I2C_RxWaterlevel;
    I2Cx->IC_DMA_TDLR = i2c_0x8c.d32;
    I2Cx->IC_DMA_RDLR = i2c_0x90.d32;

    /* Mask all interrupt */
    I2Cx->IC_INTR_MASK = 0x0;
}

/**
  * \brief  Fills each I2C_InitStruct member with its default value.
  * \param  I2C_InitStruct : pointer to a I2C_InitTypeDef structure which will be initialized.
  * \return None
  */
void I2C_StructInit(I2C_InitTypeDef *I2C_InitStruct)
{
    /* I2C source clock is 40MHz, depending on clock divider */
    I2C_InitStruct->I2C_Clock             = 40000000;
    /* I2C SCK freqency is 400KHz */
    I2C_InitStruct->I2C_ClockSpeed        = 400000;
    /* Master mode */
    I2C_InitStruct->I2C_DeviveMode        = I2C_DeviveMode_Master;
    /* 7-bit address mode */
    I2C_InitStruct->I2C_AddressMode       = I2C_AddressMode_7BIT;
    /* Set slave address */
    I2C_InitStruct->I2C_SlaveAddress      = 0;
    /* Enable acknowledge in slave mode */
    I2C_InitStruct->I2C_Ack               = ENABLE;
    /* Tx fifo depth: 24 * 8bits */
    I2C_InitStruct->I2C_TxThresholdLevel  = 0;
    /* Rx fifo depth: 24 * 8bits */
    I2C_InitStruct->I2C_RxThresholdLevel  = 0;
    /* Disable dma */
    I2C_InitStruct->I2C_TxDmaEn           = DISABLE;
    I2C_InitStruct->I2C_RxDmaEn           = DISABLE;
    /* I2C Tx waterlevel, should be less than fifo threshold.
       The best value is I2C FIFO Depth - GDMA Msize */
    I2C_InitStruct->I2C_TxWaterlevel      = 23;
    /* I2C Rx waterlevel, should be less than fifo threshold.
       The best value is GDMA Msize */
    I2C_InitStruct->I2C_RxWaterlevel      = 1;
    /* Specify I2C scl rising time with 2.2K ohm pull up resistor, the unit is ns*/
    I2C_InitStruct->I2C_RisingTimeNs     = 50;

}

/**
  * \brief  Enable or disable the specified I2C peripheral.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  NewState: New state of the I2Cx peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * \return None
  */
void I2C_Cmd(I2C_TypeDef *I2Cx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    /* Enable or disable the selected I2C peripheral */
    IC_ENABLE_TypeDef i2c_0x6c = {.d32 = I2Cx->IC_ENABLE};
    i2c_0x6c.b.ic_enable = NewState;
    I2Cx->IC_ENABLE = i2c_0x6c.d32;
}


/**
  * \brief  Checks whether the last I2Cx abort status.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \return I2C_Status: The status of I2Cx. \ref I2C_Status
  */
I2C_Status I2C_CheckAbortStatus(I2C_TypeDef *I2Cx)
{
    uint32_t abort_status = 0;

    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    /* Get abort status */
    abort_status = I2Cx->IC_TX_ABRT_SOURCE;

    if (abort_status & MS_ALL_ABORT)
    {
        /* Clear abort status */
        (void)I2Cx->IC_CLR_TX_ABRT;

        /* Check abort type */
        if (abort_status & ABRT_TXDATA_NOACK)
        {
            return I2C_ABRT_TXDATA_NOACK;
        }

        if (abort_status & ABRT_7B_ADDR_NOACK)
        {
            return I2C_ABRT_7B_ADDR_NOACK;
        }

        if (abort_status & ARB_LOST)
        {
            return I2C_ARB_LOST;
        }

        if (abort_status & ABRT_MASTER_DIS)
        {
            return I2C_ABRT_MASTER_DIS;
        }

        if (abort_status & ABRT_10ADDR1_NOACK)
        {
            return I2C_ABRT_10ADDR1_NOACK;
        }

        if (abort_status & ABRT_10ADDR2_NOACK)
        {
            return I2C_ABRT_10ADDR2_NOACK;
        }
    }

    return I2C_Success;
}

I2C_Status I2C_WaitForFIFO(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG)
{
    uint32_t timeout = I2C_TimeOut;

    /* Wait until the FIFO condition selected by I2C_FLAG becomes true:
     * - TX: writable (TX FIFO Not Full, space available)
     * - RX: readable (RX FIFO Not Empty, data available)
     */
    while (((I2Cx->IC_STATUS & I2C_FLAG) == 0) && (timeout != 0))
    {
        I2C_Status status = I2C_CheckAbortStatus(I2Cx);
        if (status != I2C_Success)
        {
            return status;
        }

        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }

    return I2C_CheckAbortStatus(I2Cx);
}

I2C_Status I2C_WaitTXIdle(I2C_TypeDef *I2Cx)
{
    uint32_t timeout = I2C_TimeOut;

    /* Wait until the transmit is fully completed:
     * - Bus becomes idle (ACTIVITY == 0)
     * - TX FIFO is empty (TFE == 1)
     */
    while ((((I2Cx->IC_STATUS & I2C_FLAG_ACTIVITY) != 0) ||
            ((I2Cx->IC_STATUS & I2C_FLAG_TFE) == 0)) &&
           (timeout != 0))
    {
        I2C_Status status = I2C_CheckAbortStatus(I2Cx);
        if (status != I2C_Success)
        {
            return status;
        }

        if (--timeout == 0)
        {
            return I2C_ERR_TIMEOUT;
        }
    }

    return I2C_Success;
}


/**
  * \brief  Send data in master mode through the I2Cx peripheral.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  pBuf: Byte to be transmitted.
  * \param  len: Data length to send.
  * \return I2C_Status: The status of I2Cx. \ref I2C_Status
  */
I2C_Status I2C_MasterWrite(I2C_TypeDef *I2Cx, uint8_t *pBuf, uint16_t len)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    I2C_Status status = I2C_Success;

    for (uint16_t i = 0; i < len; i++)
    {
        uint32_t cmd = *pBuf++;
        if (i == (len - 1))
        {
            cmd |= I2C_CMD_STOP_BIT;
        }
        I2Cx->IC_DATA_CMD = cmd;

        status = I2C_WaitForFIFO(I2Cx, I2C_FLAG_TFNF);
        if (status != I2C_Success)
        {
            return status;
        }
    }

    return I2C_WaitTXIdle(I2Cx);
}

/**
  * \brief  Send device write data in master mode through the I2Cx peripheral.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  Data: Byte to be transmitted..
  * \return None
  */
I2C_Status I2C_MasterWriteDevice(I2C_TypeDef *I2Cx, uint8_t *pBuf, uint16_t len, uint8_t *pbuf2,
                                 uint32_t len2)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    I2C_Status status = I2C_Success;

    /* Write in the DR register the data to be sent */
    for (uint16_t i = 0; i < len; i++)
    {
        I2Cx->IC_DATA_CMD = *pBuf++;

        status = I2C_WaitForFIFO(I2Cx, I2C_FLAG_TFNF);
        if (status != I2C_Success)
        {
            return status;
        }
    }

    for (uint16_t i = 0; i < len2; i++)
    {
        uint32_t cmd = *pbuf2++;
        if (i == (len2 - 1))
        {
            cmd |= I2C_CMD_STOP_BIT;
        }
        I2Cx->IC_DATA_CMD = cmd;

        status = I2C_WaitForFIFO(I2Cx, I2C_FLAG_TFNF);
        if (status != I2C_Success)
        {
            return status;
        }
    }

    return I2C_WaitTXIdle(I2Cx);
}

/**
  * \brief  Read data in master mode through the I2Cx peripheral.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  pBuf: Data buffer to receive data.
  * \param  len: Read data length.
  * \return I2C_Status: The status of I2Cx. \ref I2C_Status
  */
I2C_Status I2C_MasterRead(I2C_TypeDef *I2Cx, uint8_t *pBuf, uint16_t len)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    I2C_Status status = I2C_Success;

    if (len == 0)
    {
        return status;
    }

    /* Issue the first read cmd. If only one byte is requested, append STOP immediately. */
    uint32_t cmd = 0x0 | I2C_CMD_READ_BIT;
    if (len == 1)
    {
        cmd |= I2C_CMD_STOP_BIT;
    }
    I2Cx->IC_DATA_CMD = cmd;

    for (uint16_t i = 1; i < len; i++)
    {
        /* Issue the remaining read cmd. */
        cmd = 0x0 | I2C_CMD_READ_BIT;
        if (i == (len - 1))
        {
            cmd |= I2C_CMD_STOP_BIT;
        }
        I2Cx->IC_DATA_CMD = cmd;

        /* Read byte produced by the read. */
        status = I2C_WaitForFIFO(I2Cx, I2C_FLAG_RFNE);
        if (status != I2C_Success)
        {
            return status;
        }
        *pBuf++ = (uint8_t)I2Cx->IC_DATA_CMD;
    }

    /* Read the final byte produced by the last command. */
    status = I2C_WaitForFIFO(I2Cx, I2C_FLAG_RFNE);
    if (status != I2C_Success)
    {
        return status;
    }
    *pBuf = (uint8_t)I2Cx->IC_DATA_CMD;

    return status;
}

/**
  * \brief  Sends data and read data in master mode through the I2Cx peripheral.Attention:Read data with time out mechanism.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  pWriteBuf: Data buffer to send before read.
  * \param  Writelen: Send data length.
  * \param  pReadBuf: Data buffer to receive.
  * \param  Readlen: Receive data length.
  * \return I2C_Status: The status of I2Cx. \ref I2C_Status
  */
I2C_Status I2C_RepeatRead(I2C_TypeDef *I2Cx, uint8_t *pWriteBuf, uint16_t Writelen,
                          uint8_t *pReadBuf, uint16_t Readlen)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    I2C_Status status = I2C_Success;

    for (uint16_t i = 0; i < Writelen; i++)
    {
        I2Cx->IC_DATA_CMD = *pWriteBuf++;

        status = I2C_WaitForFIFO(I2Cx, I2C_FLAG_TFNF);
        if (status != I2C_Success)
        {
            return status;
        }
    }

    for (uint16_t i = 0; i < Readlen; i++)
    {
        /* Issue the read cmd. */
        uint32_t cmd = 0x0 | I2C_CMD_READ_BIT;
        if (i == (Readlen - 1))
        {
            cmd |= I2C_CMD_STOP_BIT;
        }
        I2Cx->IC_DATA_CMD = cmd;

        /* Read byte produced by the read. */
        status = I2C_WaitForFIFO(I2Cx, I2C_FLAG_RFNE);
        if (status != I2C_Success)
        {
            return status;
        }
        *pReadBuf++ = (uint8_t)I2Cx->IC_DATA_CMD;
    }


    return status;
}

/**
  * \brief Mask the specified I2C interrupt.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  I2C_INT: This parameter can be one of the following values:
  *         \arg I2C_INT_MST_ON_HOLD: Indicates whether a master is holding the bus.
  *         \arg I2C_INT_GEN_CALL: Set only when a General Call address is received and it is acknowledged.
  *         \arg I2C_INT_START_DET: Indicates whether a START or RESTART condition has occurred on the I2C
  *              interface regardless of whether I2C is operating in slave or master mode.
  *         \arg I2C_INT_STOP_DET: Indicates whether a STOP condition has occurred on the I2C interface regardless
  *              of whether I2C is operating in slave or master mode
  *         \arg I2C_INT_ACTIVITY: This bit captures I2C activity and stays set until it is cleared.
  *         \arg I2C_INT_RX_DONE: When the I2C is acting as a slave-transmitter, this bit is set to 1 if the
  *              master does not acknowledge a transmitted byte. This occurs on the last byte of
  *              the transmission, indicating that the transmission is done.
  *         \arg I2C_INT_TX_ABRT: This bit indicates if I2C as an I2C transmitter, is unable to complete the
  *              intended actions on the contents of the transmit FIFO.
  *         \arg I2C_INT_RD_REQ: This bit is set to 1 when acting as a slave and another I2C master
  *              is attempting to read data.
  *         \arg I2C_INT_TX_EMPTY: This bit is set to 1 when the transmit buffer is at or below the threshold value set
  *              in the IC_TX_TL register.
  *         \arg I2C_INT_TX_OVER: Set during transmit if the transmit buffer is filled to IC_TX_BUFFER_DEPTH and
  *              the processor attempts to issue another I2C command.
  *         \arg I2C_INT_RX_FULL: Set when the receive buffer reaches or goes above the RX_TL threshold in the
  *              IC_RX_TL register
  *         \arg I2C_INT_RX_OVER: Set if the receive buffer is completely filled to IC_RX_BUFFER_DEPTH and an
  *              additional byte is received from an external I2C device.
  *         \arg I2C_INT_RX_UNDER: Set if the processor attempts to read the receive buffer when it is empty by reading.
  * \return None.
  */
void I2C_INTConfig(I2C_TypeDef *I2Cx, uint16_t I2C_INT, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(I2C_GET_INT(I2C_INT));
    assert_param(IS_FUNCTIONAL_STATE(NewState));

    if (NewState != DISABLE)
    {
        /* Enable the selected I2C interrupts */
        I2Cx->IC_INTR_MASK |= I2C_INT;
    }
    else
    {
        /* Disable the selected I2C interrupts */
        I2Cx->IC_INTR_MASK &= (uint16_t)~I2C_INT;
    }
}

/**
  * \brief clear the specified I2C interrupt.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  I2C_INT: This parameter can be one of the following values:
  *         \arg I2C_INT_GEN_CALL: Set only when a General Call address is received and it is acknowledged.
  *         \arg I2C_INT_START_DET: Indicates whether a START or RESTART condition has occurred on the I2C
  *              interface regardless of whether I2C is operating in slave or master mode.
  *         \arg I2C_INT_STOP_DET: Indicates whether a STOP condition has occurred on the I2C interface regardless
  *              of whether I2C is operating in slave or master mode
  *         \arg I2C_INT_ACTIVITY: This bit captures I2C activity and stays set until it is cleared.
  *         \arg I2C_INT_RX_DONE: When the I2C is acting as a slave-transmitter, this bit is set to 1 if the
  *              master does not acknowledge a transmitted byte. This occurs on the last byte of
  *              the transmission, indicating that the transmission is done.
  *         \arg I2C_INT_TX_ABRT: This bit indicates if I2C as an I2C transmitter, is unable to complete the
  *              intended actions on the contents of the transmit FIFO.
  *         \arg I2C_INT_RD_REQ: This bit is set to 1 when acting as a slave and another I2C master
  *              is attempting to read data.
  *         \arg I2C_INT_TX_EMPTY: This bit is set to 1 when the transmit buffer is at or below the threshold value set
  *              in the IC_TX_TL register.
  *         \arg I2C_INT_TX_OVER: Set during transmit if the transmit buffer is filled to IC_TX_BUFFER_DEPTH and
  *              the processor attempts to issue another I2C command.
  *         \arg I2C_INT_RX_FULL: Set when the receive buffer reaches or goes above the RX_TL threshold in the
  *              IC_RX_TL register
  *         \arg I2C_INT_RX_OVER: Set if the receive buffer is completely filled to IC_RX_BUFFER_DEPTH and an
  *              additional byte is received from an external I2C device.
  *         \arg I2C_INT_RX_UNDER: Set if the processor attempts to read the receive buffer when it is empty by reading.
  * \return None.
  */
void I2C_ClearINTPendingBit(I2C_TypeDef *I2Cx, uint16_t I2C_IT)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(I2C_GET_INT(I2C_IT));

    switch (I2C_IT)
    {
    case I2C_INT_RX_UNDER:
        {
            (void)I2Cx->IC_CLR_RX_UNDER;
            break;
        }
    case I2C_INT_RX_OVER:
        {
            (void)I2Cx->IC_CLR_RX_OVER;
            break;
        }
    case I2C_INT_TX_OVER:
        {
            (void)I2Cx->IC_CLR_TX_OVER;
            break;
        }
    case I2C_INT_RD_REQ:
        {
            (void)I2Cx->IC_CLR_RD_REQ;
            break;
        }
    case I2C_INT_TX_ABRT:
        {
            (void)I2Cx->IC_CLR_TX_ABRT;
            break;
        }
    case I2C_INT_RX_DONE:
        {
            (void)I2Cx->IC_CLR_RX_DONE;
            break;
        }
    case I2C_INT_ACTIVITY:
        {
            (void)I2Cx->IC_CLR_ACTIVITY;
            break;
        }
    case I2C_INT_STOP_DET:
        {
            (void)I2Cx->IC_CLR_STOP_DET;
            break;
        }
    case I2C_INT_START_DET:
        {
            (void)I2Cx->IC_CLR_START_DET;
            break;
        }
    case I2C_INT_GEN_CALL:
        {
            (void)I2Cx->IC_CLR_GEN_CALL;
            break;
        }
    default:
        {
            break;
        }
    }
}

/**
 * \brief   Set slave device address.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \param   Address: Specifies the slave address which will be transmitte.
 * \return  None.
 */
void I2C_SetSlaveAddress(I2C_TypeDef *I2Cx, uint16_t Address)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    /* Configure new target address */
    IC_TAR_TypeDef i2c_0x04 = {.d32 = I2Cx->IC_TAR};
    i2c_0x04.b.ic_tar = Address;
    I2Cx->IC_TAR = i2c_0x04.d32;
}

/**
 * \brief   Write command through the I2Cx peripheral.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \param   command: Command of write or read.
 *          \arg I2C_READ_CMD: Read command. Data which want to transmit can be 0 in this situation.
 *          \arg I2C_WRITE_CMD: Write command.
 * \param   data: Data which to be transmitted.
 * \param   StopState: Whether send stop signal.
 *          \arg ENABLE: Send stop signal.
 *          \arg DISABLE: Do not send stop signal.
 * \return  None.
 */
void I2C_SendCmd(I2C_TypeDef *I2Cx, I2CSendCommend_TypeDef command, uint8_t data,
                 FunctionalState StopState)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(IS_I2C_CMD(command));

    IC_DATA_CMD_TypeDef i2c_0x10 = {.d32 = 0};
    i2c_0x10.d32 = 0;
    i2c_0x10.b.cmd = command;
    i2c_0x10.b.stop = StopState;
    i2c_0x10.b.dat = data;
    I2Cx->IC_DATA_CMD = i2c_0x10.d32;
}

/**
 * \brief   Received data by the I2Cx peripheral.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \return  Return the most recent received data.
 */
uint8_t I2C_ReceiveData(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    /* Return the data in the DR register */
    return (uint8_t)I2Cx->IC_DATA_CMD;
}

/**
 * \brief   Get data length in Rx FIFO of the I2Cx peripheral.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \return  Current data number in Rx FIFO.
 */
uint8_t I2C_GetRxFIFOLen(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    return (uint8_t)I2Cx->IC_RXFLR;
}

/**
 * \brief   Get data length in Tx FIFO of the I2Cx peripheral.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \return  Current data number in Tx FIFO.
 */
uint8_t I2C_GetTxFIFOLen(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    return (uint8_t)I2Cx->IC_TXFLR;
}

/**
 * \brief   Clear all I2C interrupt.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \return  None.
 */
void I2C_ClearAllINT(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    (void)I2Cx->IC_CLR_INTR;
}

/**
 * \brief   Check whether the specified I2C flag is set.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \param   I2C_FLAG: Specifies the flag to check.
 *          This parameter can be one of the following values:
 *          \arg I2C_FLAG_SLV_HOLD_RX_FIFO_FULL: The BUS Hold in Slave mode due to the Rx FIFO being Full and an additional byte being received.
 *          \arg I2C_FLAG_SLV_HOLD_TX_FIFO_EMPTY: The BUS Hold in Slave mode for the Read request when the Tx FIFO is empty.
 *          \arg I2C_FLAG_MST_HOLD_RX_FIFO_FULL: The BUS Hold in Master mode due to Rx FIFO is full and additional byte has been received.
 *          \arg I2C_FLAG_MST_HOLD_TX_FIFO_EMPTY: The BUS hold when the master holds the bus because of the Tx FIFO being empty.
 *          \arg I2C_FLAG_SLV_ACTIVITY: Slave FSM activity status.
 *          \arg I2C_FLAG_MST_ACTIVITY: Master FSM activity status.
 *          \arg I2C_FLAG_RFF: Receive FIFO completely full.
 *          \arg I2C_FLAG_RFNE: Receive FIFO not empty.
 *          \arg I2C_FLAG_TFE: Transmit FIFO completely empty.
 *          \arg I2C_FLAG_TFNF: Transmit FIFO not full.
 *          \arg I2C_FLAG_ACTIVITY: I2C activity status.
 */
FlagStatus I2C_GetFlagState(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG)
{
    FlagStatus bit_status = RESET;

    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(IS_I2C_GET_FLAG(I2C_FLAG));

    if ((I2Cx->IC_STATUS & I2C_FLAG) != (uint32_t)RESET)
    {
        /* I2C_FLAG is set */
        bit_status = SET;
    }

    /* Return the I2C_FLAG status */
    return  bit_status;
}

/**
 * \brief   Check whether the last I2Cx event is equal to the one passed as parameter.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \param   I2C_EVENT: specifies the event to be checked about I2C Transmit Abort Status Register.
 *          This parameter can be one of the following values:
 *          \arg ABRT_SLVRD_INTX: When the processor side responds to a slave mode request for data to be transmitted to a remote master and user send read command.
 *          \arg ABRT_SLV_ARBLOST: Slave lost the bus while transmitting data to a remote master.
 *          \arg ABRT_SLVFLUSH_TXFIFO: Slave has received a read command and some data exists in the TX FIFO so the slave issues a TX_ABRT interrupt to flush old data in TX FIFO.
 *          \arg ARB_LOST: Master has lost arbitration or the slave transmitter has lost arbitration.
 *          \arg ABRT_MASTER_DIS: User tries to initiate a Master operation with the Master mode disabled
 *          \arg ABRT_10B_RD_NORSTRT: The restart is disabled and the master sends a read command in 10-bit addressing mode.
 *          \arg ABRT_SBYTE_NORSTRT: The restart is disabled and the user is trying to send a START Byte.
 *          \arg ABRT_HS_NORSTRT: The restart is disabled and the user is trying to use the master to transfer data in High Speed mode.
 *          \arg ABRT_SBYTE_ACKDET: Master has sent a START Byte and the START Byte was acknowledged (wrong behavior).
 *          \arg ABRT_HS_ACKDET: Master is in High Speed mode and the High Speed Master code was acknowledged (wrong behavior).
 *          \arg ABRT_GCALL_READ: Sent a General Call but the user programmed the byte following the General Call to be a read from the bus.
 *          \arg ABRT_GCALL_NOACK: Sent a General Call and no slave on the bus acknowledged the General Call.
 *          \arg ABRT_TXDATA_NOACK: Master sent data byte(s) following the address, it did not receive an acknowledge from the remote slave.
 *          \arg ABRT_10ADDR2_NOACK: Master is in 10-bit address mode and the second address byte of the 10-bit address was not acknowledged by any slave.
 *          \arg ABRT_10ADDR1_NOACK: Master is in 10-bit address mode and the first 10-bit address byte was not acknowledged by any slave.
 *          \arg ABRT_7B_ADDR_NOACK: Master is in 7-bit addressing mode and th address sent was not acknowledged by any slave.
 * \return  An ErrorStatus enumeration value.
 *          \retval  SUCCESS: Last event is equal to the I2C_EVENT.
 *          \retval  ERROR: Last event is different from the I2C_EVENT.
 */
FlagStatus I2C_CheckEvent(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT)
{
    FlagStatus bit_status = RESET;

    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(IS_I2C_EVENT(I2C_EVENT));

    if ((I2Cx->IC_TX_ABRT_SOURCE & I2C_EVENT) != (uint32_t)RESET)
    {

        bit_status = SET;
    }

    /* Return the I2C event status */
    return  bit_status;
}

/**
 * \brief   Get the specified I2C interrupt status.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \param   I2C_IT: This parameter can be one of the following values:
 *          \arg I2C_INT_MST_ON_HOLD: Indicates whether a master is holding the bus.
 *          \arg I2C_INT_GEN_CALL: Set only when a General Call address is received and it is acknowledged.
 *          \arg I2C_INT_START_DET: Indicates whether a START or RESTART condition has occurred on the I2C
 *               interface regardless of whether DW_apb_i2c is operating in slave or master mode.
 *          \arg I2C_INT_STOP_DET: Indicates whether a STOP condition has occurred on the I2C interface regardless
 *               of whether DW_apb_i2c is operating in slave or master mode
 *          \arg I2C_INT_ACTIVITY: This bit captures DW_apb_i2c activity and stays set until it is cleared.
 *          \arg I2C_INT_RX_DONE: When the DW_apb_i2c is acting as a slave-transmitter, this bit is set to 1 if the
 *               master does not acknowledge a transmitted byte. This occurs on the last byte of
 *               the transmission, indicating that the transmission is done.
 *          \arg I2C_INT_TX_ABRT: This bit indicates if DW_apb_i2c, as an I2C transmitter, is unable to complete the
 *               intended actions on the contents of the transmit FIFO.
 *          \arg I2C_INT_RD_REQ: This bit is set to 1 when acting as a slave and another I2C master
 *               is attempting to read data.
 *          \arg I2C_INT_TX_EMPTY: This bit is set to 1 when the transmit buffer is at or below the threshold value set
 *               in the IC_TX_TL register.
 *          \arg I2C_INT_TX_OVER: Set during transmit if the transmit buffer is filled to IC_TX_BUFFER_DEPTH and
 *               the processor attempts to issue another I2C command.
 *          \arg I2C_INT_RX_FULL: Set when the receive buffer reaches or goes above the RX_TL threshold in the
 *               IC_RX_TL register
 *          \arg I2C_INT_RX_OVER: Set if the receive buffer is completely filled to IC_RX_BUFFER_DEPTH and an
 *               additional byte is received from an external I2C device.
 *          \arg I2C_INT_RX_UNDER: Set if the processor attempts to read the receive buffer when it is empty by reading.
 * \return  The new state of I2C_IT (SET or RESET).
 */
ITStatus I2C_GetINTStatus(I2C_TypeDef *I2Cx, uint32_t I2C_IT)
{
    ITStatus bit_status = RESET;

    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(I2C_GET_INT(I2C_IT));

    if ((I2Cx->IC_INTR_STAT & I2C_IT) != (uint32_t)RESET)
    {
        bit_status = SET;
    }

    /* Return the I2C_IT status */
    return  bit_status;
}

/**
 * \brief   Enable or disable the I2Cx GDMA interface.
 * \param   I2Cx: Select the I2C peripheral. \ref I2C_Declaration
 * \param   I2C_GDMAReq: Specifies the I2C GDMA transfer request to be enabled or disabled.
 *          This parameter can be one of the following values:
 *          \arg I2C_GDMAReq_Tx: Tx buffer GDMA transfer request.
 *          \arg I2C_GDMAReq_Rx: Rx buffer GDMA transfer request.
 * \param   NewState: New state of the selected I2C GDMA transfer request.
 *          This parameter can be: ENABLE or DISABLE.
 * \return None.
 */
void I2C_GDMACmd(I2C_TypeDef *I2Cx, I2CGdmaTransferRequests_TypeDef I2C_GDMAReq,
                 FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));
    assert_param(IS_FUNCTIONAL_STATE(NewState));
    assert_param(IS_I2C_GDMAREQ(I2C_GDMAReq));

    if (NewState != DISABLE)
    {
        /* Enable the selected I2C GDMA request */
        I2Cx->IC_DMA_CR |= I2C_GDMAReq;
    }
    else
    {
        /* Disable the selected I2C GDMA request */
        I2Cx->IC_DMA_CR &= (uint16_t)~(I2C_GDMAReq);
    }
}

/**
  * \brief  Set Clock Speed through the I2Cx peripheral.
  * \param  I2Cx: Select the I2C peripheral. \ref I2C_Declaration
  * \param  I2C_ClockSpeed: CLock speed.
  * \return None
  */
void I2C_SetClockSpeed(I2C_TypeDef *I2Cx, uint32_t I2C_ClockSpeed)
{
    uint8_t compensate_index = 0;
    uint32_t tick_clksrc_ns = 0;
    uint32_t I2CSrcClk = 0;
    uint32_t allowedSpeed = 0;
    uint32_t spklen = 0;
    uint32_t scl_low_period_spec_min_ns = 0;
    uint32_t scl_high_period_spec_min_ns = 0;
    uint32_t scl_lcnt_min = 0;
    uint32_t scl_hcnt_min = 0;
    uint32_t scl_low_period_min_ns = 0;
    uint32_t scl_high_period_min_ns = 0;
    uint32_t tick_clkspeed_ns = 0;
    uint32_t total_period_ns = 0;
    uint32_t total_hcnt_lcnt = 0;
    int32_t remaining = 0;
    uint32_t scl_lcnt = 0;
    uint32_t scl_hcnt = 0;

    /* Obtain the I2C Clock source based on the I2C divider. */
    compensate_index = I2C_GetCompIndex(I2Cx);
    if (compensate_index == 6)
    {
        I2CSrcClk = 1000000;
    }
    else if (compensate_index == 7)
    {
        I2CSrcClk = 625000;
    }
    else
    {
        I2CSrcClk = 40000000 / BIT(compensate_index);
    }

    /* Tick of I2C clock source in ns. */
    tick_clksrc_ns = NS_PER_SECOND / I2CSrcClk;

    /* Limit the maximum I2C speed according to the I2C clock source. */
    if (I2CSrcClk == 40000000)
    {
        allowedSpeed = 1000000;
    }
    else if (I2CSrcClk == 20000000)
    {
        allowedSpeed = 400000;
    }
    else
    {
        allowedSpeed = 100000;
    }
    /* Set Max I2C Clock speed. */
    I2C_ClockSpeed = (I2C_ClockSpeed > allowedSpeed) ? allowedSpeed : I2C_ClockSpeed;

    /* Set the spike time according to SPEC */
    /* For Standard mode, there is no limit on spike suppression time, use default value. */
    if (I2C_ClockSpeed <= 100000)
    {
        spklen = 0x05;
    }
    /* For Fast mode (plus), the max spike suppression time is 50ns. */
    else
    {
        spklen = 0x01;
    }

    /* Set suppression limit */
    I2Cx->IC_FS_SPKLEN = spklen & 0xFFFF;


    /******************* Calulate the SCL low high count based on SPEC *******************/
    /* Get the minimum of scl low high period specified in the spec */
    if (I2C_ClockSpeed <= 100000)
    {
        scl_low_period_spec_min_ns = I2C_STANDARD_MODE_SCL_LOW_PERIOD_MIN_NS;
        scl_high_period_spec_min_ns = I2C_STANDARD_MODE_SCL_HIGH_PERIOD_MIN_NS;
    }
    else if (I2C_ClockSpeed <= 400000)
    {
        scl_low_period_spec_min_ns = I2C_FAST_MODE_SCL_LOW_PERIOD_MIN_NS;
        scl_high_period_spec_min_ns = I2C_FAST_MODE_SCL_HIGH_PERIOD_MIN_NS;
    }
    else
    {
        scl_low_period_spec_min_ns = I2C_FAST_MODE_PLUS_SCL_LOW_PERIOD_MIN_NS;
        scl_high_period_spec_min_ns = I2C_FAST_MODE_PLUS_SCL_HIGH_PERIOD_MIN_NS;
    }

    /* Calulate the minimum of scl low high period based on LCNT/HCNT specified in hardware:
     *
     * Firstly, get the minimum LCNT HCNT specified in hardware.
     * - LCNT: IC_SS_SCL_LCNT and IC_FS_SCL_LCNT register values must be larger than IC_FS_SPKLEN + 7.
     * - HCNT: IC_SS_SCL_HCNT and IC_FS_SCL_HCNT register values must be larger than IC_FS_SPKLEN + 5.
     * Use +1 to enforce strictly greater with integer registers.
     *
     * Secondly, calulate the minimum of scl high/low period based on the LCNT/HCNT.
     */

    scl_lcnt_min = spklen + 7 + 1;
    scl_hcnt_min = spklen + 5 + 1;
    scl_low_period_min_ns = (scl_lcnt_min + I2C_SCL_LOW_PERIOD_COMPENSATE) * tick_clksrc_ns;
    scl_high_period_min_ns = (scl_hcnt_min + spklen + I2C_SCL_HIGH_PERIOD_COMPENSATE) *
                             tick_clksrc_ns;

    /* Final minimum: satisfy both spec and hardware. */
    scl_low_period_min_ns = (scl_low_period_min_ns > scl_low_period_spec_min_ns) ?
                            scl_low_period_min_ns : scl_low_period_spec_min_ns;
    scl_high_period_min_ns = (scl_high_period_min_ns > scl_high_period_spec_min_ns) ?
                             scl_high_period_min_ns : scl_high_period_spec_min_ns;

    scl_lcnt_min = ROUNDUP(scl_low_period_min_ns, tick_clksrc_ns) - I2C_SCL_LOW_PERIOD_COMPENSATE;
    scl_hcnt_min = ROUNDUP(scl_high_period_min_ns,
                           tick_clksrc_ns) - spklen - I2C_SCL_HIGH_PERIOD_COMPENSATE;

    /*************** Calulate the SCL low high count based on I2C_Clock_Speed ***************/

    /* Calculate target total (high+low) counter based on the desired frequency of I2C SCL.
     * Firstly, calulate the total period: tick_clkspeed_ns minus I2C_RisingTimeNs.
     *   - Notes: I2C_RisingTimeNs should be below the range.
     * Secondly, total_period_ns div tick_clksrc_ns, round up.
     * Then subtract SPKLEN and path compensations to get pure HCNT+LCNT sum.
     */

    tick_clkspeed_ns = NS_PER_SECOND / I2C_ClockSpeed;
    total_period_ns = tick_clkspeed_ns - I2C_RisingTimeNs;
    total_hcnt_lcnt = (ROUNDUP(total_period_ns, tick_clksrc_ns)
                       - spklen - I2C_SCL_HIGH_PERIOD_COMPENSATE - I2C_SCL_LOW_PERIOD_COMPENSATE);

    /***************** Distibute the remaining count to the SCL low high *****************/

    /* Calculate the remaining as total_hcnt_lcnt - (scl_lcnt_min + scl_hcnt_min),
     * distribute the remaining counts between low and high count.
     */

    remaining = total_hcnt_lcnt - scl_lcnt_min - scl_hcnt_min;
    scl_lcnt = scl_lcnt_min;
    scl_hcnt = scl_hcnt_min;
    if (remaining > 0)
    {
        scl_lcnt = scl_lcnt_min + (uint32_t)(remaining / 2);
        scl_hcnt = total_hcnt_lcnt - scl_lcnt;
    }

    /* Set speed mode and write counters to responsing registers */
    IC_CON_TypeDef i2c_ctrl = {.d32 = I2Cx->IC_CON};
    if (I2C_ClockSpeed <= 100000)
    {
        /*Configure I2C speed in standard mode*/
        i2c_ctrl.b.speed = 0x1;
        I2Cx->IC_CON = i2c_ctrl.d32;
        I2Cx->IC_SS_SCL_LCNT = scl_lcnt & 0xFFFF;
        I2Cx->IC_SS_SCL_HCNT = scl_hcnt & 0xFFFF;
    }
    else
    {
        /* Configure I2C speed in fast mode or fast mode plus*/
        i2c_ctrl.b.speed = 0x2;
        I2Cx->IC_CON = i2c_ctrl.d32;
        I2Cx->IC_FS_SCL_LCNT = scl_lcnt & 0xFFFF;
        I2Cx->IC_FS_SCL_HCNT = scl_hcnt & 0xFFFF;
    }
}

#if (I2C_SUPPORT_RAP_FUNCTION == 1)

void I2C_WrapperModeCmd(I2C_TypeDef *I2Cx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    WRAP_CR_TypeDef i2c_0x10c = {.d32 = I2Cx->WRAP_CR};
    i2c_0x10c.b.wrapper_clk_en = NewState;
    I2Cx->WRAP_CR = i2c_0x10c.d32;
}

bool I2C_WrapperTransModeSet(I2C_TypeDef *I2Cx, uint32_t mode)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    WRAP_CR_TypeDef i2c_0x10c = {.d32 = I2Cx->WRAP_CR};
    i2c_0x10c.b.trans_mode = mode;
    I2Cx->WRAP_CR = i2c_0x10c.d32;
    return i2c_0x10c.b.wrapper_busy;
}

void I2C_WrapperWriteDataSet(I2C_TypeDef *I2Cx, uint8_t num, const uint8_t *buf)
{
    /* num should be less than 24 */
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    I2Cx->RAP_WCMD_NUM = num - 1;

    for (uint8_t i = 0; i < num; i++)
    {
        I2Cx->RAP_WCMD_DATA = *buf++;
    }
}

uint8_t I2C_WrapperGetTxFIFOLen(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    RAP_BUF_CR_TypeDef i2c_0x114 = {.d32 = I2Cx->RAP_BUF_CR};
    return i2c_0x114.b.wrap_buf_amount;
}

void I2C_WrapperTxFIFOClear(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    RAP_BUF_CR_TypeDef i2c_0x114 = {.d32 = I2Cx->RAP_BUF_CR};
    i2c_0x114.b.wrap_buf_clear = 1;
    I2Cx->RAP_BUF_CR = i2c_0x114.d32;
}


void I2C_WrapperReadNumSet(I2C_TypeDef *I2Cx, uint16_t num)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    I2Cx->RAP_RCMD_NUM = num - 1;
}


uint8_t I2C_WrapperReceiveData(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    /* Return the data in the DR register */
    return I2C_ReceiveData(I2Cx);
}

uint8_t I2C_WrapperGetRxFIFOLen(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    return I2C_GetRxFIFOLen(I2Cx);
}


void I2C_WrapperTransStart(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    WRAP_CR_TypeDef i2c_0x10c = {.d32 = I2Cx->WRAP_CR};
    i2c_0x10c.b.start_wrapper = 1;
    I2Cx->WRAP_CR = i2c_0x10c.d32;
}

bool I2C_WrapperBusyCheck(I2C_TypeDef *I2Cx)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    WRAP_CR_TypeDef i2c_0x10c = {.d32 = I2Cx->WRAP_CR};
    return i2c_0x10c.b.wrapper_busy;
}

void I2C_RAPModeCmd(I2C_TypeDef *I2Cx, FunctionalState NewState)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    WRAP_CR_TypeDef i2c_0x10c = {.d32 = I2Cx->WRAP_CR};
    i2c_0x10c.b.rap_mode = NewState;
    I2Cx->WRAP_CR = i2c_0x10c.d32;
}

void I2C_TaskTrigger(I2C_TypeDef *I2Cx, uint32_t Task)
{
    /* Check the parameters */
    assert_param(IS_I2C_ALL_PERIPH(I2Cx));

    I2Cx->RAP_TASK |= BIT(Task);
}

#endif
