/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file wk2132_example_usage.c
 * @author PX4 Development Team
 * @brief Example usage of WK2132 register and FIFO functions
 *
 * This file demonstrates how to use the new WK2132 I2C addressing functions
 * that follow the proper bit layout as shown in the WK2132 specification.
 */

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdbool.h>
#include <stdint.h>

#include <px4_arch/wk2132.h>
#include <arch/board/board.h>

/**
 * @brief Example: Initialize WK2132 and configure UART1
 * 
 * This example shows how to use the new register read/write functions
 * to configure a WK2132 UART channel using the proper I2C addressing.
 */
int wk2132_configure_uart_example(void)
{
  FAR struct i2c_master_s *i2c;
  uint8_t device_addr = WK2132_I2C_ADDRESS;  /* From board_config.h */
  uint8_t uart_ch = 1;  /* UART channel 1 */
  uint8_t value;
  int ret;

  /* Get I2C bus instance */
  i2c = stm32_i2cbus_initialize(BOARD_I2CBUS_WK2132);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Example 1: Read Global Enable Register */
  ret = wk2132_reg_read(i2c, device_addr, uart_ch, WK2132_GENA, &value);
  if (ret < 0)
    {
      return ret;
    }
  
  /* Enable UART1 */
  value |= WK2132_GENA_UT1EN;
  ret = wk2132_reg_write(i2c, device_addr, uart_ch, WK2132_GENA, value);
  if (ret < 0)
    {
      return ret;
    }

  /* Example 2: Configure Line Control Register for 8N1 */
  value = WK2132_LCR_WLS1;  /* 8 data bits */
  /* No parity (PAEN=0), 1 stop bit (STB=0) */
  ret = wk2132_reg_write(i2c, device_addr, uart_ch, WK2132_LCR, value);
  if (ret < 0)
    {
      return ret;
    }

  /* Example 3: Enable FIFOs and reset them */
  ret = wk2132_reg_write(i2c, device_addr, uart_ch, WK2132_FCR, 0x07);
  if (ret < 0)
    {
      return ret;
    }

  /* Example 4: Enable RX interrupt */
  ret = wk2132_reg_write(i2c, device_addr, uart_ch, WK2132_SIER, 
                         WK2132_SIER_RFTRIG_IEN);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/**
 * @brief Example: Write data to FIFO
 * 
 * This example shows how to send data using the FIFO write function.
 */
int wk2132_send_data_example(void)
{
  FAR struct i2c_master_s *i2c;
  uint8_t device_addr = WK2132_I2C_ADDRESS;
  uint8_t uart_ch = 1;
  const char *message = "Hello WK2132!\n";
  int bytes_written;

  /* Get I2C bus instance */
  i2c = stm32_i2cbus_initialize(BOARD_I2CBUS_WK2132);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Send message using FIFO write function */
  bytes_written = wk2132_fifo_write(i2c, device_addr, uart_ch, 
                                    (const uint8_t *)message, 
                                    strlen(message));
  
  if (bytes_written < 0)
    {
      return bytes_written;  /* Error occurred */
    }

  return bytes_written;  /* Number of bytes written */
}

/**
 * @brief Example: Read data from FIFO
 * 
 * This example shows how to receive data using the FIFO read function.
 */
int wk2132_receive_data_example(uint8_t *buffer, size_t buffer_size)
{
  FAR struct i2c_master_s *i2c;
  uint8_t device_addr = WK2132_I2C_ADDRESS;
  uint8_t uart_ch = 1;
  int bytes_read;

  /* Validate parameters */
  if (!buffer || buffer_size == 0)
    {
      return -EINVAL;
    }

  /* Get I2C bus instance */
  i2c = stm32_i2cbus_initialize(BOARD_I2CBUS_WK2132);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Read data using FIFO read function */
  bytes_read = wk2132_fifo_read(i2c, device_addr, uart_ch, 
                                buffer, buffer_size);
  
  if (bytes_read < 0)
    {
      return bytes_read;  /* Error occurred */
    }

  return bytes_read;  /* Number of bytes read */
}

/**
 * @brief Example: Check UART status
 * 
 * This example shows how to read various status registers.
 */
int wk2132_check_status_example(void)
{
  FAR struct i2c_master_s *i2c;
  uint8_t device_addr = WK2132_I2C_ADDRESS;
  uint8_t uart_ch = 1;
  uint8_t fsr, lsr, rfcnt, tfcnt;
  int ret;

  /* Get I2C bus instance */
  i2c = stm32_i2cbus_initialize(BOARD_I2CBUS_WK2132);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Read FIFO Status Register */
  ret = wk2132_reg_read(i2c, device_addr, uart_ch, WK2132_FSR, &fsr);
  if (ret < 0)
    {
      return ret;
    }

  /* Read Line Status Register */
  ret = wk2132_reg_read(i2c, device_addr, uart_ch, WK2132_LSR, &lsr);
  if (ret < 0)
    {
      return ret;
    }

  /* Read RX FIFO Count */
  ret = wk2132_reg_read(i2c, device_addr, uart_ch, WK2132_RFCNT, &rfcnt);
  if (ret < 0)
    {
      return ret;
    }

  /* Read TX FIFO Count */
  ret = wk2132_reg_read(i2c, device_addr, uart_ch, WK2132_TFCNT, &tfcnt);
  if (ret < 0)
    {
      return ret;
    }

  /* Process status information */
  if (fsr & WK2132_FSR_RDAT)
    {
      /* RX data available */
    }

  if (fsr & WK2132_FSR_TFULL)
    {
      /* TX FIFO is full */
    }

  if (lsr & WK2132_LSR_PE)
    {
      /* Parity error */
    }

  return OK;
}