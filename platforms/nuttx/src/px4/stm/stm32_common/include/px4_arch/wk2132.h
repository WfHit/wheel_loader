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

#pragma once

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <termios.h>

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>

__BEGIN_DECLS

/* WK2132 Register Definitions - Global Registers */
#define WK2132_GENA     0x00  /* Global Enable Register */
#define WK2132_GRST     0x01  /* Global Reset Register */
#define WK2132_GMUT     0x02  /* Global Master IRQ Enable */
#define WK2132_GIER     0x10  /* Global IRQ Enable Register */
#define WK2132_GIFR     0x11  /* Global IRQ Flag Register */

/* WK2132 Register Definitions - Page Select */
#define WK2132_SPAGE    0x03  /* Sub-page Select Register */

/* WK2132 Register Definitions - Sub-page 0 (UART Registers) */
#define WK2132_SCR      0x04  /* Sub-channel Control Register */
#define WK2132_LCR      0x05  /* Line Control Register */
#define WK2132_FCR      0x06  /* FIFO Control Register */
#define WK2132_SIER     0x07  /* Sub-channel IRQ Enable Register */
#define WK2132_SIFR     0x08  /* Sub-channel IRQ Flag Register */
#define WK2132_TFCNT    0x09  /* TX FIFO Count Register */
#define WK2132_RFCNT    0x0A  /* RX FIFO Count Register */
#define WK2132_FSR      0x0B  /* FIFO Status Register */
#define WK2132_LSR      0x0C  /* Line Status Register */
#define WK2132_FDAT     0x0D  /* FIFO Data Register */

/* WK2132 Register Definitions - Sub-page 1 (Baud Rate) */
#define WK2132_BAUD1    0x04  /* Baud Rate Control Register 1 */
#define WK2132_BAUD0    0x05  /* Baud Rate Control Register 0 */
#define WK2132_PRES     0x06  /* Prescaler Register */

/* GENA Register Bits */
#define WK2132_GENA_UT4EN   0x08  /* UART4 Enable */
#define WK2132_GENA_UT3EN   0x04  /* UART3 Enable */
#define WK2132_GENA_UT2EN   0x02  /* UART2 Enable */
#define WK2132_GENA_UT1EN   0x01  /* UART1 Enable */

/* LCR Register Bits */
#define WK2132_LCR_PAM1     0x80  /* Parity Mode 1 */
#define WK2132_LCR_PAM0     0x40  /* Parity Mode 0 */
#define WK2132_LCR_PAEN     0x20  /* Parity Enable */
#define WK2132_LCR_STB      0x10  /* Stop Bit */
#define WK2132_LCR_WLS1     0x02  /* Word Length Select 1 */
#define WK2132_LCR_WLS0     0x01  /* Word Length Select 0 */

/* LSR Register Bits */
#define WK2132_LSR_OE       0x08  /* Overrun Error */
#define WK2132_LSR_BI       0x04  /* Break Interrupt */
#define WK2132_LSR_FE       0x02  /* Framing Error */
#define WK2132_LSR_PE       0x01  /* Parity Error */

/* FSR Register Bits */
#define WK2132_FSR_RFOE     0x80  /* RX FIFO Overrun Error */
#define WK2132_FSR_RFBI     0x40  /* RX FIFO Break Interrupt */
#define WK2132_FSR_RFFE     0x20  /* RX FIFO Framing Error */
#define WK2132_FSR_RFPE     0x10  /* RX FIFO Parity Error */
#define WK2132_FSR_RDAT     0x08  /* RX Data Available */
#define WK2132_FSR_TDAT     0x04  /* TX Data Available */
#define WK2132_FSR_TFULL    0x02  /* TX FIFO Full */
#define WK2132_FSR_TBUSY    0x01  /* TX Busy */

/* SIER Register Bits */
#define WK2132_SIER_FERR_IEN    0x80  /* Frame Error IRQ Enable */
#define WK2132_SIER_CTS_IEN     0x40  /* CTS IRQ Enable */
#define WK2132_SIER_RTS_IEN     0x20  /* RTS IRQ Enable */
#define WK2132_SIER_XOFF_IEN    0x10  /* XOFF IRQ Enable */
#define WK2132_SIER_TFEMPTY_IEN 0x08  /* TX FIFO Empty IRQ Enable */
#define WK2132_SIER_TFTRIG_IEN  0x04  /* TX FIFO Trigger IRQ Enable */
#define WK2132_SIER_RFTRIG_IEN  0x02  /* RX FIFO Trigger IRQ Enable */
#define WK2132_SIER_RFTOUT_IEN  0x01  /* RX FIFO Timeout IRQ Enable */

/* Configuration */
#define WK2132_CRYSTAL_FREQ     14745600  /* 14.7456 MHz crystal */
#define WK2132_MAX_PORTS        4         /* Maximum 4 UART ports */
#define WK2132_FIFO_SIZE        256       /* FIFO size */

/* Default I2C configuration */
#define WK2132_I2C_FREQUENCY    400000    /* 400 kHz */
#define WK2132_DEFAULT_ADDR     0x10      /* Default I2C address */

/* I2C Address bit definitions */
#define WK2132_ADDR_REG_ACCESS  0x00      /* Register access (bit 0 = 0) */
#define WK2132_ADDR_FIFO_ACCESS 0x01      /* FIFO access (bit 0 = 1) */

/* Channel selection bits (bits 2-1) */
#define WK2132_ADDR_CH1         0x00      /* Channel 1 (bits 2-1 = 00) */
#define WK2132_ADDR_CH2         0x02      /* Channel 2 (bits 2-1 = 01) */
#define WK2132_ADDR_CH3         0x04      /* Channel 3 (bits 2-1 = 10) */
#define WK2132_ADDR_CH4         0x06      /* Channel 4 (bits 2-1 = 11) */

/* Macro to construct I2C address */
#define WK2132_MAKE_I2C_ADDR(base, channel, is_fifo) \
    ((base) | (((channel) - 1) << 1) | ((is_fifo) ? WK2132_ADDR_FIFO_ACCESS : WK2132_ADDR_REG_ACCESS))

/* Device instance data */
struct wk2132_dev_s
{
  FAR struct i2c_master_s *i2c;     /* I2C interface */
  uint8_t                  base_addr; /* I2C base address */
  uint8_t                  port;     /* UART port number (1-4) */
  uint32_t                 baud;     /* Configured baud rate */
  uint32_t                 parity;   /* Configured parity */
  uint32_t                 nbits;    /* Number of bits */
  bool                     stopbits2; /* Two stop bits */
  sem_t                    exclsem;  /* Mutual exclusion */
  bool                     enabled;  /* Port enabled flag */
};

/* Public Functions */

/**
 * Initialize a WK2132 UART port
 *
 * @param i2c        I2C master interface
 * @param base_addr  I2C base address of WK2132 chip
 * @param port       UART port number (1-4)
 * @return           UART device structure or NULL on failure
 */
FAR struct uart_dev_s *wk2132_uart_init(FAR struct i2c_master_s *i2c,
                                         uint8_t base_addr, uint8_t port);

/**
 * Register WK2132 serial devices as /dev/ttyS* devices
 *
 * @param i2c_bus       I2C bus number
 * @param i2c_base_addr I2C base address of WK2132 chip
 * @param base_tty      Base TTY number (e.g., 6 for /dev/ttyS6)
 * @param num_ports     Number of ports to register (1-4)
 * @return              OK on success, negative on failure
 */
int wk2132_register_devices(int i2c_bus, uint8_t i2c_base_addr,
                            int base_tty, int num_ports);

__END_DECLS
