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
 * @file wk2132.c
 * @author PX4 Development Team
 * @brief WK2132 I2C to Quad UART serial driver for NuttX
 *
 * This driver provides serial port functionality for the WK2132 I2C to
 * Quad UART bridge chip, exposing up to 4 additional UART ports as
 * standard /dev/ttyS* devices.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#include <arch/board/board.h>

/* Include appropriate STM32 I2C header based on variant */
#if defined(CONFIG_ARCH_CHIP_STM32H7)
#include <stm32_i2c.h>
#elif defined(CONFIG_ARCH_CHIP_STM32F7) || defined(CONFIG_ARCH_CHIP_STM32F4)
#include <stm32_i2c.h>
#else
#error "WK2132: Unsupported STM32 variant"
#endif

#include <px4_platform/wk2132.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  wk2132_setup(FAR struct uart_dev_s *dev);
static void wk2132_shutdown(FAR struct uart_dev_s *dev);
static int  wk2132_attach(FAR struct uart_dev_s *dev);
static void wk2132_detach(FAR struct uart_dev_s *dev);
static int  wk2132_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  wk2132_receive(FAR struct uart_dev_s *dev, unsigned int *status);
static void wk2132_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool wk2132_rxavailable(FAR struct uart_dev_s *dev);
static void wk2132_send(FAR struct uart_dev_s *dev, int ch);
static void wk2132_txint(FAR struct uart_dev_s *dev, bool enable);
static bool wk2132_txready(FAR struct uart_dev_s *dev);
static bool wk2132_txempty(FAR struct uart_dev_s *dev);

static int  wk2132_i2c_write(FAR struct wk2132_dev_s *priv, uint8_t reg,
                              uint8_t value);
static int  wk2132_i2c_read(FAR struct wk2132_dev_s *priv, uint8_t reg,
                             FAR uint8_t *value);
static int  wk2132_set_baud(FAR struct wk2132_dev_s *priv, uint32_t baud);
static void wk2132_poll_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UART operations */
static const struct uart_ops_s g_wk2132_uart_ops =
{
  .setup          = wk2132_setup,
  .shutdown       = wk2132_shutdown,
  .attach         = wk2132_attach,
  .detach         = wk2132_detach,
  .ioctl          = wk2132_ioctl,
  .receive        = wk2132_receive,
  .rxint          = wk2132_rxint,
  .rxavailable    = wk2132_rxavailable,
  .send           = wk2132_send,
  .txint          = wk2132_txint,
  .txready        = wk2132_txready,
  .txempty        = wk2132_txempty,
};

/* Work queue for polling */
static struct work_s g_wk2132_poll_work;
static bool g_wk2132_poll_started = false;
static FAR struct uart_dev_s *g_wk2132_devices[WK2132_MAX_PORTS];
static int g_wk2132_device_count = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Write a register via I2C
 */
static int wk2132_i2c_write(FAR struct wk2132_dev_s *priv, uint8_t reg,
                             uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  int ret;

  /* Construct the register address with port selection */
  buffer[0] = reg | ((priv->port - 1) << 4);
  buffer[1] = value;

  /* Setup I2C message */
  msg.frequency = WK2132_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  /* Perform the transfer */
  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/**
 * @brief Read a register via I2C
 */
static int wk2132_i2c_read(FAR struct wk2132_dev_s *priv, uint8_t reg,
                            FAR uint8_t *value)
{
  struct i2c_msg_s msgs[2];
  uint8_t regaddr;
  int ret;

  /* Construct the register address with port selection */
  regaddr = reg | ((priv->port - 1) << 4);

  /* Setup I2C write message for register address */
  msgs[0].frequency = WK2132_I2C_FREQUENCY;
  msgs[0].addr      = priv->addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &regaddr;
  msgs[0].length    = 1;

  /* Setup I2C read message for data */
  msgs[1].frequency = WK2132_I2C_FREQUENCY;
  msgs[1].addr      = priv->addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = value;
  msgs[1].length    = 1;

  /* Perform the transfer */
  ret = I2C_TRANSFER(priv->i2c, msgs, 2);
  return (ret >= 0) ? OK : ret;
}

/**
 * @brief Set baud rate for a UART port
 */
static int wk2132_set_baud(FAR struct wk2132_dev_s *priv, uint32_t baud)
{
  uint32_t divisor;
  uint8_t baud1, baud0, pres;
  int ret;

  /* Calculate baud rate divisor
   * Baud = Crystal_Freq / (PRES * (BAUD1*256 + BAUD0) * 16)
   * For simplicity, we'll use PRES = 1
   */
  divisor = WK2132_CRYSTAL_FREQ / (baud * 16);

  if (divisor > 0xFFFF)
    {
      /* Use prescaler */
      pres = 4;
      divisor = WK2132_CRYSTAL_FREQ / (baud * 16 * 4);
    }
  else
    {
      pres = 1;
    }

  if (divisor > 0xFFFF)
    {
      return -EINVAL;  /* Baud rate too low */
    }

  baud1 = (divisor >> 8) & 0xFF;
  baud0 = divisor & 0xFF;

  /* Select sub-page 1 for baud rate configuration */
  ret = wk2132_i2c_write(priv, WK2132_SPAGE, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Set baud rate registers */
  ret = wk2132_i2c_write(priv, WK2132_BAUD1, baud1);
  if (ret < 0)
    {
      return ret;
    }

  ret = wk2132_i2c_write(priv, WK2132_BAUD0, baud0);
  if (ret < 0)
    {
      return ret;
    }

  ret = wk2132_i2c_write(priv, WK2132_PRES, pres);
  if (ret < 0)
    {
      return ret;
    }

  /* Return to sub-page 0 */
  ret = wk2132_i2c_write(priv, WK2132_SPAGE, 0);

  return ret;
}

/**
 * @brief Polling worker function
 */
static void wk2132_poll_worker(FAR void *arg)
{
  int i;

  /* Poll all registered devices */
  for (i = 0; i < g_wk2132_device_count; i++)
    {
      FAR struct uart_dev_s *dev = g_wk2132_devices[i];
      if (dev != NULL)
        {
          /* Check for received data */
          if (wk2132_rxavailable(dev))
            {
              uart_recvchars(dev);
            }

          /* Check for transmit ready */
          if (dev->xmit.head != dev->xmit.tail && wk2132_txready(dev))
            {
              uart_xmitchars(dev);
            }
        }
    }

  /* Schedule next poll */
  if (g_wk2132_poll_started)
    {
      work_queue(LPWORK, &g_wk2132_poll_work, wk2132_poll_worker, NULL,
                 MSEC2TICK(10));  /* Poll every 10ms */
    }
}

/**
 * @brief Setup UART port
 */
static int wk2132_setup(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t lcr = 0;
  uint8_t gena;
  int ret;

  /* Take exclusive access */
  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Read current global enable register */
  ret = wk2132_i2c_read(priv, WK2132_GENA, &gena);
  if (ret < 0)
    {
      goto errout;
    }

  /* Enable the UART port in global register */
  uint8_t gena_bit = 1 << (priv->port - 1);
  gena |= gena_bit;
  ret = wk2132_i2c_write(priv, WK2132_GENA, gena);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure LCR based on settings */
  if (priv->nbits == 7)
    {
      lcr |= WK2132_LCR_WLS0;
    }
  else if (priv->nbits == 8)
    {
      lcr |= WK2132_LCR_WLS1;
    }
  else if (priv->nbits == 9)
    {
      lcr |= WK2132_LCR_WLS1 | WK2132_LCR_WLS0;
    }

  if (priv->stopbits2)
    {
      lcr |= WK2132_LCR_STB;
    }

  if (priv->parity == 1)       /* Odd parity */
    {
      lcr |= WK2132_LCR_PAEN;
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      lcr |= WK2132_LCR_PAEN | WK2132_LCR_PAM0;
    }

  ret = wk2132_i2c_write(priv, WK2132_LCR, lcr);
  if (ret < 0)
    {
      goto errout;
    }

  /* Set baud rate */
  ret = wk2132_set_baud(priv, priv->baud);
  if (ret < 0)
    {
      goto errout;
    }

  /* Enable FIFOs and reset them */
  ret = wk2132_i2c_write(priv, WK2132_FCR, 0x07);
  if (ret < 0)
    {
      goto errout;
    }

  /* Enable RX data available interrupt */
  ret = wk2132_i2c_write(priv, WK2132_SIER, WK2132_SIER_RFTRIG_IEN);
  if (ret < 0)
    {
      goto errout;
    }

  priv->enabled = true;

  /* Start polling if not already started */
  if (!g_wk2132_poll_started)
    {
      g_wk2132_poll_started = true;
      work_queue(LPWORK, &g_wk2132_poll_work, wk2132_poll_worker, NULL, 0);
    }

errout:
  nxsem_post(&priv->exclsem);
  return ret;
}

/**
 * @brief Shutdown UART port
 */
static void wk2132_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t gena;

  /* Disable all interrupts */
  wk2132_i2c_write(priv, WK2132_SIER, 0x00);

  /* Disable the UART port in global register */
  if (wk2132_i2c_read(priv, WK2132_GENA, &gena) >= 0)
    {
      uint8_t gena_bit = 1 << (priv->port - 1);
      gena &= ~gena_bit;
      wk2132_i2c_write(priv, WK2132_GENA, gena);
    }

  priv->enabled = false;
}

/**
 * @brief Attach interrupt (not used in polled mode)
 */
static int wk2132_attach(FAR struct uart_dev_s *dev)
{
  return OK;
}

/**
 * @brief Detach interrupt (not used in polled mode)
 */
static void wk2132_detach(FAR struct uart_dev_s *dev)
{
  /* Nothing to do */
}

/**
 * @brief IOCTL handler
 */
static int wk2132_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_dev_s *dev = inode->i_private;
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  int ret = OK;

  switch (cmd)
    {
    case TCGETS:
      {
        FAR struct termios *termiosp = (FAR struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return current settings */
        termiosp->c_cflag = 0;

        if (priv->nbits == 7)
          {
            termiosp->c_cflag |= CS7;
          }
        else if (priv->nbits == 8)
          {
            termiosp->c_cflag |= CS8;
          }

        if (priv->stopbits2)
          {
            termiosp->c_cflag |= CSTOPB;
          }

        if (priv->parity == 1)
          {
            termiosp->c_cflag |= PARENB | PARODD;
          }
        else if (priv->parity == 2)
          {
            termiosp->c_cflag |= PARENB;
          }

        /* Set baud rate */
        cfsetispeed(termiosp, priv->baud);
        cfsetospeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        FAR struct termios *termiosp = (FAR struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Extract new settings */
        priv->baud = cfgetispeed(termiosp);

        if ((termiosp->c_cflag & CSIZE) == CS7)
          {
            priv->nbits = 7;
          }
        else
          {
            priv->nbits = 8;
          }

        priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;

        if (termiosp->c_cflag & PARENB)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        /* Reconfigure the UART */
        wk2132_setup(dev);
      }
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/**
 * @brief Receive character
 */
static int wk2132_receive(FAR struct uart_dev_s *dev, unsigned int *status)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t fsr;
  uint8_t lsr;
  uint8_t rxdata;
  int ret;

  /* Get FIFO status */
  ret = wk2132_i2c_read(priv, WK2132_FSR, &fsr);
  if (ret < 0)
    {
      return -EIO;
    }

  /* Get line status */
  ret = wk2132_i2c_read(priv, WK2132_LSR, &lsr);
  if (ret < 0)
    {
      return -EIO;
    }

  *status = (fsr << 8) | lsr;

  /* Read data from FIFO */
  ret = wk2132_i2c_read(priv, WK2132_FDAT, &rxdata);
  if (ret < 0)
    {
      return -EIO;
    }

  return rxdata;
}

/**
 * @brief Enable/disable RX interrupt
 */
static void wk2132_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t sier;

  /* Read current interrupt enable register */
  if (wk2132_i2c_read(priv, WK2132_SIER, &sier) < 0)
    {
      return;
    }

  /* Enable/disable RX interrupt */
  if (enable)
    {
      sier |= WK2132_SIER_RFTRIG_IEN;
    }
  else
    {
      sier &= ~WK2132_SIER_RFTRIG_IEN;
    }

  wk2132_i2c_write(priv, WK2132_SIER, sier);
}

/**
 * @brief Check if RX data available
 */
static bool wk2132_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t fsr;

  if (!priv->enabled)
    {
      return false;
    }

  /* Check FIFO status register for available data */
  if (wk2132_i2c_read(priv, WK2132_FSR, &fsr) < 0)
    {
      return false;
    }

  return (fsr & WK2132_FSR_RDAT) != 0;
}

/**
 * @brief Send character
 */
static void wk2132_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;

  /* Write data to FIFO */
  wk2132_i2c_write(priv, WK2132_FDAT, (uint8_t)ch);
}

/**
 * @brief Enable/disable TX interrupt
 */
static void wk2132_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t sier;

  /* Read current interrupt enable register */
  if (wk2132_i2c_read(priv, WK2132_SIER, &sier) < 0)
    {
      return;
    }

  /* Enable/disable TX interrupt */
  if (enable)
    {
      sier |= WK2132_SIER_TFTRIG_IEN;
    }
  else
    {
      sier &= ~WK2132_SIER_TFTRIG_IEN;
    }

  wk2132_i2c_write(priv, WK2132_SIER, sier);
}

/**
 * @brief Check if TX ready
 */
static bool wk2132_txready(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t fsr;

  if (!priv->enabled)
    {
      return false;
    }

  /* Check FIFO status register */
  if (wk2132_i2c_read(priv, WK2132_FSR, &fsr) < 0)
    {
      return false;
    }

  /* Return true if TX FIFO is not full */
  return (fsr & WK2132_FSR_TFULL) == 0;
}

/**
 * @brief Check if TX empty
 */
static bool wk2132_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t fsr;

  if (!priv->enabled)
    {
      return true;
    }

  /* Check FIFO status register */
  if (wk2132_i2c_read(priv, WK2132_FSR, &fsr) < 0)
    {
      return false;
    }

  /* Return true if TX is not busy and no data available */
  return ((fsr & WK2132_FSR_TBUSY) == 0) && ((fsr & WK2132_FSR_TDAT) == 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief Initialize a WK2132 UART port
 */
FAR struct uart_dev_s *wk2132_uart_init(FAR struct i2c_master_s *i2c,
                                         uint8_t addr, uint8_t port)
{
  FAR struct wk2132_dev_s *priv;
  FAR struct uart_dev_s *dev;
  int ret;

  /* Validate parameters */
  if (!i2c || port < 1 || port > WK2132_MAX_PORTS)
    {
      return NULL;
    }

  /* Allocate device structures */
  priv = (FAR struct wk2132_dev_s *)kmm_zalloc(sizeof(struct wk2132_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  dev = (FAR struct uart_dev_s *)kmm_zalloc(sizeof(struct uart_dev_s));
  if (dev == NULL)
    {
      kmm_free(priv);
      return NULL;
    }

  /* Initialize private structure */
  priv->i2c       = i2c;
  priv->addr      = addr;
  priv->port      = port;
  priv->baud      = 115200;  /* Default baud rate */
  priv->parity    = 0;       /* No parity */
  priv->nbits     = 8;       /* 8 data bits */
  priv->stopbits2 = false;   /* 1 stop bit */
  priv->enabled   = false;

  nxsem_init(&priv->exclsem, 0, 1);

  /* Initialize public structure */
  dev->ops      = &g_wk2132_uart_ops;
  dev->priv     = priv;
  dev->isconsole = false;

  /* Allocate RX/TX buffers */
  dev->xmit.size   = 256;
  dev->xmit.buffer = (FAR char *)kmm_malloc(dev->xmit.size);
  if (dev->xmit.buffer == NULL)
    {
      goto errout;
    }

  dev->recv.size   = 256;
  dev->recv.buffer = (FAR char *)kmm_malloc(dev->recv.size);
  if (dev->recv.buffer == NULL)
    {
      goto errout;
    }

  /* Test if device is present by reading a register */
  uint8_t test;
  ret = wk2132_i2c_read(priv, WK2132_GENA, &test);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to detect device at 0x%02x\n", addr);
      goto errout;
    }

  /* Add to device list for polling */
  if (g_wk2132_device_count < WK2132_MAX_PORTS)
    {
      g_wk2132_devices[g_wk2132_device_count++] = dev;
    }

  syslog(LOG_INFO, "WK2132: Initialized port %d at I2C 0x%02x\n", port, addr);
  return dev;

errout:
  if (dev->xmit.buffer)
    {
      kmm_free(dev->xmit.buffer);
    }
  if (dev->recv.buffer)
    {
      kmm_free(dev->recv.buffer);
    }
  kmm_free(dev);
  kmm_free(priv);
  return NULL;
}

/**
 * @brief Register WK2132 serial devices
 */
int wk2132_register_devices(int i2c_bus, uint8_t i2c_addr,
                            int base_tty, int num_ports)
{
  FAR struct i2c_master_s *i2c;
  FAR struct uart_dev_s *devs[WK2132_MAX_PORTS];
  char devpath[16];
  int i;
  int ret;

  /* Validate parameters */
  if (num_ports < 1 || num_ports > WK2132_MAX_PORTS)
    {
      return -EINVAL;
    }

  /* Get I2C bus instance */
  i2c = stm32_i2cbus_initialize(i2c_bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "WK2132: Failed to get I2C%d\n", i2c_bus);
      return -ENODEV;
    }

  /* Initialize UART ports */
  for (i = 0; i < num_ports; i++)
    {
      devs[i] = wk2132_uart_init(i2c, i2c_addr, i + 1);
      if (devs[i] == NULL)
        {
          syslog(LOG_ERR, "WK2132: Failed to initialize port %d\n", i + 1);
          ret = -ENODEV;
          goto cleanup;
        }
    }

  /* Register UART devices */
  for (i = 0; i < num_ports; i++)
    {
      snprintf(devpath, sizeof(devpath), "/dev/ttyS%d", base_tty + i);
      ret = uart_register(devpath, devs[i]);
      if (ret < 0)
        {
          syslog(LOG_ERR, "WK2132: Failed to register %s\n", devpath);
          goto cleanup;
        }

      syslog(LOG_INFO, "WK2132: Registered %s for port %d\n", devpath, i + 1);
    }

  return OK;

cleanup:
  /* Cleanup on failure */
  for (int j = 0; j < i; j++)
    {
      if (devs[j])
        {
          /* Note: uart_unregister would be called here if it existed */
          /* For now, we'll just free the memory */
          if (devs[j]->xmit.buffer)
            {
              kmm_free(devs[j]->xmit.buffer);
            }
          if (devs[j]->recv.buffer)
            {
              kmm_free(devs[j]->recv.buffer);
            }
          kmm_free(devs[j]->priv);
          kmm_free(devs[j]);
        }
    }

  return ret;
}
