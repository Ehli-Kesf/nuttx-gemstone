/****************************************************************************
 * arch/arm/src/am67/am67_mcspi.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>
#include <syslog.h>

#include "am67_mcspi.h"
#include "am67_pinmux.h"
#include "arm_internal.h"
#include "sctlr.h"

#ifdef CONFIG_AM67_MCSPI0

/****************************************************************************
 * Private Function Prototypes (board-provided)
 ****************************************************************************/

void am67_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                     bool selected);
uint8_t am67_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM67_MCSPI_MAX_DIVIDER      4096u

/* Longest wait for one status bit. One byte at the slowest clock in use
 * (1 MHz) takes 8 us. Measured with the PMU cycle counter; the R5F runs at
 * 800 MHz (799.93 MHz measured on the O1).
 */

#define AM67_MCSPI_CPU_HZ           800000000u
#define AM67_MCSPI_TIMEOUT_US       100u
#define AM67_MCSPI_TIMEOUT_CYCLES   (AM67_MCSPI_TIMEOUT_US * \
                                     (AM67_MCSPI_CPU_HZ / 1000000u))

/* Backstop in case the cycle counter ever stops: each poll is at least one
 * MCU-domain register read, so this is well above the time limit.
 */

#define AM67_MCSPI_POLL_MAX         1000000u

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am67_mcspi_dev_s
{
  struct spi_dev_s spidev;
  uint32_t         base;
  mutex_t          lock;
  uint8_t          channel;
  uint32_t         frequency;
  uint8_t          mode;
  uint8_t          nbits;
  uint32_t         chconf;
  uint32_t         chctrl;
  bool             selected;
  uint32_t         errors;     /* Failures since am67_mcspi_take_errors() */
  uint32_t         reset_at;   /* errors when the controller was last reset */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int       am67_mcspi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t  am67_mcspi_setfrequency(FAR struct spi_dev_s *dev,
                                         uint32_t frequency);
static void      am67_mcspi_setmode(FAR struct spi_dev_s *dev,
                                    enum spi_mode_e mode);
static void      am67_mcspi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t  am67_mcspi_send(FAR struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void      am67_mcspi_exchange(FAR struct spi_dev_s *dev,
                                     FAR const void *txbuffer,
                                     FAR void *rxbuffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_am67_spi0ops =
{
  .lock         = am67_mcspi_lock,
  .select       = am67_spi0select,
  .setfrequency = am67_mcspi_setfrequency,
  .setmode      = am67_mcspi_setmode,
  .setbits      = am67_mcspi_setbits,
  .status       = am67_spi0status,
  .send         = am67_mcspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = am67_mcspi_exchange,
#endif
};

static struct am67_mcspi_dev_s g_spi0dev =
{
  .spidev =
    {
      &g_am67_spi0ops
    },
  .base       = AM67_MCSPI0_BASE,
  .lock       = NXMUTEX_INITIALIZER,
  .channel    = 0,
  .frequency  = 1000000,
  .mode       = SPIDEV_MODE3,
  .nbits      = 8,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t am67_mcspi_getreg(uint32_t base, uint32_t offset)
{
  return getreg32(base + offset);
}

static inline void am67_mcspi_putreg(uint32_t base, uint32_t offset,
                                     uint32_t value)
{
  putreg32(value, base + offset);
}

static inline FAR struct am67_mcspi_dev_s *
am67_mcspi_dev(FAR struct spi_dev_s *dev)
{
  return (FAR struct am67_mcspi_dev_s *)dev;
}

static bool am67_mcspi_waitreg(uint32_t addr, uint32_t mask)
{
  const uint32_t start = (uint32_t)up_perf_gettime();
  uint32_t polls = 0;

  while ((getreg32(addr) & mask) == 0)
    {
      if ((uint32_t)up_perf_gettime() - start > AM67_MCSPI_TIMEOUT_CYCLES ||
          ++polls >= AM67_MCSPI_POLL_MAX)
        {
          return false;
        }
    }

  return true;
}

static bool am67_mcspi_waitstat(uint32_t base, uint8_t channel,
                                uint32_t mask)
{
  return am67_mcspi_waitreg(base + AM67_MCSPI_CHSTAT0 +
                            AM67_MCSPI_CH_OFFSET(channel), mask);
}

static void am67_mcspi_channel_enable(FAR struct am67_mcspi_dev_s *priv,
                                      bool enable)
{
  uint32_t chctrl = priv->chctrl;

  if (enable)
    {
      chctrl |= AM67_MCSPI_CHCTRL_EN;
    }
  else
    {
      chctrl &= ~AM67_MCSPI_CHCTRL_EN;
    }

  priv->chctrl = chctrl;
  am67_mcspi_putreg(priv->base,
                    AM67_MCSPI_CHCTRL0 + AM67_MCSPI_CH_OFFSET(priv->channel),
                    chctrl);
}

static void am67_mcspi_cs_force(FAR struct am67_mcspi_dev_s *priv,
                                bool deassert)
{
  uint32_t chconf = priv->chconf;

  if (deassert)
    {
      /* Wait for EOT so CS isn't dropped mid-word at high SCLK, which
       * truncates writes (e.g. the ICM-20948 bank-switch).
       */

      if (!am67_mcspi_waitstat(priv->base, priv->channel,
                               AM67_MCSPI_CHSTAT_EOT))
        {
          priv->errors++;
        }

      chconf &= ~AM67_MCSPI_CHCONF_FORCE;  /* FORCE=0: CS deasserted (high) */
    }
  else
    {
      chconf |= AM67_MCSPI_CHCONF_FORCE;   /* FORCE=1: CS asserted (low, EPOL=1) */
    }

  priv->chconf = chconf;
  am67_mcspi_putreg(priv->base,
                    AM67_MCSPI_CHCONF0 + AM67_MCSPI_CH_OFFSET(priv->channel),
                    chconf);

  /* Keep the channel enabled between transfers; toggling it re-inits the
   * shift logic and can drop the next transfer's first word. CS is driven
   * by FORCE above, so an idle enabled channel generates no clock.
   */

  if (!deassert)
    {
      am67_mcspi_channel_enable(priv, true);
    }
}

static uint8_t am67_mcspi_calc_divisor(uint32_t speed_hz,
                                       uint32_t ref_clk_hz)
{
  uint8_t clkd;

  for (clkd = 0; clkd < 15; clkd++)
    {
      if (speed_hz >= (ref_clk_hz >> (clkd + 1)))
        {
          break;
        }
    }

  return clkd;
}

static void am67_mcspi_apply_hwconfig(FAR struct am67_mcspi_dev_s *priv)
{
  uint32_t ref_clk = CONFIG_AM67_MCSPI0_FCLK;
  uint32_t speed_hz = priv->frequency;
  uint32_t chconf;
  uint32_t chctrl = priv->chctrl & AM67_MCSPI_CHCTRL_EN; /* preserve EN bit */
  uint8_t clkd;
  uint32_t div;
  uint32_t extclk = 0;

  if (speed_hz > ref_clk)
    {
      speed_hz = ref_clk;
    }

  if (speed_hz < (ref_clk / AM67_MCSPI_MAX_DIVIDER))
    {
      clkd = am67_mcspi_calc_divisor(speed_hz, ref_clk);
      speed_hz = ref_clk >> (clkd + 1);
      chconf = 0;
      chctrl &= ~AM67_MCSPI_CHCTRL_EXTCLK_MASK;
    }
  else
    {
      div = (ref_clk + speed_hz - 1) / speed_hz;
      speed_hz = ref_clk / div;
      clkd = (div - 1) & 0x0fu;
      extclk = (div - 1) >> 4;
      chconf = AM67_MCSPI_CHCONF_CLKG;
      chctrl &= ~AM67_MCSPI_CHCTRL_EXTCLK_MASK;
      chctrl |= (extclk << AM67_MCSPI_CHCTRL_EXTCLK_SHIFT) &
                AM67_MCSPI_CHCTRL_EXTCLK_MASK;
    }

  priv->frequency = speed_hz;

  chconf |= AM67_MCSPI_CHCONF_IS;    /* IS=1: receive from D1 (MISO) */
  chconf &= ~AM67_MCSPI_CHCONF_DPE0; /* DPE0=0: TX enabled on D0 (MOSI) */
  chconf |= AM67_MCSPI_CHCONF_DPE1;  /* DPE1=1: TX disabled on D1 */
  chconf |= AM67_MCSPI_CHCONF_EPOL;  /* EPOL=1: CS active-low */
  chconf &= ~AM67_MCSPI_CHCONF_WL_MASK;
  chconf |= ((uint32_t)(priv->nbits - 1) << AM67_MCSPI_CHCONF_WL_SHIFT);
  chconf &= ~AM67_MCSPI_CHCONF_CLKD_MASK;
  chconf |= ((uint32_t)clkd << AM67_MCSPI_CHCONF_CLKD_SHIFT);

  if ((priv->mode & SPIDEV_MODE2) != 0)
    {
      chconf |= AM67_MCSPI_CHCONF_POL;
    }
  else
    {
      chconf &= ~AM67_MCSPI_CHCONF_POL;
    }

  if ((priv->mode & SPIDEV_MODE1) != 0)
    {
      chconf |= AM67_MCSPI_CHCONF_PHA;
    }
  else
    {
      chconf &= ~AM67_MCSPI_CHCONF_PHA;
    }

  /* SPIENSLV: route this channel to its natural CS pin (chN uses CSN) */

  chconf &= ~AM67_MCSPI_CHCONF_SPIENSLV_MASK;
  chconf |= ((uint32_t)priv->channel << AM67_MCSPI_CHCONF_SPIENSLV_SHIFT) &
             AM67_MCSPI_CHCONF_SPIENSLV_MASK;

  /* Re-assert FORCE if the channel is currently selected so that
   * reconfiguring mid-transaction does not glitch the CS line.
   */

  if (priv->selected)
    {
      chconf |= AM67_MCSPI_CHCONF_FORCE;
    }

  priv->chconf = chconf;
  priv->chctrl = chctrl;

  am67_mcspi_putreg(priv->base,
                    AM67_MCSPI_CHCONF0 + AM67_MCSPI_CH_OFFSET(priv->channel),
                    chconf);
  am67_mcspi_putreg(priv->base,
                    AM67_MCSPI_CHCTRL0 + AM67_MCSPI_CH_OFFSET(priv->channel),
                    chctrl);

  if (priv->selected)
    {
      am67_mcspi_channel_enable(priv, true);
    }
}

static void am67_mcspi_clear_other_force(FAR struct am67_mcspi_dev_s *priv,
                                        uint8_t keep)
{
  uint8_t ch;

  /* SINGLE=1 allows FORCE on only one channel. Clearing CHCTRL.EN on
   * the channel we leave does not clear its FORCE bit, so that CS can
   * stay asserted while the new channel is selected.
   */

  for (ch = 0; ch < AM67_MCSPI_NCHANNELS; ch++)
    {
      uint32_t off;
      uint32_t conf;

      if (ch == keep)
        {
          continue;
        }

      off = AM67_MCSPI_CHCONF0 + AM67_MCSPI_CH_OFFSET(ch);
      conf = am67_mcspi_getreg(priv->base, off);

      if ((conf & AM67_MCSPI_CHCONF_FORCE) == 0)
        {
          continue;
        }

      conf &= ~AM67_MCSPI_CHCONF_FORCE;
      am67_mcspi_putreg(priv->base, off, conf);

      if (ch == priv->channel)
        {
          priv->chconf &= ~AM67_MCSPI_CHCONF_FORCE;
        }
    }
}

static void am67_mcspi_select_channel(FAR struct am67_mcspi_dev_s *priv,
                                      uint8_t channel)
{
  if (priv->channel == channel)
    {
      return;
    }

  am67_mcspi_channel_enable(priv, false);
  priv->channel = channel;
  am67_mcspi_apply_hwconfig(priv);
}

static void am67_mcspi_controller_init(FAR struct am67_mcspi_dev_s *priv)
{
  uint32_t regval;
  uint32_t modulctrl;
  uint8_t ch;

  /* The status waits time out on the PMU cycle counter; make sure it
   * counts (idempotent, same setup as up_perf_init()).
   */

  cp15_pmu_pmcr(PMCR_E);
  cp15_pmu_cesr(PMCESR_CCES);

  /* Set HL_SYSCONFIG to no-idle so the OCP interconnect does not gate
   * the functional clock while we poll status registers.
   */

  am67_mcspi_putreg(priv->base, AM67_MCSPI_HL_SYSCONFIG,
                    AM67_MCSPI_HL_SYSCONFIG_NOIDLE);

  /* Trigger the hardware soft reset */

  regval = am67_mcspi_getreg(priv->base, AM67_MCSPI_SYSCONFIG);
  regval |= AM67_MCSPI_SYSCONFIG_SOFTRESET;
  am67_mcspi_putreg(priv->base, AM67_MCSPI_SYSCONFIG, regval);

  /* Wait until SYSSTATUS.RESETDONE is 1. On timeout carry on: the
   * transfers then time out and are reported as errors.
   */

  if (!am67_mcspi_waitreg(priv->base + AM67_MCSPI_SYSSTATUS,
                          AM67_MCSPI_SYSSTATUS_RESETDONE))
    {
      spierr("ERROR: soft reset timeout\n");
      priv->errors++;
    }

  /* Configure CLOCKACTIVITY and SIDLEMODE */

  regval = AM67_MCSPI_SYSCONFIG_CLKACT_BOTH |
           AM67_MCSPI_SYSCONFIG_SIDLEMODE_NO;
  am67_mcspi_putreg(priv->base, AM67_MCSPI_SYSCONFIG, regval);

  /* SINGLE=1: one channel active at a time; CS controlled via FORCE bit */

  modulctrl = AM67_MCSPI_MODULCTRL_SINGLE;
  am67_mcspi_putreg(priv->base, AM67_MCSPI_MODULCTRL, modulctrl);

  /* After reset every CHCONF has EPOL=0, which drives an idle CS low.
   * The devices on this bus are active-low, so an unused channel would
   * keep its device selected and let it drive MISO against the one
   * being read. Park all four chip-selects high before any pad is
   * muxed to the controller.
   */

  for (ch = 0; ch < AM67_MCSPI_NCHANNELS; ch++)
    {
      am67_mcspi_putreg(priv->base,
                        AM67_MCSPI_CHCONF0 + AM67_MCSPI_CH_OFFSET(ch),
                        AM67_MCSPI_CHCONF_EPOL | AM67_MCSPI_CHCONF_IS |
                        AM67_MCSPI_CHCONF_DPE1);
    }

  am67_mcspi_apply_hwconfig(priv);
}

/* Shift one word. On a timeout count the error and return false; the
 * caller stops the transfer, since every later word would time out too.
 */

static bool am67_mcspi_transfer_word(FAR struct am67_mcspi_dev_s *priv,
                                     uint32_t wd, FAR uint32_t *rd)
{
  uint32_t choff = AM67_MCSPI_CH_OFFSET(priv->channel);

  if (!am67_mcspi_waitstat(priv->base, priv->channel,
                           AM67_MCSPI_CHSTAT_TXS))
    {
      spierr("ERROR: TX timeout ch%u\n", priv->channel);
      priv->errors++;
      return false;
    }

  am67_mcspi_putreg(priv->base, AM67_MCSPI_TX0 + choff, wd);

  if (!am67_mcspi_waitstat(priv->base, priv->channel,
                           AM67_MCSPI_CHSTAT_RXS))
    {
      spierr("ERROR: RX timeout ch%u\n", priv->channel);
      priv->errors++;
      return false;
    }

  *rd = am67_mcspi_getreg(priv->base, AM67_MCSPI_RX0 + choff);
  return true;
}

/****************************************************************************
 * SPI driver methods
 ****************************************************************************/

static int am67_mcspi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct am67_mcspi_dev_s *priv = am67_mcspi_dev(dev);

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

static uint32_t am67_mcspi_setfrequency(FAR struct spi_dev_s *dev,
                                        uint32_t frequency)
{
  FAR struct am67_mcspi_dev_s *priv = am67_mcspi_dev(dev);

  priv->frequency = frequency;
  am67_mcspi_apply_hwconfig(priv);
  return priv->frequency;
}

static void am67_mcspi_setmode(FAR struct spi_dev_s *dev,
                               enum spi_mode_e mode)
{
  FAR struct am67_mcspi_dev_s *priv = am67_mcspi_dev(dev);

  priv->mode = (uint8_t)mode;
  am67_mcspi_apply_hwconfig(priv);
}

static void am67_mcspi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct am67_mcspi_dev_s *priv = am67_mcspi_dev(dev);

  DEBUGASSERT(nbits == 8 || nbits == 16);
  priv->nbits = (uint8_t)nbits;
  am67_mcspi_apply_hwconfig(priv);
}

static uint32_t am67_mcspi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct am67_mcspi_dev_s *priv = am67_mcspi_dev(dev);
  uint32_t rd = 0;

  if (!priv->selected)
    {
      priv->errors++;
      return 0;
    }

  am67_mcspi_transfer_word(priv, wd, &rd);
  return rd;
}

#ifdef CONFIG_SPI_EXCHANGE
static void am67_mcspi_exchange(FAR struct spi_dev_s *dev,
                                FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords)
{
  FAR struct am67_mcspi_dev_s *priv = am67_mcspi_dev(dev);
  uint32_t rd;
  size_t i;

  if (!priv->selected)
    {
      priv->errors++;
      return;
    }

  if (priv->nbits > 8)
    {
      FAR const uint16_t *tx16 = txbuffer;
      FAR uint16_t *rx16 = rxbuffer;

      for (i = 0; i < nwords; i++)
        {
          uint32_t wd = (tx16 != NULL) ? (uint32_t)tx16[i] : 0xffffu;

          if (!am67_mcspi_transfer_word(priv, wd, &rd))
            {
              return;
            }

          if (rx16 != NULL)
            {
              rx16[i] = (uint16_t)rd;
            }
        }
    }
  else
    {
      FAR const uint8_t *tx8 = txbuffer;
      FAR uint8_t *rx8 = rxbuffer;

      for (i = 0; i < nwords; i++)
        {
          uint32_t wd = (tx8 != NULL) ? (uint32_t)tx8[i] : 0xffu;

          if (!am67_mcspi_transfer_word(priv, wd, &rd))
            {
              return;
            }

          if (rx8 != NULL)
            {
              rx8[i] = (uint8_t)rd;
            }
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void am67_spiinitialize(void)
{
  /* Controller first: the chip-selects must be parked inactive before
   * the pads are handed to MCSPI.
   */

  am67_mcspi_controller_init(&g_spi0dev);
  am67_spi_pinmux_init();
  spiinfo("MCU_MCSPI0 @ 0x%08" PRIx32 " rev=0x%08" PRIx32 "\n",
          g_spi0dev.base,
          am67_mcspi_getreg(g_spi0dev.base, AM67_MCSPI_REVISION));
}

FAR struct spi_dev_s *am67_spibus_initialize(int port)
{
  if (port != 0)
    {
      return NULL;
    }

  return &g_spi0dev.spidev;
}

void am67_mcspi_board_select(FAR struct spi_dev_s *dev, uint8_t channel,
                             bool selected)
{
  FAR struct am67_mcspi_dev_s *priv = am67_mcspi_dev(dev);

  if (selected)
    {
      am67_mcspi_clear_other_force(priv, channel);
      am67_mcspi_select_channel(priv, channel);
      am67_mcspi_cs_force(priv, false);
      priv->selected = true;
    }
  else
    {
      am67_mcspi_cs_force(priv, true);
      priv->selected = false;

      /* A word timed out: the controller may be wedged (clock gated, left
       * in slave mode, FIFO state lost). Reset it so the next transfer
       * starts clean; this also parks every chip-select.
       */

      if (priv->errors != priv->reset_at)
        {
          am67_mcspi_controller_init(priv);
          priv->reset_at = priv->errors;
        }
    }
}

int am67_mcspi_take_errors(FAR struct spi_dev_s *dev)
{
  FAR struct am67_mcspi_dev_s *priv = am67_mcspi_dev(dev);
  uint32_t errors = priv->errors;

  priv->errors = 0;
  priv->reset_at = 0;
  return errors > INT_MAX ? INT_MAX : (int)errors;
}

#endif /* CONFIG_AM67_MCSPI0 */
