/****************************************************************************
 * arch/arm/src/am67/am67_tisci.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ask the DMSC, over the secure proxy, to turn a device on. EPWM0/1 stay
 * powered down until somebody sends TISCI_MSG_SET_DEVICE; a powered-off
 * module reads as zero and the PWM setup bails out before pinmux.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include "arm_internal.h"

/* tisci/tisci.h uses begin_packed, which this NuttX does not define, so the
 * few constants this file needs are repeated here. */

#define TISCI_MSG_SET_DEVICE                     0x0200u
#define TISCI_MSG_FLAG_AOP                       (1u << 1)
#define TISCI_MSG_FLAG_ACK                       (1u << 1)
#define TISCI_MSG_VALUE_DEVICE_SW_STATE_ON       2u
#define TISCI_HOST_ID_MAIN_0_R5_1                38u

#define SEC_PROXY_TARGET_BASE                    0x4d000000u
#define SEC_PROXY_RT_BASE                        0x4a600000u
#define SEC_PROXY_THREAD_SIZE                    0x00001000u
/* J722S TISCI sec_proxy.html: non-secure MAIN R5 context is host
 * MAIN_0_R5_1. Thread 6 is its response (read), thread 7 its
 * low_priority (write). Threads 4/5 belong to the secure context. */
#define MAIN_R5_NS_RESPONSE_THRD_ID              6u
#define MAIN_R5_NS_WRITE_THRD_ID                 7u
#define MAIN_R5_NS_TX_ADDR                       (SEC_PROXY_TARGET_BASE + \
                                                  (MAIN_R5_NS_WRITE_THRD_ID * \
                                                   SEC_PROXY_THREAD_SIZE))
#define MAIN_R5_NS_RX_ADDR                       (SEC_PROXY_TARGET_BASE + \
                                                  (MAIN_R5_NS_RESPONSE_THRD_ID * \
                                                   SEC_PROXY_THREAD_SIZE))

#define SPROXY_DATA_FIRST  0x4u
#define SPROXY_DATA_LAST   0x3cu
#define SPROXY_COUNT_MASK  0xffu

static uint8_t g_seq;

static void sproxy_read_msg(uint32_t base)
{
  uint32_t off;

  for (off = SPROXY_DATA_FIRST; off <= SPROXY_DATA_LAST; off += 4u)
    {
      (void)getreg32(base + off);
    }
}

static int sproxy_xfer(const void *tx, size_t txlen,
                       void *rx, size_t rxlen)
{
  const uint32_t tx_base = MAIN_R5_NS_TX_ADDR;
  const uint32_t rx_base = MAIN_R5_NS_RX_ADDR;
  const uint32_t tx_rt = SEC_PROXY_RT_BASE +
                         (MAIN_R5_NS_WRITE_THRD_ID * SEC_PROXY_THREAD_SIZE);
  const uint32_t rx_rt = SEC_PROXY_RT_BASE +
                         (MAIN_R5_NS_RESPONSE_THRD_ID * SEC_PROXY_THREAD_SIZE);
  const uint8_t *src = tx;
  uint8_t *dst = rx;
  uint32_t off;
  size_t left = txlen;
  size_t got = 0;
  unsigned int spins;

  if ((getreg32(tx_rt) & SPROXY_COUNT_MASK) == 0u)
    {
      return -EBUSY;
    }

  if ((getreg32(rx_rt) & SPROXY_COUNT_MASK) != 0u)
    {
      sproxy_read_msg(rx_base);
    }

  for (off = SPROXY_DATA_FIRST; off <= SPROXY_DATA_LAST; off += 4u)
    {
      uint32_t word = 0;

      if (left > 0u)
        {
          size_t n = left > 4u ? 4u : left;

          memcpy(&word, src, n);
          src += n;
          left -= n;
        }

      putreg32(word, tx_base + off);
    }

  UP_DSB();

  for (spins = 0; spins < 200000u; spins++)
    {
      if ((getreg32(rx_rt) & SPROXY_COUNT_MASK) != 0u)
        {
          break;
        }
    }

  if ((getreg32(rx_rt) & SPROXY_COUNT_MASK) == 0u)
    {
      return -ETIMEDOUT;
    }

  for (off = SPROXY_DATA_FIRST; off <= SPROXY_DATA_LAST; off += 4u)
    {
      uint32_t word = getreg32(rx_base + off);

      if (dst != NULL && got < rxlen)
        {
          size_t n = rxlen - got;

          if (n > 4u)
            {
              n = 4u;
            }

          memcpy(dst + got, &word, n);
          got += n;
        }
    }

  return 0;
}

int am67_tisci_device_on(uint32_t id)
{
  uint8_t raw[17];
  struct
  {
    uint16_t type;
    uint8_t host;
    uint8_t seq;
    uint32_t flags;
    uint32_t id;
    uint32_t reserved;
    uint8_t state;
  } req;
  struct
  {
    uint16_t type;
    uint8_t host;
    uint8_t seq;
    uint32_t flags;
  } resp;
  int ret;

  memset(&req, 0, sizeof(req));
  memset(&resp, 0, sizeof(resp));

  req.type = TISCI_MSG_SET_DEVICE;
  req.host = TISCI_HOST_ID_MAIN_0_R5_1;
  req.seq = g_seq++;
  req.flags = TISCI_MSG_FLAG_AOP;
  req.id = id;
  req.state = TISCI_MSG_VALUE_DEVICE_SW_STATE_ON;

  /* The wire format is packed. The local struct may insert padding. */
  memcpy(raw + 0, &req.type, 2);
  raw[2] = req.host;
  raw[3] = req.seq;
  memcpy(raw + 4, &req.flags, 4);
  memcpy(raw + 8, &req.id, 4);
  memcpy(raw + 12, &req.reserved, 4);
  raw[16] = req.state;

  ret = sproxy_xfer(raw, sizeof(raw), &resp, sizeof(resp));

  if (ret < 0)
    {
      syslog(LOG_ERR, "tisci dev %" PRIu32 " on: xfer %d\n", id, ret);
      return ret;
    }

  if ((resp.flags & TISCI_MSG_FLAG_ACK) == 0u)
    {
      syslog(LOG_ERR, "tisci dev %" PRIu32 " on: nak flags 0x%" PRIx32 "\n",
             id, resp.flags);
      return -EIO;
    }

  syslog(LOG_INFO, "tisci dev %" PRIu32 " on: ack\n", id);
  return 0;
}
