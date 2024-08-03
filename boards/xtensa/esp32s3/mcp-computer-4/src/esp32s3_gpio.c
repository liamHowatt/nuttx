/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_gpio.c
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

#include <sys/types.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <assert.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "esp32s3-devkit.h"
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_ESP32S3_GPIO_IRQ)
#  error "NGPIOINT is > 0 and GPIO interrupts aren't enabled"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcp_half_s
{
  struct gpio_dev_s dev;
  unsigned pin;
};

struct mcp_s
{
  struct mcp_half_s half[2];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gp_read(struct gpio_dev_s *dev, bool *value);
static int gp_write(struct gpio_dev_s *dev, bool value);
// static int gp_attach(struct gpio_dev_s *dev,
//                      pin_interrupt_t callback);
// static int gp_enable(struct gpio_dev_s *dev, bool enable);
static int gp_setpintype(FAR struct gpio_dev_s *dev,
                         enum gpio_pintype_e pintype);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gp_ops =
{
  .go_read   = gp_read,
  .go_write  = gp_write,
  .go_attach = NULL,
  .go_enable = NULL,
  .go_setpintype = gp_setpintype,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gp_read
 ****************************************************************************/

static int gp_read(struct gpio_dev_s *dev, bool *value)
{
  struct mcp_half_s *half = (struct mcp_half_s *)dev;

  DEBUGASSERT(half != NULL && value != NULL);
  gpioinfo("Reading... pin %u\n", half->pin);

  *value = esp32s3_gpioread(half->pin);
  return OK;
}

/****************************************************************************
 * Name: gp_write
 ****************************************************************************/

static int gp_write(struct gpio_dev_s *dev, bool value)
{
  gpioinfo("Writing %d\n", (int)value);
  return value == 0 ? OK : ERROR;
}

/****************************************************************************
 * Name: gp_setpintype
 ****************************************************************************/

static int gp_setpintype(FAR struct gpio_dev_s *dev,
                         enum gpio_pintype_e pintype)
{
  struct mcp_half_s *half = (struct mcp_half_s *)dev;

  DEBUGASSERT(half != NULL);
  gpioinfo("Setting type %d... pin %u\n", pintype, half->pin);

  switch (pintype) {
    case GPIO_INPUT_PIN:
      esp32s3_configgpio(half->pin, INPUT_PULLUP);
      break;
    case GPIO_OUTPUT_PIN:
      esp32s3_configgpio(half->pin, OUTPUT);
      break;
    default:
      return ERROR;
  }

  half->dev.gp_pintype = pintype;

  return OK;
}


static void register_mcp_half(struct mcp_half_s *half, unsigned idx, unsigned pin, bool is_clk)
{
  half->dev.gp_pintype = GPIO_INPUT_PIN;
  half->dev.gp_ops = &gp_ops;
  half->pin = pin;

  esp32s3_gpiowrite(pin, 0);
  esp32s3_configgpio(pin, INPUT_PULLUP);

  char name[16];
  snprintf(name, sizeof(name), "mcp%u_%s", idx, is_clk ? "clk" : "dat");

  gpio_pin_register_byname(&half->dev, name);
}

static void register_mcp(struct mcp_s *mcp, unsigned idx, unsigned clk_pin, unsigned dat_pin)
{
  register_mcp_half(&mcp->half[0], idx, clk_pin, true);
  register_mcp_half(&mcp->half[1], idx, dat_pin, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_gpio_init
 ****************************************************************************/

int esp32s3_gpio_init(void)
{
  static struct mcp_s mcp_0;
  register_mcp(&mcp_0, 0, 21, 14);
  static struct mcp_s mcp_1;
  register_mcp(&mcp_1, 1, 16, 15);

  return OK;
}

#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
