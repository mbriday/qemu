/*
 * Nucleo 32 F303 Machine model (STM32F303 based)
 * Copyright (c) 2020 M. Briday <mikael.briday@ls2n.fr>
 *
 * based on Netduino Plus 2 Machine Model
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "hw/arm/stm32f303_soc.h"
#include "hw/arm/boot.h"
#include "hw/irq.h"

static void led_irq_handler(void *opaque, int n, int level) {
  /* There should only be one IRQ for the LED */
  assert(n == 0);

  /* Assume that the IRQ is only triggered if the LED has changed state.
   * If this is not correct, we may get multiple LED Offs or Ons in a row.
   */
  switch (level) {
  case 0:
    printf("LED Off\r");
    fflush(stdout);
    break;
  case 1:
    printf("LED On \r");
    fflush(stdout);
    break;
  }
}

static void nucleo32_f303_init(MachineState *machine)
{
    STM32F303State *dev = STM32F303_SOC(qdev_new(TYPE_STM32F303_SOC));
    qdev_prop_set_string((DeviceState*)dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE((DeviceState *)dev), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       FLASH_SIZE);

    DeviceState *gpio_b = DEVICE(&(dev->gpio[1]));
	assert(gpio_b);

	/* Connect LED to GPIO B (gpio[1]) pin 3 */
	qemu_irq *led_irq = qemu_allocate_irqs(led_irq_handler, NULL, 1);
	qdev_connect_gpio_out(gpio_b, 3, led_irq[0]);
}

static void nucleo32_f303_machine_init(MachineClass *mc)
{
    mc->desc = "Nucleo 32 (F303) Machine";
    mc->init = nucleo32_f303_init;
}

DEFINE_MACHINE("nucleo32_f303", nucleo32_f303_machine_init)
