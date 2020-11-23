/*
 * STM32F3XX DUMMY_UART
 *
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
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/char/stm32f3xx_dummy_uart.h"

#ifndef STM_DUMMY_UART_ERR_DEBUG
#define STM_DUMMY_UART_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_DUMMY_UART_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

static void stm32f3xx_dummy_uart_reset(DeviceState *dev)
{
    STM32F3XXDUMMY_UARTState *s = STM32F3XX_DUMMY_UART(dev);
    s->dummy_uart_tdr = 0x00000000;
}

static uint64_t stm32f3xx_dummy_uart_read(void *opaque, hwaddr addr,
                                   unsigned int size) {
    //STM32F3XXDUMMY_UARTState *s = opaque;
	//do noting here
	return 0;
}

static void stm32f3xx_dummy_uart_write(void *opaque, hwaddr addr,
                       uint64_t val64, unsigned int size)
{
  STM32F3XXDUMMY_UARTState *s = opaque;

  //DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);
  //printf("DUMMY_UART Address: 0x%" HWADDR_PRIx "\n", addr);
  //fflush(stdout);

  switch (addr) {
  case DUMMY_UART_TDR:
	s->dummy_uart_tdr = val64 & 0xff;
	//printf("write DUMMY_UART DR %c",(char)(s->dummy_uart_tdr));
	//fflush(stdout);
	qemu_set_irq(s->irq, true);
	break;
  default:
    break;
  }
}

static const MemoryRegionOps stm32f3xx_dummy_uart_ops = {
    .read = stm32f3xx_dummy_uart_read,
    .write = stm32f3xx_dummy_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_stm32f3xx_dummy_uart = {
    .name = TYPE_STM32F3XX_DUMMY_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(dummy_uart_tdr, STM32F3XXDUMMY_UARTState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f3xx_dummy_uart_init(Object *obj)
{
    STM32F3XXDUMMY_UARTState *s = STM32F3XX_DUMMY_UART(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->mmio, obj, &stm32f3xx_dummy_uart_ops, s,
                          TYPE_STM32F3XX_DUMMY_UART, 0x32);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void stm32f3xx_dummy_uart_realize(DeviceState *dev, Error **errp)
{
    STM32F3XXDUMMY_UARTState *s = STM32F3XX_DUMMY_UART(dev);
    qdev_init_gpio_out(DEVICE(s), &(s->irq), 1); 	
}

static void stm32f3xx_dummy_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f3xx_dummy_uart_realize;
    dc->reset = stm32f3xx_dummy_uart_reset;
    dc->vmsd = &vmstate_stm32f3xx_dummy_uart;
}

static const TypeInfo stm32f3xx_dummy_uart_info = {
    .name          = TYPE_STM32F3XX_DUMMY_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F3XXDUMMY_UARTState),
    .instance_init = stm32f3xx_dummy_uart_init,
    .class_init    = stm32f3xx_dummy_uart_class_init,
};

static void stm32f3xx_dummy_uart_register_types(void)
{
    type_register_static(&stm32f3xx_dummy_uart_info);
}

type_init(stm32f3xx_dummy_uart_register_types)
