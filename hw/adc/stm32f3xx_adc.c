/*
 * STM32F3XX ADC
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
#include "hw/adc/stm32f3xx_adc.h"

#ifndef STM_ADC_ERR_DEBUG
#define STM_ADC_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_ADC_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

static void stm32f3xx_adc_reset(DeviceState *dev)
{
    STM32F3XXADCState *s = STM32F3XX_ADC(dev);
    s->adc_dr = 0x00000000;
}

static uint64_t stm32f3xx_adc_read(void *opaque, hwaddr addr,
                                   unsigned int size) {
  STM32F3XXADCState *s = opaque;

  DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);
  printf("ADC Address: 0x%" HWADDR_PRIx "\n", addr);
  fflush(stdout);

  switch (addr) {
  case ADC_ISR:
	printf("read ADC ISR");
	fflush(stdout);
    return 1 << 2 | 1 << 0; // bit2 is EOC, bit 0 is ADRDY
  case ADC_DR:
	printf("read ADC DR");
	fflush(stdout);
    return s->adc_dr;
  default:
    break;
  }

  return 0;
}

static void stm32f3xx_adc_write(void *opaque, hwaddr addr,
                       uint64_t val64, unsigned int size)
{
    //STM32F3XXADCState *s = opaque;
	//do noting here
}

static const MemoryRegionOps stm32f3xx_adc_ops = {
    .read = stm32f3xx_adc_read,
    .write = stm32f3xx_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_stm32f3xx_adc = {
    .name = TYPE_STM32F3XX_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(adc_dr, STM32F3XXADCState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f3xx_adc_init(Object *obj)
{
    STM32F3XXADCState *s = STM32F3XX_ADC(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->mmio, obj, &stm32f3xx_adc_ops, s,
                          TYPE_STM32F3XX_ADC, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void stm32f3xx_adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32f3xx_adc_reset;
    dc->vmsd = &vmstate_stm32f3xx_adc;
}

static const TypeInfo stm32f3xx_adc_info = {
    .name          = TYPE_STM32F3XX_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F3XXADCState),
    .instance_init = stm32f3xx_adc_init,
    .class_init    = stm32f3xx_adc_class_init,
};

static void stm32f3xx_adc_register_types(void)
{
    type_register_static(&stm32f3xx_adc_info);
}

type_init(stm32f3xx_adc_register_types)
