/*
 * STM32F3xx RCC stub.
 * This part does not emulate the RCC (Reset and Clock Control), 
 * but set the status registers bits so that the program that waits
 * for a dedicated bit his not blocked: 
 * i.e active wait for the PLL to stop or start.
 *
 * Copyright (c) 2020 M. Briday <mikael.briday@ls2n.fr>
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
#include "qemu/log.h"
#include "migration/vmstate.h"
#include "hw/misc/stm32f3xx_rcc.h"

static void stm32f3xx_rcc_reset(DeviceState *dev)
{
    STM32F3XXRCCState *s = STM32F3XX_RCC(dev);
    s->cr = 0x0;
}

static uint64_t stm32f3xx_rcc_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    STM32F3XXRCCState *s = opaque;
    assert(size == 4);
    switch (addr) {
		case RCC_CR: 
			  return s->cr;
			  break;
		case RCC_CFGR:
		case RCC_CIR:
		case RCC_APB2RSTR:
		case RCC_APB1RSTR:
		case RCC_AHBENR:
		case RCC_APB2ENR:
		case RCC_APB1ENR:
		case RCC_BDCR:
		case RCC_CSR:
		case RCC_AHBRSTR:
		case RCC_CFGR2:
		case RCC_CFGR3:
		default:
			  return 0;
			  break;
    }
}

static void stm32f3xx_rcc_write(void *opaque, hwaddr addr,
                       uint64_t val64, unsigned int size)
{
    STM32F3XXRCCState *s = opaque;
    assert(size == 4);
    uint32_t value = val64;

    switch (addr) {
		case RCC_CR:  /* stub to set PLL status according ON/OFF */
			  value &= ~(1<<RCC_CR_PLLREADDY_Pos); /* RO bit */
			  if (value & (1<<RCC_CR_PLLON_Pos)) 
			  {
				  value |= (1<<RCC_CR_PLLREADDY_Pos);
			  }
			  s->cr = value;
			  break;
		case RCC_CFGR:
		case RCC_CIR:
		case RCC_APB2RSTR:
		case RCC_APB1RSTR:
		case RCC_AHBENR:
		case RCC_APB2ENR:
		case RCC_APB1ENR:
		case RCC_BDCR:
		case RCC_CSR:
		case RCC_AHBRSTR:
		case RCC_CFGR2:
		case RCC_CFGR3:
		default:
			  break;
    }
}

static const MemoryRegionOps stm32f3xx_rcc_ops = {
    .read = stm32f3xx_rcc_read,
    .write = stm32f3xx_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32f3xx_rcc_init(Object *obj)
{
    STM32F3XXRCCState *s = STM32F3XX_RCC(obj);

    memory_region_init_io(&s->mmio, obj, &stm32f3xx_rcc_ops, s,
                          TYPE_STM32F3XX_RCC, 0x40);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static const VMStateDescription vmstate_stm32f3xx_rcc = {
    .name = TYPE_STM32F3XX_RCC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cr, STM32F3XXRCCState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f3xx_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32f3xx_rcc_reset;
    dc->vmsd = &vmstate_stm32f3xx_rcc;
}

static const TypeInfo stm32f3xx_rcc_info = {
    .name          = TYPE_STM32F3XX_RCC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F3XXRCCState),
    .instance_init = stm32f3xx_rcc_init,
    .class_init    = stm32f3xx_rcc_class_init,
};

static void stm32f3xx_rcc_register_types(void)
{
    type_register_static(&stm32f3xx_rcc_info);
}

type_init(stm32f3xx_rcc_register_types)
