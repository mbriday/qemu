/*
 * STM32F3xx 
 * Copyright (c) 2020 M. Briday <mikael.briday@ls2n.fr>
 *
 * based on i.MX processors GPIO emulation.
 * Copyright (C) 2015 Jean-Christophe Dubois <jcd@tribudubois.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/gpio/stm32f3xx_gpio.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/main-loop.h"


#ifndef DEBUG_STM32F3XX_GPIO
#define DEBUG_STM32F3XX_GPIO 0
#endif

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_STM32F3XX_GPIO) { \
            fprintf(stderr, "[%s] %s: " fmt , TYPE_STM32F3XX_GPIO, \
                                             __func__, ##args); \
        } \
    } while (0)

static const char *stm32_gpio_reg_name(uint32_t reg)
{
    switch (reg) {
		case MODER_ADDR:
			return "MODER";
		case OTYPER_ADDR:
			return "OTYPER";
		case OSPEEDR_ADDR:
			return "OSPEEDR";
		case PUPDR_ADDR:
			return "PUPDR";
		case IDR_ADDR:
			return "IDR";
		case ODR_ADDR:
			return "ODR";
		case BSRR_ADDR:
			return "BSRR";
		case LCKR_ADDR:
			return "LCKR";
		case AFR1_ADDR:
			return "AFR1";
		case AFR2_ADDR:
			return "AFR2";
		case BRR_ADDR:
			return "BRR";
		default:
			return "[?]";
    }
}


/* Trigger fired when a GPIO input pin changes state (based
 * on an external stimulus from the machine).
 */
static void stm32f3xx_gpio_in_trigger(void *opaque, int irq, int level)
{
}

static uint64_t stm32f3xx_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    STM32F3XXGPIOState *s = (STM32F3XXGPIOState *)opaque;

    assert(size == 4);

    switch (offset) {
		case MODER_ADDR:
            return s->GPIOx_MODER;
		case OTYPER_ADDR:
			return s->GPIOx_OTYPER;
		case PUPDR_ADDR:
            return s->GPIOx_PUPDR;
        case IDR_ADDR:
            return s->GPIOx_IDR;
        case ODR_ADDR:
            return s->GPIOx_ODR;
        case BSRR_ADDR:
        case BRR_ADDR:
			qemu_log_mask(LOG_GUEST_ERROR,"%s: Write-only register %s\n",__FUNCTION__,stm32_gpio_reg_name(offset));
            return 0;
        case OSPEEDR_ADDR: //not implemented
        case LCKR_ADDR:    //not implemented
        case AFR1_ADDR:    //not implemented
        case AFR2_ADDR:    //not implemented
        default:
			qemu_log_mask(LOG_GUEST_ERROR,"%s: Not implemented: %s (0x%x)\n",__FUNCTION__,
					stm32_gpio_reg_name((int)offset),(int)offset);
            return 0;
    }
}

/* Write the Output Data Register.
 * Propagates the changes to the output IRQs.
 * Perhaps we should also update the input to match the output for
 * pins configured as outputs... */
static void stm32f3xx_gpio_ODR_write(STM32F3XXGPIOState *s, uint32_t new_value)
{
    uint32_t old_value;
    uint16_t changed, changed_out;
    old_value = s->GPIOx_ODR;

    /* Update register value.  Per documentation, the upper 16 bits
     * always read as 0. */
    s->GPIOx_ODR = new_value & 0x0000ffff;

    /* Get pins that changed value */
    changed = old_value ^ new_value;

    /* Get changed pins that are outputs - we will not touch input pins */
    changed_out = changed & s->dir_mask;

	/* update IDR */
	s->GPIOx_IDR &= ~(s->dir_mask);
	s->GPIOx_IDR |= (s->dir_mask & new_value);

    if (changed_out) {
		qemu_set_irq(s->out_irq, changed_out);
    }
}

/* Update the dir_mask, in function of MODER value*/
static void stm32f3xx_gpio_update_dir(STM32F3XXGPIOState *s)
{
	for(int pin = 0;pin < 16;pin++)
	{
		const int offset = pin << 1;
        /* If the mode is 0, the pin is input.  Otherwise, it
         * is output.
         */
		if(((s->GPIOx_MODER >> offset) & 0x3) == 0)
		{
			s->dir_mask &= ~(1<<pin); //input
		} else {
			s->dir_mask |=  (1<<pin); //output
		}
	}
	qemu_set_irq(s->out_irq, 0);
}

static void stm32f3xx_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                           unsigned size)
{
    DPRINTF("(%s, value = 0x%" PRIx32 ")\n", stm32_gpio_reg_name(offset),
            (uint32_t)value);
    uint32_t set_mask, reset_mask;
    STM32F3XXGPIOState *s = (STM32F3XXGPIOState *)opaque;

    assert(size == 4);

    switch (offset) {
        case MODER_ADDR:
            s->GPIOx_MODER = value;
			stm32f3xx_gpio_update_dir(s);
            break;
		case OTYPER_ADDR:
            s->GPIOx_OTYPER = value;
			break;
		case PUPDR_ADDR:
			s->GPIOx_PUPDR = value;
            break;
        case IDR_ADDR:
			qemu_log_mask(LOG_GUEST_ERROR,"%s: Read-only register %s\n",__FUNCTION__,stm32_gpio_reg_name(offset));
            break;
        case ODR_ADDR:
            stm32f3xx_gpio_ODR_write(s, value);
            break;
        case BSRR_ADDR:
            /* Setting a bit sets or resets the corresponding bit in the output
             * register.  The lower 16 bits perform resets, and the upper 16
             * bits perform sets.  Register is write-only and so does not need
             * to store a value.  Sets take priority over resets, so we do
             * resets first.
             */
            set_mask = value & 0x0000ffff;
            reset_mask = ~(value >> 16) & 0x0000ffff;
            stm32f3xx_gpio_ODR_write(s,
                    (s->GPIOx_ODR & reset_mask) | set_mask);
            break;
        case BRR_ADDR:
            /* Setting a bit resets the corresponding bit in the output
             * register.  Register is write-only and so does not need to store
             * a value. */
            reset_mask = ~value & 0x0000ffff;
            stm32f3xx_gpio_ODR_write(s, s->GPIOx_ODR & reset_mask);
            break;
        case OSPEEDR_ADDR: //not implemented
        case LCKR_ADDR:    //not implemented
        case AFR1_ADDR:    //not implemented
        case AFR2_ADDR:    //not implemented
        default:
			qemu_log_mask(LOG_GUEST_ERROR,"%s: Not implemented: %s (0x%x)\n",__FUNCTION__,
					stm32_gpio_reg_name((int)offset),(int)offset);
            break;
    }
}

static const MemoryRegionOps stm32f3xx_gpio_ops = {
    .read = stm32f3xx_gpio_read,
    .write = stm32f3xx_gpio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_stm32f3xx_gpio = {
    .name = TYPE_STM32F3XX_GPIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(GPIOx_MODER, STM32F3XXGPIOState),
        VMSTATE_UINT32(GPIOx_OTYPER, STM32F3XXGPIOState),
        VMSTATE_UINT32(GPIOx_PUPDR, STM32F3XXGPIOState),
        VMSTATE_UINT32(GPIOx_ODR, STM32F3XXGPIOState),
        VMSTATE_UINT16(GPIOx_IDR, STM32F3XXGPIOState),
        VMSTATE_UINT16(dir_mask, STM32F3XXGPIOState),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32f3xx_gpio_properties[] = {
    DEFINE_PROP_BOOL("gpio", STM32F3XXGPIOState, periph , true),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f3xx_gpio_reset(DeviceState *dev)
{
    STM32F3XXGPIOState *s = STM32F3XX_GPIO(dev);

    s->GPIOx_MODER = 0x0;
	s->GPIOx_OTYPER = 0x0;
	s->GPIOx_PUPDR = 0x0;
    s->GPIOx_ODR = 0;
    s->GPIOx_IDR = 0;

    s->dir_mask = 0; /* input = 0, output = 1 */

	qemu_set_irq(s->out_irq, 0);
}

static void stm32f3xx_gpio_realize(DeviceState *dev, Error **errp)
{
    STM32F3XXGPIOState *s = STM32F3XX_GPIO(dev);
	SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32f3xx_gpio_ops, s,
                          TYPE_STM32F3XX_GPIO, STM32F3XX_GPIO_MEM_SIZE);

    qdev_init_gpio_in(DEVICE(s), stm32f3xx_gpio_in_trigger, STM32F3XX_GPIO_PIN_COUNT);
    //qdev_init_gpio_out(DEVICE(s), s->out_irq, STM32F3XX_GPIO_PIN_COUNT); 
    qdev_init_gpio_out(DEVICE(s), &(s->out_irq), 1); 

    for(int pin = 0; pin < STM32F3XX_GPIO_PIN_COUNT; pin++) {
        sysbus_init_irq(sbd, &s->in_irq[pin]);
    }
    sysbus_init_mmio(sbd, &s->iomem);


}

static void stm32f3xx_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f3xx_gpio_realize;
    dc->reset = stm32f3xx_gpio_reset;
    device_class_set_props(dc, stm32f3xx_gpio_properties);
    dc->vmsd = &vmstate_stm32f3xx_gpio;
    dc->desc = "STM32 F3xx GPIO controller";
}

static const TypeInfo stm32f3xx_gpio_info = {
    .name = TYPE_STM32F3XX_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F3XXGPIOState),
    .class_init = stm32f3xx_gpio_class_init,
};

static void stm32f3xx_gpio_register_types(void)
{
    type_register_static(&stm32f3xx_gpio_info);
}

type_init(stm32f3xx_gpio_register_types)
