/*
 * STM32F3xx GPIO
 * Copyright (c) 2020 M. Briday <mikael.briday@ls2n.fr>
 *
 * based on
 * i.MX processors GPIO registers definition.
 * Copyright (C) 2015 Jean-Christophe Dubois <jcd@tribudubois.net>
 *
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

#ifndef STM32F3XX_GPIO_H
#define STM32F3XX_GPIO_H

#include "hw/sysbus.h"

#define TYPE_STM32F3XX_GPIO "stm32f3xx.gpio"
#define STM32F3XX_GPIO(obj) OBJECT_CHECK(STM32F3XXGPIOState, (obj),TYPE_STM32F3XX_GPIO)

#define STM32F3XX_GPIO_MEM_SIZE 0x30

/* STM32F3xx GPIO memory map */
#define  MODER_ADDR    0x00  /* GPIO port mode register               */
#define  OTYPER_ADDR   0x04  /* GPIO port output type register        */
#define  OSPEEDR_ADDR  0x08  /* GPIO port output speed register       */
#define  PUPDR_ADDR    0x0C  /* GPIO port pull-up/pull-down register  */
#define  IDR_ADDR      0x10  /* GPIO port input data register         */
#define  ODR_ADDR      0x14  /* GPIO port output data register        */
#define  BSRR_ADDR     0x18  /* GPIO port bit set/reset register      */
#define  LCKR_ADDR     0x1C  /* GPIO port configuration lock register */
#define  AFR1_ADDR     0x20  /* GPIO alternate function registers     */
#define  AFR2_ADDR     0x24  /* GPIO alternate function registers     */
#define  BRR_ADDR      0x28  /* GPIO bit reset register */

#define STM32F3XX_GPIO_PIN_COUNT 16


typedef struct STM32F3XXGPIOState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    /* Properties TODO don't know the goal.*/
    bool periph;

	uint32_t GPIOx_MODER;
	uint32_t GPIOx_OTYPER; /* push/pull = 0, open drain is 1 */
	uint32_t GPIOx_PUPDR;  /* pull up/down reg (unused)*/
    uint32_t GPIOx_ODR;
    uint16_t GPIOx_IDR;

    uint16_t dir_mask; /* input = 0, output = 1 */ 
	
	/* IRQ for external usage - output */
    qemu_irq out_irq[STM32F3XX_GPIO_PIN_COUNT];
	/* IRQ for internal usage => other STM32 peripherals */
    qemu_irq in_irq[STM32F3XX_GPIO_PIN_COUNT];
} STM32F3XXGPIOState;

#endif /* STM32F3XX_GPIO_H */
