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

#ifndef HW_STM32F3XX_DUMMY_UART_H
#define HW_STM32F3XX_DUMMY_UART_H

#include "hw/sysbus.h"
#include "hw/irq.h"

#define DUMMY_UART_TDR    0x28 //transmit data

#define TYPE_STM32F3XX_DUMMY_UART "stm32f3xx-dummy_uart"
#define STM32F3XX_DUMMY_UART(obj) \
    OBJECT_CHECK(STM32F3XXDUMMY_UARTState, (obj), TYPE_STM32F3XX_DUMMY_UART)

typedef struct {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    uint32_t dummy_uart_tdr;

    qemu_irq irq;
} STM32F3XXDUMMY_UARTState;

#endif /* HW_STM32F3XX_DUMMY_UART_H */
