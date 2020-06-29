/*
 * STM32F3xx RCC
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

#ifndef HW_STM_STM32F3XX_RCC_H
#define HW_STM_STM32F3XX_RCC_H

#include "hw/sysbus.h"
#include "hw/hw.h"


#define  RCC_CR           0x00 /*!< RCC clock control register,                */
#define  RCC_CFGR         0x04 /*!< RCC clock configuration register,          */
#define  RCC_CIR          0x08 /*!< RCC clock interrupt register,              */
#define  RCC_APB2RSTR     0x0C /*!< RCC APB2 peripheral reset register,        */
#define  RCC_APB1RSTR     0x10 /*!< RCC APB1 peripheral reset register,        */
#define  RCC_AHBENR       0x14 /*!< RCC AHB peripheral clock register,         */
#define  RCC_APB2ENR      0x18 /*!< RCC APB2 peripheral clock enable register, */
#define  RCC_APB1ENR      0x1C /*!< RCC APB1 peripheral clock enable register, */
#define  RCC_BDCR         0x20 /*!< RCC Backup domain control register,        */
#define  RCC_CSR          0x24 /*!< RCC clock control & status register,       */
#define  RCC_AHBRSTR      0x28 /*!< RCC AHB peripheral reset register,         */
#define  RCC_CFGR2        0x2C /*!< RCC clock configuration register 2,        */
#define  RCC_CFGR3        0x30 /*!< RCC clock configuration register 3,        */

#define RCC_CR_PLLON_Pos  24     /* bit number to start PLL */
#define RCC_CR_PLLREADDY_Pos  25 /* bit number to check PLL step*/

#define TYPE_STM32F3XX_RCC "stm32f3xx-rcc"
#define STM32F3XX_RCC(obj) \
    OBJECT_CHECK(STM32F3XXRCCState, (obj), TYPE_STM32F3XX_RCC)

typedef struct {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;
	uint32_t cr;

} STM32F3XXRCCState;

#endif
