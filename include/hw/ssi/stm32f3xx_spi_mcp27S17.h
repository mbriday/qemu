/*
 * STM32F3XX SPI
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

#ifndef HW_STM32F3XX_SPI_H
#define HW_STM32F3XX_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"

#define STM_SPI_CR1     0x00
#define STM_SPI_CR2     0x04
#define STM_SPI_SR      0x08
#define STM_SPI_DR      0x0C
#define STM_SPI_CRCPR   0x10
#define STM_SPI_RXCRCR  0x14
#define STM_SPI_TXCRCR  0x18

#define STM_SPI_CR1_SPE  (1 << 6)
#define STM_SPI_CR1_MSTR (1 << 2)

#define STM_SPI_SR_RXNE   1

#define TYPE_STM32F3XX_SPI "stm32f3xx-spi"
#define STM32F3XX_SPI(obj) \
    OBJECT_CHECK(STM32F3XXSPIState, (obj), TYPE_STM32F3XX_SPI)

#define MCP32S17_IODIRA    0x0	//direction input(1), output(0)
#define MCP32S17_IODIRB    0x1
#define MCP32S17_IOPOLA    0x2	//polarity (toggle) -> not used
#define MCP32S17_IOPOLB    0x3
#define MCP32S17_GPINTENA  0x4	//interrupt on change
#define MCP32S17_GPINTENB  0x5
#define MCP32S17_DEFVALA   0x6	//default value (pin differs => it)
#define MCP32S17_DEFVALB   0x7
#define MCP32S17_INTCONA   0x8	// if if pin!=defval(1) or 
#define MCP32S17_INTCONB   0x9	// pin!=previous => edge detection(0)
#define MCP32S17_IOCON     0xA	// or 0xB. Config. default Ok => not used 
#define MCP32S17_GPPUA     0xC	//pullup
#define MCP32S17_GPPUB     0xD
#define MCP32S17_INTFA     0xE	//interrupt flag
#define MCP32S17_INTFB     0xF
#define MCP32S17_INTCAPA   0x10 //gpio capture
#define MCP32S17_INTCAPB   0x11
#define MCP32S17_GPIOA     0x12
#define MCP32S17_GPIOB     0x13 
#define MCP32S17_OLATA	   0x14 //output latch
#define MCP32S17_OLATB	   0x15

typedef struct {
	/* spi frame FSM */
	uint32_t frameByte; //init to 0.
	uint8_t frame[16];
	/* regs => IODIRA, ...) */
	uint8_t regs[22];

} MCP23S17State;


typedef struct {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    uint32_t spi_cr1;
    uint32_t spi_cr2;
    uint32_t spi_sr;
    uint32_t spi_dr;
    uint32_t spi_crcpr;
    uint32_t spi_rxcrcr;
    uint32_t spi_txcrcr;

    qemu_irq out_irq;
    qemu_irq irq;
	MCP23S17State mcp; /* MCP23S17 chip*/
} STM32F3XXSPIState;

void stm32f3xx_spi_updateCS(STM32F3XXSPIState *dev,uint32_t state);
#endif /* HW_STM32F3XX_SPI_H */
