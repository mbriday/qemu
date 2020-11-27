/*
 * STM32F405 SPI
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
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/ssi/stm32f3xx_spi_mcp27S17.h"
#include "migration/vmstate.h"
#include "hw/irq.h"

#ifndef STM_SPI_ERR_DEBUG
	#define STM_SPI_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_SPI_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)

#define SPI_DEBUG

static uint8_t read_mcp23s17(STM32F3XXSPIState *dev, uint32_t addr)
{
	MCP23S17State *mcp = &(dev->mcp);
	uint8_t data = mcp->regs[addr];
	uint8_t dir = 0;
	uint8_t pu = 0;
	switch(addr)
	{
		case MCP32S17_IODIRA  : /* nothing */
		case MCP32S17_IODIRB  :
		case MCP32S17_IOPOLA  :
		case MCP32S17_IOPOLB  :
		case MCP32S17_GPINTENA:
		case MCP32S17_GPINTENB:
		case MCP32S17_DEFVALA :
		case MCP32S17_DEFVALB :
		case MCP32S17_INTCONA :
		case MCP32S17_INTCONB :
		case MCP32S17_IOCON   :
		case MCP32S17_GPPUA   :
		case MCP32S17_GPPUB   :
		case MCP32S17_INTFA   :
		case MCP32S17_INTFB   :
		case MCP32S17_INTCAPA :
		case MCP32S17_INTCAPB :
		case MCP32S17_GPIOA   :
			break;
		case MCP32S17_GPIOB   :
			//special case for GPIOB =>
			//4 DIP switch 0-3: GPIOB & DIR & pullup
			//4 push buttons 4-7 (pullup required): GPIOB & DIR & PU
			dir = mcp->regs[MCP32S17_IODIRB]; //dir
			pu  = mcp->regs[MCP32S17_GPPUB];  //pullup
			data = mcp->regs[MCP32S17_GPIOB] & dir & pu;
			break;
		case MCP32S17_OLATA	  :
		case MCP32S17_OLATB	  :
			break;
	}
	return data;
}

static void updateOUT_PORT(STM32F3XXSPIState *dev, int port)
{
	if(port == 0) {
		//on the board, only PORTA has outputs
		qemu_set_irq(dev->out_irq, 0);
	}
}

static void write_mcp23s17(STM32F3XXSPIState *dev, uint32_t addr, uint32_t newValue)
{
	MCP23S17State *mcp = &(dev->mcp);
	//const uint32_t oldValue = mcp->regs[addr];
	switch(addr)
	{
		case MCP32S17_IODIRA  : mcp->regs[addr] = newValue; updateOUT_PORT(dev,0);
			break;
		case MCP32S17_IODIRB  : mcp->regs[addr] = newValue; updateOUT_PORT(dev,1);
			break;
		case MCP32S17_IOPOLA  : mcp->regs[addr] = newValue; updateOUT_PORT(dev,0);
			break;
		case MCP32S17_IOPOLB  : mcp->regs[addr] = newValue; updateOUT_PORT(dev,1);
			break;
		case MCP32S17_GPINTENA: mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_GPINTENB: mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_DEFVALA : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_DEFVALB : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_INTCONA : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_INTCONB : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_IOCON   :
			if(newValue & 0x80) printf("error: only one bank address mode implemented!\n");
			if(newValue & 0x40) printf("error: mirror mode not implemented!\n");
			if(newValue & 0x02) printf("error: intpol not implemented!\n");
			mcp->regs[addr] = newValue & 0x7E; //remove bit 7 and bit 0
			break;
		case MCP32S17_GPPUA   : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_GPPUB   : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_INTFA   : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_INTFB   : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_INTCAPA : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_INTCAPB : mcp->regs[addr] = newValue;   //TODO
			break;
		case MCP32S17_GPIOA   :
			//writing to GPIO implies OLAT (for outputs)
			mcp->regs[addr] = mcp->regs[MCP32S17_OLATB] = newValue; 
			updateOUT_PORT(dev,0);
			break;
		case MCP32S17_GPIOB   :
			//writing to GPIO implies OLAT (for outputs)
			mcp->regs[addr] = mcp->regs[MCP32S17_OLATB] = newValue; 
			updateOUT_PORT(dev,1);
			break;
		case MCP32S17_OLATA	  :
			//writing to GPIO implies OLAT (for outputs)
			mcp->regs[addr] = mcp->regs[MCP32S17_OLATB] = newValue; 
			updateOUT_PORT(dev,0);
			break;
		case MCP32S17_OLATB	  :
			//writing to GPIO implies OLAT (for outputs)
			mcp->regs[addr] = mcp->regs[MCP32S17_OLATB] = newValue; 
			updateOUT_PORT(dev,1);
			break;
	}
	mcp->frame[2] = mcp->regs[addr];
}

//static void write_mcp23s17(STM32F3XXSPIState *dev, uint32_t addr, uint32_t newValue)
//{
//	MCP23S17State *mcp = &(dev->mcp);
//	const uint32_t oldValue = mcp->regs[addr];
//	if(newValue != oldValue) 
//	{
//		printf("write %x to reg %x\n",newValue,addr);
//
//		//GPIOA update => send interrupt for GUI
//		if((addr == MCP32S17_IODIRA) ||
//		   (addr == MCP32S17_OLATA ) ||
//		   (addr == MCP32S17_GPIOA ) ||
//		   (addr == MCP32S17_IOPOLA))
//		{
//			//store new register state;
//			mcp->regs[addr] = newValue;
//			//writing to GPIO implies LATCH.
//			if(addr == MCP32S17_GPIOA) mcp->regs[MCP32S17_OLATA] = newValue;
//			const uint32_t changed_out = addr; //gives the address.
//			qemu_set_irq(dev->out_irq, changed_out);
//		}
//		if(addr == MCP32S17_IOCON)
//		{
//			if(newValue & 0x80) printf("error: only one bank address mode implemented!\n");
//			if(newValue & 0x40) printf("error: mirror mode not implemented!\n");
//			if(newValue & 0x02) printf("error: intpol not implemented!\n");
//			mcp->regs[addr] = mcp->frame[2] & 0x7E; //remove bit 7 and bit 0
//		}
//	}
//}

void stm32f3xx_spi_updateCS(STM32F3XXSPIState *dev,uint32_t state)
{

	MCP23S17State *mcp = &(dev->mcp);
	if(state)
	{
		/* end of transaction => update MCP */
		/* perform a WRITE transaction */
		if(mcp->frameByte == 3) {
			if((mcp->frame[0] & 0xF1) == 0x40) { //first byte => WRITE
				uint32_t addr = mcp->frame[1]; 
				if(addr < 22) { //22 addresses only
					write_mcp23s17(dev, addr, mcp->frame[2]);
				}
			}
		}
#ifdef SPI_DEBUG
		printf("spi transaction complete: ");
		for(int i=0;i<mcp->frameByte;i++) printf("%02x ",mcp->frame[i]);
		printf("\n");
#endif

	} else {
		/* start transaction */
	}
	/* either end or start of a transaction */
	mcp->frameByte = 0;
}

static void stm32f3xx_spi_reset(DeviceState *dev)
{
    STM32F3XXSPIState *s = STM32F3XX_SPI(dev);

    s->spi_cr1 = 0x00000000;
    s->spi_cr2 = 0x00000000;
    s->spi_sr = 0x0000000A;
    s->spi_dr = 0x0000000C;
    s->spi_crcpr = 0x00000007;
    s->spi_rxcrcr = 0x00000000;
    s->spi_txcrcr = 0x00000000;
}

/* entry point for a transaction on DR. called when there is a write */
static void stm32f3xx_spi_transfer(STM32F3XXSPIState *s)
{
	MCP23S17State *mcp = &(s->mcp);

	mcp->frame[mcp->frameByte] = (s->spi_dr) & 0xff;
	mcp->frameByte  = (mcp->frameByte+1)%16; //prevent overflow
	/* read access? */
	if(mcp->frameByte == 3) {
		if((mcp->frame[0] & 0xF1) == 0x41) { //first byte => READ
			uint32_t addr = mcp->frame[1]; 
			if(addr < 22) { //22 addresses only

				//store in frame for debug
				//and in dr for the next read access in the application code.
				s->spi_dr = mcp->frame[2] = read_mcp23s17(s,addr);
			}
		}
	}
}

static uint64_t stm32f3xx_spi_read(void *opaque, hwaddr addr,
                                     unsigned int size)
{
    STM32F3XXSPIState *s = opaque;

    DB_PRINT("Address: 0x%" HWADDR_PRIx "\n", addr);

    switch (addr) {
    case STM_SPI_CR1:
        return s->spi_cr1;
    case STM_SPI_CR2:
        qemu_log_mask(LOG_UNIMP, "%s: Interrupts and DMA are not implemented\n",
                      __func__);
        return s->spi_cr2;
    case STM_SPI_SR:
        return s->spi_sr;
    case STM_SPI_DR:
        return s->spi_dr;
    case STM_SPI_CRCPR:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented, the registers " \
                      "are included for compatibility\n", __func__);
        return s->spi_crcpr;
    case STM_SPI_RXCRCR:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented, the registers " \
                      "are included for compatibility\n", __func__);
        return s->spi_rxcrcr;
    case STM_SPI_TXCRCR:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented, the registers " \
                      "are included for compatibility\n", __func__);
        return s->spi_txcrcr;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr);
    }

    return 0;
}

static void stm32f3xx_spi_write(void *opaque, hwaddr addr,
                                uint64_t val64, unsigned int size)
{
    STM32F3XXSPIState *s = opaque;
    uint32_t value = val64;

    DB_PRINT("Address: 0x%" HWADDR_PRIx ", Value: 0x%x\n", addr, value);

    switch (addr) {
    case STM_SPI_CR1:
        s->spi_cr1 = value;
        return;
    case STM_SPI_CR2:
        qemu_log_mask(LOG_UNIMP, "%s: " \
                      "Interrupts and DMA are not implemented\n", __func__);
        s->spi_cr2 = value;
        return;
    case STM_SPI_SR:
        /* Read only register, except for clearing the CRCERR bit, which
         * is not supported
         */
        return;
    case STM_SPI_DR:
        s->spi_dr = value;
        stm32f3xx_spi_transfer(s);
        return;
    case STM_SPI_CRCPR:
        qemu_log_mask(LOG_UNIMP, "%s: CRC is not implemented\n", __func__);
        return;
    case STM_SPI_RXCRCR:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read only register: " \
                      "0x%" HWADDR_PRIx "\n", __func__, addr);
        return;
    case STM_SPI_TXCRCR:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Read only register: " \
                      "0x%" HWADDR_PRIx "\n", __func__, addr);
        return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
    }
}

static const MemoryRegionOps stm32f3xx_spi_ops = {
    .read = stm32f3xx_spi_read,
    .write = stm32f3xx_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_stm32f3xx_spi = {
    .name = TYPE_STM32F3XX_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(spi_cr1, STM32F3XXSPIState),
        VMSTATE_UINT32(spi_cr2, STM32F3XXSPIState),
        VMSTATE_UINT32(spi_sr, STM32F3XXSPIState),
        VMSTATE_UINT32(spi_dr, STM32F3XXSPIState),
        VMSTATE_UINT32(spi_crcpr, STM32F3XXSPIState),
        VMSTATE_UINT32(spi_rxcrcr, STM32F3XXSPIState),
        VMSTATE_UINT32(spi_txcrcr, STM32F3XXSPIState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32f3xx_spi_init(Object *obj)
{
    STM32F3XXSPIState *s = STM32F3XX_SPI(obj);
    //DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &stm32f3xx_spi_ops, s,
                          TYPE_STM32F3XX_SPI, 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    //s->ssi = ssi_create_bus(dev, "ssi");
}

static void stm32f3xx_spi_realize(DeviceState *dev, Error **errp)
{
    STM32F3XXSPIState *s = STM32F3XX_SPI(dev);

    qdev_init_gpio_out(DEVICE(s), &(s->out_irq), 1); 
}

static void stm32f3xx_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f3xx_spi_realize;
    dc->reset = stm32f3xx_spi_reset;
    dc->vmsd = &vmstate_stm32f3xx_spi;
}

static const TypeInfo stm32f3xx_spi_info = {
    .name          = TYPE_STM32F3XX_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F3XXSPIState),
    .instance_init = stm32f3xx_spi_init,
    .class_init    = stm32f3xx_spi_class_init,
};

static void stm32f3xx_spi_register_types(void)
{
    type_register_static(&stm32f3xx_spi_info);
}

type_init(stm32f3xx_spi_register_types)
