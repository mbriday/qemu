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
#include "ui/console.h"
#include "qemu/main-loop.h"

#define REMOTE_GPIO_MAGICK   0xDEADBEEF
#define REMOTE_ADC_MAGICK    0xCAFECAFE
#define REMOTE_SERIAL_MAGICK 0xABCDEF01
#define REMOTE_MCP_MAGICK    0xFEEDFEED
#define REMOTE_RESET_MAGICK  0xBADF00D

/* message queue, thread and mutex to communicate with GUI */
mqd_t mq;
QemuMutex dat_lock;
QemuThread thread;

typedef struct {
      uint32_t magick;
      uint16_t changed_out;	/* output bits updated */
      uint16_t dir_mask;    /* 1=>output, 0=>input */
      uint16_t output;      /* new output value    */
	  uint16_t gpio;		    /* 0 GPIOA, 1 GPIOB, … */
} gpio_out_msg;

typedef struct {
      uint32_t magick;
      uint32_t c;			/* character */
} uart_out_msg;

typedef struct {
      uint32_t magick;
      uint32_t pin;	    /* pin number (0..15 */
      uint32_t state;   /* 0 or 1 */
	  uint32_t gpio;	/* 0 GPIOA, 1 GPIOB, … */
} gpio_in_msg;

typedef struct {
      uint32_t magick;
      uint32_t id;	    /* ADC id */
      uint32_t value;   /* 0 to 4095 */
} adc_in_msg;

/* structure to get access to the full dev
 * when updating a gpio out
 */
typedef struct {
	STM32F303State *dev;
	STM32F3XXGPIOState *gpio;
} GPIODevAccess;

/* this handler is called:
 * * if at least one pin is updated changed_out is a mask of pins in 
 * output mode that toggled see GPIOx_ODR to get the new values.
 * * if the dir_mask is updated
 * * at startup
 **/
static void gpio_out_update_handler(void *opaque, int n, int changed_out) {
	STM32F3XXGPIOState *gpio= ((GPIODevAccess *)opaque)->gpio;
	STM32F303State	    *dev= ((GPIODevAccess *)opaque)->dev;

	/* There should only be one IRQ */
	assert(n == 0);

	qemu_mutex_lock(&dat_lock);
	gpio_out_msg msg; /* msg to interact without external tool */
	msg.magick = REMOTE_GPIO_MAGICK;
	msg.changed_out = changed_out;
	msg.dir_mask    = gpio->dir_mask;
	msg.output      = gpio->GPIOx_ODR;
	msg.gpio        = gpio->id; /* TODO update */
	/* send new value */
	mq_send(mq,(const char *)&msg,sizeof(msg),0);
	qemu_mutex_unlock(&dat_lock);
	//printf("*");
	//fflush(stdout);

	//special case: ultra-sound SRF on PA10 
	if(msg.gpio == 0 && msg.dir_mask & (1<<10) && msg.output & (1<<10) && msg.changed_out & (1<<10)) {
		printf("SRF05 Trigger");
		fflush(stdout);
		ptimer_state *timer = gpio->srf05.timerLatency;
		ptimer_transaction_begin(timer);
		ptimer_set_freq(timer,1000); //1Khz
        ptimer_set_count(timer, 20);
        //ptimer_set_limit(timer, 20, 0); //20 ms
		ptimer_run(timer,true); //oneshot
		ptimer_transaction_commit(timer);
		
	}
	//special case for MCP CS on PA11:
	if(msg.gpio == 0 && msg.dir_mask & (1<<11) && msg.changed_out & (1<<11)) {
		stm32f3xx_spi_updateCS(&(dev->spi[0]),msg.output & (1<<11));
	}
}

static void spi_mcp_update_handler(void *opaque, int n, int changed_out) {
	STM32F303State *dev = (STM32F303State *)opaque;
	MCP23S17State *mcp = &(dev->spi[0].mcp);
	/* There should only be one IRQ */
	assert(n == 0);
	//GPIOA
	qemu_mutex_lock(&dat_lock);
	gpio_out_msg msg; /* msg to interact without external tool */
	msg.magick = REMOTE_MCP_MAGICK;
	msg.changed_out = changed_out; //unused
	msg.dir_mask    = ~(mcp->regs[MCP32S17_IODIRA]) & 0xFF; //0 means output…
	msg.output      = mcp->regs[MCP32S17_OLATA] ^ mcp->regs[MCP32S17_IOPOLA] ;
	msg.gpio        = 0;
	/* send new value */
	mq_send(mq,(const char *)&msg,sizeof(msg),0);
	qemu_mutex_unlock(&dat_lock);
}

static void uart_out_update_handler(void *opaque, int n, int changed_out) {
	STM32F3XXDUMMY_UARTState *uart= (STM32F3XXDUMMY_UARTState *)opaque;
	//printf("%c",(char)uart->dummy_uart_tdr);
	//fflush(stdout);

	qemu_mutex_lock(&dat_lock);
	uart_out_msg msg; /* msg to interact without external tool */
	msg.magick = REMOTE_SERIAL_MAGICK;
	msg.c      = (uint16_t)uart->dummy_uart_tdr;
	/* send new value */
	mq_send(mq,(const char *)&msg,sizeof(msg),0);
	qemu_mutex_unlock(&dat_lock);
}

static void updateMCPFromOutside(STM32F303State *dev, uint32_t pin, uint32_t state)
{
	//printf("received IRQ MCP from GUI %d\n",pin);
	MCP23S17State *mcp = &(dev->spi[0].mcp);
	//uint32_t prev = mcp->regs[MCP32S17_GPIOB];
	if(state) {
		mcp->regs[MCP32S17_GPIOB] |= 1 << pin;
	} else {
		mcp->regs[MCP32S17_GPIOB] &= ~(1 << pin);
	}
	//printf("update GPIOB %x -> %x\n",prev,mcp->regs[MCP32S17_GPIOB]);

	uint8_t it = mcp23s17_interruptMgmt(mcp,pin,state);
	if(it)
	{
		//send IRQ to SYSCFG -> EXTI.
		//MCP23S17 GPIOB associated to PA9
		const int gpioPin  = 9;
		const int gpioPort = 0;
		qemu_irq irq = qdev_get_gpio_in(DEVICE(&(dev->syscfg)),gpioPin+gpioPort*16);	
		//IRQ is a falling edge if IOCON.INTPOL = 0, else a rising edge.
		//INTPOL is bit 1
		qemu_set_irq(irq, (mcp->regs[MCP32S17_IOCON] >> 1) & 1); 
	}
}

static void updateGPIOFromOutside(STM32F303State *dev, uint32_t port, uint32_t pin, uint32_t state)
{
	//printf("received IRQ from GUI\n");
	//printf("gpio %d, pin %d, state %d\n",gpio,pin, state);
	STM32F3XXGPIOState *gpio = &(dev->gpio[port]);
	//send IRQ to SYSCFG -> EXTI.
	qemu_irq irq = qdev_get_gpio_in(DEVICE(&(dev->syscfg)),pin+port*16);	
	qemu_set_irq(irq, state);
	if(state) {
		gpio->GPIOx_IDR |= 1 << pin;
	} else {
		gpio->GPIOx_IDR &= ~(1 << pin);
	}
}

static void srf05_interrupt_lat(void *opaque)
{
    STM32F303State *dev = opaque;
	STM32F3XXGPIOState *gpio = &(dev->gpio[0]); //port A
	//dev->srf05;
    printf("Timer US pulse starts..");
	fflush(stdout);
	updateGPIOFromOutside(dev,0,10,1); //set PA10.
	ptimer_state *timer = gpio->srf05.timerDistance;
	ptimer_transaction_begin(timer);
	ptimer_set_freq(timer,1000000); //1Mhz
	const int dist = 58*110; //58us/cm * 110 cm.
    ptimer_set_count(timer, dist);
    //ptimer_set_limit(timer, 20, 0); //20 ms
	ptimer_run(timer,true); //oneshot
	ptimer_transaction_commit(timer);
}

static void srf05_interrupt_measure(void *opaque)
{
    STM32F303State *dev = opaque;
	//dev->srf05;
    printf("done.\n");
	fflush(stdout);
	updateGPIOFromOutside(dev,0,10,0); //reset PA10.
}


/* thread that listens messages from the Gui */
static void* remote_gpio_thread(void * arg)
{
	STM32F303State *dev = (STM32F303State *)arg;
    //Here we receive the data from the queue 
    const int MSG_MAX = 8192;
    char buf[MSG_MAX];
    gpio_in_msg *msgGpio = (gpio_in_msg *)&buf;
    adc_in_msg  *msgAdc  = (adc_in_msg *)&buf;
    mqd_t mq = mq_open("/to_qemu",O_CREAT | O_RDONLY,S_IRUSR | S_IWUSR,NULL);
    if(mq<0) {
        perror("I can't open mq");
        exit(1);
    }
    while(1) {
        int res = mq_receive(mq,buf,MSG_MAX,NULL);
        if(res<0) {
            perror("I can't receive");
            exit(1);
        }
        if((int) msgGpio->magick == REMOTE_GPIO_MAGICK) {
			//printf("msg from gpio: ");
			//fflush(stdout);

			if(res != sizeof(gpio_in_msg)) continue;
        	if(msgGpio->pin < 16) {
				qemu_mutex_lock_iothread();
				updateGPIOFromOutside(dev,msgGpio->gpio, msgGpio->pin, msgGpio->state);
				qemu_mutex_unlock_iothread();
        	}
		} else if((int) msgAdc->magick == REMOTE_ADC_MAGICK) {
			//printf("msg from adc: %d",msgAdc->value);
			//fflush(stdout);
			if(msgAdc->id < STM_NUM_ADCS) {
				STM32F3XXADCState *adc = &(dev->adc[msgAdc->id]);
				adc->adc_dr = msgAdc->value;
			}
		} else if((int) msgGpio->magick == REMOTE_MCP_MAGICK) {
			//printf("msg from mcp: %d",msgGpio->pin);
			//fflush(stdout);
			if(res != sizeof(gpio_in_msg)) continue;
        	if(msgGpio->pin < 16 && msgGpio->gpio == 1) {
				qemu_mutex_lock_iothread();
				updateMCPFromOutside(dev, msgGpio->pin, msgGpio->state);
				qemu_mutex_unlock_iothread();
        	}
		} else {
        	printf("Wrong message received\n");
			fflush(stdout);
        }
    }
}


static void nucleo32_f303_init(MachineState *machine)
{
    printf("STM32F303 - Coro Lab board - version 2020-12-03.\n");
	fflush(stdout);
    STM32F303State *dev = STM32F303_SOC(qdev_new(TYPE_STM32F303_SOC));
    qdev_prop_set_string((DeviceState*)dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE((DeviceState *)dev), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       FLASH_SIZE);

	system_clock_scale = NANOSECONDS_PER_SECOND / 6400000; //6.4 MHz

	qemu_mutex_init(&dat_lock);
	mq = mq_open("/from_qemu",O_CREAT | O_WRONLY,S_IRUSR | S_IWUSR,NULL);
	qemu_thread_create(&thread, "remote_gpio", remote_gpio_thread, dev, QEMU_THREAD_JOINABLE);

	/* connect each gpio to the handler for communication with external tools (gui) */
	for(int i=0;i<STM_NUM_GPIOS;i++)
	{
		DeviceState *gpio = DEVICE(&(dev->gpio[i]));
		STM32F3XXGPIOState *gpioState = (STM32F3XXGPIOState *)(&(dev->gpio[i]));
		GPIODevAccess *gpioda = malloc(sizeof(GPIODevAccess));
		gpioda->dev  = dev;
		gpioda->gpio = gpioState;

		if(i == 0) { //GPIOA
			gpioState->srf05.timerLatency  = ptimer_init(srf05_interrupt_lat,dev,PTIMER_POLICY_DEFAULT);
			gpioState->srf05.timerDistance = ptimer_init(srf05_interrupt_measure,dev,PTIMER_POLICY_DEFAULT);
		} else {
			gpioState->srf05.timerLatency  = NULL;
			gpioState->srf05.timerDistance = NULL;
		}
		assert(gpio);
		
		qemu_irq *gpio_irq = qemu_allocate_irqs(gpio_out_update_handler, gpioda, 1);
		qdev_connect_gpio_out(gpio, 0, gpio_irq[0]);
	}

	for(int i=0;i<STM_NUM_SPIS;i++)
	{
		qemu_irq *spi_irq = qemu_allocate_irqs(spi_mcp_update_handler, dev, 1);
		qdev_connect_gpio_out(DEVICE(dev->spi), 0, spi_irq[0]);
	}
	/* connect each uart to the handler for communication with external tools (gui) */
	for(int i=0;i<STM_NUM_UARTS;i++)
	{
		DeviceState *uart = DEVICE(&(dev->uart[i]));
		assert(uart);
		qemu_irq *uart_irq = qemu_allocate_irqs(uart_out_update_handler, uart, 1);
		qdev_connect_gpio_out(uart, 0, uart_irq[0]);
	}

	/* send a 'reset' message so that GUI can be reinitialized */
	int val = REMOTE_RESET_MAGICK;
	mq_send(mq,(const char *)&val,sizeof(int),0);
}

static void nucleo32_f303_machine_init(MachineClass *mc)
{
    mc->desc = "Nucleo 32 (F303) Machine";
    mc->init = nucleo32_f303_init;
}

DEFINE_MACHINE("nucleo32_f303", nucleo32_f303_machine_init)
