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

#define REMOTE_GPIO_MAGICK 0xDEADBEEF

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
      uint32_t pin;	    /* pin number (0..15 */
      uint32_t state;   /* 0 or 1 */
	  uint32_t gpio;	/* 0 GPIOA, 1 GPIOB, … */
} gpio_in_msg;

/* this handler is called if at least one pin is updated 
 * changed_out is a mask of pins in output mode that toggled
 * see GPIOx_ODR to get the new values.
 **/
static void gpio_out_update_handler(void *opaque, int n, int changed_out) {
	STM32F3XXGPIOState *gpio= (STM32F3XXGPIOState *)opaque;

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
}

/* thread that listens messages from the Gui */
static void* remote_gpio_thread(void * arg)
{
	STM32F303State *dev = (STM32F303State *)arg;
    //Here we receive the data from the queue 
    const int MSG_MAX = 8192;
    char buf[MSG_MAX];
    gpio_in_msg *mg = (gpio_in_msg *)&buf;
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
        if(res != sizeof(gpio_in_msg)) continue;
        if((int) mg->magick != REMOTE_GPIO_MAGICK) {
            printf("Wrong message received\n");
        }
        if(mg->pin < 16) {
            qemu_mutex_lock_iothread();
			//printf("received IRQ from GUI\n");
			//printf("gpio %d, pin %d, state %d\n",mg->gpio,mg->pin, mg->state);
			STM32F3XXGPIOState *gpio = &(dev->gpio[mg->gpio]);
			if(mg->state) {
				gpio->GPIOx_IDR |= 1 << mg->pin;
			} else {
				gpio->GPIOx_IDR &= ~(1 << mg->pin);
			}
            //mpc8xxx_gpio_set_irq(arg,mg->pin,mg->state);
            qemu_mutex_unlock_iothread();
        }
    }
}

static void nucleo32_f303_init(MachineState *machine)
{
    STM32F303State *dev = STM32F303_SOC(qdev_new(TYPE_STM32F303_SOC));
    qdev_prop_set_string((DeviceState*)dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m4"));
    sysbus_realize_and_unref(SYS_BUS_DEVICE((DeviceState *)dev), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu),
                       machine->kernel_filename,
                       FLASH_SIZE);

	qemu_mutex_init(&dat_lock);
	mq = mq_open("/from_qemu",O_CREAT | O_WRONLY,S_IRUSR | S_IWUSR,NULL);
	qemu_thread_create(&thread, "remote_gpio", remote_gpio_thread, dev, QEMU_THREAD_JOINABLE);

	/* connect each gpio to the handler for communication with external tools (gui) */
	for(int i=0;i<STM_NUM_GPIOS;i++)
	{
		DeviceState *gpio = DEVICE(&(dev->gpio[i]));
		assert(gpio);
		qemu_irq *gpio_irq = qemu_allocate_irqs(gpio_out_update_handler, gpio, 1);
		qdev_connect_gpio_out(gpio, 0, gpio_irq[0]);
	}

}

static void nucleo32_f303_machine_init(MachineClass *mc)
{
    mc->desc = "Nucleo 32 (F303) Machine";
    mc->init = nucleo32_f303_init;
}

DEFINE_MACHINE("nucleo32_f303", nucleo32_f303_machine_init)
