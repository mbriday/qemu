/*
 * for STM32F3XX Timer
 *
 * adapted for STM32F2XX Timer
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 * and 
 * STM32 Microcontroller Timer module
 * Copyright (C) 2010 Andrew Hankins
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
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/timer/stm32f3xx_timer.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/ptimer.h"
#include "qemu/main-loop.h"

#ifndef STM_TIMER_ERR_DEBUG
#define STM_TIMER_ERR_DEBUG 0
#endif

#define DB_PRINT_L(lvl, fmt, args...) do { \
    if (STM_TIMER_ERR_DEBUG >= lvl) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

#define DB_PRINT(fmt, args...) DB_PRINT_L(1, fmt, ## args)


static uint32_t stm32f3xx_timer_get_count(STM32F3XXTimerState *s)
{
    const uint64_t cnt = ptimer_get_count(s->timer);
	uint32_t result;
    if (s->tim_cr1 &  TIM_CR1_DIR) // down
    {
        result = cnt & 0xffff;
    }
    else	// up (but ptimer counts down)
    {
        result = s->tim_arr - (cnt & 0xffff);
    }
	return result;
}

static void stm32f3xx_timer_set_count(STM32F3XXTimerState *s, uint32_t cnt)
{
	ptimer_transaction_begin(s->timer);
    if (s->tim_cr1 &  TIM_CR1_DIR) // down
    {
        ptimer_set_count(s->timer, cnt & 0xFFFF);
    }
    else
    {
		const uint16_t value = (0x10000 + s->tim_arr - cnt) & 0xFFFF;
        ptimer_set_count(s->timer, value);
    }
	ptimer_transaction_commit(s->timer);
}

/* state:
 * * 0 => start/stop (depends on tim_cr.cen bit)
 * * 1 => update (change parameters)
 */
static void stm32f3xx_update_ptimer(STM32F3XXTimerState *s,int state)
{
	ptimer_state *timer = s->timer;
	if(s->tim_cr1 & TIM_CR1_CEN) {
		//timer on: take into account parameters
		// * psc: prescaler
		// * arr: autoreload
		// * counter value (aka cnt)
		// * direction (even if ptimer counts down)
		ptimer_transaction_begin(timer);
		const int32_t freq = s->freq_hz / (s->tim_psc+1);
		DB_PRINT("set frequency to %d\n",freq);
		ptimer_set_freq(timer,freq);
		DB_PRINT("set limit to %d\n",s->tim_arr);
        ptimer_set_limit(timer, s->tim_arr, 1);
		if(state == 0) { //if we update, we do not call ptimer_run
			DB_PRINT("RUN!\n");
			ptimer_run(timer,false);
		}
		ptimer_transaction_commit(timer);
	} else {
		//timer not runnning.
		//Did we stop?
		if(state == 0) {
			ptimer_transaction_begin(timer);
			ptimer_stop(timer);
			ptimer_transaction_commit(timer);
		}
		//else it was only a parameters 
		//modification with a non running timer.
	}
}

static void stm32f3xx_timer_interrupt(void *opaque)
{
    STM32F3XXTimerState *s = opaque;

    if (s->tim_dier & TIM_DIER_UIE && s->tim_cr1 & TIM_CR1_CEN) {
		//interrupt mode
        qemu_irq_pulse(s->irq);
		DB_PRINT("it sent.\n");
    } else {
		//in polling mode, the status register is updated.
        s->tim_sr |= 1;
	}
	if(s->tim_cr1 & TIM_CR1_OPM) { //One Pulse Mode
		s->tim_cr1 &= ~TIM_CR1_CEN;  //stop
		stm32f3xx_update_ptimer(s,0);
		
	}

    //if (s->tim_ccmr1 & (TIM_CCMR1_OC2M2 | TIM_CCMR1_OC2M1) &&
    //    !(s->tim_ccmr1 & TIM_CCMR1_OC2M0) &&
    //    s->tim_ccmr1 & TIM_CCMR1_OC2PE &&
    //    s->tim_ccer & TIM_CCER_CC2E) {
    //    /* PWM 2 - Mode 1 */
    //    DB_PRINT("PWM2 Duty Cycle: %d%%\n",
    //            s->tim_ccr2 / (100 * (s->tim_psc + 1)));
    //}
}

static void stm32f3xx_timer_reset(DeviceState *dev)
{
    STM32F3XXTimerState *s = STM32F3XXTIMER(dev);
    //int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
	DB_PRINT("reset timer\n");

    s->tim_cr1 = 0;
    s->tim_cr2 = 0;
    s->tim_smcr = 0;
    s->tim_dier = 0;
    s->tim_sr = 0;
    s->tim_egr = 0;
    s->tim_ccmr1 = 0;
    s->tim_ccmr2 = 0;
    s->tim_ccer = 0;
    s->tim_psc = 0;
    s->tim_arr = 0xffff;
    s->tim_ccr1 = 0;
    s->tim_ccr2 = 0;
    s->tim_ccr3 = 0;
    s->tim_ccr4 = 0;

	stm32f3xx_timer_set_count(s, 0);
}


static uint64_t stm32f3xx_timer_read(void *opaque, hwaddr offset,
                           unsigned size)
{
	DB_PRINT("stm32f3xx_timer_read\n");
    STM32F3XXTimerState *s = opaque;

    switch (offset) {
    case TIM_CR1:
        return s->tim_cr1;
    //case TIM_CR2:
    //    return s->tim_cr2;
    //case TIM_SMCR:
    //    return s->tim_smcr;
    case TIM_DIER:
        return s->tim_dier;
    case TIM_SR:
        return s->tim_sr;
    //case TIM_EGR:
    //    return s->tim_egr;
    //case TIM_CCMR1:
    //    return s->tim_ccmr1;
    //case TIM_CCMR2:
    //    return s->tim_ccmr2;
    //case TIM_CCER:
    //    return s->tim_ccer;
	case TIM_CNT:
		return stm32f3xx_timer_get_count(s);
    case TIM_PSC:
        return s->tim_psc;
    case TIM_ARR:
        return s->tim_arr;
    //case TIM_CCR1:
    //    return s->tim_ccr1;
    //case TIM_CCR2:
    //    return s->tim_ccr2;
    //case TIM_CCR3:
    //    return s->tim_ccr3;
    //case TIM_CCR4:
    //    return s->tim_ccr4;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
    }

    return 0;
}

static void stm32f3xx_timer_write(void *opaque, hwaddr offset,
                        uint64_t val64, unsigned size)
{
    STM32F3XXTimerState *s = opaque;
    uint32_t value = val64;
	uint16_t updated;
	uint16_t cnt;

	DB_PRINT("stm32f3xx_timer_write\n");
    DB_PRINT("Write 0x%x, 0x%"HWADDR_PRIx"\n", value, offset);

    switch (offset) {
    case TIM_CR1:
		updated = value ^ s->tim_cr1;
		if(updated & TIM_CR1_DIR)
		{
			cnt = stm32f3xx_timer_get_count(s);
			s->tim_cr1 = value;
			stm32f3xx_timer_set_count(s,cnt);
		} else {
			s->tim_cr1 = value;
		}

		stm32f3xx_update_ptimer(s,0);
        return;
    //case TIM_CR2:
    //    s->tim_cr2 = value;
    //    return;
    //case TIM_SMCR:
    //    s->tim_smcr = value;
    //    return;
    case TIM_DIER:
        s->tim_dier = value;
        return;
    case TIM_SR:
        /* This is set by hardware and cleared by software */
        s->tim_sr = value;
        return;
    //case TIM_EGR:
    //    s->tim_egr = value;
    //    if (s->tim_egr & TIM_EGR_UG) {
    //        timer_val = 0;
    //        break;
    //    }
    //    return;
    //case TIM_CCMR1:
    //    s->tim_ccmr1 = value;
    //    return;
    //case TIM_CCMR2:
    //    s->tim_ccmr2 = value;
    //    return;
    //case TIM_CCER:
    //    s->tim_ccer = value;
    //    return;
    case TIM_PSC:
    //    timer_val = stm32f3xx_ns_to_ticks(s, now) - s->tick_offset;
        s->tim_psc = value & 0xFFFF;
		stm32f3xx_update_ptimer(s,1);
        break;
    case TIM_CNT:
		stm32f3xx_timer_set_count(s,value);
		stm32f3xx_update_ptimer(s,1);
        break;
    case TIM_ARR:
		//ARR may update the counter, has ptimer is counting down...
		cnt = stm32f3xx_timer_get_count(s);
		s->tim_arr = value;
		stm32f3xx_timer_set_count(s,cnt);
		stm32f3xx_update_ptimer(s,1);
        return;
    //case TIM_CCR1:
    //    s->tim_ccr1 = value;
    //    return;
    //case TIM_CCR2:
    //    s->tim_ccr2 = value;
    //    return;
    //case TIM_CCR3:
    //    s->tim_ccr3 = value;
    //    return;
    //case TIM_CCR4:
    //    s->tim_ccr4 = value;
    //    return;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, offset);
        return;
    }
}

static const MemoryRegionOps stm32f3xx_timer_ops = {
    .read = stm32f3xx_timer_read,
    .write = stm32f3xx_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_stm32f3xx_timer = {
    .name = TYPE_STM32F3XX_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(tim_cr1, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_cr2, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_smcr, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_dier, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_sr, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_egr, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_ccmr1, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_ccmr2, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_ccer, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_psc, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_arr, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_ccr1, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_ccr2, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_ccr3, STM32F3XXTimerState),
        VMSTATE_UINT32(tim_ccr4, STM32F3XXTimerState),
        VMSTATE_END_OF_LIST()
    }
};

static Property stm32f3xx_timer_properties[] = {
    DEFINE_PROP_UINT64("clock-frequency", struct STM32F3XXTimerState,freq_hz, 72000000),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f3xx_timer_init(Object *obj)
{
    STM32F3XXTimerState *s = STM32F3XXTIMER(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->iomem, obj, &stm32f3xx_timer_ops, s, "stm32f3xx_timer", 0x400);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    s->timer = ptimer_init(stm32f3xx_timer_interrupt,s,PTIMER_POLICY_DEFAULT);
}

static void stm32f3xx_timer_realize(DeviceState *dev, Error **errp)
{
}

static void stm32f3xx_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32f3xx_timer_reset;
    device_class_set_props(dc, stm32f3xx_timer_properties);
    dc->vmsd = &vmstate_stm32f3xx_timer;
    dc->realize = stm32f3xx_timer_realize;
}

static const TypeInfo stm32f3xx_timer_info = {
    .name          = TYPE_STM32F3XX_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F3XXTimerState),
    .instance_init = stm32f3xx_timer_init,
    .class_init    = stm32f3xx_timer_class_init,
};

static void stm32f3xx_timer_register_types(void)
{
    type_register_static(&stm32f3xx_timer_info);
}

type_init(stm32f3xx_timer_register_types)
