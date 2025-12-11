/*
 * Simple Adder Accelerator (Asynchronous Version)
 *
 * Copyright (c) 2024
 *
 * This is a custom QEMU device that performs addition with simulated latency.
 * The device requires explicit START command and STATUS polling.
 */

#include "qemu/osdep.h"
#include "hw/misc/adder_accelerator.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "trace.h"

// Timer callback - computation complete
static void adder_compute_done(void *opaque)
{
    AdderAcceleratorState *s = ADDER_ACCELERATOR(opaque);

    // Perform the actual computation
    s->reg_result = s->reg_a + s->reg_b;

    // Clear BUSY, set VALID
    s->reg_status &= ~ADDER_STATUS_BUSY;
    s->reg_status |= ADDER_STATUS_VALID;

    // Clear control register
    s->reg_control = 0;

    qemu_log("adder: computation complete, result = 0x%08x (%u)\n",
             s->reg_result, s->reg_result);

    // Optionally raise IRQ (for interrupt-driven mode)
    // qemu_irq_raise(s->irq);
}

// Read from device registers
static uint64_t adder_accelerator_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    AdderAcceleratorState *s = ADDER_ACCELERATOR(opaque);
    uint32_t value = 0;

    switch (offset) {
    case ADDER_REG_A:
        value = s->reg_a;
        qemu_log("adder: read REG_A = 0x%08x\n", value);
        break;
    case ADDER_REG_B:
        value = s->reg_b;
        qemu_log("adder: read REG_B = 0x%08x\n", value);
        break;
    case ADDER_REG_RESULT:
        value = s->reg_result;
        qemu_log("adder: read REG_RESULT = 0x%08x (%u)\n", value, value);
        break;
    case ADDER_REG_STATUS:
        value = s->reg_status;
        qemu_log("adder: read REG_STATUS = 0x%08x (VALID=%d, BUSY=%d)\n",
                 value,
                 (value & ADDER_STATUS_VALID) ? 1 : 0,
                 (value & ADDER_STATUS_BUSY) ? 1 : 0);
        break;
    case ADDER_REG_CONTROL:
        value = s->reg_control;
        qemu_log("adder: read REG_CONTROL = 0x%08x\n", value);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "adder_accelerator: invalid read at offset 0x%"HWADDR_PRIx"\n",
                      offset);
        break;
    }

    return value;
}

// Write to device registers
static void adder_accelerator_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    AdderAcceleratorState *s = ADDER_ACCELERATOR(opaque);

    switch (offset) {
    case ADDER_REG_A:
        s->reg_a = value;
        // Clear VALID when input changes
        s->reg_status &= ~ADDER_STATUS_VALID;
        qemu_log("adder: write REG_A = 0x%08x\n", (uint32_t)value);
        break;

    case ADDER_REG_B:
        s->reg_b = value;
        // Clear VALID when input changes
        s->reg_status &= ~ADDER_STATUS_VALID;
        qemu_log("adder: write REG_B = 0x%08x\n", (uint32_t)value);
        break;

    case ADDER_REG_RESULT:
        // Result register is read-only, ignore writes
        qemu_log_mask(LOG_GUEST_ERROR,
                      "adder_accelerator: write to read-only RESULT register\n");
        break;

    case ADDER_REG_STATUS:
        // Writing 0 clears the status
        if (value == 0) {
            s->reg_status = 0;
        }
        qemu_log("adder: write REG_STATUS = 0x%08x\n", (uint32_t)value);
        break;

    case ADDER_REG_CONTROL:
        qemu_log("adder: write REG_CONTROL = 0x%08x\n", (uint32_t)value);

        // Check for START command
        if ((value & ADDER_CONTROL_START) && !(s->reg_status & ADDER_STATUS_BUSY)) {
            // Set BUSY, clear VALID
            s->reg_status |= ADDER_STATUS_BUSY;
            s->reg_status &= ~ADDER_STATUS_VALID;
            s->reg_control = value;

            qemu_log("adder: computation started (A=0x%08x, B=0x%08x)\n",
                     s->reg_a, s->reg_b);

            // Schedule timer for computation completion
            timer_mod(s->timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ADDER_COMPUTE_DELAY_NS);
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "adder_accelerator: invalid write at offset 0x%"HWADDR_PRIx"\n",
                      offset);
        break;
    }
}

// Memory region operations
static const MemoryRegionOps adder_accelerator_ops = {
    .read = adder_accelerator_read,
    .write = adder_accelerator_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

// Device realize (called when device is instantiated)
static void adder_accelerator_realize(DeviceState *dev, Error **errp)
{
    AdderAcceleratorState *s = ADDER_ACCELERATOR(dev);

    // Create timer for async computation
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, adder_compute_done, s);
}

// Device unrealize (cleanup)
static void adder_accelerator_unrealize(DeviceState *dev)
{
    AdderAcceleratorState *s = ADDER_ACCELERATOR(dev);

    timer_free(s->timer);
}

// Device initialization
static void adder_accelerator_init(Object *obj)
{
    AdderAcceleratorState *s = ADDER_ACCELERATOR(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    // Initialize memory region (5 registers * 4 bytes = 20 bytes)
    memory_region_init_io(&s->iomem, obj, &adder_accelerator_ops, s,
                         TYPE_ADDER_ACCELERATOR, 0x14);
    sysbus_init_mmio(sbd, &s->iomem);

    // Initialize IRQ (for interrupt-driven mode)
    sysbus_init_irq(sbd, &s->irq);
}

// Device reset
static void adder_accelerator_reset(DeviceState *dev)
{
    AdderAcceleratorState *s = ADDER_ACCELERATOR(dev);

    // Cancel any pending computation
    if (s->timer) {
        timer_del(s->timer);
    }

    s->reg_a = 0;
    s->reg_b = 0;
    s->reg_result = 0;
    s->reg_status = 0;
    s->reg_control = 0;
}

// VM state description (for save/restore)
static const VMStateDescription vmstate_adder_accelerator = {
    .name = TYPE_ADDER_ACCELERATOR,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(reg_a, AdderAcceleratorState),
        VMSTATE_UINT32(reg_b, AdderAcceleratorState),
        VMSTATE_UINT32(reg_result, AdderAcceleratorState),
        VMSTATE_UINT32(reg_status, AdderAcceleratorState),
        VMSTATE_UINT32(reg_control, AdderAcceleratorState),
        VMSTATE_TIMER_PTR(timer, AdderAcceleratorState),
        VMSTATE_END_OF_LIST()
    }
};

// Device class initialization
static void adder_accelerator_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = adder_accelerator_realize;
    dc->unrealize = adder_accelerator_unrealize;
    device_class_set_legacy_reset(dc, adder_accelerator_reset);
    dc->vmsd = &vmstate_adder_accelerator;
    dc->user_creatable = false;
}

// Type information
static const TypeInfo adder_accelerator_info = {
    .name          = TYPE_ADDER_ACCELERATOR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AdderAcceleratorState),
    .instance_init = adder_accelerator_init,
    .class_init    = adder_accelerator_class_init,
};

// Register the type
static void adder_accelerator_register_types(void)
{
    type_register_static(&adder_accelerator_info);
}

type_init(adder_accelerator_register_types)
