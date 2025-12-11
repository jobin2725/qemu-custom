#ifndef HW_ADDER_ACCELERATOR_H
#define HW_ADDER_ACCELERATOR_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_ADDER_ACCELERATOR "adder-accelerator"
OBJECT_DECLARE_SIMPLE_TYPE(AdderAcceleratorState, ADDER_ACCELERATOR)

// Register offsets
#define ADDER_REG_A       0x00
#define ADDER_REG_B       0x04
#define ADDER_REG_RESULT  0x08
#define ADDER_REG_STATUS  0x0C
#define ADDER_REG_CONTROL 0x10

// Status bits
#define ADDER_STATUS_VALID  (1 << 0)  // Result is valid
#define ADDER_STATUS_BUSY   (1 << 1)  // Computation in progress

// Control bits
#define ADDER_CONTROL_START (1 << 0)  // Start computation

// Simulated computation delay in nanoseconds (10us = 10000ns)
#define ADDER_COMPUTE_DELAY_NS 10000

struct AdderAcceleratorState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    // Registers
    uint32_t reg_a;
    uint32_t reg_b;
    uint32_t reg_result;
    uint32_t reg_status;
    uint32_t reg_control;
};

#endif /* HW_ADDER_ACCELERATOR_H */