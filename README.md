# QEMU Custom Modification for MMIO Accelerator
Forked from [QEMU](https://github.com/qemu/qemu) official repo, and added MMIO accelerator.
This repo is used for writing board support package (BSP) for RISC-V systems.

Note that current version uses `QEMU 10.2.0-rc3`.

## Dependencies
```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y \
    git \
    ninja-build \
    build-essential \
    pkg-config \
    libglib2.0-dev \
    libpixman-1-dev \
    python3 \
    python3-pip \
    libslirp-dev \
    libfdt-dev

# Python packages
pip3 install sphinx sphinx_rtd_theme setuptools
```

## Custom Device Implementation & Build
Follow below instructions for adding a basic MMIO adder unit to the RISC-V VirtIO platform.

### Overview on Modifications
Below are the summary of major modifications to the original qemu repo.
```plain text
qemu/
├── hw/
│   └── misc/
│       ├── adder_accelerator.c      # To be Added (New)
│       ├── meson.build              # To Be Modified
│       └── Kconfig                  # To Be Modified
├── include/
│   └── hw/
│       └── misc/
│           └── adder_accelerator.h  # To be Added (New)
└── hw/riscv/
    └── virt.c                       # To Be Modified
```

### Device Implementation
1. `include/hw/misc/adder_accelerator.h`
    ```C
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
    ```

2. `hw/misc/adder_accelerator.c`
    ```C
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
    ```

### Build Files
1. `hw/misc/meson.build`
    Add at the very bottom of the file.
    ```meson
    # Custom Adder Accelerator
    system_ss.add(when: 'CONFIG_ADDER_ACCELERATOR', if_true: files('adder_accelerator.c'))
    ```

2. `hw/misc/Kconfig`
    Add at the very bottom of the file.
    ```Kconfig
    config ADDER_ACCELERATOR
    bool
    default y if RISCV_VIRT
    ```

### Add Device to RISC-V Virt Machine
1. `include/hw/riscv/virt.h`
    Add enum element to the existing one
    ```C
    enum {
        VIRT_DEBUG,
        VIRT_MROM,
        VIRT_TEST,
        VIRT_RTC,
        VIRT_CLINT,
        VIRT_ACLINT_SSWI,
        VIRT_PCIE_PIO,
        VIRT_PLIC,
        VIRT_APLIC_M,
        VIRT_APLIC_S,
        VIRT_UART0,
        VIRT_ADDER_ACCEL,    // Added!
        VIRT_VIRTIO,
        VIRT_FW_CFG,
        // ...
    };
    ```
2. `hw/riscv/virt.c`
    Add the header to the include section:
    ```C
    #include "hw/misc/adder_accelerator.h"
    ```

    Then, add the device to the memory map:
    ```C
    static const MemMapEntry virt_memmap[] = {
        [VIRT_DEBUG] =        {        0x0,         0x100 },
        [VIRT_MROM] =         {     0x1000,        0xf000 },
        [VIRT_TEST] =         {   0x100000,        0x1000 },
        [VIRT_RTC] =          {   0x101000,        0x1000 },
        [VIRT_CLINT] =        {  0x2000000,       0x10000 },
        [VIRT_ACLINT_SSWI] =  {  0x2F00000,        0x4000 },
        [VIRT_PCIE_PIO] =     {  0x3000000,       0x10000 },
        [VIRT_IOMMU_SYS] =    {  0x3010000,        0x1000 },
        [VIRT_PLATFORM_BUS] = {  0x4000000,     0x2000000 },
        [VIRT_PLIC] =         {  0xc000000, VIRT_PLIC_SIZE(VIRT_CPUS_MAX * 2) },
        [VIRT_APLIC_M] =      {  0xc000000, APLIC_SIZE(VIRT_CPUS_MAX) },
        [VIRT_APLIC_S] =      {  0xd000000, APLIC_SIZE(VIRT_CPUS_MAX) },
        [VIRT_UART0] =        { 0x10000000,         0x100 },
        [VIRT_ADDER_ACCEL] =  { 0x10010000,          0x14 }, /* 5 registers = 20 bytes */
        [VIRT_VIRTIO] =       { 0x10001000,        0x1000 },
        [VIRT_FW_CFG] =       { 0x10100000,          0x18 },
        [VIRT_FLASH] =        { 0x20000000,     0x4000000 },
        [VIRT_IMSIC_M] =      { 0x24000000, VIRT_IMSIC_MAX_SIZE },
        [VIRT_IMSIC_S] =      { 0x28000000, VIRT_IMSIC_MAX_SIZE },
        [VIRT_PCIE_ECAM] =    { 0x30000000,    0x10000000 },
        [VIRT_PCIE_MMIO] =    { 0x40000000,    0x40000000 },
        [VIRT_DRAM] =         { 0x80000000,           0x0 },
    };
    ```

    Create the device in `virt_machine_init()` function, near the UART device creation:
    ```C
    static void virt_machine_init(MachineState *machine)
    {
        // ...

        serial_mm_init(system_memory, s->memmap[VIRT_UART0].base,
            0, qdev_get_gpio_in(mmio_irqchip, UART0_IRQ), 399193,
            serial_hd(0), DEVICE_LITTLE_ENDIAN);

        /* Create Adder Accelerator */
        DeviceState *adder_dev = qdev_new(TYPE_ADDER_ACCELERATOR);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(adder_dev), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(adder_dev), 0, s->memmap[VIRT_ADDER_ACCEL].base);    

        sysbus_create_simple("goldfish_rtc", s->memmap[VIRT_RTC].base,
            qdev_get_gpio_in(mmio_irqchip, RTC_IRQ));    
        // ...
    }
    ```

## Build
1. Configure
    ```bash
    # Build Directory
    mkdir -p build
    cd build

    # Configure (Only RISC-V 64-bit)
    ../configure --target-list=riscv64-softmmu \
                --enable-debug \
                --disable-werror
    ```
2. Build
    ```bash
    make -j$(nproc)
    ```

3. Check
The target binary should be under build/ dir.
    ```bash
    ./qemu-system-riscv64 --version
    ```

