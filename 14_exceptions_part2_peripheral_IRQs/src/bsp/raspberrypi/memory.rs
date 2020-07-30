// SPDX-License-Identifier: MIT OR Apache-2.0
//
// Copyright (c) 2018-2020 Andre Richter <andre.o.richter@gmail.com>

//! BSP Memory Management.

pub mod mmu;

//--------------------------------------------------------------------------------------------------
// Public Definitions
//--------------------------------------------------------------------------------------------------

/// The early boot core's stack address.
pub const BOOT_CORE_STACK_START: usize = 0x80_000;

/// The board's memory map.
#[rustfmt::skip]
pub(super) mod map {
    pub const END_INCLUSIVE:                            usize =        0xFFFF_FFFF;

    pub const GPIO_OFFSET:                              usize =        0x0020_0000;
    pub const UART_OFFSET:                              usize =        0x0020_1000;
    pub const USB_OFFSET:                               usize =        0x0098_0000;

    /// Physical devices.
    #[cfg(feature = "bsp_rpi3")]
    pub mod mmio {
        use super::*;

        pub const DMA_HEAP_START:                       usize =        0x0020_0000;
        pub const DMA_HEAP_END_INCLUSIVE:               usize =        0x005F_FFFF;
        pub const BASE:                                 usize =        0x3F00_0000;
        pub const PERIPHERAL_INTERRUPT_CONTROLLER_BASE: usize = BASE + 0x0000_B200;
        pub const MAILBOX_BASE:                         usize = BASE + 0x0000_B880;
        pub const GPIO_BASE:                            usize = BASE + GPIO_OFFSET;
        pub const PL011_UART_BASE:                      usize = BASE + UART_OFFSET;
        pub const USB_BASE:                             usize = BASE + USB_OFFSET;
        pub const LOCAL_INTERRUPT_CONTROLLER_BASE:      usize =        0x4000_0000;
        pub const END_INCLUSIVE:                        usize =        0x4000_FFFF;
    }

    /// Physical devices.
    #[cfg(feature = "bsp_rpi4")]
    pub mod mmio {
        use super::*;

        pub const BASE:                                 usize =        0xFE00_0000;
        pub const GPIO_BASE:                            usize = BASE + GPIO_OFFSET;
        pub const PL011_UART_BASE:                      usize = BASE + UART_OFFSET;
        pub const GICD_BASE:                            usize =        0xFF84_1000;
        pub const GICC_BASE:                            usize =        0xFF84_2000;
        pub const END_INCLUSIVE:                        usize =        0xFF84_FFFF;
    }
}
