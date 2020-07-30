use crate::{
    bsp,
    bsp::{
        device_driver::{
            common::MMIODerefWrapper, IRQNumber, Mailbox, Message, PropertyTag,
            PropertyTagPowerState, PropertyTags,
        },
        MAILBOX,
    },
    driver, exception, println, time,
    time::interface::TimeManager,
};
use core::{fmt, ops, time::Duration, u32::MAX};
use cortex_a::barrier::{dmb, SY};
use register::{mmio::*, register_bitfields, register_structs};

register_bitfields! {
    u32,

    CORE_AHB_CFG [
        GLOBALINT_MASK OFFSET(0) NUMBITS(1) [
            Disabled = 0,
            Enabled = 1
        ],
        MAX_AXI_BURST OFFSET(1) NUMBITS(2) [],
        WAIT_AXI_WRITES OFFSET(4) NUMBITS(1) [],
        DMA_ENABLE OFFSET(5) NUMBITS(1) [],
        AHB_SINGLE OFFSET(23) NUMBITS(1) []
    ],

    CORE_USB_CFG [
        PHY_IF OFFSET(3) NUMBITS(1) [],
        ULPI_UTMI_SEL OFFSET(4) NUMBITS(1) [],
        SRP_CAPABLE OFFSET(8) NUMBITS(1) [],
        HNP_CAPABLE OFFSET(9) NUMBITS(1) [],
        ULPI_FSLS OFFSET(17) NUMBITS(2) [],
        ULPI_CLK_SUS_M OFFSET(19) NUMBITS(1) [],
        ULPI_EXT_VBUS_DRV OFFSET(20) NUMBITS(1) [
            Disabled = 0,
            Enabled = 1
        ],
        TERM_SEL_DL_PULSE OFFSET(22) NUMBITS(1) [
            Disabled = 0,
            Enabled = 1
        ]
    ],

    CORE_RESET [
        SOFT_RESET OFFSET(0) NUMBITS(1) [],
        RX_FIFO_FLUSH OFFSET(4) NUMBITS(1) [],
        TX_FIFO_FLUSH OFFSET(5) NUMBITS(1) [],
        TX_FIFO_NUM OFFSET(6) NUMBITS(5) [],
        AHB_IDLE OFFSET(31) NUMBITS(1) []
    ],

    CORE_INT_MASK [
        MODE_MISMATCH OFFSET(0) NUMBITS(1) [],
        SOF_INTR OFFSET(3) NUMBITS(1) [],
        RX_STS_Q_LVL OFFSET(4) NUMBITS(1) [],
        USB_SUSPEND OFFSET(11) NUMBITS(1) [],
        PORT_INTR OFFSET(24) NUMBITS(1) [],
        HC_INTR OFFSET(25) NUMBITS(1) [],
        CON_ID_STS_CHNG OFFSET(28) NUMBITS(1) [],
        DISCONNECT OFFSET(29) NUMBITS(1) [],
        SESS_REQ_INTR OFFSET(30) NUMBITS(1) [],
        WKUP_INTR OFFSET(31) NUMBITS(1) []
    ],

    CORE_HW_CFG2 [
        OP_MODE OFFSET(0) NUMBITS(1) [],
        ARCHITECTURE OFFSET(3) NUMBITS(2) [],
        HS_PHY_TYPE OFFSET(6) NUMBITS(4) [
            Not_Supported = 0,
            UTMI = 1,
            ULPI = 2,
            UTMI_ULPI = 3
        ],
        FS_PHY_TYPE OFFSET(8) NUMBITS(2) [
            Dedicated = 1
        ],
        NUM_HOST_CHANNELS OFFSET(14) NUMBITS(4) []
    ],

    HOST_CFG [
        FSLS_PCLK_SEL OFFSET(0) NUMBITS(2) [
            MHZ_30_60 = 0,
            MHZ_48 = 1,
            MHZ_6 = 2
        ]
    ],

    HOST_PORT [
        CONNECT OFFSET(0) NUMBITS(1) [],
        CONNECT_CHANGED OFFSET(1) NUMBITS(1) [],
        ENABLE OFFSET(2) NUMBITS(1) [],
        ENABLE_CHANGED OFFSET(3) NUMBITS(1) [],
        OVERCURRENT OFFSET(4) NUMBITS(1) [],
        OVERCURRENT_CHANGED OFFSET(5) NUMBITS(1) [],
        RESET OFFSET(8) NUMBITS(1) [],
        POWER OFFSET(12) NUMBITS(1) [],
        SPEED OFFSET(17) NUMBITS(2) [
            High = 0,
            Full = 1,
            Low = 2,
            Unknown = 3
        ]
    ]
}

register_structs! {
    #[allow(non_snake_case)]
    pub RegisterBlock {
        (0x000 => CORE_OTG_CTRL: ReadWrite<u32>),
        (0x004 => CORE_OTG_INT: ReadWrite<u32>),
        (0x008 => CORE_AHB_CFG: ReadWrite<u32, CORE_AHB_CFG::Register>),
        (0x00C => CORE_USB_CFG: ReadWrite<u32, CORE_USB_CFG::Register>),
        (0x010 => CORE_RESET: ReadWrite<u32, CORE_RESET::Register>),
        (0x014 => CORE_INT_STAT: ReadWrite<u32>),
        (0x018 => CORE_INT_MASK: ReadWrite<u32, CORE_INT_MASK::Register>),
        (0x01C => CORE_RX_STAT_RD: ReadOnly<u32>),
        (0x020 => CORE_RX_STAT_POP: ReadOnly<u32>),
        (0x024 => CORE_RX_FIFO_SIZE: ReadWrite<u32>),
        (0x028 => CORE_NPER_TX_FIFO_SIZE: ReadWrite<u32>),
        (0x02C => CORE_NPER_TX_STAT: ReadOnly<u32>),
        (0x030 => CORE_I2C_CTRL: ReadWrite<u32>),
        (0x034 => CORE_PHY_VENDOR_CTRL: ReadWrite<u32>),
        (0x038 => CORE_GPIO: ReadWrite<u32>),
        (0x03C => CORE_USER_ID: ReadWrite<u32>),
        (0x040 => CORE_VENDOR_ID: ReadWrite<u32>),
        (0x044 => CORE_HW_CFG1: ReadOnly<u32>),
        (0x048 => CORE_HW_CFG2: ReadOnly<u32, CORE_HW_CFG2::Register>),
        (0x04C => _reserved1),
        (0x0A0 => @END),
    }
}

register_structs! {
    #[allow(non_snake_case)]
    pub HostRegisterBlock {
        (0x000 => HOST_CFG: ReadWrite<u32, HOST_CFG::Register>),
        (0x004 => _reserved1),
        (0x040 => HOST_PORT: ReadWrite<u32, HOST_PORT::Register>),
        (0x044 => @END),
    }
}

register_structs! {
    #[allow(non_snake_case)]
    pub PowerRegisterBlock {
        (0x000 => CFG: ReadWrite<u32>),
        (0x004 => @END),
    }
}

//--------------------------------------------------------------------------------------------------
// Public Definitions
//--------------------------------------------------------------------------------------------------

/// Representation of the DWHCI HW.
pub struct DWHCI {
    base_addr: usize,
    irq_number: IRQNumber,
    host: DWHCIHost,
}

impl ops::Deref for DWHCI {
    type Target = RegisterBlock;

    fn deref(&self) -> &Self::Target {
        unsafe { &*self.ptr() }
    }
}

/// Representation of the DWHCI HW.
pub struct DWHCIHost {
    base_addr: usize,
    power_regs: MMIODerefWrapper<PowerRegisterBlock>,
}

impl ops::Deref for DWHCIHost {
    type Target = HostRegisterBlock;

    fn deref(&self) -> &Self::Target {
        unsafe { &*self.ptr() }
    }
}

//--------------------------------------------------------------------------------------------------
// Public Code
//--------------------------------------------------------------------------------------------------

impl DWHCI {
    pub const VENDOR_ID: u32 = 0x4F54280A;
    pub const MAX_CHANNELS: u32 = 16;

    /// Create an instance.
    ///
    /// # Safety
    ///
    /// - The user must ensure to provide the correct `base_addr`.
    pub const unsafe fn new(base_addr: usize, irq_number: bsp::device_driver::IRQNumber) -> Self {
        let host = DWHCIHost::new(base_addr + 0x400, base_addr + 0xE00);

        Self {
            base_addr,
            irq_number,
            host,
        }
    }

    /// Return a pointer to the register block.
    fn ptr(&self) -> *const RegisterBlock {
        self.base_addr as *const _
    }

    /// Returns core vendor id
    pub fn core_vendor_id(&self) -> u32 {
        return self.CORE_VENDOR_ID.get();
    }

    fn power_on(&self) -> Result<(), &str> {
        let power_on_tag = &mut PropertyTagPowerState {
            device_id: PropertyTagPowerState::DEVICE_ID_USB_HCD,
            state: PropertyTagPowerState::POWER_STATE_ON | PropertyTagPowerState::POWER_STATE_WAIT,
        };
        let tag = PropertyTag::new(PropertyTags::SET_POWER_STATE, power_on_tag);
        let mut msg = Message::new(&tag);
        match MAILBOX.send(Mailbox::BCM_MAILBOX_PROP_CHANNEL, &mut msg) {
            Ok(reply) => {
                return if reply.state & PropertyTagPowerState::POWER_STATE_NO_DEVICE == 0
                    && reply.state & PropertyTagPowerState::POWER_STATE_ON > 0
                {
                    Ok(())
                } else {
                    Err("invalid USB core power state")
                }
            }
            _ => {}
        }

        Err("failed to power on USB core")
    }

    fn enable_global_interrupts(&self) {
        self.CORE_AHB_CFG
            .write(CORE_AHB_CFG::GLOBALINT_MASK::Disabled);
    }

    fn enable_common_interrupts(&self) {
        self.CORE_INT_STAT.set(MAX);
    }

    fn enable_host_interrupts(&self) {
        self.CORE_INT_MASK.set(0);
        self.enable_common_interrupts();

        self.CORE_INT_MASK.modify(CORE_INT_MASK::HC_INTR::SET);
    }

    fn init_core(&self) -> Result<(), &str> {
        self.CORE_USB_CFG.write(
            CORE_USB_CFG::ULPI_EXT_VBUS_DRV::Disabled + CORE_USB_CFG::TERM_SEL_DL_PULSE::Disabled,
        );

        match self.reset() {
            Err(_) => return Err("failed to reset core"),
            _ => (),
        }

        self.CORE_USB_CFG
            .write(CORE_USB_CFG::ULPI_UTMI_SEL::CLEAR + CORE_USB_CFG::PHY_IF::CLEAR);

        if self.CORE_HW_CFG2.read(CORE_HW_CFG2::ARCHITECTURE) != 2 {
            return Err("invalid architecture");
        }

        if self
            .CORE_HW_CFG2
            .matches_all(CORE_HW_CFG2::HS_PHY_TYPE::ULPI + CORE_HW_CFG2::FS_PHY_TYPE::Dedicated)
        {
            self.CORE_USB_CFG
                .write(CORE_USB_CFG::ULPI_FSLS::SET + CORE_USB_CFG::ULPI_CLK_SUS_M::SET);
        } else {
            self.CORE_USB_CFG
                .write(CORE_USB_CFG::ULPI_FSLS::CLEAR + CORE_USB_CFG::ULPI_CLK_SUS_M::CLEAR);
        }

        if self.CORE_HW_CFG2.read(CORE_HW_CFG2::NUM_HOST_CHANNELS) <= 4
            || self.CORE_HW_CFG2.read(CORE_HW_CFG2::NUM_HOST_CHANNELS) > DWHCI::MAX_CHANNELS
        {
            return Err("invalid number of channels");
        }

        self.CORE_AHB_CFG.write(
            CORE_AHB_CFG::DMA_ENABLE::SET
                + CORE_AHB_CFG::WAIT_AXI_WRITES::SET
                + CORE_AHB_CFG::MAX_AXI_BURST::CLEAR,
        );
        self.CORE_USB_CFG
            .write(CORE_USB_CFG::HNP_CAPABLE::CLEAR + CORE_USB_CFG::SRP_CAPABLE::CLEAR);

        self.enable_common_interrupts();

        Ok(())
    }

    fn init_host(&self) -> Result<(), &str> {
        self.host.power_regs.CFG.set(0);

        self.host.HOST_CFG.write(HOST_CFG::FSLS_PCLK_SEL.val(0));
        if self
            .CORE_HW_CFG2
            .matches_all(CORE_HW_CFG2::HS_PHY_TYPE::ULPI + CORE_HW_CFG2::FS_PHY_TYPE::Dedicated)
            && self.CORE_USB_CFG.is_set(CORE_USB_CFG::ULPI_FSLS)
        {
            self.host.HOST_CFG.write(HOST_CFG::FSLS_PCLK_SEL::MHZ_48);
        } else {
            self.host.HOST_CFG.write(HOST_CFG::FSLS_PCLK_SEL::MHZ_30_60);
        }

        // todo dyn FIFO

        self.flush_tx_fifo(0x10);
        self.flush_rx_fifo();

        if !self.host.HOST_PORT.is_set(HOST_PORT::POWER) {
            self.host.HOST_PORT.modify(
                HOST_PORT::CONNECT_CHANGED::CLEAR
                    + HOST_PORT::ENABLE::CLEAR
                    + HOST_PORT::ENABLE_CHANGED::CLEAR
                    + HOST_PORT::OVERCURRENT_CHANGED::CLEAR
                    + HOST_PORT::POWER::SET,
            );
        }

        self.enable_host_interrupts();

        Ok(())
    }

    fn flush_tx_fifo(&self, fifo: u8) {
        self.CORE_RESET.write(
            CORE_RESET::TX_FIFO_FLUSH::SET
                + CORE_RESET::TX_FIFO_NUM.val(0x1F)
                + CORE_RESET::TX_FIFO_NUM.val((fifo << 6).into()),
        );

        while self.CORE_RESET.is_set(CORE_RESET::TX_FIFO_FLUSH) {
            time::time_manager().spin_for(Duration::from_micros(1));
        }
    }

    fn flush_rx_fifo(&self) {
        self.CORE_RESET.write(CORE_RESET::RX_FIFO_FLUSH::SET);

        while self.CORE_RESET.is_set(CORE_RESET::RX_FIFO_FLUSH) {
            time::time_manager().spin_for(Duration::from_micros(1));
        }
    }

    fn enable_root_port(&self) -> bool {
        let mut wait_ms = 20;

        while !self.host.HOST_PORT.is_set(HOST_PORT::CONNECT) {
            wait_ms -= 1;
            if wait_ms == 0 {
                return false;
            }

            time::time_manager().spin_for(Duration::from_millis(1));
        }

        time::time_manager().spin_for(Duration::from_millis(100)); // see USB 2.0 spec

        self.host.HOST_PORT.modify(
            HOST_PORT::CONNECT_CHANGED::CLEAR
                + HOST_PORT::ENABLE::CLEAR
                + HOST_PORT::ENABLE_CHANGED::CLEAR
                + HOST_PORT::OVERCURRENT_CHANGED::CLEAR
                + HOST_PORT::RESET::SET,
        );

        time::time_manager().spin_for(Duration::from_millis(100)); // see USB 2.0 spec (tDRSTR)

        self.host.HOST_PORT.modify(
            HOST_PORT::CONNECT_CHANGED::CLEAR
                + HOST_PORT::ENABLE::CLEAR
                + HOST_PORT::ENABLE_CHANGED::CLEAR
                + HOST_PORT::OVERCURRENT_CHANGED::CLEAR
                + HOST_PORT::RESET::CLEAR,
        );

        time::time_manager().spin_for(Duration::from_millis(20)); // see USB 2.0 spec (tRSTRCY)

        true
    }

    fn init_root_port(&self) -> bool {
        match self.host.HOST_PORT.read(HOST_PORT::SPEED) {
            3 => return false,
            _ => (),
        }

        true
    }

    fn reset(&self) -> Result<(), &str> {
        let mut wait_ms = 100;

        while !self.CORE_RESET.is_set(CORE_RESET::AHB_IDLE) {
            wait_ms -= 1;
            if wait_ms == 0 {
                return Err("time out waiting for AHB idle");
            }

            time::time_manager().spin_for(Duration::from_millis(1));
        }

        self.CORE_RESET.write(CORE_RESET::SOFT_RESET::SET);

        wait_ms = 10;

        while self.CORE_RESET.is_set(CORE_RESET::SOFT_RESET) {
            wait_ms -= 1;
            if wait_ms == 0 {
                return Err("time out waiting for reset completion");
            }

            time::time_manager().spin_for(Duration::from_millis(1));
        }

        time::time_manager().spin_for(Duration::from_millis(100));

        Ok(())
    }
}

impl exception::asynchronous::interface::IRQHandler for DWHCI {
    fn handle(&self) -> Result<(), &'static str> {
        Ok(())
    }
}

impl fmt::Display for DWHCI {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "OTG_CTRL: {:X}\n\
            AHB_CFG: {:X}\n\
            USB CFG: {:X}\n\
            RESET: {:X}\n",
            self.CORE_OTG_CTRL.get(),
            self.CORE_AHB_CFG.get(),
            self.CORE_USB_CFG.get(),
            self.CORE_RESET.get()
        )
    }
}

impl DWHCIHost {
    /// Create an instance.
    ///
    /// # Safety
    ///
    /// - The user must ensure to provide the correct `base_addr`.
    pub const unsafe fn new(base_addr: usize, power_base_addr: usize) -> Self {
        let power_regs: MMIODerefWrapper<PowerRegisterBlock> =
            MMIODerefWrapper::new(power_base_addr);

        Self {
            base_addr,
            power_regs,
        }
    }

    /// Return a pointer to the register block.
    fn ptr(&self) -> *const HostRegisterBlock {
        self.base_addr as *const _
    }
}

//------------------------------------------------------------------------------
// OS Interface Code
//------------------------------------------------------------------------------

impl driver::interface::DeviceDriver for DWHCI {
    fn compatible(&self) -> &str {
        "BCM DWHCI"
    }

    fn init(&self) -> Result<(), ()> {
        unsafe {
            dmb(SY);
        }

        if self.core_vendor_id() != DWHCI::VENDOR_ID {
            return Err(());
        }

        match self.power_on() {
            Err(msg) => panic!("failed to power on USB core: {}", msg),
            _ => (),
        }

        match self.init_core() {
            Err(msg) => panic!("failed to init core: {}", msg),
            _ => (),
        }

        // self.enable_global_interrupts();

        match self.init_host() {
            Err(msg) => panic!("failed to init host port: {}", msg),
            _ => (),
        }

        if !self.enable_root_port() {
            return Ok(());
        }

        if !self.init_root_port() {
            return Ok(());
        }

        println!("initialized and very happy to be here");

        Ok(())
    }

    fn register_and_enable_irq_handler(&'static self) -> Result<(), &'static str> {
        use bsp::exception::asynchronous::irq_manager;
        use exception::asynchronous::{interface::IRQManager, IRQDescriptor};

        let descriptor = IRQDescriptor {
            name: "BCM DWHCI",
            handler: self,
        };

        return Ok(());

        irq_manager().register_handler(self.irq_number, descriptor)?;
        irq_manager().enable(self.irq_number);

        Ok(())
    }
}
