use core::{ops, ptr};

extern crate alloc;
use cortex_a::barrier::{dmb, dsb, SY};
use register::{mmio::*, register_bitfields, register_structs};

use self::alloc::{alloc::alloc_zeroed, boxed::Box};
use crate::driver;
use core::{
    alloc::Layout,
    intrinsics::{size_of, size_of_val},
};
use cortex_a::asm;

register_bitfields! {
u32,

STATUS [
FULL OFFSET(31) NUMBITS(1) [],
EMPTY OFFSET(30) NUMBITS(1) []
]
}

register_structs! {
    #[allow(non_snake_case)]
    pub RegisterBlock {
        (0x00 => READ: ReadOnly<u32>),
        (0x04 => _reserved1),
        (0x18 => READ_STATUS: ReadOnly<u32, STATUS::Register>),
        (0x1C => _reserved2),
        (0x20 => WRITE: WriteOnly<u32>),
        (0x24 => _reserved3),
        (0x38 => WRITE_STATUS: ReadOnly<u32, STATUS::Register>),
        (0x40 => @END),
    }
}

pub struct Mailbox {
    base_addr: usize,
}

impl ops::Deref for Mailbox {
    type Target = RegisterBlock;

    fn deref(&self) -> &Self::Target {
        unsafe { &*self.ptr() }
    }
}

impl Mailbox {
    pub const BCM_MAILBOX_PROP_CHANNEL: u32 = 8;

    /// Create an instance.
    ///
    /// # Safety
    ///
    /// - The user must ensure to provide the correct `base_addr`.
    pub const unsafe fn new(base_addr: usize) -> Self {
        Self { base_addr }
    }

    pub fn send<'a, T: Tag>(
        &self,
        channel: u32,
        message: &'a mut Message<'a, T>,
    ) -> Result<&'a T, ()> {
        unsafe {
            dsb(SY);
            dmb(SY);
        }

        loop {
            if !self.WRITE_STATUS.is_set(STATUS::FULL) {
                break;
            }
        }

        let msg: Option<Box<RawMessage>>;

        unsafe {
            msg = message.marshal();
        }

        if msg.is_none() {
            return Result::Err(());
        }

        let opt = msg.unwrap();
        let msg = opt.as_ref();
        let contents_addr = msg as *const RawMessage as u32;
        let val = (contents_addr & !0xF) | (channel & 0xF);

        self.WRITE.set(val);

        loop {
            loop {
                if !self.READ_STATUS.is_set(STATUS::EMPTY) {
                    break;
                }

                asm::nop();
            }

            let response: u32 = self.READ.get();

            if ((response & 0xF) == channel) && ((response & !0xF) == contents_addr) {
                return if msg.request_code != 0x80000000 {
                    Err(())
                } else {
                    unsafe { Ok(message.read()) }
                };
            }
        }
    }

    /// Return a pointer to the register block.
    fn ptr(&self) -> *const RegisterBlock {
        self.base_addr as *const _
    }
}

pub trait Tag {
    fn value_length(&self) -> usize {
        return size_of_val(&self);
    }
}

#[repr(C)]
pub struct PropertyTag<'a, T: Tag> {
    id: u32,
    buf_size: u32,
    value_length: u32,
    tag: &'a T,
}

impl<'a, T: Tag> PropertyTag<'a, T> {
    pub fn new(id: u32, tag: &'a mut T) -> Self {
        let size: usize = size_of_val(tag);

        Self {
            id,
            buf_size: size as u32,
            value_length: tag.value_length() as u32,
            tag,
        }
    }
}

pub struct PropertyTags {}

impl PropertyTags {
    pub const GET_FIRMWARE_REVISION: u32 = 0x00000001;
    pub const GET_BOARD_MODEL: u32 = 0x00010001;
    pub const GET_BOARD_REVISION: u32 = 0x00010002;
    pub const GET_MAC_ADDRESS: u32 = 0x00010003;
    pub const GET_BOARD_SERIAL: u32 = 0x00010004;
    pub const GET_ARM_MEMORY: u32 = 0x00010005;
    pub const GET_VC_MEMORY: u32 = 0x00010006;
    pub const SET_POWER_STATE: u32 = 0x00028001;
    pub const GET_CLOCK_RATE: u32 = 0x00030002;
    pub const GET_TEMPERATURE: u32 = 0x00030006;
    pub const GET_EDID_BLOCK: u32 = 0x00030020;
    pub const GET_DISPLAY_DIMENSIONS: u32 = 0x00040003;
    pub const GET_COMMAND_LINE: u32 = 0x00050001;
}

#[repr(C)]
pub struct PropertyTagPowerState {
    pub device_id: u32,
    pub state: u32,
}

impl PropertyTagPowerState {
    pub const DEVICE_ID_SD_CARD: u32 = 0;
    pub const DEVICE_ID_USB_HCD: u32 = 3;
    pub const DEVICE_ID_ACT_LED: u32 = 130;
    pub const POWER_STATE_OFF: u32 = 0b00;
    pub const POWER_STATE_ON: u32 = 0b01;
    pub const POWER_STATE_WAIT: u32 = 0b10;
    pub const POWER_STATE_NO_DEVICE: u32 = 0b10;
}

impl Tag for PropertyTagPowerState {}

#[repr(C)]
pub struct PropertyTagTemperature {
    pub temperature_id: u32,
    pub value: u32,
}

impl PropertyTagTemperature {
    pub const TEMPERATURE_ID: u32 = 0;
}

impl Tag for PropertyTagTemperature {
    fn value_length(&self) -> usize {
        return 4;
    }
}

#[repr(C)]
struct RawMessage {
    size: u32,
    request_code: u32,
    // Tag is at least 3 fields
    tag: [u32; 3],
    end: u32,
}

#[repr(C)]
pub struct Message<'a, T: Tag> {
    size: u32,
    request_code: u32,
    tag: &'a PropertyTag<'a, T>,
    tag_location: *mut u32,
}

impl<'a, T: Tag> Message<'a, T> {
    pub fn new(tag: &'a PropertyTag<'a, T>) -> Self {
        Self {
            size: 0,
            request_code: 0,
            tag,
            tag_location: 0 as *mut u32,
        }
    }

    unsafe fn marshal(&mut self) -> Option<Box<RawMessage>> {
        let mut size = size_of::<RawMessage>();
        let tag_contents_size = size_of_val(self.tag.tag);

        size = size + tag_contents_size;

        // Align on 16 bytes, the last four bits of the mem ptr are used for mbox channel selection
        let layout = match Layout::from_size_align(size, 16) {
            Ok(layout) => layout,
            Err(_e) => return None,
        };

        let raw = alloc_zeroed(layout);
        let raw_msg = raw as *mut RawMessage;

        (*raw_msg).size = size as u32;
        (*raw_msg).request_code = self.request_code;

        self.tag_location = (&(*raw_msg).tag as *const u32 as *mut u32).offset(3);

        // Cast the PropertyTag pointer to a mutable pointer referencing to the buffer field of the
        // dereference raw_msg
        // There will be a quiz on this afterwards
        ptr::copy(
            self.tag as *const PropertyTag<T> as *const u32,
            &(*raw_msg).tag as *const u32 as *mut u32,
            3,
        );
        // copy contents
        ptr::copy(
            self.tag.tag as *const dyn Tag as *const u32,
            self.tag_location,
            tag_contents_size / 4,
        );

        // We've allocated zeroed memory, the tag end is already set to zero

        Some(Box::from_raw(raw_msg))
    }

    unsafe fn read(&mut self) -> &T {
        let tag = self.tag_location as *mut T;
        &*tag
    }
}

//------------------------------------------------------------------------------
// OS Interface Code
//------------------------------------------------------------------------------

impl driver::interface::DeviceDriver for Mailbox {
    fn compatible(&self) -> &str {
        "BCM Mailbox"
    }

    fn init(&self) -> Result<(), ()> {
        Ok(())
    }
}
