extern crate alloc;
use alloc::vec::Vec;

pub enum DeviceClass {
    MassStorage,
}

pub struct Device {
    name: &'static str,
    kind: DeviceClass,
}

pub struct DeviceManager {
    devices: Vec<Device>,
}
