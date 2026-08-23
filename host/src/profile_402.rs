//! CiA 402 transport glue: [`DriveIo`] over the master's SDO path.
//!
//! Production drives expose 0x6040/0x6041 through PDOs for real-time use;
//! this SDO implementation is for setup, commissioning and low-frequency
//! control — exactly the python-canopen fallback tier, but explicit.

use canopen_402::{Drive402, DriveIo};
use canopen_core::datatypes::{DataType, Value};
use canopen_core::types::Address;

use crate::client::RemoteNode;
use crate::error::HostError;

/// Controlword object.
pub const CW_ADDR: Address = Address::new(0x6040, 0);
/// Statusword object.
pub const SW_ADDR: Address = Address::new(0x6041, 0);

/// [`DriveIo`] backed by SDO reads/writes on a [`RemoteNode`].
pub struct SdoDriveIo<'a> {
    node: &'a RemoteNode<'a>,
}

impl<'a> SdoDriveIo<'a> {
    /// Binds to a remote node handle.
    #[must_use]
    pub const fn new(node: &'a RemoteNode<'a>) -> Self {
        Self { node }
    }

    /// A ready-to-use [`Drive402`] driver over SDO.
    #[must_use]
    pub fn drive(self) -> Drive402<Self> {
        Drive402::new(self)
    }
}

impl DriveIo for SdoDriveIo<'_> {
    type Error = HostError;

    fn write_controlword(&mut self, cw: u16) -> std::result::Result<(), HostError> {
        self.node.sdo_write(CW_ADDR, &cw.to_le_bytes())
    }

    fn read_statusword(&mut self) -> std::result::Result<u16, HostError> {
        match self.node.sdo_read(SW_ADDR, DataType::Unsigned16)? {
            Value::Uint(v) => Ok(v as u16),
            _ => Err(HostError::Protocol("statusword decoded to unexpected type")),
        }
    }
}
