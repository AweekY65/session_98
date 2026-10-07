//! Local-only ISO-TP (ISO 15765-2) codec, segmentation and reassembly.
//!
//! Everything operates on in-memory [`CanFrame`] values and a manually
//! advanced [`VirtualClock`]. No SocketCAN, no sockets, no hardware, no
//! real-time sleeping.

pub mod clock;
pub mod error;
pub mod frame;
pub mod receiver;
pub mod sender;

pub use clock::VirtualClock;
pub use error::IsoTpError;
pub use frame::{CanFrame, FlowStatus, Pci, StMin};
pub use receiver::{Receiver, ReceiverConfig, RxOutput, RxState};
pub use sender::{Sender, SenderConfig, TxState};

pub const CAN_FRAME_LEN: usize = 8;
pub const SF_MAX_PAYLOAD: usize = 7;
pub const FF_DATA_BYTES: usize = 6;
pub const CF_DATA_BYTES: usize = 7;
/// 12-bit length field in the First Frame.
pub const MAX_ISOTP_PAYLOAD: usize = 4095;
