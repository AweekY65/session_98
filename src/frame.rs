use crate::error::IsoTpError;
use crate::{CAN_FRAME_LEN, CF_DATA_BYTES, FF_DATA_BYTES, SF_MAX_PAYLOAD};

/// Classic CAN frame: 11/29-bit id plus up to 8 data bytes.
/// Purely an in-memory value type; nothing is ever written to a real bus.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub data: [u8; CAN_FRAME_LEN],
    pub len: u8,
}

impl CanFrame {
    pub fn new(id: u32, payload: &[u8]) -> Result<Self, IsoTpError> {
        if payload.len() > CAN_FRAME_LEN {
            return Err(IsoTpError::FrameTooLong { len: payload.len() });
        }
        let mut data = [0u8; CAN_FRAME_LEN];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            id,
            data,
            len: payload.len() as u8,
        })
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

/// STmin: minimum separation time between consecutive frames.
/// Stored as microseconds. Byte encoding per ISO 15765-2:
/// 0x00..=0x7F -> milliseconds, 0xF1..=0xF9 -> 100..900 microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct StMin(pub u64);

impl StMin {
    pub const ZERO: StMin = StMin(0);

    pub fn from_millis(ms: u64) -> Self {
        StMin(ms * 1_000)
    }

    pub fn from_micros(us: u64) -> Self {
        StMin(us)
    }

    pub fn as_micros(self) -> u64 {
        self.0
    }

    pub fn from_byte(b: u8) -> Result<Self, IsoTpError> {
        match b {
            0x00..=0x7F => Ok(StMin::from_millis(b as u64)),
            0xF1..=0xF9 => Ok(StMin::from_micros((b - 0xF0) as u64 * 100)),
            _ => Err(IsoTpError::InvalidStMin(b)),
        }
    }

    pub fn to_byte(self) -> u8 {
        if self.0 % 1_000 == 0 && self.0 / 1_000 <= 0x7F {
            (self.0 / 1_000) as u8
        } else if self.0 % 100 == 0 && (100..=900).contains(&self.0) {
            0xF0 + (self.0 / 100) as u8
        } else {
            // Round up to the next millisecond, clamped to the encodable range.
            let ms = (self.0 + 999) / 1_000;
            ms.min(0x7F) as u8
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowStatus {
    ClearToSend,
    Wait,
    Overflow,
}

impl FlowStatus {
    fn from_nibble(v: u8) -> Result<Self, IsoTpError> {
        match v {
            0 => Ok(FlowStatus::ClearToSend),
            1 => Ok(FlowStatus::Wait),
            2 => Ok(FlowStatus::Overflow),
            _ => Err(IsoTpError::InvalidFlowStatus(v)),
        }
    }

    fn to_nibble(self) -> u8 {
        match self {
            FlowStatus::ClearToSend => 0,
            FlowStatus::Wait => 1,
            FlowStatus::Overflow => 2,
        }
    }
}

/// Decoded Protocol Control Information (first byte(s) of an ISO-TP frame).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pci {
    SingleFrame { len: u8 },
    FirstFrame { len: u16 },
    ConsecutiveFrame { sn: u8 },
    FlowControl { status: FlowStatus, block_size: u8, st_min: StMin },
}

pub fn decode_pci(data: &[u8]) -> Result<Pci, IsoTpError> {
    let b0 = *data.first().ok_or(IsoTpError::EmptyFrame)?;
    match b0 >> 4 {
        0x0 => {
            let len = b0 & 0x0F;
            if len == 0 || len as usize > SF_MAX_PAYLOAD {
                return Err(IsoTpError::InvalidSingleFrameLength(len));
            }
            if data.len() < 1 + len as usize {
                return Err(IsoTpError::LengthMismatch {
                    declared: len as usize,
                    actual: data.len() - 1,
                });
            }
            Ok(Pci::SingleFrame { len })
        }
        0x1 => {
            let b1 = *data.get(1).ok_or(IsoTpError::LengthMismatch {
                declared: 2,
                actual: data.len(),
            })?;
            let len = (((b0 & 0x0F) as u16) << 8) | b1 as u16;
            if (len as usize) <= SF_MAX_PAYLOAD {
                return Err(IsoTpError::InvalidFirstFrameLength(len));
            }
            Ok(Pci::FirstFrame { len })
        }
        0x2 => Ok(Pci::ConsecutiveFrame { sn: b0 & 0x0F }),
        0x3 => {
            let bs = *data.get(1).ok_or(IsoTpError::LengthMismatch {
                declared: 3,
                actual: data.len(),
            })?;
            let st = *data.get(2).ok_or(IsoTpError::LengthMismatch {
                declared: 3,
                actual: data.len(),
            })?;
            Ok(Pci::FlowControl {
                status: FlowStatus::from_nibble(b0 & 0x0F)?,
                block_size: bs,
                st_min: StMin::from_byte(st)?,
            })
        }
        t => Err(IsoTpError::InvalidPciType(t)),
    }
}

pub fn single_frame(id: u32, payload: &[u8]) -> Result<CanFrame, IsoTpError> {
    if payload.is_empty() || payload.len() > SF_MAX_PAYLOAD {
        return Err(IsoTpError::InvalidSingleFrameLength(payload.len() as u8));
    }
    let mut data = [0u8; CAN_FRAME_LEN];
    data[0] = payload.len() as u8;
    data[1..1 + payload.len()].copy_from_slice(payload);
    CanFrame::new(id, &data[..1 + payload.len()])
}

pub fn first_frame(id: u32, total_len: u16, first: &[u8]) -> Result<CanFrame, IsoTpError> {
    if first.len() != FF_DATA_BYTES {
        return Err(IsoTpError::LengthMismatch {
            declared: FF_DATA_BYTES,
            actual: first.len(),
        });
    }
    let mut data = [0u8; CAN_FRAME_LEN];
    data[0] = 0x10 | ((total_len >> 8) as u8 & 0x0F);
    data[1] = total_len as u8;
    data[2..2 + FF_DATA_BYTES].copy_from_slice(first);
    CanFrame::new(id, &data)
}

pub fn consecutive_frame(id: u32, sn: u8, chunk: &[u8]) -> Result<CanFrame, IsoTpError> {
    if chunk.is_empty() || chunk.len() > CF_DATA_BYTES {
        return Err(IsoTpError::LengthMismatch {
            declared: CF_DATA_BYTES,
            actual: chunk.len(),
        });
    }
    let mut data = [0u8; CAN_FRAME_LEN];
    data[0] = 0x20 | (sn & 0x0F);
    data[1..1 + chunk.len()].copy_from_slice(chunk);
    CanFrame::new(id, &data)
}

pub fn flow_control(id: u32, status: FlowStatus, block_size: u8, st_min: StMin) -> CanFrame {
    let data = [
        0x30 | status.to_nibble(),
        block_size,
        st_min.to_byte(),
        0, 0, 0, 0, 0,
    ];
    CanFrame {
        id,
        data,
        len: 8,
    }
}
