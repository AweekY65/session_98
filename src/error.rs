use std::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IsoTpError {
    EmptyFrame,
    FrameTooLong { len: usize },
    InvalidPciType(u8),
    InvalidSingleFrameLength(u8),
    InvalidFirstFrameLength(u16),
    InvalidFlowStatus(u8),
    InvalidStMin(u8),
    PayloadTooLarge { len: usize, max: usize },
    BufferLimitExceeded { needed: usize, max: usize },
    SequenceMismatch { expected: u8, got: u8 },
    DuplicateSequence { sn: u8 },
    LengthMismatch { declared: usize, actual: usize },
    UnexpectedFrame(&'static str),
    Timeout(&'static str),
    FlowControlOverflow,
}

impl fmt::Display for IsoTpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IsoTpError::EmptyFrame => write!(f, "empty CAN frame"),
            IsoTpError::FrameTooLong { len } => {
                write!(f, "CAN frame data too long: {} bytes (max 8)", len)
            }
            IsoTpError::InvalidPciType(t) => write!(f, "invalid PCI type nibble: {:#x}", t),
            IsoTpError::InvalidSingleFrameLength(l) => {
                write!(f, "invalid single frame length: {}", l)
            }
            IsoTpError::InvalidFirstFrameLength(l) => {
                write!(f, "invalid first frame length: {} (must be 8..=4095)", l)
            }
            IsoTpError::InvalidFlowStatus(s) => write!(f, "invalid flow status: {:#x}", s),
            IsoTpError::InvalidStMin(v) => write!(f, "invalid STmin byte: {:#x}", v),
            IsoTpError::PayloadTooLarge { len, max } => {
                write!(f, "payload too large: {} bytes (max {})", len, max)
            }
            IsoTpError::BufferLimitExceeded { needed, max } => {
                write!(f, "buffer limit exceeded: need {} bytes (max {})", needed, max)
            }
            IsoTpError::SequenceMismatch { expected, got } => {
                write!(f, "sequence mismatch: expected SN {}, got {}", expected, got)
            }
            IsoTpError::DuplicateSequence { sn } => {
                write!(f, "duplicate consecutive frame, SN {}", sn)
            }
            IsoTpError::LengthMismatch { declared, actual } => {
                write!(f, "length mismatch: declared {}, got {}", declared, actual)
            }
            IsoTpError::UnexpectedFrame(ctx) => write!(f, "unexpected frame: {}", ctx),
            IsoTpError::Timeout(which) => write!(f, "timeout: {}", which),
            IsoTpError::FlowControlOverflow => write!(f, "flow control: overflow abort"),
        }
    }
}

impl std::error::Error for IsoTpError {}
