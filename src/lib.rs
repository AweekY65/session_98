//! Local-only CAN ISO-TP (ISO 15765-2 subset) codec.
//!
//! Everything operates on in-memory [`CanFrame`] values. No SocketCAN, no
//! sockets, no vehicle/ECU access, no external services, no real-time sleeps.
//! Timing semantics (STmin) are expressed through a [`VirtualClock`].

use std::fmt;

/// Classic CAN data length.
pub const CAN_FRAME_MAX_LEN: usize = 8;
/// Maximum payload carried by a Single Frame.
pub const SF_MAX_PAYLOAD: usize = 7;
/// Payload bytes carried by a First Frame.
pub const FF_DATA_BYTES: usize = 6;
/// Payload bytes carried by a Consecutive Frame.
pub const CF_DATA_BYTES: usize = 7;
/// Absolute protocol maximum payload (12-bit FF_DL).
pub const MAX_PAYLOAD: usize = 0x0FFF;

/// A classic CAN frame kept purely in memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFrame {
    pub id: u32,
    pub dlc: u8,
    pub data: [u8; CAN_FRAME_MAX_LEN],
}

impl CanFrame {
    pub fn new(id: u32, payload: &[u8]) -> Result<Self, IsotpError> {
        if payload.len() > CAN_FRAME_MAX_LEN {
            return Err(IsotpError::InvalidFrame("CAN payload exceeds 8 bytes"));
        }
        let mut data = [0u8; CAN_FRAME_MAX_LEN];
        data[..payload.len()].copy_from_slice(payload);
        Ok(CanFrame { id, dlc: payload.len() as u8, data })
    }

    pub fn bytes(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }
}

/// Errors produced by the codec, reassembler and sender state machine.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IsotpError {
    PayloadEmpty,
    PayloadTooLarge { len: usize, max: usize },
    InvalidFrame(&'static str),
    UnexpectedFrame(&'static str),
    SequenceMismatch { expected: u8, got: u8 },
    DuplicateFrame { sn: u8 },
    Incomplete { declared: usize, received: usize },
    FlowControlOverflow,
}

impl fmt::Display for IsotpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IsotpError::PayloadEmpty => write!(f, "payload is empty"),
            IsotpError::PayloadTooLarge { len, max } => {
                write!(f, "payload length {} exceeds limit {}", len, max)
            }
            IsotpError::InvalidFrame(m) => write!(f, "invalid frame: {}", m),
            IsotpError::UnexpectedFrame(m) => write!(f, "unexpected frame: {}", m),
            IsotpError::SequenceMismatch { expected, got } => {
                write!(f, "sequence mismatch: expected SN={}, got SN={}", expected, got)
            }
            IsotpError::DuplicateFrame { sn } => write!(f, "duplicate consecutive frame SN={}", sn),
            IsotpError::Incomplete { declared, received } => {
                write!(f, "transfer incomplete: {} of {} bytes received", received, declared)
            }
            IsotpError::FlowControlOverflow => write!(f, "receiver signalled overflow, aborting"),
        }
    }
}

impl std::error::Error for IsotpError {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PciType {
    SingleFrame,
    FirstFrame,
    ConsecutiveFrame,
    FlowControl,
}

pub fn pci_type(frame: &CanFrame) -> Option<PciType> {
    let b = frame.bytes();
    if b.is_empty() {
        return None;
    }
    match b[0] >> 4 {
        0x0 => Some(PciType::SingleFrame),
        0x1 => Some(PciType::FirstFrame),
        0x2 => Some(PciType::ConsecutiveFrame),
        0x3 => Some(PciType::FlowControl),
        _ => None,
    }
}

/// Flow Control frame semantics (PCI nibble 0x3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowStatus {
    ClearToSend,
    Wait,
    Overflow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FlowControl {
    pub status: FlowStatus,
    /// Number of CFs the receiver accepts before the next FC. 0 = unlimited.
    pub block_size: u8,
    /// Minimum separation time between CFs, in milliseconds (virtual).
    pub stmin_ms: u8,
}

impl FlowControl {
    pub fn encode(&self, id: u32) -> CanFrame {
        let fs = match self.status {
            FlowStatus::ClearToSend => 0x0,
            FlowStatus::Wait => 0x1,
            FlowStatus::Overflow => 0x2,
        };
        CanFrame::new(id, &[0x30 | fs, self.block_size, self.stmin_ms])
            .expect("flow control frame always fits")
    }

    pub fn parse(frame: &CanFrame) -> Result<Self, IsotpError> {
        let b = frame.bytes();
        if b.len() < 3 || b[0] >> 4 != 0x3 {
            return Err(IsotpError::InvalidFrame("not a flow control frame"));
        }
        let status = match b[0] & 0x0F {
            0x0 => FlowStatus::ClearToSend,
            0x1 => FlowStatus::Wait,
            0x2 => FlowStatus::Overflow,
            _ => return Err(IsotpError::InvalidFrame("reserved flow status")),
        };
        Ok(FlowControl { status, block_size: b[1], stmin_ms: b[2] })
    }
}

/// Segment a payload into ISO-TP frames (SF, or FF + CFs with sequence
/// numbers wrapping 15 -> 0). Rejects payloads above [`MAX_PAYLOAD`].
pub fn segment(id: u32, payload: &[u8]) -> Result<Vec<CanFrame>, IsotpError> {
    if payload.is_empty() {
        return Err(IsotpError::PayloadEmpty);
    }
    if payload.len() > MAX_PAYLOAD {
        return Err(IsotpError::PayloadTooLarge { len: payload.len(), max: MAX_PAYLOAD });
    }
    if payload.len() <= SF_MAX_PAYLOAD {
        let mut buf = [0u8; CAN_FRAME_MAX_LEN];
        buf[0] = payload.len() as u8; // PCI: SF, SF_DL in low nibble
        buf[1..1 + payload.len()].copy_from_slice(payload);
        return Ok(vec![CanFrame::new(id, &buf[..1 + payload.len()])?]);
    }

    let total = payload.len();
    let mut frames = Vec::with_capacity(1 + (total - FF_DATA_BYTES + CF_DATA_BYTES - 1) / CF_DATA_BYTES);

    let mut ff = [0u8; CAN_FRAME_MAX_LEN];
    ff[0] = 0x10 | ((total >> 8) as u8 & 0x0F);
    ff[1] = (total & 0xFF) as u8;
    ff[2..2 + FF_DATA_BYTES].copy_from_slice(&payload[..FF_DATA_BYTES]);
    frames.push(CanFrame::new(id, &ff)?);

    let mut sn: u8 = 1;
    let mut offset = FF_DATA_BYTES;
    while offset < total {
        let end = (offset + CF_DATA_BYTES).min(total);
        let n = end - offset;
        let mut buf = [0u8; CAN_FRAME_MAX_LEN];
        buf[0] = 0x20 | sn;
        buf[1..1 + n].copy_from_slice(&payload[offset..end]);
        frames.push(CanFrame::new(id, &buf[..1 + n])?);
        sn = sn.wrapping_add(1) & 0x0F;
        offset = end;
    }
    Ok(frames)
}

/// Outcome of feeding one frame into the [`Reassembler`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RxOutcome {
    /// A complete payload (from a SF, or the final CF of a multi-frame transfer).
    Completed(Vec<u8>),
    FirstFrameAccepted { declared: usize },
    ConsecutiveAccepted { received: usize, declared: usize },
}

struct RxState {
    declared: usize,
    buf: Vec<u8>,
    expected_sn: u8,
}

/// Receive-side reassembly state machine with bounded memory.
///
/// `max_payload` caps the declared FF length; oversized First Frames are
/// rejected *before* any buffer is allocated.
pub struct Reassembler {
    max_payload: usize,
    state: Option<RxState>,
}

impl Reassembler {
    pub fn new(max_payload: usize) -> Self {
        Reassembler { max_payload: max_payload.min(MAX_PAYLOAD), state: None }
    }

    pub fn is_receiving(&self) -> bool {
        self.state.is_some()
    }

    /// `(received, declared)` while a multi-frame transfer is in progress.
    pub fn pending(&self) -> Option<(usize, usize)> {
        self.state.as_ref().map(|s| (s.buf.len(), s.declared))
    }

    pub fn reset(&mut self) {
        self.state = None;
    }

    pub fn on_frame(&mut self, frame: &CanFrame) -> Result<RxOutcome, IsotpError> {
        let bytes = frame.bytes();
        if bytes.is_empty() {
            return Err(IsotpError::InvalidFrame("empty CAN frame"));
        }
        match pci_type(frame).ok_or(IsotpError::InvalidFrame("unknown PCI type"))? {
            PciType::SingleFrame => {
                if let Some(st) = self.state.take() {
                    return Err(IsotpError::Incomplete { declared: st.declared, received: st.buf.len() });
                }
                let len = (bytes[0] & 0x0F) as usize;
                if len == 0 || len > SF_MAX_PAYLOAD {
                    return Err(IsotpError::InvalidFrame("invalid SF_DL"));
                }
                if bytes.len() - 1 < len {
                    return Err(IsotpError::InvalidFrame("SF_DL exceeds frame data"));
                }
                Ok(RxOutcome::Completed(bytes[1..1 + len].to_vec()))
            }
            PciType::FirstFrame => {
                if let Some(st) = self.state.take() {
                    return Err(IsotpError::Incomplete { declared: st.declared, received: st.buf.len() });
                }
                if bytes.len() < 2 {
                    return Err(IsotpError::InvalidFrame("first frame too short"));
                }
                let declared = (((bytes[0] & 0x0F) as usize) << 8) | bytes[1] as usize;
                if declared <= SF_MAX_PAYLOAD {
                    return Err(IsotpError::InvalidFrame("FF_DL too small; use a single frame"));
                }
                if declared > self.max_payload {
                    // Rejected before any allocation happens.
                    return Err(IsotpError::PayloadTooLarge { len: declared, max: self.max_payload });
                }
                let mut buf = Vec::with_capacity(declared);
                buf.extend_from_slice(&bytes[2..]);
                self.state = Some(RxState { declared, buf, expected_sn: 1 });
                Ok(RxOutcome::FirstFrameAccepted { declared })
            }
            PciType::ConsecutiveFrame => {
                let Some(mut st) = self.state.take() else {
                    return Err(IsotpError::UnexpectedFrame("consecutive frame without first frame"));
                };
                let sn = bytes[0] & 0x0F;
                if sn != st.expected_sn {
                    let prev_sn = (st.expected_sn + 15) & 0x0F;
                    return Err(if sn == prev_sn {
                        IsotpError::DuplicateFrame { sn }
                    } else {
                        IsotpError::SequenceMismatch { expected: st.expected_sn, got: sn }
                    });
                }
                let remaining = st.declared - st.buf.len();
                let take = remaining.min(bytes.len() - 1);
                st.buf.extend_from_slice(&bytes[1..1 + take]);
                st.expected_sn = st.expected_sn.wrapping_add(1) & 0x0F;
                if st.buf.len() == st.declared {
                    Ok(RxOutcome::Completed(st.buf))
                } else {
                    let received = st.buf.len();
                    let declared = st.declared;
                    self.state = Some(st);
                    Ok(RxOutcome::ConsecutiveAccepted { received, declared })
                }
            }
            PciType::FlowControl => {
                Err(IsotpError::UnexpectedFrame("flow control is not reassembled"))
            }
        }
    }
}

/// Deterministic virtual clock. No real sleeping ever happens; tests advance
/// time explicitly.
#[derive(Debug, Clone, Copy, Default)]
pub struct VirtualClock {
    now_ms: u64,
}

impl VirtualClock {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn now(&self) -> u64 {
        self.now_ms
    }
    pub fn advance(&mut self, ms: u64) {
        self.now_ms += ms;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SenderState {
    /// FF sent, waiting for a Flow Control frame.
    WaitFlowControl,
    /// Cleared to send CFs, honouring STmin and the remaining block budget.
    Sending,
    /// Block budget exhausted, waiting for the next Flow Control frame.
    WaitBlock,
    Done,
    Aborted,
}

/// Transmit-side state machine. Frames and timestamps are produced purely in
/// memory against a [`VirtualClock`].
pub struct Sender {
    frames: Vec<CanFrame>, // consecutive frames only; the first frame was handed out by `start`
    next: usize,
    block_remaining: u32,
    stmin_ms: u64,
    pub ready_at: u64,
    pub state: SenderState,
}

impl Sender {
    /// Segments the payload and returns the sender plus the first frame to
    /// transmit (SF or FF). For SF payloads the sender is immediately `Done`.
    pub fn start(id: u32, payload: &[u8]) -> Result<(Self, CanFrame), IsotpError> {
        let mut frames = segment(id, payload)?;
        let first = frames.remove(0);
        let state = if frames.is_empty() { SenderState::Done } else { SenderState::WaitFlowControl };
        Ok((
            Sender { frames, next: 0, block_remaining: 0, stmin_ms: 0, ready_at: 0, state },
            first,
        ))
    }

    pub fn on_flow_control(&mut self, fc: &FlowControl, clock: &VirtualClock) -> Result<(), IsotpError> {
        match fc.status {
            FlowStatus::ClearToSend => {
                self.block_remaining =
                    if fc.block_size == 0 { u32::MAX } else { fc.block_size as u32 };
                self.stmin_ms = fc.stmin_ms as u64;
                // STmin also separates the FC reception from the first CF.
                self.ready_at = clock.now() + self.stmin_ms;
                self.state = SenderState::Sending;
                Ok(())
            }
            FlowStatus::Wait => {
                self.state = SenderState::WaitFlowControl;
                Ok(())
            }
            FlowStatus::Overflow => {
                self.state = SenderState::Aborted;
                Err(IsotpError::FlowControlOverflow)
            }
        }
    }

    /// Returns the next CF if the state machine and virtual clock allow it.
    pub fn poll(&mut self, clock: &VirtualClock) -> Option<CanFrame> {
        if self.state != SenderState::Sending || clock.now() < self.ready_at {
            return None;
        }
        let frame = self.frames[self.next];
        self.next += 1;
        if self.next == self.frames.len() {
            self.state = SenderState::Done;
        } else if self.block_remaining != u32::MAX {
            self.block_remaining -= 1;
            if self.block_remaining == 0 {
                self.state = SenderState::WaitBlock;
                return Some(frame);
            }
        }
        if self.state == SenderState::Sending {
            self.ready_at = clock.now() + self.stmin_ms;
        }
        Some(frame)
    }
}

/// Convenience driver: transmits a payload honouring block size and STmin,
/// auto-answering with CTS flow control whenever the sender waits. Returns
/// `(virtual_timestamp_ms, frame)` pairs. Purely in-memory.
pub fn transmit(
    id: u32,
    payload: &[u8],
    block_size: u8,
    stmin_ms: u8,
    clock: &mut VirtualClock,
) -> Result<Vec<(u64, CanFrame)>, IsotpError> {
    let (mut sender, first) = Sender::start(id, payload)?;
    let mut out = vec![(clock.now(), first)];
    if sender.state == SenderState::Done {
        return Ok(out);
    }
    let fc = FlowControl { status: FlowStatus::ClearToSend, block_size, stmin_ms };
    sender.on_flow_control(&fc, clock)?;
    loop {
        match sender.state {
            SenderState::Sending => {
                if clock.now() < sender.ready_at {
                    clock.advance(sender.ready_at - clock.now());
                }
                if let Some(f) = sender.poll(clock) {
                    out.push((clock.now(), f));
                }
            }
            SenderState::WaitBlock => sender.on_flow_control(&fc, clock)?,
            SenderState::Done => break,
            SenderState::Aborted => return Err(IsotpError::FlowControlOverflow),
            SenderState::WaitFlowControl => break,
        }
    }
    Ok(out)
}
