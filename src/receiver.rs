use crate::clock::VirtualClock;
use crate::error::IsoTpError;
use crate::frame::{self, CanFrame, FlowStatus, Pci, StMin};
use crate::{CAN_FRAME_LEN, CF_DATA_BYTES, FF_DATA_BYTES};

#[derive(Debug, Clone)]
pub struct ReceiverConfig {
    /// CAN id of frames we accept.
    pub rx_id: u32,
    /// CAN id used for Flow Control frames we emit.
    pub fc_tx_id: u32,
    /// Hard cap on a single reassembled payload. A First Frame declaring more
    /// is rejected before any buffer is allocated.
    pub max_payload: usize,
    /// Hard cap on total reassembly buffer memory held by this receiver.
    pub max_buffer: usize,
    /// Block size advertised in our Flow Control frames (0 = no blocking).
    pub block_size: u8,
    /// STmin advertised in our Flow Control frames.
    pub st_min: StMin,
    /// N_Cr: max gap between consecutive frames before the transfer aborts.
    pub cf_timeout_us: u64,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            rx_id: 0x700,
            fc_tx_id: 0x708,
            max_payload: crate::MAX_ISOTP_PAYLOAD,
            max_buffer: crate::MAX_ISOTP_PAYLOAD,
            block_size: 8,
            st_min: StMin::ZERO,
            cf_timeout_us: 1_000_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxState {
    Idle,
    Receiving,
}

#[derive(Debug)]
pub enum RxOutput {
    None,
    /// Receiver asks the peer to (re)start / continue sending.
    SendFlowControl(CanFrame),
    /// A complete payload of exactly the declared length was reassembled.
    Complete(Vec<u8>),
}

/// Reassembles SF / FF+CF streams with sequence, duplicate, length and
/// timeout validation.
pub struct Receiver {
    cfg: ReceiverConfig,
    state: RxState,
    buffer: Vec<u8>,
    declared: usize,
    expected_sn: u8,
    last_sn: Option<u8>,
    block_received: u8,
    last_rx_us: u64,
}

impl Receiver {
    pub fn new(cfg: ReceiverConfig) -> Self {
        Self {
            cfg,
            state: RxState::Idle,
            buffer: Vec::new(),
            declared: 0,
            expected_sn: 1,
            last_sn: None,
            block_received: 0,
            last_rx_us: 0,
        }
    }

    pub fn state(&self) -> RxState {
        self.state
    }

    pub fn buffered_len(&self) -> usize {
        self.buffer.len()
    }

    fn reset(&mut self) {
        self.state = RxState::Idle;
        self.buffer = Vec::new();
        self.declared = 0;
        self.expected_sn = 1;
        self.last_sn = None;
        self.block_received = 0;
    }

    fn make_fc(&self) -> CanFrame {
        frame::flow_control(
            self.cfg.fc_tx_id,
            FlowStatus::ClearToSend,
            self.cfg.block_size,
            self.cfg.st_min,
        )
    }

    pub fn on_frame(
        &mut self,
        frame: &CanFrame,
        clock: &VirtualClock,
    ) -> Result<RxOutput, IsoTpError> {
        if frame.id != self.cfg.rx_id {
            return Ok(RxOutput::None);
        }
        let pci = frame::decode_pci(frame.as_slice())?;
        match pci {
            Pci::SingleFrame { len } => {
                if self.state != RxState::Idle {
                    self.reset();
                    return Err(IsoTpError::UnexpectedFrame(
                        "single frame during multi-frame reception",
                    ));
                }
                let payload = frame.as_slice()[1..1 + len as usize].to_vec();
                if payload.len() > self.cfg.max_payload {
                    return Err(IsoTpError::PayloadTooLarge {
                        len: payload.len(),
                        max: self.cfg.max_payload,
                    });
                }
                Ok(RxOutput::Complete(payload))
            }
            Pci::FirstFrame { len } => self.on_first_frame(frame, len, clock),
            Pci::ConsecutiveFrame { sn } => self.on_consecutive_frame(frame, sn, clock),
            Pci::FlowControl { .. } => Ok(RxOutput::None),
        }
    }

    fn on_first_frame(
        &mut self,
        frame: &CanFrame,
        declared: u16,
        clock: &VirtualClock,
    ) -> Result<RxOutput, IsoTpError> {
        self.reset();
        let declared = declared as usize;
        // Reject malicious / oversized declarations BEFORE allocating.
        if declared > self.cfg.max_payload {
            return Err(IsoTpError::PayloadTooLarge {
                len: declared,
                max: self.cfg.max_payload,
            });
        }
        if declared > self.cfg.max_buffer {
            return Err(IsoTpError::BufferLimitExceeded {
                needed: declared,
                max: self.cfg.max_buffer,
            });
        }
        if frame.len as usize != CAN_FRAME_LEN {
            return Err(IsoTpError::LengthMismatch {
                declared: CAN_FRAME_LEN,
                actual: frame.len as usize,
            });
        }
        self.buffer = Vec::with_capacity(declared);
        self.buffer
            .extend_from_slice(&frame.data[2..2 + FF_DATA_BYTES]);
        self.declared = declared;
        self.expected_sn = 1;
        self.last_sn = None;
        self.block_received = 0;
        self.last_rx_us = clock.now_us();
        self.state = RxState::Receiving;
        Ok(RxOutput::SendFlowControl(self.make_fc()))
    }

    fn on_consecutive_frame(
        &mut self,
        frame: &CanFrame,
        sn: u8,
        clock: &VirtualClock,
    ) -> Result<RxOutput, IsoTpError> {
        if self.state != RxState::Receiving {
            return Err(IsoTpError::UnexpectedFrame(
                "consecutive frame without first frame",
            ));
        }
        if frame.len as usize != CAN_FRAME_LEN {
            self.reset();
            return Err(IsoTpError::LengthMismatch {
                declared: CAN_FRAME_LEN,
                actual: frame.len as usize,
            });
        }
        if sn != self.expected_sn {
            let err = if self.last_sn == Some(sn) {
                IsoTpError::DuplicateSequence { sn }
            } else {
                IsoTpError::SequenceMismatch {
                    expected: self.expected_sn,
                    got: sn,
                }
            };
            self.reset();
            return Err(err);
        }
        let remaining = self.declared - self.buffer.len();
        if remaining == 0 {
            self.reset();
            return Err(IsoTpError::LengthMismatch {
                declared: self.declared,
                actual: self.declared + CF_DATA_BYTES,
            });
        }
        let take = remaining.min(CF_DATA_BYTES);
        self.buffer.extend_from_slice(&frame.data[1..1 + take]);
        self.last_rx_us = clock.now_us();
        self.last_sn = Some(sn);

        if self.buffer.len() == self.declared {
            let done = std::mem::take(&mut self.buffer);
            self.reset();
            return Ok(RxOutput::Complete(done));
        }
        if self.buffer.len() > self.declared || self.buffer.len() > self.cfg.max_buffer {
            let actual = self.buffer.len();
            let declared = self.declared;
            self.reset();
            return Err(IsoTpError::LengthMismatch { declared, actual });
        }
        self.expected_sn = (self.expected_sn + 1) & 0x0F;
        self.block_received += 1;
        if self.cfg.block_size != 0 && self.block_received >= self.cfg.block_size {
            self.block_received = 0;
            return Ok(RxOutput::SendFlowControl(self.make_fc()));
        }
        Ok(RxOutput::None)
    }

    /// Must be called periodically; enforces the N_Cr consecutive-frame timeout
    /// so a transfer that is short of its declared length never succeeds silently.
    pub fn on_tick(&mut self, clock: &VirtualClock) -> Result<(), IsoTpError> {
        if self.state == RxState::Receiving
            && clock.now_us().saturating_sub(self.last_rx_us) > self.cfg.cf_timeout_us
        {
            self.reset();
            return Err(IsoTpError::Timeout(
                "N_Cr: consecutive frame not received (incomplete payload)",
            ));
        }
        Ok(())
    }
}
