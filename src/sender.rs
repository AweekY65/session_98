use crate::clock::VirtualClock;
use crate::error::IsoTpError;
use crate::frame::{self, CanFrame, FlowStatus, Pci, StMin};
use crate::{CF_DATA_BYTES, FF_DATA_BYTES, MAX_ISOTP_PAYLOAD, SF_MAX_PAYLOAD};

#[derive(Debug, Clone)]
pub struct SenderConfig {
    /// CAN id used for SF/FF/CF frames we transmit.
    pub tx_id: u32,
    /// CAN id on which Flow Control frames from the peer are accepted.
    pub rx_id: u32,
    /// If true, wait for a Flow Control frame after the First Frame before
    /// sending any Consecutive Frame (standard ISO-TP behaviour).
    pub require_flow_control: bool,
    /// N_Bs: max time to wait for a Flow Control frame.
    pub fc_timeout_us: u64,
}

impl Default for SenderConfig {
    fn default() -> Self {
        Self {
            tx_id: 0x700,
            rx_id: 0x708,
            require_flow_control: true,
            fc_timeout_us: 1_000_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxState {
    Idle,
    /// First Frame sent, waiting for Flow Control.
    WaitFc,
    /// Cleared to send Consecutive Frames.
    SendCf,
    Done,
    Failed,
}

/// Segments a payload into SF or FF+CF and honours Flow Control
/// (block size + STmin) using a caller-supplied virtual clock.
pub struct Sender {
    cfg: SenderConfig,
    payload: Vec<u8>,
    offset: usize,
    next_sn: u8,
    /// Remaining CF credit in the current block; `None` = unlimited (BS = 0).
    block_credit: Option<u8>,
    st_min: StMin,
    next_tx_us: u64,
    fc_wait_since_us: u64,
    state: TxState,
}

impl Sender {
    pub fn new(payload: Vec<u8>, cfg: SenderConfig) -> Result<Self, IsoTpError> {
        if payload.is_empty() || payload.len() > MAX_ISOTP_PAYLOAD {
            return Err(IsoTpError::PayloadTooLarge {
                len: payload.len(),
                max: MAX_ISOTP_PAYLOAD,
            });
        }
        Ok(Self {
            cfg,
            payload,
            offset: 0,
            next_sn: 1,
            block_credit: None,
            st_min: StMin::ZERO,
            next_tx_us: 0,
            fc_wait_since_us: 0,
            state: TxState::Idle,
        })
    }

    pub fn state(&self) -> TxState {
        self.state
    }

    pub fn is_done(&self) -> bool {
        self.state == TxState::Done
    }

    /// Emits the first frame of the transfer (SF or FF).
    pub fn start(&mut self, clock: &VirtualClock) -> Result<CanFrame, IsoTpError> {
        if self.state != TxState::Idle {
            return Err(IsoTpError::UnexpectedFrame("sender already started"));
        }
        if self.payload.len() <= SF_MAX_PAYLOAD {
            self.state = TxState::Done;
            return frame::single_frame(self.cfg.tx_id, &self.payload);
        }
        let total = self.payload.len() as u16;
        let first: Vec<u8> = self.payload[..FF_DATA_BYTES].to_vec();
        self.offset = FF_DATA_BYTES;
        self.next_sn = 1;
        if self.cfg.require_flow_control {
            self.state = TxState::WaitFc;
            self.fc_wait_since_us = clock.now_us();
        } else {
            self.state = TxState::SendCf;
            self.block_credit = None;
            self.st_min = StMin::ZERO;
            self.next_tx_us = clock.now_us();
        }
        frame::first_frame(self.cfg.tx_id, total, &first)
    }

    /// Feeds a received frame; only Flow Control frames addressed to us matter.
    pub fn on_frame(&mut self, frame: &CanFrame, clock: &VirtualClock) -> Result<(), IsoTpError> {
        if frame.id != self.cfg.rx_id {
            return Ok(());
        }
        let pci = frame::decode_pci(frame.as_slice())?;
        let (status, bs, st_min) = match pci {
            Pci::FlowControl {
                status,
                block_size,
                st_min,
            } => (status, block_size, st_min),
            _ => return Ok(()),
        };
        match status {
            FlowStatus::ClearToSend => {
                if self.state != TxState::WaitFc && self.state != TxState::SendCf {
                    return Err(IsoTpError::UnexpectedFrame("flow control while not sending"));
                }
                self.block_credit = if bs == 0 { None } else { Some(bs) };
                self.st_min = st_min;
                self.next_tx_us = clock.now_us();
                self.state = TxState::SendCf;
                Ok(())
            }
            FlowStatus::Wait => {
                if self.state == TxState::SendCf || self.state == TxState::WaitFc {
                    self.state = TxState::WaitFc;
                    self.fc_wait_since_us = clock.now_us();
                }
                Ok(())
            }
            FlowStatus::Overflow => {
                self.state = TxState::Failed;
                Err(IsoTpError::FlowControlOverflow)
            }
        }
    }

    /// Returns the next Consecutive Frame when one is due, else `None`.
    pub fn poll(&mut self, clock: &VirtualClock) -> Option<CanFrame> {
        if self.state != TxState::SendCf {
            return None;
        }
        let now = clock.now_us();
        if now < self.next_tx_us {
            return None;
        }
        if let Some(credit) = self.block_credit {
            if credit == 0 {
                self.state = TxState::WaitFc;
                self.fc_wait_since_us = now;
                return None;
            }
        }
        let end = (self.offset + CF_DATA_BYTES).min(self.payload.len());
        let chunk = self.payload[self.offset..end].to_vec();
        let sn = self.next_sn;
        let frame = frame::consecutive_frame(self.cfg.tx_id, sn, &chunk).ok()?;
        self.offset = end;
        self.next_sn = (self.next_sn + 1) & 0x0F;
        if let Some(credit) = &mut self.block_credit {
            *credit -= 1;
        }
        self.next_tx_us = now + self.st_min.as_micros();
        if self.offset >= self.payload.len() {
            self.state = TxState::Done;
        }
        Some(frame)
    }

    /// Must be called periodically; enforces the N_Bs flow-control timeout.
    pub fn on_tick(&mut self, clock: &VirtualClock) -> Result<(), IsoTpError> {
        if self.state == TxState::WaitFc
            && clock.now_us().saturating_sub(self.fc_wait_since_us) > self.cfg.fc_timeout_us
        {
            self.state = TxState::Failed;
            return Err(IsoTpError::Timeout("N_Bs: flow control not received"));
        }
        Ok(())
    }
}
