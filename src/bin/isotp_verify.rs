//! Standalone verification runner (no `cargo test` required).
//!
//!     cargo run --bin isotp_verify
//!
//! Prints one PASS/FAIL line per check and exits non-zero on any failure.
//! All data lives in memory; nothing touches SocketCAN, vehicles, ECUs,
//! networks or real-time sleeps.

use isotp_local::*;

struct Harness {
    pass: usize,
    fail: usize,
}

impl Harness {
    fn run<F>(&mut self, name: &str, f: F)
    where
        F: FnOnce() -> Result<(), String>,
    {
        match f() {
            Ok(()) => {
                self.pass += 1;
                println!("PASS  {}", name);
            }
            Err(e) => {
                self.fail += 1;
                println!("FAIL  {}  ->  {}", name, e);
            }
        }
    }
}

fn ensure(cond: bool, msg: impl Into<String>) -> Result<(), String> {
    if cond { Ok(()) } else { Err(msg.into()) }
}

fn sample_payload(len: usize) -> Vec<u8> {
    (0..len).map(|i| (i % 251) as u8).collect()
}

fn reassemble_all(frames: &[CanFrame], max: usize) -> Result<Vec<u8>, IsotpError> {
    let mut rx = Reassembler::new(max);
    let mut out = None;
    for f in frames {
        if let RxOutcome::Completed(data) = rx.on_frame(f)? {
            out = Some(data);
        }
    }
    out.ok_or(IsotpError::UnexpectedFrame("transfer never completed"))
}

fn main() {
    let mut h = Harness { pass: 0, fail: 0 };

    h.run("single_frame_roundtrip", || {
        let payload = b"hello";
        let frames = segment(0x7E0, payload).map_err(|e| e.to_string())?;
        ensure(frames.len() == 1, "expected exactly one frame")?;
        ensure(frames[0].bytes()[0] == 0x05, "SF PCI/SF_DL byte wrong")?;
        let got = reassemble_all(&frames, MAX_PAYLOAD).map_err(|e| e.to_string())?;
        ensure(got == payload, "payload mismatch")
    });

    h.run("single_frame_max_7_bytes", || {
        let payload = sample_payload(7);
        let frames = segment(0x7E0, &payload).map_err(|e| e.to_string())?;
        ensure(frames.len() == 1, "7 bytes must fit in one SF")?;
        let got = reassemble_all(&frames, MAX_PAYLOAD).map_err(|e| e.to_string())?;
        ensure(got == payload, "payload mismatch")
    });

    h.run("multi_frame_roundtrip", || {
        let payload = sample_payload(100);
        let frames = segment(0x7E0, &payload).map_err(|e| e.to_string())?;
        // 1 FF (6 bytes) + ceil(94/7) = 14 CFs
        ensure(frames.len() == 15, format!("expected 15 frames, got {}", frames.len()))?;
        ensure(frames[0].bytes()[0] >> 4 == 0x1, "first frame PCI wrong")?;
        ensure(frames[0].bytes()[1] == 100, "FF_DL low byte wrong")?;
        for (i, f) in frames[1..].iter().enumerate() {
            let want_sn = ((i + 1) & 0x0F) as u8;
            ensure(f.bytes()[0] == 0x20 | want_sn, format!("CF {} SN wrong", i))?;
        }
        let got = reassemble_all(&frames, MAX_PAYLOAD).map_err(|e| e.to_string())?;
        ensure(got == payload, "reassembled payload mismatch")
    });

    h.run("sequence_number_wraparound", || {
        let payload = sample_payload(300); // 42 CFs -> SN wraps 15 -> 0 multiple times
        let frames = segment(0x7E0, &payload).map_err(|e| e.to_string())?;
        ensure(frames.len() == 43, format!("expected 43 frames, got {}", frames.len()))?;
        let sns: Vec<u8> = frames[1..].iter().map(|f| f.bytes()[0] & 0x0F).collect();
        ensure(sns[14] == 0x0F && sns[15] == 0x00 && sns[16] == 0x01,
               "SN did not wrap 15 -> 0 -> 1")?;
        let got = reassemble_all(&frames, MAX_PAYLOAD).map_err(|e| e.to_string())?;
        ensure(got == payload, "payload mismatch after SN wraparound")
    });

    h.run("dropped_frame_detected", || {
        let payload = sample_payload(100);
        let frames = segment(0x7E0, &payload).map_err(|e| e.to_string())?;
        let mut rx = Reassembler::new(MAX_PAYLOAD);
        let mut dropped_err = None;
        for (i, f) in frames.iter().enumerate() {
            if i == 3 { continue; } // drop one consecutive frame
            if let Err(e) = rx.on_frame(f) {
                dropped_err = Some(e);
                break;
            }
        }
        match dropped_err {
            Some(IsotpError::SequenceMismatch { expected: 3, got: 4 }) => Ok(()),
            other => Err(format!("expected SequenceMismatch 3/4, got {:?}", other)),
        }
    });

    h.run("duplicate_frame_detected", || {
        let payload = sample_payload(100);
        let frames = segment(0x7E0, &payload).map_err(|e| e.to_string())?;
        let mut rx = Reassembler::new(MAX_PAYLOAD);
        let mut err = None;
        for (i, f) in frames.iter().enumerate() {
            if let Err(e) = rx.on_frame(f) { err = Some(e); break; }
            if i == 2 {
                // replay the same consecutive frame
                if let Err(e) = rx.on_frame(f) { err = Some(e); break; }
            }
        }
        match err {
            Some(IsotpError::DuplicateFrame { sn: 2 }) => Ok(()),
            other => Err(format!("expected DuplicateFrame sn=2, got {:?}", other)),
        }
    });

    h.run("length_error_sf_dl_mismatch", || {
        // SF declares 7 bytes but the frame only carries 3 data bytes.
        let bad = CanFrame::new(0x7E0, &[0x07, 0xAA, 0xBB, 0xCC]).map_err(|e| e.to_string())?;
        let mut rx = Reassembler::new(MAX_PAYLOAD);
        match rx.on_frame(&bad) {
            Err(IsotpError::InvalidFrame(_)) => Ok(()),
            other => Err(format!("expected InvalidFrame, got {:?}", other)),
        }
    });

    h.run("length_error_truncated_transfer", || {
        let payload = sample_payload(100);
        let frames = segment(0x7E0, &payload).map_err(|e| e.to_string())?;
        let mut rx = Reassembler::new(MAX_PAYLOAD);
        // Feed FF + only half of the CFs, then a new SF: the truncated
        // transfer must be reported, never silently accepted.
        for f in &frames[..8] {
            rx.on_frame(f).map_err(|e| e.to_string())?;
        }
        ensure(rx.is_receiving(), "receiver should be mid-transfer")?;
        let sf = segment(0x7E0, b"ping").map_err(|e| e.to_string())?;
        match rx.on_frame(&sf[0]) {
            Err(IsotpError::Incomplete { declared: 100, received }) => {
                ensure(received < 100, "received count wrong")
            }
            other => Err(format!("expected Incomplete, got {:?}", other)),
        }
    });

    h.run("length_error_extra_data_after_completion", || {
        let payload = sample_payload(20); // FF + 2 CFs exactly
        let frames = segment(0x7E0, &payload).map_err(|e| e.to_string())?;
        let mut rx = Reassembler::new(MAX_PAYLOAD);
        let mut completed = false;
        for f in &frames {
            if let RxOutcome::Completed(_) = rx.on_frame(f).map_err(|e| e.to_string())? {
                completed = true;
            }
        }
        ensure(completed, "transfer should have completed")?;
        // One more CF beyond the declared length must not succeed silently.
        let extra = CanFrame::new(0x7E0, &[0x23, 1, 2, 3, 4, 5, 6, 7]).map_err(|e| e.to_string())?;
        match rx.on_frame(&extra) {
            Err(IsotpError::UnexpectedFrame(_)) => Ok(()),
            other => Err(format!("expected UnexpectedFrame, got {:?}", other)),
        }
    });

    h.run("flow_control_encode_decode", || {
        let fc = FlowControl { status: FlowStatus::ClearToSend, block_size: 8, stmin_ms: 5 };
        let frame = fc.encode(0x7E8);
        ensure(frame.bytes()[0] == 0x30 && frame.bytes()[1] == 8 && frame.bytes()[2] == 5,
               "FC encoding wrong")?;
        let parsed = FlowControl::parse(&frame).map_err(|e| e.to_string())?;
        ensure(parsed == fc, "FC roundtrip mismatch")?;
        let not_fc = CanFrame::new(0x7E8, &[0x10, 0x00]).map_err(|e| e.to_string())?;
        ensure(FlowControl::parse(&not_fc).is_err(), "non-FC frame parsed as FC")
    });

    h.run("flow_control_block_size", || {
        let payload = sample_payload(30); // FF + 4 CFs
        let (mut tx, _ff) = Sender::start(0x7E0, &payload).map_err(|e| e.to_string())?;
        let clock = VirtualClock::new();
        let cts2 = FlowControl { status: FlowStatus::ClearToSend, block_size: 2, stmin_ms: 0 };
        tx.on_flow_control(&cts2, &clock).map_err(|e| e.to_string())?;
        ensure(tx.poll(&clock).is_some(), "CF1 missing")?;
        ensure(tx.poll(&clock).is_some(), "CF2 missing")?;
        ensure(tx.poll(&clock).is_none(), "block of 2 must pause the sender")?;
        ensure(tx.state == SenderState::WaitBlock, "sender should wait for next FC")?;
        tx.on_flow_control(&cts2, &clock).map_err(|e| e.to_string())?;
        ensure(tx.poll(&clock).is_some(), "CF3 missing")?;
        ensure(tx.poll(&clock).is_some(), "CF4 missing")?;
        ensure(tx.state == SenderState::Done, "sender should be done")
    });

    h.run("flow_control_stmin_virtual_clock", || {
        let payload = sample_payload(50); // FF + 7 CFs
        let mut clock = VirtualClock::new();
        let log = transmit(0x7E0, &payload, 0, 5, &mut clock).map_err(|e| e.to_string())?;
        ensure(log.len() == 8, format!("expected 8 frames, got {}", log.len()))?;
        for w in log.windows(2) {
            ensure(w[1].0 - w[0].0 == 5, "STmin=5ms spacing violated")?;
        }
        ensure(clock.now() == 35, format!("virtual time should be 35ms, got {}", clock.now()))
    });

    h.run("flow_control_wait_and_overflow", || {
        let payload = sample_payload(30);
        let (mut tx, _ff) = Sender::start(0x7E0, &payload).map_err(|e| e.to_string())?;
        let clock = VirtualClock::new();
        let wait = FlowControl { status: FlowStatus::Wait, block_size: 0, stmin_ms: 0 };
        tx.on_flow_control(&wait, &clock).map_err(|e| e.to_string())?;
        ensure(tx.poll(&clock).is_none(), "sender must not send while FS=Wait")?;
        ensure(tx.state == SenderState::WaitFlowControl, "state should be WaitFlowControl")?;
        let overflow = FlowControl { status: FlowStatus::Overflow, block_size: 0, stmin_ms: 0 };
        match tx.on_flow_control(&overflow, &clock) {
            Err(IsotpError::FlowControlOverflow) => {}
            other => return Err(format!("expected FlowControlOverflow, got {:?}", other)),
        }
        ensure(tx.state == SenderState::Aborted, "sender should be aborted")
    });

    h.run("oversized_first_frame_rejected", || {
        // Malicious FF declares the 12-bit maximum while the receiver only
        // allows 64 bytes: reject before allocating anything.
        let evil = CanFrame::new(0x7E0, &[0x1F, 0xFF, 1, 2, 3, 4, 5, 6]).map_err(|e| e.to_string())?;
        let mut rx = Reassembler::new(64);
        match rx.on_frame(&evil) {
            Err(IsotpError::PayloadTooLarge { len: 4095, max: 64 }) => {}
            other => return Err(format!("expected PayloadTooLarge, got {:?}", other)),
        }
        ensure(!rx.is_receiving(), "no transfer state may be kept")?;
        ensure(rx.pending().is_none(), "no buffer may be cached")
    });

    h.run("segment_rejects_oversized_payload", || {
        let huge = vec![0u8; MAX_PAYLOAD + 1];
        match segment(0x7E0, &huge) {
            Err(IsotpError::PayloadTooLarge { len, max }) => {
                ensure(len == MAX_PAYLOAD + 1 && max == MAX_PAYLOAD, "wrong limits reported")
            }
            other => Err(format!("expected PayloadTooLarge, got {:?}", other)),
        }
    });

    h.run("end_to_end_transmit_reassemble", || {
        let payload = sample_payload(500);
        let mut clock = VirtualClock::new();
        let log = transmit(0x7E0, &payload, 4, 1, &mut clock).map_err(|e| e.to_string())?;
        let frames: Vec<CanFrame> = log.iter().map(|(_, f)| *f).collect();
        let got = reassemble_all(&frames, 1024).map_err(|e| e.to_string())?;
        ensure(got == payload, "end-to-end payload mismatch")
    });

    println!("--------------------------------------------------");
    println!("{} passed, {} failed", h.pass, h.fail);
    std::process::exit(if h.fail == 0 { 0 } else { 1 });
}
