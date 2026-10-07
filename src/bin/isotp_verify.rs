//! Standalone ISO-TP verification runner.
//!
//! Usage: cargo run --bin isotp_verify
//! Prints one PASS/FAIL line per check and exits non-zero on any failure.

use isotp_local::frame;
use isotp_local::*;

type Check = Result<(), String>;

fn err<T: std::fmt::Display>(e: T) -> String {
    e.to_string()
}

/// Drives a sender/receiver pair (with flow control handshake) until the
/// receiver reports a complete payload. All timing is virtual.
fn pump(tx: &mut Sender, rx: &mut Receiver, clock: &mut VirtualClock) -> Result<Vec<u8>, IsoTpError> {
    let first = tx.start(clock)?;
    let mut pending_fc: Vec<CanFrame> = Vec::new();
    match rx.on_frame(&first, clock)? {
        RxOutput::SendFlowControl(fc) => pending_fc.push(fc),
        RxOutput::Complete(d) => return Ok(d),
        RxOutput::None => {}
    }
    for _ in 0..100_000 {
        for fc in pending_fc.drain(..) {
            tx.on_frame(&fc, clock)?;
        }
        let mut progressed = false;
        while let Some(cf) = tx.poll(clock) {
            progressed = true;
            match rx.on_frame(&cf, clock)? {
                RxOutput::SendFlowControl(fc) => pending_fc.push(fc),
                RxOutput::Complete(d) => return Ok(d),
                RxOutput::None => {}
            }
        }
        tx.on_tick(clock)?;
        rx.on_tick(clock)?;
        if !progressed && pending_fc.is_empty() {
            clock.advance_us(1_000); // let STmin elapse
        }
    }
    Err(IsoTpError::Timeout("pump iteration guard"))
}

/// Segments a payload without flow control and returns all frames.
fn segment_all(payload: &[u8], clock: &mut VirtualClock) -> Result<Vec<CanFrame>, IsoTpError> {
    let cfg = SenderConfig {
        require_flow_control: false,
        ..Default::default()
    };
    let mut tx = Sender::new(payload.to_vec(), cfg)?;
    let mut frames = vec![tx.start(clock)?];
    while let Some(f) = tx.poll(clock) {
        frames.push(f);
    }
    Ok(frames)
}

fn check_single_frame_roundtrip() -> Check {
    let mut clock = VirtualClock::new();
    let payload = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x42];
    let frames = segment_all(&payload, &mut clock).map_err(err)?;
    if frames.len() != 1 {
        return Err(format!("expected 1 frame, got {}", frames.len()));
    }
    let mut rx = Receiver::new(ReceiverConfig::default());
    match rx.on_frame(&frames[0], &clock).map_err(err)? {
        RxOutput::Complete(d) if d == payload => Ok(()),
        other => Err(format!("unexpected receiver output: {:?}", other)),
    }
}

fn check_sf_length_validation() -> Check {
    // SF declares 5 bytes but carries only 2.
    let short = CanFrame::new(0x700, &[0x05, 0xAA, 0xBB]).map_err(err)?;
    match frame::decode_pci(short.as_slice()) {
        Err(IsoTpError::LengthMismatch { .. }) => {}
        other => return Err(format!("expected LengthMismatch, got {:?}", other)),
    }
    // SF length nibble of 0 is invalid.
    let zero = CanFrame::new(0x700, &[0x00, 0xAA]).map_err(err)?;
    match frame::decode_pci(zero.as_slice()) {
        Err(IsoTpError::InvalidSingleFrameLength(0)) => Ok(()),
        other => Err(format!("expected InvalidSingleFrameLength, got {:?}", other)),
    }
}

fn check_multi_frame_roundtrip() -> Check {
    let mut clock = VirtualClock::new();
    let payload: Vec<u8> = (0..100u32).map(|i| (i * 7 % 251) as u8).collect();
    let mut tx = Sender::new(payload.clone(), SenderConfig::default()).map_err(err)?;
    let mut rx = Receiver::new(ReceiverConfig::default());
    let got = pump(&mut tx, &mut rx, &mut clock).map_err(err)?;
    if got != payload {
        return Err("reassembled payload differs".into());
    }
    if !tx.is_done() {
        return Err("sender not done".into());
    }
    Ok(())
}

fn check_sequence_wraparound() -> Check {
    // 300 bytes -> FF + 42 CFs, sequence number wraps 15 -> 0 multiple times.
    let mut clock = VirtualClock::new();
    let payload: Vec<u8> = (0..300u32).map(|i| (i % 256) as u8).collect();
    let frames = segment_all(&payload, &mut clock).map_err(err)?;
    if frames.len() != 1 + 42 {
        return Err(format!("expected 43 frames, got {}", frames.len()));
    }
    let mut rx = Receiver::new(ReceiverConfig::default());
    let mut out = None;
    for f in &frames {
        if let RxOutput::Complete(d) = rx.on_frame(f, &clock).map_err(err)? {
            out = Some(d);
        }
    }
    match out {
        Some(d) if d == payload => Ok(()),
        _ => Err("wraparound reassembly failed".into()),
    }
}

fn check_dropped_frame_detected() -> Check {
    let mut clock = VirtualClock::new();
    let payload = vec![0x11u8; 60];
    let frames = segment_all(&payload, &mut clock).map_err(err)?;
    let mut rx = Receiver::new(ReceiverConfig::default());
    // Feed FF + CF1, skip CF2, then CF3 -> sequence mismatch must fire.
    rx.on_frame(&frames[0], &clock).map_err(err)?;
    rx.on_frame(&frames[1], &clock).map_err(err)?;
    match rx.on_frame(&frames[3], &clock) {
        Err(IsoTpError::SequenceMismatch { expected: 2, got: 3 }) => Ok(()),
        other => Err(format!("expected SequenceMismatch, got {:?}", other)),
    }
}

fn check_duplicate_frame_detected() -> Check {
    let mut clock = VirtualClock::new();
    let payload = vec![0x22u8; 60];
    let frames = segment_all(&payload, &mut clock).map_err(err)?;
    let mut rx = Receiver::new(ReceiverConfig::default());
    rx.on_frame(&frames[0], &clock).map_err(err)?;
    rx.on_frame(&frames[1], &clock).map_err(err)?;
    match rx.on_frame(&frames[1], &clock) {
        Err(IsoTpError::DuplicateSequence { sn: 1 }) => Ok(()),
        other => Err(format!("expected DuplicateSequence, got {:?}", other)),
    }
}

fn check_length_errors() -> Check {
    let mut clock = VirtualClock::new();
    // (a) Declared length never reached -> N_Cr timeout, no silent success.
    let mut rx = Receiver::new(ReceiverConfig::default());
    let ff = frame::first_frame(0x700, 50, &[0x01; 6]).map_err(err)?;
    rx.on_frame(&ff, &clock).map_err(err)?;
    let cf = frame::consecutive_frame(0x700, 1, &[0x02; 7]).map_err(err)?;
    rx.on_frame(&cf, &clock).map_err(err)?;
    clock.advance_ms(2_000);
    match rx.on_tick(&clock) {
        Err(IsoTpError::Timeout(_)) => {}
        other => return Err(format!("expected Timeout, got {:?}", other)),
    }
    // (b) More data than declared -> extra CF after completion is rejected.
    let mut rx2 = Receiver::new(ReceiverConfig::default());
    let ff2 = frame::first_frame(0x700, 8, &[0x01; 6]).map_err(err)?;
    rx2.on_frame(&ff2, &clock).map_err(err)?;
    let cf2 = frame::consecutive_frame(0x700, 1, &[0x02; 7]).map_err(err)?;
    match rx2.on_frame(&cf2, &clock).map_err(err)? {
        RxOutput::Complete(d) if d.len() == 8 => {}
        other => return Err(format!("expected Complete(8), got {:?}", other)),
    }
    let cf3 = frame::consecutive_frame(0x700, 2, &[0x03; 7]).map_err(err)?;
    match rx2.on_frame(&cf3, &clock) {
        Err(IsoTpError::UnexpectedFrame(_)) => Ok(()),
        other => Err(format!("expected UnexpectedFrame, got {:?}", other)),
    }
}

fn check_flow_control_block_size() -> Check {
    let clock = VirtualClock::new();
    let payload = vec![0x33u8; 40]; // FF + 5 CFs
    let mut tx = Sender::new(payload, SenderConfig::default()).map_err(err)?;
    let mut rx = Receiver::new(ReceiverConfig {
        block_size: 2,
        ..Default::default()
    });
    let ff = tx.start(&clock).map_err(err)?;
    let fc = match rx.on_frame(&ff, &clock).map_err(err)? {
        RxOutput::SendFlowControl(fc) => fc,
        _ => return Err("expected FC after FF".into()),
    };
    tx.on_frame(&fc, &clock).map_err(err)?;
    // Exactly BS=2 frames, then the sender must stop.
    let mut sent = 0;
    while let Some(cf) = tx.poll(&clock) {
        sent += 1;
        rx.on_frame(&cf, &clock).map_err(err)?;
    }
    if sent != 2 {
        return Err(format!("expected 2 CFs in block, got {}", sent));
    }
    if tx.state() != TxState::WaitFc {
        return Err(format!("expected WaitFc, got {:?}", tx.state()));
    }
    // The receiver emitted a fresh FC after the 2nd CF (discarded above).
    // Now prove the full BS=2 handshake completes on a fresh pair.
    let mut clock2 = VirtualClock::new();
    let payload2 = vec![0x44u8; 40];
    let mut tx2 = Sender::new(payload2.clone(), SenderConfig::default()).map_err(err)?;
    let mut rx2 = Receiver::new(ReceiverConfig {
        block_size: 2,
        ..Default::default()
    });
    let got = pump(&mut tx2, &mut rx2, &mut clock2).map_err(err)?;
    if got != payload2 {
        return Err("block-size handshake payload mismatch".into());
    }
    Ok(())
}

fn check_stmin_virtual_clock() -> Check {
    let mut clock = VirtualClock::new();
    let payload = vec![0x55u8; 50];
    let mut tx = Sender::new(payload, SenderConfig::default()).map_err(err)?;
    tx.start(&clock).map_err(err)?;
    let fc = frame::flow_control(0x708, FlowStatus::ClearToSend, 0, StMin::from_millis(10));
    tx.on_frame(&fc, &clock).map_err(err)?;
    if tx.poll(&clock).is_none() {
        return Err("first CF should be immediate after FC".into());
    }
    if tx.poll(&clock).is_some() {
        return Err("second CF must wait for STmin".into());
    }
    clock.advance_ms(5);
    if tx.poll(&clock).is_some() {
        return Err("second CF arrived before STmin elapsed".into());
    }
    clock.advance_ms(5);
    if tx.poll(&clock).is_none() {
        return Err("second CF missing after STmin elapsed".into());
    }
    Ok(())
}

fn check_flow_control_wait_and_overflow() -> Check {
    let clock = VirtualClock::new();
    let payload = vec![0x66u8; 50];
    let mut tx = Sender::new(payload, SenderConfig::default()).map_err(err)?;
    tx.start(&clock).map_err(err)?;
    // Wait keeps the sender parked.
    let wait = frame::flow_control(0x708, FlowStatus::Wait, 0, StMin::ZERO);
    tx.on_frame(&wait, &clock).map_err(err)?;
    if tx.poll(&clock).is_some() {
        return Err("sender must not transmit while FC.Wait".into());
    }
    // CTS resumes.
    let cts = frame::flow_control(0x708, FlowStatus::ClearToSend, 0, StMin::ZERO);
    tx.on_frame(&cts, &clock).map_err(err)?;
    if tx.poll(&clock).is_none() {
        return Err("sender should resume after FC.CTS".into());
    }
    // Overflow aborts.
    let ovf = frame::flow_control(0x708, FlowStatus::Overflow, 0, StMin::ZERO);
    match tx.on_frame(&ovf, &clock) {
        Err(IsoTpError::FlowControlOverflow) => Ok(()),
        other => Err(format!("expected FlowControlOverflow, got {:?}", other)),
    }
}

fn check_oversized_first_frame_rejected() -> Check {
    let clock = VirtualClock::new();
    let mut rx = Receiver::new(ReceiverConfig {
        max_payload: 64,
        max_buffer: 64,
        ..Default::default()
    });
    // Malicious FF declares 4095 bytes against a 64-byte cap.
    let evil = frame::first_frame(0x700, 4095, &[0xAA; 6]).map_err(err)?;
    match rx.on_frame(&evil, &clock) {
        Err(IsoTpError::PayloadTooLarge { len: 4095, max: 64 }) => {}
        other => return Err(format!("expected PayloadTooLarge, got {:?}", other)),
    }
    if rx.state() != RxState::Idle || rx.buffered_len() != 0 {
        return Err("receiver must not allocate on rejected FF".into());
    }
    // Receiver still works afterwards.
    let sf = frame::single_frame(0x700, &[1, 2, 3]).map_err(err)?;
    match rx.on_frame(&sf, &clock).map_err(err)? {
        RxOutput::Complete(d) if d == vec![1, 2, 3] => Ok(()),
        other => Err(format!("receiver broken after rejected FF: {:?}", other)),
    }
}

fn check_payload_limits() -> Check {
    // 4096 bytes exceeds the 12-bit ISO-TP length field.
    match Sender::new(vec![0u8; 4096], SenderConfig::default()) {
        Err(IsoTpError::PayloadTooLarge { len: 4096, .. }) => {}
        other => return Err(format!("expected PayloadTooLarge, got {:?}", other.is_ok())),
    }
    // Empty payload is invalid too.
    match Sender::new(Vec::new(), SenderConfig::default()) {
        Err(IsoTpError::PayloadTooLarge { .. }) => {}
        other => return Err(format!("expected error for empty payload, ok={}", other.is_ok())),
    }
    // Exactly 4095 bytes must segment and reassemble.
    let mut clock = VirtualClock::new();
    let payload: Vec<u8> = (0..4095u32).map(|i| (i % 256) as u8).collect();
    let frames = segment_all(&payload, &mut clock).map_err(err)?;
    let mut rx = Receiver::new(ReceiverConfig::default());
    let mut out = None;
    for f in &frames {
        if let RxOutput::Complete(d) = rx.on_frame(f, &clock).map_err(err)? {
            out = Some(d);
        }
    }
    match out {
        Some(d) if d == payload => Ok(()),
        _ => Err("4095-byte roundtrip failed".into()),
    }
}

fn check_fc_timeout() -> Check {
    let mut clock = VirtualClock::new();
    let payload = vec![0x77u8; 50];
    let mut tx = Sender::new(payload, SenderConfig::default()).map_err(err)?;
    tx.start(&clock).map_err(err)?;
    clock.advance_ms(2_000); // no FC arrives
    match tx.on_tick(&clock) {
        Err(IsoTpError::Timeout(_)) => Ok(()),
        other => Err(format!("expected N_Bs Timeout, got {:?}", other)),
    }
}

fn main() {
    let tests: Vec<(&str, fn() -> Check)> = vec![
        ("single_frame_roundtrip", check_single_frame_roundtrip),
        ("sf_length_validation", check_sf_length_validation),
        ("multi_frame_roundtrip", check_multi_frame_roundtrip),
        ("sequence_number_wraparound", check_sequence_wraparound),
        ("dropped_frame_detected", check_dropped_frame_detected),
        ("duplicate_frame_detected", check_duplicate_frame_detected),
        ("length_declared_vs_actual", check_length_errors),
        ("flow_control_block_size", check_flow_control_block_size),
        ("stmin_virtual_clock", check_stmin_virtual_clock),
        ("flow_control_wait_overflow", check_flow_control_wait_and_overflow),
        ("oversized_first_frame_rejected", check_oversized_first_frame_rejected),
        ("payload_limits", check_payload_limits),
        ("fc_wait_timeout", check_fc_timeout),
    ];
    let mut failed = 0usize;
    for (name, test) in &tests {
        match test() {
            Ok(()) => println!("PASS {}", name),
            Err(e) => {
                println!("FAIL {}: {}", name, e);
                failed += 1;
            }
        }
    }
    println!("---");
    println!("{} passed, {} failed, {} total", tests.len() - failed, failed, tests.len());
    std::process::exit(if failed == 0 { 0 } else { 1 });
}
