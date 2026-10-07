/// Manually advanced clock used to simulate ISO-TP timing (STmin, N_Bs, N_Cr)
/// without ever calling `sleep` or touching the real system clock.
#[derive(Debug, Clone, Default)]
pub struct VirtualClock {
    now_us: u64,
}

impl VirtualClock {
    pub fn new() -> Self {
        Self { now_us: 0 }
    }

    pub fn now_us(&self) -> u64 {
        self.now_us
    }

    pub fn advance_us(&mut self, delta_us: u64) {
        self.now_us = self.now_us.saturating_add(delta_us);
    }

    pub fn advance_ms(&mut self, delta_ms: u64) {
        self.advance_us(delta_ms.saturating_mul(1_000));
    }
}
