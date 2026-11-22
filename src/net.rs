use crate::sim::{Client, InputMessage, Server};
use crate::types::Tick;
use crate::world::Snapshot;
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug)]
pub struct NetConfig {
    pub uplink_latency: Tick,
    pub downlink_latency: Tick,
    pub jitter: Tick,
    pub input_loss: f64,
    pub snapshot_loss: f64,
    pub seed: u64,
}

impl Default for NetConfig {
    fn default() -> Self {
        Self {
            uplink_latency: 0,
            downlink_latency: 0,
            jitter: 0,
            input_loss: 0.0,
            snapshot_loss: 0.0,
            seed: 1,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NetStats {
    pub inputs_sent: usize,
    pub inputs_dropped: usize,
    pub snapshots_sent: usize,
    pub snapshots_dropped: usize,
}

struct Lcg(u64);

impl Lcg {
    fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

pub struct MockNetwork {
    cfg: NetConfig,
    rng: Lcg,
    tick: Tick,
    inbound_inputs: BTreeMap<Tick, Vec<InputMessage>>,
    inbound_snapshots: BTreeMap<Tick, Vec<Snapshot>>,
    pub stats: NetStats,
    pub sent_snapshots: BTreeMap<Tick, Snapshot>,
}

impl MockNetwork {
    pub fn new(cfg: NetConfig) -> Self {
        Self {
            cfg,
            rng: Lcg(cfg.seed),
            tick: 0,
            inbound_inputs: BTreeMap::new(),
            inbound_snapshots: BTreeMap::new(),
            stats: NetStats::default(),
            sent_snapshots: BTreeMap::new(),
        }
    }

    pub fn tick(&self) -> Tick {
        self.tick
    }

    pub fn advance(&mut self) {
        self.tick += 1;
    }

    fn delay(&mut self, base: Tick) -> Tick {
        if self.cfg.jitter == 0 {
            base
        } else {
            base + self.rng.next_u64() % (self.cfg.jitter + 1)
        }
    }

    pub fn send_input(&mut self, msg: InputMessage) {
        self.stats.inputs_sent += 1;
        if self.cfg.input_loss > 0.0 && self.rng.next_f64() < self.cfg.input_loss {
            self.stats.inputs_dropped += 1;
            return;
        }
        let due = self.tick + self.delay(self.cfg.uplink_latency);
        self.inbound_inputs.entry(due).or_default().push(msg);
    }

    pub fn send_snapshot(&mut self, snap: Snapshot) {
        self.stats.snapshots_sent += 1;
        self.sent_snapshots.insert(snap.tick, snap.clone());
        if self.cfg.snapshot_loss > 0.0 && self.rng.next_f64() < self.cfg.snapshot_loss {
            self.stats.snapshots_dropped += 1;
            return;
        }
        let due = self.tick + self.delay(self.cfg.downlink_latency);
        self.inbound_snapshots.entry(due).or_default().push(snap);
    }

    pub fn deliver_inputs(&mut self, server: &mut Server) -> usize {
        match self.inbound_inputs.remove(&self.tick) {
            Some(msgs) => {
                let count = msgs.len();
                for msg in msgs {
                    let _ = server.submit_input(msg);
                }
                count
            }
            None => 0,
        }
    }

    pub fn deliver_snapshots(&mut self, client: &mut Client) -> usize {
        match self.inbound_snapshots.remove(&self.tick) {
            Some(snaps) => {
                let count = snaps.len();
                for snap in snaps {
                    client.receive_snapshot(&snap);
                }
                count
            }
            None => 0,
        }
    }
}
