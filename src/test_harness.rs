use crate::net::{MockNetwork, NetConfig, NetStats};
use crate::sim::{Client, Server};
use crate::types::{ShipInput, Vec3, POS_EPS};
use crate::world::{snapshot, Entity};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde::Serialize;
use std::fmt::Debug;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TestReport {
    pub passed: Vec<String>,
    pub failed: Vec<String>,
}

impl TestReport {
    pub fn new() -> Self {
        Self {
            passed: Vec::new(),
            failed: Vec::new(),
        }
    }

    pub fn record(&mut self, name: impl Into<String>, result: Result<(), String>) {
        let name = name.into();
        match result {
            Ok(()) => self.passed.push(name),
            Err(reason) => self.failed.push(format!("{name}: {reason}")),
        }
    }

    pub fn all_passed(&self) -> bool {
        self.failed.is_empty()
    }

    pub fn summary(&self) -> String {
        format!("{} passed, {} failed", self.passed.len(), self.failed.len())
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("TestReport is serializable")
    }
}

impl Default for TestReport {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Default)]
pub struct TestRunner {
    pub report: TestReport,
}

impl TestRunner {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn run(&mut self, name: &str, test: impl FnOnce() -> Result<(), String>) {
        self.report.record(name, test());
    }

    pub fn finish(self) -> TestReport {
        self.report
    }
}

pub struct DeterminismChecker;

impl DeterminismChecker {
    pub fn bit_exact<T: PartialEq + Debug>(actual: T, expected: T) -> Result<(), String> {
        if actual == expected {
            Ok(())
        } else {
            Err(format!("expected {expected:?}, got {actual:?}"))
        }
    }

    pub fn within_epsilon(actual: f64, expected: f64, epsilon: f64) -> Result<(), String> {
        let difference = (actual - expected).abs();
        if difference <= epsilon {
            Ok(())
        } else {
            Err(format!(
                "|{actual} - {expected}| = {difference} > {epsilon}"
            ))
        }
    }

    pub fn position(actual: Vec3, expected: Vec3, epsilon: f64) -> Result<(), String> {
        Self::within_epsilon(actual.x, expected.x, epsilon)?;
        Self::within_epsilon(actual.y, expected.y, epsilon)?;
        Self::within_epsilon(actual.z, expected.z, epsilon)
    }
}

#[derive(Clone)]
pub struct SeededInputs {
    rng: StdRng,
}

impl SeededInputs {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed),
        }
    }

    pub fn sequence(&mut self, count: usize) -> Vec<ShipInput> {
        self.by_ref().take(count).collect()
    }
}

impl Iterator for SeededInputs {
    type Item = ShipInput;

    fn next(&mut self) -> Option<Self::Item> {
        Some(ShipInput::new(
            self.rng.random_range(-1.0..=1.0),
            self.rng.random_range(-1.0..=1.0),
            self.rng.random_range(-1.0..=1.0),
            self.rng.random_range(-1.0..=1.0),
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeamId {
    Replication,
    InputAuthority,
    PhysicsPrediction,
    Ifcs,
    Meshing,
    FloatingOrigin,
    Interest,
    Backend,
}

impl SeamId {
    pub const ALL: [SeamId; 8] = [
        Self::Replication,
        Self::InputAuthority,
        Self::PhysicsPrediction,
        Self::Ifcs,
        Self::Meshing,
        Self::FloatingOrigin,
        Self::Interest,
        Self::Backend,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Replication => "seam1-replication",
            Self::InputAuthority => "seam2-input-authority",
            Self::PhysicsPrediction => "seam3-physics-prediction",
            Self::Ifcs => "seam4-ifcs",
            Self::Meshing => "seam5-meshing",
            Self::FloatingOrigin => "seam6-floating-origin",
            Self::Interest => "seam7-interest",
            Self::Backend => "seam8-backend",
        }
    }
}

pub fn seam_templates() -> TestReport {
    let mut runner = TestRunner::new();
    for seam in SeamId::ALL {
        runner.run(seam.name(), || Ok(()));
    }
    runner.finish()
}

#[derive(Clone, Debug)]
pub struct HeadlessRun {
    pub ticks: u32,
    pub server_tick: u64,
    pub client_tick: u64,
    pub rollbacks: usize,
    pub stale_snapshots: usize,
    pub stats: NetStats,
    pub confirmed_matches_sent_snapshot: bool,
    pub predicted_pos: Vec3,
    pub authoritative_pos: Vec3,
}

pub fn run_headless(cfg: NetConfig, ticks: u32, seed: u64, fault_tick: Option<u32>) -> HeadlessRun {
    let ifcs = Default::default();
    let mut server = Server::new(ifcs);
    let entity = server.spawn(Entity::cube(1000.0, true, Some(1)));
    let initial = snapshot(&server.world, server.tick);

    let mut client = Client::new(1, ifcs);
    client.receive_snapshot(&initial);
    client.predict(entity);

    let mut inputs = SeededInputs::new(seed);
    let mut net = MockNetwork::new(cfg);

    for tick in 0..ticks {
        let input = inputs.next().expect("seeded inputs are infinite");
        client.step(input);
        net.send_input(client.send_input(entity, input));
        if fault_tick == Some(tick) {
            if let Some(entity_state) = client.world.get_mut(entity) {
                entity_state.body.pos.x += 5.0;
            }
        }
        net.deliver_inputs(&mut server);
        let authoritative = server.step();
        net.send_snapshot(authoritative);
        net.deliver_snapshots(&mut client);
        net.advance();
    }

    let confirmed_matches_sent_snapshot = client
        .last_snapshot_tick
        .and_then(|tick| net.sent_snapshots.get(&tick))
        .is_some_and(|sent| {
            client
                .confirmed
                .iter()
                .all(|(id, body)| sent.bodies.get(id) == Some(body))
        });

    HeadlessRun {
        ticks,
        server_tick: server.tick,
        client_tick: client.tick,
        rollbacks: client.rollbacks,
        stale_snapshots: client.stale_snapshots_ignored,
        stats: net.stats,
        confirmed_matches_sent_snapshot,
        predicted_pos: client
            .body(entity)
            .map(|body| body.pos)
            .unwrap_or(Vec3::ZERO),
        authoritative_pos: server
            .world
            .get(entity)
            .map(|entity| entity.body.pos)
            .unwrap_or(Vec3::ZERO),
    }
}

pub fn convergence_check(run: &HeadlessRun) -> Result<(), String> {
    DeterminismChecker::position(run.predicted_pos, run.authoritative_pos, POS_EPS)
}
