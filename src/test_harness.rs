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
    pub skipped: Vec<String>,
}

impl TestReport {
    pub fn new() -> Self {
        Self {
            passed: Vec::new(),
            failed: Vec::new(),
            skipped: Vec::new(),
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
        let mut summary = format!("{} passed, {} failed", self.passed.len(), self.failed.len());
        if !self.skipped.is_empty() {
            summary.push_str(&format!(", {} skipped", self.skipped.len()));
        }
        summary
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

    pub fn skip(&mut self, name: &str) {
        self.report.skipped.push(name.to_string());
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
        runner.skip(seam.name());
    }
    runner.finish()
}

pub fn run_seam_suite() -> TestReport {
    let mut runner = TestRunner::new();

    runner.run(SeamId::Replication.name(), || {
        let mut world = crate::world::World::new();
        let id = world.spawn(crate::world::Entity::cube(100.0, true, Some(1)));
        let _hidden = world.spawn(crate::world::Entity::cube(100.0, false, None));
        let snap = snapshot(&world, 1);
        if snap.bodies.len() != 1 || !snap.bodies.contains_key(&id) {
            return Err("opt-in replication failed".to_string());
        }
        Ok(())
    });

    runner.run(SeamId::InputAuthority.name(), || {
        let mut server = Server::new(Default::default());
        let id = server.spawn(Entity::cube(1000.0, true, Some(1)));
        let bad = crate::sim::InputMessage {
            tick: 0,
            client: 2,
            target: id,
            input: ShipInput::NONE,
        };
        if server.submit_input(bad) != Err(crate::world::AuthorityError::NotOwner) {
            return Err("non-owner input was not rejected".to_string());
        }
        Ok(())
    });

    runner.run(SeamId::PhysicsPrediction.name(), || {
        let run = run_headless(NetConfig::default(), 60, 42, Some(30));
        convergence_check(&run)
    });

    runner.run(SeamId::Ifcs.name(), || {
        let cfg = crate::ifcs::IfcsConfig::default();
        let body = crate::types::ShipBody::new_cube(1000.0);
        let ft = crate::ifcs::control_law(&ShipInput::new(10.0, 0.0, 0.0, 0.0), &body, &cfg);
        if ft.force.length() > body.mass * cfg.g_limit_ms2 + 1e-6 {
            return Err("g-limit exceeded".to_string());
        }
        Ok(())
    });

    runner.run(SeamId::Meshing.name(), || {
        let mut mesh = crate::meshing::MeshCluster::new();
        mesh.add_server(1);
        mesh.add_server(2);
        mesh.spawn(1, crate::types::ShipBody::new_cube(10.0), 1)
            .map_err(|e| format!("{e:?}"))?;
        let (token, lease) = mesh.begin_handoff(1, 1, 2).map_err(|e| format!("{e:?}"))?;
        let signed = lease.sign(b"mesh-secret");
        if !signed.verify(b"mesh-secret") {
            return Err("signed lease failed verification".to_string());
        }
        mesh.acknowledge(token).map_err(|e| format!("{e:?}"))?;
        mesh.commit(token).map_err(|e| format!("{e:?}"))?;
        if mesh.authority_of(1) != Some(2) {
            return Err("authority did not transfer on commit".to_string());
        }
        Ok(())
    });

    runner.run(SeamId::FloatingOrigin.name(), || {
        let mut world = crate::floating_origin::FloatingOriginWorld::new(
            crate::floating_origin::FloatingOrigin::at_origin(),
            Default::default(),
        );
        let p = crate::floating_origin::WorldPoint::new(5000.0, 0.0, 0.0);
        world.spawn(1, p, crate::types::ShipBody::new_cube(10.0));
        if !world.move_camera(p) {
            return Err("rebase did not trigger".to_string());
        }
        let local = world.local_body(1).ok_or("missing local body")?;
        DeterminismChecker::position(local.pos, Vec3::ZERO, 1e-6)
    });

    runner.run(SeamId::Interest.name(), || {
        let mut mgr = crate::interest::InterestManager::new(Default::default());
        mgr.register_client(1, Vec3::ZERO);
        mgr.set_position(10, Vec3::new(500.0, 0.0, 0.0));
        for _ in 0..10 {
            mgr.update();
        }
        if !mgr.can_target(1, 10) {
            return Err("nearby entity not visible".to_string());
        }
        Ok(())
    });

    runner.run(SeamId::Backend.name(), || {
        let persistence =
            crate::backend::Persistence::open_in_memory().map_err(|e| format!("{e:?}"))?;
        let mut svc = crate::backend::BackendService::new(b"harness-secret", persistence);
        let token = svc.auth.issue_token_with_expiry("alice", 1000);
        let user = svc
            .auth
            .verify_at(&token, 500)
            .map_err(|e| format!("{e:?}"))?;
        if user != "alice" {
            return Err("unexpected user".to_string());
        }
        let _ = svc.join(&token, 4).map_err(|e| format!("{e:?}"))?;
        Ok(())
    });

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
