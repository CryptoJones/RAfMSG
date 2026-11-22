use crate::ifcs::IfcsConfig;
use crate::physics::advance;
use crate::types::{EntityId, Quat, ShipBody, ShipInput, Vec3, DT, ORI_EPS, POS_EPS};
use lightyear::prelude::{Interpolated, Predicted, Tick};
use lightyear_core::history_buffer::HistoryBuffer;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReconcileConfig {
    pub pos_epsilon: f64,
    pub ori_epsilon: f64,
    pub max_rollback_ticks: u32,
    pub interpolation_delay_ticks: u32,
    pub history_ticks: u32,
}

impl Default for ReconcileConfig {
    fn default() -> Self {
        Self {
            pos_epsilon: POS_EPS,
            ori_epsilon: ORI_EPS,
            max_rollback_ticks: 10,
            interpolation_delay_ticks: 6,
            history_ticks: 32,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Interpolator {
    buffer: BTreeMap<u32, ShipBody>,
    pub delay_ticks: u32,
}

impl Interpolator {
    pub fn new(delay_ticks: u32) -> Self {
        Self {
            buffer: BTreeMap::new(),
            delay_ticks,
        }
    }

    pub fn push(&mut self, tick: u32, body: ShipBody) {
        self.buffer.insert(tick, body);
        let cutoff = tick.saturating_sub(self.delay_ticks + 8);
        let stale: Vec<u32> = self
            .buffer
            .keys()
            .copied()
            .filter(|t| *t < cutoff)
            .collect();
        for tick in stale {
            self.buffer.remove(&tick);
        }
    }

    pub fn sample(&self, current_tick: u32) -> Option<ShipBody> {
        let target = current_tick.saturating_sub(self.delay_ticks);
        let before = self.buffer.range(..=target).next_back();
        let after = self.buffer.range(target..).next();
        match (before, after) {
            (Some((at, a)), Some((bt, b))) => {
                let alpha = if *at == *bt {
                    0.0
                } else {
                    (target - *at) as f64 / (*bt - *at) as f64
                };
                Some(lerp_body(*a, *b, alpha))
            }
            (Some((_, body)), None) | (None, Some((_, body))) => Some(*body),
            (None, None) => None,
        }
    }

    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }
}

fn lerp(a: Vec3, b: Vec3, alpha: f64) -> Vec3 {
    Vec3::new(
        a.x + (b.x - a.x) * alpha,
        a.y + (b.y - a.y) * alpha,
        a.z + (b.z - a.z) * alpha,
    )
}

fn lerp_body(a: ShipBody, b: ShipBody, alpha: f64) -> ShipBody {
    ShipBody {
        pos: lerp(a.pos, b.pos, alpha),
        vel: lerp(a.vel, b.vel, alpha),
        ori: Quat::new(
            a.ori.x + (b.ori.x - a.ori.x) * alpha,
            a.ori.y + (b.ori.y - a.ori.y) * alpha,
            a.ori.z + (b.ori.z - a.ori.z) * alpha,
            a.ori.w + (b.ori.w - a.ori.w) * alpha,
        )
        .normalize(),
        ang_vel: lerp(a.ang_vel, b.ang_vel, alpha),
        mass: a.mass,
    }
}

pub struct PredictionClient {
    pub tick: u32,
    pub config: ReconcileConfig,
    pub ifcs: IfcsConfig,
    predicted: BTreeSet<EntityId>,
    input_history: BTreeMap<u32, ShipInput>,
    body_history: BTreeMap<EntityId, HistoryBuffer<ShipBody>>,
    pub predicted_bodies: BTreeMap<EntityId, ShipBody>,
    pub confirmed_bodies: BTreeMap<EntityId, ShipBody>,
    interpolators: BTreeMap<EntityId, Interpolator>,
    pub last_snapshot_tick: Option<u32>,
    pub rollbacks: usize,
    pub stale_snapshots_ignored: usize,
}

impl PredictionClient {
    pub fn new(config: ReconcileConfig, ifcs: IfcsConfig) -> Self {
        Self {
            tick: 0,
            config,
            ifcs,
            predicted: BTreeSet::new(),
            input_history: BTreeMap::new(),
            body_history: BTreeMap::new(),
            predicted_bodies: BTreeMap::new(),
            confirmed_bodies: BTreeMap::new(),
            interpolators: BTreeMap::new(),
            last_snapshot_tick: None,
            rollbacks: 0,
            stale_snapshots_ignored: 0,
        }
    }

    pub fn predict(&mut self, entity: EntityId, body: ShipBody) {
        self.predicted.insert(entity);
        self.predicted_bodies.insert(entity, body);
        self.body_history.insert(entity, HistoryBuffer::default());
    }

    pub fn observe(&mut self, entity: EntityId) {
        self.interpolators
            .entry(entity)
            .or_insert_with(|| Interpolator::new(self.config.interpolation_delay_ticks));
    }

    pub fn is_predicted(&self, entity: EntityId) -> bool {
        self.predicted.contains(&entity)
    }

    pub fn is_interpolated(&self, entity: EntityId) -> bool {
        self.interpolators.contains_key(&entity)
    }

    pub fn step(&mut self, input: ShipInput) {
        let tick = self.tick;
        self.input_history.insert(tick, input);
        let cutoff = tick.saturating_sub(self.config.history_ticks);
        self.input_history.retain(|t, _| *t >= cutoff);

        for (&entity, body) in &mut self.predicted_bodies {
            advance(body, &input, &self.ifcs, DT);
            if let Some(history) = self.body_history.get_mut(&entity) {
                history.add(Tick(tick), Some(*body));
            }
        }
        self.prune_history(tick);
        self.tick += 1;
    }

    fn prune_history(&mut self, tick: u32) {
        let keep_from = tick.saturating_sub(self.config.history_ticks);
        for history in self.body_history.values_mut() {
            while history
                .oldest()
                .is_some_and(|(oldest, _)| oldest.0 < keep_from)
            {
                let _ = history.pop();
            }
        }
    }

    pub fn receive_snapshot(&mut self, tick: u32, bodies: BTreeMap<EntityId, ShipBody>) {
        if self.last_snapshot_tick.is_some_and(|last| tick <= last) {
            self.stale_snapshots_ignored += 1;
            return;
        }
        self.last_snapshot_tick = Some(tick);
        for (&entity, &body) in &bodies {
            self.confirmed_bodies.insert(entity, body);
            if let Some(interpolator) = self.interpolators.get_mut(&entity) {
                interpolator.push(tick, body);
            }
        }

        for &entity in &self.predicted {
            let authoritative = match bodies.get(&entity).copied() {
                Some(body) => body,
                None => continue,
            };
            let predicted_now = match self.predicted_bodies.get(&entity).copied() {
                Some(body) => body,
                None => continue,
            };
            let lag = self.tick.saturating_sub(tick);
            let mut replay = authoritative;
            if lag <= self.config.max_rollback_ticks {
                for t in (tick + 1)..self.tick {
                    let input = self
                        .input_history
                        .get(&t)
                        .copied()
                        .unwrap_or(ShipInput::NONE);
                    advance(&mut replay, &input, &self.ifcs, DT);
                }
            }
            let diverged = replay.pos.distance(predicted_now.pos) > self.config.pos_epsilon
                || replay.ori.angle_to(predicted_now.ori) > self.config.ori_epsilon;
            if diverged {
                self.rollbacks += 1;
                self.predicted_bodies.insert(entity, replay);
            }
        }
    }

    pub fn predicted_body(&self, entity: EntityId) -> Option<ShipBody> {
        self.predicted_bodies.get(&entity).copied()
    }

    pub fn interpolated_body(&self, entity: EntityId) -> Option<ShipBody> {
        self.interpolators.get(&entity)?.sample(self.tick)
    }

    pub fn history_len(&self, entity: EntityId) -> usize {
        self.body_history.get(&entity).map_or(0, HistoryBuffer::len)
    }

    pub fn tick_stamp(&self) -> Tick {
        Tick(self.tick)
    }
}

pub fn predicted_component() -> Predicted {
    Predicted
}

pub fn interpolated_component() -> Interpolated {
    Interpolated
}
