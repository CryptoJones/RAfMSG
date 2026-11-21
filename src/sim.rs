use crate::ifcs::IfcsConfig;
use crate::physics::advance;
use crate::types::{ClientId, EntityId, ShipBody, ShipInput, Tick, DT, ORI_EPS, POS_EPS};
use crate::world::{apply_snapshot, snapshot, AuthorityError, Entity, Snapshot, World};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InputMessage {
    pub tick: Tick,
    pub client: ClientId,
    pub target: EntityId,
    pub input: ShipInput,
}

pub struct Server {
    pub tick: Tick,
    pub world: World,
    pub cfg: IfcsConfig,
    last_input: BTreeMap<EntityId, ShipInput>,
    pending: BTreeMap<Tick, Vec<InputMessage>>,
}

impl Server {
    pub fn new(cfg: IfcsConfig) -> Self {
        Self {
            tick: 0,
            world: World::new(),
            cfg,
            last_input: BTreeMap::new(),
            pending: BTreeMap::new(),
        }
    }

    pub fn spawn(&mut self, entity: Entity) -> EntityId {
        self.world.spawn(entity)
    }

    pub fn submit_input(&mut self, msg: InputMessage) -> Result<(), AuthorityError> {
        match self.world.get(msg.target) {
            None => return Err(AuthorityError::UnknownEntity),
            Some(entity) if entity.authority != Some(msg.client) => {
                return Err(AuthorityError::NotOwner);
            }
            Some(_) => {}
        }
        self.pending.entry(self.tick).or_default().push(msg);
        Ok(())
    }

    pub fn step(&mut self) -> Snapshot {
        let tick = self.tick;
        let pending = self.pending.get(&tick).cloned().unwrap_or_default();
        let last_input = self.last_input.clone();

        for (&id, entity) in self.world.entities.iter_mut() {
            if let Some(msg) = pending.iter().find(|msg| msg.target == id) {
                entity.input = msg.input;
            } else if let Some(&previous) = last_input.get(&id) {
                entity.input = previous;
            }
            advance(&mut entity.body, &entity.input, &self.cfg, DT);
        }

        for (&id, entity) in &self.world.entities {
            self.last_input.insert(id, entity.input);
        }
        self.pending.retain(|&t, _| t > tick);
        self.tick += 1;

        snapshot(&self.world, self.tick)
    }
}

pub struct Client {
    pub tick: Tick,
    pub client_id: ClientId,
    pub world: World,
    pub cfg: IfcsConfig,
    predicted: Vec<EntityId>,
    input_history: BTreeMap<Tick, ShipInput>,
    pub rollbacks: usize,
    pub last_snapshot_tick: Option<Tick>,
    pub confirmed: BTreeMap<EntityId, ShipBody>,
    pub stale_snapshots_ignored: usize,
}

impl Client {
    pub fn new(client_id: ClientId, cfg: IfcsConfig) -> Self {
        Self {
            tick: 0,
            client_id,
            world: World::new(),
            cfg,
            predicted: Vec::new(),
            input_history: BTreeMap::new(),
            rollbacks: 0,
            last_snapshot_tick: None,
            confirmed: BTreeMap::new(),
            stale_snapshots_ignored: 0,
        }
    }

    pub fn predict(&mut self, id: EntityId) {
        if !self.predicted.contains(&id) {
            self.predicted.push(id);
        }
    }

    pub fn send_input(&self, target: EntityId, input: ShipInput) -> InputMessage {
        InputMessage {
            tick: self.tick,
            client: self.client_id,
            target,
            input,
        }
    }

    pub fn step(&mut self, input: ShipInput) {
        let tick = self.tick;
        self.input_history.insert(tick, input);

        for &id in &self.predicted {
            if let Some(entity) = self.world.entities.get_mut(&id) {
                entity.input = input;
                advance(&mut entity.body, &input, &self.cfg, DT);
            }
        }
        self.tick += 1;
    }

    pub fn receive_snapshot(&mut self, snap: &Snapshot) {
        if self
            .last_snapshot_tick
            .is_some_and(|previous| snap.tick <= previous)
        {
            self.stale_snapshots_ignored += 1;
            return;
        }
        self.last_snapshot_tick = Some(snap.tick);
        for (&id, &body) in &snap.bodies {
            self.confirmed.insert(id, body);
        }

        let mut corrections = Vec::new();

        for &id in &self.predicted {
            let authoritative = match snap.bodies.get(&id) {
                Some(body) => *body,
                None => continue,
            };
            let predicted_now = match self.world.get(id) {
                Some(entity) => entity.body,
                None => continue,
            };

            let mut replay = authoritative;
            for t in (snap.tick + 1)..self.tick {
                let input = self
                    .input_history
                    .get(&t)
                    .copied()
                    .unwrap_or(ShipInput::NONE);
                advance(&mut replay, &input, &self.cfg, DT);
            }

            let diverged = replay.pos.distance(predicted_now.pos) > POS_EPS
                || replay.ori.angle_to(predicted_now.ori) > ORI_EPS;
            if diverged {
                corrections.push((id, replay));
            }
        }

        apply_snapshot(&mut self.world, snap);
        for (id, replay) in corrections {
            self.rollbacks += 1;
            if let Some(entity) = self.world.entities.get_mut(&id) {
                entity.body = replay;
            }
        }
    }

    pub fn body(&self, id: EntityId) -> Option<ShipBody> {
        self.world.get(id).map(|entity| entity.body)
    }
}
