use crate::types::{ClientId, EntityId, ShipInput};
use bevy_input::prelude::*;
use lightyear::prelude::Tick;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum InputSource {
    Keyboard,
    Gamepad(u8),
    Touch(u8),
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SequencedInput {
    pub sequence: u64,
    pub tick: u32,
    pub client: ClientId,
    pub entity: EntityId,
    pub source: InputSource,
    pub throttle: f64,
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
}

impl SequencedInput {
    pub fn new(
        sequence: u64,
        tick: u32,
        client: ClientId,
        entity: EntityId,
        source: InputSource,
        input: ShipInput,
    ) -> Self {
        Self {
            sequence,
            tick,
            client,
            entity,
            source,
            throttle: input.throttle,
            yaw: input.yaw,
            pitch: input.pitch,
            roll: input.roll,
        }
    }

    pub fn input(&self) -> ShipInput {
        ShipInput::new(self.throttle, self.yaw, self.pitch, self.roll)
    }

    pub fn tick_stamp(&self) -> Tick {
        Tick(self.tick)
    }
}

impl From<&SequencedInput> for crate::sim::InputMessage {
    fn from(msg: &SequencedInput) -> Self {
        Self {
            tick: msg.tick as u64,
            client: msg.client,
            target: msg.entity,
            input: msg.input(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InputViolation {
    NotOwner,
    InvalidRange,
    StaleSequence,
    DuplicateTick,
    RateLimited,
    HookRejected,
}

#[derive(Debug, Default)]
pub struct InputBuffer {
    entries: BTreeMap<u32, SequencedInput>,
    last_sequence: u64,
    last_input: ShipInput,
}

impl InputBuffer {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, input: &SequencedInput) -> Result<(), InputViolation> {
        if input.sequence <= self.last_sequence {
            return Err(InputViolation::StaleSequence);
        }
        if self.entries.contains_key(&input.tick) {
            return Err(InputViolation::DuplicateTick);
        }
        self.last_sequence = input.sequence;
        self.last_input = input.input();
        self.entries.insert(input.tick, input.clone());
        Ok(())
    }

    pub fn at(&self, tick: u32) -> ShipInput {
        self.entries
            .get(&tick)
            .map(SequencedInput::input)
            .unwrap_or(self.last_input)
    }

    pub fn replay(&self, from: u32, to: u32, mut visit: impl FnMut(u32, ShipInput)) {
        for tick in from..=to {
            visit(tick, self.at(tick));
        }
    }

    pub fn prune(&mut self, before: u32) -> usize {
        let stale: Vec<u32> = self
            .entries
            .keys()
            .copied()
            .filter(|tick| *tick < before)
            .collect();
        let mut removed = 0;
        for tick in stale {
            if self.entries.remove(&tick).is_some() {
                removed += 1;
            }
        }
        removed
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

#[derive(Debug, Default)]
pub struct AuthorityTable {
    owned: BTreeMap<ClientId, BTreeSet<EntityId>>,
    owner_of: BTreeMap<EntityId, ClientId>,
}

impl AuthorityTable {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn grant(&mut self, client: ClientId, entity: EntityId) {
        if let Some(old) = self.owner_of.insert(entity, client) {
            if let Some(set) = self.owned.get_mut(&old) {
                set.remove(&entity);
            }
        }
        self.owned.entry(client).or_default().insert(entity);
    }

    pub fn revoke(&mut self, entity: EntityId) -> bool {
        match self.owner_of.remove(&entity) {
            Some(owner) => {
                if let Some(set) = self.owned.get_mut(&owner) {
                    set.remove(&entity);
                }
                true
            }
            None => false,
        }
    }

    pub fn owner_of(&self, entity: EntityId) -> Option<ClientId> {
        self.owner_of.get(&entity).copied()
    }

    pub fn is_owner(&self, client: ClientId, entity: EntityId) -> bool {
        self.owner_of(entity) == Some(client)
    }
}

pub trait AntiCheatHook: Send + Sync {
    fn validate(
        &self,
        input: &SequencedInput,
        owner: Option<ClientId>,
    ) -> Result<(), InputViolation>;
}

#[derive(Default)]
pub struct NoopHook;

impl AntiCheatHook for NoopHook {
    fn validate(
        &self,
        _input: &SequencedInput,
        _owner: Option<ClientId>,
    ) -> Result<(), InputViolation> {
        Ok(())
    }
}

pub struct ServerInputValidator {
    pub authority: AuthorityTable,
    pub max_per_tick: usize,
    hooks: Vec<Box<dyn AntiCheatHook>>,
    buffers: BTreeMap<ClientId, InputBuffer>,
    counts: BTreeMap<(ClientId, u32), usize>,
}

impl ServerInputValidator {
    pub fn new(max_per_tick: usize) -> Self {
        Self {
            authority: AuthorityTable::new(),
            max_per_tick,
            hooks: Vec::new(),
            buffers: BTreeMap::new(),
            counts: BTreeMap::new(),
        }
    }

    pub fn add_hook(&mut self, hook: Box<dyn AntiCheatHook>) {
        self.hooks.push(hook);
    }

    pub fn ingest(&mut self, input: SequencedInput) -> Result<(), InputViolation> {
        if !self.authority.is_owner(input.client, input.entity) {
            return Err(InputViolation::NotOwner);
        }
        for value in [input.throttle, input.yaw, input.pitch, input.roll] {
            if !value.is_finite() || !(-1.0..=1.0).contains(&value) {
                return Err(InputViolation::InvalidRange);
            }
        }
        let owner = self.authority.owner_of(input.entity);
        for hook in &self.hooks {
            hook.validate(&input, owner)
                .map_err(|_| InputViolation::HookRejected)?;
        }
        let window = input.tick / 10;
        let count = self.counts.entry((input.client, window)).or_insert(0);
        if *count >= self.max_per_tick {
            return Err(InputViolation::RateLimited);
        }
        self.buffers.entry(input.client).or_default().push(&input)?;
        *count += 1;
        Ok(())
    }

    pub fn input_at(&self, client: ClientId, tick: u32) -> ShipInput {
        self.buffers
            .get(&client)
            .map(|buffer| buffer.at(tick))
            .unwrap_or(ShipInput::NONE)
    }

    pub fn replay(&self, client: ClientId, from: u32, to: u32, visit: impl FnMut(u32, ShipInput)) {
        if let Some(buffer) = self.buffers.get(&client) {
            buffer.replay(from, to, visit);
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct KeyboardBindings {
    pub forward: KeyCode,
    pub backward: KeyCode,
    pub yaw_left: KeyCode,
    pub yaw_right: KeyCode,
    pub pitch_up: KeyCode,
    pub pitch_down: KeyCode,
    pub roll_left: KeyCode,
    pub roll_right: KeyCode,
}

impl Default for KeyboardBindings {
    fn default() -> Self {
        Self {
            forward: KeyCode::KeyW,
            backward: KeyCode::KeyS,
            yaw_left: KeyCode::KeyA,
            yaw_right: KeyCode::KeyD,
            pitch_up: KeyCode::ArrowUp,
            pitch_down: KeyCode::ArrowDown,
            roll_left: KeyCode::KeyQ,
            roll_right: KeyCode::KeyE,
        }
    }
}

impl KeyboardBindings {
    pub fn sample(&self, keys: &ButtonInput<KeyCode>) -> ShipInput {
        let axis = |positive: KeyCode, negative: KeyCode| -> f64 {
            let positive = if keys.pressed(positive) { 1.0 } else { 0.0 };
            let negative = if keys.pressed(negative) { 1.0 } else { 0.0 };
            positive - negative
        };
        ShipInput::new(
            axis(self.forward, self.backward),
            axis(self.yaw_right, self.yaw_left),
            axis(self.pitch_up, self.pitch_down),
            axis(self.roll_right, self.roll_left),
        )
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GamepadAxes {
    pub throttle: f64,
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
}

impl GamepadAxes {
    pub fn sample(&self, deadzone: f64) -> ShipInput {
        let clean = |value: f64| {
            if value.abs() < deadzone {
                0.0
            } else {
                value.clamp(-1.0, 1.0)
            }
        };
        ShipInput::new(
            clean(self.throttle),
            clean(self.yaw),
            clean(self.pitch),
            clean(self.roll),
        )
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TouchStick {
    pub x: f64,
    pub y: f64,
}

impl TouchStick {
    pub fn axes(&self, deadzone: f64) -> (f64, f64) {
        let clean = |value: f64| {
            if value.abs() < deadzone {
                0.0
            } else {
                value.clamp(-1.0, 1.0)
            }
        };
        (clean(self.x), clean(self.y))
    }
}

#[derive(Debug, Default)]
pub struct InputRouter {
    devices: Vec<InputSource>,
    latest: BTreeMap<InputSource, ShipInput>,
}

impl InputRouter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_device(&mut self, source: InputSource) {
        if !self.devices.contains(&source) {
            self.devices.push(source);
        }
    }

    pub fn submit(&mut self, source: InputSource, input: ShipInput) {
        self.add_device(source);
        self.latest.insert(source, input);
    }

    pub fn merged(&self) -> ShipInput {
        for source in self.devices.iter().rev() {
            if let Some(input) = self.latest.get(source) {
                return *input;
            }
        }
        ShipInput::NONE
    }

    pub fn device_count(&self) -> usize {
        self.devices.len()
    }
}
