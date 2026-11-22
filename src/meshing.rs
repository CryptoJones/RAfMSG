use crate::types::{EntityId, ShipBody};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

pub type ServerId = u64;
pub type HandoffToken = u64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshError {
    UnknownEntity,
    UnknownServer,
    NotAuthority,
    BadState,
    AlreadyOwned,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Lease {
    pub entity: EntityId,
    pub owner: ServerId,
    pub generation: u64,
    pub body: WireBodyMesh,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WireBodyMesh {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub mass: f64,
}

impl From<ShipBody> for WireBodyMesh {
    fn from(body: ShipBody) -> Self {
        Self {
            x: body.pos.x,
            y: body.pos.y,
            z: body.pos.z,
            vx: body.vel.x,
            vy: body.vel.y,
            vz: body.vel.z,
            mass: body.mass,
        }
    }
}

impl From<WireBodyMesh> for ShipBody {
    fn from(wire: WireBodyMesh) -> Self {
        Self {
            pos: crate::types::Vec3::new(wire.x, wire.y, wire.z),
            vel: crate::types::Vec3::new(wire.vx, wire.vy, wire.vz),
            ..ShipBody::new_cube(wire.mass)
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum HandoffState {
    Preparing,
    AwaitingAck { token: HandoffToken },
    Committed { token: HandoffToken },
    Aborted { token: HandoffToken },
}

#[derive(Clone, Debug)]
struct PendingHandoff {
    entity: EntityId,
    from: ServerId,
    to: ServerId,
    state: HandoffState,
}

#[derive(Clone, Debug, Default)]
pub struct Coordinator {
    load: BTreeMap<ServerId, usize>,
}

impl Coordinator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register(&mut self, server: ServerId) {
        self.load.entry(server).or_insert(0);
    }

    pub fn unregister(&mut self, server: ServerId) -> bool {
        self.load.remove(&server).is_some()
    }

    pub fn assign(&mut self) -> Result<ServerId, MeshError> {
        let chosen = self
            .load
            .iter()
            .min_by_key(|(server, count)| (**count, **server))
            .map(|(server, _)| *server)
            .ok_or(MeshError::UnknownServer)?;
        *self.load.get_mut(&chosen).expect("assigned server exists") += 1;
        Ok(chosen)
    }

    pub fn release(&mut self, server: ServerId) {
        if let Some(count) = self.load.get_mut(&server) {
            *count = count.saturating_sub(1);
        }
    }

    pub fn load_of(&self, server: ServerId) -> usize {
        self.load.get(&server).copied().unwrap_or(0)
    }

    pub fn server_count(&self) -> usize {
        self.load.len()
    }
}

#[derive(Clone, Debug, Default)]
pub struct MeshCluster {
    pub coordinator: Coordinator,
    authority: BTreeMap<EntityId, (ServerId, u64)>,
    bodies: BTreeMap<EntityId, ShipBody>,
    shadows: BTreeMap<ServerId, BTreeMap<EntityId, ShipBody>>,
    handoffs: BTreeMap<HandoffToken, PendingHandoff>,
    next_token: HandoffToken,
    pub commits: usize,
    pub aborts: usize,
}

impl MeshCluster {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_server(&mut self, server: ServerId) {
        self.coordinator.register(server);
        self.shadows.entry(server).or_default();
    }

    pub fn remove_server(&mut self, server: ServerId) -> bool {
        let removed = self.coordinator.unregister(server);
        self.shadows.remove(&server);
        removed
    }

    pub fn spawn(
        &mut self,
        entity: EntityId,
        body: ShipBody,
        owner: ServerId,
    ) -> Result<(), MeshError> {
        if !self.coordinator.load.contains_key(&owner) {
            return Err(MeshError::UnknownServer);
        }
        if self.authority.contains_key(&entity) {
            return Err(MeshError::AlreadyOwned);
        }
        self.authority.insert(entity, (owner, 0));
        self.bodies.insert(entity, body);
        self.rebuild_shadows();
        Ok(())
    }

    pub fn authority_of(&self, entity: EntityId) -> Option<ServerId> {
        self.authority.get(&entity).map(|(server, _)| *server)
    }

    pub fn generation_of(&self, entity: EntityId) -> Option<u64> {
        self.authority
            .get(&entity)
            .map(|(_, generation)| *generation)
    }

    pub fn authoritative_body(&self, entity: EntityId) -> Option<ShipBody> {
        self.bodies.get(&entity).copied()
    }

    pub fn shadow_body(&self, server: ServerId, entity: EntityId) -> Option<ShipBody> {
        self.shadows.get(&server)?.get(&entity).copied()
    }

    pub fn is_shadow(&self, server: ServerId, entity: EntityId) -> bool {
        self.authority_of(entity) != Some(server) && self.shadow_body(server, entity).is_some()
    }

    fn rebuild_shadows(&mut self) {
        let authority: BTreeMap<EntityId, ServerId> = self
            .authority
            .iter()
            .map(|(entity, (server, _))| (*entity, *server))
            .collect();
        for (server, shadows) in &mut self.shadows {
            shadows.clear();
            for (&entity, &body) in &self.bodies {
                if authority.get(&entity) != Some(server) {
                    shadows.insert(entity, body);
                }
            }
        }
    }

    pub fn begin_handoff(
        &mut self,
        entity: EntityId,
        from: ServerId,
        to: ServerId,
    ) -> Result<(HandoffToken, Lease), MeshError> {
        if !self.authority.contains_key(&entity) {
            return Err(MeshError::UnknownEntity);
        }
        if self.authority_of(entity) != Some(from) {
            return Err(MeshError::NotAuthority);
        }
        if !self.coordinator.load.contains_key(&to) {
            return Err(MeshError::UnknownServer);
        }
        let token = self.next_token;
        self.next_token += 1;
        let (_, generation) = self.authority[&entity];
        let body = self.bodies[&entity];
        self.handoffs.insert(
            token,
            PendingHandoff {
                entity,
                from,
                to,
                state: HandoffState::Preparing,
            },
        );
        Ok((
            token,
            Lease {
                entity,
                owner: to,
                generation: generation + 1,
                body: body.into(),
            },
        ))
    }

    pub fn state_of(&self, token: HandoffToken) -> Option<HandoffState> {
        self.handoffs
            .get(&token)
            .map(|handoff| handoff.state.clone())
    }

    pub fn acknowledge(&mut self, token: HandoffToken) -> Result<(), MeshError> {
        match self.handoffs.get_mut(&token) {
            Some(handoff) if handoff.state == HandoffState::Preparing => {
                handoff.state = HandoffState::AwaitingAck { token };
                Ok(())
            }
            _ => Err(MeshError::BadState),
        }
    }

    pub fn commit(&mut self, token: HandoffToken) -> Result<(), MeshError> {
        let handoff = self
            .handoffs
            .get(&token)
            .cloned()
            .ok_or(MeshError::BadState)?;
        if !matches!(handoff.state, HandoffState::AwaitingAck { .. }) {
            return Err(MeshError::BadState);
        }
        let record = self
            .authority
            .get_mut(&handoff.entity)
            .ok_or(MeshError::UnknownEntity)?;
        record.0 = handoff.to;
        record.1 += 1;
        self.coordinator.release(handoff.from);
        if let Some(handoff) = self.handoffs.get_mut(&token) {
            handoff.state = HandoffState::Committed { token };
        }
        self.rebuild_shadows();
        self.commits += 1;
        Ok(())
    }

    pub fn recover(&mut self, token: HandoffToken, failed: ServerId) -> Result<(), MeshError> {
        let handoff = self
            .handoffs
            .get(&token)
            .cloned()
            .ok_or(MeshError::BadState)?;
        if handoff.from != failed
            || !matches!(
                handoff.state,
                HandoffState::Preparing | HandoffState::AwaitingAck { .. }
            )
        {
            return Err(MeshError::BadState);
        }
        let record = self
            .authority
            .get_mut(&handoff.entity)
            .ok_or(MeshError::UnknownEntity)?;
        record.0 = handoff.to;
        record.1 += 1;
        if let Some(handoff) = self.handoffs.get_mut(&token) {
            handoff.state = HandoffState::Aborted { token };
        }
        self.rebuild_shadows();
        self.aborts += 1;
        Ok(())
    }

    pub fn authority_count(&self) -> usize {
        self.authority.len()
    }

    pub fn owned_by(&self, server: ServerId) -> BTreeSet<EntityId> {
        self.authority
            .iter()
            .filter(|(_, (owner, _))| *owner == server)
            .map(|(entity, _)| *entity)
            .collect()
    }
}
