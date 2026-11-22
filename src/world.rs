use crate::types::{ClientId, EntityId, ShipBody, ShipInput, Tick};
use std::collections::BTreeMap;

#[derive(Clone, Debug)]
pub struct Entity {
    pub body: ShipBody,
    pub input: ShipInput,
    pub replicate: bool,
    pub authority: Option<ClientId>,
    pub server_only: Option<u64>,
}

impl Entity {
    pub fn cube(mass: f64, replicate: bool, authority: Option<ClientId>) -> Self {
        Self {
            body: ShipBody::new_cube(mass),
            input: ShipInput::NONE,
            replicate,
            authority,
            server_only: None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct World {
    next_id: EntityId,
    pub entities: BTreeMap<EntityId, Entity>,
}

impl World {
    pub fn new() -> Self {
        Self {
            next_id: 1,
            entities: BTreeMap::new(),
        }
    }

    pub fn spawn(&mut self, entity: Entity) -> EntityId {
        let id = self.next_id;
        self.next_id += 1;
        self.entities.insert(id, entity);
        id
    }

    pub fn get(&self, id: EntityId) -> Option<&Entity> {
        self.entities.get(&id)
    }

    pub fn get_mut(&mut self, id: EntityId) -> Option<&mut Entity> {
        self.entities.get_mut(&id)
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Snapshot {
    pub tick: Tick,
    pub bodies: BTreeMap<EntityId, ShipBody>,
}

pub fn snapshot(world: &World, tick: Tick) -> Snapshot {
    let mut bodies = BTreeMap::new();
    for (&id, entity) in &world.entities {
        if entity.replicate {
            bodies.insert(id, entity.body);
        }
    }
    Snapshot { tick, bodies }
}

pub fn apply_snapshot(world: &mut World, snap: &Snapshot) {
    for (&id, &body) in &snap.bodies {
        if let Some(entity) = world.entities.get_mut(&id) {
            entity.body = body;
        } else {
            world.entities.insert(
                id,
                Entity {
                    body,
                    input: ShipInput::NONE,
                    replicate: true,
                    authority: None,
                    server_only: None,
                },
            );
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuthorityError {
    UnknownEntity,
    NotOwner,
    StaleTick,
}

pub fn try_client_mutate(
    world: &mut World,
    client: ClientId,
    id: EntityId,
    body: ShipBody,
) -> Result<(), AuthorityError> {
    match world.entities.get_mut(&id) {
        None => Err(AuthorityError::UnknownEntity),
        Some(entity) if entity.authority != Some(client) => Err(AuthorityError::NotOwner),
        Some(entity) => {
            entity.body = body;
            Ok(())
        }
    }
}
