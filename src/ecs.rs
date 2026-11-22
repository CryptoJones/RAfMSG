use crate::ifcs::IfcsConfig;
use crate::types::{ClientId, ShipBody, ShipInput, DT};
use bevy_ecs::component::ComponentId;
use bevy_ecs::prelude::*;
use std::collections::BTreeSet;

pub type EcsWorld = World;

#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct Body(pub ShipBody);

#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PilotInput(pub ShipInput);

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Server;

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct OwnedBy(pub ClientId);

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Replicate;

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Predicted;

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq)]
pub struct Interpolated;

#[derive(Resource, Default, Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickCounter(pub u64);

#[derive(Resource, Default, Clone, Copy, Debug, PartialEq, Eq)]
pub struct DespawnCount(pub usize);

#[derive(Resource, Clone, Copy, Debug, Default)]
pub struct FlightConfig(pub IfcsConfig);

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LifecycleSet {
    Tick,
    Spawn,
    Advance,
    Cleanup,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SpawnOpts {
    pub replicate: bool,
    pub server: bool,
    pub owner: Option<ClientId>,
    pub predicted: bool,
    pub interpolated: bool,
}

#[derive(Debug, Default)]
pub struct ComponentRegistry {
    names: BTreeSet<&'static str>,
}

impl ComponentRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn register<C: Component>(
        &mut self,
        world: &mut EcsWorld,
        name: &'static str,
    ) -> ComponentId {
        let id = world.register_component::<C>();
        self.names.insert(name);
        id
    }

    pub fn is_registered(&self, name: &str) -> bool {
        self.names.contains(name)
    }

    pub fn len(&self) -> usize {
        self.names.len()
    }

    pub fn is_empty(&self) -> bool {
        self.names.is_empty()
    }
}

pub fn new_world() -> EcsWorld {
    let mut world = EcsWorld::new();
    world.insert_resource(TickCounter::default());
    world.insert_resource(DespawnCount::default());
    world.insert_resource(FlightConfig::default());
    world
}

pub fn spawn_ship(
    world: &mut EcsWorld,
    body: ShipBody,
    input: ShipInput,
    opts: SpawnOpts,
) -> Entity {
    let mut entity = world.spawn((Body(body), PilotInput(input)));
    if opts.replicate {
        entity.insert(Replicate);
    }
    if opts.server {
        entity.insert(Server);
    }
    if let Some(owner) = opts.owner {
        entity.insert(OwnedBy(owner));
    }
    if opts.predicted {
        entity.insert(Predicted);
    }
    if opts.interpolated {
        entity.insert(Interpolated);
    }
    entity.id()
}

pub fn despawn_ship(world: &mut EcsWorld, entity: Entity) -> bool {
    if world.get_entity(entity).is_ok() {
        world.despawn(entity);
        world.resource_mut::<DespawnCount>().0 += 1;
        true
    } else {
        false
    }
}

pub fn body_of(world: &EcsWorld, entity: Entity) -> Option<ShipBody> {
    world.get::<Body>(entity).map(|body| body.0)
}

pub fn replicated_bodies(world: &mut EcsWorld) -> Vec<(Entity, ShipBody)> {
    let mut query = world.query_filtered::<(Entity, &Body), With<Replicate>>();
    query
        .iter(world)
        .map(|(entity, body)| (entity, body.0))
        .collect()
}

pub fn owned_by(world: &mut EcsWorld, client: ClientId) -> Vec<Entity> {
    let mut query = world.query_filtered::<(Entity, &OwnedBy), With<PilotInput>>();
    query
        .iter(world)
        .filter(|(_, owner)| owner.0 == client)
        .map(|(entity, _)| entity)
        .collect()
}

pub fn tick_system(mut tick: ResMut<TickCounter>) {
    tick.0 += 1;
}

pub fn spawn_system(mut commands: Commands, tick: Res<TickCounter>) {
    if tick.0 == 1 {
        commands.spawn((
            Body(ShipBody::new_cube(1.0)),
            PilotInput(ShipInput::NONE),
            Server,
            Replicate,
        ));
    }
}

pub fn advance_system(
    tick: Res<TickCounter>,
    cfg: Res<FlightConfig>,
    mut bodies: Query<&mut Body, With<Server>>,
) {
    if tick.0 == 0 {
        return;
    }
    let input = ShipInput::new(1.0, 0.0, 0.0, 0.0);
    for mut body in &mut bodies {
        crate::physics::advance(&mut body.0, &input, &cfg.0, DT);
    }
}

pub fn cleanup_system(
    mut commands: Commands,
    tick: Res<TickCounter>,
    mut count: ResMut<DespawnCount>,
    entities: Query<Entity, With<Server>>,
) {
    if tick.0 != 4 {
        return;
    }
    for entity in &entities {
        commands.entity(entity).despawn();
        count.0 += 1;
    }
}

pub fn lifecycle_schedule() -> Schedule {
    let mut schedule = Schedule::default();
    schedule.configure_sets(
        (
            LifecycleSet::Tick,
            LifecycleSet::Spawn,
            LifecycleSet::Advance,
            LifecycleSet::Cleanup,
        )
            .chain(),
    );
    schedule.add_systems(tick_system.in_set(LifecycleSet::Tick));
    schedule.add_systems(spawn_system.in_set(LifecycleSet::Spawn));
    schedule.add_systems(advance_system.in_set(LifecycleSet::Advance));
    schedule.add_systems(cleanup_system.in_set(LifecycleSet::Cleanup));
    schedule
}
