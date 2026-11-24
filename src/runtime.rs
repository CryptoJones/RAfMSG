use crate::backend::{BackendError, BackendService, IdentityRegistry, PersistedShip};
use crate::ecs::{new_world, spawn_ship, Body, EcsWorld, PilotInput, SpawnOpts};
use crate::floating_origin::{FloatingOrigin, FloatingOriginWorld, WorldPoint};
use crate::ifcs::{control_law, IfcsConfig};
use crate::input::{InputSource, SequencedInput, ServerInputValidator};
use crate::interest::{InterestConfig, InterestManager};
use crate::meshing::{MeshCluster, MeshError, ServerId};
use crate::network::{
    decode, encode, BandwidthProfiler, Channel, MemoryTransport, SnapshotMessage, Transport,
};
use crate::physics::advance;
use crate::physics3d::{
    apply_force, default_world, spawn_dynamic_sphere, BodyHandle, PhysicsWorld3d,
};
use crate::prediction::{PredictionClient, ReconcileConfig};
use crate::types::{ClientId, EntityId, ShipBody, ShipInput, Vec3, DT};
use bevy_ecs::entity::Entity as EcsEntity;
use std::collections::BTreeMap;

/// Unified runtime integrating all eight RAfMSG seams into a single tick loop.
pub struct IntegratedRuntime {
    pub tick: u32,
    pub server_id: ServerId,
    pub ifcs: IfcsConfig,
    pub backend: BackendService,
    pub identity: IdentityRegistry,
    pub mesh: MeshCluster,
    pub origin_world: FloatingOriginWorld,
    pub ecs: EcsWorld,
    pub physics3d: PhysicsWorld3d,
    pub interest: InterestManager,
    pub validator: ServerInputValidator,
    pub transport: MemoryTransport,
    pub profiler: BandwidthProfiler,
    pub clients: BTreeMap<ClientId, PredictionClient>,
    ecs_entities: BTreeMap<EntityId, EcsEntity>,
    rapier_handles: BTreeMap<EntityId, BodyHandle>,
    next_entity_id: EntityId,
    next_sequence: u64,
}

impl IntegratedRuntime {
    pub fn new(
        _secret: &[u8],
        ifcs: IfcsConfig,
        interest_cfg: InterestConfig,
        backend: BackendService,
    ) -> Self {
        let mut mesh = MeshCluster::new();
        let server_id = 1;
        mesh.add_server(server_id);
        Self {
            tick: 0,
            server_id,
            ifcs,
            backend,
            identity: IdentityRegistry::new(),
            mesh,
            origin_world: FloatingOriginWorld::new(FloatingOrigin::at_origin(), ifcs),
            ecs: new_world(),
            physics3d: default_world(),
            interest: InterestManager::new(interest_cfg),
            validator: ServerInputValidator::new(10),
            transport: MemoryTransport::new(),
            profiler: BandwidthProfiler::default(),
            clients: BTreeMap::new(),
            ecs_entities: BTreeMap::new(),
            rapier_handles: BTreeMap::new(),
            next_entity_id: 1,
            next_sequence: 1,
        }
    }

    pub fn authenticate_and_join(
        &mut self,
        token: &str,
        now: u64,
        session_capacity: usize,
        spawn_pos: WorldPoint,
    ) -> Result<(ClientId, EntityId), RuntimeError> {
        let user = self.backend.auth.verify_at(token, now)?;
        let _session = self.backend.sessions.matchmake(&user, session_capacity)?;
        let client_id = self.identity.get_or_assign(&user);

        let entity_id = self.next_entity_id;
        self.next_entity_id += 1;

        let mut body = ShipBody::new_cube(1000.0);
        body.pos = spawn_pos.into();

        let owner = self.mesh.coordinator.assign()?;
        self.mesh.spawn(entity_id, body, owner)?;

        self.origin_world.spawn(entity_id, spawn_pos, body);
        let ecs_entity = spawn_ship(
            &mut self.ecs,
            body,
            ShipInput::NONE,
            SpawnOpts {
                replicate: true,
                server: true,
                owner: Some(client_id),
                predicted: false,
                interpolated: false,
            },
        );
        self.ecs_entities.insert(entity_id, ecs_entity);

        let (handle, _) =
            spawn_dynamic_sphere(&mut self.physics3d, spawn_pos.into(), 1.0, body.mass, false);
        self.rapier_handles.insert(entity_id, handle);

        self.validator.authority.grant(client_id, entity_id);
        self.interest.register_client(client_id, spawn_pos.into());
        self.interest.set_position(entity_id, spawn_pos.into());

        let mut pred = PredictionClient::new(ReconcileConfig::default(), self.ifcs);
        pred.predict(
            entity_id,
            self.origin_world.local_body(entity_id).unwrap_or(body),
        );
        self.clients.insert(client_id, pred);

        Ok((client_id, entity_id))
    }

    pub fn submit_input(
        &mut self,
        client: ClientId,
        entity: EntityId,
        input: ShipInput,
    ) -> Result<(), crate::input::InputViolation> {
        let seq = self.next_sequence;
        self.next_sequence += 1;
        let msg = SequencedInput::new(seq, self.tick, client, entity, InputSource::Keyboard, input);
        self.validator.ingest(msg)?;
        if let Some(ecs_ent) = self.ecs_entities.get(&entity) {
            if let Some(mut pilot) = self.ecs.get_mut::<PilotInput>(*ecs_ent) {
                pilot.0 = input;
            }
        }
        Ok(())
    }

    pub fn step(&mut self, camera_world: WorldPoint) {
        self.origin_world.move_camera(camera_world);

        let entities: Vec<EntityId> = self.ecs_entities.keys().copied().collect();
        for entity_id in &entities {
            let input = self
                .ecs_entities
                .get(entity_id)
                .and_then(|e| self.ecs.get::<PilotInput>(*e))
                .map(|p| p.0)
                .unwrap_or(ShipInput::NONE);

            if let Some(ecs_ent) = self.ecs_entities.get(entity_id) {
                if let Some(mut body_comp) = self.ecs.get_mut::<Body>(*ecs_ent) {
                    let ft = control_law(&input, &body_comp.0, &self.ifcs);
                    advance(&mut body_comp.0, &input, &self.ifcs, DT);
                    if let Some(&handle) = self.rapier_handles.get(entity_id) {
                        apply_force(&mut self.physics3d, handle, ft.force);
                    }
                    self.interest.set_position(*entity_id, body_comp.0.pos);
                }
            }

            if let Some(client) = self.validator.authority.owner_of(*entity_id) {
                if let Some(pred) = self.clients.get_mut(&client) {
                    pred.step(input);
                }
            }
        }

        self.physics3d.step();
        self.interest.update();
        self.tick += 1;

        let full_snap = self.origin_world.replicated_snapshot(self.tick);
        let client_ids: Vec<ClientId> = self.clients.keys().copied().collect();
        for client_id in client_ids {
            let visible = self.interest.visible_for(client_id);
            let filtered_bodies: Vec<(EntityId, crate::network::WireBody)> = full_snap
                .bodies
                .iter()
                .filter(|(id, _)| {
                    self.mesh.authority_of(*id).is_some()
                        && (visible.is_empty() || visible.contains(id))
                })
                .cloned()
                .collect();
            let client_snap = SnapshotMessage {
                tick: self.tick,
                bodies: filtered_bodies,
            };
            if let Ok(bytes) = encode(&client_snap) {
                let raw_len = client_snap.bodies.len() * std::mem::size_of::<ShipBody>();
                let _ = self.profiler.record_send(&client_snap, raw_len);
                let _ = self.transport.send(Channel::Unreliable, client_id, &bytes);
            }
        }

        for (client_id, _channel, payload) in self.transport.recv() {
            if let Ok(msg) = decode::<SnapshotMessage>(&payload) {
                if let Some(pred) = self.clients.get_mut(&client_id) {
                    let world_snap = msg.to_world_snapshot();
                    pred.receive_snapshot(msg.tick, world_snap.bodies);
                }
            }
        }
    }

    pub fn persist_client_ship(
        &self,
        token: &str,
        now: u64,
        entity: EntityId,
    ) -> Result<(), BackendError> {
        let body = self
            .origin_world
            .world_body(entity)
            .unwrap_or_else(|| ShipBody::new_cube(1000.0));
        let ship = PersistedShip {
            pos_x: body.pos.x,
            pos_y: body.pos.y,
            pos_z: body.pos.z,
            vel_x: body.vel.x,
            vel_y: body.vel.y,
            vel_z: body.vel.z,
            hull: 100.0,
            inventory: BTreeMap::new(),
        };
        self.backend.persist_authenticated(token, now, &ship)
    }

    pub fn body_pos(&self, entity: EntityId) -> Option<Vec3> {
        self.origin_world.world_body(entity).map(|b| b.pos)
    }
}

#[derive(Debug, PartialEq)]
pub enum RuntimeError {
    Backend(BackendError),
    Mesh(MeshError),
}

impl From<BackendError> for RuntimeError {
    fn from(err: BackendError) -> Self {
        Self::Backend(err)
    }
}

impl From<MeshError> for RuntimeError {
    fn from(err: MeshError) -> Self {
        Self::Mesh(err)
    }
}
