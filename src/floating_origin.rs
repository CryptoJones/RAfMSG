use crate::ifcs::IfcsConfig;
use crate::physics::advance;
use crate::types::{EntityId, ShipBody, ShipInput, Vec3};
use nalgebra::Vector3;
use std::collections::BTreeMap;

pub const DEFAULT_REBASE_THRESHOLD: f64 = 1000.0;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WorldPoint(pub Vector3<f64>);

impl WorldPoint {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vector3::new(x, y, z))
    }
}

impl From<Vec3> for WorldPoint {
    fn from(v: Vec3) -> Self {
        Self::new(v.x, v.y, v.z)
    }
}

impl From<WorldPoint> for Vec3 {
    fn from(point: WorldPoint) -> Self {
        Vec3::new(point.0.x, point.0.y, point.0.z)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct FloatingOrigin {
    pub anchor: WorldPoint,
    pub threshold: f64,
    pub rebase_count: u64,
}

impl FloatingOrigin {
    pub fn new(anchor: WorldPoint, threshold: f64) -> Self {
        Self {
            anchor,
            threshold,
            rebase_count: 0,
        }
    }

    pub fn at_origin() -> Self {
        Self::new(WorldPoint::new(0.0, 0.0, 0.0), DEFAULT_REBASE_THRESHOLD)
    }

    pub fn to_local(&self, world: WorldPoint) -> WorldPoint {
        WorldPoint(world.0 - self.anchor.0)
    }

    pub fn to_world(&self, local: WorldPoint) -> WorldPoint {
        WorldPoint(local.0 + self.anchor.0)
    }

    pub fn maybe_rebase(&mut self, camera: WorldPoint) -> bool {
        let distance = (camera.0 - self.anchor.0).norm();
        if distance > self.threshold {
            self.anchor = camera;
            self.rebase_count += 1;
            true
        } else {
            false
        }
    }

    pub fn rebase_delta(&self, previous: WorldPoint) -> WorldPoint {
        WorldPoint(self.anchor.0 - previous.0)
    }
}

#[derive(Clone, Debug)]
pub struct LocalBody {
    pub body: ShipBody,
    pub world: WorldPoint,
}

#[derive(Clone, Debug)]
pub struct FloatingOriginWorld {
    pub origin: FloatingOrigin,
    pub cfg: IfcsConfig,
    bodies: BTreeMap<EntityId, LocalBody>,
    pub last_rebase_delta: Option<WorldPoint>,
}

impl FloatingOriginWorld {
    pub fn new(origin: FloatingOrigin, cfg: IfcsConfig) -> Self {
        Self {
            origin,
            cfg,
            bodies: BTreeMap::new(),
            last_rebase_delta: None,
        }
    }

    pub fn spawn(&mut self, entity: EntityId, world: WorldPoint, body: ShipBody) {
        let local = self.origin.to_local(world);
        let mut local_body = body;
        local_body.pos = local.into();
        self.bodies.insert(
            entity,
            LocalBody {
                body: local_body,
                world,
            },
        );
    }

    pub fn local_body(&self, entity: EntityId) -> Option<ShipBody> {
        self.bodies.get(&entity).map(|entry| entry.body)
    }

    pub fn world_body(&self, entity: EntityId) -> Option<ShipBody> {
        let entry = self.bodies.get(&entity)?;
        let mut body = entry.body;
        body.pos = self
            .origin
            .to_world(WorldPoint::from(entry.body.pos))
            .into();
        Some(body)
    }

    pub fn move_camera(&mut self, camera: WorldPoint) -> bool {
        let previous = self.origin.anchor;
        let rebased = self.origin.maybe_rebase(camera);
        if rebased {
            let delta = self.origin.rebase_delta(previous);
            for entry in self.bodies.values_mut() {
                let current = WorldPoint::from(entry.body.pos);
                entry.body.pos = WorldPoint(current.0 - delta.0).into();
            }
            self.last_rebase_delta = Some(delta);
        }
        rebased
    }

    pub fn step(&mut self, input: ShipInput) {
        for entry in self.bodies.values_mut() {
            advance(&mut entry.body, &input, &self.cfg, crate::types::DT);
            entry.world = self.origin.to_world(WorldPoint::from(entry.body.pos));
        }
    }

    pub fn replicated_position(&self, entity: EntityId) -> Option<Vec3> {
        self.bodies.get(&entity).map(|entry| entry.body.pos)
    }
}

pub fn f32_absolute_error(world: WorldPoint) -> f32 {
    let quantized = Vector3::new(
        world.0.x as f32 as f64,
        world.0.y as f32 as f64,
        world.0.z as f32 as f64,
    );
    (quantized - world.0).norm() as f32
}

pub fn f32_local_error(origin: &FloatingOrigin, world: WorldPoint) -> f32 {
    let local = origin.to_local(world);
    let quantized = Vector3::new(
        local.0.x as f32 as f64,
        local.0.y as f32 as f64,
        local.0.z as f32 as f64,
    );
    (quantized - local.0).norm() as f32
}
