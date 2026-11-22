use crate::types::{ClientId, EntityId, Vec3};
use rstar::{Point, RTree};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InterestConfig {
    pub sensor_range: f64,
    pub exit_range: f64,
    pub update_interval_ticks: u32,
    pub hysteresis_updates: u32,
}

impl Default for InterestConfig {
    fn default() -> Self {
        Self {
            sensor_range: 10_000.0,
            exit_range: 12_000.0,
            update_interval_ticks: 10,
            hysteresis_updates: 2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Indexed(pub EntityId, pub [f64; 3]);

impl Point for Indexed {
    type Scalar = f64;
    const DIMENSIONS: usize = 3;

    fn generate(mut generator: impl FnMut(usize) -> Self::Scalar) -> Self {
        Self(0, [generator(0), generator(1), generator(2)])
    }

    fn nth(&self, index: usize) -> Self::Scalar {
        self.1[index]
    }

    fn nth_mut(&mut self, index: usize) -> &mut Self::Scalar {
        &mut self.1[index]
    }
}

#[derive(Clone, Debug)]
pub struct InterestManager {
    pub config: InterestConfig,
    pub tick: u32,
    positions: BTreeMap<EntityId, Vec3>,
    index: RTree<Indexed>,
    observers: BTreeMap<ClientId, Vec3>,
    views: BTreeMap<ClientId, BTreeSet<EntityId>>,
    misses: BTreeMap<(ClientId, EntityId), u32>,
    pub updates: usize,
}

impl InterestManager {
    pub fn new(config: InterestConfig) -> Self {
        Self {
            config,
            tick: 0,
            positions: BTreeMap::new(),
            index: RTree::new(),
            observers: BTreeMap::new(),
            views: BTreeMap::new(),
            misses: BTreeMap::new(),
            updates: 0,
        }
    }

    pub fn register_client(&mut self, client: ClientId, position: Vec3) {
        self.observers.insert(client, position);
        self.views.entry(client).or_default();
    }

    pub fn set_observer_position(&mut self, client: ClientId, position: Vec3) {
        self.observers.insert(client, position);
    }

    pub fn set_position(&mut self, entity: EntityId, position: Vec3) {
        self.positions.insert(entity, position);
        self.rebuild_index();
    }

    pub fn entity_exists(&self, entity: EntityId) -> bool {
        self.positions.contains_key(&entity)
    }

    fn rebuild_index(&mut self) {
        self.index = RTree::bulk_load(
            self.positions
                .iter()
                .map(|(entity, pos)| Indexed(*entity, [pos.x, pos.y, pos.z]))
                .collect(),
        );
    }

    pub fn nearby(&self, center: Vec3, radius: f64) -> Vec<EntityId> {
        self.index
            .locate_within_distance(Indexed(0, [center.x, center.y, center.z]), radius * radius)
            .map(|entry| entry.0)
            .collect()
    }

    pub fn update(&mut self) -> bool {
        self.tick += 1;
        if !self.tick.is_multiple_of(self.config.update_interval_ticks) {
            return false;
        }
        self.rebuild_index();
        let clients: Vec<ClientId> = self.observers.keys().copied().collect();
        for client in clients {
            let observer = self.observers[&client];
            let candidates: Vec<EntityId> = self.nearby(observer, self.config.exit_range);
            let mut view = self.views.entry(client).or_default().clone();

            let mut dropped = Vec::new();
            for &entity in &view {
                let distance = self
                    .positions
                    .get(&entity)
                    .map(|pos| pos.distance(observer))
                    .unwrap_or(f64::INFINITY);
                if distance <= self.config.sensor_range {
                    self.misses.remove(&(client, entity));
                } else if distance > self.config.exit_range {
                    let misses = self.misses.entry((client, entity)).or_insert(0);
                    *misses += 1;
                    if *misses >= self.config.hysteresis_updates {
                        dropped.push(entity);
                    }
                }
            }
            for entity in dropped {
                view.remove(&entity);
                self.misses.remove(&(client, entity));
            }
            for entity in candidates {
                let distance = self.positions[&entity].distance(observer);
                if distance <= self.config.sensor_range {
                    view.insert(entity);
                    self.misses.remove(&(client, entity));
                }
            }
            self.views.insert(client, view);
        }
        self.updates += 1;
        true
    }

    pub fn visible_for(&self, client: ClientId) -> BTreeSet<EntityId> {
        self.views.get(&client).cloned().unwrap_or_default()
    }

    pub fn can_target(&self, client: ClientId, entity: EntityId) -> bool {
        self.visible_for(client).contains(&entity)
    }

    pub fn relevant_for(&self, client: ClientId, candidates: &[EntityId]) -> Vec<EntityId> {
        let view = self.visible_for(client);
        candidates
            .iter()
            .copied()
            .filter(|entity| view.contains(entity))
            .collect()
    }

    pub fn client_count(&self) -> usize {
        self.observers.len()
    }
}
