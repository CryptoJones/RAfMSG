use bevy_replicon::prelude::ReplicationMode;
use lightyear::prelude::Tick;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

pub use crate::types::{ClientId, EntityId};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetError {
    Encode,
    Decode,
    Disconnected,
    Timeout,
    UnknownPeer,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    Reliable,
    Unreliable,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyncMode {
    Full,
    Once,
    None,
}

impl SyncMode {
    pub fn to_lightyear(self) -> Option<ReplicationMode> {
        match self {
            Self::Full => Some(ReplicationMode::OnChange),
            Self::Once => Some(ReplicationMode::Once),
            Self::None => None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Target {
    All,
    Single(ClientId),
    None,
}

impl Target {
    pub fn includes(&self, client: ClientId) -> bool {
        match self {
            Self::All => true,
            Self::Single(owner) => *owner == client,
            Self::None => false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct WireBody {
    pub px: f64,
    pub py: f64,
    pub pz: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
    pub qw: f64,
    pub wx: f64,
    pub wy: f64,
    pub wz: f64,
    pub mass: f64,
}

impl From<crate::types::ShipBody> for WireBody {
    fn from(body: crate::types::ShipBody) -> Self {
        Self {
            px: body.pos.x,
            py: body.pos.y,
            pz: body.pos.z,
            vx: body.vel.x,
            vy: body.vel.y,
            vz: body.vel.z,
            qx: body.ori.x,
            qy: body.ori.y,
            qz: body.ori.z,
            qw: body.ori.w,
            wx: body.ang_vel.x,
            wy: body.ang_vel.y,
            wz: body.ang_vel.z,
            mass: body.mass,
        }
    }
}

impl From<WireBody> for crate::types::ShipBody {
    fn from(wire: WireBody) -> Self {
        Self {
            pos: crate::types::Vec3::new(wire.px, wire.py, wire.pz),
            vel: crate::types::Vec3::new(wire.vx, wire.vy, wire.vz),
            ori: crate::types::Quat::new(wire.qx, wire.qy, wire.qz, wire.qw),
            ang_vel: crate::types::Vec3::new(wire.wx, wire.wy, wire.wz),
            mass: wire.mass,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct InputMessage {
    pub tick: u32,
    pub client: u64,
    pub throttle: f64,
    pub yaw: f64,
    pub pitch: f64,
    pub roll: f64,
}

impl InputMessage {
    pub fn tick(&self) -> Tick {
        Tick(self.tick)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SnapshotMessage {
    pub tick: u32,
    pub bodies: Vec<(EntityId, WireBody)>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Delta {
    pub tick: u32,
    pub changed: Vec<(EntityId, WireBody)>,
    pub removed: Vec<EntityId>,
}

pub fn encode<T: Serialize>(message: &T) -> Result<Vec<u8>, NetError> {
    bincode::serialize(message).map_err(|_| NetError::Encode)
}

pub fn decode<T: for<'de> Deserialize<'de>>(bytes: &[u8]) -> Result<T, NetError> {
    bincode::deserialize(bytes).map_err(|_| NetError::Decode)
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BandwidthStats {
    pub messages: usize,
    pub raw_bytes: usize,
    pub wire_bytes: usize,
}

impl BandwidthStats {
    pub fn compression_ratio(&self) -> f64 {
        if self.wire_bytes == 0 {
            return 0.0;
        }
        self.raw_bytes as f64 / self.wire_bytes as f64
    }

    pub fn record(&mut self, raw_bytes: usize, wire_bytes: usize) {
        self.messages += 1;
        self.raw_bytes += raw_bytes;
        self.wire_bytes += wire_bytes;
    }
}

#[derive(Clone, Debug, Default)]
pub struct BandwidthProfiler {
    pub sent: BandwidthStats,
    pub received: BandwidthStats,
}

impl BandwidthProfiler {
    pub fn record_send<T: Serialize>(
        &mut self,
        message: &T,
        raw_bytes: usize,
    ) -> Result<Vec<u8>, NetError> {
        let bytes = encode(message)?;
        self.sent.record(raw_bytes, bytes.len());
        Ok(bytes)
    }

    pub fn record_receive<T: for<'de> Deserialize<'de>>(
        &mut self,
        bytes: &[u8],
        raw_bytes: usize,
    ) -> Result<T, NetError> {
        let message = decode(bytes)?;
        self.received.record(raw_bytes, bytes.len());
        Ok(message)
    }
}

pub trait Transport {
    fn send(&mut self, channel: Channel, peer: ClientId, payload: &[u8]) -> Result<(), NetError>;
    fn recv(&mut self) -> Vec<(ClientId, Channel, Vec<u8>)>;
}

#[derive(Default)]
pub struct MemoryTransport {
    inbox: VecDeque<(ClientId, Channel, Vec<u8>)>,
    pub sent: Vec<(ClientId, Channel, Vec<u8>)>,
}

impl MemoryTransport {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn deliver(&mut self, client: ClientId, channel: Channel, payload: Vec<u8>) {
        self.inbox.push_back((client, channel, payload));
    }

    pub fn drop_pending(&mut self) -> usize {
        let count = self.inbox.len();
        self.inbox.clear();
        count
    }
}

impl Transport for MemoryTransport {
    fn send(&mut self, channel: Channel, peer: ClientId, payload: &[u8]) -> Result<(), NetError> {
        self.sent.push((peer, channel, payload.to_vec()));
        self.inbox.push_back((peer, channel, payload.to_vec()));
        Ok(())
    }

    fn recv(&mut self) -> Vec<(ClientId, Channel, Vec<u8>)> {
        self.inbox.drain(..).collect()
    }
}

pub struct UdpTransport {
    socket: UdpSocket,
    peers: BTreeMap<ClientId, SocketAddr>,
}

impl UdpTransport {
    pub fn bind(addr: &str) -> Result<Self, NetError> {
        let socket = UdpSocket::bind(addr).map_err(|_| NetError::UnknownPeer)?;
        socket
            .set_read_timeout(Some(Duration::from_millis(20)))
            .map_err(|_| NetError::UnknownPeer)?;
        Ok(Self {
            socket,
            peers: BTreeMap::new(),
        })
    }

    pub fn local_addr(&self) -> SocketAddr {
        self.socket
            .local_addr()
            .expect("bound socket has an address")
    }

    pub fn add_peer(&mut self, client: ClientId, addr: SocketAddr) {
        self.peers.insert(client, addr);
    }
}

impl Transport for UdpTransport {
    fn send(&mut self, _channel: Channel, peer: ClientId, payload: &[u8]) -> Result<(), NetError> {
        let addr = self.peers.get(&peer).ok_or(NetError::UnknownPeer)?;
        self.socket
            .send_to(payload, addr)
            .map_err(|_| NetError::Disconnected)?;
        Ok(())
    }

    fn recv(&mut self) -> Vec<(ClientId, Channel, Vec<u8>)> {
        let mut out = Vec::new();
        let mut buf = [0u8; 4096];
        while let Ok((len, addr)) = self.socket.recv_from(&mut buf) {
            let client = self
                .peers
                .iter()
                .find(|(_, known)| **known == addr)
                .map(|(id, _)| *id)
                .unwrap_or(0);
            out.push((client, Channel::Unreliable, buf[..len].to_vec()));
        }
        out
    }
}

#[derive(Default)]
pub struct ReliableQueue {
    next_seq: u64,
    pending: BTreeMap<u64, Vec<u8>>,
    acked: BTreeSet<u64>,
}

impl ReliableQueue {
    pub fn send(&mut self, payload: &[u8]) -> u64 {
        let seq = self.next_seq;
        self.next_seq += 1;
        self.pending.insert(seq, payload.to_vec());
        seq
    }

    pub fn ack(&mut self, seq: u64) -> bool {
        self.acked.insert(seq);
        self.pending.remove(&seq).is_some()
    }

    pub fn retransmit<T: Transport>(&self, transport: &mut T, peer: ClientId) -> usize {
        let mut count = 0;
        for (seq, payload) in &self.pending {
            if !self.acked.contains(seq) && transport.send(Channel::Reliable, peer, payload).is_ok()
            {
                count += 1;
            }
        }
        count
    }

    pub fn pending_count(&self) -> usize {
        self.pending.len()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConnectionState {
    Disconnected,
    Connecting,
    Connected,
    TimedOut,
}

#[derive(Clone, Debug)]
pub struct Connection {
    pub client: ClientId,
    pub state: ConnectionState,
    pub last_heartbeat: u32,
    pub timeout_ticks: u32,
}

impl Connection {
    pub fn begin(client: ClientId, tick: u32, timeout_ticks: u32) -> Self {
        Self {
            client,
            state: ConnectionState::Connecting,
            last_heartbeat: tick,
            timeout_ticks,
        }
    }

    pub fn connect(client: ClientId, tick: u32, timeout_ticks: u32) -> Self {
        Self {
            client,
            state: ConnectionState::Connected,
            last_heartbeat: tick,
            timeout_ticks,
        }
    }

    pub fn heartbeat(&mut self, tick: u32) {
        self.last_heartbeat = tick;
        if self.state == ConnectionState::TimedOut || self.state == ConnectionState::Connecting {
            self.state = ConnectionState::Connected;
        }
    }

    pub fn update(&mut self, tick: u32) -> bool {
        let live =
            self.state == ConnectionState::Connected || self.state == ConnectionState::Connecting;
        if live && tick.saturating_sub(self.last_heartbeat) > self.timeout_ticks {
            self.state = ConnectionState::TimedOut;
            return true;
        }
        false
    }

    pub fn disconnect(&mut self) {
        self.state = ConnectionState::Disconnected;
    }
}

#[derive(Clone, Debug, Default)]
pub struct ReplicationRules {
    modes: BTreeMap<&'static str, SyncMode>,
    targets: BTreeMap<&'static str, Target>,
    rate_ticks: u32,
    sent_once: BTreeSet<(ClientId, EntityId)>,
}

impl ReplicationRules {
    pub fn new() -> Self {
        Self {
            rate_ticks: 1,
            ..Default::default()
        }
    }

    pub fn with_rate(rate_ticks: u32) -> Self {
        Self {
            rate_ticks: rate_ticks.max(1),
            ..Default::default()
        }
    }

    pub fn register(&mut self, name: &'static str, mode: SyncMode, target: Target) {
        self.modes.insert(name, mode);
        self.targets.insert(name, target);
    }

    pub fn mode_for(&self, name: &str) -> SyncMode {
        self.modes.get(name).copied().unwrap_or(SyncMode::None)
    }

    pub fn lightyear_mode(&self, name: &str) -> Option<ReplicationMode> {
        self.mode_for(name).to_lightyear()
    }

    pub fn should_send(
        &mut self,
        name: &str,
        client: ClientId,
        entity: EntityId,
        tick: u32,
    ) -> bool {
        if !tick.is_multiple_of(self.rate_ticks) {
            return false;
        }
        let mode = self.mode_for(name);
        let target = self.targets.get(name).cloned().unwrap_or(Target::All);
        if mode == SyncMode::None || !target.includes(client) {
            return false;
        }
        if mode == SyncMode::Once {
            return self.sent_once.insert((client, entity));
        }
        true
    }
}

pub fn build_snapshot(
    bodies: &BTreeMap<EntityId, crate::types::ShipBody>,
    client: ClientId,
    tick: u32,
    rules: &mut ReplicationRules,
) -> SnapshotMessage {
    let mut wire = Vec::new();
    for (&entity, body) in bodies {
        if rules.should_send("ShipBody", client, entity, tick) {
            wire.push((entity, WireBody::from(*body)));
        }
    }
    SnapshotMessage { tick, bodies: wire }
}

pub fn encode_delta(
    tick: u32,
    baseline: &BTreeMap<EntityId, WireBody>,
    current: &BTreeMap<EntityId, WireBody>,
) -> Delta {
    let mut changed = Vec::new();
    for (entity, body) in current {
        if baseline.get(entity) != Some(body) {
            changed.push((*entity, *body));
        }
    }
    let removed = baseline
        .keys()
        .filter(|id| !current.contains_key(*id))
        .copied()
        .collect();
    Delta {
        tick,
        changed,
        removed,
    }
}

pub fn apply_delta(baseline: &mut BTreeMap<EntityId, WireBody>, delta: &Delta) {
    for entity in &delta.removed {
        baseline.remove(entity);
    }
    for (entity, body) in &delta.changed {
        baseline.insert(*entity, *body);
    }
}
