use hmac::{Hmac, Mac};
use rusqlite::{params, Connection};
use serde::{Deserialize, Serialize};
use sha2::Sha256;
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use uuid::Uuid;

type HmacSha256 = Hmac<Sha256>;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BackendError {
    InvalidToken,
    SessionNotFound,
    SessionFull,
    AlreadyJoined,
    NotJoined,
    Persistence(String),
}

fn hex_encode(bytes: &[u8]) -> String {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(DIGITS[(byte >> 4) as usize] as char);
        out.push(DIGITS[(byte & 0x0f) as usize] as char);
    }
    out
}

#[derive(Clone, Debug)]
pub struct AuthService {
    secret: Vec<u8>,
}

impl AuthService {
    pub fn new(secret: &[u8]) -> Self {
        Self {
            secret: secret.to_vec(),
        }
    }

    fn signature(&self, user: &str) -> Vec<u8> {
        let mut mac =
            HmacSha256::new_from_slice(&self.secret).expect("HMAC accepts any key length");
        mac.update(user.as_bytes());
        mac.finalize().into_bytes().to_vec()
    }

    pub fn issue_token(&self, user: &str) -> String {
        format!("{}.{}", user, hex_encode(&self.signature(user)))
    }

    pub fn verify(&self, token: &str) -> Result<String, BackendError> {
        let (user, signature) = token.rsplit_once('.').ok_or(BackendError::InvalidToken)?;
        let expected = hex_encode(&self.signature(user));
        if signature == expected {
            Ok(user.to_string())
        } else {
            Err(BackendError::InvalidToken)
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SessionId(pub u128);

impl SessionId {
    pub fn new() -> Self {
        Self(Uuid::from_u128(rand_session_id()).as_u128())
    }

    pub fn as_string(self) -> String {
        Uuid::from_u128(self.0).to_string()
    }
}

impl Default for SessionId {
    fn default() -> Self {
        Self::new()
    }
}

fn rand_session_id() -> u128 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(1);
    let high = COUNTER.fetch_add(1, Ordering::Relaxed) as u128;
    (high << 64) | 0x9aaa_aaaa_aaaa_aaaa
}

#[derive(Clone, Debug)]
pub struct Session {
    pub id: SessionId,
    pub capacity: usize,
    pub members: BTreeSet<String>,
    pub active: bool,
}

impl Session {
    pub fn new(capacity: usize) -> Self {
        Self {
            id: SessionId::new(),
            capacity,
            members: BTreeSet::new(),
            active: true,
        }
    }

    pub fn is_full(&self) -> bool {
        self.members.len() >= self.capacity
    }
}

#[derive(Clone, Debug, Default)]
pub struct SessionManager {
    pub sessions: BTreeMap<SessionId, Session>,
}

impl SessionManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn create(&mut self, capacity: usize) -> SessionId {
        let session = Session::new(capacity);
        let id = session.id;
        self.sessions.insert(id, session);
        id
    }

    pub fn join(&mut self, id: SessionId, user: &str) -> Result<(), BackendError> {
        let session = self
            .sessions
            .get_mut(&id)
            .ok_or(BackendError::SessionNotFound)?;
        if !session.active {
            return Err(BackendError::SessionNotFound);
        }
        if session.members.contains(user) {
            return Err(BackendError::AlreadyJoined);
        }
        if session.is_full() {
            return Err(BackendError::SessionFull);
        }
        session.members.insert(user.to_string());
        Ok(())
    }

    pub fn leave(&mut self, id: SessionId, user: &str) -> Result<(), BackendError> {
        let session = self
            .sessions
            .get_mut(&id)
            .ok_or(BackendError::SessionNotFound)?;
        if session.members.remove(user) {
            Ok(())
        } else {
            Err(BackendError::NotJoined)
        }
    }

    pub fn destroy(&mut self, id: SessionId) -> bool {
        self.sessions.remove(&id).is_some()
    }

    pub fn matchmake(&mut self, user: &str, capacity: usize) -> Result<SessionId, BackendError> {
        if let Some(id) = self
            .sessions
            .iter()
            .find(|(_, session)| session.active && !session.is_full())
            .map(|(id, _)| *id)
        {
            self.join(id, user)?;
            return Ok(id);
        }
        let id = self.create(capacity);
        self.join(id, user)?;
        Ok(id)
    }

    pub fn members_of(&self, id: SessionId) -> Option<&BTreeSet<String>> {
        self.sessions.get(&id).map(|session| &session.members)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PersistedShip {
    pub pos_x: f64,
    pub pos_y: f64,
    pub pos_z: f64,
    pub vel_x: f64,
    pub vel_y: f64,
    pub vel_z: f64,
    pub hull: f64,
    pub inventory: BTreeMap<String, u64>,
}

impl PersistedShip {
    pub fn at_origin(hull: f64) -> Self {
        Self {
            pos_x: 0.0,
            pos_y: 0.0,
            pos_z: 0.0,
            vel_x: 0.0,
            vel_y: 0.0,
            vel_z: 0.0,
            hull,
            inventory: BTreeMap::new(),
        }
    }
}

pub struct Persistence {
    connection: Connection,
}

impl Persistence {
    pub fn open_in_memory() -> Result<Self, BackendError> {
        Self::open(":memory:")
    }

    pub fn open(path: impl AsRef<Path>) -> Result<Self, BackendError> {
        let connection =
            Connection::open(path).map_err(|error| BackendError::Persistence(error.to_string()))?;
        connection
            .execute(
                "CREATE TABLE IF NOT EXISTS ships (user TEXT PRIMARY KEY, state TEXT NOT NULL)",
                [],
            )
            .map_err(|error| BackendError::Persistence(error.to_string()))?;
        Ok(Self { connection })
    }

    pub fn save(&self, user: &str, ship: &PersistedShip) -> Result<(), BackendError> {
        let json = serde_json::to_string(ship)
            .map_err(|error| BackendError::Persistence(error.to_string()))?;
        self.connection
            .execute(
                "INSERT INTO ships (user, state) VALUES (?1, ?2) \
                 ON CONFLICT(user) DO UPDATE SET state = excluded.state",
                params![user, json],
            )
            .map_err(|error| BackendError::Persistence(error.to_string()))?;
        Ok(())
    }

    pub fn load(&self, user: &str) -> Result<Option<PersistedShip>, BackendError> {
        let json: Option<String> = self
            .connection
            .query_row(
                "SELECT state FROM ships WHERE user = ?1",
                params![user],
                |row| row.get(0),
            )
            .optional()
            .map_err(|error| BackendError::Persistence(error.to_string()))?;
        match json {
            Some(json) => serde_json::from_str(&json)
                .map(Some)
                .map_err(|error| BackendError::Persistence(error.to_string())),
            None => Ok(None),
        }
    }

    pub fn delete(&self, user: &str) -> Result<bool, BackendError> {
        let removed = self
            .connection
            .execute("DELETE FROM ships WHERE user = ?1", params![user])
            .map_err(|error| BackendError::Persistence(error.to_string()))?;
        Ok(removed > 0)
    }
}

use rusqlite::OptionalExtension;

pub struct BackendService {
    pub auth: AuthService,
    pub sessions: SessionManager,
    pub persistence: Persistence,
}

impl BackendService {
    pub fn new(secret: &[u8], persistence: Persistence) -> Self {
        Self {
            auth: AuthService::new(secret),
            sessions: SessionManager::new(),
            persistence,
        }
    }

    pub fn authenticate(&self, token: &str) -> Result<String, BackendError> {
        self.auth.verify(token)
    }

    pub fn join(
        &mut self,
        token: &str,
        capacity: usize,
    ) -> Result<(String, SessionId), BackendError> {
        let user = self.auth.verify(token)?;
        let session = self.sessions.matchmake(&user, capacity)?;
        Ok((user, session))
    }

    pub fn persist(&self, user: &str, ship: &PersistedShip) -> Result<(), BackendError> {
        self.persistence.save(user, ship)
    }

    pub fn reconnect(&self, user: &str) -> Result<Option<PersistedShip>, BackendError> {
        self.persistence.load(user)
    }
}
