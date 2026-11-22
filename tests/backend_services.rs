use ragfmsg::backend::*;
use std::collections::BTreeMap;

fn backend() -> BackendService {
    BackendService::new(b"test-secret", Persistence::open_in_memory().unwrap())
}

#[test]
fn backend_auth_token_roundtrip_and_tamper() {
    let auth = AuthService::new(b"secret");
    let token = auth.issue_token("alice");
    assert_eq!(auth.verify(&token).unwrap(), "alice");
    assert_eq!(
        auth.verify("alice.deadbeef"),
        Err(BackendError::InvalidToken)
    );
    let mut flipped = token.clone();
    let last = flipped.pop().unwrap();
    flipped.push(if last == '0' { '1' } else { '0' });
    assert_eq!(auth.verify(&flipped), Err(BackendError::InvalidToken));
    assert_eq!(auth.verify("not-a-token"), Err(BackendError::InvalidToken));
    let other = AuthService::new(b"different-secret");
    assert_eq!(other.verify(&token), Err(BackendError::InvalidToken));
}

#[test]
fn backend_session_join_leave_and_capacity() {
    let mut backend = backend();
    let id = backend.sessions.create(2);
    backend.sessions.join(id, "alice").unwrap();
    backend.sessions.join(id, "bob").unwrap();
    assert_eq!(
        backend.sessions.join(id, "carol"),
        Err(BackendError::SessionFull)
    );
    assert_eq!(
        backend.sessions.join(id, "alice"),
        Err(BackendError::AlreadyJoined)
    );
    backend.sessions.leave(id, "alice").unwrap();
    assert_eq!(
        backend.sessions.leave(id, "alice"),
        Err(BackendError::NotJoined)
    );
    backend.sessions.join(id, "carol").unwrap();
    assert_eq!(backend.sessions.members_of(id).unwrap().len(), 2);
}

#[test]
fn backend_session_destroy_and_missing() {
    let mut backend = backend();
    let id = backend.sessions.create(4);
    assert!(backend.sessions.destroy(id));
    assert!(!backend.sessions.destroy(id));
    assert_eq!(
        backend.sessions.join(id, "alice"),
        Err(BackendError::SessionNotFound)
    );
}

#[test]
fn backend_matchmaking_fills_then_creates() {
    let mut backend = backend();
    let first = backend.sessions.matchmake("alice", 2).unwrap();
    let second = backend.sessions.matchmake("bob", 2).unwrap();
    assert_eq!(first, second, "second player should join the open session");
    let third = backend.sessions.matchmake("carol", 2).unwrap();
    assert_ne!(third, first, "full session should trigger a new one");
    assert_eq!(backend.sessions.sessions.len(), 2);
}

#[test]
fn backend_persistence_roundtrip() {
    let store = Persistence::open_in_memory().unwrap();
    let mut ship = PersistedShip::at_origin(100.0);
    ship.pos_x = 1234.5;
    ship.vel_z = -7.25;
    ship.inventory.insert("fuel".to_string(), 42);
    store.save("alice", &ship).unwrap();

    let loaded = store.load("alice").unwrap().unwrap();
    assert_eq!(loaded, ship);
    assert_eq!(loaded.inventory.get("fuel"), Some(&42));
    assert!(store.load("bob").unwrap().is_none());
}

#[test]
fn backend_persistence_delete() {
    let store = Persistence::open_in_memory().unwrap();
    store
        .save("alice", &PersistedShip::at_origin(10.0))
        .unwrap();
    assert!(store.delete("alice").unwrap());
    assert!(!store.delete("alice").unwrap());
    assert!(store.load("alice").unwrap().is_none());
}

#[test]
fn backend_full_lifecycle_auth_join_spawn_persist_rejoin() {
    let mut backend = backend();
    let token = backend.auth.issue_token("alice");
    let (user, session) = backend.join(&token, 4).unwrap();
    assert_eq!(user, "alice");
    assert_eq!(backend.sessions.members_of(session).unwrap().len(), 1);

    let mut ship = PersistedShip::at_origin(100.0);
    ship.pos_x = 500.0;
    ship.inventory = BTreeMap::from([("quantum".to_string(), 3)]);
    backend.persist(&user, &ship).unwrap();
    backend.sessions.leave(session, &user).unwrap();

    let restored = backend.reconnect(&user).unwrap().unwrap();
    assert_eq!(restored.pos_x, 500.0);
    assert_eq!(restored.inventory.get("quantum"), Some(&3));
}

#[test]
fn backend_crash_recovery_from_file_database() {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(1);
    let path = std::env::temp_dir().join(format!(
        "ragfmsg-backend-{}-{}.db",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let _ = std::fs::remove_file(&path);

    let mut ship = PersistedShip::at_origin(75.0);
    ship.pos_y = 999.0;
    {
        let store = Persistence::open(&path).unwrap();
        store.save("alice", &ship).unwrap();
    }
    {
        let store = Persistence::open(&path).unwrap();
        let restored = store.load("alice").unwrap().unwrap();
        assert_eq!(restored, ship, "state survives a full connection restart");
    }
    let _ = std::fs::remove_file(&path);
}
