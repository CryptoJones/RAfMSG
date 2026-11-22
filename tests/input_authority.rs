use bevy_input::prelude::*;
use ragfmsg::input::*;
use ragfmsg::types::ShipInput;

fn input(client: u64, entity: u64, seq: u64, tick: u32, throttle: f64) -> SequencedInput {
    SequencedInput::new(
        seq,
        tick,
        client,
        entity,
        InputSource::Keyboard,
        ShipInput::new(throttle, 0.25, 0.0, 0.0),
    )
}

#[test]
fn input_keyboard_device_sampling() {
    let bindings = KeyboardBindings::default();
    let mut keys = ButtonInput::<KeyCode>::default();
    assert_eq!(bindings.sample(&keys), ShipInput::NONE);
    keys.press(KeyCode::KeyW);
    keys.press(KeyCode::KeyD);
    let sampled = bindings.sample(&keys);
    assert_eq!(sampled.throttle, 1.0);
    assert_eq!(sampled.yaw, 1.0);
    keys.press(KeyCode::KeyS);
    assert_eq!(bindings.sample(&keys).throttle, 0.0);
}

#[test]
fn input_gamepad_and_touch_devices() {
    let pad = GamepadAxes {
        throttle: 2.0,
        yaw: 0.005,
        pitch: -0.5,
        roll: 0.0,
    };
    let sampled = pad.sample(0.01);
    assert_eq!(sampled.throttle, 1.0);
    assert_eq!(sampled.yaw, 0.0);
    assert_eq!(sampled.pitch, -0.5);

    let stick = TouchStick { x: 0.5, y: -2.0 };
    assert_eq!(stick.axes(0.01), (0.5, -1.0));
}

#[test]
fn input_router_merges_multiple_devices() {
    let mut router = InputRouter::new();
    router.submit(InputSource::Gamepad(0), ShipInput::new(0.5, 0.0, 0.0, 0.0));
    assert_eq!(router.merged().throttle, 0.5);
    router.add_device(InputSource::Keyboard);
    router.submit(InputSource::Keyboard, ShipInput::new(1.0, 0.0, 0.0, 0.0));
    assert_eq!(router.merged().throttle, 1.0);
    assert_eq!(router.device_count(), 2);
}

#[test]
fn input_buffer_tick_tagging_and_fallback() {
    let mut buffer = InputBuffer::new();
    buffer.push(&input(1, 10, 1, 5, 0.5)).unwrap();
    assert_eq!(buffer.at(5).throttle, 0.5);
    assert_eq!(
        buffer.at(6).throttle,
        0.5,
        "lost input falls back to last known"
    );
    assert_eq!(
        buffer.push(&input(1, 10, 1, 6, 1.0)),
        Err(InputViolation::StaleSequence)
    );
    assert_eq!(
        buffer.push(&input(1, 10, 2, 5, 1.0)),
        Err(InputViolation::DuplicateTick)
    );
    assert_eq!(buffer.len(), 1);
}

#[test]
fn input_buffer_replay_and_prune() {
    let mut buffer = InputBuffer::new();
    buffer.push(&input(1, 10, 1, 1, 0.25)).unwrap();
    buffer.push(&input(1, 10, 2, 2, 0.5)).unwrap();
    buffer.push(&input(1, 10, 3, 3, 1.0)).unwrap();
    let mut seen = Vec::new();
    buffer.replay(1u32, 3, |tick, value| seen.push((tick, value.throttle)));
    assert_eq!(seen, vec![(1, 0.25), (2, 0.5), (3, 1.0)]);
    assert_eq!(buffer.prune(3), 2);
    assert_eq!(buffer.len(), 1);
}

#[test]
fn input_authority_table() {
    let mut table = AuthorityTable::new();
    table.grant(1, 10);
    assert!(table.is_owner(1, 10));
    assert!(!table.is_owner(2, 10));
    table.grant(2, 10);
    assert!(!table.is_owner(1, 10));
    assert_eq!(table.owner_of(10), Some(2));
    assert!(table.revoke(10));
    assert!(!table.revoke(10));
}

#[test]
fn input_server_rejects_unauthoritative_and_invalid() {
    let mut validator = ServerInputValidator::new(3);
    validator.authority.grant(1, 10);
    validator.authority.grant(2, 20);

    assert_eq!(
        validator.ingest(input(2, 10, 1, 1, 0.5)),
        Err(InputViolation::NotOwner)
    );
    assert_eq!(
        validator.ingest(input(1, 10, 1, 1, 2.0)),
        Err(InputViolation::InvalidRange)
    );
    assert_eq!(
        validator.ingest(input(1, 10, 1, 1, f64::NAN)),
        Err(InputViolation::InvalidRange)
    );
    assert!(validator.ingest(input(1, 10, 1, 1, 0.5)).is_ok());
    assert_eq!(validator.input_at(1, 1).throttle, 0.5);
}

#[test]
fn input_rate_limit_and_replay() {
    let mut validator = ServerInputValidator::new(2);
    validator.authority.grant(1, 10);
    assert!(validator.ingest(input(1, 10, 1, 5, 0.25)).is_ok());
    assert!(validator.ingest(input(1, 10, 2, 6, 0.5)).is_ok());
    assert_eq!(
        validator.ingest(input(1, 10, 3, 7, 0.75)),
        Err(InputViolation::RateLimited)
    );
    let mut seen: Vec<(u32, f64)> = Vec::new();
    validator.replay(1, 1u32, 6, |tick, value| seen.push((tick, value.throttle)));
    assert_eq!(seen.last().unwrap(), &(6, 0.5));
}

struct BlockHighThrottle;

impl AntiCheatHook for BlockHighThrottle {
    fn validate(&self, input: &SequencedInput, _owner: Option<u64>) -> Result<(), InputViolation> {
        if input.throttle > 0.5 {
            Err(InputViolation::HookRejected)
        } else {
            Ok(())
        }
    }
}

#[test]
fn input_anticheat_hook() {
    let mut validator = ServerInputValidator::new(10);
    validator.authority.grant(1, 10);
    validator.add_hook(Box::new(BlockHighThrottle));
    assert!(validator.ingest(input(1, 10, 1, 1, 0.4)).is_ok());
    assert_eq!(
        validator.ingest(input(1, 10, 2, 2, 0.9)),
        Err(InputViolation::HookRejected)
    );
}
