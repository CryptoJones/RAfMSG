//! Client network channel contract from the Ghidra 12.1.4 decompile on pluto
//! (`/mnt/hdd1/repos/OpenSourceSpaceGame/StarCitizen/functions/`).
//!
//! Sources:
//! - `FUN_1414a0f80` (`CNetChannel::InactivityTimerCallback`):
//!   - Reads `net_inactivitytimeout` (`fVar10`). Timeout is only active when
//!     `0.0 < timeout` and channel state `< 2` (0 = Connecting, 1 = Connected).
//!   - Converts seconds to 100ns ticks via `(longlong)(timeout * 1e+07)` and
//!     adds to `last_recv` (`param_1 + 0x148`). If `now >= last_recv + ticks`,
//!     logs `timeSinceRecv > net_inactivitytimeout` and disconnects.
//! - `FUN_1414a4560` (`CNetChannel::ProcessPacket`):
//!   - Packet type `0` in `Connecting` state checks magic `0x794fde53`
//!     (`sessionGuid`, `service_id`, `m_val`) or `0xdd3a1ee5` (`sessionGuid`)
//!     and matches `sessionGuid` against the channel's expected GUID.
//!   - Packet type `4` reads `pingTime` and `pongTime` for RTT / liveness.
//!
//! Comparison with `ragfmsg::network::Connection`:
//! - `Connection` models the same 4-state lifecycle (`Connecting`, `Connected`,
//!   `TimedOut`, `Disconnected`) and only times out from `Connecting` or
//!   `Connected` (`state < 2` in the decompile).

use ragfmsg::network::{Connection, ConnectionState};

const TICKS_PER_SECOND_100NS: f64 = 1e7;

fn inactivity_expired(last_recv_100ns: i64, now_100ns: i64, timeout_secs: f32, state: u8) -> bool {
    if timeout_secs <= 0.0 || state >= 2 {
        return false;
    }
    let timeout_ticks = (timeout_secs as f64 * TICKS_PER_SECOND_100NS) as i64;
    now_100ns >= last_recv_100ns + timeout_ticks
}

#[test]
fn cnetchannel_inactivity_only_triggers_when_timeout_positive_and_state_live() {
    // State 0 (Connecting) and 1 (Connected) can time out; >= 2 cannot.
    assert!(!inactivity_expired(0, 50_000_000, 0.0, 1));
    assert!(!inactivity_expired(0, 50_000_000, -1.0, 1));
    assert!(!inactivity_expired(0, 50_000_000, 1.0, 2));
    assert!(inactivity_expired(0, 10_000_000, 1.0, 0));
    assert!(inactivity_expired(0, 10_000_000, 1.0, 1));
    assert!(!inactivity_expired(0, 9_999_999, 1.0, 1));
}

#[test]
fn ragfmsg_connection_matches_cnetchannel_live_state_guard() {
    let mut conn = Connection::begin(1, 0, 60);
    assert_eq!(conn.state, ConnectionState::Connecting);
    assert!(!conn.update(60));
    assert!(conn.update(61));
    assert_eq!(conn.state, ConnectionState::TimedOut);

    // Once TimedOut (state >= 2), update does not re-trigger timeout.
    assert!(!conn.update(120));

    // Heartbeat restores Connected (state == 1) and resets liveness.
    conn.heartbeat(120);
    assert_eq!(conn.state, ConnectionState::Connected);
    assert!(!conn.update(180));
    assert!(conn.update(181));
    assert_eq!(conn.state, ConnectionState::TimedOut);
}
