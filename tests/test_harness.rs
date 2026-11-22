use ragfmsg::net::NetConfig;
use ragfmsg::test_harness::*;
use ragfmsg::types::Vec3;

#[test]
fn harness_runner_records_and_serializes() {
    let mut runner = TestRunner::new();
    runner.run("passing", || Ok(()));
    runner.run("failing", || Err("deliberate".to_string()));
    let report = runner.finish();
    assert_eq!(report.passed, vec!["passing"]);
    assert_eq!(report.failed, vec!["failing: deliberate"]);
    assert!(!report.all_passed());
    assert_eq!(report.summary(), "1 passed, 1 failed");
    let json = report.to_json();
    assert!(json.contains("\"passed\""));
    assert!(json.contains("failing"));
}

#[test]
fn harness_determinism_checkers() {
    assert!(DeterminismChecker::bit_exact(1u64, 1u64).is_ok());
    assert!(DeterminismChecker::bit_exact(1u64, 2u64).is_err());
    assert!(DeterminismChecker::within_epsilon(1.001, 1.0, 0.01).is_ok());
    assert!(DeterminismChecker::within_epsilon(1.1, 1.0, 0.01).is_err());
    assert!(DeterminismChecker::position(Vec3::ZERO, Vec3::new(0.001, 0.0, 0.0), 0.01).is_ok());
    assert!(DeterminismChecker::position(Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), 0.01).is_err());
}

#[test]
fn harness_seeded_inputs_reproducible() {
    let a = SeededInputs::new(42).sequence(20);
    let b = SeededInputs::new(42).sequence(20);
    let c = SeededInputs::new(43).sequence(20);
    assert_eq!(a, b);
    assert_ne!(a, c);
    assert!(a
        .iter()
        .all(|input| input.throttle.abs() <= 1.0 && input.yaw.abs() <= 1.0));
}

#[test]
fn harness_seam_templates_cover_all_eight_seams() {
    let report = seam_templates();
    assert_eq!(report.passed.len(), 8);
    assert!(report.all_passed());
    for seam in SeamId::ALL {
        assert!(report.passed.contains(&seam.name().to_string()));
    }
}

#[test]
fn harness_headless_run_is_deterministic() {
    let cfg = NetConfig {
        uplink_latency: 1,
        downlink_latency: 2,
        jitter: 1,
        input_loss: 0.1,
        snapshot_loss: 0.1,
        seed: 7,
    };
    let first = run_headless(cfg, 200, 1234, None);
    let second = run_headless(cfg, 200, 1234, None);
    assert_eq!(first.server_tick, second.server_tick);
    assert_eq!(first.stats, second.stats);
    assert_eq!(first.rollbacks, second.rollbacks);
    assert_eq!(first.authoritative_pos, second.authoritative_pos);
    assert_eq!(first.predicted_pos, second.predicted_pos);
}

#[test]
fn harness_headless_confirmed_equals_real_sent_snapshot() {
    let cfg = NetConfig {
        input_loss: 0.2,
        snapshot_loss: 0.2,
        jitter: 2,
        seed: 99,
        ..Default::default()
    };
    let run = run_headless(cfg, 300, 55, None);
    assert!(run.confirmed_matches_sent_snapshot);
    assert!(run.stats.inputs_dropped > 0);
    assert!(run.stats.snapshots_dropped > 0);
    assert!(convergence_check(&run).is_ok());
}

#[test]
fn harness_fault_injection_increases_rollbacks() {
    let cfg = NetConfig::default();
    let clean = run_headless(cfg, 120, 2026, None);
    let faulty = run_headless(cfg, 120, 2026, Some(60));
    assert_eq!(clean.rollbacks, 0);
    assert!(faulty.rollbacks > clean.rollbacks);
    assert!(
        convergence_check(&faulty).is_ok(),
        "reconciliation should still converge"
    );
}

#[test]
fn harness_latency_does_not_corrupt_final_state() {
    let cfg = NetConfig {
        uplink_latency: 3,
        downlink_latency: 4,
        jitter: 2,
        input_loss: 0.05,
        snapshot_loss: 0.05,
        seed: 11,
    };
    let run = run_headless(cfg, 240, 77, None);
    assert_eq!(run.server_tick, 240);
    assert_eq!(run.client_tick, 240);
    assert!(run.confirmed_matches_sent_snapshot);
    assert!(convergence_check(&run).is_ok());
}
