//! The optional `serde` feature: the generated types of the `marv` dialect serialise the way the
//! types of the `mavlink` crate do with its own `serde` feature.
//!
//! Run with `cargo test --features serde --test serde -- --nocapture`.

#![cfg(feature = "serde")]

use marv_dialect::marv::{HEARTBEAT_DATA, MARV_TEST_DATA, MavMessage};
use serde_json::{Value, json};

#[test]
fn marv_test_serialises_with_its_four_fields() {
    let message = MavMessage::MARV_TEST(MARV_TEST_DATA {
        seq: 7,
        time_ms: 1000,
        value: -2,
        tag: 1,
    });
    let value = serde_json::to_value(&message).unwrap();
    println!("MARV_TEST as JSON: {value}");
    let expected = json!({"type": "MARV_TEST", "seq": 7, "time_ms": 1000, "value": -2, "tag": 1});
    assert_eq!(value, expected, "got {value}");
    let fields = value.as_object().unwrap();
    for name in ["seq", "time_ms", "value", "tag"] {
        assert!(fields.contains_key(name), "no field {name} in {value}");
    }
}

#[test]
fn common_message_serialises() {
    let message = MavMessage::HEARTBEAT(HEARTBEAT_DATA::default());
    let value = serde_json::to_value(&message).unwrap();
    println!("HEARTBEAT as JSON: {value}");
    let fields = value.as_object().unwrap();
    assert_eq!(
        fields.get("type"),
        Some(&Value::from("HEARTBEAT")),
        "got {value}"
    );
    for name in [
        "custom_mode",
        "mavtype",
        "autopilot",
        "base_mode",
        "system_status",
        "mavlink_version",
    ] {
        assert!(fields.contains_key(name), "no field {name} in {value}");
    }
}
