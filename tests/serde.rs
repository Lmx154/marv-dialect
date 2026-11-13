//! The optional `serde` feature: the generated types of the `marv` dialect serialise the way the
//! types of the `mavlink` crate do with its own `serde` feature.
//!
//! Run with `cargo test --features serde --test serde -- --nocapture`.

#![cfg(feature = "serde")]

use marv_dialect::marv::{
    HEARTBEAT_DATA, MARV_DEVICE_STATUS_DATA, MARV_TEST_DATA, MarvDevice, MarvDeviceError,
    MarvDeviceState, MavMessage,
};
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
fn marv_device_status_serialises_with_its_thirteen_fields() {
    let message = MavMessage::MARV_DEVICE_STATUS(MARV_DEVICE_STATUS_DATA {
        time_boot_ms: 1000,
        samples: 2000,
        errors: 3,
        timeouts: 4,
        last_sample_age_ms: 5,
        recoveries: 6,
        attempts: 7,
        overflows: 8,
        config_restored: 9,
        device_id: MarvDevice::MARV_DEVICE_BARO,
        state: MarvDeviceState::MARV_DEVICE_STATE_OK,
        last_error: MarvDeviceError::MARV_DEVICE_ERROR_NONE,
        flags: 1,
    });
    let value = serde_json::to_value(&message).unwrap();
    println!("MARV_DEVICE_STATUS as JSON: {value}");
    let fields = value.as_object().unwrap();
    assert_eq!(
        fields.get("type"),
        Some(&Value::from("MARV_DEVICE_STATUS")),
        "got {value}"
    );
    for (name, expected) in [
        ("time_boot_ms", json!(1000)),
        ("samples", json!(2000)),
        ("errors", json!(3)),
        ("timeouts", json!(4)),
        ("last_sample_age_ms", json!(5)),
        ("recoveries", json!(6)),
        ("attempts", json!(7)),
        ("overflows", json!(8)),
        ("config_restored", json!(9)),
        ("device_id", json!({"type": "MARV_DEVICE_BARO"})),
        ("state", json!({"type": "MARV_DEVICE_STATE_OK"})),
        ("last_error", json!({"type": "MARV_DEVICE_ERROR_NONE"})),
        ("flags", json!(1)),
    ] {
        assert_eq!(fields.get(name), Some(&expected), "field {name} in {value}");
    }
    assert_eq!(fields.len(), 14, "type and thirteen fields: {value}");
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
