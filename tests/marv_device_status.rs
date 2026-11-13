//! The `MARV_DEVICE_STATUS` message (id 42421), its enums and the command `MAV_CMD_MARV_DEVICE`
//! (42430) of `definitions/marv.xml`: none of them is in the `common` dialect or in the `mavlink`
//! crate.
//!
//! Run with `cargo test --test marv_device_status -- --nocapture` to see the id and the CRC_EXTRA.

use marv_dialect::marv::{
    self, COMMAND_LONG_DATA, MARV_DEVICE_STATUS_DATA, MarvDevice, MarvDeviceAction,
    MarvDeviceError, MarvDeviceState, MavCmd,
};
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message, MessageData};
use num_traits::FromPrimitive;

/// CRC_EXTRA of `MARV_DEVICE_STATUS`, pinned: dependants rely on it.
const CRC_EXTRA: u8 = 231;

fn sample() -> MARV_DEVICE_STATUS_DATA {
    MARV_DEVICE_STATUS_DATA {
        time_boot_ms: 0x0102_0304,
        samples: 0x0506_0708,
        errors: 0x090A_0B0C,
        timeouts: 0x0D0E_0F10,
        last_sample_age_ms: 0x1112,
        recoveries: 0x1314,
        attempts: 0x1516,
        overflows: 0x1718,
        config_restored: 0x191A,
        device_id: MarvDevice::MARV_DEVICE_ESC, // 12
        state: MarvDeviceState::MARV_DEVICE_STATE_RECOVERING, // 4
        last_error: MarvDeviceError::MARV_DEVICE_ERROR_STUCK, // 6
        flags: 0x03,
    }
}

/// The 30 payload bytes of `sample()`, worked out by hand: the four `u32` at 0, 4, 8, 12, the
/// five `u16` at 16, 18, 20, 22, 24, the four `u8` at 26, 27, 28, 29, all little endian.
const SAMPLE_BYTES: [u8; 30] = [
    0x04, 0x03, 0x02, 0x01, // 0  time_boot_ms
    0x08, 0x07, 0x06, 0x05, // 4  samples
    0x0C, 0x0B, 0x0A, 0x09, // 8  errors
    0x10, 0x0F, 0x0E, 0x0D, // 12 timeouts
    0x12, 0x11, // 16 last_sample_age_ms
    0x14, 0x13, // 18 recoveries
    0x16, 0x15, // 20 attempts
    0x18, 0x17, // 22 overflows
    0x1A, 0x19, // 24 config_restored
    12,   // 26 device_id
    4,    // 27 state
    6,    // 28 last_error
    0x03, // 29 flags
];

/// CRC-16/MCRF4XX (the MAVLink checksum), written here so that the CRC_EXTRA is not taken from
/// the code under test.
fn crc16_mcrf4xx(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        let mut tmp = byte ^ (crc & 0xFF) as u8;
        tmp ^= tmp << 4;
        let tmp = tmp as u16;
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    }
    crc
}

fn frame_crc(data: &[u8], extra: u8) -> u16 {
    let mut all = data.to_vec();
    all.push(extra);
    crc16_mcrf4xx(&all)
}

fn serialize(message: &MARV_DEVICE_STATUS_DATA) -> MAVLinkV2MessageRaw {
    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 7,
    };
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message_data(header, message);
    frame
}

#[test]
fn id_and_crc_extra() {
    // The CRC_EXTRA is the CRC of "<message name> " followed by "<type> <name> " of every base
    // field in wire order (largest type first, then XML order; an enum field counts as its base
    // type), folded to one byte.
    let text = b"MARV_DEVICE_STATUS \
        uint32_t time_boot_ms uint32_t samples uint32_t errors uint32_t timeouts \
        uint16_t last_sample_age_ms uint16_t recoveries uint16_t attempts uint16_t overflows \
        uint16_t config_restored \
        uint8_t device_id uint8_t state uint8_t last_error uint8_t flags ";
    let crc = crc16_mcrf4xx(text);
    let by_hand = ((crc & 0xFF) ^ (crc >> 8)) as u8;

    println!(
        "MARV_DEVICE_STATUS id {} CRC_EXTRA {} (0x{:02X})",
        MARV_DEVICE_STATUS_DATA::ID,
        MARV_DEVICE_STATUS_DATA::EXTRA_CRC,
        MARV_DEVICE_STATUS_DATA::EXTRA_CRC
    );
    assert_eq!(MARV_DEVICE_STATUS_DATA::ID, 42421);
    assert_eq!(MARV_DEVICE_STATUS_DATA::NAME, "MARV_DEVICE_STATUS");
    assert_eq!(MARV_DEVICE_STATUS_DATA::ENCODED_LEN, 30);
    assert_eq!(MARV_DEVICE_STATUS_DATA::EXTRA_CRC, by_hand);
    assert_eq!(
        <marv::MavMessage as Message>::extra_crc(42421),
        MARV_DEVICE_STATUS_DATA::EXTRA_CRC
    );
    assert_eq!(MARV_DEVICE_STATUS_DATA::EXTRA_CRC, CRC_EXTRA);
}

#[test]
fn payload_layout_and_round_trip() {
    let frame = serialize(&sample());
    assert_eq!(frame.payload(), SAMPLE_BYTES);
    assert_eq!(frame.payload_length(), 30);
    assert_eq!(frame.message_id(), 42421);

    // STX, len, incompat, compat, seq, sys, comp, msgid[3], payload, crc[2]
    let raw = frame.raw_bytes();
    assert_eq!(raw.len(), 10 + 30 + 2);
    assert_eq!(
        raw[..10],
        [0xFD, 30, 0, 0, 7, 1, 1, 0xB5, 0xA5, 0x00],
        "42421 = 0x00A5B5"
    );
    assert_eq!(raw[10..40], SAMPLE_BYTES);
    let crc = frame_crc(&raw[1..40], CRC_EXTRA);
    assert_eq!(raw[40..42], crc.to_le_bytes());
    assert!(frame.has_valid_crc::<marv::MavMessage>());

    let back = MARV_DEVICE_STATUS_DATA::deser(MavlinkVersion::V2, frame.payload()).unwrap();
    assert_eq!(back, sample());

    let message =
        <marv::MavMessage as Message>::parse(MavlinkVersion::V2, 42421, frame.payload()).unwrap();
    assert_eq!(message, marv::MavMessage::MARV_DEVICE_STATUS(sample()));

    // The message itself (not the frame) writes all 30 bytes.
    let mut buf = [0u8; 40];
    let n = sample().ser(MavlinkVersion::V2, &mut buf);
    assert_eq!(n, 30);
    assert_eq!(buf[..30], SAMPLE_BYTES);
}

#[test]
fn trailing_zero_bytes_are_cut_and_still_decode() {
    // state NONE-like zeros: ABSENT (0), no error (0) and no flags (0) are the last three bytes.
    let sent = MARV_DEVICE_STATUS_DATA {
        state: MarvDeviceState::MARV_DEVICE_STATE_ABSENT,
        last_error: MarvDeviceError::MARV_DEVICE_ERROR_NONE,
        flags: 0,
        ..sample()
    };
    let frame = serialize(&sent);
    assert_eq!(frame.payload_length(), 27);
    assert_eq!(frame.payload(), &SAMPLE_BYTES[..27]);
    assert!(frame.has_valid_crc::<marv::MavMessage>());
    let back = MARV_DEVICE_STATUS_DATA::deser(MavlinkVersion::V2, frame.payload()).unwrap();
    assert_eq!(back, sent);
    let message =
        <marv::MavMessage as Message>::parse(MavlinkVersion::V2, 42421, frame.payload()).unwrap();
    assert_eq!(message, marv::MavMessage::MARV_DEVICE_STATUS(sent));

    // Only the flags are zero: one byte is cut.
    let sent = MARV_DEVICE_STATUS_DATA {
        flags: 0,
        ..sample()
    };
    let frame = serialize(&sent);
    assert_eq!(frame.payload_length(), 29);
    let back = MARV_DEVICE_STATUS_DATA::deser(MavlinkVersion::V2, frame.payload()).unwrap();
    assert_eq!(back, sent);

    // device_id 0 is not a MARV_DEVICE: a payload of one zero byte (an all-zero message, cut) is
    // refused.
    let back = MARV_DEVICE_STATUS_DATA::deser(MavlinkVersion::V2, &[0]);
    assert!(back.is_err(), "device_id 0 was accepted: {back:?}");
}

#[test]
fn a_wrong_crc_extra_is_rejected() {
    let frame = serialize(&sample());
    assert!(frame.has_valid_crc_with_extra_crc(CRC_EXTRA));
    assert!(frame.has_valid_crc::<marv::MavMessage>());
    for wrong in [CRC_EXTRA ^ 0x01, CRC_EXTRA.wrapping_add(1), 0, 100] {
        assert!(
            !frame.has_valid_crc_with_extra_crc(wrong),
            "CRC_EXTRA {wrong} accepted"
        );
    }

    // A sender that seeded the checksum with another CRC_EXTRA: the same frame with the
    // checksum bytes replaced.
    let mut forged = serialize(&sample());
    let wrong_crc = frame_crc(&frame.raw_bytes()[1..40], CRC_EXTRA ^ 0x01);
    forged.as_mut_slice()[40..42].copy_from_slice(&wrong_crc.to_le_bytes());
    assert_eq!(forged.payload(), SAMPLE_BYTES);
    assert!(!forged.has_valid_crc::<marv::MavMessage>());
    assert!(forged.has_valid_crc_with_extra_crc(CRC_EXTRA ^ 0x01));
}

#[test]
fn enum_discriminants() {
    let devices: [(MarvDevice, u32); 11] = [
        (MarvDevice::MARV_DEVICE_IMU1, 1),
        (MarvDevice::MARV_DEVICE_IMU2, 2),
        (MarvDevice::MARV_DEVICE_BARO, 3),
        (MarvDevice::MARV_DEVICE_MAG, 4),
        (MarvDevice::MARV_DEVICE_GNSS, 5),
        (MarvDevice::MARV_DEVICE_BATT, 6),
        (MarvDevice::MARV_DEVICE_RC, 7),
        (MarvDevice::MARV_DEVICE_SD, 9),
        (MarvDevice::MARV_DEVICE_PSRAM, 10),
        (MarvDevice::MARV_DEVICE_LED, 11),
        (MarvDevice::MARV_DEVICE_ESC, 12),
    ];
    for (entry, value) in devices {
        assert_eq!(entry as u32, value, "{entry:?}");
        assert_eq!(MarvDevice::from_u32(value), Some(entry));
    }
    for absent in [0u32, 8, 13, 255] {
        assert_eq!(MarvDevice::from_u32(absent), None, "{absent}");
    }

    let states: [(MarvDeviceState, u32); 6] = [
        (MarvDeviceState::MARV_DEVICE_STATE_ABSENT, 0),
        (MarvDeviceState::MARV_DEVICE_STATE_STARTING, 1),
        (MarvDeviceState::MARV_DEVICE_STATE_OK, 2),
        (MarvDeviceState::MARV_DEVICE_STATE_DEGRADED, 3),
        (MarvDeviceState::MARV_DEVICE_STATE_RECOVERING, 4),
        (MarvDeviceState::MARV_DEVICE_STATE_FAILED, 5),
    ];
    for (entry, value) in states {
        assert_eq!(entry as u32, value, "{entry:?}");
        assert_eq!(MarvDeviceState::from_u32(value), Some(entry));
    }
    assert_eq!(MarvDeviceState::from_u32(6), None);

    let errors: [(MarvDeviceError, u32); 7] = [
        (MarvDeviceError::MARV_DEVICE_ERROR_NONE, 0),
        (MarvDeviceError::MARV_DEVICE_ERROR_BUS, 1),
        (MarvDeviceError::MARV_DEVICE_ERROR_TIMEOUT, 2),
        (MarvDeviceError::MARV_DEVICE_ERROR_ID_MISMATCH, 3),
        (MarvDeviceError::MARV_DEVICE_ERROR_CONFIG_MISMATCH, 4),
        (MarvDeviceError::MARV_DEVICE_ERROR_POR, 5),
        (MarvDeviceError::MARV_DEVICE_ERROR_STUCK, 6),
    ];
    for (entry, value) in errors {
        assert_eq!(entry as u32, value, "{entry:?}");
        assert_eq!(MarvDeviceError::from_u32(value), Some(entry));
    }
    assert_eq!(MarvDeviceError::from_u32(7), None);

    let actions: [(MarvDeviceAction, u32); 2] = [
        (MarvDeviceAction::MARV_DEVICE_ACTION_RESTART, 1),
        (MarvDeviceAction::MARV_DEVICE_ACTION_SELF_TEST, 2),
    ];
    for (entry, value) in actions {
        assert_eq!(entry as u32, value, "{entry:?}");
        assert_eq!(MarvDeviceAction::from_u32(value), Some(entry));
    }
    assert_eq!(MarvDeviceAction::from_u32(0), None);
    assert_eq!(MarvDeviceAction::from_u32(3), None);
}

#[test]
fn command_marv_device() {
    assert_eq!(MavCmd::MAV_CMD_MARV_DEVICE as u32, 42430);
    assert_eq!(MavCmd::from_u32(42430), Some(MavCmd::MAV_CMD_MARV_DEVICE));

    let command = COMMAND_LONG_DATA {
        target_system: 1,
        target_component: 1,
        command: MavCmd::MAV_CMD_MARV_DEVICE,
        confirmation: 0,
        param1: MarvDevice::MARV_DEVICE_MAG as u32 as f32,
        param2: MarvDeviceAction::MARV_DEVICE_ACTION_SELF_TEST as u32 as f32,
        ..Default::default()
    };
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message_data(MavHeader::default(), &command);
    assert!(frame.has_valid_crc::<marv::MavMessage>());
    // param1..param7 (f32), then the command (u16), then the three u8 of the target.
    let payload = frame.payload();
    assert_eq!(payload[..4], 4.0f32.to_le_bytes());
    assert_eq!(payload[4..8], 2.0f32.to_le_bytes());
    assert_eq!(payload[8..28], [0u8; 20]);
    assert_eq!(payload[28..30], 42430u16.to_le_bytes());

    let back = COMMAND_LONG_DATA::deser(MavlinkVersion::V2, frame.payload()).unwrap();
    assert_eq!(back, command);
    assert_eq!(back.command, MavCmd::MAV_CMD_MARV_DEVICE);
    assert_eq!(
        MarvDevice::from_f32(back.param1),
        Some(MarvDevice::MARV_DEVICE_MAG)
    );
    assert_eq!(
        MarvDeviceAction::from_f32(back.param2),
        Some(MarvDeviceAction::MARV_DEVICE_ACTION_SELF_TEST)
    );

    let message = <marv::MavMessage as Message>::parse(
        MavlinkVersion::V2,
        COMMAND_LONG_DATA::ID,
        frame.payload(),
    )
    .unwrap();
    assert_eq!(message, marv::MavMessage::COMMAND_LONG(command));
}

#[test]
fn the_common_commands_are_still_in_the_marv_enum() {
    for (command, value) in [
        (MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 241),
        (MavCmd::MAV_CMD_PREFLIGHT_STORAGE, 245),
        (MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 246),
        (MavCmd::MAV_CMD_REQUEST_MESSAGE, 512),
        (MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL, 511),
    ] {
        assert_eq!(command as u32, value, "{command:?}");
        assert_eq!(MavCmd::from_u32(value), Some(command));
        // The same entry of the common dialect has the same value.
        assert_eq!(
            marv_dialect::common::MavCmd::from_u32(value).map(|c| c as u32),
            Some(value)
        );
    }
    // And the common dialect has no MARV command.
    assert_eq!(marv_dialect::common::MavCmd::from_u32(42430), None);
}
