//! The `MARV_TEST` message of `definitions/marv.xml`: a link test message (id 42420) that is not in
//! the `common` dialect and has no counterpart in the `mavlink` crate.
//!
//! Run with `cargo test --test marv_test -- --nocapture` to see the id and the CRC_EXTRA.

use marv_dialect::marv::{self, MARV_TEST_DATA};
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message, MessageData};

const SEQ: u32 = 0x0102_0304;
const TIME_MS: u32 = 0x0A0B_0C0D;
const VALUE: i32 = -2;
const TAG: u8 = 1;

fn sample() -> MARV_TEST_DATA {
    MARV_TEST_DATA {
        seq: SEQ,
        time_ms: TIME_MS,
        value: VALUE,
        tag: TAG,
    }
}

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

#[test]
fn id_and_crc_extra() {
    // The CRC_EXTRA is the CRC of "<message name> " followed by "<type> <name> " of every base
    // field in wire order (largest type first, then XML order), folded to one byte.
    let text = b"MARV_TEST uint32_t seq uint32_t time_ms int32_t value uint8_t tag ";
    let crc = crc16_mcrf4xx(text);
    let by_hand = ((crc & 0xFF) ^ (crc >> 8)) as u8;

    println!(
        "MARV_TEST id {} CRC_EXTRA {} (0x{:02X})",
        MARV_TEST_DATA::ID,
        MARV_TEST_DATA::EXTRA_CRC,
        MARV_TEST_DATA::EXTRA_CRC
    );
    assert_eq!(MARV_TEST_DATA::ID, 42420);
    assert_eq!(MARV_TEST_DATA::NAME, "MARV_TEST");
    assert_eq!(MARV_TEST_DATA::ENCODED_LEN, 13);
    assert_eq!(MARV_TEST_DATA::EXTRA_CRC, by_hand);
    assert_eq!(
        <marv::MavMessage as Message>::extra_crc(42420),
        MARV_TEST_DATA::EXTRA_CRC
    );
    assert_eq!(
        MARV_TEST_DATA::EXTRA_CRC,
        100,
        "pinned: dependants rely on it"
    );
}

#[test]
fn frame_payload_layout_and_round_trip() {
    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 7,
    };
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message_data(header, &sample());

    // seq, time_ms, value (little endian), then tag: 13 bytes, no trailing zero to cut.
    let expected: [u8; 13] = [
        0x04, 0x03, 0x02, 0x01, // seq
        0x0D, 0x0C, 0x0B, 0x0A, // time_ms
        0xFE, 0xFF, 0xFF, 0xFF, // value = -2
        0x01, // tag
    ];
    assert_eq!(frame.payload(), expected);
    assert_eq!(frame.payload_length(), 13);
    assert_eq!(frame.message_id(), 42420);

    // STX, len, incompat, compat, seq, sys, comp, msgid[3], payload, crc[2]
    let raw = frame.raw_bytes();
    assert_eq!(raw.len(), 10 + 13 + 2);
    assert_eq!(
        raw[..10],
        [0xFD, 13, 0, 0, 7, 1, 1, 0xB4, 0xA5, 0x00],
        "42420 = 0x00A5B4"
    );
    assert_eq!(raw[10..23], expected);
    let crc = frame_crc(&raw[1..23], MARV_TEST_DATA::EXTRA_CRC);
    assert_eq!(raw[23..25], crc.to_le_bytes());
    assert!(frame.has_valid_crc::<marv::MavMessage>());

    let back = MARV_TEST_DATA::deser(MavlinkVersion::V2, frame.payload()).unwrap();
    assert_eq!(back, sample());

    let message =
        <marv::MavMessage as Message>::parse(MavlinkVersion::V2, 42420, frame.payload()).unwrap();
    assert_eq!(message, marv::MavMessage::MARV_TEST(sample()));
}

fn frame_crc(data: &[u8], extra: u8) -> u16 {
    let mut all = data.to_vec();
    all.push(extra);
    crc16_mcrf4xx(&all)
}

#[test]
fn trailing_zero_bytes_are_cut_and_still_decode() {
    // value 0 and tag 0 are the last five bytes: MAVLink 2 cuts them.
    let sent = MARV_TEST_DATA {
        seq: SEQ,
        time_ms: TIME_MS,
        value: 0,
        tag: 0,
    };
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message_data(MavHeader::default(), &sent);
    assert_eq!(
        frame.payload(),
        [0x04, 0x03, 0x02, 0x01, 0x0D, 0x0C, 0x0B, 0x0A]
    );
    assert_eq!(frame.payload_length(), 8);
    assert!(frame.has_valid_crc::<marv::MavMessage>());
    let back = MARV_TEST_DATA::deser(MavlinkVersion::V2, frame.payload()).unwrap();
    assert_eq!(back, sent);

    // Only the tag is zero: one byte is cut.
    let sent = MARV_TEST_DATA {
        seq: SEQ,
        time_ms: TIME_MS,
        value: VALUE,
        tag: 0,
    };
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message_data(MavHeader::default(), &sent);
    assert_eq!(frame.payload_length(), 12);
    let back = MARV_TEST_DATA::deser(MavlinkVersion::V2, frame.payload()).unwrap();
    assert_eq!(back, sent);

    // An all-zero message is cut to one byte (MAVLink 2 keeps at least the first byte).
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message_data(MavHeader::default(), &MARV_TEST_DATA::default());
    assert_eq!(frame.payload_length(), 1);
    let back = MARV_TEST_DATA::deser(MavlinkVersion::V2, frame.payload()).unwrap();
    assert_eq!(back, MARV_TEST_DATA::default());
}

#[test]
fn the_common_part_of_marv_is_the_common_dialect() {
    macro_rules! same {
        ($($name:ident),*) => {$(
            assert_eq!(
                <marv::$name as MessageData>::ID,
                <marv_dialect::common::$name as MessageData>::ID,
                stringify!($name)
            );
            assert_eq!(
                <marv::$name as MessageData>::EXTRA_CRC,
                <marv_dialect::common::$name as MessageData>::EXTRA_CRC,
                stringify!($name)
            );
            assert_eq!(
                <marv::$name as MessageData>::ENCODED_LEN,
                <marv_dialect::common::$name as MessageData>::ENCODED_LEN,
                stringify!($name)
            );
        )*};
    }
    for id in [0u32, 22, 30, 70, 109] {
        assert_eq!(
            <marv::MavMessage as Message>::extra_crc(id),
            <marv_dialect::common::MavMessage as Message>::extra_crc(id),
            "CRC_EXTRA of id {id}"
        );
    }
    same!(
        HEARTBEAT_DATA,
        PARAM_VALUE_DATA,
        RC_CHANNELS_OVERRIDE_DATA,
        RADIO_STATUS_DATA,
        ATTITUDE_DATA,
        HIGHRES_IMU_DATA
    );
}
