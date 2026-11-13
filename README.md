# marv-dialect

The MAVLink `common` dialect for the MARV V2 flight controller, generated at build time with extension fields and usable without the standard library, and the `marv` dialect, which is `common` plus the project's own messages (so far the test message `MARV_TEST` and the device diagnostics message `MARV_DEVICE_STATUS`, with its enums and one command). The crate holds the message structs (`common::HIGHRES_IMU_DATA`, `common::HEARTBEAT_DATA`, ...), their ids, CRC seeds and `ser` / `deser`, and re-exports the `mavlink-core` items needed to frame them (`MAVLinkV2MessageRaw`, `MavHeader`, `MessageData`, `calculate_crc`, the error types). It has no code of its own.

**Status: version 0.1.0 (`publish = false`).**

## Works with

The crate is pure: it does no I/O and names no HAL or MCU, so it works anywhere `mavlink-core` 0.21 (with its `embedded` feature) builds.

## Why it exists

The stock `mavlink` 0.21.0 crate cannot be built without `std` when extension fields are on: its feature `mav2-message-extensions` switches on the optional normal dependency `mavlink-bindgen` (which pulls in `regex` and `quick-xml`). Without that feature `HIGHRES_IMU` has no `id` field, which the flight controller's sensor layout needs, and `COMMAND_ACK`, `GPS_RAW_INT`, `SCALED_PRESSURE` and `AUTOPILOT_VERSION` lose their extension fields too. This crate runs the same generator (`mavlink-bindgen` =0.21.0) as a build dependency only, over the vendored XML, and depends at run time on `mavlink-core` =0.21.0, `num-traits`, `num-derive` and `bitflags`.

## Use

From a `no_std` crate:

```toml
marv-dialect = { git = "https://github.com/Lmx154/marv-dialect.git", default-features = false }
```

```rust
use marv_dialect::common::{HIGHRES_IMU_DATA, HighresImuUpdatedFlags};
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader};

let imu = HIGHRES_IMU_DATA {
    id: 2,
    fields_updated: HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC,
    ..Default::default()
};
let mut frame = MAVLinkV2MessageRaw::new();
frame.serialize_message_data(MavHeader::default(), &imu);
let bytes = frame.raw_bytes(); // ready to send
```

From a `std` crate, switch on `features = ["std"]` (it only adds `mavlink-core/std`). There are no `dialect-*` features: both dialects, `common` and `marv`, are always built. For the target, `cargo tree -e normal --target thumbv8m.main-none-eabihf | grep -cE 'mavlink-bindgen|quick-xml|regex|mavlink v'` prints 0.

## The `serde` feature

`features = ["serde"]` (off by default) makes the generated messages, enums and bit flags derive `Serialize` and `Deserialize`, as the `serde` feature of the `mavlink` crate does for its dialects. A host program that inspects the fields of any message generically (`serde_json::to_value(message)`, for example a ground station's message view) needs it; flight firmware and other embedded users leave it off, and with it off the crate and its normal dependencies are the same as without the feature (no `serde` in `cargo tree -e normal --target thumbv8m.main-none-eabihf`). The feature switches on `std`, because `mavlink-core` 0.21 supports `serde` only with `std`: a `no_std` build with `serde` is not supported. A `MARV_TEST` message serialises to `{"type":"MARV_TEST","seq":7,"time_ms":1000,"value":-2,"tag":1}` (key order as serde_json sorts it); `tests/serde.rs` checks it.

## The `marv` dialect and `MARV_TEST`

`definitions/marv.xml` includes `common.xml` and defines the project's own messages, with ids from the private range (42000 to 42999; this README gave no number, so 42420 from the owner's brief was taken, inside that range). `build.rs` generates it next to `common.xml`, so the crate has two modules: `marv_dialect::common`, unchanged (the module the other crates use, with the same wire format), and `marv_dialect::marv`, which holds every `common` message under the same ids and CRC seeds, plus the messages of `marv.xml`. Use `marv::MavMessage` as the message enum of a link that must carry both; use `common` where only `common` messages are needed.

**`MARV_TEST` is a test message.** It is a link test message with no meaning to a vehicle: the flight controller and the ground station send it to each other over the radio link to show that a message outside the common dialect passes. Nothing acts on it. On 2026-11-13 it was carried both ways through an ExpressLRS 4.0.1 radio (TX backpack 1.5.9) between the MARV V2 board and the ground station: 193 of 202 received by the station (the 9 lost all in the first second of listening), 100 of 100 received by the board.

| | |
|---|---|
| type | `marv_dialect::marv::MARV_TEST_DATA` (enum variant `marv::MavMessage::MARV_TEST`) |
| id | 42420 (0x00A5B4) |
| CRC_EXTRA | 100 (0x64) |
| payload | 13 bytes |

| field | type | wire offset | meaning |
|---|---|---|---|
| `seq` | `uint32_t` | 0 | counter that the sender increments by one per message |
| `time_ms` | `uint32_t` | 4 | the sender's clock in milliseconds |
| `value` | `int32_t` | 8 | any test value |
| `tag` | `uint8_t` | 12 | the sender's role: 1 = flight controller, 2 = ground station |

There are no extension fields. As in every MAVLink 2 frame, trailing zero bytes of the payload are cut on the wire (at least one byte is kept) and read back as zero. `tests/marv_test.rs` checks the payload bytes, the CRC_EXTRA (computed there by hand), the cut and the equality of the `common` part of `marv` with `common`.

```rust
use marv_dialect::marv::MARV_TEST_DATA;
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader};

let test = MARV_TEST_DATA { seq: 1, time_ms: 1000, value: -2, tag: 1 };
let mut frame = MAVLinkV2MessageRaw::new();
frame.serialize_message_data(MavHeader::default(), &test);
let bytes = frame.raw_bytes(); // ready to send
```

## `MARV_DEVICE_STATUS`

**Status: defined and host-tested (`tests/marv_device_status.rs`, `tests/same_wire.rs`, `tests/serde.rs`); not yet sent by any firmware.** The message is the health of one device of the flight controller, for the ground station: one row (one message) per device; every counter is since boot and wraps. The design is section 4 of `_plan/decision-foundation-20261113.md` (milestone FD1).

| | |
|---|---|
| type | `marv_dialect::marv::MARV_DEVICE_STATUS_DATA` (enum variant `marv::MavMessage::MARV_DEVICE_STATUS`) |
| id | 42421 (0x00A5B5) |
| CRC_EXTRA | 231 (0xE7) |
| payload | 30 bytes, fixed (no extension fields; trailing zero bytes are cut on the wire as in any MAVLink 2 frame) |

| field | type | wire offset | meaning |
|---|---|---|---|
| `time_boot_ms` | `uint32_t` | 0 | the flight controller's time since boot, in milliseconds |
| `samples` | `uint32_t` | 4 | samples delivered by the device since boot |
| `errors` | `uint32_t` | 8 | failed transfers or reads since boot |
| `timeouts` | `uint32_t` | 12 | reads that timed out since boot |
| `last_sample_age_ms` | `uint16_t` | 16 | age of the last sample in milliseconds, saturating; 65535 = no sample yet |
| `recoveries` | `uint16_t` | 18 | recoveries since boot |
| `attempts` | `uint16_t` | 20 | restart attempts since boot |
| `overflows` | `uint16_t` | 22 | samples lost to a full buffer since boot |
| `config_restored` | `uint16_t` | 24 | times the device configuration was written again since boot |
| `device_id` | `uint8_t`, `MARV_DEVICE` | 26 | the device the row is about |
| `state` | `uint8_t`, `MARV_DEVICE_STATE` | 27 | state of the device |
| `last_error` | `uint8_t`, `MARV_DEVICE_ERROR` | 28 | the last error of the device |
| `flags` | `uint8_t` | 29 | bit 0: the sample source is HIL; bit 1: a self-test is running (a plain `uint8_t`, no bit mask enum) |

The generated enum types (variants carry the XML names, for example `MarvDevice::MARV_DEVICE_IMU1`):

| XML enum | Rust type | values |
|---|---|---|
| `MARV_DEVICE` | `marv::MarvDevice` | 1 `IMU1`, 2 `IMU2`, 3 `BARO`, 4 `MAG`, 5 `GNSS`, 6 `BATT`, 7 `RC`, 9 `SD`, 10 `PSRAM`, 11 `LED`, 12 `ESC` (8 is deliberately unused; 0 is not a device) |
| `MARV_DEVICE_STATE` | `marv::MarvDeviceState` | 0 `ABSENT`, 1 `STARTING`, 2 `OK`, 3 `DEGRADED`, 4 `RECOVERING`, 5 `FAILED` |
| `MARV_DEVICE_ERROR` | `marv::MarvDeviceError` | 0 `NONE`, 1 `BUS`, 2 `TIMEOUT`, 3 `ID_MISMATCH`, 4 `CONFIG_MISMATCH`, 5 `POR`, 6 `STUCK` |
| `MARV_DEVICE_ACTION` | `marv::MarvDeviceAction` | 1 `RESTART`, 2 `SELF_TEST` |

The entries are named `MARV_DEVICE_<name>`, `MARV_DEVICE_STATE_<name>`, `MARV_DEVICE_ERROR_<name>` and `MARV_DEVICE_ACTION_<name>`. Because the generated enums refuse a value that is not an entry, a message whose `device_id` is 0 or 8 (or whose `state` or `last_error` is out of range) does not decode: `deser` returns an error. `marv_dialect::common` does not contain any of this.

## `MAV_CMD_MARV_DEVICE`

**Status: defined and host-tested (`tests/marv_device_status.rs`); not yet sent or acted on by any firmware.** Command 42430, restart or self-test one device. `marv.xml` extends the `MAV_CMD` enum of `common.xml` with this one entry (the generator accepts that), so `marv::MavCmd::MAV_CMD_MARV_DEVICE` exists next to every `common` command, with the same values; `common::MavCmd` is unchanged.

| param | meaning |
|---|---|
| 1 | the device: a `MARV_DEVICE` value |
| 2 | the action: a `MARV_DEVICE_ACTION` value (1 restart, 2 self-test) |
| 3 to 7 | unused (0) |

It travels in `COMMAND_LONG` (`marv::COMMAND_LONG_DATA`, `command: MavCmd::MAV_CMD_MARV_DEVICE`); the parameters are floats on the wire, so `MarvDevice::from_f32(param1)` (`num_traits::FromPrimitive`) reads the device.

## Adding a message

Add it to `definitions/marv.xml` with an id from the private range (42000 to 42999) that is not used yet. `build.rs` already generates that file next to `common.xml`. A message of `marv.xml` has no counterpart in the `mavlink` crate, so `tests/same_wire.rs` skips it: add its id and name to `OWN_MESSAGES` there, and test it in its own file, as `tests/marv_test.rs` does for `MARV_TEST` and `tests/marv_device_status.rs` for `MARV_DEVICE_STATUS`.

## Lock-step

The flight controller (this crate) and the ground station (the `mavlink` crate) must write and read the same bytes. `tests/same_wire.rs` is the check, against the std `mavlink` crate as a dev-dependency: `cargo test --features std --test same_wire -- --nocapture`. It checks the sha256 of each vendored file (and that they equal the files in the `mavlink` 0.21.0 package, when that package is in the local cargo registry), then for every message id the name, the CRC seed, the encoded length and the bytes of a non-zero payload after a round trip in both directions, then a positive control that shows the comparison would catch a dialect built without extension fields, then `HIGHRES_IMU` with `id` 2 and NaN fields in both directions. A change of the XML or of the generator version is made at both ends at once, gated by this test. The check uses its own SHA-256 and a small XML scan, so it adds no dependency.

## Vendored

`definitions/` holds byte-identical copies of the files of the `mavlink` 0.21.0 package (`mavlink/message_definitions/v1.0/`):

| file | sha256 |
|---|---|
| `common.xml` | `f157929cf3c4fa8f15485a1b6697b7afb64fc0435ca2b8ab2ac9d37e35ee2dfd` |
| `standard.xml` | `fd57e0f3e767821310ba51384faeb6d7a6141f279cecfffe6c7c14b9385569ca` |
| `minimal.xml` | `7bbf84456396089cf4dd9661ebe94e48a81009c047e826ed2778b4b9be60145c` |

`common.xml` and `marv.xml` are generated (`marv.xml` is this crate's own file and is not in this table); `common.xml` includes the other two.

## Licence

The crate is MIT (`LICENSE`). The MAVLink project states at https://mavlink.io/en/#license: "The message definition XML files and the generated C-language version of MAVLink (a header-only library) are made available under the MIT-licence." `definitions/LICENSE-MAVLINK` says where the three XML files come from and quotes that statement; `definitions/COPYING-MAVLINK` is the licence file that ships next to them upstream, unchanged.
