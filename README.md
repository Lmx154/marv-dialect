# marv-dialect

The MAVLink `common` dialect for the MARV V2 flight controller, generated at build time with extension fields and usable without the standard library, and the `marv` dialect, which is `common` plus the project's own messages (so far one test message, `MARV_TEST`). The crate holds the message structs (`common::HIGHRES_IMU_DATA`, `common::HEARTBEAT_DATA`, ...), their ids, CRC seeds and `ser` / `deser`, and re-exports the `mavlink-core` items needed to frame them (`MAVLinkV2MessageRaw`, `MavHeader`, `MessageData`, `calculate_crc`, the error types). It has no code of its own.

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

## Adding a message

Add it to `definitions/marv.xml` with an id from the private range (42000 to 42999) that is not used yet. `build.rs` already generates that file next to `common.xml`. A message of `marv.xml` has no counterpart in the `mavlink` crate, so `tests/same_wire.rs` skips it: add its id and name to `OWN_MESSAGES` there, and test it in its own file, as `tests/marv_test.rs` does for `MARV_TEST`.

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
