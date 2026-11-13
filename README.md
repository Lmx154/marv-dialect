# marv-dialect

The MAVLink `common` dialect for the MARV V2 flight controller, generated at build time with extension fields and usable without the standard library. The crate holds the message structs (`common::HIGHRES_IMU_DATA`, `common::HEARTBEAT_DATA`, ...), their ids, CRC seeds and `ser` / `deser`, and re-exports the `mavlink-core` items needed to frame them (`MAVLinkV2MessageRaw`, `MavHeader`, `MessageData`, `calculate_crc`, the error types). It has no code of its own.

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

From a `std` crate, switch on `features = ["std"]` (it only adds `mavlink-core/std`). There are no `dialect-*` features: the dialect is always `common`. For the target, `cargo tree -e normal --target thumbv8m.main-none-eabihf | grep -cE 'mavlink-bindgen|quick-xml|regex|mavlink v'` prints 0.

## Adding a message

Add `definitions/marv.xml` that starts with `<include>common.xml</include>`, give the new messages ids from the private range, and generate it in `build.rs` next to `common.xml`. A new message has no counterpart in the `mavlink` crate, so `tests/same_wire.rs` must be told to skip it.

## Lock-step

The flight controller (this crate) and the ground station (the `mavlink` crate) must write and read the same bytes. `tests/same_wire.rs` is the check, against the std `mavlink` crate as a dev-dependency: `cargo test --features std --test same_wire -- --nocapture`. It checks the sha256 of each vendored file (and that they equal the files in the `mavlink` 0.21.0 package, when that package is in the local cargo registry), then for every message id the name, the CRC seed, the encoded length and the bytes of a non-zero payload after a round trip in both directions, then a positive control that shows the comparison would catch a dialect built without extension fields, then `HIGHRES_IMU` with `id` 2 and NaN fields in both directions. A change of the XML or of the generator version is made at both ends at once, gated by this test. The check uses its own SHA-256 and a small XML scan, so it adds no dependency.

## Vendored

`definitions/` holds byte-identical copies of the files of the `mavlink` 0.21.0 package (`mavlink/message_definitions/v1.0/`):

| file | sha256 |
|---|---|
| `common.xml` | `f157929cf3c4fa8f15485a1b6697b7afb64fc0435ca2b8ab2ac9d37e35ee2dfd` |
| `standard.xml` | `fd57e0f3e767821310ba51384faeb6d7a6141f279cecfffe6c7c14b9385569ca` |
| `minimal.xml` | `7bbf84456396089cf4dd9661ebe94e48a81009c047e826ed2778b4b9be60145c` |

Only `common.xml` is generated; it includes the other two.

## Licence

The crate is MIT (`LICENSE`). The MAVLink project states at https://mavlink.io/en/#license: "The message definition XML files and the generated C-language version of MAVLink (a header-only library) are made available under the MIT-licence." `definitions/LICENSE-MAVLINK` says where the three XML files come from and quotes that statement; `definitions/COPYING-MAVLINK` is the licence file that ships next to them upstream, unchanged.
