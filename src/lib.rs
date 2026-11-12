//! The MAVLink `common` dialect, generated at build time with extension fields, for `no_std`
//! targets.
//!
//! The stock `mavlink` crate cannot be built without `std` when extension fields are on, and
//! without extension fields `HIGHRES_IMU` has no `id`. This crate runs the same generator
//! (`mavlink-bindgen` 0.21.0, as a build dependency only) over `definitions/common.xml` and
//! depends at run time on `mavlink-core` 0.21.0 alone. Nothing here depends on the `mavlink`
//! crate, on `regex` or on `quick-xml` when it is built for a target.
//!
//! ```
//! use marv_dialect::common::{HIGHRES_IMU_DATA, HighresImuUpdatedFlags};
//! use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MessageData};
//!
//! let imu = HIGHRES_IMU_DATA { id: 2, fields_updated: HighresImuUpdatedFlags::HIGHRES_IMU_UPDATED_XACC, ..Default::default() };
//! let mut frame = MAVLinkV2MessageRaw::new();
//! frame.serialize_message_data(MavHeader::default(), &imu);
//! let back = HIGHRES_IMU_DATA::deser(marv_dialect::MavlinkVersion::V2, frame.payload()).unwrap();
//! assert_eq!(back.id, 2);
//! ```
//!
//! # Adding a message later
//!
//! Put a `marv.xml` next to the three vendored files in `definitions/`, start it with
//! `<include>common.xml</include>`, define the new messages with ids from the private range, and
//! generate it in `build.rs` in the same way as `common.xml`. A new message exists only here, so
//! the lock-step test in `tests/same_wire.rs` must be told that it has no counterpart in the
//! `mavlink` crate.
//!
//! # Lock-step
//!
//! Both ends of the link (the flight controller with this crate, the ground station with the
//! `mavlink` crate) must produce and read the same bytes. `tests/same_wire.rs`
//! (`cargo test --features std --test same_wire`) checks the hashes of the vendored XML files and,
//! for every message id, the name, the CRC seed, the encoded length and the bytes of a non-zero
//! payload against the `mavlink` crate. Any change of the XML files or of the generator version
//! is made at both ends at once, gated by that test.

#![cfg_attr(not(feature = "std"), no_std)]

pub use mavlink_core::error::{MessageReadError, MessageWriteError, ParserError};
pub use mavlink_core::types;
pub use mavlink_core::{
    MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, Message, MessageData, calculate_crc,
};

include!(concat!(env!("OUT_DIR"), "/mod.rs"));
