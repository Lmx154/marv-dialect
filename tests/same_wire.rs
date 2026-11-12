//! Lock-step check: the dialect generated here must be the same on the wire as the one in the
//! std `mavlink` crate (a dev-dependency, built with `mav2-message-extensions`).
//!
//! Run with `cargo test --features std --test same_wire -- --nocapture` to see the counts.

use std::collections::BTreeMap;
use std::path::PathBuf;

use marv_dialect::common as ours;
use marv_dialect::{MAVLinkV2MessageRaw, MavHeader, MavlinkVersion, MessageData, calculate_crc};
use mavlink::dialects::common as theirs;

// ---------------------------------------------------------------------------------------------
// The vendored files
// ---------------------------------------------------------------------------------------------

const VENDORED: [(&str, &str); 3] = [
    (
        "common.xml",
        "f157929cf3c4fa8f15485a1b6697b7afb64fc0435ca2b8ab2ac9d37e35ee2dfd",
    ),
    (
        "standard.xml",
        "fd57e0f3e767821310ba51384faeb6d7a6141f279cecfffe6c7c14b9385569ca",
    ),
    (
        "minimal.xml",
        "7bbf84456396089cf4dd9661ebe94e48a81009c047e826ed2778b4b9be60145c",
    ),
];

fn definitions(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("definitions")
        .join(name)
}

/// SHA-256 (FIPS 180-4), written here so that the test needs no hash dependency.
fn sha256_hex(data: &[u8]) -> String {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let mut h: [u32; 8] = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
        0x5be0cd19,
    ];
    let mut padded = data.to_vec();
    padded.push(0x80);
    while padded.len() % 64 != 56 {
        padded.push(0);
    }
    padded.extend_from_slice(&((data.len() as u64) * 8).to_be_bytes());
    for block in padded.as_chunks::<64>().0 {
        let mut w = [0u32; 64];
        for (i, word) in block.as_chunks::<4>().0.iter().enumerate() {
            w[i] = u32::from_be_bytes([word[0], word[1], word[2], word[3]]);
        }
        for i in 16..64 {
            let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
            let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let [mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut hh] = h;
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ (!e & g);
            let t1 = hh
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            hh = g;
            g = f;
            f = e;
            e = d.wrapping_add(t1);
            d = c;
            c = b;
            b = a;
            a = t1.wrapping_add(t2);
        }
        for (x, y) in h.iter_mut().zip([a, b, c, d, e, f, g, hh]) {
            *x = x.wrapping_add(y);
        }
    }
    h.iter().map(|x| format!("{x:08x}")).collect()
}

#[test]
fn sha256_matches_known_vectors() {
    assert_eq!(
        sha256_hex(b""),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    );
    assert_eq!(
        sha256_hex(b"abc"),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    );
    let long = [b'a'; 1000];
    assert_eq!(
        sha256_hex(&long),
        "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3"
    );
}

/// The folder `message_definitions/v1.0` of the `mavlink` 0.21.0 package in the cargo registry on
/// this machine, if the package is there.
fn mavlink_package_definitions() -> Option<PathBuf> {
    let cargo_home = std::env::var_os("CARGO_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cargo")))?;
    for entry in std::fs::read_dir(cargo_home.join("registry").join("src"))
        .ok()?
        .flatten()
    {
        let dir = entry
            .path()
            .join("mavlink-0.21.0/mavlink/message_definitions/v1.0");
        if dir.is_dir() {
            return Some(dir);
        }
    }
    None
}

#[test]
fn vendored_files_are_the_pinned_ones_and_the_ones_of_the_mavlink_package() {
    let package = mavlink_package_definitions();
    match &package {
        Some(dir) => println!(
            "comparing with the mavlink 0.21.0 package at {}",
            dir.display()
        ),
        None => println!(
            "NOTE: mavlink 0.21.0 package not found in the cargo registry; comparison with it skipped"
        ),
    }
    for (name, expected) in VENDORED {
        let ours = std::fs::read(definitions(name)).expect("vendored file");
        assert_eq!(
            sha256_hex(&ours),
            expected,
            "definitions/{name} differs from the pinned sha256"
        );
        if let Some(dir) = &package {
            let theirs = std::fs::read(dir.join(name)).expect("file of the mavlink package");
            assert!(
                ours == theirs,
                "definitions/{name} differs from the file in the mavlink 0.21.0 package"
            );
        }
    }
}

// ---------------------------------------------------------------------------------------------
// What the XML says
// ---------------------------------------------------------------------------------------------

struct Field {
    /// Size of one element in bytes.
    element: usize,
    /// Size of the whole field in bytes.
    size: usize,
    enumeration: Option<String>,
    extension: bool,
}

struct Layout {
    name: String,
    fields: Vec<Field>,
    /// Payload bytes before `<extensions/>`.
    base_len: usize,
    /// Payload bytes with the extension fields.
    full_len: usize,
}

fn attribute<'a>(tag: &'a str, name: &str) -> Option<&'a str> {
    let key = format!("{name}=\"");
    let start = tag.find(&key)? + key.len();
    Some(&tag[start..start + tag[start..].find('"')?])
}

fn type_size(ty: &str) -> (usize, usize) {
    let (base, count) = match ty.split_once('[') {
        Some((base, rest)) => (base, rest.trim_end_matches(']').parse::<usize>().unwrap()),
        None => (ty, 1),
    };
    let size = match base {
        "char" | "uint8_t" | "int8_t" | "uint8_t_mavlink_version" => 1,
        "uint16_t" | "int16_t" => 2,
        "uint32_t" | "int32_t" | "float" => 4,
        "uint64_t" | "int64_t" | "double" => 8,
        other => panic!("unknown field type {other}"),
    };
    (size, size * count)
}

/// Enum name -> values of its entries that are plain numbers, from the three vendored files.
fn xml_enums() -> BTreeMap<String, Vec<u64>> {
    let mut enums: BTreeMap<String, Vec<u64>> = BTreeMap::new();
    for (file, _) in VENDORED {
        let text = std::fs::read_to_string(definitions(file)).unwrap();
        let mut current = String::new();
        for line in text.lines().map(str::trim) {
            if line.starts_with("<enum ") {
                current = attribute(line, "name").unwrap().to_string();
            } else if line.starts_with("<entry ") {
                let value = attribute(line, "value").unwrap();
                let parsed = match value.strip_prefix("0x") {
                    Some(hex) => u64::from_str_radix(hex, 16).ok(),
                    None => value.parse().ok(),
                };
                if let Some(v) = parsed {
                    enums.entry(current.clone()).or_default().push(v);
                }
            }
        }
    }
    enums
}

/// Message id -> layout, from the three vendored files (`common.xml` includes the other two).
fn xml_layouts() -> BTreeMap<u32, Layout> {
    let mut layouts = BTreeMap::new();
    for (file, _) in VENDORED {
        let text = std::fs::read_to_string(definitions(file)).unwrap();
        let mut current: Option<(u32, Layout)> = None;
        let mut in_extensions = false;
        for line in text.lines() {
            let line = line.trim();
            if line.starts_with("<message ") {
                let id = attribute(line, "id").unwrap().parse().unwrap();
                let name = attribute(line, "name").unwrap().to_string();
                current = Some((
                    id,
                    Layout {
                        name,
                        fields: Vec::new(),
                        base_len: 0,
                        full_len: 0,
                    },
                ));
                in_extensions = false;
            } else if line.starts_with("<extensions") {
                in_extensions = true;
            } else if line.starts_with("<field ") {
                let (_, layout) = current.as_mut().expect("field outside a message");
                let (element, size) = type_size(attribute(line, "type").unwrap());
                layout.fields.push(Field {
                    element,
                    size,
                    enumeration: attribute(line, "enum").map(str::to_string),
                    extension: in_extensions,
                });
                layout.full_len += size;
                if !in_extensions {
                    layout.base_len += size;
                }
            } else if line.starts_with("</message>") {
                let (id, layout) = current.take().unwrap();
                assert!(
                    layouts.insert(id, layout).is_none(),
                    "id {id} defined twice"
                );
            }
        }
    }
    layouts
}

// ---------------------------------------------------------------------------------------------
// Every message id
// ---------------------------------------------------------------------------------------------

/// Payload candidates for one message. Every byte is non-zero, so every field, extension fields
/// included, is non-zero. First a per-id pattern with a valid, non-zero value written into each
/// enum field (a plain pattern is not a valid enum value); then the plain pattern; then constant
/// fills.
fn candidates(id: u32, layout: &Layout) -> Vec<Vec<u8>> {
    let len = layout.full_len;
    let pattern: Vec<u8> = (0..len)
        .map(|i| ((i + id as usize) % 255) as u8 + 1)
        .collect();

    // Wire order: base fields by element size, largest first (stable), then the extensions.
    let mut order: Vec<&Field> = layout.fields.iter().collect();
    order.sort_by_key(|f| {
        (
            f.extension,
            std::cmp::Reverse(if f.extension { 0 } else { f.element }),
        )
    });
    let enums = xml_enums();
    let mut valid = pattern.clone();
    let mut offset = 0;
    for field in order {
        if let Some(values) = field.enumeration.as_ref().and_then(|name| enums.get(name)) {
            // Prefer a value with no zero byte; otherwise any non-zero value that fits.
            let fits =
                |v: &&u64| **v != 0 && v.to_le_bytes()[field.size.min(8)..].iter().all(|&b| b == 0);
            let dense = |v: &&u64| v.to_le_bytes()[..field.size.min(8)].iter().all(|&b| b != 0);
            if field.size <= 8 {
                let pick = values
                    .iter()
                    .filter(fits)
                    .filter(dense)
                    .min()
                    .or_else(|| values.iter().filter(fits).min())
                    // An enum whose only value is 0: that byte stays zero, every other one is not.
                    .or_else(|| {
                        values
                            .iter()
                            .filter(|v| {
                                v.to_le_bytes()[field.size.min(8)..].iter().all(|&b| b == 0)
                            })
                            .min()
                    });
                if let Some(v) = pick {
                    valid[offset..offset + field.size]
                        .copy_from_slice(&v.to_le_bytes()[..field.size]);
                }
            }
        }
        offset += field.size;
    }

    let mut all = vec![valid, pattern];
    all.extend((1..=16u8).map(|fill| vec![fill; len]));
    all
}

fn ours_parse(id: u32, payload: &[u8]) -> Option<ours::MavMessage> {
    <ours::MavMessage as marv_dialect::Message>::parse(MavlinkVersion::V2, id, payload).ok()
}
fn theirs_parse(id: u32, payload: &[u8]) -> Option<theirs::MavMessage> {
    <theirs::MavMessage as mavlink::Message>::parse(mavlink::MavlinkVersion::V2, id, payload).ok()
}
fn ours_ser(m: &ours::MavMessage) -> Vec<u8> {
    let mut buf = [0u8; 300];
    let n = marv_dialect::Message::ser(m, MavlinkVersion::V2, &mut buf);
    buf[..n].to_vec()
}
fn theirs_ser(m: &theirs::MavMessage) -> Vec<u8> {
    let mut buf = [0u8; 300];
    let n = mavlink::Message::ser(m, mavlink::MavlinkVersion::V2, &mut buf);
    buf[..n].to_vec()
}

#[test]
fn every_message_is_the_same_on_the_wire() {
    // Positive control (see below): messages whose extension fields the stock dialect loses when
    // it is built without `mav2-message-extensions`.
    const WITH_EXTENSIONS: [&str; 5] = [
        "HIGHRES_IMU",
        "COMMAND_ACK",
        "GPS_RAW_INT",
        "SCALED_PRESSURE",
        "AUTOPILOT_VERSION",
    ];

    let layouts = xml_layouts();
    assert!(
        layouts.len() > 200,
        "only {} messages in the XML",
        layouts.len()
    );

    // The ids of the dialect are the ids of the XML, on both sides, and nothing else.
    let mut seen = 0;
    for id in 0..=0x00FF_FFFFu32 {
        let a = <ours::MavMessage as marv_dialect::Message>::default_message_from_id(id).is_some();
        let b = <theirs::MavMessage as mavlink::Message>::default_message_from_id(id).is_some();
        assert_eq!(a, b, "id {id} exists on one side only");
        assert_eq!(
            a,
            layouts.contains_key(&id),
            "id {id}: dialect and XML disagree"
        );
        seen += a as usize;
    }
    assert_eq!(seen, layouts.len());

    let (mut compared, mut not_comparable) = (0usize, Vec::new());
    let mut extended = 0usize;
    let mut checked_extension_messages = Vec::new();

    for (&id, layout) in &layouts {
        let a = <ours::MavMessage as marv_dialect::Message>::default_message_from_id(id).unwrap();
        let b = <theirs::MavMessage as mavlink::Message>::default_message_from_id(id).unwrap();
        assert_eq!(
            marv_dialect::Message::message_name(&a),
            layout.name,
            "name of id {id}"
        );
        assert_eq!(
            mavlink::Message::message_name(&b),
            layout.name,
            "name of id {id}"
        );
        assert_eq!(
            <ours::MavMessage as marv_dialect::Message>::extra_crc(id),
            <theirs::MavMessage as mavlink::Message>::extra_crc(id),
            "CRC seed of {}",
            layout.name
        );
        // Defaults are all zero; this only proves the two sides agree on the empty case.
        assert_eq!(
            ours_ser(&a),
            theirs_ser(&b),
            "default payload of {}",
            layout.name
        );

        // A payload of the full encoded length with no zero byte.
        let mut done = false;
        for payload in candidates(id, layout) {
            let (pa, pb) = (ours_parse(id, &payload), theirs_parse(id, &payload));
            assert_eq!(
                pa.is_some(),
                pb.is_some(),
                "{}: only one side parses the payload",
                layout.name
            );
            let (Some(pa), Some(pb)) = (pa, pb) else {
                continue;
            };

            // Direction 1: payload -> parse here, parse there -> serialise both -> same bytes.
            let (sa, sb) = (ours_ser(&pa), theirs_ser(&pb));
            assert_eq!(sa, sb, "{}: bytes differ after the round trip", layout.name);
            // The encoded length is the XML's full length, less trailing zero bytes (an enum value
            // with a zero byte can end the wire order), and the bytes are the ones that went in.
            let stripped = payload.iter().rposition(|&b| b != 0).map_or(1, |i| i + 1);
            assert_eq!(sa.len(), stripped, "{}: encoded length", layout.name);
            assert_eq!(
                sa,
                payload[..stripped],
                "{}: round trip changed the payload",
                layout.name
            );
            // Direction 2: what one side wrote is parsed by the other and written back unchanged.
            let back_a = ours_parse(id, &sb).expect("ours reads theirs");
            let back_b = theirs_parse(id, &sa).expect("theirs reads ours");
            assert_eq!(
                ours_ser(&back_a),
                sb,
                "{}: ours reading theirs",
                layout.name
            );
            assert_eq!(
                theirs_ser(&back_b),
                sa,
                "{}: theirs reading ours",
                layout.name
            );

            if layout.full_len > layout.base_len {
                extended += 1;
                // Positive control: bytes beyond the base length are carried, and are non-zero.
                assert!(
                    sa.len() > layout.base_len,
                    "{}: extension bytes missing",
                    layout.name
                );
                assert!(
                    sa[layout.base_len..].iter().any(|&byte| byte != 0),
                    "{}",
                    layout.name
                );
                if WITH_EXTENSIONS.contains(&layout.name.as_str()) {
                    checked_extension_messages.push(layout.name.clone());
                }
            }
            compared += 1;
            done = true;
            break;
        }
        if !done {
            // No non-zero fill is a valid value for every enum field. What is still compared:
            // name, CRC seed, default payload (above), and that both sides refuse each candidate.
            not_comparable.push(layout.name.clone());
        }
    }

    println!(
        "message ids: {}; compared fully: {compared}; not comparable: {}",
        layouts.len(),
        not_comparable.len()
    );
    println!(
        "not comparable (no candidate payload is valid for every enum field): {not_comparable:?}"
    );
    println!("messages with extension fields compared fully: {extended}");
    assert_eq!(compared + not_comparable.len(), layouts.len());
    assert!(compared > 200, "only {compared} messages compared fully");
    for name in WITH_EXTENSIONS {
        assert!(
            checked_extension_messages.iter().any(|n| n == name),
            "{name} was not compared with its extension bytes"
        );
    }
}

/// Positive control: the comparison above can see a dialect built without extension fields.
///
/// An all-zero payload would prove nothing. MAVLink 2 strips trailing zero bytes from the payload,
/// and the CRC seed (`extra_crc`) is computed from the base fields only, so a dialect without the
/// extension fields has the same name, the same seed and, for an all-zero message, the same bytes
/// (nothing). The difference shows only in a payload whose extension bytes are non-zero: it is
/// longer than the base length, and the bytes beyond the base length are the extension fields.
/// A dialect without them would write at most `base_len` bytes, which the equalities in
/// `every_message_is_the_same_on_the_wire` (length equal to the XML's full length, bytes equal to
/// the payload that went in) would reject.
#[test]
fn the_comparison_detects_a_dialect_without_extension_fields() {
    let layouts = xml_layouts();
    let mut zero_payloads = 0;
    for name in [
        "HIGHRES_IMU",
        "COMMAND_ACK",
        "GPS_RAW_INT",
        "SCALED_PRESSURE",
        "AUTOPILOT_VERSION",
    ] {
        let (&id, layout) = layouts.iter().find(|(_, l)| l.name == name).unwrap();
        assert!(
            layout.full_len > layout.base_len,
            "{name}: the XML has no extension fields"
        );
        let payload = candidates(id, layout).remove(0);
        let a = ours_parse(id, &payload).expect("parses");
        let b = theirs_parse(id, &payload).expect("parses");
        let (sa, sb) = (ours_ser(&a), theirs_ser(&b));
        assert_eq!(sa, sb);
        assert!(
            sa.len() > layout.base_len,
            "{name}: {} bytes, base {}",
            sa.len(),
            layout.base_len
        );
        assert!(
            sa[layout.base_len..].iter().any(|&byte| byte != 0),
            "{name}: extension bytes are zero"
        );

        // What a dialect without extension fields would have written: the base bytes only.
        let without_extensions = &payload[..layout.base_len];
        assert_ne!(
            sa.as_slice(),
            without_extensions,
            "{name}: the comparison would not notice"
        );
        assert_eq!(&sa[..layout.base_len], without_extensions);

        // And an all-zero message cannot tell the two apart.
        let zeros = vec![0u8; layout.full_len];
        if let (Some(za), Some(zb)) = (ours_parse(id, &zeros), theirs_parse(id, &zeros)) {
            assert_eq!(ours_ser(&za), theirs_ser(&zb));
            assert_eq!(
                ours_ser(&za).len(),
                1,
                "{name}: an all-zero payload is stripped to one byte"
            );
            zero_payloads += 1;
        } else {
            println!(
                "{name}: an all-zero payload is not a valid message (enum field), no zero-payload check"
            );
        }
    }
    assert!(
        zero_payloads >= 4,
        "only {zero_payloads} zero payloads checked"
    );
    // The compile-time form of the same fact for the field the flight controller needs.
    let imu = ours::HIGHRES_IMU_DATA {
        id: 2,
        ..Default::default()
    };
    assert_eq!(imu.id, 2);
    assert_eq!(ours::HIGHRES_IMU_DATA::ENCODED_LEN, 63);
}

// ---------------------------------------------------------------------------------------------
// HIGHRES_IMU with an instance id and NaN fields, through the frame layer
// ---------------------------------------------------------------------------------------------

fn nan_imu() -> ours::HIGHRES_IMU_DATA {
    ours::HIGHRES_IMU_DATA {
        time_usec: 123_456_789,
        xacc: 1.5,
        yacc: -2.5,
        zacc: -9.81,
        xgyro: f32::NAN,
        ygyro: f32::NAN,
        zgyro: f32::NAN,
        xmag: f32::NAN,
        ymag: f32::NAN,
        zmag: f32::NAN,
        abs_pressure: f32::NAN,
        diff_pressure: f32::NAN,
        pressure_alt: f32::NAN,
        temperature: f32::NAN,
        fields_updated: ours::HighresImuUpdatedFlags::from_bits_truncate(0x0007),
        id: 2,
    }
}

fn same_floats(a: &[f32], b: &[f32]) -> bool {
    a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits())
}

#[test]
fn highres_imu_with_an_instance_is_read_by_the_mavlink_crate() {
    let m = nan_imu();
    let mut frame = MAVLinkV2MessageRaw::new();
    frame.serialize_message_data(
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 7,
        },
        &m,
    );
    let bytes = frame.raw_bytes().to_vec();

    let mut reader = mavlink::MavlinkReader::new(&bytes[..]);
    let (header, message): (mavlink::MavHeader, theirs::MavMessage) = reader
        .read_message(mavlink::MavlinkVersion::V2)
        .expect("the mavlink crate reads our frame");
    assert_eq!(
        (header.system_id, header.component_id, header.sequence),
        (1, 1, 7)
    );
    let theirs::MavMessage::HIGHRES_IMU(read) = message else {
        panic!("wrong message")
    };
    assert_eq!(read.id, 2);
    assert_eq!(read.time_usec, m.time_usec);
    assert_eq!(read.fields_updated.bits(), 0x0007);
    assert!(same_floats(
        &[
            read.xacc,
            read.yacc,
            read.zacc,
            read.xgyro,
            read.ygyro,
            read.zgyro,
            read.xmag,
            read.ymag,
            read.zmag,
            read.abs_pressure,
            read.diff_pressure,
            read.pressure_alt,
            read.temperature
        ],
        &[
            m.xacc,
            m.yacc,
            m.zacc,
            m.xgyro,
            m.ygyro,
            m.zgyro,
            m.xmag,
            m.ymag,
            m.zmag,
            m.abs_pressure,
            m.diff_pressure,
            m.pressure_alt,
            m.temperature
        ],
    ));
}

#[test]
fn highres_imu_written_by_the_mavlink_crate_is_read_here() {
    let m = nan_imu();
    let theirs_message = theirs::MavMessage::HIGHRES_IMU(theirs::HIGHRES_IMU_DATA {
        time_usec: m.time_usec,
        xacc: m.xacc,
        yacc: m.yacc,
        zacc: m.zacc,
        xgyro: m.xgyro,
        ygyro: m.ygyro,
        zgyro: m.zgyro,
        xmag: m.xmag,
        ymag: m.ymag,
        zmag: m.zmag,
        abs_pressure: m.abs_pressure,
        diff_pressure: m.diff_pressure,
        pressure_alt: m.pressure_alt,
        temperature: m.temperature,
        fields_updated: theirs::HighresImuUpdatedFlags::from_bits_truncate(0x0007),
        id: 2,
    });
    let mut bytes = Vec::new();
    let header = mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 9,
    };
    mavlink::write_versioned_msg(
        &mut bytes,
        mavlink::MavlinkVersion::V2,
        header,
        &theirs_message,
    )
    .expect("the mavlink crate writes the frame");

    // STX, len, incompat, compat, seq, sys, comp, msgid[3], payload, crc[2]
    assert_eq!(bytes[0], 0xFD);
    let len = bytes[1] as usize;
    assert_eq!(bytes.len(), 12 + len, "no signature expected");
    assert_eq!(&bytes[7..10], &[105, 0, 0]);
    let crc = calculate_crc(&bytes[1..10 + len], ours::HIGHRES_IMU_DATA::EXTRA_CRC);
    assert_eq!(
        crc.to_le_bytes(),
        [bytes[10 + len], bytes[11 + len]],
        "CRC checked with our seed"
    );

    let read = ours::HIGHRES_IMU_DATA::deser(MavlinkVersion::V2, &bytes[10..10 + len])
        .expect("we read their payload");
    assert_eq!(read.id, 2);
    assert_eq!(read.time_usec, m.time_usec);
    assert_eq!(read.fields_updated.bits(), 0x0007);
    assert!(same_floats(
        &[
            read.xacc,
            read.yacc,
            read.zacc,
            read.xgyro,
            read.ygyro,
            read.zgyro,
            read.xmag,
            read.ymag,
            read.zmag,
            read.abs_pressure,
            read.diff_pressure,
            read.pressure_alt,
            read.temperature
        ],
        &[
            m.xacc,
            m.yacc,
            m.zacc,
            m.xgyro,
            m.ygyro,
            m.zgyro,
            m.xmag,
            m.ymag,
            m.zmag,
            m.abs_pressure,
            m.diff_pressure,
            m.pressure_alt,
            m.temperature
        ],
    ));
}
