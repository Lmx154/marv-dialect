use std::path::PathBuf;

fn main() {
    let root = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap());

    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rerun-if-changed=definitions");

    // The generated module list is guarded by `feature = "dialect-common"` and
    // `feature = "dialect-marv"`, and the generated code mentions the features `serde`,
    // `arbitrary` and `ts-rs`. This crate offers none of them: it switches the first two on for
    // itself and tells rustc the others are known.
    println!("cargo::rustc-cfg=feature=\"dialect-common\"");
    println!("cargo::rustc-cfg=feature=\"dialect-marv\"");
    println!(
        "cargo::rustc-check-cfg=cfg(feature, values(\"dialect-common\", \"dialect-marv\", \"serde\", \"arbitrary\", \"ts-rs\"))"
    );

    let common = root.join("definitions").join("common.xml");
    let marv = root.join("definitions").join("marv.xml");
    let result = mavlink_bindgen::generate(
        mavlink_bindgen::XmlDefinitions::Files(vec![common, marv]),
        out,
    )
    .expect("generating the dialects from definitions/common.xml and definitions/marv.xml");
    mavlink_bindgen::emit_cargo_build_messages(&result);
}
