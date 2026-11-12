//! Generates Rust types for the Nav2 actions in `msgs/nav2_msgs`.
//!
//! `hiroz-msgs` bundles the standard ROS packages but not `nav2_msgs`. The
//! generated code refers to the standard types in `hiroz_msgs` and carries the
//! ROS 2 type hashes Nav2 checks, computed against the Lyrical definitions.

use std::collections::HashSet;
use std::path::{Path, PathBuf};

use hiroz_codegen::{GeneratorConfig, MessageGenerator, bundled_assets_dir_for};

/// Standard packages the Nav2 actions refer to, directly or through the
/// action protocol's own messages.
const STANDARD_PACKAGES: [&str; 7] = [
    "action_msgs",
    "builtin_interfaces",
    "geometry_msgs",
    "nav_msgs",
    "service_msgs",
    "std_msgs",
    "unique_identifier_msgs",
];

fn main() -> anyhow::Result<()> {
    let out_dir = PathBuf::from(std::env::var("OUT_DIR")?);
    let nav2_msgs = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR")?).join("msgs/nav2_msgs");
    println!("cargo:rerun-if-changed={}", nav2_msgs.display());

    let assets = bundled_assets_dir_for("lyrical");
    let standard: Vec<PathBuf> = STANDARD_PACKAGES
        .iter()
        .map(|name| assets.join(name))
        .collect();

    let config = GeneratorConfig {
        generate_cdr: true,
        generate_protobuf: false,
        generate_type_info: true,
        is_humble: false,
        output_dir: out_dir,
        external_crate: Some("hiroz_msgs".to_string()),
        local_packages: HashSet::from(["nav2_msgs".to_string()]),
        json_out: None,
    };
    let standard: Vec<&Path> = standard.iter().map(PathBuf::as_path).collect();
    MessageGenerator::new(config).generate_from_msg_files_with_deps(&[&nav2_msgs], &standard)
}
