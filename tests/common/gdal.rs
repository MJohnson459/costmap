//! Helpers for tests that call the GDAL command-line tools.
//!
//! Shared by the `costmap` and `costmap-geo` test crates through `#[path]` modules.

use std::{path::Path, process::Command};

/// Whether the GDAL tools can be used by the calling test.
///
/// Returns `false` after printing a notice when GDAL is not installed, so the test can
/// return early. Panics instead when the `CI` environment variable is set, so a CI job
/// without GDAL fails rather than passing without running the interop tests.
pub fn gdal_available(test: &str) -> bool {
    let installed = Command::new("gdalinfo")
        .arg("--version")
        .output()
        .is_ok_and(|output| output.status.success());
    if !installed {
        assert!(
            std::env::var_os("CI").is_none(),
            "{test}: GDAL is not installed (gdalinfo is not on PATH) but CI is set"
        );
        eprintln!("skipping {test}: gdalinfo is not on PATH");
    }
    installed
}

/// Run a GDAL tool, panicking with its stderr if it fails.
pub fn run_gdal(program: &str, args: &[&Path]) {
    let output = Command::new(program).args(args).output().unwrap();
    assert!(
        output.status.success(),
        "{program} failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}
