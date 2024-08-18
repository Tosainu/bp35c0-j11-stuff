//! This build script copies the `memory.x` file from the crate root into
//! a directory where the linker can always find it at build time.
//! For many projects this is optional, as the linker always searches the
//! project root directory -- wherever `Cargo.toml` is. However, if you
//! are using a workspace or have a more complicated build setup, this
//! build script becomes required. Additionally, by requesting that
//! Cargo re-run the build script whenever `memory.x` is changed,
//! updating `memory.x` ensures a rebuild of the application with the
//! new memory settings.

use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;

fn main() {
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory.x");

    let (id, password) = File::open("secrets.txt")
        .and_then(|file| {
            let mut file = BufReader::new(file);
            let mut id = String::new();
            file.read_line(&mut id)?;
            let mut password = String::new();
            file.read_line(&mut password)?;
            Ok((id, password))
        })
        .unwrap_or_else(|_| {
            (
                "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx".into(),
                "xxxxxxxxxxxx".into(),
            )
        });
    println!("cargo:rustc-env=SECRETS_ROUTE_B_ID={id}");
    println!("cargo:rustc-env=SECRETS_ROUTE_B_PASSWORD={password}");
    println!("cargo:rerun-if-changed=secrets.txt");
}
