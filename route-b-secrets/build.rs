fn main() {
    println!("cargo:rerun-if-changed=src/secrets.rs");
    println!("cargo::rustc-check-cfg=cfg(route_b_fallback_keys)");
    if !std::path::Path::new("src/secrets.rs").is_file() {
        println!("cargo::rustc-cfg=route_b_fallback_keys");
    }
}
