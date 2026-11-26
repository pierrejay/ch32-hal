fn main() {
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    // The HAL's `memory-x` feature provides memory.x, except on the H417: its V3F core runs from
    // the shared SRAM, laid out by h417/memory.x and h417/link.x, which take precedence (this
    // crate's link search path comes first).
    if std::env::var_os("CARGO_FEATURE_H417").is_some() {
        let out = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
        std::fs::write(out.join("memory.x"), include_bytes!("h417/memory.x")).unwrap();
        std::fs::write(out.join("link.x"), include_bytes!("h417/link.x")).unwrap();
        println!("cargo:rustc-link-search={}", out.display());
    }
    println!("cargo:rerun-if-changed=h417/memory.x");
    println!("cargo:rerun-if-changed=h417/link.x");
}
