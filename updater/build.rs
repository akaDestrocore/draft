use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=custom.x");
    
    let out: &PathBuf = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    
    // copy memory.x to the output directory
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
        
    // create custom.x for our additional sections
    File::create(out.join("custom.x"))
        .unwrap()
        .write_all(include_bytes!("custom.x"))
        .unwrap();
    
    println!("cargo:rustc-link-search={}", out.display());
}