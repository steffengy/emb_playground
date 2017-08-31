use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;
use std::str;

fn no_ident(u: &u8) -> bool {
    match *u {
        b'a'...b'z' | b'A'...b'Z' | b'0'...b'9' | b'_' => false,
        _ => true,
    }
}

fn main() {
    if env::var("CARGO_PKG_NAME").unwrap() == "board" {
        // Put the linker script somewhere the linker can find it
        let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

        let mut bytes: &[u8] = include_bytes!("memory.ld.x");
        let mut output = vec![];

        while let Some(next) = bytes.iter().position(|x| *x == b'$') {
            let (old, part) = (&bytes[..next], &bytes[next+1..]);
            output.extend_from_slice(old);
            let end = part.iter().position(no_ident).unwrap_or(part.len());
            let var = str::from_utf8(&part[..end]).unwrap();
            match env::var(var) {
                Ok(x) => output.extend_from_slice(x.as_bytes()),
                Err(err) => panic!("failed to get environment variable \"{}\": {:?}", var, err),
            }
            bytes = &part[end..];
        }
        output.extend_from_slice(bytes);

        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(&output)
            .unwrap();
        println!("cargo:rustc-link-search={}", out.display());

        println!("cargo:rerun-if-changed=build.rs");
        println!("cargo:rerun-if-changed=boards.toml");
        println!("cargo:rerun-if-changed=memory.x");
    }

    // Similar to not-supported target specific features
    // https://github.com/rust-lang/cargo/issues/1197 
    // We need to do this at this level, so we only recompile
    // our crates per board and not e.g. core or other dependencies
    for feature in env::var("BOARD_FEATURES").unwrap().split(" ") {
        println!("cargo:rustc-cfg=feature=\"{}\"", feature);
    }
}
