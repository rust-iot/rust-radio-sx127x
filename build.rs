// following https://doc.rust-lang.org/cargo/reference/build-scripts.html
use std::env;
//use std::io::Write;         //needed for debugging
//use std::path::PathBuf;     //needed for one approach
//use std::fs;                //needed for one approach

fn main() {
    // This file (build.rs) needs to be in the package root.
    // It is called before the linker and used to arrange for the linker to find the
    // proper memory.x file for the MCU. The memory.x files are assumed to be in a
    // directory memoryMaps/xxx/  where xxx is replaced by an one of

    let mcus = [
        "STM32F042",
        "STM32F030XC",
        "STM32F100",
        "STM32F101",
        "STM32F103",
        "STM32F303XC",
        "STM32F401",
        "STM32F411",
        "STM32F722",
        "STM32H742",
        "STM32L0X2",
        "STM32L100",
        "STM32L151",
        "STM32L4X2",
        "LM3S6965",
        "GD32VF103CB",
        "GD32VF103C8",
        "GD32VF103_EXTRA",
    ];

    // For example,   memoryMaps/STM32F401/memory.x
    // Note that the MCU string must be in upper case because it is also used to
    // find the  CARGO_FEATURE_xxx environment variable (eg CARGO_FEATURE_STM32F401)
    // which is in upper case.

    // There are two possible appraches: one is to copy the appropriate memory.x file into the OUT_DIR
    // where compiled pieces are placed for linking, then add that dir to the linker search path.
    // (Adding to path may be redundant, the OUT_DIR is probably already in the search path.)
    // The second approach is to simply add the location of the appropriate memory.x file to
    // the search path. This is not only quicker, but has the advantage that the memory.x file
    // can be MCU specific. In the copy appraoch the OUT_DIR is only MCU triple specific, so there
    // could be conflicts (in the unlikely situation) where two mcu's have the same triple and
    // different memory layouts.

    // The memoryNote.txt file is just to record some debugging information
    //let mut df = std::fs::File::create("memoryNote.txt").unwrap();

    // It is assumed that only one MCU feature will be specified. If there are more then
    // only the first is found (but actual code may be a mess if cargo really lets you do that).

    let pre = "CARGO_FEATURE_".to_owned();

    // For debugging. Write all CARGO_FEATURE_  mcu variables to file
    //for m in &mcus {
    //   match env::var_os(pre.clone() + m) {
    //      None    => df.write(format!("{}{} is not set\n", pre, m).as_bytes()).unwrap(),
    //      Some(x) => df.write(format!("{}{} is {:?}\n", pre, m, x).as_bytes()).unwrap()
    //      };
    //   };

    // For debugging. Write all env variables to file
    //df.write(format!("env::vars() gives\n").as_bytes()).unwrap();
    //for (key, value) in env::vars() {
    //     df.write(format!("   {:?}: {:?}\n", key, value).as_bytes()).unwrap();
    //     };

    // Compare mcus elements against CARGO_FEATURE_* env variables to determine directory of
    // memory.x file to use.
    // If there is no MCU feature identified then the usual default is that memory.x is
    // searched for in the package root.

    let mut indir: String = "".to_string();
    let d = "memoryMaps/".to_owned();
    for m in &mcus {
        let v = env::var_os(pre.clone() + m);
        if v.is_some() {
            indir = d + m;
            break;
        };
    }

    // Adding an empty search path causes problems compiling the crate, so skip if memory.x not found.
    // This allows 'cargo build  --features $MCU ' to work
    // but do not expect to compile examples. (There will be a 'cannot find linker script memory.x' error.)

    if !indir.is_empty() {
        //df.write(format!("in mcu found condition.\n").as_bytes()).unwrap();
        let infile = indir.clone() + "/memory.x";

        // one approach
        //let outdir  = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
        //let outfile = &PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("memory.x");
        //fs::copy(&infile, outfile).unwrap();  // Copy memory.x to OUT_DIR. Possibly should handle error.
        //println!("cargo:rustc-link-search={}", outdir.display());

        // other approach
        println!("cargo:rustc-link-search={}", indir);

        println!("cargo:rerun-if-changed=build.rs");
        println!("cargo:rerun-if-changed={}", infile);
    } else {
        //df.write(format!("mcu NOT found condition.\n").as_bytes()).unwrap();
        println!();
    }
}
