use std::{env, path::PathBuf};

fn main() {
    compile_lib();
    create_bindings();
}

fn compile_lib() {
    cc::Build::new()
        .file("./sdk-nrf/drivers/wifi/nrf700x/osal/os_if/src/osal.c")
        .define("CONFIG_NRF_WIFI_LOW_POWER", None)
        .include("./sdk-nrf/drivers/wifi/nrf700x/osal/os_if/inc")
        .compile("nrf700x");

    println!("cargo:rustc-link-lib=nrf700x");
}

fn create_bindings() {
    // Tell cargo to invalidate the built crate whenever the wrapper changes
    println!("cargo:rerun-if-changed=wrapper.h");

    // The bindgen::Builder is the main entry point
    // to bindgen, and lets you build up options for
    // the resulting bindings.
    let bindings = bindgen::Builder::default()
        // The input header we would like to generate
        // bindings for.
        .header("wrapper.h")
        // Tell cargo to invalidate the built crate whenever any of the
        // included header files changed.
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .clang_arg("-I./sdk-nrf/drivers/wifi")
        .clang_arg("-DCONFIG_NRF_WIFI_LOW_POWER")
        .use_core()
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    // Write the bindings to the $OUT_DIR/bindings.rs file.
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
