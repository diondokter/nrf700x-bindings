use std::{env, ffi::OsStr, path::PathBuf};

fn main() {
    compile_lib();
    create_bindings();
}

const INCLUDES: &[&str] = &[
    "./sdk-nrf/drivers/wifi/nrf700x/osal/fw_if/umac_if/inc/fw",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/fw_if/umac_if/inc",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/hw_if/hal/inc/fw",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/hw_if/hal/inc",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/os_if/inc",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/bus_if/bal/inc",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/bus_if/bus/qspi/inc",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/utils/inc"
];

const SOURCES: &[&str] = &[
    "./sdk-nrf/drivers/wifi/nrf700x/osal/fw_if/umac_if/src",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/os_if/src",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/hw_if/hal/src",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/bus_if/bal/src",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/bus_if/bus/qspi/src",
    "./sdk-nrf/drivers/wifi/nrf700x/osal/utils/src",
];

fn compile_lib() {
    cc::Build::new()
        .files(SOURCES.iter().map(|path|
            std::fs::read_dir(path)
                .unwrap()
                .map(|f| f.unwrap())
                .filter(|f| f.path().extension() == Some(OsStr::new("c")))
                .map(|f| f.path()),
        ).flatten())
        .define("CONFIG_NRF_WIFI_LOW_POWER", None)
        .define("CONFIG_NRF700X_MAX_TX_PENDING_QLEN", "1024")
        .define("CONFIG_NRF700X_TX_MAX_DATA_SIZE", "1024")
        .define("CONFIG_NRF700X_MAX_TX_TOKENS", "16")
        .warnings(false)
        .includes(INCLUDES)
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
        .clang_args(INCLUDES.iter().map(|include| format!("-I{include}")))
        .clang_arg("-DCONFIG_NRF_WIFI_LOW_POWER")
        .default_enum_style(bindgen::EnumVariation::Rust { non_exhaustive: false })
        .bitfield_enum(".*_FLAGS")
        .bitfield_enum(".*_flags")
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
