use std::{fs::File, io::Read, path::Path};

use glob::glob;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    // println!("cargo:rerun-if-changed=src/bindings.rs");

    create_bindings();
    compile_lib();
    extract_fw_patches();
}

static C_FILES: &[&str] = &[
    "nrf700x/osal/fw_if/umac_if/src/default/fmac_api.c",
    "nrf700x/osal/fw_if/umac_if/src/fmac_api_common.c",
    "nrf700x/osal/fw_if/umac_if/src/fmac_util.c",
    "nrf700x/osal/fw_if/umac_if/src/event.c",
    "nrf700x/osal/fw_if/umac_if/src/rx.c",
    "nrf700x/osal/hw_if/hal/src/hpqm.c",
    "nrf700x/osal/hw_if/hal/src/hal_mem.c",
    "nrf700x/osal/os_if/src/osal.c",
    "nrf700x/osal/hw_if/hal/src/hal_api.c",
    "nrf700x/osal/utils/src/queue.c",
    "nrf700x/osal/utils/src/list.c",
    "nrf700x/osal/hw_if/hal/src/hal_interrupt.c",
    "nrf700x/osal/hw_if/hal/src/hal_reg.c",
    "nrf700x/osal/bus_if/bal/src/bal.c",
    "nrf700x/osal/hw_if/hal/src/pal.c",
    "nrf700x/osal/hw_if/hal/src/hal_fw_patch_loader.c"
];

fn compile_lib() {
    let mut build = cc::Build::new();

    if cfg!(windows) {
        build.compiler("clang");
    }

    build
        .files(C_FILES.into_iter().map(|p| {
            if p.starts_with("nrf700x") {
                format!("./sdk-nrf/drivers/wifi/{p}")
            } else {
                p.to_string()
            }
        }))
        .define("CONFIG_WIFI_NRF700X", "true")
        .define("CONFIG_NRF700X_SCAN_ONLY_MODE", "true")
        .define("CONFIG_NRF700X_RX_NUM_BUFS", "2")
        .define("CONFIG_NRF700X_RX_MAX_DATA_SIZE", "2048")
        .define("CONFIG_NRF_WIFI_IFACE_MTU", "1600")
        .warnings(false)
        .extra_warnings(false)
        .includes(
            glob("./sdk-nrf/drivers/wifi/nrf700x/osal/**/inc/**/*.h")
                .unwrap()
                .map(|x| x.unwrap().parent().unwrap().to_path_buf()),
        )
        .flag("-w") // Disable warnings. I don't care. It's not my code
        .debug(true)
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
        .header("wrapper.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .clang_arg("-I./sdk-nrf/drivers/wifi")
        .clang_args(
            glob("./sdk-nrf/drivers/wifi/nrf700x/osal/**/inc/**/*.h")
                .unwrap()
                .map(|x| x.unwrap().parent().unwrap().display().to_string())
                .map(|include| format!("-I{include}")),
        )
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        })
        .bitfield_enum(".*_FLAGS")
        .bitfield_enum(".*_flags")
        .use_core()
        .formatter(bindgen::Formatter::Rustfmt)
        // Finish the builder and generate the bindings.
        .generate()
        // Unwrap the Result and panic on failure.
        .expect("Unable to generate bindings");

    bindings
        .write_to_file("./src/bindings.rs")
        .expect("Couldn't write bindings!");
}

fn extract_fw_patches() {
    const PATCHES_FILE: &str =
        "sdk-nrf/drivers/wifi/nrf700x/osal/fw_if/umac_if/inc/fw/rpu_fw_patches.h";

    let mut patches_string = String::new();
    File::open(PATCHES_FILE)
        .unwrap()
        .read_to_string(&mut patches_string)
        .unwrap();

    let non_radio_test_start = patches_string.find("#else").unwrap();

    let patches_string = &patches_string[non_radio_test_start..];

    for item in patches_string
        .split_inclusive("const unsigned char __aligned(4) ")
        .skip(1)
    {
        let item_name = &item[..item.find("[").unwrap()];
        let item_data = &item[item.find("{").unwrap()..item.find("}").unwrap()]
            .strip_prefix("{")
            .unwrap()
            .replace("/n", "")
            .replace("/t", "");

        let mut data = Vec::new();

        for byte in item_data.split(",") {
            let byte = u8::from_str_radix(byte.trim().strip_prefix("0x").unwrap(), 16).unwrap();
            data.push(byte);
        }

        let out_dir = std::env::var("OUT_DIR").unwrap();
        File::create(Path::new(&out_dir).join(&format!("{item_name}.bin"))).unwrap();
    }
}
