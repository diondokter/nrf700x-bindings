#![cfg_attr(not(test), no_std)]
#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

include!("bindings.rs");

impl nrf_wifi_status {
    pub fn is_success(&self) -> bool {
        match self {
            nrf_wifi_status::NRF_WIFI_STATUS_SUCCESS => true,
            nrf_wifi_status::NRF_WIFI_STATUS_FAIL => false,
        }
    }

    pub fn failed(&self) -> bool {
        !self.is_success()
    }
}

#[no_mangle]
pub static nrf_wifi_umac_patch_pri_bimg: &[u8; 964] = include_bytes!(concat!(
    env!("OUT_DIR"),
    "/nrf_wifi_umac_patch_pri_bimg.bin"
));
#[no_mangle]
pub static nrf_wifi_umac_patch_sec_bin: &[u8; 37576] =
    include_bytes!(concat!(env!("OUT_DIR"), "/nrf_wifi_umac_patch_sec_bin.bin"));
#[no_mangle]
pub static nrf_wifi_lmac_patch_pri_bimg: &[u8; 796] = include_bytes!(concat!(
    env!("OUT_DIR"),
    "/nrf_wifi_lmac_patch_pri_bimg.bin"
));
#[no_mangle]
pub static nrf_wifi_lmac_patch_sec_bin: &[u8; 22448] =
    include_bytes!(concat!(env!("OUT_DIR"), "/nrf_wifi_lmac_patch_sec_bin.bin"));
