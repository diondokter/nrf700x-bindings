#![allow(unused_variables)]

use defmt::{info, trace};
use nrf700x_sys::{nrf_wifi_bal_ops, nrf_wifi_osal_dma_dir, nrf_wifi_osal_priv, nrf_wifi_status};

#[no_mangle]
extern "C" fn get_bus_ops() -> *const nrf_wifi_bal_ops {
    info!("Getting bus ops");

    static BAL_OPS: nrf_wifi_bal_ops = nrf_wifi_bal_ops {
        init: Some(init),
        deinit: Some(deinit),
        dev_add: Some(dev_add),
        dev_rem: Some(dev_rem),
        dev_init: Some(dev_init),
        dev_deinit: Some(dev_deinit),
        read_word: Some(read_word),
        write_word: Some(write_word),
        read_block: Some(read_block),
        write_block: Some(write_block),
        dma_map: Some(dma_map),
        dma_unmap: Some(dma_unmap),
    };

    &BAL_OPS
}

unsafe extern "C" fn init(
    opriv: *mut nrf_wifi_osal_priv,
    cfg_params: *mut core::ffi::c_void,
    intr_callbk_fn: core::option::Option<
        unsafe extern "C" fn(hal_ctx: *mut core::ffi::c_void) -> nrf_wifi_status,
    >,
) -> *mut core::ffi::c_void {
    trace!("Called BUS init");
    todo!();
}
unsafe extern "C" fn deinit(bus_priv: *mut core::ffi::c_void) {
    trace!("Called BUS deinit");
    todo!();
}
unsafe extern "C" fn dev_add(
    bus_priv: *mut core::ffi::c_void,
    bal_dev_ctx: *mut core::ffi::c_void,
) -> *mut core::ffi::c_void {
    trace!("Called BUS dev_add");
    todo!();
}
unsafe extern "C" fn dev_rem(bus_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called BUS dev_rem");
    todo!();
}
unsafe extern "C" fn dev_init(bus_dev_ctx: *mut core::ffi::c_void) -> nrf_wifi_status {
    trace!("Called BUS dev_init");
    todo!();
}
unsafe extern "C" fn dev_deinit(bus_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called BUS dev_deinit");
    todo!();
}
unsafe extern "C" fn read_word(
    bus_dev_ctx: *mut core::ffi::c_void,
    addr_offset: core::ffi::c_ulong,
) -> core::ffi::c_uint {
    trace!("Called BUS read_word");
    todo!();
}
unsafe extern "C" fn write_word(
    bus_dev_ctx: *mut core::ffi::c_void,
    addr_offset: core::ffi::c_ulong,
    val: core::ffi::c_uint,
) {
    trace!("Called BUS write_word");
    todo!();
}
unsafe extern "C" fn read_block(
    bus_dev_ctx: *mut core::ffi::c_void,
    dest_addr: *mut core::ffi::c_void,
    src_addr_offset: core::ffi::c_ulong,
    len: usize,
) {
    trace!("Called BUS read_block");
    todo!();
}
unsafe extern "C" fn write_block(
    bus_dev_ctx: *mut core::ffi::c_void,
    dest_addr_offset: core::ffi::c_ulong,
    src_addr: *const core::ffi::c_void,
    len: usize,
) {
    trace!("Called BUS write_block");
    todo!();
}
unsafe extern "C" fn dma_map(
    bus_dev_ctx: *mut core::ffi::c_void,
    virt_addr: core::ffi::c_ulong,
    len: usize,
    dma_dir: nrf_wifi_osal_dma_dir,
) -> core::ffi::c_ulong {
    trace!("Called BUS dma_map");
    todo!();
}
unsafe extern "C" fn dma_unmap(
    bus_dev_ctx: *mut core::ffi::c_void,
    phy_addr: core::ffi::c_ulong,
    len: usize,
    dma_dir: nrf_wifi_osal_dma_dir,
) -> core::ffi::c_ulong {
    trace!("Called BUS dma_unmap");
    todo!();
}
