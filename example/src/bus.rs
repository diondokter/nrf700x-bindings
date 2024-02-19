#![allow(unused_variables)]

extern crate alloc;

use alloc::boxed::Box;
use defmt::{info, trace};
use embedded_hal::spi::{Operation, SpiDevice};
use nrf700x_sys::{
    nrf_wifi_bal_cfg_params, nrf_wifi_bal_dev_ctx, nrf_wifi_bal_ops, nrf_wifi_fmac_dev_ctx,
    nrf_wifi_hal_dev_ctx, nrf_wifi_osal_dma_dir, nrf_wifi_osal_priv, nrf_wifi_status,
};

use crate::OsContext;

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
    let cfg_params = cfg_params.cast::<nrf_wifi_bal_cfg_params>();

    defmt::trace!("Called BUS init",);

    defmt::dbg!(Box::into_raw(Box::new(BalContext { intr_callbk_fn })).cast())
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

    let bus_priv = bus_priv.cast::<BalContext>();
    let bal_dev_ctx = bal_dev_ctx.cast::<nrf_wifi_bal_dev_ctx>();

    defmt::dbg!(Box::into_raw(Box::new(BusContext { bal_dev_ctx })).cast())
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
    defmt::trace!("Called BUS read_word from {:X}", addr_offset);

    let bus_dev_ctx = bus_dev_ctx.cast::<BusContext>();
    let bus_device = (*bus_dev_ctx).get_bus_device_object();

    let mut buffer = [0; 4];
    (*bus_device).read(addr_offset as u32, &mut buffer);
    u32::from_le_bytes(buffer)
}
unsafe extern "C" fn write_word(
    bus_dev_ctx: *mut core::ffi::c_void,
    addr_offset: core::ffi::c_ulong,
    val: core::ffi::c_uint,
) {
    trace!("Called BUS write_word to {:X}", addr_offset);

    let bus_dev_ctx = bus_dev_ctx.cast::<BusContext>();
    let bus_device = (*bus_dev_ctx).get_bus_device_object();

    (*bus_device).write(addr_offset as u32, &mut val.to_le_bytes());
}
unsafe extern "C" fn read_block(
    bus_dev_ctx: *mut core::ffi::c_void,
    dest_addr: *mut core::ffi::c_void,
    src_addr_offset: core::ffi::c_ulong,
    len: usize,
) {
    trace!("Called BUS read_block from {:X}", src_addr_offset);

    let bus_dev_ctx = bus_dev_ctx.cast::<BusContext>();
    let bus_device = (*bus_dev_ctx).get_bus_device_object();

    (*bus_device).read(src_addr_offset as u32, core::slice::from_raw_parts_mut(dest_addr.cast(), len));
}
unsafe extern "C" fn write_block(
    bus_dev_ctx: *mut core::ffi::c_void,
    dest_addr_offset: core::ffi::c_ulong,
    src_addr: *const core::ffi::c_void,
    len: usize,
) {
    trace!("Called BUS write_block to {:X}", dest_addr_offset);

    let bus_dev_ctx = bus_dev_ctx.cast::<BusContext>();
    let bus_device = (*bus_dev_ctx).get_bus_device_object();

    (*bus_device).write(dest_addr_offset as u32, core::slice::from_raw_parts(src_addr.cast(), len));
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

#[derive(Debug)]
struct BalContext {
    intr_callbk_fn: core::option::Option<
        unsafe extern "C" fn(hal_ctx: *mut core::ffi::c_void) -> nrf_wifi_status,
    >,
}

struct BusContext {
    bal_dev_ctx: *mut nrf_wifi_bal_dev_ctx,
}

impl BusContext {
    unsafe fn get_bus_device_object(&self) -> BusDeviceObject {
        let hal_dev_ctx = (*self.bal_dev_ctx).hal_dev_ctx.cast::<nrf_wifi_hal_dev_ctx>();
        let fmac_dec_ctx = (*hal_dev_ctx).mac_dev_ctx.cast::<nrf_wifi_fmac_dev_ctx>();
        let os_dev_ctx = (*fmac_dec_ctx).os_dev_ctx.cast::<OsContext>();    
        (*os_dev_ctx).bus
    }
}

pub type BusDeviceObject = *mut dyn BusDevice;

pub trait BusDevice {
    fn read(&mut self, addr: u32, buf: &mut [u8]);
    fn write(&mut self, addr: u32, buf: &[u8]);
}

impl<T: SpiDevice> BusDevice for T {
    fn read(&mut self, addr: u32, buf: &mut [u8]) {
        self.transaction(&mut [
            Operation::Write(&[
                0x0B,
                (addr >> 16) as u8,
                (addr >> 8) as u8,
                addr as u8,
                0x00,
            ]),
            Operation::Read(buf),
        ])
        .unwrap()
    }

    fn write(&mut self, addr: u32, buf: &[u8]) {
        self.transaction(&mut [
            Operation::Write(&[
                0x02,
                (addr >> 16) as u8 | 0x80,
                (addr >> 8) as u8,
                addr as u8,
            ]),
            Operation::Write(buf),
        ])
        .unwrap()
    }
}
