#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(unused_must_use)]
#![allow(async_fn_in_trait)]

use core::ffi::c_void;
use core::ptr::addr_of_mut;

use crate::bus::BusDeviceObject;
use cortex_m_rt::exception;
use defmt_rtt as _; // global logger
use embassy_executor::Spawner;
use embassy_nrf::gpio::{AnyPin, Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_nrf::spim::Spim;
use embassy_nrf::{bind_interrupts, spim};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use nrf700x_sys::{nrf_wifi_fmac_fw_info, nrf_wifi_fw_info};

use {embassy_nrf as _, panic_probe as _};

mod alloc_impl;
mod bus;
mod os;

bind_interrupts!(struct Irqs {
    SERIAL0 => spim::InterruptHandler<embassy_nrf::peripherals::SERIAL0>;
});

#[embassy_executor::task]
async fn run_wifi_work_queue() -> ! {
    os::run_work_queue().await
}

#[embassy_executor::task]
async fn blink_task(led: AnyPin) -> ! {
    let mut led = Output::new(led, Level::High, OutputDrive::Standard);
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    alloc_impl::init_heap();

    defmt::info!("Hello World!");
    let config: embassy_nrf::config::Config = Default::default();
    let p = embassy_nrf::init(config);
    spawner.spawn(blink_task(p.P1_06.degrade())).unwrap();
    spawner.spawn(run_wifi_work_queue()).unwrap();

    let sck = p.P0_17;
    let csn = p.P0_18;
    let dio0 = p.P0_13;
    let dio1 = p.P0_14;
    let _dio2 = p.P0_15;
    let _dio3 = p.P0_16;
    //let coex_req = Output::new(p.P0_28, Level::High, OutputDrive::Standard);
    //let coex_status0 = Output::new(p.P0_30, Level::High, OutputDrive::Standard);
    //let coex_status1 = Output::new(p.P0_29, Level::High, OutputDrive::Standard);
    //let coex_grant = Output::new(p.P0_24, Level::High, OutputDrive::Standard);
    let _bucken = Output::new(p.P0_12.degrade(), Level::Low, OutputDrive::HighDrive);
    let _iovdd_ctl = Output::new(p.P0_31.degrade(), Level::Low, OutputDrive::Standard);
    let _host_irq = Input::new(p.P0_23.degrade(), Pull::None);

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M8;
    let spim = Spim::new(p.SERIAL0, Irqs, sck, dio1, dio0, config);
    let csn = Output::new(csn, Level::High, OutputDrive::HighDrive);
    let spi: BusDeviceObject =
        static_cell::make_static!(ExclusiveDevice::new(spim, csn, Delay));

    /*
    // QSPI is not working well yet.
    let mut config = qspi::Config::default();
    config.read_opcode = qspi::ReadOpcode::READ4IO;
    config.write_opcode = qspi::WriteOpcode::PP4IO;
    config.write_page_size = qspi::WritePageSize::_256BYTES;
    config.frequency = qspi::Frequency::M8; // NOTE: Waking RPU works reliably only with lowest frequency (8MHz)

    let irq = interrupt::take!(QSPI);
    let qspi: qspi::Qspi<_> = qspi::Qspi::new(p.QSPI, irq, sck, csn, dio0, dio1, dio2, dio3, config);
    let bus = QspiBus { qspi };
    */

    unsafe {
        let mut data_config = nrf700x_sys::nrf_wifi_data_config_params {
            rate_protection_type: 1,
            aggregation: nrf700x_sys::NRF_WIFI_FEATURE_DISABLE as _,
            wmm: nrf700x_sys::NRF_WIFI_FEATURE_DISABLE as _,
            max_num_tx_agg_sessions: 4,
            max_num_rx_agg_sessions: 4,
            max_tx_aggregation: 16,
            reorder_buf_size: 1,
            max_rxampdu_size: nrf700x_sys::max_rx_ampdu_size::MAX_RX_AMPDU_SIZE_8KB as _,
        };
        let mut rx_buf_pools: [nrf700x_sys::rx_buf_pool_params; 3] = [
            nrf700x_sys::rx_buf_pool_params {
                buf_sz: 2048,
                num_bufs: 2,
            },
            nrf700x_sys::rx_buf_pool_params {
                buf_sz: 2048,
                num_bufs: 2,
            },
            nrf700x_sys::rx_buf_pool_params {
                buf_sz: 2048,
                num_bufs: 2,
            },
        ];
        let mut callbk_fns = nrf700x_sys::nrf_wifi_fmac_callbk_fns {
            scan_start_callbk_fn: Some(scan_start_callbk_fn),
            scan_done_callbk_fn: Some(scan_done_callbk_fn),
            scan_abort_callbk_fn: Some(scan_abort_callbk_fn),
            scan_res_callbk_fn: Some(scan_res_callbk_fn),
            disp_scan_res_callbk_fn: Some(disp_scan_res_callbk_fn),
        };

        let fpriv = nrf700x_sys::nrf_wifi_fmac_init(
            addr_of_mut!(data_config),
            addr_of_mut!(rx_buf_pools[0]),
            addr_of_mut!(callbk_fns),
        );

        defmt::info!("fpriv: {}", fpriv);

        let mut os_ctx = OsContext { bus: spi };

        defmt::info!("os_ctx: {}", (&mut os_ctx) as *mut OsContext);

        let fmac_dev_ctx =
            nrf700x_sys::nrf_wifi_fmac_dev_add(fpriv, ((&mut os_ctx) as *mut OsContext).cast());

        defmt::info!("fmac_dev_ctx: {}", (&mut os_ctx) as *mut OsContext);

        let mut version = 0u32;
        if nrf700x_sys::nrf_wifi_fmac_ver_get(fmac_dev_ctx, &mut version as *mut _).failed() {
            defmt::error!("fmac version is error");
        } else {
            defmt::info!("fmac version is {}", version);
        }

        let mut fw_info = nrf_wifi_fmac_fw_info {
            lmac_patch_pri: nrf_wifi_fw_info {
                data: nrf700x_sys::nrf_wifi_lmac_patch_pri_bimg
                    .as_ptr()
                    .cast_mut()
                    .cast(),
                size: nrf700x_sys::nrf_wifi_lmac_patch_pri_bimg.len() as _,
            },
            lmac_patch_sec: nrf_wifi_fw_info {
                data: nrf700x_sys::nrf_wifi_lmac_patch_sec_bin
                    .as_ptr()
                    .cast_mut()
                    .cast(),
                size: nrf700x_sys::nrf_wifi_lmac_patch_sec_bin.len() as _,
            },
            umac_patch_pri: nrf_wifi_fw_info {
                data: nrf700x_sys::nrf_wifi_umac_patch_pri_bimg
                    .as_ptr()
                    .cast_mut()
                    .cast(),
                size: nrf700x_sys::nrf_wifi_umac_patch_pri_bimg.len() as _,
            },
            umac_patch_sec: nrf_wifi_fw_info {
                data: nrf700x_sys::nrf_wifi_umac_patch_sec_bin
                    .as_ptr()
                    .cast_mut()
                    .cast(),
                size: nrf700x_sys::nrf_wifi_umac_patch_sec_bin.len() as _,
            },
        };

        if nrf700x_sys::nrf_wifi_fmac_fw_load(fmac_dev_ctx, &mut fw_info as *mut _).failed() {
            defmt::error!("Failed to load fw patches!");
        }

        let mut version = 0u32;
        if nrf700x_sys::nrf_wifi_fmac_ver_get(fmac_dev_ctx, &mut version as *mut _).failed() {
            defmt::error!("fmac version is error");
        } else {
            defmt::info!("fmac version is {}", version);
        }

    }
}

struct OsContext {
    bus: BusDeviceObject,
}

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    defmt::error!("Hardfault!: {}", defmt::Debug2Format(&ef));
    loop {
        cortex_m::asm::bkpt();
    }
}

unsafe extern "C" fn scan_start_callbk_fn(
    _os_vif_ctx: *mut c_void,
    _scan_start_event: *mut nrf700x_sys::nrf_wifi_umac_event_trigger_scan,
    _event_len: u32,
) {
    defmt::info!("scan_start_callbk_fn");
}

unsafe extern "C" fn scan_done_callbk_fn(
    _os_vif_ctx: *mut c_void,
    _scan_done_event: *mut nrf700x_sys::nrf_wifi_umac_event_trigger_scan,
    _event_len: u32,
) {
    defmt::info!("scan_done_callbk_fn");
}

/** Callback function to be called when a scan is aborted. */
unsafe extern "C" fn scan_abort_callbk_fn(
    _os_vif_ctx: *mut c_void,
    _scan_done_event: *mut nrf700x_sys::nrf_wifi_umac_event_trigger_scan,
    _event_len: u32,
) {
    defmt::info!("scan_abort_callbk_fn");
}

/** Callback function to be called when a scan result is received. */
unsafe extern "C" fn scan_res_callbk_fn(
    _os_vif_ctx: *mut c_void,
    _scan_res: *mut nrf700x_sys::nrf_wifi_umac_event_new_scan_results,
    _event_len: u32,
    _more_res: bool,
) {
    defmt::info!("scan_res_callbk_fn");
}

/** Callback function to be called when a display scan result is received. */
unsafe extern "C" fn disp_scan_res_callbk_fn(
    _os_vif_ctx: *mut c_void,
    _scan_res: *mut nrf700x_sys::nrf_wifi_umac_event_new_scan_display_results,
    _event_len: u32,
    _more_res: bool,
) {
    defmt::info!("disp_scan_res_callbk_fn");
}
