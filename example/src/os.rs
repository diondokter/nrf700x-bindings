#![allow(unused_variables)]

extern crate alloc;

use core::{
    ffi::c_void,
    mem::{size_of, size_of_val},
    ptr::null_mut,
    sync::atomic::{AtomicBool, AtomicU16},
};

use alloc::{
    boxed::Box,
    vec::{self, Vec},
};

use defmt::{info, trace};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use futures::FutureExt;
use nrf700x_sys::{
    nrf_wifi_assert_op_type, nrf_wifi_bal_dev_ctx, nrf_wifi_bus_spi_dev_ctx, nrf_wifi_fmac_dev_ctx,
    nrf_wifi_hal_dev_ctx, nrf_wifi_osal_dma_dir, nrf_wifi_osal_host_map, nrf_wifi_osal_ops,
    nrf_wifi_status, nrf_wifi_tasklet_type,
};

use crate::{bus::BusDeviceObject, OsContext};

#[no_mangle]
extern "C" fn get_os_ops() -> *const nrf_wifi_osal_ops {
    info!("Getting OS ops");

    static OS_OPS: nrf_wifi_osal_ops = nrf_wifi_osal_ops {
        mem_alloc: Some(mem_alloc),
        mem_zalloc: Some(mem_zalloc),
        mem_free: Some(mem_free),
        mem_cpy: Some(mem_cpy),
        mem_set: Some(mem_set),
        mem_cmp: Some(mem_cmp),
        iomem_mmap: Some(iomem_mmap),
        iomem_unmap: Some(iomem_unmap),
        iomem_read_reg32: Some(iomem_read_reg32),
        iomem_write_reg32: Some(iomem_write_reg32),
        iomem_cpy_from: Some(iomem_cpy_from),
        iomem_cpy_to: Some(iomem_cpy_to),
        qspi_read_reg32: Some(qspi_read_reg32),
        qspi_write_reg32: Some(qspi_write_reg32),
        qspi_cpy_from: Some(qspi_cpy_from),
        qspi_cpy_to: Some(qspi_cpy_to),
        spi_read_reg32: Some(spi_read_reg32),
        spi_write_reg32: Some(spi_write_reg32),
        spi_cpy_from: Some(spi_cpy_from),
        spi_cpy_to: Some(spi_cpy_to),
        spinlock_alloc: Some(spinlock_alloc),
        spinlock_free: Some(spinlock_free),
        spinlock_init: Some(spinlock_init),
        spinlock_take: Some(spinlock_take),
        spinlock_rel: Some(spinlock_rel),
        spinlock_irq_take: Some(spinlock_irq_take),
        spinlock_irq_rel: Some(spinlock_irq_rel),
        log_dbg: Some(c_log_dbg),
        log_info: Some(c_log_info),
        log_err: Some(c_log_err),
        llist_node_alloc: Some(llist_node_alloc),
        llist_node_free: Some(llist_node_free),
        llist_node_data_get: Some(llist_node_data_get),
        llist_node_data_set: Some(llist_node_data_set),
        llist_alloc: Some(llist_alloc),
        llist_free: Some(llist_free),
        llist_init: Some(llist_init),
        llist_add_node_tail: Some(llist_add_node_tail),
        llist_add_node_head: Some(llist_add_node_head),
        llist_get_node_head: Some(llist_get_node_head),
        llist_get_node_nxt: Some(llist_get_node_nxt),
        llist_del_node: Some(llist_del_node),
        llist_len: Some(llist_len),
        nbuf_alloc: Some(nbuf_alloc),
        nbuf_free: Some(nbuf_free),
        nbuf_headroom_res: Some(nbuf_headroom_res),
        nbuf_headroom_get: Some(nbuf_headroom_get),
        nbuf_data_size: Some(nbuf_data_size),
        nbuf_data_get: Some(nbuf_data_get),
        nbuf_data_put: Some(nbuf_data_put),
        nbuf_data_push: Some(nbuf_data_push),
        nbuf_data_pull: Some(nbuf_data_pull),
        nbuf_get_priority: Some(nbuf_get_priority),
        tasklet_alloc: Some(tasklet_alloc),
        tasklet_free: Some(tasklet_free),
        tasklet_init: Some(tasklet_init),
        tasklet_schedule: Some(tasklet_schedule),
        tasklet_kill: Some(tasklet_kill),
        sleep_ms: Some(sleep_ms),
        delay_us: Some(delay_us),
        time_get_curr_us: Some(time_get_curr_us),
        time_elapsed_us: Some(time_elapsed_us),
        bus_pcie_init: Some(bus_pcie_init),
        bus_pcie_deinit: Some(bus_pcie_deinit),
        bus_pcie_dev_add: Some(bus_pcie_dev_add),
        bus_pcie_dev_rem: Some(bus_pcie_dev_rem),
        bus_pcie_dev_init: Some(bus_pcie_dev_init),
        bus_pcie_dev_deinit: Some(bus_pcie_dev_deinit),
        bus_pcie_dev_intr_reg: Some(bus_pcie_dev_intr_reg),
        bus_pcie_dev_intr_unreg: Some(bus_pcie_dev_intr_unreg),
        bus_pcie_dev_dma_map: Some(bus_pcie_dev_dma_map),
        bus_pcie_dev_dma_unmap: Some(bus_pcie_dev_dma_unmap),
        bus_pcie_dev_host_map_get: Some(bus_pcie_dev_host_map_get),
        bus_qspi_init: Some(bus_qspi_init),
        bus_qspi_deinit: Some(bus_qspi_deinit),
        bus_qspi_dev_add: Some(bus_qspi_dev_add),
        bus_qspi_dev_rem: Some(bus_qspi_dev_rem),
        bus_qspi_dev_init: Some(bus_qspi_dev_init),
        bus_qspi_dev_deinit: Some(bus_qspi_dev_deinit),
        bus_qspi_dev_intr_reg: Some(bus_qspi_dev_intr_reg),
        bus_qspi_dev_intr_unreg: Some(bus_qspi_dev_intr_unreg),
        bus_qspi_dev_host_map_get: Some(bus_qspi_dev_host_map_get),
        bus_spi_init: Some(bus_spi_init),
        bus_spi_deinit: Some(bus_spi_deinit),
        bus_spi_dev_add: Some(bus_spi_dev_add),
        bus_spi_dev_rem: Some(bus_spi_dev_rem),
        bus_spi_dev_init: Some(bus_spi_dev_init),
        bus_spi_dev_deinit: Some(bus_spi_dev_deinit),
        bus_spi_dev_intr_reg: Some(bus_spi_dev_intr_reg),
        bus_spi_dev_intr_unreg: Some(bus_spi_dev_intr_unreg),
        bus_spi_dev_host_map_get: Some(bus_spi_dev_host_map_get),
        assert: Some(assert),
        strlen: Some(strlen),
    };

    &OS_OPS
}

unsafe extern "C" fn mem_alloc(size: usize) -> *mut core::ffi::c_void {
    tinyrlibc::malloc(size).cast()
}
unsafe extern "C" fn mem_zalloc(size: usize) -> *mut core::ffi::c_void {
    tinyrlibc::calloc(1, size).cast()
}
unsafe extern "C" fn mem_free(buf: *mut core::ffi::c_void) {
    tinyrlibc::free(buf.cast());
}
unsafe extern "C" fn mem_cpy(
    dest: *mut core::ffi::c_void,
    src: *const core::ffi::c_void,
    count: usize,
) -> *mut core::ffi::c_void {
    // trace!("Called OS mem_cpy");
    src.copy_to(dest, count);
    dest
}
unsafe extern "C" fn mem_set(
    start: *mut core::ffi::c_void,
    val: core::ffi::c_int,
    size: usize,
) -> *mut core::ffi::c_void {
    // trace!("Called OS mem_set");
    start.write_bytes(val as u8, size);
    start
}
unsafe extern "C" fn mem_cmp(
    addr1: *const core::ffi::c_void,
    addr2: *const core::ffi::c_void,
    size: usize,
) -> core::ffi::c_int {
    trace!("Called OS mem_cmp");

    for i in 0..size {
        match addr1
            .cast::<u8>()
            .add(i)
            .read()
            .cmp(&addr2.cast::<u8>().add(i).read())
        {
            core::cmp::Ordering::Less => return -1,
            core::cmp::Ordering::Equal => continue,
            core::cmp::Ordering::Greater => return 1,
        }
    }

    0
}
unsafe extern "C" fn iomem_mmap(
    addr: core::ffi::c_ulong,
    size: core::ffi::c_ulong,
) -> *mut core::ffi::c_void {
    trace!("Called OS iomem_mmap");
    todo!();
}
unsafe extern "C" fn iomem_unmap(addr: *mut core::ffi::c_void) {
    trace!("Called OS iomem_unmap");
    todo!();
}
unsafe extern "C" fn iomem_read_reg32(addr: *const core::ffi::c_void) -> core::ffi::c_uint {
    trace!("Called OS iomem_read_reg32");
    todo!();
}
unsafe extern "C" fn iomem_write_reg32(addr: *mut core::ffi::c_void, val: core::ffi::c_uint) {
    trace!("Called OS iomem_write_reg32");
    todo!();
}
unsafe extern "C" fn iomem_cpy_from(
    dest: *mut core::ffi::c_void,
    src: *const core::ffi::c_void,
    count: usize,
) {
    trace!("Called OS iomem_cpy_from");
    todo!();
}
unsafe extern "C" fn iomem_cpy_to(
    dest: *mut core::ffi::c_void,
    src: *const core::ffi::c_void,
    count: usize,
) {
    trace!("Called OS iomem_cpy_to");
    todo!();
}
unsafe extern "C" fn qspi_read_reg32(
    priv_: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
) -> core::ffi::c_uint {
    trace!("Called OS qspi_read_reg32");
    todo!();
}
unsafe extern "C" fn qspi_write_reg32(
    priv_: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    val: core::ffi::c_uint,
) {
    trace!("Called OS qspi_write_reg32");
    todo!();
}
unsafe extern "C" fn qspi_cpy_from(
    priv_: *mut core::ffi::c_void,
    dest: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    count: usize,
) {
    trace!("Called OS qspi_cpy_from");
    todo!();
}
unsafe extern "C" fn qspi_cpy_to(
    priv_: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    src: *const core::ffi::c_void,
    count: usize,
) {
    trace!("Called OS qspi_cpy_to");
    todo!();
}
unsafe extern "C" fn spi_read_reg32(
    priv_: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
) -> core::ffi::c_uint {
    let mut buffer = [0; 4];
    (*(*priv_.cast::<SpiDevice>()).device_object).read(addr, &mut buffer);
    let value = u32::from_le_bytes(buffer);
    trace!(
        "Called OS spi_read_reg32: addr={:08X} value={:08X}",
        addr,
        value
    );

    value
}
unsafe extern "C" fn spi_write_reg32(
    priv_: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    val: core::ffi::c_uint,
) {
    trace!(
        "Called OS spi_write_reg32: addr={:08X} value={:08X}",
        addr,
        val
    );
    (*(*priv_.cast::<SpiDevice>()).device_object).write(addr, &val.to_le_bytes());
}
unsafe extern "C" fn spi_cpy_from(
    priv_: *mut core::ffi::c_void,
    dest: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    count: usize,
) {
    (*(*priv_.cast::<SpiDevice>()).device_object).read(
        addr as u32,
        core::slice::from_raw_parts_mut(dest.cast(), count),
    );

    defmt::flush();
    trace!(
        "Called OS spi_cpy_from {:08X} len={} buf={:02X}",
        addr,
        count,
        core::slice::from_raw_parts_mut(dest.cast::<u8>(), count)
    );
    defmt::flush();
}
unsafe extern "C" fn spi_cpy_to(
    priv_: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    src: *const core::ffi::c_void,
    count: usize,
) {
    defmt::flush();
    trace!(
        "Called OS spi_cpy_to {:08X} len={} buf={:02X})",
        addr,
        count,
        core::slice::from_raw_parts(src.cast::<u8>(), count)
    );
    defmt::flush();

    (*(*priv_.cast::<SpiDevice>()).device_object)
        .write(addr as u32, core::slice::from_raw_parts(src.cast(), count));
}
unsafe extern "C" fn spinlock_alloc() -> *mut core::ffi::c_void {
    // trace!("Called OS spinlock_alloc");
    Box::into_raw(Box::new(AtomicBool::new(false))).cast()
}
unsafe extern "C" fn spinlock_free(lock: *mut core::ffi::c_void) {
    trace!("Called OS spinlock_free");
    drop(Box::<AtomicBool>::from_raw(lock.cast()));
}
unsafe extern "C" fn spinlock_init(lock: *mut core::ffi::c_void) {
    // trace!("Called OS spinlock_init");
    (*lock.cast::<AtomicBool>()).store(false, core::sync::atomic::Ordering::SeqCst);
}
unsafe extern "C" fn spinlock_take(lock: *mut core::ffi::c_void) {
    trace!("Called OS spinlock_take");
    while !(*lock.cast::<AtomicBool>()).swap(true, core::sync::atomic::Ordering::SeqCst) {
        core::hint::spin_loop();
    }
}
unsafe extern "C" fn spinlock_rel(lock: *mut core::ffi::c_void) {
    trace!("Called OS spinlock_rel");
    (*lock.cast::<AtomicBool>()).store(false, core::sync::atomic::Ordering::SeqCst);
}
unsafe extern "C" fn spinlock_irq_take(
    lock: *mut core::ffi::c_void,
    flags: *mut core::ffi::c_ulong,
) {
    trace!("Called OS spinlock_irq_take");
    let restore_state = critical_section::acquire();
    assert!(size_of_val(&restore_state) <= size_of::<core::ffi::c_ulong>());
    *(flags.cast()) = restore_state;

    spinlock_take(lock);
}
unsafe extern "C" fn spinlock_irq_rel(
    lock: *mut core::ffi::c_void,
    flags: *mut core::ffi::c_ulong,
) {
    trace!("Called OS spinlock_irq_rel");
    spinlock_rel(lock);
    critical_section::release(*(flags.cast()));
}

#[no_mangle]
unsafe extern "C" fn rust_log_dbg(log: *const core::ffi::c_char, len: usize) {
    defmt::flush();
    defmt::debug!(
        "Lib log:\n--- {}",
        core::str::from_utf8(core::slice::from_raw_parts(log.cast::<u8>(), len))
            .unwrap()
            .trim_end()
    );
    defmt::flush();
}
#[no_mangle]
unsafe extern "C" fn rust_log_info(log: *const core::ffi::c_char, len: usize) {
    defmt::flush();
    defmt::info!(
        "Lib log:\n--- {}",
        core::str::from_utf8(core::slice::from_raw_parts(log.cast::<u8>(), len))
            .unwrap()
            .trim_end()
    );
    defmt::flush();
}
#[no_mangle]
unsafe extern "C" fn rust_log_err(log: *const core::ffi::c_char, len: usize) {
    defmt::flush();
    defmt::error!(
        "Lib log:\n--- {}",
        core::str::from_utf8(core::slice::from_raw_parts(log.cast::<u8>(), len))
            .unwrap()
            .trim_end()
    );
    defmt::flush();
}
unsafe extern "C" fn llist_node_alloc() -> *mut core::ffi::c_void {
    trace!("Called OS llist_node_alloc");
    Box::into_raw(Box::new(LinkedListNode {
        next: null_mut(),
        data: null_mut(),
    }))
    .cast()
}
unsafe extern "C" fn llist_node_free(node: *mut core::ffi::c_void) {
    trace!("Called OS llist_node_free");
    drop(Box::from_raw(node.cast::<LinkedListNode>()));
}
unsafe extern "C" fn llist_node_data_get(node: *mut core::ffi::c_void) -> *mut core::ffi::c_void {
    trace!("Called OS llist_node_data_get");
    (*node.cast::<LinkedListNode>()).data
}
unsafe extern "C" fn llist_node_data_set(
    node: *mut core::ffi::c_void,
    data: *mut core::ffi::c_void,
) {
    trace!("Called OS llist_node_data_set");
    (*node.cast::<LinkedListNode>()).data = data;
}
unsafe extern "C" fn llist_alloc() -> *mut core::ffi::c_void {
    // trace!("Called OS llist_alloc");
    Box::into_raw(Box::new(LinkedList {
        head: null_mut(),
        tail: null_mut(),
        len: 0,
    }))
    .cast()
}
unsafe extern "C" fn llist_free(llist: *mut core::ffi::c_void) {
    trace!("Called OS llist_free");
    drop(Box::from_raw(llist.cast::<LinkedList>()));
}
unsafe extern "C" fn llist_init(llist: *mut core::ffi::c_void) {
    // trace!("Called OS llist_init");
    *llist.cast::<LinkedList>() = LinkedList {
        head: null_mut(),
        tail: null_mut(),
        len: 0,
    };
}
unsafe extern "C" fn llist_add_node_tail(
    llist: *mut core::ffi::c_void,
    llist_node: *mut core::ffi::c_void,
) {
    trace!("Called OS llist_add_node_tail");

    if (*llist.cast::<LinkedList>()).head.is_null() {
        llist_add_node_head(llist, llist_node);
        return;
    }

    (*(*llist.cast::<LinkedList>()).tail).next = llist_node.cast();
    (*llist.cast::<LinkedList>()).tail = llist_node.cast();
    (*llist.cast::<LinkedList>()).len += 1;
}
unsafe extern "C" fn llist_add_node_head(
    llist: *mut core::ffi::c_void,
    llist_node: *mut core::ffi::c_void,
) {
    trace!("Called OS llist_add_node_head");

    (*llist_node.cast::<LinkedListNode>()).next = (*llist.cast::<LinkedList>()).head.cast();
    (*llist.cast::<LinkedList>()).head = llist_node.cast();
    (*llist.cast::<LinkedList>()).len += 1;
}
unsafe extern "C" fn llist_get_node_head(llist: *mut core::ffi::c_void) -> *mut core::ffi::c_void {
    trace!("Called OS llist_get_node_head");
    (*llist.cast::<LinkedList>()).head.cast()
}
unsafe extern "C" fn llist_get_node_nxt(
    llist: *mut core::ffi::c_void,
    llist_node: *mut core::ffi::c_void,
) -> *mut core::ffi::c_void {
    trace!("Called OS llist_get_node_nxt");
    (*llist_node.cast::<LinkedListNode>()).next.cast()
}
unsafe extern "C" fn llist_del_node(
    llist: *mut core::ffi::c_void,
    llist_node: *mut core::ffi::c_void,
) {
    trace!("Called OS llist_del_node");
    let mut current = llist_get_node_head(llist);

    if current.is_null() {
        unreachable!()
    }

    if llist_get_node_head(llist) == llist_node {
        (*llist.cast::<LinkedList>()).head = llist_get_node_nxt(llist, llist_node).cast();
        if (*llist.cast::<LinkedList>()).head.is_null() {
            (*llist.cast::<LinkedList>()).tail = null_mut();
        }
        return;
    }

    loop {
        let next = llist_get_node_nxt(llist, current);

        if next.is_null() {
            unreachable!()
        }

        if next == llist_node {
            let next_after = llist_get_node_nxt(llist, next);
            (*current.cast::<LinkedListNode>()).next = next_after.cast();
            (*llist.cast::<Option<LinkedList>>()).as_mut().unwrap().len -= 1;

            if (*llist.cast::<LinkedList>()).tail == llist_node.cast() {
                (*llist.cast::<LinkedList>()).tail = current.cast();
            }

            return;
        }

        current = next;
    }
}
unsafe extern "C" fn llist_len(llist: *mut core::ffi::c_void) -> core::ffi::c_uint {
    trace!("Called OS llist_len");
    (*llist.cast::<Option<LinkedList>>()).as_mut().unwrap().len as _
}

/// Allocate a network buffer of size @size.
unsafe extern "C" fn nbuf_alloc(size: core::ffi::c_uint) -> *mut core::ffi::c_void {
    trace!("Called OS nbuf_alloc");
    Box::into_raw(Box::new(NetworkBuffer::new(size as usize))).cast()
}
/// Free a network buffer(@nbuf) which was allocated by @nbuf_alloc.
unsafe extern "C" fn nbuf_free(nbuf: *mut core::ffi::c_void) {
    trace!("Called OS nbuf_free");
    drop(Box::<NetworkBuffer>::from_raw(nbuf.cast()));
}
/// Reserve headroom at the beginning of the data area of a network buffer(@nbuf).
unsafe extern "C" fn nbuf_headroom_res(nbuf: *mut core::ffi::c_void, size: core::ffi::c_uint) {
    trace!("Called OS nbuf_headroom_res");
    (*nbuf.cast::<NetworkBuffer>()).reserve_headroom(size as usize);
}
/// Get the size of the reserved headroom at the beginning of the data area of a network buffer(@nbuf).
unsafe extern "C" fn nbuf_headroom_get(nbuf: *mut core::ffi::c_void) -> core::ffi::c_uint {
    trace!("Called OS nbuf_headroom_get");
    (*nbuf.cast::<NetworkBuffer>()).headroom() as _
}
/// Get the size of the data area of a network buffer(@nbuf).
unsafe extern "C" fn nbuf_data_size(nbuf: *mut core::ffi::c_void) -> core::ffi::c_uint {
    trace!("Called OS nbuf_data_size");
    (*nbuf.cast::<NetworkBuffer>()).len() as _
}
// Get the pointer to the data area of a network buffer(@nbuf).
unsafe extern "C" fn nbuf_data_get(nbuf: *mut core::ffi::c_void) -> *mut core::ffi::c_void {
    trace!("Called OS nbuf_data_get");
    (*nbuf.cast::<NetworkBuffer>())
        .data_mut()
        .as_mut_ptr()
        .cast()
}
/// Increase the data area of a network buffer(@nbuf) by @size bytes at the end of the area and return the pointer to
/// the beginning of the data area.
unsafe extern "C" fn nbuf_data_put(
    nbuf: *mut core::ffi::c_void,
    size: core::ffi::c_uint,
) -> *mut core::ffi::c_void {
    trace!("Called OS nbuf_data_put");
    (*nbuf.cast::<NetworkBuffer>()).put(size as usize);
    nbuf_data_get(nbuf)
}
/// Increase the data area of a network buffer(@nbuf) by @size bytes at the start of the area and return the pointer to
/// the beginning of the data area.
unsafe extern "C" fn nbuf_data_push(
    nbuf: *mut core::ffi::c_void,
    size: core::ffi::c_uint,
) -> *mut core::ffi::c_void {
    (*nbuf.cast::<NetworkBuffer>()).push(size as usize);
    nbuf_data_get(nbuf)
}
/// Decrease the data area of a network buffer(@nbuf) by @size bytes at the start of the area and
/// return the pointer to the beginning of the data area.
unsafe extern "C" fn nbuf_data_pull(
    nbuf: *mut core::ffi::c_void,
    size: core::ffi::c_uint,
) -> *mut core::ffi::c_void {
    (*nbuf.cast::<NetworkBuffer>()).pull(size as usize);
    nbuf_data_get(nbuf)
}
/// Get the priority of a network buffer(@nbuf).
unsafe extern "C" fn nbuf_get_priority(nbuf: *mut core::ffi::c_void) -> core::ffi::c_uchar {
    trace!("Called OS nbuf_get_priority");
    0
}

unsafe extern "C" fn tasklet_alloc(type_: core::ffi::c_int) -> *mut core::ffi::c_void {
    // trace!("Called OS tasklet_alloc");

    static NEXT_INDEX: AtomicU16 = AtomicU16::new(0);

    let type_ = match type_ {
        0 => nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_BH,
        1 => nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_IRQ,
        2 => nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_TX_DONE,
        3 => nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_RX,
        _ => unreachable!(),
    };

    let ptr = mem_alloc(size_of::<Tasklet>());
    ptr.cast::<Tasklet>().write(Tasklet {
        tasklet_type: type_,
        callback: None,
        data: 0,
        index: NEXT_INDEX.fetch_add(1, core::sync::atomic::Ordering::SeqCst),
    });

    ptr
}
unsafe extern "C" fn tasklet_free(tasklet: *mut core::ffi::c_void) {
    trace!("Called OS tasklet_free");
    mem_free(tasklet);
}
unsafe extern "C" fn tasklet_init(
    tasklet: *mut core::ffi::c_void,
    callback: core::option::Option<unsafe extern "C" fn(arg1: core::ffi::c_ulong)>,
    data: core::ffi::c_ulong,
) {
    trace!("Called OS tasklet_init");
    (*tasklet.cast::<Tasklet>()).callback = callback;
    (*tasklet.cast::<Tasklet>()).data = data;
}
unsafe extern "C" fn tasklet_schedule(tasklet: *mut core::ffi::c_void) {
    trace!("Called OS tasklet_schedule");
    match (*tasklet.cast::<Tasklet>()).tasklet_type {
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_BH => WORK_QUEUE_BH
            .try_send((
                (*tasklet.cast::<Tasklet>()).callback.unwrap(),
                (*tasklet.cast::<Tasklet>()).data,
                (*tasklet.cast::<Tasklet>()).index,
            ))
            .expect("WORK_QUEUE_BH has space left"),
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_IRQ => WORK_QUEUE_IRQ
            .try_send((
                (*tasklet.cast::<Tasklet>()).callback.unwrap(),
                (*tasklet.cast::<Tasklet>()).data,
                (*tasklet.cast::<Tasklet>()).index,
            ))
            .expect("WORK_QUEUE_IRQ has space left"),
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_TX_DONE => WORK_QUEUE_TX_DONE
            .try_send((
                (*tasklet.cast::<Tasklet>()).callback.unwrap(),
                (*tasklet.cast::<Tasklet>()).data,
                (*tasklet.cast::<Tasklet>()).index,
            ))
            .expect("WORK_QUEUE_TX_DONE has space left"),
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_RX => WORK_QUEUE_RX
            .try_send((
                (*tasklet.cast::<Tasklet>()).callback.unwrap(),
                (*tasklet.cast::<Tasklet>()).data,
                (*tasklet.cast::<Tasklet>()).index,
            ))
            .expect("WORK_QUEUE_RX has space left"),
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_MAX => unreachable!(),
    }
}
unsafe extern "C" fn tasklet_kill(tasklet: *mut core::ffi::c_void) {
    trace!("Called OS tasklet_kill");

    fn remove_task_from_queue(
        queue: &Channel<CriticalSectionRawMutex, (unsafe extern "C" fn(u32), u32, u16), 16>,
        current_task_index: u16,
    ) {
        let mut first_seen_index = None;

        loop {
            match queue.try_receive() {
                Ok(item) if first_seen_index == Some(item.2) => break,
                Ok(item) => {
                    if first_seen_index.is_none() {
                        first_seen_index = Some(item.2);
                    }

                    if item.2 == current_task_index {
                        break;
                    }

                    queue.try_send(item).expect("WORK_QUEUE_BH has space left");
                }
                Err(_) => break,
            }
        }
    }

    let current_task_index = (*tasklet.cast::<Tasklet>()).index;

    match (*tasklet.cast::<Tasklet>()).tasklet_type {
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_BH => {
            remove_task_from_queue(&WORK_QUEUE_BH, current_task_index)
        }
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_IRQ => {
            remove_task_from_queue(&WORK_QUEUE_IRQ, current_task_index)
        }
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_TX_DONE => {
            remove_task_from_queue(&WORK_QUEUE_TX_DONE, current_task_index)
        }
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_RX => {
            remove_task_from_queue(&WORK_QUEUE_RX, current_task_index)
        }
        nrf_wifi_tasklet_type::NRF_WIFI_TASKLET_TYPE_MAX => unreachable!(),
    }
}

unsafe extern "C" fn sleep_ms(msecs: core::ffi::c_int) -> core::ffi::c_int {
    // trace!("Called OS sleep_ms");

    embassy_time::block_for(embassy_time::Duration::from_millis(msecs as _));

    0
}
unsafe extern "C" fn delay_us(usecs: core::ffi::c_int) -> core::ffi::c_int {
    // trace!("Called OS delay_us");

    embassy_time::block_for(embassy_time::Duration::from_micros(usecs as _));

    0
}
unsafe extern "C" fn time_get_curr_us() -> core::ffi::c_ulong {
    // trace!("Called OS time_get_curr_us");
    embassy_time::Instant::now().as_micros() as _
}
unsafe extern "C" fn time_elapsed_us(start_time_us: core::ffi::c_ulong) -> core::ffi::c_uint {
    // trace!("Called OS time_elapsed_us");
    sleep_ms(10);
    time_get_curr_us() - start_time_us
}
unsafe extern "C" fn bus_pcie_init(
    dev_name: *const core::ffi::c_char,
    vendor_id: core::ffi::c_uint,
    sub_vendor_id: core::ffi::c_uint,
    device_id: core::ffi::c_uint,
    sub_device_id: core::ffi::c_uint,
) -> *mut core::ffi::c_void {
    trace!("Called OS bus_pcie_init");
    todo!();
}
unsafe extern "C" fn bus_pcie_deinit(os_pcie_priv: *mut core::ffi::c_void) {
    trace!("Called OS bus_pcie_deinit");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_add(
    pcie_priv: *mut core::ffi::c_void,
    osal_pcie_dev_ctx: *mut core::ffi::c_void,
) -> *mut core::ffi::c_void {
    trace!("Called OS bus_pcie_dev_add");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_rem(os_pcie_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_pcie_dev_rem");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_init(os_pcie_dev_ctx: *mut core::ffi::c_void) -> nrf_wifi_status {
    trace!("Called OS bus_pcie_dev_init");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_deinit(os_pcie_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_pcie_dev_deinit");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_intr_reg(
    os_pcie_dev_ctx: *mut core::ffi::c_void,
    callbk_data: *mut core::ffi::c_void,
    callback_fn: core::option::Option<
        unsafe extern "C" fn(callbk_data: *mut core::ffi::c_void) -> core::ffi::c_int,
    >,
) -> nrf_wifi_status {
    trace!("Called OS bus_pcie_dev_intr_reg");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_intr_unreg(os_pcie_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_pcie_dev_intr_unreg");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_dma_map(
    os_pcie_dev_ctx: *mut core::ffi::c_void,
    virt_addr: *mut core::ffi::c_void,
    size: usize,
    dir: nrf_wifi_osal_dma_dir,
) -> *mut core::ffi::c_void {
    trace!("Called OS bus_pcie_dev_dma_map");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_dma_unmap(
    os_pcie_dev_ctx: *mut core::ffi::c_void,
    dma_addr: *mut core::ffi::c_void,
    size: usize,
    dir: nrf_wifi_osal_dma_dir,
) {
    trace!("Called OS bus_pcie_dev_dma_unmap");
    todo!();
}
unsafe extern "C" fn bus_pcie_dev_host_map_get(
    os_pcie_dev_ctx: *mut core::ffi::c_void,
    host_map: *mut nrf_wifi_osal_host_map,
) {
    trace!("Called OS bus_pcie_dev_host_map_get");
    todo!();
}
unsafe extern "C" fn bus_qspi_init() -> *mut core::ffi::c_void {
    trace!("Called OS bus_qspi_init");
    todo!();
}
unsafe extern "C" fn bus_qspi_deinit(os_qspi_priv: *mut core::ffi::c_void) {
    trace!("Called OS bus_qspi_deinit");
    todo!();
}
unsafe extern "C" fn bus_qspi_dev_add(
    qspi_priv: *mut core::ffi::c_void,
    osal_qspi_dev_ctx: *mut core::ffi::c_void,
) -> *mut core::ffi::c_void {
    trace!("Called OS bus_qspi_dev_add");
    todo!();
}
unsafe extern "C" fn bus_qspi_dev_rem(os_qspi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_qspi_dev_rem");
    todo!();
}
unsafe extern "C" fn bus_qspi_dev_init(os_qspi_dev_ctx: *mut core::ffi::c_void) -> nrf_wifi_status {
    trace!("Called OS bus_qspi_dev_init");
    todo!();
}
unsafe extern "C" fn bus_qspi_dev_deinit(os_qspi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_qspi_dev_deinit");
    todo!();
}
unsafe extern "C" fn bus_qspi_dev_intr_reg(
    os_qspi_dev_ctx: *mut core::ffi::c_void,
    callbk_data: *mut core::ffi::c_void,
    callback_fn: core::option::Option<
        unsafe extern "C" fn(callbk_data: *mut core::ffi::c_void) -> core::ffi::c_int,
    >,
) -> nrf_wifi_status {
    trace!("Called OS bus_qspi_dev_intr_reg");
    todo!();
}
unsafe extern "C" fn bus_qspi_dev_intr_unreg(os_qspi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_qspi_dev_intr_unreg");
    todo!();
}
unsafe extern "C" fn bus_qspi_dev_host_map_get(
    os_qspi_dev_ctx: *mut core::ffi::c_void,
    host_map: *mut nrf_wifi_osal_host_map,
) {
    trace!("Called OS bus_qspi_dev_host_map_get");
    todo!();
}

struct SpiBus {
    dummy: u32,
}

struct SpiDevice {
    device_object: BusDeviceObject,
    os_context: *mut OsContext,
}

unsafe fn get_os_context(ptr: *mut nrf_wifi_bus_spi_dev_ctx) -> *mut OsContext {
    let bal_dev_ctx = (*ptr).bal_dev_ctx.cast::<nrf_wifi_bal_dev_ctx>();
    let hal_dev_ctx = (*bal_dev_ctx).hal_dev_ctx.cast::<nrf_wifi_hal_dev_ctx>();
    let fmac_dec_ctx = (*hal_dev_ctx).mac_dev_ctx.cast::<nrf_wifi_fmac_dev_ctx>();
    (*fmac_dec_ctx).os_dev_ctx.cast::<OsContext>()
}

unsafe extern "C" fn bus_spi_init() -> *mut core::ffi::c_void {
    trace!("Called OS bus_spi_init");
    Box::into_raw(Box::new(SpiBus { dummy: 0 })).cast()
}
unsafe extern "C" fn bus_spi_deinit(os_spi_priv: *mut core::ffi::c_void) {
    trace!("Called OS bus_spi_deinit");
    drop(Box::from_raw(os_spi_priv));
}
unsafe extern "C" fn bus_spi_dev_add(
    spi_priv: *mut core::ffi::c_void,
    osal_spi_dev_ctx: *mut core::ffi::c_void,
) -> *mut core::ffi::c_void {
    trace!("Called OS bus_spi_dev_add: {}", osal_spi_dev_ctx);

    let _spi_priv = spi_priv.cast::<SpiBus>();
    let osal_spi_dev_ctx = osal_spi_dev_ctx.cast::<nrf_wifi_bus_spi_dev_ctx>();

    Box::into_raw(Box::new(SpiDevice {
        device_object: (*get_os_context(osal_spi_dev_ctx)).bus,
        os_context: get_os_context(osal_spi_dev_ctx),
    }))
    .cast()
}
unsafe extern "C" fn bus_spi_dev_rem(os_spi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_spi_dev_rem");

    drop(Box::from_raw(os_spi_dev_ctx.cast::<SpiDevice>()));
}
unsafe extern "C" fn bus_spi_dev_init(os_spi_dev_ctx: *mut core::ffi::c_void) -> nrf_wifi_status {
    trace!("Called OS bus_spi_dev_init");
    nrf_wifi_status::NRF_WIFI_STATUS_SUCCESS
}
unsafe extern "C" fn bus_spi_dev_deinit(os_spi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_spi_dev_deinit");
}
unsafe extern "C" fn bus_spi_dev_intr_reg(
    os_spi_dev_ctx: *mut core::ffi::c_void,
    callbk_data: *mut core::ffi::c_void,
    callback_fn: core::option::Option<
        unsafe extern "C" fn(callbk_data: *mut core::ffi::c_void) -> core::ffi::c_int,
    >,
) -> nrf_wifi_status {
    trace!("Called OS bus_spi_dev_intr_reg: {}", os_spi_dev_ctx);

    let os_ctx = (*os_spi_dev_ctx.cast::<SpiDevice>()).os_context;

    critical_section::with(|cs| {
        *(*os_ctx).bus_interrupt.borrow_ref_mut(cs) = Some(crate::BusInterrupt {
            interrupt_data: callbk_data,
            interrupt_callback: callback_fn.unwrap(),
        });
    });

    nrf_wifi_status::NRF_WIFI_STATUS_SUCCESS
}
unsafe extern "C" fn bus_spi_dev_intr_unreg(os_spi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_spi_dev_intr_unreg");

    let os_ctx = (*os_spi_dev_ctx.cast::<SpiDevice>()).os_context;

    critical_section::with(|cs| {
        *(*os_ctx).bus_interrupt.borrow_ref_mut(cs) = None;
    });
}
unsafe extern "C" fn bus_spi_dev_host_map_get(
    os_spi_dev_ctx: *mut core::ffi::c_void,
    host_map: *mut nrf_wifi_osal_host_map,
) {
    trace!("Called OS bus_spi_dev_host_map_get");
    (*host_map).addr = 0;
    (*host_map).size = 0;
}
unsafe extern "C" fn assert(
    test_val: core::ffi::c_int,
    val: core::ffi::c_int,
    op: nrf_wifi_assert_op_type,
    assert_msg: *mut core::ffi::c_char,
) {
    trace!("Called OS assert");
    todo!();
}
unsafe extern "C" fn strlen(str_: *const core::ffi::c_void) -> core::ffi::c_uint {
    trace!("Called OS strlen");
    tinyrlibc::strlen(str_.cast()) as _
}

struct LinkedList {
    head: *mut LinkedListNode,
    tail: *mut LinkedListNode,
    len: usize,
}

struct LinkedListNode {
    next: *mut LinkedListNode,
    data: *mut c_void,
}

struct Tasklet {
    tasklet_type: nrf_wifi_tasklet_type,
    callback: core::option::Option<unsafe extern "C" fn(arg1: core::ffi::c_ulong)>,
    data: core::ffi::c_ulong,
    index: u16,
}

static WORK_QUEUE_BH: Channel<
    CriticalSectionRawMutex,
    (
        unsafe extern "C" fn(arg1: core::ffi::c_ulong),
        core::ffi::c_ulong,
        u16,
    ),
    16,
> = Channel::new();
static WORK_QUEUE_IRQ: Channel<
    CriticalSectionRawMutex,
    (
        unsafe extern "C" fn(arg1: core::ffi::c_ulong),
        core::ffi::c_ulong,
        u16,
    ),
    16,
> = Channel::new();
static WORK_QUEUE_TX_DONE: Channel<
    CriticalSectionRawMutex,
    (
        unsafe extern "C" fn(arg1: core::ffi::c_ulong),
        core::ffi::c_ulong,
        u16,
    ),
    16,
> = Channel::new();
static WORK_QUEUE_RX: Channel<
    CriticalSectionRawMutex,
    (
        unsafe extern "C" fn(arg1: core::ffi::c_ulong),
        core::ffi::c_ulong,
        u16,
    ),
    16,
> = Channel::new();

pub async fn run_work_queue() -> ! {
    loop {
        // Ordering is user cfg in nordic code.
        // Ordering seems important, but not clear what the best ordering is...
        futures::select_biased! {
            (function, arg, index) = WORK_QUEUE_IRQ.receive().fuse() => {
                defmt::debug!("Running IRQ work item {} function {} with arg {}", index, function, arg);
                unsafe { function(arg) };
            },
            (function, arg, index) = WORK_QUEUE_TX_DONE.receive().fuse() => {
                defmt::debug!("Running TX_DONE work item {} function {} with arg {}", index, function, arg);
                unsafe { function(arg) };
            },
            (function, arg, index) = WORK_QUEUE_RX.receive().fuse() => {
                defmt::debug!("Running RX work item {} function {} with arg {}", index, function, arg);
                unsafe { function(arg) };
            },
            (function, arg, index) = WORK_QUEUE_BH.receive().fuse() => {
                defmt::debug!("Running BH work item {} function {} with arg {}", index, function, arg);
                unsafe { function(arg) };
            }
        }
    }
}

extern "C" {
    fn c_log_dbg(fmt: *const core::ffi::c_char, args: nrf700x_sys::va_list) -> core::ffi::c_int;
    fn c_log_info(fmt: *const core::ffi::c_char, args: nrf700x_sys::va_list) -> core::ffi::c_int;
    fn c_log_err(fmt: *const core::ffi::c_char, args: nrf700x_sys::va_list) -> core::ffi::c_int;
}

struct NetworkBuffer {
    buffer: Vec<u8>,
    start: usize,
    end: usize,
}

impl NetworkBuffer {
    pub fn new(size: usize) -> Self {
        Self {
            buffer: alloc::vec![0; size],
            start: 0,
            end: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.end - self.start
    }

    pub fn headroom(&self) -> usize {
        self.start
    }

    pub fn reserve_headroom(&mut self, size: usize) {
        if self.len() == 0 {
            self.start = size;
            self.end = size;
        } else {
            if self.start < size {
                let len = self.len();

                self.buffer
                    .as_mut_slice()
                    .copy_within(self.start..self.end, size);

                self.start = size;
                self.end = size + len;
            }
        }
    }

    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.buffer[self.start..self.end]
    }

    pub fn put(&mut self, amount: usize) {
        self.end += amount;
    }

    pub fn push(&mut self, amount: usize) {
        self.start = self.start.checked_sub(amount).unwrap();
    }

    pub fn pull(&mut self, amount: usize) {
        self.start += amount;
    }
}
