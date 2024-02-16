#![allow(unused_variables)]

extern crate alloc;

use core::{ffi::c_void, mem::{size_of, size_of_val}, ptr::null_mut, sync::atomic::AtomicBool};

use alloc::{alloc::Layout, boxed::Box};

use defmt::{info, trace};
use nrf700x_sys::{
    nrf_wifi_assert_op_type, nrf_wifi_osal_dma_dir, nrf_wifi_osal_host_map, nrf_wifi_osal_ops,
    nrf_wifi_status, va_list,
};

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
        log_dbg: Some(log_dbg),
        log_info: Some(log_info),
        log_err: Some(log_err),
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
    trace!("Called OS mem_alloc");
    let ptr: *mut usize =
        alloc::alloc::alloc(Layout::from_size_align(size + size_of::<usize>(), 4).unwrap()).cast();
    ptr.write_volatile(size);
    ptr.add(1).cast()
}
unsafe extern "C" fn mem_zalloc(size: usize) -> *mut core::ffi::c_void {
    trace!("Called OS mem_zalloc");
    let ptr: *mut usize =
        alloc::alloc::alloc_zeroed(Layout::from_size_align(size + size_of::<usize>(), 4).unwrap())
            .cast();
    ptr.write_volatile(size);
    ptr.add(1).cast()
}
unsafe extern "C" fn mem_free(buf: *mut core::ffi::c_void) {
    trace!("Called OS mem_free");
    let ptr = buf.cast::<usize>().offset(-1);
    alloc::alloc::dealloc(
        ptr.cast(),
        Layout::from_size_align(*ptr + size_of::<usize>(), 4).unwrap(),
    )
}
unsafe extern "C" fn mem_cpy(
    dest: *mut core::ffi::c_void,
    src: *const core::ffi::c_void,
    count: usize,
) -> *mut core::ffi::c_void {
    trace!("Called OS mem_cpy");
    src.copy_to(dest, count);
    dest
}
unsafe extern "C" fn mem_set(
    start: *mut core::ffi::c_void,
    val: core::ffi::c_int,
    size: usize,
) -> *mut core::ffi::c_void {
    trace!("Called OS mem_set");
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
    trace!("Called OS spi_read_reg32");
    todo!();
}
unsafe extern "C" fn spi_write_reg32(
    priv_: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    val: core::ffi::c_uint,
) {
    trace!("Called OS spi_write_reg32");
    todo!();
}
unsafe extern "C" fn spi_cpy_from(
    priv_: *mut core::ffi::c_void,
    dest: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    count: usize,
) {
    trace!("Called OS spi_cpy_from");
    todo!();
}
unsafe extern "C" fn spi_cpy_to(
    priv_: *mut core::ffi::c_void,
    addr: core::ffi::c_ulong,
    src: *const core::ffi::c_void,
    count: usize,
) {
    trace!("Called OS spi_cpy_to");
    todo!();
}
unsafe extern "C" fn spinlock_alloc() -> *mut core::ffi::c_void {
    trace!("Called OS spinlock_alloc");
    Box::into_raw(Box::new(AtomicBool::new(false))).cast()
}
unsafe extern "C" fn spinlock_free(lock: *mut core::ffi::c_void) {
    trace!("Called OS spinlock_free");
    drop(Box::<AtomicBool>::from_raw(lock.cast()));
}
unsafe extern "C" fn spinlock_init(lock: *mut core::ffi::c_void) {
    trace!("Called OS spinlock_init");
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
unsafe extern "C" fn log_dbg(fmt: *const core::ffi::c_char, args: va_list) -> core::ffi::c_int {
    trace!("Called OS log_dbg");
    todo!();
}
unsafe extern "C" fn log_info(fmt: *const core::ffi::c_char, args: va_list) -> core::ffi::c_int {
    trace!("Called OS log_info");
    todo!();
}
unsafe extern "C" fn log_err(fmt: *const core::ffi::c_char, args: va_list) -> core::ffi::c_int {
    trace!("Called OS log_err");
    todo!();
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
    trace!("Called OS llist_alloc");
    Box::into_raw(Box::new(Option::<LinkedList>::None)).cast()
}
unsafe extern "C" fn llist_free(llist: *mut core::ffi::c_void) {
    trace!("Called OS llist_free");
    drop(Box::from_raw(llist.cast::<Option<LinkedList>>()));
}
unsafe extern "C" fn llist_init(llist: *mut core::ffi::c_void) {
    trace!("Called OS llist_init");
    *llist.cast::<Option<LinkedList>>() = Some(LinkedList {
        head: null_mut(),
        tail: null_mut(),
        len: 0,
    });
}
unsafe extern "C" fn llist_add_node_tail(
    llist: *mut core::ffi::c_void,
    llist_node: *mut core::ffi::c_void,
) {
    trace!("Called OS llist_add_node_tail");
    let list = (*llist.cast::<Option<LinkedList>>()).as_mut().unwrap();
    (*list.tail).next = llist_node.cast();
    list.tail = llist_node.cast();
    list.len += 1;
}
unsafe extern "C" fn llist_add_node_head(
    llist: *mut core::ffi::c_void,
    llist_node: *mut core::ffi::c_void,
) {
    trace!("Called OS llist_add_node_head");
    let list = (*llist.cast::<Option<LinkedList>>()).as_mut().unwrap();
    (*llist_node.cast::<LinkedListNode>()).next = list.head.cast();
    list.head = llist_node.cast();
    list.len += 1;
}
unsafe extern "C" fn llist_get_node_head(llist: *mut core::ffi::c_void) -> *mut core::ffi::c_void {
    trace!("Called OS llist_get_node_head");
    (*llist.cast::<Option<LinkedList>>())
        .as_mut()
        .unwrap()
        .head
        .cast()
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
    let current = llist_get_node_head(llist);

    if current.is_null() {
        unreachable!()
    }

    loop {
        let next = llist_get_node_nxt(llist, current);

        if next.is_null() {
            unreachable!();
        }

        if next == llist_node {
            let next_after = llist_get_node_nxt(llist, next);
            (*current.cast::<LinkedListNode>()).next = next_after.cast();
            (*llist.cast::<Option<LinkedList>>()).as_mut().unwrap().len -= 1;
            return;
        }
    }
}
unsafe extern "C" fn llist_len(llist: *mut core::ffi::c_void) -> core::ffi::c_uint {
    trace!("Called OS llist_len");
    (*llist.cast::<Option<LinkedList>>()).as_mut().unwrap().len as _
}
unsafe extern "C" fn nbuf_alloc(size: core::ffi::c_uint) -> *mut core::ffi::c_void {
    trace!("Called OS nbuf_alloc");
    todo!();
}
unsafe extern "C" fn nbuf_free(nbuf: *mut core::ffi::c_void) {
    trace!("Called OS nbuf_free");
    todo!();
}
unsafe extern "C" fn nbuf_headroom_res(nbuf: *mut core::ffi::c_void, size: core::ffi::c_uint) {
    trace!("Called OS nbuf_headroom_res");
    todo!();
}
unsafe extern "C" fn nbuf_headroom_get(nbuf: *mut core::ffi::c_void) -> core::ffi::c_uint {
    trace!("Called OS nbuf_headroom_get");
    todo!();
}
unsafe extern "C" fn nbuf_data_size(nbuf: *mut core::ffi::c_void) -> core::ffi::c_uint {
    trace!("Called OS nbuf_data_size");
    todo!();
}
unsafe extern "C" fn nbuf_data_get(nbuf: *mut core::ffi::c_void) -> *mut core::ffi::c_void {
    trace!("Called OS nbuf_data_get");
    todo!();
}
unsafe extern "C" fn nbuf_data_put(
    nbuf: *mut core::ffi::c_void,
    size: core::ffi::c_uint,
) -> *mut core::ffi::c_void {
    trace!("Called OS nbuf_data_put");
    todo!();
}
unsafe extern "C" fn nbuf_data_push(
    nbuf: *mut core::ffi::c_void,
    size: core::ffi::c_uint,
) -> *mut core::ffi::c_void {
    trace!("Called OS nbuf_data_push");
    todo!();
}
unsafe extern "C" fn nbuf_data_pull(
    nbuf: *mut core::ffi::c_void,
    size: core::ffi::c_uint,
) -> *mut core::ffi::c_void {
    trace!("Called OS nbuf_data_pull");
    todo!();
}
unsafe extern "C" fn nbuf_get_priority(nbuf: *mut core::ffi::c_void) -> core::ffi::c_uchar {
    trace!("Called OS nbuf_get_priority");
    todo!();
}
unsafe extern "C" fn tasklet_alloc(type_: core::ffi::c_int) -> *mut core::ffi::c_void {
    // Type is nrf_wifi_tasklet_type
    trace!("Called OS tasklet_alloc");
    todo!();
}
unsafe extern "C" fn tasklet_free(tasklet: *mut core::ffi::c_void) {
    trace!("Called OS tasklet_free");
    todo!();
}
unsafe extern "C" fn tasklet_init(
    tasklet: *mut core::ffi::c_void,
    callback: core::option::Option<unsafe extern "C" fn(arg1: core::ffi::c_ulong)>,
    data: core::ffi::c_ulong,
) {
    trace!("Called OS tasklet_init");
    todo!();
}
unsafe extern "C" fn tasklet_schedule(tasklet: *mut core::ffi::c_void) {
    trace!("Called OS tasklet_schedule");
    todo!();
}
unsafe extern "C" fn tasklet_kill(tasklet: *mut core::ffi::c_void) {
    trace!("Called OS tasklet_kill");
    todo!();
}
unsafe extern "C" fn sleep_ms(msecs: core::ffi::c_int) -> core::ffi::c_int {
    trace!("Called OS sleep_ms");
    todo!();
}
unsafe extern "C" fn delay_us(usecs: core::ffi::c_int) -> core::ffi::c_int {
    trace!("Called OS delay_us");
    todo!();
}
unsafe extern "C" fn time_get_curr_us() -> core::ffi::c_ulong {
    trace!("Called OS time_get_curr_us");
    todo!();
}
unsafe extern "C" fn time_elapsed_us(start_time_us: core::ffi::c_ulong) -> core::ffi::c_uint {
    trace!("Called OS time_elapsed_us");
    todo!();
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
unsafe extern "C" fn bus_spi_init() -> *mut core::ffi::c_void {
    trace!("Called OS bus_spi_init");
    todo!();
}
unsafe extern "C" fn bus_spi_deinit(os_spi_priv: *mut core::ffi::c_void) {
    trace!("Called OS bus_spi_deinit");
    todo!();
}
unsafe extern "C" fn bus_spi_dev_add(
    spi_priv: *mut core::ffi::c_void,
    osal_spi_dev_ctx: *mut core::ffi::c_void,
) -> *mut core::ffi::c_void {
    trace!("Called OS bus_spi_dev_add");
    todo!();
}
unsafe extern "C" fn bus_spi_dev_rem(os_spi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_spi_dev_rem");
    todo!();
}
unsafe extern "C" fn bus_spi_dev_init(os_spi_dev_ctx: *mut core::ffi::c_void) -> nrf_wifi_status {
    trace!("Called OS bus_spi_dev_init");
    todo!();
}
unsafe extern "C" fn bus_spi_dev_deinit(os_spi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_spi_dev_deinit");
    todo!();
}
unsafe extern "C" fn bus_spi_dev_intr_reg(
    os_spi_dev_ctx: *mut core::ffi::c_void,
    callbk_data: *mut core::ffi::c_void,
    callback_fn: core::option::Option<
        unsafe extern "C" fn(callbk_data: *mut core::ffi::c_void) -> core::ffi::c_int,
    >,
) -> nrf_wifi_status {
    trace!("Called OS bus_spi_dev_intr_reg");
    todo!();
}
unsafe extern "C" fn bus_spi_dev_intr_unreg(os_spi_dev_ctx: *mut core::ffi::c_void) {
    trace!("Called OS bus_spi_dev_intr_unreg");
    todo!();
}
unsafe extern "C" fn bus_spi_dev_host_map_get(
    os_spi_dev_ctx: *mut core::ffi::c_void,
    host_map: *mut nrf_wifi_osal_host_map,
) {
    trace!("Called OS bus_spi_dev_host_map_get");
    todo!();
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
    todo!();
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
