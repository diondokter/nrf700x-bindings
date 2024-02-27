#![allow(unused_variables)]

extern crate alloc;

use embedded_hal::spi::{Operation, SpiDevice};
pub type BusDeviceObject = *mut dyn BusDevice;

pub trait BusDevice {
    fn read(&mut self, addr: u32, buf: &mut [u8]);
    fn write(&mut self, addr: u32, buf: &[u8]);
    fn read_sr0(&mut self) -> u8;
    fn read_sr1(&mut self) -> u8;
    fn read_sr2(&mut self) -> u8;
    fn write_sr2(&mut self, val: u8);
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
            // Operation::Read(&mut [0; 8][..(regions::get_latency_for_address(addr) * 4) as usize]),
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
        .unwrap();
    }

    fn read_sr0(&mut self) -> u8 {
        let mut buf = [0; 2];
        self.transfer(&mut buf, &[0x05]).unwrap();
        let val = buf[1];
        defmt::trace!("read sr0 = {:02x}", val);
        val
    }

    fn read_sr1(&mut self) -> u8 {
        let mut buf = [0; 2];
        self.transfer(&mut buf, &[0x1f]).unwrap();
        let val = buf[1];
        defmt::trace!("read sr1 = {:02x}", val);
        val
    }

    fn read_sr2(&mut self) -> u8 {
        let mut buf = [0; 2];
        self.transfer(&mut buf, &[0x2f]).unwrap();
        let val = buf[1];
        defmt::trace!("read sr2 = {:02x}", val);
        val
    }

    fn write_sr2(&mut self, val: u8) {
        defmt::trace!("write sr2 = {:02x}", val);
        self.write(&[0x3f, val]).unwrap();
    }
}

#[rustfmt::skip]
pub(crate) mod regions {

    #[derive(Copy, Clone, Debug, defmt::Format)]
    pub(crate) struct MemoryRegion {
        start: u32,
        end: u32,

        /// Number of dummy 32bit words
        latency: u32,

        rpu_mem_start: u32,
        rpu_mem_end: u32,
        processor_restriction: Option<Processor>,
    }

    use super::*;
	pub(crate) const SYSBUS       : &MemoryRegion = &MemoryRegion { start: 0x000000, end: 0x008FFF, latency: 1, rpu_mem_start: 0xA4000000, rpu_mem_end: 0xA4FFFFFF, processor_restriction: None };
	pub(crate) const EXT_SYS_BUS  : &MemoryRegion = &MemoryRegion { start: 0x009000, end: 0x03FFFF, latency: 2, rpu_mem_start: 0,          rpu_mem_end: 0,          processor_restriction: None };
	pub(crate) const PBUS         : &MemoryRegion = &MemoryRegion { start: 0x040000, end: 0x07FFFF, latency: 1, rpu_mem_start: 0xA5000000, rpu_mem_end: 0xA5FFFFFF, processor_restriction: None };
	pub(crate) const PKTRAM       : &MemoryRegion = &MemoryRegion { start: 0x0C0000, end: 0x0F0FFF, latency: 0, rpu_mem_start: 0xB0000000, rpu_mem_end: 0xB0FFFFFF, processor_restriction: None };
	pub(crate) const GRAM         : &MemoryRegion = &MemoryRegion { start: 0x080000, end: 0x092000, latency: 1, rpu_mem_start: 0xB7000000, rpu_mem_end: 0xB7FFFFFF, processor_restriction: None };
	pub(crate) const LMAC_ROM     : &MemoryRegion = &MemoryRegion { start: 0x100000, end: 0x134000, latency: 1, rpu_mem_start: 0x80000000, rpu_mem_end: 0x80033FFF, processor_restriction: Some(Processor::LMAC) }; // ROM
	pub(crate) const LMAC_RET_RAM : &MemoryRegion = &MemoryRegion { start: 0x140000, end: 0x14C000, latency: 1, rpu_mem_start: 0x80040000, rpu_mem_end: 0x8004BFFF, processor_restriction: Some(Processor::LMAC) }; // retained RAM
	pub(crate) const LMAC_SRC_RAM : &MemoryRegion = &MemoryRegion { start: 0x180000, end: 0x190000, latency: 1, rpu_mem_start: 0x80080000, rpu_mem_end: 0x8008FFFF, processor_restriction: Some(Processor::LMAC) }; // scratch RAM
	pub(crate) const UMAC_ROM     : &MemoryRegion = &MemoryRegion { start: 0x200000, end: 0x261800, latency: 1, rpu_mem_start: 0x80000000, rpu_mem_end: 0x800617FF, processor_restriction: Some(Processor::UMAC) }; // ROM
	pub(crate) const UMAC_RET_RAM : &MemoryRegion = &MemoryRegion { start: 0x280000, end: 0x2A4000, latency: 1, rpu_mem_start: 0x80080000, rpu_mem_end: 0x800A3FFF, processor_restriction: Some(Processor::UMAC) }; // retained RAM
	pub(crate) const UMAC_SRC_RAM : &MemoryRegion = &MemoryRegion { start: 0x300000, end: 0x338000, latency: 1, rpu_mem_start: 0x80100000, rpu_mem_end: 0x80137FFF, processor_restriction: Some(Processor::UMAC) }; // scratch RAM

    pub(crate) const REGIONS: [&MemoryRegion; 11] = [
        SYSBUS, EXT_SYS_BUS, PBUS, PKTRAM, GRAM, LMAC_ROM, LMAC_RET_RAM, LMAC_SRC_RAM, UMAC_ROM, UMAC_RET_RAM, UMAC_SRC_RAM
    ];

    #[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
    pub(crate) enum Processor {
        LMAC,
        UMAC,
    }
    
    #[doc(alias = "pal_rpu_addr_offset_get")]
    pub(crate) fn remap_global_addr_to_region_and_offset(rpu_addr: u32, processor: Option<Processor>) -> (&'static MemoryRegion, u32) {
        defmt::unwrap!(
            REGIONS
                .into_iter()
                .filter(|region| region.processor_restriction.is_none() || region.processor_restriction == processor)
                .find(|region| rpu_addr >= region.rpu_mem_start && rpu_addr <= region.rpu_mem_end)
                .map(|region| (region, rpu_addr - region.rpu_mem_start))
        )
    }

    pub(crate) fn get_latency_for_address(addr: u32) -> u32 {
        defmt::unwrap!(REGIONS.into_iter().find(|region| addr >= region.start && addr <= region.end)).latency
    }
}
