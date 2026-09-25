//! The TrangorgeOS host resource layer.
//!
//! This is the **only** boundary between the driver and the rest of the system.
//! Workspace rules forbid a driver from hardcoding physical addresses or
//! poking configuration space directly, so MMIO mappings, DMA memory, PCI config
//! space and ACPI tables are all requested from `ds-manager` through the
//! [`Platform`] abstraction.
//!
//! Abstracting the transport behind a trait buys two things directly:
//!
//! 1. The driver logic can be unit tested with no kernel present (see
//!    `MockPlatform` in this module), so the tests actually run instead of merely
//!    type-checking;
//! 2. Swapping the transport later (shared-memory ring, ioctl) leaves
//!    `driver.rs` untouched.
//!
//! The real implementation is [`SyscallPlatform`], which turns each operation
//! into a single `DsCmd`.

use kapi_abi::{
    DsCmd, DsError, DsMsg,
    payloads::sys::{AcpiTableRequest, acpi_table_reply},
};

/// DMA allocation flags. Bit layout matches `ds-mem`'s `DmaFlags`.
pub const DMA_COHERENT: u32 = 1 << 0;
pub const DMA_HIGH_MEM: u32 = 1 << 1;
pub const DMA_CONTIGUOUS: u32 = 1 << 2;

/// A physical range the kernel mapped for the driver to use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MappedRegion {
    pub phys: u64,
    pub virt: u64,
    pub size: u64,
}

impl MappedRegion {
    #[inline]
    pub const fn new(phys: u64, virt: u64, size: u64) -> Self {
        Self { phys, virt, size }
    }

    #[inline]
    pub const fn is_valid(self) -> bool {
        self.size != 0
    }

    /// The range's virtual base.
    ///
    /// # Safety
    ///
    /// The caller must keep the mapping alive and writable; only the
    /// [`Platform`] implementation that created it may release it.
    #[inline]
    pub unsafe fn as_mut_ptr(self) -> *mut u8 {
        self.virt as *mut u8
    }
}

/// One ACPI SDT the kernel mapped on our behalf.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AcpiView {
    pub virt: u64,
    pub len: u32,
}

impl AcpiView {
    /// A read-only view of the table bytes.
    ///
    /// # Safety
    ///
    /// The mapping must stay valid for the lifetime of the returned slice.
    /// `ds-manager` holds it until the driver releases the request.
    #[inline]
    pub unsafe fn bytes(&self) -> &[u8] {
        if self.len == 0 {
            return &[];
        }
        unsafe { core::slice::from_raw_parts(self.virt as *const u8, self.len as usize) }
    }
}

/// Every host capability the driver needs.
///
/// Deliberately kept narrow: it exposes only what IOMMU bring-up really needs,
/// so the driver cannot acquire capabilities outside its remit.
pub trait Platform {
    /// Map a device register window.
    fn map_mmio(&self, phys: u64, size: u64) -> Result<MappedRegion, DsError>;

    /// Release a mapping taken by [`Self::map_mmio`].
    fn unmap_mmio(&self, region: MappedRegion) -> Result<(), DsError>;

    /// Allocate device-visible (DMA) memory.
    fn alloc_dma(&self, size: u64, flags: u32) -> Result<MappedRegion, DsError>;

    /// Release memory taken by [`Self::alloc_dma`].
    fn free_dma(&self, region: MappedRegion) -> Result<(), DsError>;

    /// Read PCI config space (offset & 0xFC).
    fn read_pci(&self, requester: u32, offset: u32) -> Result<u32, DsError>;

    /// Write PCI config space (offset & 0xFC).
    fn write_pci(&self, requester: u32, offset: u32, value: u32) -> Result<(), DsError>;

    /// Fetch one ACPI table by four-character signature (e.g. `b"DMAR"`).
    fn acpi_table(&self, signature: [u8; 4], instance: u32) -> Result<AcpiView, DsError>;
}

/// The real platform implementation, talking to `ds-manager` through `kapi-syscall`.
#[derive(Clone, Copy, Debug, Default)]
pub struct SyscallPlatform;

impl SyscallPlatform {
    #[inline]
    pub const fn new() -> Self {
        Self
    }

    /// Translate one syscall's status code into a [`DsError`].
    #[inline]
    fn check(reply: DsMsg) -> Result<DsMsg, DsError> {
        if reply.is_ok() {
            Ok(reply)
        } else {
            Err(DsError::from_u32(reply.status as u32))
        }
    }
}

impl Platform for SyscallPlatform {
    fn map_mmio(&self, phys: u64, size: u64) -> Result<MappedRegion, DsError> {
        if size == 0 {
            return Err(DsError::InvalidMessage);
        }
        let reply = Self::check(kapi_syscall::sys_ipc_call(DsCmd::SysMapMmio, phys, size, 0))?;
        Ok(MappedRegion::new(phys, reply.arg0, size))
    }

    fn unmap_mmio(&self, region: MappedRegion) -> Result<(), DsError> {
        Self::check(kapi_syscall::sys_ipc_call(
            DsCmd::SysUnmapMmio,
            region.virt,
            region.size,
            0,
        ))?;
        Ok(())
    }

    fn alloc_dma(&self, size: u64, flags: u32) -> Result<MappedRegion, DsError> {
        if size == 0 || size & 0xFFF != 0 {
            return Err(DsError::InvalidMessage);
        }
        let reply =
            Self::check(kapi_syscall::sys_ipc_call(DsCmd::SysAllocDma, size, flags as u64, 0))?;
        Ok(MappedRegion::new(reply.arg0, reply.arg1, size))
    }

    fn free_dma(&self, region: MappedRegion) -> Result<(), DsError> {
        Self::check(kapi_syscall::sys_ipc_call(
            DsCmd::SysFreeDma,
            region.phys,
            region.size,
            0,
        ))?;
        Ok(())
    }

    fn read_pci(&self, requester: u32, offset: u32) -> Result<u32, DsError> {
        let reply = Self::check(kapi_syscall::sys_ipc_call(
            DsCmd::PciRead,
            requester as u64,
            (offset & 0xFC) as u64,
            0,
        ))?;
        Ok(reply.arg0 as u32)
    }

    fn write_pci(&self, requester: u32, offset: u32, value: u32) -> Result<(), DsError> {
        Self::check(kapi_syscall::sys_ipc_call(
            DsCmd::PciWrite,
            requester as u64,
            (offset & 0xFC) as u64,
            value as u64,
        ))?;
        Ok(())
    }

    fn acpi_table(&self, signature: [u8; 4], instance: u32) -> Result<AcpiView, DsError> {
        let request = AcpiTableRequest::new(signature, instance);
        let reply = Self::check(kapi_syscall::sys_ipc_call(
            DsCmd::SysAcpiTable,
            &request as *const _ as u64,
            core::mem::size_of::<AcpiTableRequest>() as u64,
            0,
        ))?;

        // Convention: arg0 = virtual base, arg1 = length (see acpi_table_reply).
        let virt = reply.arg(acpi_table_reply::VIRT_BASE);
        let len = reply.arg(acpi_table_reply::LENGTH);
        if virt == 0 || len == 0 {
            return Err(DsError::DeviceNotFound);
        }
        Ok(AcpiView { virt, len: len as u32 })
    }
}


#[cfg(test)]
pub(crate) mod mock {
    //! A test `Platform` implementation, shared with the `driver` tests.

    use super::{AcpiView, DsError, MappedRegion, Platform, DMA_CONTIGUOUS};
    use core::cell::RefCell;

    /// A fake platform that never touches a kernel: it records requests and
    /// hands out mappings from buffers the test owns. This lets the discovery
    /// and mapping logic in `driver.rs` actually execute.
    pub struct MockPlatform {
        next_virt: RefCell<u64>,
        pub pci: RefCell<Vec<(u32, u32, u32)>>,
        pub acpi: RefCell<Option<(u64, u32)>>,
        pub fail_next: RefCell<Option<DsError>>,
    }

    impl MockPlatform {
        pub fn new(acpi_virt: u64, acpi_len: u32) -> Self {
            Self {
                next_virt: RefCell::new(0x1_0000_0000),
                pci: RefCell::new(Vec::new()),
                acpi: RefCell::new(if acpi_len == 0 {
                    None
                } else {
                    Some((acpi_virt, acpi_len))
                }),
                fail_next: RefCell::new(None),
            }
        }

        fn take_virt(&self, size: u64) -> u64 {
            let mut next = self.next_virt.borrow_mut();
            let base = *next;
            *next += size.max(0x1000);
            base
        }

        /// Make the next platform call fail, to exercise error paths.
        pub fn fail_next(&self, error: DsError) {
            *self.fail_next.borrow_mut() = Some(error);
        }

        fn check(&self) -> Result<(), DsError> {
            match self.fail_next.borrow_mut().take() {
                Some(error) => Err(error),
                None => Ok(()),
            }
        }
    }

    impl Platform for MockPlatform {
        fn map_mmio(&self, phys: u64, size: u64) -> Result<MappedRegion, DsError> {
            self.check()?;
            if size == 0 {
                return Err(DsError::InvalidMessage);
            }
            Ok(MappedRegion::new(phys, self.take_virt(size), size))
        }

        fn unmap_mmio(&self, region: MappedRegion) -> Result<(), DsError> {
            self.check()?;
            assert_ne!(region.virt, 0);
            Ok(())
        }

        fn alloc_dma(&self, size: u64, flags: u32) -> Result<MappedRegion, DsError> {
            self.check()?;
            if size == 0 || size & 0xFFF != 0 {
                return Err(DsError::InvalidMessage);
            }
            // The IOMMU walks these pages in hardware, so they must be contiguous.
            assert_ne!(
                flags & DMA_CONTIGUOUS,
                0,
                "IOMMU tables must be contiguous"
            );
            Ok(MappedRegion::new(0x8000_0000, self.take_virt(size), size))
        }

        fn free_dma(&self, region: MappedRegion) -> Result<(), DsError> {
            self.check()?;
            assert_ne!(region.phys, 0);
            Ok(())
        }

        fn read_pci(&self, requester: u32, offset: u32) -> Result<u32, DsError> {
            self.check()?;
            for (r, o, v) in self.pci.borrow().iter() {
                if *r == requester && *o == (offset & 0xFC) {
                    return Ok(*v);
                }
            }
            Err(DsError::DeviceNotFound)
        }

        fn write_pci(&self, requester: u32, offset: u32, value: u32) -> Result<(), DsError> {
            self.check()?;
            let mut pci = self.pci.borrow_mut();
            for (r, o, v) in pci.iter_mut() {
                if *r == requester && *o == (offset & 0xFC) {
                    *v = value;
                    return Ok(());
                }
            }
            pci.push((requester, offset & 0xFC, value));
            Ok(())
        }

        fn acpi_table(&self, signature: [u8; 4], instance: u32) -> Result<AcpiView, DsError> {
            self.check()?;
            if signature != *b"DMAR" || instance != 0 {
                return Err(DsError::DeviceNotFound);
            }
            let (virt, len) = (*self.acpi.borrow()).ok_or(DsError::DeviceNotFound)?;
            Ok(AcpiView { virt, len })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::mock::MockPlatform;
    use super::*;
    use super::*;
    use core::cell::RefCell;

    #[test]
    fn mock_platform_hands_out_distinct_physical_windows() {
        let platform = MockPlatform::new(0, 0);
        let a = platform.alloc_dma(0x1000, DMA_CONTIGUOUS).expect("alloc");
        let b = platform.alloc_dma(0x1000, DMA_CONTIGUOUS).expect("alloc");

        assert_eq!(a.phys, 0x8000_0000);
        assert_ne!(a.virt, b.virt, "every mapping must get its own window");
        assert!(a.is_valid());
        platform.free_dma(a).expect("free");
    }

    #[test]
    fn misaligned_requests_are_refused_before_reaching_the_kernel() {
        let platform = MockPlatform::new(0, 0);
        assert_eq!(
            platform.alloc_dma(0x800, DMA_CONTIGUOUS).unwrap_err(),
            DsError::InvalidMessage
        );
        assert_eq!(platform.alloc_dma(0, DMA_CONTIGUOUS).unwrap_err(), DsError::InvalidMessage);
        assert_eq!(platform.map_mmio(0x1000, 0).unwrap_err(), DsError::InvalidMessage);
    }

    #[test]
    fn kernel_errors_propagate_unchanged() {
        let platform = MockPlatform::new(0, 0);
        platform.fail_next(DsError::PermissionDenied);
        assert_eq!(
            platform.alloc_dma(0x1000, DMA_CONTIGUOUS).unwrap_err(),
            DsError::PermissionDenied
        );
        // the injected failure happens only once
        assert!(platform.alloc_dma(0x1000, DMA_CONTIGUOUS).is_ok());
    }

    #[test]
    fn pci_access_space_is_round_tripped() {
        let platform = MockPlatform::new(0, 0);
        platform.write_pci(0x0000_0100, 0x04, 0x0000_0007).expect("write");
        assert_eq!(platform.read_pci(0x0000_0100, 0x04).expect("read"), 0x07);
        // an unaligned offset is masked down to the DWORD boundary
        assert_eq!(platform.read_pci(0x0000_0100, 0x06).expect("read"), 0x07);
        assert_eq!(
            platform.read_pci(0xdead_beef, 0x00).unwrap_err(),
            DsError::DeviceNotFound
        );
    }

    #[test]
    fn acpi_views_expose_the_table_bytes() {
        let table = [0x44u8, 0x4d, 0x41, 0x52, 0xde, 0xad];
        let platform = MockPlatform::new(table.as_ptr() as u64, table.len() as u32);

        let view = platform.acpi_table(*b"DMAR", 0).expect("table");
        assert_eq!(unsafe { view.bytes() }, &table[..]);
        assert_eq!(platform.acpi_table(*b"IVRS", 0).unwrap_err(), DsError::DeviceNotFound);
    }
}
