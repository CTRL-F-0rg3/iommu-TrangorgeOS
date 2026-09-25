//! TrangorgeOS 宿主资源层。
//!
//! 这是驱动与系统其余部分之间**唯一**的边界。工作区规则禁止驱动硬编码物理
//! 地址或直接访问配置空间，因此 MMIO 映射、DMA 内存、PCI 配置空间与 ACPI 表
//! 一律通过 [`Platform`] 抽象向 `ds-manager` 申请。
//!
//! 把传输层抽象成 trait 有两个直接好处：
//!
//! 1. 驱动逻辑可以在没有内核的环境下做单元测试（见本模块的 `MockPlatform`），
//!    测试真的跑得起来，而不只是做类型检查；
//! 2. 将来换传输方式（共享内存 ring、ioctl）不影响 `driver.rs` 的逻辑。
//!
//! 真实实现是 [`SyscallPlatform`]，它把每个操作翻译成一个 `DsCmd`。

use kapi_abi::{
    DsCmd, DsError, DsMsg,
    payloads::sys::{AcpiTableRequest, acpi_table_reply},
};

/// DMA 分配属性。位定义与 `ds-mem` 的 `DmaFlags` 一致。
pub const DMA_COHERENT: u32 = 1 << 0;
pub const DMA_HIGH_MEM: u32 = 1 << 1;
pub const DMA_CONTIGUOUS: u32 = 1 << 2;

/// 一段由内核映射给驱动使用的物理区间。
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

    /// 区间的虚拟基址。
    ///
    /// # Safety
    ///
    /// 调用方必须保证该映射仍然存活且可写；映射只能由创建它的
    /// [`Platform`] 实现释放。
    #[inline]
    pub unsafe fn as_mut_ptr(self) -> *mut u8 {
        self.virt as *mut u8
    }
}

/// 内核映射过来的一张 ACPI SDT 表。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AcpiView {
    pub virt: u64,
    pub len: u32,
}

impl AcpiView {
    /// 表的只读字节视图。
    ///
    /// # Safety
    ///
    /// 必须保证该映射在返回切片的生命周期内有效。`ds-manager` 在驱动回复
    /// 释放请求之前会一直持有映射。
    #[inline]
    pub unsafe fn bytes(&self) -> &[u8] {
        if self.len == 0 {
            return &[];
        }
        unsafe { core::slice::from_raw_parts(self.virt as *const u8, self.len as usize) }
    }
}

/// 驱动需要的全部宿主能力。
///
/// 刻意保持窄接口：只暴露 IOMMU bring-up 真正需要的操作，避免驱动获得超出
/// 职责范围的能力。
pub trait Platform {
    /// 映射设备寄存器窗口。
    fn map_mmio(&self, phys: u64, size: u64) -> Result<MappedRegion, DsError>;

    /// 解除 [`Self::map_mmio`] 的映射。
    fn unmap_mmio(&self, region: MappedRegion) -> Result<(), DsError>;

    /// 分配设备可见（DMA）内存。
    fn alloc_dma(&self, size: u64, flags: u32) -> Result<MappedRegion, DsError>;

    /// 释放 [`Self::alloc_dma`] 的内存。
    fn free_dma(&self, region: MappedRegion) -> Result<(), DsError>;

    /// 读 PCI 配置空间（offset & 0xFC）。
    fn read_pci(&self, requester: u32, offset: u32) -> Result<u32, DsError>;

    /// 写 PCI 配置空间（offset & 0xFC）。
    fn write_pci(&self, requester: u32, offset: u32, value: u32) -> Result<(), DsError>;

    /// 按四字符签名取一张 ACPI 表（例如 `b"DMAR"`）。
    fn acpi_table(&self, signature: [u8; 4], instance: u32) -> Result<AcpiView, DsError>;
}

/// 通过 `kapi-syscall` 与 `ds-manager` 通信的真实平台实现。
#[derive(Clone, Copy, Debug, Default)]
pub struct SyscallPlatform;

impl SyscallPlatform {
    #[inline]
    pub const fn new() -> Self {
        Self
    }

    /// 把一次 syscall 的状态码翻译成 `DsError`。
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

        // 约定：arg0 = 虚拟基址，arg1 = 长度（见 acpi_table_reply）。
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
    //! Testowa implementacja `Platform` — udostępniona dla testów `driver`.

    use super::{AcpiView, DsError, MappedRegion, Platform, DMA_CONTIGUOUS};
    use core::cell::RefCell;

    /// 一个不碰内核的假平台：记录请求，把映射落在测试自己准备的缓冲上。
    /// 这样 `driver.rs` 的发现/映射逻辑可以被真正执行。
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

        /// 让下一次平台调用失败，用于验证错误路径。
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
            // Strony IOMMU czyta sprzętowo, więc muszą być ciągłe.
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
        assert_ne!(a.virt, b.virt, "każde mapowanie musi dostać osobne okno");
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
        // 注入的失败只发生一次
        assert!(platform.alloc_dma(0x1000, DMA_CONTIGUOUS).is_ok());
    }

    #[test]
    fn pci_access_space_is_round_tripped() {
        let platform = MockPlatform::new(0, 0);
        platform.write_pci(0x0000_0100, 0x04, 0x0000_0007).expect("write");
        assert_eq!(platform.read_pci(0x0000_0100, 0x04).expect("read"), 0x07);
        // 未对齐 offset 会被掩到 DWORD 边界
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
