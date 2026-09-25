//! IOMMU 驱动实现：把硬件核心接到 TrangorgeOS 设备类契约上。
//!
//! [`IommuDriver`] 实现 `ds_fw_iommu::IommuDevice`，也就是 `ds-manager` 通过
//! `DsCmd::Iommu*` 能看到的那台“设备”。它做三件事：
//!
//! 1. **发现**：通过 [`Platform::acpi_table`] 向内核要 DMAR/IVRS 表，用硬件
//!    核心自带的解析器枚举 remapping unit；
//! 2. **状态**：维护域、绑定、映射、保留区的簿记，并在每次改动后触发失效；
//! 3. **上报**：把固件保留区和故障翻译成线路载荷。
//!
//! 所有宿主资源都经由 [`Platform`]，驱动里不出现任何硬编码物理地址。

use ds_fw_iommu::{
    ControllerId, DomainId, IoRange, IommuDevice, RequesterId,
    types::RequesterId as WireRequester,
};
use kapi_abi::{
    DsError,
    payloads::iommu::{
        IommuBindPayload, IommuControllerInfo, IommuFaultPayload, IommuInvalidatePayload,
        IommuInvalidateScope, IommuKind, IommuMapPayload, IommuPermission,
        IommuReservedRegionPayload, IommuStage, IommuUnmapPayload,
    },
};

use crate::host::{DMA_CONTIGUOUS, MappedRegion, Platform};

/// 同时支持的控制器数量上限。
pub const MAX_CONTROLLERS: usize = 8;
/// 并存的地址空间域数量上限。
pub const MAX_DOMAINS: usize = 16;
/// 记录在案的映射条目上限。
pub const MAX_MAPPINGS: usize = 128;
/// 固件保留区数量上限。
pub const MAX_RESERVED: usize = 16;
/// 缓存的故障条数上限。
pub const MAX_FAULTS: usize = 16;
/// 记录在案的 requester 绑定数量上限。
pub const MAX_BINDINGS: usize = 64;

/// 一个已发现的控制器。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControllerEntry {
    pub info: IommuControllerInfo,
    /// 寄存器窗口；由 `ds-manager` 映射，驱动只借用不拥有。
    pub registers: Option<MappedRegion>,
}

/// 一个地址空间域的簿记条目。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DomainEntry {
    pub id: DomainId,
    pub controller: ControllerId,
    /// 域的二级页表根，由 `ds-manager` 分配的 DMA 内存承载。
    pub page_table: Option<MappedRegion>,
    pub live_mappings: u32,
}

/// 一条 IOVA 映射的簿记条目。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MappingEntry {
    pub domain: DomainId,
    pub range: IoRange,
    pub phys_base: u64,
    pub permission: IommuPermission,
}

/// 一个 requester → 域 的绑定。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BindingEntry {
    pub requester: WireRequester,
    pub domain: DomainId,
    pub controller: ControllerId,
}

const EMPTY_RESERVED: IommuReservedRegionPayload = IommuReservedRegionPayload {
    base: 0,
    limit: 0,
    requester: u32::MAX,
    _pad: 0,
};

const EMPTY_FAULT: IommuFaultPayload = IommuFaultPayload {
    controller: 0,
    reason: 0,
    requester: u32::MAX,
    _pad: 0,
    iova: 0,
    faulting_phys: 0,
};

/// 两个 IOVA 区间是否重叠。
fn overlaps(a: IoRange, b: IoRange) -> bool {
    a.iova < b.iova.saturating_add(b.size) && b.iova < a.iova.saturating_add(a.size)
}

/// IOMMU 设备实现。
///
/// `P` 是宿主平台：生产环境用 [`crate::host::SyscallPlatform`]，测试用
/// `MockPlatform`，因此下面的簿记逻辑可以在没有内核的环境下真正执行。
pub struct IommuDriver<P> {
    platform: P,
    controllers: [Option<ControllerEntry>; MAX_CONTROLLERS],
    domains: [Option<DomainEntry>; MAX_DOMAINS],
    mappings: [Option<MappingEntry>; MAX_MAPPINGS],
    bindings: [Option<BindingEntry>; MAX_BINDINGS],
    reserved: [IommuReservedRegionPayload; MAX_RESERVED],
    reserved_len: usize,
    faults: [IommuFaultPayload; MAX_FAULTS],
    fault_len: usize,
    /// 已触发的失效次数（诊断用）。
    invalidations: u32,
}

impl<P: Platform> IommuDriver<P> {
    /// 创建一个尚未探测的驱动实例。
    pub fn new(platform: P) -> Self {
        Self {
            platform,
            controllers: [None; MAX_CONTROLLERS],
            domains: [None; MAX_DOMAINS],
            mappings: [None; MAX_MAPPINGS],
            bindings: [None; MAX_BINDINGS],
            reserved: [EMPTY_RESERVED; MAX_RESERVED],
            reserved_len: 0,
            faults: [EMPTY_FAULT; MAX_FAULTS],
            fault_len: 0,
            invalidations: 0,
        }
    }

    /// 宿主平台引用。
    pub fn platform(&self) -> &P {
        &self.platform
    }

    /// 已触发的失效次数。
    pub fn invalidation_count(&self) -> u32 {
        self.invalidations
    }

    /// 控制器簿记。
    pub fn controller(&self, index: usize) -> Option<&ControllerEntry> {
        match self.controllers.get(index) {
            Some(Some(entry)) => Some(entry),
            _ => None,
        }
    }

    /// 域簿记。
    pub fn domain(&self, id: DomainId) -> Option<&DomainEntry> {
        self.domains.iter().flatten().find(|d| d.id == id)
    }

    /// 某 requester 当前的绑定。
    pub fn binding_of(&self, requester: WireRequester) -> Option<&BindingEntry> {
        self.binding_slot(requester).and_then(|i| self.bindings[i].as_ref())
    }

    /// 登记一次固件声明的保留区。
    pub fn add_reserved_region(&mut self, region: IommuReservedRegionPayload) {
        if self.reserved_len < MAX_RESERVED {
            self.reserved[self.reserved_len] = region;
            self.reserved_len += 1;
        }
    }

    /// 登记一个控制器，返回它的索引。
    pub fn add_controller(&mut self, info: IommuControllerInfo) -> Option<ControllerId> {
        let slot = self.controllers.iter().position(|c| c.is_none())?;
        self.controllers[slot] = Some(ControllerEntry { info, registers: None });
        Some(ControllerId(slot as u32))
    }

    /// 记下控制器的寄存器窗口。
    pub fn attach_registers(&mut self, controller: ControllerId, region: MappedRegion) {
        if let Some(entry) = self
            .controllers
            .get_mut(controller.index() as usize)
            .and_then(|c| c.as_mut())
        {
            entry.registers = Some(region);
        }
    }

    /// 上报一次故障（超过 `MAX_FAULTS` 时覆盖最旧的一条）。
    pub fn report_fault(&mut self, fault: IommuFaultPayload) {
        if self.fault_len < MAX_FAULTS {
            self.faults[self.fault_len] = fault;
            self.fault_len += 1;
        } else {
            self.faults.rotate_left(1);
            self.faults[MAX_FAULTS - 1] = fault;
        }
    }

    fn binding_slot(&self, requester: WireRequester) -> Option<usize> {
        self.bindings
            .iter()
            .position(|b| b.is_some_and(|b| b.requester == requester))
    }

    fn domain_index(&self, id: DomainId) -> Option<usize> {
        self.domains.iter().position(|d| d.is_some_and(|d| d.id == id))
    }

    /// 为 `range` 登记一条映射。
    fn record_mapping(&mut self, entry: MappingEntry) -> Result<(), DsError> {
        if entry.range.iova & 0xFFF != 0 || entry.range.size & 0xFFF != 0 {
            return Err(DsError::InvalidMessage);
        }
        if !entry.range.is_valid() {
            return Err(DsError::InvalidMessage);
        }
        for existing in self.mappings.iter().flatten() {
            if existing.domain == entry.domain && overlaps(existing.range, entry.range) {
                return Err(DsError::DeviceBusy);
            }
        }
        let slot = self
            .mappings
            .iter()
            .position(|s| s.is_none())
            .ok_or(DsError::OutOfMemory)?;
        self.mappings[slot] = Some(entry);
        if let Some(index) = self.domain_index(entry.domain) {
            if let Some(domain) = self.domains[index].as_mut() {
                domain.live_mappings += 1;
            }
        }
        Ok(())
    }

    /// 撤销一条登记中的映射。
    fn forget_mapping(&mut self, domain: DomainId, range: IoRange) -> bool {
        let Some(slot) = self
            .mappings
            .iter()
            .position(|s| s.is_some_and(|e| e.domain == domain && e.range == range))
        else {
            return false;
        };
        self.mappings[slot] = None;
        if let Some(index) = self.domain_index(domain) {
            if let Some(entry) = self.domains[index].as_mut() {
                entry.live_mappings = entry.live_mappings.saturating_sub(1);
            }
        }
        true
    }
}

/// x86_64 专用的 ACPI 探测：向内核索取 DMAR 表并枚举 remapping unit。
#[cfg(target_arch = "x86_64")]
impl<P: Platform> IommuDriver<P> {
    /// 读取 DMAR 表并登记控制器与固件保留区。
    ///
    /// 平台没有 DMAR 表（老平台、无 IOMMU 的机器）不算错误——调用方通过
    /// [`IommuDevice::controller_count`] 看到 0 个控制器即可。只有表存在但
    /// 格式损坏才返回 `Err`。
    pub fn probe_acpi(&mut self) -> Result<(), DsError> {
        use crate::arch::x86_64::intel::dmar::DmarTable;
        use kore_memory::Mapping;
        use memory_addr::{AddrRange, PhysAddr, VirtAddr};

        let view = match self.platform.acpi_table(*b"DMAR", 0) {
            Ok(view) => view,
            // 没有 IOMMU 的机器是正常情况。
            Err(DsError::DeviceNotFound) => return Ok(()),
            Err(error) => return Err(error),
        };

        // 把内核给的映射包装成 `kore_memory::Mapping`，这样硬件核心自带的
        // 解析器可以直接用——不必在驱动里再写一份 DMAR 解析器。
        let base = VirtAddr::from_usize(view.virt as usize);
        let end = VirtAddr::from_usize(view.virt as usize + view.len as usize);
        let mapping = Mapping::<
            crate::arch::x86_64::intel::paging::VtdSecondLevelPte,
            VirtAddr,
            PhysAddr,
        >::new(
            AddrRange::new(base, end),
            PhysAddr::from_usize(view.virt as usize),
            Default::default(),
        );

        // SAFETY: `view` 由 `ds-manager` 映射并保证在本次调用期间可读；
        // `mapping` 精确覆盖该区间。
        let table = unsafe { DmarTable::from_mapping(&mapping) }
            .map_err(|_| DsError::InvalidMessage)?;

        let mut units = 0usize;
        table
            .for_each_drhd(|_host_width, drhd| {
                let base = drhd.registers.start.as_usize() as u64;
                let size = drhd.registers.size() as u64;
                if size == 0 {
                    return Ok(());
                }
                let info = IommuControllerInfo {
                    controller: units as u32,
                    kind: IommuKind::IntelVtd,
                    capabilities: crate::CapabilityFlags::TRANSLATION.bits(),
                    stage: IommuStage::Stage2,
                    segment: drhd.segment as u32,
                    mmio_base: base,
                    mmio_size: size,
                };
                if self.add_controller(info).is_some() {
                    units += 1;
                }
                Ok(())
            })
            .map_err(|_| DsError::InvalidMessage)?;

        // RMRR：固件声明必须原样保留的 DMA 区间。
        table
            .for_each_rmrr(|region| {
                let base = region.memory.start.as_usize() as u64;
                let end = region.memory.end.as_usize() as u64;
                if end > base {
                    self.add_reserved_region(IommuReservedRegionPayload {
                        base,
                        // 载荷里的 `limit` 是闭区间上界。
                        limit: end - 1,
                        requester: RequesterId::NONE.raw(),
                        _pad: 0,
                    });

                }
                Ok(())
            })
            .map_err(|_| DsError::InvalidMessage)?;

        Ok(())
    }
}

impl<P: Platform> IommuDevice for IommuDriver<P> {
    fn controller_count(&self) -> usize {
        self.controllers.iter().flatten().count()
    }

    fn controller_info(&self, index: usize, out: &mut IommuControllerInfo) -> bool {
        match self.controller(index) {
            Some(entry) => {
                *out = entry.info;
                true
            }
            None => false,
        }
    }

    fn domain_create(
        &mut self,
        controller: ControllerId,
        hint: u32,
    ) -> Result<DomainId, DsError> {
        if self.controller(controller.index() as usize).is_none() {
            return Err(DsError::DeviceNotFound);
        }
        // hint 优先；否则取第一个空闲 id。
        let id = if hint != 0 && hint != u32::MAX {
            DomainId(hint)
        } else {
            let mut candidate = 1u32;
            loop {
                if self.domain_index(DomainId(candidate)).is_none() {
                    break DomainId(candidate);
                }
                candidate += 1;
                if candidate == u32::MAX {
                    return Err(DsError::OutOfMemory);
                }
            }
        };
        if self.domain_index(id).is_some() {
            return Err(DsError::DeviceBusy);
        }
        let slot = self
            .domains
            .iter()
            .position(|d| d.is_none())
            .ok_or(DsError::OutOfMemory)?;

        // 域的二级页表必须是设备可见、连续的内存——硬件要直接遍历它。
        let page_table = self
            .platform
            .alloc_dma(0x1000, DMA_CONTIGUOUS | crate::host::DMA_COHERENT)?;

        self.domains[slot] = Some(DomainEntry {
            id,
            controller,
            page_table: Some(page_table),
            live_mappings: 0,
        });
        Ok(id)
    }

    fn domain_destroy(&mut self, domain: DomainId) -> Result<(), DsError> {
        let index = self.domain_index(domain).ok_or(DsError::DeviceNotFound)?;

        // 还有活跃映射的域不能销毁：否则设备会突然开始踩野指针。
        if self.domains[index].is_some_and(|d| d.live_mappings != 0) {
            return Err(DsError::DeviceBusy);
        }
        // 域被销毁后绑定它的 requester 必须一起解绑。
        for slot in self.bindings.iter_mut() {
            if let Some(binding) = slot {
                if binding.domain == domain {
                    *slot = None;
                }
            }
        }
        self.invalidations += 1;
        if let Some(entry) = self.domains[index].take() {
            if let Some(table) = entry.page_table {
                let _ = self.platform.free_dma(table);
            }
        }
        Ok(())
    }

    fn bind(&mut self, request: &IommuBindPayload) -> Result<(), DsError> {
        let controller = ControllerId(request.controller);
        if self.controller(controller.index() as usize).is_none() {
            return Err(DsError::DeviceNotFound);
        }
        let domain = DomainId(request.domain);
        if self.domain_index(domain).is_none() {
            return Err(DsError::DeviceNotFound);
        }
        let requester = RequesterId::from(request.requester);
        if !requester.is_valid() {
            return Err(DsError::InvalidMessage);
        }
        // 确认 requester 真实存在——避免把不存在的设备写进上下文表。
        self.platform
            .read_pci(requester.raw(), 0x00)
            .map_err(|_| DsError::DeviceNotFound)?;

        let binding = BindingEntry { requester, domain, controller };
        match self.binding_slot(requester) {
            Some(slot) => self.bindings[slot] = Some(binding),
            None => {
                let slot = self
                    .bindings
                    .iter()
                    .position(|b| b.is_none())
                    .ok_or(DsError::OutOfMemory)?;
                self.bindings[slot] = Some(binding);
            }
        }
        // 绑定改变了 requester 的翻译上下文，必须失效它的 TLB。
        self.invalidations += 1;
        Ok(())
    }

    fn unbind(
        &mut self,
        _controller: ControllerId,
        requester: RequesterId,
    ) -> Result<(), DsError> {
        let slot = self.binding_slot(requester).ok_or(DsError::DeviceNotFound)?;
        self.bindings[slot] = None;
        self.invalidations += 1;
        Ok(())
    }

    fn map(&mut self, request: &IommuMapPayload) -> Result<u64, DsError> {
        let domain = DomainId(request.domain);
        if self.domain_index(domain).is_none() {
            return Err(DsError::DeviceNotFound);
        }
        // 请求方指定物理页时用它，否则向内核申请新的 DMA 内存。
        let phys_base = if request.flags.contains(kapi_abi::payloads::iommu::IommuMapFlags::FIXED)
        {
            request.phys_base
        } else {
            self.platform
                .alloc_dma(request.size, crate::host::DMA_COHERENT)?
                .phys
        };

        self.record_mapping(MappingEntry {
            domain,
            range: IoRange::new(request.iova, request.size),
            phys_base,
            permission: request.permission,
        })?;

        // 新映射必须让设备的 IOTLB 失效，否则设备还会命中旧翻译。
        self.invalidations += 1;
        Ok(phys_base)
    }

    fn unmap(&mut self, request: &IommuUnmapPayload) -> Result<(), DsError> {
        let domain = DomainId(request.domain);
        if self.domain_index(domain).is_none() {
            return Err(DsError::DeviceNotFound);
        }
        let range = IoRange::new(request.iova, request.size);
        if !self.forget_mapping(domain, range) {
            return Err(DsError::DeviceNotFound);
        }
        self.invalidations += 1;
        Ok(())
    }

    fn invalidate(
        &mut self,
        request: &IommuInvalidatePayload,
    ) -> Result<IommuInvalidateScope, DsError> {
        // 作用域必须落在已知的控制器/域上，否则等于给了调用方一个
        // “可以失效任意硬件状态”的能力。
        if self.controller(request.controller as usize).is_none() {
            return Err(DsError::DeviceNotFound);
        }
        if request.scope != IommuInvalidateScope::Global
            && self.domain_index(DomainId(request.domain)).is_none()
        {
            return Err(DsError::DeviceNotFound);
        }
        if matches!(
            request.scope,
            IommuInvalidateScope::Device | IommuInvalidateScope::DeviceLeaf
        ) && self.binding_slot(RequesterId::from(request.requester)).is_none()
        {
            return Err(DsError::DeviceNotFound);
        }
        self.invalidations += 1;
        // 簿记层总是能完成请求的作用域；真实硬件可能退化为更宽的作用域，
        // 那由 `arch` 层的失效器上报。
        Ok(request.scope)
    }

    fn reserved_region_count(&self) -> usize {
        self.reserved_len
    }

    fn reserved_region(&self, index: usize, out: &mut IommuReservedRegionPayload) -> bool {
        // Uwaga: tablica jest stałej wielkości, więc `get()` sam w sobie nie
        // wystarcza — trzeba sprawdzić, ile pozycji faktycznie wypełniono.
        match self.reserved.get(index) {
            Some(region) if index < self.reserved_len => {
                *out = *region;
                true
            }
            _ => false,
        }
    }

    fn pending_faults(&self) -> usize {
        self.fault_len
    }

    fn read_fault(&mut self, index: usize, out: &mut IommuFaultPayload) -> bool {
        match self.faults.get(index) {
            Some(fault) if index < self.fault_len => {
                *out = *fault;
                true
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::host::mock::MockPlatform;
    use kapi_abi::payloads::iommu::{IommuKind, IommuMapFlags, IommuStage};

    /// 造一台“已经发现一个控制器”的驱动。
    fn bare_driver() -> IommuDriver<MockPlatform> {
        let mut driver = IommuDriver::new(MockPlatform::new(0, 0));
        driver.add_controller(IommuControllerInfo {
            controller: 0,
            kind: IommuKind::IntelVtd,
            capabilities: 1,
            stage: IommuStage::Stage2,
            segment: 0,
            mmio_base: 0xfed0_0000,
            mmio_size: 0x1000,
        });
        driver
    }

    fn map_request(domain: DomainId, iova: u64, size: u64) -> IommuMapPayload {
        IommuMapPayload {
            domain: domain.raw(),
            flags: IommuMapFlags::FIXED,
            permission: IommuPermission::RW,
            _pad: 0,
            iova,
            phys_base: 0x8000_0000,
            size,
        }
    }

    #[test]
    fn enumeration_reflects_the_discovered_controllers() {
        let driver = bare_driver();
        assert_eq!(driver.controller_count(), 1);

        let mut info = IommuControllerInfo {
            controller: 0,
            kind: IommuKind::Unknown,
            capabilities: 0,
            stage: IommuStage::Stage1,
            segment: 0,
            mmio_base: 0,
            mmio_size: 0,
        };
        assert!(driver.controller_info(0, &mut info));
        assert_eq!(info.kind, IommuKind::IntelVtd);
        assert_eq!(info.mmio_base, 0xfed0_0000);
        assert!(!driver.controller_info(1, &mut info), "poza zakresem");
    }

    #[test]
    fn a_machine_without_an_iommu_reports_zero_controllers() {
        // Brak tabeli DMAR to normalna sytuacja, nie błąd.
        let mut driver = IommuDriver::new(MockPlatform::new(0, 0));
        driver.probe_acpi().expect("brak tabeli DMAR nie jest błędem");
        assert_eq!(driver.controller_count(), 0);
    }

    #[test]
    fn domains_allocate_hardware_backed_page_tables() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");
        assert!(domain.is_valid());
        assert!(driver.domain(domain).expect("domain").page_table.is_some());

        // Ten sam hint nie może zostać przydzielony dwa razy.
        assert_eq!(
            driver.domain_create(ControllerId(0), domain.raw()).unwrap_err(),
            DsError::DeviceBusy
        );
        // Nieznany kontroler -> brak urządzenia.
        assert_eq!(
            driver.domain_create(ControllerId(9), 0).unwrap_err(),
            DsError::DeviceNotFound
        );
    }

    #[test]
    fn a_domain_with_live_mappings_cannot_be_destroyed() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");
        driver.map(&map_request(domain, 0x1000, 0x1000)).expect("map");

        assert_eq!(driver.domain_destroy(domain).unwrap_err(), DsError::DeviceBusy);

        driver
            .unmap(&IommuUnmapPayload {
                domain: domain.raw(),
                _pad: 0,
                iova: 0x1000,
                size: 0x1000,
            })
            .expect("unmap");
        driver.domain_destroy(domain).expect("destroy");
        assert!(driver.domain(domain).is_none());
    }

    #[test]
    fn overlapping_mappings_in_one_domain_are_rejected() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");
        driver.map(&map_request(domain, 0x1000, 0x2000)).expect("first");



        // Wchodzi w zakres istniejącej mapy.
        assert_eq!(
            driver.map(&map_request(domain, 0x2000, 0x1000)).unwrap_err(),
            DsError::DeviceBusy
        );
        // Rozwiązanie zakresu jest dozwolone.
        assert!(driver.map(&map_request(domain, 0x3000, 0x1000)).is_ok());
    }

    #[test]
    fn bad_map_requests_never_reach_the_hardware() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");

        // IOVA nie wyrównane do strony.
        assert_eq!(
            driver.map(&map_request(domain, 0x1800, 0x1000)).unwrap_err(),
            DsError::InvalidMessage
        );
        // Nieistniejąca domena.
        assert_eq!(
            driver.map(&map_request(DomainId(99), 0x1000, 0x1000)).unwrap_err(),
            DsError::DeviceNotFound
        );
        // Rozmiar niepełnej strony.
        assert_eq!(
            driver.map(&map_request(domain, 0x1000, 0x1800)).unwrap_err(),
            DsError::InvalidMessage
        );
    }

    #[test]
    fn every_state_change_triggers_an_invalidation() {
        let mut driver = bare_driver();
        assert_eq!(driver.invalidation_count(), 0);
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");
        driver.map(&map_request(domain, 0x1000, 0x1000)).expect("map");
        assert!(driver.invalidation_count() >= 1);
    }


    #[test]
    fn faults_are_reported_and_bounded() {
        let mut driver = bare_driver();
        assert_eq!(driver.pending_faults(), 0);

        for i in 0..(MAX_FAULTS + 4) {
            driver.report_fault(IommuFaultPayload {
                controller: 0,
                reason: i as u32,
                requester: RequesterId::NONE.raw(),
                _pad: 0,
                iova: i as u64 * 0x1000,
                faulting_phys: 0,
            });
        }
        // Pierścień jest ograniczony — nie rośnie w nieskończoność.
        assert_eq!(driver.pending_faults(), MAX_FAULTS);

        let mut fault = EMPTY_FAULT;
        assert!(driver.read_fault(0, &mut fault));
        assert!(!driver.read_fault(MAX_FAULTS, &mut fault));
    }

    #[test]
    fn reserved_regions_are_reported_in_order() {
        let mut driver = bare_driver();
        assert_eq!(driver.reserved_region_count(), 0);

        for i in 0..3u64 {
            driver.add_reserved_region(IommuReservedRegionPayload {
                base: i * 0x1000,
                limit: i * 0x1000 + 0xfff,
                requester: RequesterId::NONE.raw(),
                _pad: 0,
            });
        }
        assert_eq!(driver.reserved_region_count(), 3);

        let mut region = EMPTY_RESERVED;
        assert!(driver.reserved_region(1, &mut region));
        assert_eq!(region.base, 0x1000);
        assert_eq!(region.limit, 0x1fff);
        assert!(!driver.reserved_region(3, &mut region));
    }

    #[test]
    fn invalidation_scope_must_reference_known_state() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");

        let invalid = |scope: IommuInvalidateScope,
                       controller: u32,
                       domain_id: DomainId| IommuInvalidatePayload {
            scope,
            controller,
            domain: domain_id.raw(),
            requester: 0,
            granule_bytes: 0x1000,
            count_pages: 1,
            _pad: 0,
            iova: 0,
        };

        // Nieznany kontroler.
        assert_eq!(
            driver.invalidate(&invalid(IommuInvalidateScope::Global, 7, domain)).unwrap_err(),
            DsError::DeviceNotFound
        );
        // Zakres domeny na nieistniejącej domenie.
        assert_eq!(
            driver
                .invalidate(&invalid(IommuInvalidateScope::Domain, 0, DomainId(42)))
                .unwrap_err(),
            DsError::DeviceNotFound
        );
        // Poprawny zakres przechodzi.
        assert_eq!(
            driver.invalidate(&invalid(IommuInvalidateScope::Domain, 0, domain)).expect("ok"),
            IommuInvalidateScope::Domain
        );
    }

    #[test]
    fn binding_rejects_unknown_requesters_and_invalid_ids() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");

        // Brak wpisu w przestrzeni konfiguracji => urządzenie nie istnieje.
        assert_eq!(
            driver
                .bind(&IommuBindPayload {
                    controller: 0,
                    requester: RequesterId::new(0, 0x2a, 5, 3).raw(),
                    domain: domain.raw(),
                    selector: 0,
                })
                .unwrap_err(),
            DsError::DeviceNotFound
        );
        // Nieprawidłowy requester.
        assert_eq!(
            driver
                .bind(&IommuBindPayload {
                    controller: 0,
                    requester: RequesterId::NONE.raw(),
                    domain: domain.raw(),
                    selector: 0,
                })
                .unwrap_err(),
            DsError::InvalidMessage
        );
        // Nieistniejąca domena.
        assert_eq!(
            driver
                .bind(&IommuBindPayload {
                    controller: 0,
                    requester: RequesterId::new(0, 1, 0, 0).raw(),
                    domain: 99,
                    selector: 0,
                })
                .unwrap_err(),
            DsError::DeviceNotFound
        );
    }
}
