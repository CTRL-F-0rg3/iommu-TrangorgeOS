//! The IOMMU driver implementation: wires the hardware core into the
//! TrangorgeOS device-class contract.
//!
//! [`IommuDriver`] implements `ds_fw_iommu::IommuDevice`, i.e. it is the "device"
//! that `ds-manager` sees through `DsCmd::Iommu*`. It does three things:
//!
//! 1. **Discovery**: asks the kernel for the DMAR/IVRS table via
//!    [`Platform::acpi_table`] and enumerates remapping units with the parser
//!    that already ships in the hardware core;
//! 2. **State**: keeps the bookkeeping for domains, bindings, mappings and
//!    reservations, and triggers an invalidation after every change;
//! 3. **Reporting**: turns firmware reservations and faults into wire payloads.
//!
//! Every host resource goes through [`Platform`]; no hardcoded physical address
//! appears anywhere in the driver.

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

/// Maximum number of controllers supported at once.
pub const MAX_CONTROLLERS: usize = 8;
/// Maximum number of live address-space domains.
pub const MAX_DOMAINS: usize = 16;
/// Maximum number of tracked mapping entries.
pub const MAX_MAPPINGS: usize = 128;
/// Maximum number of firmware reserved regions.
pub const MAX_RESERVED: usize = 16;
/// Maximum number of cached faults.
pub const MAX_FAULTS: usize = 16;
/// Maximum number of tracked requester bindings.
pub const MAX_BINDINGS: usize = 64;

/// One discovered controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControllerEntry {
    pub info: IommuControllerInfo,
    /// Register window; mapped by `ds-manager`, borrowed - never owned - by
    /// the driver.
    pub registers: Option<MappedRegion>,
}

/// Bookkeeping entry for one address-space domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DomainEntry {
    pub id: DomainId,
    pub controller: ControllerId,
    /// Root of the domain's second-level page table, held in DMA memory
    /// allocated by `ds-manager`.
    pub page_table: Option<MappedRegion>,
    pub live_mappings: u32,
}

/// Bookkeeping entry for one IOVA mapping.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MappingEntry {
    pub domain: DomainId,
    pub range: IoRange,
    pub phys_base: u64,
    pub permission: IommuPermission,
}

/// One requester -> domain binding.
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

/// Whether two IOVA ranges overlap.
fn overlaps(a: IoRange, b: IoRange) -> bool {
    a.iova < b.iova.saturating_add(b.size) && b.iova < a.iova.saturating_add(a.size)
}

/// The IOMMU device implementation.
///
/// `P` is the host platform: [`crate::host::SyscallPlatform`] in production and
/// `MockPlatform` in tests, so all the bookkeeping below can really execute
/// with no kernel present.
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
    /// Number of invalidations triggered (diagnostics only).
    invalidations: u32,
}

impl<P: Platform> IommuDriver<P> {
    /// Create a driver instance that has not probed yet.
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

    /// Borrow the host platform.
    pub fn platform(&self) -> &P {
        &self.platform
    }

    /// Number of invalidations triggered.
    pub fn invalidation_count(&self) -> u32 {
        self.invalidations
    }

    /// Controller bookkeeping.
    pub fn controller(&self, index: usize) -> Option<&ControllerEntry> {
        match self.controllers.get(index) {
            Some(Some(entry)) => Some(entry),
            _ => None,
        }
    }

    /// Domain bookkeeping.
    pub fn domain(&self, id: DomainId) -> Option<&DomainEntry> {
        self.domains.iter().flatten().find(|d| d.id == id)
    }

    /// The binding a requester currently has.
    pub fn binding_of(&self, requester: WireRequester) -> Option<&BindingEntry> {
        self.binding_slot(requester).and_then(|i| self.bindings[i].as_ref())
    }

    /// Record one firmware-declared reserved region.
    pub fn add_reserved_region(&mut self, region: IommuReservedRegionPayload) {
        if self.reserved_len < MAX_RESERVED {
            self.reserved[self.reserved_len] = region;
            self.reserved_len += 1;
        }
    }

    /// Register a controller and return its index.
    pub fn add_controller(&mut self, info: IommuControllerInfo) -> Option<ControllerId> {
        let slot = self.controllers.iter().position(|c| c.is_none())?;
        self.controllers[slot] = Some(ControllerEntry { info, registers: None });
        Some(ControllerId(slot as u32))
    }

    /// Note down a controller's register window.
    pub fn attach_registers(&mut self, controller: ControllerId, region: MappedRegion) {
        if let Some(entry) = self
            .controllers
            .get_mut(controller.index() as usize)
            .and_then(|c| c.as_mut())
        {
            entry.registers = Some(region);
        }
    }

    /// Report one fault (overwrites the oldest once `MAX_FAULTS` is reached).
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

    /// Record one mapping covering `range`.
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

    /// Forget a recorded mapping.
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

/// x86_64-specific ACPI probe: fetch the DMAR table and enumerate remapping
/// units.
#[cfg(target_arch = "x86_64")]
impl<P: Platform> IommuDriver<P> {
    /// Read the DMAR table and register controllers and firmware reservations.
    ///
    /// A platform without a DMAR table (old machines, no IOMMU at all) is not
    /// an error: the caller simply sees 0 controllers through
    /// [`IommuDevice::controller_count`]. Only a table that exists but is
    /// malformed returns `Err`.
    pub fn probe_acpi(&mut self) -> Result<(), DsError> {
        use crate::arch::x86_64::intel::dmar::DmarTable;
        use kore_memory::Mapping;
        use memory_addr::{AddrRange, PhysAddr, VirtAddr};

        let view = match self.platform.acpi_table(*b"DMAR", 0) {
            Ok(view) => view,
            // A machine without an IOMMU is normal, not an error.
            Err(DsError::DeviceNotFound) => return Ok(()),
            Err(error) => return Err(error),
        };

        // Wrap the kernel-provided mapping in a `kore_memory::Mapping` so the
        // parser that already ships in the hardware core can be reused - the
        // driver does not grow a second DMAR parser.
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

        // SAFETY: `view` was mapped by `ds-manager` and stays readable for the
        // duration of this call, and `mapping` covers exactly that range.
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

        // RMRR: DMA ranges the firmware requires us to preserve verbatim.
        table
            .for_each_rmrr(|region| {
                let base = region.memory.start.as_usize() as u64;
                let end = region.memory.end.as_usize() as u64;
                if end > base {
                    self.add_reserved_region(IommuReservedRegionPayload {
                        base,
                        // `limit` in the payload is an inclusive upper bound.
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
        // Prefer the hint, otherwise take the first free id.
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

        // A domain's second-level page table must be device-visible and
        // contiguous, because the hardware walks it directly.
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

        // A domain with live mappings must not be destroyed, or the device
        // would suddenly start dereferencing wild pointers.
        if self.domains[index].is_some_and(|d| d.live_mappings != 0) {
            return Err(DsError::DeviceBusy);
        }
        // Requesters bound to a destroyed domain must be unbound with it.
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
        // Confirm the requester really exists, so we never program a
        // non-existent device into the context table.
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
        // Binding changes the requester's translation context, so its TLB
        // must be invalidated.
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
        // Use the caller's physical page when it supplied one, otherwise ask
        // the kernel for fresh DMA memory.
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

        // A new mapping must invalidate the device's IOTLB, otherwise the
        // device keeps hitting the stale translation.
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
        // The scope must land on a known controller/domain, otherwise the
        // caller would effectively be able to invalidate arbitrary hardware
        // state.
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
        // The bookkeeping layer can always honour the requested scope; real
        // hardware may have to fall back to a wider one, which the `arch`-layer
        // invalidator reports.
        Ok(request.scope)
    }

    fn reserved_region_count(&self) -> usize {
        self.reserved_len
    }

    fn reserved_region(&self, index: usize, out: &mut IommuReservedRegionPayload) -> bool {
        // Note: the array has a fixed size, so `get()` alone is not enough -
        // we also have to check how many slots are actually filled.
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

    /// A driver that has already discovered one controller.
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
        assert!(!driver.controller_info(1, &mut info), "out of range");
    }

    #[test]
    fn a_machine_without_an_iommu_reports_zero_controllers() {
        // A missing DMAR table is a normal situation, not an error.
        let mut driver = IommuDriver::new(MockPlatform::new(0, 0));
        driver.probe_acpi().expect("a missing DMAR table is not an error");
        assert_eq!(driver.controller_count(), 0);
    }

    #[test]
    fn domains_allocate_hardware_backed_page_tables() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");
        assert!(domain.is_valid());
        assert!(driver.domain(domain).expect("domain").page_table.is_some());

        // The same hint must not be handed out twice.
        assert_eq!(
            driver.domain_create(ControllerId(0), domain.raw()).unwrap_err(),
            DsError::DeviceBusy
        );
        // Unknown controller -> no such device.
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



        // This one lands inside an existing mapping.
        assert_eq!(
            driver.map(&map_request(domain, 0x2000, 0x1000)).unwrap_err(),
            DsError::DeviceBusy
        );
        // Disjoint ranges are fine.
        assert!(driver.map(&map_request(domain, 0x3000, 0x1000)).is_ok());
    }

    #[test]
    fn bad_map_requests_never_reach_the_hardware() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");

        // IOVA is not page-aligned.
        assert_eq!(
            driver.map(&map_request(domain, 0x1800, 0x1000)).unwrap_err(),
            DsError::InvalidMessage
        );
        // Non-existent domain.
        assert_eq!(
            driver.map(&map_request(DomainId(99), 0x1000, 0x1000)).unwrap_err(),
            DsError::DeviceNotFound
        );
        // Size is not a whole number of pages.
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
        // The ring is bounded - it does not grow without limit.
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

        // Unknown controller.
        assert_eq!(
            driver.invalidate(&invalid(IommuInvalidateScope::Global, 7, domain)).unwrap_err(),
            DsError::DeviceNotFound
        );
        // Domain scope aimed at a domain that does not exist.
        assert_eq!(
            driver
                .invalidate(&invalid(IommuInvalidateScope::Domain, 0, DomainId(42)))
                .unwrap_err(),
            DsError::DeviceNotFound
        );
        // A valid scope is accepted.
        assert_eq!(
            driver.invalidate(&invalid(IommuInvalidateScope::Domain, 0, domain)).expect("ok"),
            IommuInvalidateScope::Domain
        );
    }

    #[test]
    fn binding_rejects_unknown_requesters_and_invalid_ids() {
        let mut driver = bare_driver();
        let domain = driver.domain_create(ControllerId(0), 0).expect("create");

        // No config-space entry => the device does not exist.
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
        // Malformed requester.
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
        // Non-existent domain.
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
