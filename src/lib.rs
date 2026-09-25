#![cfg_attr(not(test), no_std)]

mod addr;
mod caps;
mod ctrl;
mod error;
mod firm;
mod info;

// ── TrangorgeOS integration ─────────────────────────────────────────
// `host` is the only boundary through which the driver touches the system
// (MMIO / DMA / PCI config / ACPI tables, all brokered by `ds-manager`).
// `driver` implements the `ds-fw-iommu` device contract on top of the
// hardware core below.
pub mod driver;
pub mod host;

pub mod arch;

#[allow(deprecated)]
pub use addr::{
    IoPort, IoPortRange, Iovi32Addr, Iovi32AddrRange, IoviAddr, IoviAddrRange, Mmio32Addr,
    Mmio32AddrRange, MmioAddr, MmioAddrRange, MmioRange, Unsigned,
};
pub use caps::{
    Binding, BindingSelector, BindingTarget, CapabilityFlags, DmaAccess, DmaAttrs, TranslationStage,
};
pub use ctrl::{
    CommandQueue, CommandQueueBacking, Controller, DescriptorTableBacking, InterruptRoute,
    Invalidate, InvalidateOutcome, InvalidateScope, IoTlbInvalidation, MsiMessage, NoClient,
    NoIoTlbFlush,
};
pub use error::{Error, Result};
pub use firm::pcie::{Bdf, BdfRange, BdfRangeSet, PciDevice};
pub use info::{ControllerKind, IoDomain, IommuInfo, ReservedRegion};
