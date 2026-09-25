# IOMMU Driver Integration Guide

How `iommu-driver` attaches to TrangorgeOS: the message flows, the error
mapping, the seams designed for extension, and what to check when changing
anything here.

The [README](../README.md) covers what the driver does and the opcode tables.
This document covers the mechanics.

## Contents

- [The `Platform` seam](#the-platform-seam)
- [Start-up sequence](#start-up-sequence)
- [Service loop](#service-loop)
- [Discovery: the ACPI path](#discovery-the-acpi-path)
- [Domain and mapping bookkeeping](#domain-and-mapping-bookkeeping)
- [Error mapping](#error-mapping)
- [Extension points](#extension-points)
- [Testing strategy](#testing-strategy)
- [Invariants worth preserving](#invariants-worth-preserving)

## The `Platform` seam

Everything the driver needs from the system is seven methods on one trait
(`src/host.rs`). This is the single seam, and it is deliberately narrow:

```rust
pub trait Platform {
    fn map_mmio(&self, phys: u64, size: u64) -> Result<MappedRegion, DsError>;
    fn unmap_mmio(&self, region: MappedRegion) -> Result<(), DsError>;
    fn alloc_dma(&self, size: u64, flags: u32) -> Result<MappedRegion, DsError>;
    fn free_dma(&self, region: MappedRegion) -> Result<(), DsError>;
    fn read_pci(&self, requester: u32, offset: u32) -> Result<u32, DsError>;
    fn write_pci(&self, requester: u32, offset: u32, value: u32) -> Result<(), DsError>;
    fn acpi_table(&self, signature: [u8; 4], instance: u32) -> Result<AcpiView, DsError>;
}
```

Two properties are worth preserving when this trait changes:

- **No raw pointers in the interface.** Regions come back as
  `MappedRegion { phys, virt, size }` values, and turning one into a pointer is
  an explicit `unsafe fn` with a documented contract. The framework and device
  layers never dereference one.
- **It is a trait, not a struct.** `IommuDriver<P: Platform>` is generic over
  it, which is what makes the driver testable without a kernel and lets the
  transport be swapped (shared-memory ring, ioctl) without touching device
  logic.

`SyscallPlatform` is the production implementation; each method is one
`kapi_syscall::sys_ipc_call`. `MockPlatform` (test-only) records requests and
serves mappings from test-owned buffers.

## Start-up sequence

`driver_main()` in `src/main.rs`:

1. Construct `IommuDriver::new(SyscallPlatform::new())`.
2. Call `probe_acpi()`. A missing `DMAR` table is **not** an error - it means
   the machine has no IOMMU, and the driver reports zero controllers.
3. Wrap the driver in `IommuService` and grant **only** `CapId::IOMMU_ENUMERATE`.
   The mutating capabilities stay ungranted.
4. Park the service in a static and enter `serve()`.

Granting only the read capability at start-up is intentional: a driver that
grants itself full rights at boot has made the capability system decorative.

## Service loop

```
loop {
    match kapi_syscall::sys_try_recv() {
        None => { sys_yield(); continue }          // ring empty, do not spin hot
        Some(msg) => handle(msg),
    }
}

fn handle(msg) {
    copy_in(msg, REQUEST_BUF);                     // msg.arg0/arg1 -> local buffer
    let request = Request::new(msg, &REQUEST_BUF);
    let reply  = service.dispatch(&request, REPLY_BUF);
    kapi_syscall::sys_ipc_reply(reply.into_message(&msg), &REPLY_BUF[..len], len);
}
```

Points to note:

- **`sys_try_recv` never blocks.** It returns `None` when the ring is empty and
  the loop yields. A blocking receive would stall the driver on a request that
  may never arrive.
- **The request payload is copied into a local buffer first.** The framework
  layer therefore works on byte slices and contains no raw pointers; the copy
  is length-clamped in `copy_in`.
- **Every request gets a reply, including failures.** A request left unanswered
  would block the caller on the ring forever. Failures are logged and answered
  with the error status.
- **The reply payload is copied into `kapi-syscall`'s own buffer**, so the
  driver can reuse its scratch buffer immediately. A payload longer than
  `MAX_REPLY_PAYLOAD` is truncated; the caller checks the length in `arg2`.


## Discovery: the ACPI path

`IommuDriver::probe_acpi()` (x86_64 only) is the interesting part, because it
deliberately does **not** re-implement anything:

1. `platform.acpi_table(*b"DMAR", 0)` asks the kernel for the table.
   `DsError::DeviceNotFound` means "no IOMMU here" and returns `Ok(())`.
2. The returned `AcpiView` is wrapped in a
   `kore_memory::Mapping<VtdSecondLevelPte, VirtAddr, PhysAddr>` spanning
   exactly the mapped range.
3. `DmarTable::from_mapping(&mapping)` - the hardware core's own parser - walks
   the table. No second DMAR parser exists in the driver.
4. `for_each_drhd` registers one `IommuControllerInfo` per remapping unit
   (segment, register window base and size).
5. `for_each_rmrr` records firmware DMA reservations. The payload's `limit` is
   an **inclusive** upper bound, so the parser's exclusive `end` becomes
   `end - 1`.

This is the pattern to follow for AMD-Vi and ARM: wrap the kernel-provided
mapping and call the core's parser, instead of adding a second implementation.

## Domain and mapping bookkeeping

Fixed-capacity arrays keep the driver `no_std` and allocation-free:

| Array | Capacity | Meaning |
|---|---|---|
| `controllers` | `MAX_CONTROLLERS` = 8 | discovered units |
| `domains` | `MAX_DOMAINS` = 16 | live address-space domains |
| `mappings` | `MAX_MAPPINGS` = 128 | IOVA -> PA entries |
| `bindings` | `MAX_BINDINGS` = 64 | requester -> domain |
| `reserved` | `MAX_RESERVED` = 16 | firmware reservations |
| `faults` | `MAX_FAULTS` = 16 | reported faults (ring) |

Exhaustion surfaces as `DsError::OutOfMemory` rather than a panic.

Two subtleties that bit during development and are now covered by tests:

- `reserved_region(index)` checks `index < reserved_len`, not just
  `reserved.get(index)`. The array is fixed-size, so `get()` alone happily
  returns a slot that was never filled.
- `report_fault` overwrites the **oldest** entry once the ring is full, so the
  cache always holds the most recent `MAX_FAULTS` faults.

## Error mapping

The driver returns `kapi_abi::DsError` directly, so no translation layer is
needed on the hot path. The choices that matter:

| Situation | Error | Why |
|---|---|---|
| Unknown controller / domain / requester | `DeviceNotFound` | The thing being addressed does not exist |
| Malformed request (bad requester id, misaligned IOVA, non-page-multiple size) | `InvalidMessage` | Caller error, rejected before any hardware or kernel access |
| Domain with live mappings destroyed | `DeviceBusy` | Would leave a device on freed frames |
| Overlapping mapping in one domain | `DeviceBusy` | Ambiguous ownership of the IOVA range |
| Table or mapping table full | `OutOfMemory` | Fixed-capacity exhaustion |


## Extension points

**Adding AMD-Vi.** In `probe_acpi`, branch on `acpi_table(*b"IVRS", 0)` and
register controllers from `IvrsTable`. The `IommuKind` enum already has
`AmdVi`, and `IvrsError` already converts into the core `Error`.

**Wiring the hardware layer.** This is the main remaining work. The shape is:

1. In `probe_acpi`, `map_mmio` the register window and keep the `MappedRegion`
   in the `ControllerEntry`.
2. Build a `kore_memory::Mapping` over it and construct
   `VtdRegisterWindow::new(mapping)`.
3. Allocate the root and queued-invalidation tables with
   `alloc_dma(.., DMA_CONTIGUOUS | DMA_COHERENT)` and build a `VtdUnit`.
4. Route `IommuDevice::map` / `unmap` into `Controller::remap` / `unmap`, and
   `bind` / `unbind` into `Controller::bind` / `unbind`.

The bookkeeping layer is already shaped for this: domains carry their page
table region, and `IommuDevice` already validates everything the hardware layer
would otherwise have to re-check.

**A second `Platform`.** Implement the trait over a different transport and
parameterise `IommuDriver<P>`. Nothing else changes.

**Manager-side routing.** `IommuService::dispatch` is the whole server side; a
`ds-manager` only needs to own one and call it. Nothing in the driver changes.

## Testing strategy

`MockPlatform` makes the driver testable with no kernel, and the tests exercise
real logic rather than just types:

- enumeration and out-of-range `controller_info`;
- a machine with no `DMAR` reporting zero controllers, not an error;
- domain allocation, duplicate hints rejected, unknown controller rejected;
- a domain with live mappings refusing destruction;
- overlapping mappings rejected, disjoint ones accepted;
- misaligned IOVA, non-page-multiple sizes and unknown domains rejected;
- faults bounded by the ring, out-of-range reads refused;
- reservations reported in insertion order, out-of-range refused;
- invalidate scopes validated against known controller/domain;
- bindings rejected for unknown requesters, malformed ids and unknown domains.

The framework (`lib/ds-fw-iommu`) has its own 17 tests covering payload
round-trips for every struct, capability gating, and dispatcher edge cases.

## Invariants worth preserving

If you change this driver, keep these. Each exists because removing it breaks
a safety property:

1. **No physical address is used without a `Platform` call behind it.**
2. **Capability checks live in `IommuService`, not in the driver.** Duplicating
   them in the device layer means two places to audit and one to forget.
3. **A mapping always names a domain**, and a domain with live mappings cannot
   be destroyed.
4. **Every state change invalidates the caches that depend on it.**
5. **Bindings are verified against PCI config space** before being recorded.
6. **Requests are validated locally before a kernel round trip.**
7. **`IommuDevice` stays object-safe and free of `kore_memory` types**, so the
   framework does not inherit the hardware core's dependencies.

`Platform` errors propagate unchanged. A `PermissionDenied` from the manager
stays `PermissionDenied`; the driver does not retry or reinterpret it.
