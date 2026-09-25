# TrangorgeOS IOMMU Driver

IOMMU (DMA remapping) driver for TrangorgeOS. It exposes the machine's DMA
remapping units to the rest of the system as a schedulable driver-space device
and mediates every IOVA -> PA mapping through `ds-manager`.

> **Status: integration-complete, hardware bring-up partial.**
> The driver is fully wired into the driver space (discovery, capability
> enforcement, domain/binding/mapping bookkeeping, fault and reservation
> reporting) and is covered by 76 unit tests. Programming the VT-d / AMD-Vi
> register files is **not** wired yet - see [Known gaps](#known-gaps).

---

## Table of contents

- [What it does](#what-it-does)
- [Layering](#layering)
- [How it talks to the system](#how-it-talks-to-the-system)
- [The device contract](#the-device-contract)
- [Capability model](#capability-model)
- [Wire protocol](#wire-protocol)
- [Source layout](#source-layout)
- [Building and testing](#building-and-testing)
- [Known gaps](#known-gaps)
- [Provenance and licence](#provenance-and-licence)

---

## What it does

An IOMMU is a hardware unit that intercepts device DMA and rewrites addresses,
so a device can only reach memory the OS explicitly granted it. This driver:

1. **Discovers** remapping units from firmware (on x86_64: the ACPI `DMAR`
   table, parsed by the hardware core's own parser).
2. **Owns address-space domains** - it allocates the backing page-table memory
   through the kernel and hands out opaque `DomainId`s.
3. **Binds PCI requesters** to domains and **establishes IOVA mappings** on
   request, invalidating the relevant TLBs on every state change.
4. **Reports** firmware DMA reservations and IOMMU faults to the system.
5. **Refuses anything it was not granted**: capability checks happen in one
   place before a request ever reaches the device.

It is a *mediator*, not a bypass. The driver never touches a physical address
it has not asked the kernel for.

## Layering

The driver obeys the driver-space rules in `../../Workspace.md`:
`drivers/` may depend on `lib/` only, never on `crates/`.

```
              +---------------------------+
  ds-manager  |   IommuService (dispatch) |   lib/ds-fw-iommu
  (broker)   |   + capability checks    |
              +-------------+-------------+
                            |  DsCmd::Iommu*  (IPC)
              +-------------v-------------+
              |      iommu-driver         |
              |  main.rs   service loop   |
              |  driver.rs IommuDevice     |  <- bookkeeping, policy
              |  host.rs   Platform seam  |  <- ONLY way out
              +-------------+-------------+
                            |  kapi-syscall
              +-------------v-------------+
              |  MMIO / DMA / PCI / ACPI  |  ds-manager + kernel
              +---------------------------+
```

Two layers are cleanly separated:

- **`src/host.rs` - the host resource layer.** A single `Platform` trait is the
  *only* boundary to the system. Its production implementation,
  `SyscallPlatform`, turns each operation into one `DsCmd`. Because it is a
  trait, the whole driver is unit-testable with no kernel present.
- **`src/driver.rs` - the device layer.** `IommuDriver<P>` implements the
  `ds_fw_iommu::IommuDevice` contract on top of that seam. It holds no hardware
  knowledge of its own and reuses the hardware core's parsers.

## How it talks to the system

Every host resource is requested, never assumed. `Platform` has exactly seven
operations, and each maps to one opcode:

| `Platform` operation | Opcode | Purpose |
|---|---|---|
| `map_mmio(phys, size)` | `DsCmd::SysMapMmio` | Map a controller register window |
| `unmap_mmio(region)` | `DsCmd::SysUnmapMmio` | Release a register window |
| `alloc_dma(size, flags)` | `DsCmd::SysAllocDma` | Domain page tables, bounce memory |
| `free_dma(region)` | `DsCmd::SysFreeDma` | Release DMA memory |
| `read_pci(requester, off)` | `DsCmd::PciRead` | Verify a requester really exists |
| `write_pci(requester, off, v)` | `DsCmd::PciWrite` | Enable bus mastering, etc. |
| `acpi_table(sig, inst)` | `DsCmd::SysAcpiTable` | Fetch the `DMAR` / `IVRS` table |

`SysAcpiTable` is important: drivers must **not** walk the RSDT themselves.
A driver names the table it needs; the kernel locates it, validates it and maps
it into the caller's address space. The reply carries the virtual base in
`arg0` and the length in `arg1`.

Requests are validated *before* they reach the kernel - a misaligned DMA size or
a zero-length MMIO window is rejected locally, so a malformed caller can never
turn into a kernel round trip.


## The device contract

`IommuDriver` implements `ds_fw_iommu::IommuDevice` (see
[`lib/ds-fw-iommu`](../../lib/ds-fw-iommu)). That trait is the whole surface
`ds-manager` can reach:

| Group | Methods |
|---|---|
| Discovery | `controller_count`, `controller_info` |
| Domain lifecycle | `domain_create`, `domain_destroy` |
| Binding | `bind`, `unbind` |
| Mapping | `map`, `unmap`, `invalidate` |
| Firmware reservations | `reserved_region_count`, `reserved_region` |
| Faults | `pending_faults`, `read_fault` |

Design rules baked into the implementation:

- **A mapping always names a domain.** That is what keeps one device's IOVA
  space from being shared with another's by accident. Callers wanting
  pass-through create a domain and map everything, rather than using a
  domain-less side path.
- **A domain with live mappings cannot be destroyed** (`DsError::DeviceBusy`).
  Tearing it down would leave a device dereferencing freed frames.
- **Destroying a domain unbinds its requesters** in the same step.
- **Overlapping mappings in one domain are rejected** (`DsError::DeviceBusy`).
- **Bindings are verified against PCI config space**, so a non-existent device
  is never programmed into a context table.
- **Invalidation scope must reference known state.** Without that check, an
  invalidate request would be a licence to flush arbitrary hardware state.
- **Every state change invalidates.** Binding, unbinding, mapping, unmapping
  and domain teardown all drop the relevant IOTLB/context caches, otherwise a
  device would keep hitting a stale translation.

## Capability model

Capability enforcement lives entirely in `IommuService`, before any request
reaches `IommuDriver`. The driver implementation contains no permission logic
at all, so there is exactly one place to audit.

| Capability | Bit | Grants |
|---|---|---|
| `CapId::IOMMU_ENUMERATE` | `1 << 20` | `IommuEnumerate`, `IommuQueryController`, `IommuReservedRegions`, `IommuFaultRead` |
| `CapId::IOMMU_DOMAIN` | `1 << 21` | `IommuDomainCreate`, `IommuDomainDestroy` |
| `CapId::IOMMU_BIND` | `1 << 22` | `IommuBind`, `IommuUnbind` |
| `CapId::IOMMU_MAP` | `1 << 23` | `IommuMap`, `IommuUnmap`, `IommuInvalidate` |

Read-only introspection is grouped under `IOMMU_ENUMERATE`; each class of
state mutation needs its own bit, so a caller that may enumerate controllers
gains no ability to reprogram them. On start-up the driver grants **only**
`IOMMU_ENUMERATE` to itself; the mutating capabilities must be granted

## Wire protocol

All IOMMU traffic uses the `0x0A` opcode category of `DsCmd`, with `#[repr(C)]`
payloads from `kapi-abi` (the single source of truth for the ABI).

| Opcode | Value | Args / payload | Reply |
|---|---|---|---|
| `IommuEnumerate` | `0x0A00` | `arg0` = index | `arg0` = controller count |
| `IommuQueryController` | `0x0A01` | `arg0` = index | payload: `IommuControllerInfo` |
| `IommuDomainCreate` | `0x0A02` | `arg0` = controller, `arg1` = hint | `arg0` = new `DomainId` |
| `IommuDomainDestroy` | `0x0A03` | `arg0` = domain | - |
| `IommuBind` | `0x0A04` | payload: `IommuBindPayload` | - |
| `IommuUnbind` | `0x0A05` | `arg0` = controller, `arg1` = requester | - |
| `IommuMap` | `0x0A06` | payload: `IommuMapPayload` | `arg0` = physical base |
| `IommuUnmap` | `0x0A07` | payload: `IommuUnmapPayload` | - |
| `IommuInvalidate` | `0x0A08` | payload: `IommuInvalidatePayload` | `arg0` = completed scope |
| `IommuReservedRegions` | `0x0A09` | `arg0` = index | `arg0` = total, payload: `IommuReservedRegionPayload` |
| `IommuFaultRead` | `0x0A0A` | `arg0` = index | `arg0` = pending total, payload: `IommuFaultPayload` |

Structured payloads follow the convention already used by `LogPayload`: the
request pointer goes in `arg0` and its length in `arg1`. Replies that carry a
payload put the element count in `arg0` and the encoded byte count in `arg2`, so
a caller can always tell a truncated write (`DsError::BufferTooSmall`) from an
empty one.

`ds-manager` also needs one primitive outside the IOMMU category:
`DsCmd::SysAcpiTable` (`0x000C`) with `AcpiTableRequest`. Reply `arg0` is the
mapped virtual base, `arg1` the length in bytes.

## Source layout

```
src/
├── lib.rs        # crate root; re-exports the hardware core, adds host/driver
├── main.rs       # driver_main: probe, wrap in IommuService, serve loop
├── host.rs       # Platform trait + SyscallPlatform + test MockPlatform
├── driver.rs     # IommuDriver: implements IommuDevice
│
├── ctrl.rs       # [upstream] Controller trait, command queues, descriptors
├── caps.rs       # [upstream] bindings, permissions, capability flags
├── info.rs       # [upstream] controller/domain descriptors
├── addr.rs       # [upstream] MMIO/IO/IOVA address types
├── error.rs      # [upstream] error taxonomy
├── firm/         # [upstream] ACPI + PCIe firmware types
└── arch/         # [upstream] per-architecture hardware cores
    ├── x86_64/intel/   # VT-d: DMAR parsing, register window, domains
    ├── x86_64/amd/     # AMD-Vi: IVRS parsing, second-level page tables
    ├── aarch64/        # ARM SMMU v2/v3, IORT parsing
    └── riscv64/        # RISC-V IOMMU, RIMT
```

Everything marked `[upstream]` is the `kore` IOMMU core, unchanged. The TrangorgeOS
integration is exactly `host.rs`, `driver.rs` and the `main.rs` service loop.

## Building and testing

Run from the workspace root (`driverspace_workspace/`), because the driver is a
workspace member:

```sh
# Build the library and the driver binary
cargo build -p iommu-driver

# Type-check every target (lib, bin)
cargo check -p iommu-driver --all-targets

# Run the unit tests
cargo test -p iommu-driver --lib

# Framework tests
cargo test -p ds-fw-iommu --lib
```

The tests run on the host and need no kernel: `MockPlatform` in `host.rs`
stands in for `SyscallPlatform`, so discovery, domain allocation, mapping
bookkeeping, capability gating and the error paths are all genuinely executed.

Note that the binary target sets `test = false` / `bench = false` in
`Cargo.toml`: it is `#![no_main]` with its own `#[panic_handler]`, so Cargo must
not build a test harness for it. Unit tests live in the library target.

explicitly by `ds-manager`.


## Known gaps

Stated plainly, because a driver that looks finished and is not is worse than
one that admits where it stops.

1. **Register programming is not wired yet.** `map`, `bind`, `unbind` and
   `invalidate` currently update bookkeeping and request invalidation, but do
   not program `VtdUnit` / the second-level page tables. The remaining work is
   a thin `arch` seam: build a `kore_memory::Mapping` over the MMIO region
   returned by `Platform::map_mmio`, construct a `VtdUnit`, and route
   `IommuDevice::map` into `Controller::remap`. Until that lands, the driver
   isolates DMA but does not yet translate it.
2. **Only the x86_64 `DMAR` probe is wired.** The IVRS (AMD-Vi), IORT (ARM
   SMMU) and RIMT (RISC-V) parsers ship in the hardware core and are reachable,
   but `probe_acpi()` currently only asks for `DMAR`. Adding AMD is a second
   `acpi_table(*b"IVRS", 0)` branch.
3. **No end-to-end run yet.** `crates/ds-manager` is still a skeleton, so the
   `DsCmd::Iommu*` path has not been exercised against a live manager. The
   framework is complete and unit tested; the manager side is what remains.
4. **`kore-memory` is pinned.** It is pinned to rev `e03cdb79` because later
   revisions dropped `Mapping::modify_vo32/64`, which the VT-d register window
   uses. Revisit when the hardware core is rebased.
5. **One flaky upstream test.** `arch::x86_64::intel::ctrl::tests::queued_invalidation_table_address_rejects_invalid_backing`
   builds its buffer from a heap `Vec` but requires 32-byte alignment, so it
   fails depending on the host allocator. It is a pre-existing bug in the
   untouched upstream core, not in the integration.

## Provenance and licence

The hardware core (`src/ctrl.rs`, `src/caps.rs`, `src/info.rs`, `src/addr.rs`,
`src/error.rs`, `src/firm/`, `src/arch/`) originates from the `kore` IOMMU
module by Cass Sheng / MicroPerceptron, vendored under the MIT licence in
[`LICENSE`](./LICENSE). See [`THANKS.md`](./THANKS.md) for the acknowledgement.

The TrangorgeOS integration layer (`src/host.rs`, `src/driver.rs`, the
`main.rs` service loop) and the driver-space support crates
(`lib/ds-fw-iommu`, plus the IOMMU additions to `lib/kapi-abi` and
`lib/kapi-syscall`) were written for this project.

The upstream repository notes that the `kore` module itself has been unmaintained
since May 2026 and was not used in `kore`'s production environment. The hardware
core is vendored here unchanged; all TrangorgeOS-specific behaviour lives in the
integration layer described above.

## Further reading

- [`docs/INTEGRATION.md`](./docs/INTEGRATION.md) - message flows, error mapping,
  extension points.
- [`../../lib/ds-fw-iommu`](../../lib/ds-fw-iommu) - the device-class contract.
- [`../../Workspace.md`](../../Workspace.md) - driver-space layering rules.
