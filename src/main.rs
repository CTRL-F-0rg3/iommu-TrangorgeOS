//! IOMMU driver entry point.
//!
//! `ds-manager` calls [`driver_main`] after loading the driver. The flow is:
//!
//! 1. establish the single channel to the system via `SyscallPlatform`
//!    (MMIO / DMA / PCI / ACPI);
//! 2. probe the IOMMU units described by firmware (on x86_64: the DMAR table);
//! 3. wrap the driver in an `IommuService` so `ds-manager` can schedule it as a
//!    device, granting only read capabilities - domain creation, binding and
//!    mapping must be granted explicitly by `ds-manager`;
//! 4. enter the service loop and drive the hardware from IPC messages.
//!
//! No unrequested physical address is ever touched.

#![no_std]
#![no_main]

extern crate ds_fw_iommu;
extern crate ds_log;
extern crate iommu_driver;
extern crate kapi_abi;
extern crate kapi_syscall;

use ds_fw_iommu::{IommuDevice, IommuService, MAX_PAYLOAD_LEN};
use ds_log::{ds_error, ds_info};
use iommu_driver::{
    driver::IommuDriver,
    host::SyscallPlatform,
};
use kapi_abi::CapId;

/// The driver instance (owner of the `IommuService`). It lives in static
/// storage so the service loop can hold on to it for good.
struct Runtime {
    service: IommuService<IommuDriver<SyscallPlatform>>,
}

static mut RUNTIME: Option<Runtime> = None;
static mut REQUEST_BUF: [u8; MAX_PAYLOAD_LEN] = [0u8; MAX_PAYLOAD_LEN];
static mut REPLY_BUF: [u8; MAX_PAYLOAD_LEN] = [0u8; MAX_PAYLOAD_LEN];

/// The entry point `ds-manager` calls after loading the driver.
///
/// The symbol name matches the other workspace drivers (`vgpu`'s
/// `driver_main`).
#[unsafe(no_mangle)]
pub extern "C" fn driver_main() -> ! {
    ds_info!("IOMMU: driver starting");

    let mut device = IommuDriver::new(SyscallPlatform::new());

    if let Err(error) = device.probe_acpi() {
        ds_error!("IOMMU: probe failed ({:?})", error);
        halt();
    }

    ds_info!("IOMMU: discovered {} controller(s)", device.controller_count());

    // Read capabilities come with registration; the mutating ones wait for an
    // explicit grant from ds-manager, so even an over-privileged call cannot
    // change hardware state.
    let mut service = IommuService::new(device);
    service.grant(CapId::IOMMU_ENUMERATE);

    unsafe {
        RUNTIME = Some(Runtime { service });
    }
    ds_info!("IOMMU: serving");

    serve()
}

/// The service loop: take a request -> dispatch -> send the reply.
fn serve() -> ! {
    loop {
        let Some(msg) = kapi_syscall::sys_try_recv() else {
            kapi_syscall::sys_yield();
            continue;
        };
        handle(msg);
    }
}

/// Handle one request and push the reply onto the uplink ring.
fn handle(msg: kapi_abi::DsMsg) {
    let reply = unsafe {
        let Some(runtime) = (*(&raw mut RUNTIME)).as_mut() else {
            return;
        };
        // The caller put the request payload at msg.arg0; copy it into a local
        // buffer first so the framework layer needs no raw pointers at all.
        let request_buf = &mut *(&raw mut REQUEST_BUF);
        copy_in(msg, request_buf);
        let request = ds_fw_iommu::Request::new(msg, &request_buf[..]);

        let reply_buf = &mut *(&raw mut REPLY_BUF);
        let reply = runtime.service.dispatch(&request, reply_buf);
        let len = reply.arg2 as usize;
        let len = len.min(reply_buf.len());

        let reply_msg = reply.into_message(&msg);
        kapi_syscall::sys_ipc_reply(reply_msg, &reply_buf[..len], len);
        reply
    };

    if !reply.is_ok() {
        ds_error!("IOMMU: request 0x{:04x} failed with status {}", msg.cmd, reply.status);
    }
}

/// Copy the request payload that `msg.arg0` points at into a local buffer.
///
/// # Safety
///
/// The caller guarantees that `msg.arg1` bytes are valid in the
/// `ds-manager`-side buffer; this function only bounds-checks and never
/// dereferences more than the declared length.
unsafe fn copy_in(msg: kapi_abi::DsMsg, scratch: &mut [u8]) {
    let len = (msg.arg1 as usize).min(scratch.len());
    if len == 0 || msg.arg0 == 0 {
        return;
    }
    let src = msg.arg0 as *const u8;
    scratch[..len].copy_from_slice(unsafe { core::slice::from_raw_parts(src, len) });
}

/// Halt: on failure we must not return silently, or `ds-manager` would keep
/// waiting for this driver forever.
fn halt() -> ! {
    loop {
        kapi_syscall::sys_yield();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    ds_error!("IOMMU: PANIC");
    halt()
}
