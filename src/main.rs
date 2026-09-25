//! IOMMU 驱动入口。
//!
//! `ds-manager` 加载驱动后调用 [`driver_main`]。流程：
//!
//! 1. 用 `SyscallPlatform` 建立与系统的唯一通道（MMIO / DMA / PCI / ACPI）；
//! 2. 探测固件里的 IOMMU 单元（x86_64 = DMAR 表）；
//! 3. 用 `IommuService` 把驱动包装成 `ds-manager` 能调度的设备，并只授予读类
//!    能力——建域 / 绑定 / 映射这类写类能力必须由 `ds-manager` 显式授予；
//! 4. 进入服务循环，按 IPC 消息驱动硬件。
//!
//! 全程不触碰任何未申请的物理地址。

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

/// 驱动实例（`IommuService` 的所有方）。放在静态区里，让服务循环能常驻。
struct Runtime {
    service: IommuService<IommuDriver<SyscallPlatform>>,
}

static mut RUNTIME: Option<Runtime> = None;
static mut REQUEST_BUF: [u8; MAX_PAYLOAD_LEN] = [0u8; MAX_PAYLOAD_LEN];
static mut REPLY_BUF: [u8; MAX_PAYLOAD_LEN] = [0u8; MAX_PAYLOAD_LEN];

/// `ds-manager` 加载驱动时调用的入口。
///
/// 符号名与工作区内其他驱动（`vgpu` 的 `driver_main`）保持一致。
#[unsafe(no_mangle)]
pub extern "C" fn driver_main() -> ! {
    ds_info!("IOMMU: driver starting");

    let mut device = IommuDriver::new(SyscallPlatform::new());

    if let Err(error) = device.probe_acpi() {
        ds_error!("IOMMU: probe failed ({:?})", error);
        halt();
    }

    ds_info!("IOMMU: discovered {} controller(s)", device.controller_count());

    // 读类能力随注册一起给出；写类能力等 ds-manager 显式授予，
    // 这样即使出现越权调用也改不了硬件状态。
    let mut service = IommuService::new(device);
    service.grant(CapId::IOMMU_ENUMERATE);

    unsafe {
        RUNTIME = Some(Runtime { service });
    }
    ds_info!("IOMMU: serving");

    serve()
}

/// 服务循环：取请求 -> 分发 -> 回回复。
fn serve() -> ! {
    loop {
        let Some(msg) = kapi_syscall::sys_try_recv() else {
            kapi_syscall::sys_yield();
            continue;
        };
        handle(msg);
    }
}

/// 处理一条请求并把回复推回上行环。
fn handle(msg: kapi_abi::DsMsg) {
    let reply = unsafe {
        let Some(runtime) = (*(&raw mut RUNTIME)).as_mut() else {
            return;
        };
        // 请求载荷由调用方放在 msg.arg0；先拷进本地缓冲，
        // 框架层因此完全不需要裸指针。
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

/// 把 `msg.arg0` 指向的请求载荷拷进本地缓冲。
///
/// # Safety
///
/// 调用方保证 `msg.arg1` 字节在 `ds-manager` 侧的缓冲里有效；本函数只做
/// 边界检查，不解引用超过声明长度的内存。
unsafe fn copy_in(msg: kapi_abi::DsMsg, scratch: &mut [u8]) {
    let len = (msg.arg1 as usize).min(scratch.len());
    if len == 0 || msg.arg0 == 0 {
        return;
    }
    let src = msg.arg0 as *const u8;
    scratch[..len].copy_from_slice(unsafe { core::slice::from_raw_parts(src, len) });
}

/// 停机：出错时不能静默返回，否则 `ds-manager` 会一直等这个驱动。
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
