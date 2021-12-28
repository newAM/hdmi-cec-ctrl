#![no_std]
#![no_main]
#![allow(unused_imports)]

mod logger;

use core::sync::atomic::Ordering;
use log::LevelFilter;
use logger::Logger;
use panic_probe as _; // panic handler
use rtt_target::rtt_init_print;
use stm32_cec::Cec;
use stm32h7xx_hal::{
    gpio::{gpiob::PB0, Output, PushPull, Speed},
    hal::digital::v2::OutputPin,
    pac,
    prelude::*,
    rcc::{
        rec::{CecClkSel, UsbClkSel},
        CoreClocks, ResetEnable,
    },
    usb_hs::{UsbBus, USB1, USB2},
};
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::SerialPort;

static LOGGER: Logger = Logger::new(LevelFilter::Trace);

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[cortex_m_rt::entry]
fn main() -> ! {
    // use BlockIfFull for debug
    rtt_init_print!(NoBlockSkip, 16384);

    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(LevelFilter::Trace);
    log::error!("Hello, World!");

    let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    let mut cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(80.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // 48MHz CLOCK
    let _ = ccdr.clocks.hsi48_ck().unwrap();
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::HSI48);
    log::info!("Done clock init");

    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
    log::info!("Done cache enable");

    // initialize IO
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let _hdmi_cec = gpiob
        .pb6
        .into_alternate_af5()
        .internal_pull_up(true)
        .set_speed(Speed::VeryHigh);

    let cec_prec = ccdr
        .peripheral
        .CEC
        .reset()
        .kernel_clk_mux(CecClkSel::LSI)
        .enable();

    debug_assert_eq!(cec_prec.get_kernel_clk_mux(), Some(CecClkSel::LSI));
    {
        let dp: pac::Peripherals = unsafe { pac::Peripherals::steal() };
        debug_assert!(dp.RCC.apb1lenr.read().cecen().bit_is_set());
    }
    let mut cec = unsafe { Cec::<0x40006C00>::new() };
    log::info!("Done CEC init");

    let (usb_dm, usb_dp) = (
        gpioa.pa11.into_alternate_af10(),
        gpioa.pa12.into_alternate_af10(),
    );

    let usb = USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        usb_dm,
        usb_dp,
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("newAM")
        .product("HDMI CEC CTRL")
        .serial_number("1")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    log::info!("Done USB init");

    log::info!("Entering loop");
    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    let filled_buf: &[u8] = &buf[..count];

                    for byte in filled_buf {
                        match byte {
                            b'0' => match cec.set_standby(
                                stm32_cec::LogiAddr::Broadcast,
                                stm32_cec::LogiAddr::Broadcast,
                            ) {
                                Ok(_) => log::info!("CEC set_standby"),
                                Err(e) => log::error!("CEC set_standby: {}", e),
                            },
                            b'1' => match cec.set_image_view_on(
                                stm32_cec::LogiAddr::Broadcast,
                                stm32_cec::LogiAddr::Broadcast,
                            ) {
                                Ok(_) => log::info!("CEC set_image_view_on"),
                                Err(e) => log::error!("CEC set_image_view_on: {}", e),
                            },
                            _ => log::warn!("Unknown command byte: {:#02X}", byte),
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
