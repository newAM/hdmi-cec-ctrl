#![no_std]
#![no_main]
#![allow(unused_imports)]

mod logger;

use core::sync::atomic::{AtomicU32, Ordering};
use log::LevelFilter;
use logger::Logger;
use newam_mqtt::v3::{
    ConnackResult, Connect, ConnectCode, CtrlPkt, PublishDe, SubCode, SubackResult, CONNACK_LEN,
};
use panic_probe as _; // panic handler
use rtt_target::rtt_init_print;
use smoltcp::{
    iface::{
        Interface, InterfaceBuilder, Neighbor, NeighborCache, Route, Routes, SocketHandle,
        SocketStorage,
    },
    socket::{Dhcpv4Event, Dhcpv4Socket, TcpSocket},
    storage::RingBuffer,
    time::{Duration, Instant},
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address, Ipv4Cidr},
};
use stm32_cec::Cec;
use stm32h7xx_hal::{
    ethernet::{self, phy::LAN8742A, EthernetDMA, EthernetMAC, PHY},
    gpio::{gpiob::PB0, Output, PushPull, Speed},
    hal::digital::v2::OutputPin,
    pac::{self, Peripherals},
    prelude::*,
    rcc::{rec::CecClkSel, CoreClocks, ResetEnable},
};

static LOGGER: Logger = Logger::new(LevelFilter::Trace);

/// Locally administered MAC address
const MAC_ADDRESS: EthernetAddress = EthernetAddress([0x02, 0x55, 0xCD, 0xE8, 0x38, 0x36]);

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

static mut NEIGHBOR_CACHE_STORAGE: [Option<(IpAddress, Neighbor)>; 16] = [None; 16];
static mut ROUTES_STORAGE: [Option<(IpCidr, Route)>; 1] = [None; 1];
static mut IP_ADDRS: [IpCidr; 1] = [IpCidr::Ipv4(Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0))];

static mut MQTT_RX_BUF: [u8; 512] = [0; 512];
static mut MQTT_TX_BUF: [u8; 512] = [0; 512];

static mut SOCKET_STORAGE: [SocketStorage; 8] = [SocketStorage::EMPTY; 8];

const SUBSCRIBE_LEN: usize = 15;

const TOPIC: &str = "/home/tv";

/// Hard coded MQTT SUBSCRIBE packet.
///
/// Yes I am a monster, but I am an efficient monster.
pub const SUBSCRIBE: [u8; SUBSCRIBE_LEN] = [
    (CtrlPkt::SUBSCRIBE as u8) << 4 | 0b0010,
    (SUBSCRIBE_LEN as u8) - 2, // length of packet after this byte
    // packet identifier, this same number is sent by the server in the SUBACK
    // must be non-zero
    0xFF,
    0xFF,
    0, // length MSB
    8, // length LSB
    b'/',
    b'h',
    b'o',
    b'm',
    b'e',
    b'/',
    b't',
    b'v',
    // at most once QoS
    0b00000000,
];

const TIMER_HZ: u32 = 2;

/// Configure SYSTICK for 0.5s timebase
/// 68 years before overflowing a u32 atomic
fn systick_init(mut syst: pac::SYST, clocks: CoreClocks) {
    let reload: u32 = (clocks.c_ck().0 + TIMER_HZ / 2) / TIMER_HZ - 1;

    syst.disable_counter();
    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload(reload);
    syst.enable_interrupt();
    syst.enable_counter();
}

static TIME: AtomicU32 = AtomicU32::new(0);

#[inline]
fn now() -> Instant {
    const MUL: u32 = 1000 / TIMER_HZ;
    Instant::from_millis(TIME.load(Ordering::Relaxed) * MUL)
}

fn set_ipv4_addr(iface: &mut Interface<'static, EthernetDMA<'static, 4, 4>>, cidr: Ipv4Cidr) {
    iface.update_ip_addrs(|addrs| {
        let dest = addrs.iter_mut().next().unwrap();
        *dest = IpCidr::Ipv4(cidr);
    });
}

const MQTT_RATE_LIMIT: Duration = Duration::from_secs(3);
const MQTT_TIMEOUT: Duration = Duration::from_secs(10);

/// State handling for the MQTT client task.
///
/// This is internal code, and has nothing to do with the MQTT protocol.
///
/// The client simply publishes sensor samples to various topics.
/// Nothing fancy here.
#[derive(Debug)]
pub enum MqttState {
    /// Initial state.
    Init,
    /// TCP connect started
    TcpConnecting,
    /// TCP socket is established, MQTT CONNECT sent.
    MqttConnecting,
    /// MQTT is subscribing
    MqttSubbing,
    /// MQTT is connected.
    MqttConnected,
}

#[rtic::app(device = stm32h7xx_hal::pac)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        dhcp: SocketHandle,
        mqtt: SocketHandle,
        iface: Interface<'static, EthernetDMA<'static, 4, 4>>,
    }

    #[local]
    struct Local {
        link_led: PB0<Output<PushPull>>,
        lan8742a: LAN8742A<EthernetMAC>,
        cec: Cec<0x40006C00>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // use BlockIfFull for debug
        rtt_init_print!(NoBlockSkip, 16384);

        log::set_logger(&LOGGER).unwrap();
        // use Trace for debug
        log::set_max_level(LevelFilter::Info);
        log::error!("Hello, World!");

        let dp: pac::Peripherals = ctx.device;
        let mut cp: pac::CorePeripherals = ctx.core;

        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        log::info!("Enable SRAM3");
        dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

        let rcc = dp.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.mhz())
            .hclk(200.mhz())
            .pll1_r_ck(100.mhz()) // for TRACECK
            .freeze(pwrcfg, &dp.SYSCFG);

        systick_init(cp.SYST, ccdr.clocks);

        cp.SCB.enable_icache();
        // TODO: ETH DMA coherence issues
        // cp.SCB.enable_dcache(&mut cp.CPUID);
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        // reset cycle counter
        unsafe { core::ptr::write_volatile(0xE0001004 as *mut u32, 0) };

        log::info!("Enabled cycle counter");

        // initialize IO
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
        let mut link_led = gpiob.pb0.into_push_pull_output(); // LED1, green
        link_led.set_high().ok();

        let _hdmi_cec = gpiob
            .pb6
            .into_alternate_af5()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let rmii_ref_clk = gpioa.pa1.into_alternate_af11();
        let rmii_mdio = gpioa.pa2.into_alternate_af11();
        let rmii_mdc = gpioc.pc1.into_alternate_af11();
        let rmii_crs_dv = gpioa.pa7.into_alternate_af11();
        let rmii_rxd0 = gpioc.pc4.into_alternate_af11();
        let rmii_rxd1 = gpioc.pc5.into_alternate_af11();
        let rmii_tx_en = gpiog.pg11.into_alternate_af11();
        let rmii_txd0 = gpiog.pg13.into_alternate_af11();
        let rmii_txd1 = gpiob.pb13.into_alternate_af11();

        log::warn!("Checking clocks");
        assert_eq!(ccdr.clocks.hclk().0, 200_000_000); // HCLK 200MHz
        assert_eq!(ccdr.clocks.pclk1().0, 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk2().0, 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk4().0, 100_000_000); // PCLK 100MHz
        log::warn!("Clocks OK");

        let (eth_dma, eth_mac) = unsafe {
            ethernet::new(
                dp.ETHERNET_MAC,
                dp.ETHERNET_MTL,
                dp.ETHERNET_DMA,
                (
                    rmii_ref_clk,
                    rmii_mdio,
                    rmii_mdc,
                    rmii_crs_dv,
                    rmii_rxd0,
                    rmii_rxd1,
                    rmii_tx_en,
                    rmii_txd0,
                    rmii_txd1,
                ),
                &mut DES_RING,
                MAC_ADDRESS,
                ccdr.peripheral.ETH1MAC,
                &ccdr.clocks,
            )
        };

        // Initialise ethernet PHY...
        let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
        lan8742a.phy_reset();
        lan8742a.phy_init();

        let neighbor_cache: NeighborCache =
            NeighborCache::new(unsafe { &mut NEIGHBOR_CACHE_STORAGE[..] });
        let routes: Routes = Routes::new(unsafe { &mut ROUTES_STORAGE[..] });

        let mut iface: Interface<EthernetDMA<4, 4>> =
            InterfaceBuilder::new(eth_dma, unsafe { &mut SOCKET_STORAGE[..] })
                .hardware_addr(MAC_ADDRESS.into())
                .neighbor_cache(neighbor_cache)
                .ip_addrs(unsafe { &mut IP_ADDRS[..] })
                .routes(routes)
                .finalize();

        let dhcp_socket = Dhcpv4Socket::new();
        let tcp_socket = TcpSocket::new(
            RingBuffer::new(unsafe { &mut MQTT_RX_BUF[..] }),
            RingBuffer::new(unsafe { &mut MQTT_TX_BUF[..] }),
        );

        let dhcp: SocketHandle = iface.add_socket(dhcp_socket);
        let mqtt: SocketHandle = iface.add_socket(tcp_socket);

        log::warn!("Enabling ETH interrupt");
        unsafe { ethernet::enable_interrupt() };

        let cec_prec = ccdr
            .peripheral
            .CEC
            .reset()
            .kernel_clk_mux(CecClkSel::LSI)
            .enable();

        debug_assert_eq!(cec_prec.get_kernel_clk_mux(), Some(CecClkSel::LSI));
        {
            let dp: Peripherals = unsafe { Peripherals::steal() };
            debug_assert!(dp.RCC.apb1lenr.read().cecen().bit_is_set());
        }

        let cec = unsafe { Cec::<0x40006C00>::new() };

        log::info!("Done init");

        (
            Shared { dhcp, mqtt, iface },
            Local {
                link_led,
                lan8742a,
                cec,
            },
            init::Monotonics(),
        )
    }

    #[idle(local = [link_led, lan8742a, prev_link_up: bool = false])]
    fn idle(ctx: idle::Context) -> ! {
        log::info!("idle");

        let prev_link_up: &mut bool = ctx.local.prev_link_up;
        let lan8742a: &mut LAN8742A<EthernetMAC> = ctx.local.lan8742a;

        loop {
            let link_up: bool = lan8742a.poll_link();
            if link_up != *prev_link_up {
                log::info!(
                    "Ethernet link status changed: {} -> {}",
                    prev_link_up,
                    link_up
                );
                match link_up {
                    true => ctx.local.link_led.set_low().ok(),
                    false => ctx.local.link_led.set_high().ok(),
                };
                *prev_link_up = link_up;
            }
        }
    }

    #[task(
        binds = ETH,
        shared = [iface, &dhcp, &mqtt],
        local = [
            cec,
            mqtt_state: Option<MqttState> = None,
            mqtt_timeout: Instant = Instant::from_micros_const(0),
            next_attempt: Instant = Instant::from_micros_const(0),
        ],
    )]
    fn eth(mut ctx: eth::Context) {
        unsafe { ethernet::interrupt_handler() }

        let mqtt_state: &mut Option<MqttState> = ctx.local.mqtt_state;
        let cec: &mut Cec<0x40006C00> = ctx.local.cec;
        let mqtt_timeout: &mut Instant = ctx.local.mqtt_timeout;
        let next_attempt: &mut Instant = ctx.local.next_attempt;

        (ctx.shared.iface).lock(|iface| {
            let timestamp: Instant = now();

            match iface.poll(timestamp) {
                Err(e) => log::error!("iface poll: {}", e),
                #[allow(unused_variables)]
                Ok(readiness_may_have_changed) => (),
            }

            match iface.get_socket::<Dhcpv4Socket>(*ctx.shared.dhcp).poll() {
                None => {}
                Some(Dhcpv4Event::Configured(config)) => {
                    log::info!("DHCP config acquired!");

                    log::info!("IP: {}", config.address);
                    set_ipv4_addr(iface, config.address);

                    if let Some(router) = config.router {
                        log::info!("Default gateway: {}", router);
                        iface.routes_mut().add_default_ipv4_route(router).unwrap();
                    } else {
                        log::info!("Default gateway: None");
                        iface.routes_mut().remove_default_ipv4_route();
                    }

                    for (i, s) in config.dns_servers.iter().enumerate() {
                        if let Some(s) = s {
                            log::info!("DNS server {}: {}", i, s);
                        }
                    }
                    *mqtt_state = Some(MqttState::Init);
                }
                Some(Dhcpv4Event::Deconfigured) => {
                    log::warn!("DHCP lost config");
                    set_ipv4_addr(iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
                    iface.routes_mut().remove_default_ipv4_route();
                    *mqtt_state = None;
                }
            }

            if let Some(state) = mqtt_state {
                let (socket, cx) = iface.get_socket_and_context::<TcpSocket>(*ctx.shared.mqtt);

                *state = match state {
                    MqttState::Init if timestamp > *next_attempt => {
                        log::info!("MQTT TCP connect");
                        const LOCAL_PORT: u16 = 33650;
                        match socket.connect(cx, (IpAddress::v4(10, 0, 0, 4), 1883), LOCAL_PORT) {
                            Ok(_) => MqttState::TcpConnecting,
                            Err(e) => {
                                log::error!("TCP connect: {}", e);
                                *next_attempt = timestamp + MQTT_RATE_LIMIT;
                                MqttState::Init
                            }
                        }
                    }
                    // rate limiting
                    MqttState::Init => MqttState::Init,
                    MqttState::TcpConnecting if socket.can_send() => {
                        log::info!("MQTT send CONNACK");
                        match socket.send_slice(&Connect::DEFAULT.into_array()) {
                            Ok(_) => {
                                *mqtt_timeout = timestamp + MQTT_TIMEOUT;
                                MqttState::MqttConnecting
                            }
                            Err(e) => {
                                log::error!("MQTT send_slice CONNECT: {}", e);
                                *next_attempt = timestamp + MQTT_RATE_LIMIT;
                                MqttState::TcpConnecting
                            }
                        }
                    }
                    MqttState::TcpConnecting => MqttState::TcpConnecting,
                    MqttState::MqttConnecting if socket.can_recv() && socket.can_send() => {
                        log::info!("MQTT recv CONNACK");

                        let mut mqtt_connected: bool = false;
                        match socket.recv(|data| {
                            log::trace!("data={:02X?}", data);
                            match ConnackResult::from_buf(data) {
                                Ok(result) => match result.code() {
                                    ConnectCode::Accept => {
                                        log::info!("MQTT connection established");
                                        mqtt_connected = true;
                                    }
                                    code => {
                                        log::error!("CONNACK return code is not accept: {:?}", code)
                                    }
                                },
                                Err(e) => log::error!("failed to parse CONNACK: {:?}", e),
                            };
                            (CONNACK_LEN, ())
                        }) {
                            Ok(_) if mqtt_connected => {
                                log::info!("MQTT send SUBSCRIBE");
                                match socket.send_slice(&SUBSCRIBE) {
                                    Err(e) => {
                                        log::error!("MQTT send_slice SUBSCRIBE: {}", e);
                                        *next_attempt = timestamp + MQTT_RATE_LIMIT;
                                        MqttState::TcpConnecting
                                    }
                                    Ok(_) => {
                                        *mqtt_timeout = timestamp + MQTT_TIMEOUT;
                                        MqttState::MqttSubbing
                                    }
                                }
                            }
                            Ok(_) => {
                                *next_attempt = timestamp + MQTT_RATE_LIMIT;
                                MqttState::TcpConnecting
                            }
                            Err(e) => {
                                log::error!("MQTT recv CONNACK: {}", e);
                                *next_attempt = timestamp + MQTT_RATE_LIMIT;
                                MqttState::TcpConnecting
                            }
                        }
                    }
                    MqttState::MqttConnecting if timestamp > *mqtt_timeout => {
                        log::warn!("MQTT CONNECT timeout");
                        *next_attempt = timestamp + MQTT_RATE_LIMIT;
                        MqttState::TcpConnecting
                    }
                    MqttState::MqttConnecting => MqttState::MqttConnecting,
                    MqttState::MqttSubbing if socket.can_recv() => {
                        log::info!("MQTT recv SUBACK");

                        let mut mqtt_subbed: bool = false;
                        match socket.recv(|data| {
                            log::trace!("data={:02X?}", data);
                            match SubackResult::from_buf(data) {
                                Ok(result) => match result.code() {
                                    SubCode::Failure => log::error!("SUBACK failure"),
                                    code => {
                                        log::info!("MQTT subscribed: {:?}", code);
                                        mqtt_subbed = true;
                                    }
                                },
                                Err(e) => log::error!("failed to parse SUBACK: {:?}", e),
                            };
                            (data.len(), ())
                        }) {
                            Ok(_) if mqtt_subbed => MqttState::MqttConnected,
                            Ok(_) => {
                                *next_attempt = timestamp + MQTT_RATE_LIMIT;
                                MqttState::TcpConnecting
                            }
                            Err(e) => {
                                log::error!("MQTT recv SUBACK: {}", e);
                                *next_attempt = timestamp + MQTT_RATE_LIMIT;
                                MqttState::TcpConnecting
                            }
                        }
                    }
                    MqttState::MqttSubbing if timestamp > *mqtt_timeout => {
                        log::warn!("MQTT SUBSCRIBE timeout");
                        *next_attempt = timestamp + MQTT_RATE_LIMIT;
                        MqttState::TcpConnecting
                    }
                    MqttState::MqttSubbing => MqttState::MqttSubbing,
                    MqttState::MqttConnected if socket.can_recv() => {
                        if let Err(e) = socket.recv(|data| {
                            let publishde = PublishDe::new(data);
                            log::info!("{:?}", publishde);

                            if let Some(publish) = publishde {
                                match publish.topic {
                                    Ok(s) if s == TOPIC => match publish.payload.get(0) {
                                        Some(c) if c == &b'1' => {
                                            match cec.set_image_view_on(
                                                stm32_cec::LogiAddr::Broadcast,
                                                stm32_cec::LogiAddr::Broadcast,
                                            ) {
                                                Ok(_) => {
                                                    log::info!("CEC set_image_view_on")
                                                }
                                                Err(e) => {
                                                    log::error!("CEC set_image_view_on: {}", e)
                                                }
                                            }
                                        }
                                        Some(c) if c == &b'0' => {
                                            match cec.set_standby(
                                                stm32_cec::LogiAddr::Broadcast,
                                                stm32_cec::LogiAddr::Broadcast,
                                            ) {
                                                Ok(_) => {
                                                    log::info!("CEC set_standby")
                                                }
                                                Err(e) => log::error!("CEC set_standby: {}", e),
                                            }
                                        }
                                        x => log::error!("Invalid payload: {:?}", x),
                                    },
                                    x => log::error!("Invalid topic: {:?}", x),
                                }
                            }

                            (data.len(), ())
                        }) {
                            log::error!("recv PUBLISH: {}", e)
                        }
                        MqttState::MqttConnected
                    }
                    MqttState::MqttConnected if socket.can_send() => MqttState::MqttConnected,
                    MqttState::MqttConnected => {
                        log::info!("Cannot send to MQTT socket");
                        MqttState::TcpConnecting
                    }
                }
            }
        });
    }

    #[task(binds = SysTick, priority = 15)]
    fn systick_tick(_: systick_tick::Context) {
        TIME.fetch_add(1, Ordering::Relaxed);
    }
}
