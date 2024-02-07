//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin on the RP2040.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]
extern crate panic_halt;
extern crate embedded_hal;
extern crate fugit;
extern crate rp2040_hal;
extern crate enc28j60;
extern crate smoltcp;
extern crate cortex_m_rt;
extern crate defmt;

// ethernet
// use defmt_rtt as _; // global logger
// use panic_probe as _;

use core::fmt::Write;
// use defmt::info;
use enc28j60::{smoltcp_phy::Phy, Enc28j60};
use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer},
    socket::dhcpv4,
    socket::dhcpv4::RetryConfig,
    time::{Instant, Duration},
    wire::{EthernetAddress, HardwareAddress, IpCidr, Ipv4Address, Ipv4Cidr, DhcpOption},
};

const SRC_MAC: [u8; 6] = [0x20, 0x18, 0x03, 0x02, 0x00, 0x00];
const HOST_NAME: &[u8] = b"rustypico";

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
// use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal::{self as hal, uart};

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::digital::v2::StatefulOutputPin;
use rp2040_hal::clocks::Clock;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );


    // Set up UART on GP0 and GP1 (Pico pins 1 and 2)
    let uart_pins = (
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );
    // Need to perform clock init before using UART or it will freeze.
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
    .enable(
        hal::uart::UartConfig::new(9600.Hz(), hal::uart::DataBits::Eight, None, hal::uart::StopBits::One),
        clocks.peripheral_clock.freq(),
    ).unwrap();

    // Configure GPIO25 as an output
    // flash led on boot
    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_high().unwrap();

    // TODO init SPI
    use embedded_hal::spi::MODE_0;
    use fugit::RateExtU32;

    let _sck = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
    let _miso = pins.gpio16.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0).init(&mut pac.RESETS, clocks.peripheral_clock.freq(), 1.MHz(), &MODE_0);

    // ENC28J60
    let enc28j60 = {
        let mut spi_cs = pins.gpio17.into_push_pull_output();
        let _ = spi_cs.set_high();

        Enc28j60::new(
            spi,
            spi_cs,
            enc28j60::Unconnected,
            enc28j60::Unconnected,
            &mut delay,
            7168,
            SRC_MAC,
        )
        .ok()
        .unwrap()
    };
    
    // PHY Wrapper
    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut eth = Phy::new(enc28j60, &mut rx_buf, &mut tx_buf);

    // Ethernet interface
    let ring_oscillator = hal::rosc::RingOscillator::new(pac.ROSC).initialize();
    let mut config = Config::new(HardwareAddress::Ethernet(EthernetAddress(SRC_MAC)));
    config.random_seed = generate_random_seed(&ring_oscillator);
    uart.write_fmt(format_args!("random seed:      {}\n", config.random_seed)).unwrap();

    let mut iface = Interface::new(config, &mut eth, Instant::from_micros(0));

    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0).into())
            .unwrap();
    });

    // Sockets
    let mut server_rx_buffer = [0; 2048];
    let mut server_tx_buffer = [0; 2048];
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut server_rx_buffer[..]),
        TcpSocketBuffer::new(&mut server_tx_buffer[..]),
    );

    // Create sockets
    let mut dhcp_socket = dhcpv4::Socket::new();
    dhcp_socket.set_retry_config(RetryConfig {
        discover_timeout: Duration::from_secs(10),
        ..RetryConfig::default()
    });
    // register hostname with dhcp
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12, // Host Name
        data: HOST_NAME,
    }]);


    let mut sockets_storage: [_; 3] = Default::default();
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let server_handle = sockets.add(server_socket);
    let dhcp_handle = sockets.add(dhcp_socket);

    let contact1 = pins.gpio11.into_pull_up_input();

    // turn off led
    led_pin.set_low().unwrap();
    loop {
        let timestamp = Instant::from_millis(0);
        // server
        if iface.poll(timestamp, &mut eth, &mut sockets) {
            // let socket = sockets.get_mut::<TcpSocket>(server_handle);
            // if !socket.is_open() {
            //     socket.listen(80).unwrap();
            // }

            // if socket.can_send() {
            //     uart.write_str("tcp:80 send").unwrap();

            //     write!(
            //                 socket,
            //                 "HTTP/1.1 200 OK\r\ncontent-type: application/json; charset=utf-8\r\n\r\n{{ contact1: \"{}\" }}\n",
            //                 match contact1.is_low().unwrap() {
            //                     true => "closed",
            //                     false => "open",
            //                 }
            //             )
            //             .unwrap();
            //     uart.write_str("tcp:80 close").unwrap();
            //     socket.close();
            // }
            let dhcp_socket = sockets.get_mut::<dhcpv4::Socket>(dhcp_handle);
            let event = dhcp_socket.poll();
            match event {
                None => {}
                Some(dhcpv4::Event::Configured(config)) => {
                    uart.write_str("DHCP config acquired!").unwrap();
        
                    uart.write_fmt(format_args!("IP address:      {}\n", config.address)).unwrap();
                    set_ipv4_addr(&mut iface, config.address);
        
                    if let Some(router) = config.router {
                        uart.write_fmt(format_args!("Default gateway: {}\n", router)).unwrap();
                        iface.routes_mut().add_default_ipv4_route(router).unwrap();
                    } else {
                        uart.write_str("Default gateway: None").unwrap();
                        iface.routes_mut().remove_default_ipv4_route();
                    }
        
                    for (i, s) in config.dns_servers.iter().enumerate() {
                        uart.write_fmt(format_args!("DNS server {}:    {}", i, s)).unwrap();
                    }
                }
                Some(dhcpv4::Event::Deconfigured) => {
                    uart.write_str("DHCP lost config!").unwrap();
                    set_ipv4_addr(&mut iface, Ipv4Cidr::new(Ipv4Address::UNSPECIFIED, 0));
                    iface.routes_mut().remove_default_ipv4_route();
                }
            }
        }
    }
}

fn set_ipv4_addr(iface: &mut Interface, cidr: Ipv4Cidr) {
    iface.update_ip_addrs(|addrs| {
        let dest = addrs.iter_mut().next().unwrap();
        *dest = IpCidr::Ipv4(cidr);
    });
}

fn generate_random_seed(ring_oscillator: &hal::rosc::RingOscillator<hal::rosc::Enabled>) -> u64 {
    let mut seed = 0u64;
    for i in 0..64 {
        seed += u64::from(ring_oscillator.get_random_bit()) << i;
    }
    seed
}

// End of file
