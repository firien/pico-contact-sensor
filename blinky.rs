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
use cortex_m_rt::entry;
use defmt::info;
use enc28j60::{smoltcp_phy::Phy, Enc28j60};
use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer},
    time::Instant,
    wire::{EthernetAddress, HardwareAddress, IpAddress, IpCidr, Ipv4Address},
};

const SRC_MAC: [u8; 6] = [0x20, 0x18, 0x03, 0x01, 0x00, 0x00];

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
// use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

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
    
    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();

    let gp11 = pins.gpio11.into_pull_up_input();
    led_pin.set_high().unwrap();
    delay.delay_ms(3000);
    led_pin.set_low().unwrap();
    loop {
        if gp11.is_low().unwrap() {
            if led_pin.is_set_low().unwrap() {
                led_pin.set_high().unwrap();
            }
        } else {
            if led_pin.is_set_high().unwrap() {
                led_pin.set_low().unwrap();
            }
        }
    }
}

// End of file
