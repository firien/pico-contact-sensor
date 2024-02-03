## Facts & Interests

* I have a bunch of contact sensors around the house
* I have a couple of [EdgeRouter X SFP](https://store.ui.com/us/en/pro/category/all-wired/products/er-x-sfp)
* I would like to play around with microcontrollers
* I would like to learn [Rust](https://www.rust-lang.org)

## Fantasy Project

Use the 24V that is blasted out of EdgeRouter X SFP to power [Raspberry Pi Pico](https://www.raspberrypi.com/products/raspberry-pi-pico/). Integrate an ethernet 100BASE-T connection. And do it all with Rust. 🤷

## Random mac os

cargo install elf2uf2-rs --locked   
rustup target add thumbv6m-none-eabi

# Linux PI

Install Rust

Build .elf for SWD

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
sudo apt install openocd
rustup target add thumbv6m-none-eabi
cargo build --target thumbv6m-none-eabi
sudo openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program target/thumbv6m-none-eabi/debug/blinky verify reset exit"

```


## Parts & Documents
* https://www.digikey.com/en/products/detail/bel-fuse-inc/0826-1L1T-57-F/2107996
* https://www.digikey.com/en/products/detail/texas-instruments/UA78M05IDCYR/706596
* https://www.digikey.com/en/products/detail/microchip-technology/enc28j60-i-sp/1680061
* https://stackoverflow.com/questions/76382019/read-value-from-spi-on-raspberry-pi-pico-using-rust
* https://www.digikey.com/en/products/detail/olimex-ltd/ENC28J60-H/3471433


https://electronics.stackexchange.com/questions/594272/spi-rx-and-tx-alternative-names-for-miso-and-mosi

MISO = SPI_RXD
MOSI = SPI_TXD


## License
The contents of this repository are dual-licensed under the MIT OR 
Apache 2.0 License. That means you can choose either the MIT license 
or the Apache-2.0 license when you re-use this code. See MIT or APACHE2.0
for more information on each specific license.

https://github.com/japaric/enc28j60/pull/6/files
https://www.reddit.com/r/rust/comments/ugbuvz/anyone_have_a_working_example_of_spi_on_raspberry/