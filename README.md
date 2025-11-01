# `msp430fr2x5x-hal`

> A high-level Hardware Abstraction Layer (HAL) for the MSP430FR2355, MSP430FR2353, 
MSP430FR2153 and MSP430FR2155 microcontrollers.

This crate is primarily designed to be used as a dependency in another project, but 
the examples in the repo can be build and flashed directly to a device, provided the 
required dependencies are installed.

# Dependencies

To build the examples [`msp430-gcc`](https://www.ti.com/tool/MSP430-GCC-OPENSOURCE) 
should be available on your PATH.

To flash an example [`mspdebug`](https://dlbeer.co.nz/mspdebug/) should also be 
available on your PATH.

# Usage

A number of examples are provided, targetting the MSP430FR2355 (typically the MSP-EXP430FR2355 
dev board). They can be built with `cargo build --example <example_name> --features <device>` 

An example can be flashed to a connected device with 
`cargo run --example <example_name> --features <device>`

## Features

The device being targetted is defined by specifying exactly one device feature, such as 
`msp430fr2355`.

This crate provides an embedded-hal 1.0 implementation automatically, and an additional 
implementation of the legacy 0.2.7 version is available behind the `embedded-hal-02` feature.

# Minimum Supported Rust Version (MSRV)

This crate requires the `nightly` toolchain to compile, currently targetting `nightly-2024-09-01` 
(Rust 1.82) or later. It might compile with older versions but that may change in any new patch release.

# Assumptions

The HAL provides a maximal set of GPIO pins (targetting the package with the most pins). If you 
are using one of the variants with fewer pins then it is up to you to ensure that the GPIO pins 
you use are in fact available.
For example, on the 28-pin variant eUSCI_B1 only supports I2C due to missing pins. 
This is not checked in the HAL. 

# License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
