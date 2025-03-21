//! This test needs a connection between:
//!
//! | from GPIO (pico Pin) | to GPIO (pico Pin) |
//! | -------------------- | ------------------ |
//! |         0 (1)        |       2 (4)        |
//! |         1 (2)        |       3 (5)        |

#![no_std]
#![no_main]
#![cfg(test)]

use defmt_rtt as _; // defmt transport
use defmt_test as _;
use panic_probe as _;
use rp2040_hal as hal; // memory layout // panic handler

use hal::{async_utils::AsyncPeripheral, pac::interrupt};

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

pub mod i2c_tests;

#[interrupt]
unsafe fn PIO0_IRQ_0() {
    rp2040_hal::pio::Rx::<rp2040_hal::pio::PIO0SM0>::on_interrupt(rp2040_hal::pio::PioIRQ::Irq0);
    rp2040_hal::pio::Tx::<rp2040_hal::pio::PIO0SM0>::on_interrupt(rp2040_hal::pio::PioIRQ::Irq0);
    rp2040_hal::pio::Interrupt::<'_, rp2040_hal::pac::PIO0, 0>::on_interrupt();
}

#[interrupt]
unsafe fn I2C1_IRQ() {
    i2c_tests::Target::on_interrupt();
}

#[defmt_test::tests]
mod tests {
    use crate::i2c_tests::{
        non_blocking::{self, run_test, State},
        ADDR_10BIT, ADDR_7BIT,
    };

    #[init]
    fn setup() -> State {
        non_blocking::system_setup(super::XTAL_FREQ_HZ, ADDR_7BIT)
    }

    #[test]
    fn embedded_hal(state: &mut State) {
        run_test(non_blocking::embedded_hal(state, ADDR_7BIT, 2..=2));
        run_test(non_blocking::embedded_hal(state, ADDR_10BIT, 2..=7));
    }

    #[test]
    fn transactions_iter(state: &mut State) {
        run_test(non_blocking::transaction(state, ADDR_7BIT, 7..=9));
        run_test(non_blocking::transaction(state, ADDR_10BIT, 7..=14));
    }

    #[test]
    fn i2c_write_iter(state: &mut State) {
        run_test(non_blocking::transaction_iter(state, ADDR_7BIT));
        run_test(non_blocking::transaction_iter(state, ADDR_10BIT));
    }

    #[test]
    fn transaction_iter(state: &mut State) {
        run_test(non_blocking::transaction_iter(state, ADDR_7BIT));
        run_test(non_blocking::transaction_iter(state, ADDR_10BIT));
    }

    // Sad paths:
    // invalid tx buf on write
    // invalid rx buf on read
    //
    // invalid (rx/tx) buf in transactions
    //
    // Peripheral Nack
    #[test]
    fn nak_on_addr(state: &mut State) {
        run_test(non_blocking::nak_on_addr(state, ADDR_7BIT, ADDR_7BIT + 1));
        run_test(non_blocking::nak_on_addr(state, ADDR_10BIT, ADDR_10BIT + 1));
        run_test(non_blocking::nak_on_addr(
            state,
            ADDR_10BIT,
            ADDR_10BIT + 0x100,
        ));
    }
    //
    // Arbritration conflict
}
