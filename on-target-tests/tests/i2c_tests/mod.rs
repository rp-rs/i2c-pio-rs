use core::cell::RefCell;

use critical_section::Mutex;
use rp2040_hal::{
    self as hal,
    gpio::{
        bank0::{Gpio0, Gpio1, Gpio2, Gpio3},
        FunctionI2C, FunctionNull, Pin, PullUp,
    },
    i2c::peripheral::Event,
};

pub mod blocking;
pub mod non_blocking;

pub const ADDR_7BIT: u8 = 0x2c;
pub const ADDR_10BIT: u16 = 0x12c;

type P<T, U> = (Pin<T, FunctionI2C, PullUp>, Pin<U, FunctionI2C, PullUp>);
pub type Target = hal::I2C<hal::pac::I2C1, P<Gpio2, Gpio3>, hal::i2c::Peripheral>;
pub type Controller<'a> = i2c_pio::I2C<'a, hal::pac::PIO0, hal::pio::SM0, CtrlPinSda, CtrlPinScl>;
pub type CtrlPinSda = Pin<Gpio0, FunctionNull, PullUp>;
pub type CtrlPinScl = Pin<Gpio1, FunctionNull, PullUp>;
type MutexCell<T> = Mutex<RefCell<T>>;
type FIFOBuffer = heapless::Vec<u8, 128>;

#[derive(Debug, defmt::Format, Default)]
pub struct TargetState {
    first: bool,
    gen: Generator,
    vec: FIFOBuffer,
    restart_cnt: u32,
    stop_cnt: u32,
}
impl TargetState {
    pub const fn new() -> TargetState {
        TargetState {
            first: true,
            gen: Generator::fib(),
            vec: FIFOBuffer::new(),
            restart_cnt: 0,
            stop_cnt: 0,
        }
    }
}

#[derive(Debug, defmt::Format, Clone, Copy)]
pub enum Generator {
    Sequence(u8),
    Fibonacci(u8, u8),
}
impl Generator {
    const fn fib() -> Generator {
        Generator::Fibonacci(0, 1)
    }
    const fn seq() -> Generator {
        Generator::Sequence(0)
    }
    fn switch(&mut self) {
        *self = if matches!(self, Generator::Sequence(_)) {
            Generator::Fibonacci(0, 1)
        } else {
            Generator::Sequence(0)
        };
    }
}
impl Default for Generator {
    fn default() -> Self {
        Generator::Sequence(0)
    }
}
impl Iterator for Generator {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        let out;
        match self {
            Generator::Sequence(prev) => {
                (out, *prev) = (*prev, prev.wrapping_add(1));
            }
            Generator::Fibonacci(a, b) => {
                out = *a;
                (*a, *b) = (*b, a.wrapping_add(*b));
            }
        }
        Some(out)
    }
}

fn target_handler(
    target: &mut Target,
    evt: rp2040_hal::i2c::peripheral::Event,
    payload: &mut TargetState,
    throttle: bool,
) {
    let TargetState {
        first,
        gen,
        vec,
        restart_cnt,
        stop_cnt,
    } = payload;
    match evt {
        Event::Start => *first = true,
        Event::Restart => *restart_cnt += 1,
        Event::TransferRead => {
            let n = throttle.then_some(1).unwrap_or(target.tx_fifo_available());
            let v: FIFOBuffer = gen.take(n.into()).collect();
            target.write(&v);
        }
        Event::TransferWrite => {
            if *first {
                gen.switch();
                *first = false;
            }
            // when throttling, treat 1 byte at a time
            let max = throttle
                .then_some(1)
                .unwrap_or_else(|| target.rx_fifo_used().into());
            vec.extend(target.take(max));
        }
        Event::Stop => {
            *stop_cnt += 1;
            // notify the other core a stop was detected
            cortex_m::asm::sev();
        }
    }
}

pub trait ValidAddress: i2c_pio::ValidAddressMode + rp2040_hal::i2c::ValidAddress {}
impl<T> ValidAddress for T where T: rp2040_hal::i2c::ValidAddress + i2c_pio::ValidAddressMode {}
