use core::{cell::RefCell, ops::RangeInclusive};

use critical_section::Mutex;
use fugit::{HertzU32, RateExtU32};

use hal::{
    gpio::FunctionNull,
    pac::PIO0,
    pio::{PIOExt, UninitStateMachine, PIO, PIO0SM0},
};
use i2c_pio::Error;
use rp2040_hal::{
    self as hal,
    clocks::init_clocks_and_plls,
    gpio::{FunctionI2C, Pin, PullUp},
    pac,
    watchdog::Watchdog,
    Clock, Timer,
};

use super::{
    Controller, CtrlPinScl, CtrlPinSda, FIFOBuffer, Generator, MutexCell, Target, TargetState,
    ValidAddress,
};

pub struct State {
    pio: PIO<PIO0>,
    i2c_components: Option<((CtrlPinSda, CtrlPinScl), UninitStateMachine<PIO0SM0>)>,

    timer: hal::Timer,
    resets: hal::pac::RESETS,
    ref_clock_freq: HertzU32,
}

static TARGET: MutexCell<Option<Target>> = Mutex::new(RefCell::new(None));
static PAYLOAD: MutexCell<TargetState> = MutexCell::new(RefCell::new(TargetState::new()));
static TIMER: MutexCell<Option<Timer>> = MutexCell::new(RefCell::new(None));

macro_rules! assert_vec_eq {
    ($e:expr) => {
        critical_section::with(|cs| {
            let v = &mut PAYLOAD.borrow_ref_mut(cs).vec;
            assert_eq!(*v, $e, "FIFO");
            v.clear();
        });
    };
}
macro_rules! assert_restart_count {
    ($e:expr) => {{
        let restart_cnt: u32 = critical_section::with(|cs| PAYLOAD.borrow_ref(cs).restart_cnt);
        defmt::assert!(
            $e.contains(&restart_cnt),
            "restart count out of range {} ∉ {}",
            restart_cnt,
            $e
        );
    }};
}

pub fn system_setup<T: ValidAddress>(xtal_freq_hz: u32, addr: T) -> State {
    unsafe {
        hal::sio::spinlock_reset();
    }
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let ctrl_sda_pin: Pin<_, FunctionNull, PullUp> = pins.gpio0.reconfigure();
    let ctrl_scl_pin: Pin<_, FunctionNull, PullUp> = pins.gpio1.reconfigure();

    let trg_sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio2.reconfigure();
    let trg_scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio3.reconfigure();

    let (pio, sm, ..) = pac.PIO0.split(&mut pac.RESETS);
    let i2c_target = hal::I2C::new_peripheral_event_iterator(
        pac.I2C1,
        trg_sda_pin,
        trg_scl_pin,
        &mut pac.RESETS,
        addr,
    );

    critical_section::with(|cs| TARGET.replace(cs, Some(i2c_target)));

    static STACK: rp2040_hal::multicore::Stack<10240> = rp2040_hal::multicore::Stack::new();
    unsafe {
        // delegate I2C1 irqs to core 1
        // If the IRQ is not defined, core 1 will just spin in the default IRQ without doing anything
        hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo)
            .cores()
            .get_mut(1)
            .expect("core 1 is not available")
            .spawn(STACK.take().unwrap(), || {
                pac::NVIC::unpend(hal::pac::Interrupt::I2C1_IRQ);
                pac::NVIC::unmask(hal::pac::Interrupt::I2C1_IRQ);

                loop {
                    cortex_m::asm::wfi()
                }
            })
            .expect("failed to start second core.");
    }

    State {
        pio,
        i2c_components: Some(((ctrl_sda_pin, ctrl_scl_pin), sm)),
        timer,
        resets: pac.RESETS,
        ref_clock_freq: clocks.system_clock.freq(),
    }
}

pub fn test_setup<'s, T: ValidAddress>(
    state: &'s mut State,
    addr: T,
    throttling: bool,
) -> Controller<'s> {
    let ((sda, scl), sm) = state
        .i2c_components
        .take()
        .expect("I2C components are available");

    // TARGET is shared with core1. Therefore this needs to happen in a cross-core
    // critical-section.
    critical_section::with(|cs| {
        // reset peripheral
        let (i2c, (sda, scl)) = TARGET
            .replace(cs, None)
            .expect("State contains a target")
            .free(&mut state.resets);

        // reset payload storage
        PAYLOAD.replace_with(cs, |_| TargetState::new());

        // remove timer/disable throttling
        TIMER.replace(cs, throttling.then_some(state.timer));

        //
        TARGET.replace(
            cs,
            Some(hal::I2C::new_peripheral_event_iterator(
                i2c,
                sda,
                scl,
                &mut state.resets,
                addr,
            )),
        );
    });

    i2c_pio::I2C::new(
        &mut state.pio,
        sda,
        scl,
        sm,
        200.kHz(),
        state.ref_clock_freq.clone(),
    )
}

macro_rules! test_teardown {
    ($state:expr, $controller:expr) => {
        $state.i2c_components = Some($controller.free());
    };
}
/// Wait for the expected count of Stop event to ensure the target side has finished processing
/// requests.
///
/// If a test ends with a write command, there is a risk that the test will check the content of
/// the shared buffer while the target handler hasn't finished processing its fifo.
pub fn wait_stop_count(stop_cnt: u32) {
    while critical_section::with(|cs| PAYLOAD.borrow_ref(cs).stop_cnt) < stop_cnt {
        cortex_m::asm::wfe();
    }
    defmt::flush();
}

pub fn peripheral_handler() {
    critical_section::with(|cs| {
        let Some(ref mut target) = *TARGET.borrow_ref_mut(cs) else {
            return;
        };

        let mut timer = TIMER.borrow_ref_mut(cs);

        while let Some(evt) = target.next_event() {
            if let Some(t) = timer.as_mut() {
                use embedded_hal_0_2::blocking::delay::DelayUs;
                t.delay_us(50);
            }

            super::target_handler(
                target,
                evt,
                &mut *PAYLOAD.borrow_ref_mut(cs),
                timer.is_some(),
            );
        }
    })
}

pub fn write<T: ValidAddress>(state: &mut State, addr: T) {
    use embedded_hal_0_2::blocking::i2c::Write;
    let mut controller = test_setup(state, addr, false);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    assert_eq!(controller.write(addr, &samples).is_ok(), true);
    wait_stop_count(1);

    assert_restart_count!((0..=0));
    assert_vec_eq!(samples);

    test_teardown!(state, controller);
}
pub fn write_iter<T: ValidAddress>(state: &mut State, addr: T) {
    let mut controller = test_setup(state, addr, false);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    controller
        .write_iter(addr, samples.iter().cloned())
        .expect("Successful write_iter");
    wait_stop_count(1);

    assert_restart_count!((0..=0));
    assert_vec_eq!(samples);
    test_teardown!(state, controller);
}

pub fn write_iter_read<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    let mut controller = test_setup(state, addr, false);

    let samples_seq: FIFOBuffer = Generator::seq().take(25).collect();
    let samples_fib: FIFOBuffer = Generator::fib().take(25).collect();
    let mut v = [0u8; 25];
    controller
        .write_iter_read(addr, samples_fib.iter().cloned(), &mut v)
        .expect("Successful write_iter_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples_seq);
    assert_vec_eq!(samples_fib);
    test_teardown!(state, controller);
}

pub fn write_read<T: ValidAddress>(state: &mut State, addr: T, restart_count: RangeInclusive<u32>) {
    use embedded_hal_0_2::blocking::i2c::WriteRead;
    let mut controller = test_setup(state, addr, false);

    let samples_seq: FIFOBuffer = Generator::seq().take(25).collect();
    let samples_fib: FIFOBuffer = Generator::fib().take(25).collect();
    let mut v = [0u8; 25];
    controller
        .write_read(addr, &samples_fib, &mut v)
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples_seq);
    assert_vec_eq!(samples_fib);
    test_teardown!(state, controller);
}

pub fn read<T: ValidAddress>(state: &mut State, addr: T, restart_count: RangeInclusive<u32>) {
    use embedded_hal_0_2::blocking::i2c::Read;
    let mut controller = test_setup(state, addr, false);

    let mut v = [0u8; 25];
    controller.read(addr, &mut v).expect("successfully read");
    wait_stop_count(1);

    let samples: FIFOBuffer = Generator::fib().take(25).collect();
    assert_restart_count!(restart_count);
    assert_eq!(v, samples);
    assert_vec_eq!([]);
    test_teardown!(state, controller);
}

pub fn transactions_read<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::{I2c, Operation};
    let mut controller = test_setup(state, addr, false);

    let mut v = [0u8; 25];
    controller
        .transaction(addr, &mut [Operation::Read(&mut v)])
        .expect("successfully write_read");
    wait_stop_count(1);

    let samples: FIFOBuffer = Generator::fib().take(25).collect();
    assert_restart_count!(restart_count);
    assert_eq!(v, samples);
    assert_vec_eq!([]);
    test_teardown!(state, controller);
}

pub fn transactions_write<T: ValidAddress>(state: &mut State, addr: T) {
    use embedded_hal::i2c::{I2c, Operation};
    let mut controller = test_setup(state, addr, false);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    controller
        .transaction(addr, &mut [Operation::Write(&samples)])
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!((0..=0));
    assert_vec_eq!(samples);
    test_teardown!(state, controller);
}

pub fn transactions_read_write<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::{I2c, Operation};
    let mut controller = test_setup(state, addr, true);

    let samples_seq: FIFOBuffer = Generator::seq().take(25).collect();
    let samples_fib: FIFOBuffer = Generator::fib().take(25).collect();
    let mut v = [0u8; 25];
    controller
        .transaction(
            addr,
            &mut [Operation::Read(&mut v), Operation::Write(&samples_seq)],
        )
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples_fib);
    assert_vec_eq!(samples_seq);
    test_teardown!(state, controller);
}

pub fn transactions_write_read<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::{I2c, Operation};
    let mut controller = test_setup(state, addr, false);

    let samples_seq: FIFOBuffer = Generator::seq().take(25).collect();
    let mut v = [0u8; 25];

    controller
        .transaction(
            addr,
            &mut [Operation::Write(&samples_seq), Operation::Read(&mut v)],
        )
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples_seq);
    assert_vec_eq!(samples_seq);
    test_teardown!(state, controller);
}

pub fn transaction<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    // Throttling is important for this test as it also ensures that the Target implementation
    // does not "waste" bytes that would be discarded otherwise.
    //
    // One down side of this is that the Target implementation is unable to detect restarts
    // between consicutive write operations
    use embedded_hal::i2c::{I2c, Operation};
    let mut controller = test_setup(state, addr, true);

    let mut v = ([0u8; 14], [0u8; 25], [0u8; 25], [0u8; 14], [0u8; 14]);
    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    controller
        .transaction(
            addr,
            &mut [
                Operation::Write(&samples), // goes to v2
                Operation::Read(&mut v.0),
                Operation::Read(&mut v.1),
                Operation::Read(&mut v.2),
                Operation::Write(&samples), // goes to v3
                Operation::Read(&mut v.3),
                Operation::Write(&samples), // goes to v4
                Operation::Write(&samples), // remains in buffer
                Operation::Write(&samples), // remains in buffer
                Operation::Read(&mut v.4),
            ],
        )
        .expect("successfully write_read");
    wait_stop_count(1);

    // There are 14restarts in this sequence but because of latency in the target handling, it
    // may only detect 7.
    assert_restart_count!(restart_count);

    // assert writes
    let e: FIFOBuffer = itertools::chain!(
        samples.iter(),
        samples.iter(),
        samples.iter(),
        samples.iter(),
        samples.iter(),
    )
    .cloned()
    .collect();
    assert_vec_eq!(e);

    // assert reads
    let g: FIFOBuffer = Generator::seq().take(92).collect();
    let h: FIFOBuffer = itertools::chain!(
        v.0.into_iter(),
        v.1.into_iter(),
        v.2.into_iter(),
        v.3.into_iter(),
        v.4.into_iter()
    )
    .collect();
    assert_eq!(g, h);
    test_teardown!(state, controller);
}

pub fn transactions_iter<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::{I2c, Operation};
    let mut controller = test_setup(state, addr, false);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    let mut v = [0u8; 25];
    controller
        .transaction(
            addr,
            &mut [Operation::Write(&samples), Operation::Read(&mut v)],
        )
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples);
    assert_vec_eq!(samples);
    test_teardown!(state, controller);
}

pub fn embedded_hal<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    // Throttling is important for this test as it also ensures that the Target implementation
    // does not "waste" bytes that would be discarded otherwise.
    //
    // One down side of this is that the Target implementation is unable to detect restarts
    // between consicutive write operations
    use embedded_hal::i2c::I2c;
    let mut controller = test_setup(state, addr, true);

    let samples1: FIFOBuffer = Generator::seq().take(25).collect();
    let samples2: FIFOBuffer = Generator::fib().take(14).collect();
    let mut v = ([0; 14], [0; 25], [0; 25], [0; 14], [0; 14]);

    let mut case = || {
        controller.write(addr, &samples1)?;
        wait_stop_count(1);
        controller.read(addr, &mut v.0)?;
        wait_stop_count(2);
        controller.read(addr, &mut v.1)?;
        wait_stop_count(3);
        controller.read(addr, &mut v.2)?;
        wait_stop_count(4);
        controller.write_read(addr, &samples2, &mut v.3)?;
        wait_stop_count(5);
        controller.write(addr, &samples2)?;
        wait_stop_count(6);
        controller.write(addr, &samples1)?;
        wait_stop_count(7);
        controller.write_read(addr, &samples1, &mut v.4)?;
        wait_stop_count(8);
        Ok::<(), i2c_pio::Error>(())
    };
    case().expect("Successful test");

    // There are 14restarts in this sequence but because of latency in the target handling, it
    // may only detect 7.
    assert_restart_count!(restart_count);

    // assert writes
    let e: FIFOBuffer = itertools::chain!(
        Generator::seq().take(25),
        Generator::fib().take(14),
        Generator::fib().take(14),
        Generator::seq().take(25),
        Generator::seq().take(25),
    )
    .collect();
    assert_vec_eq!(e);

    // assert reads
    let g: FIFOBuffer = itertools::chain!(
        Generator::seq().take(64),
        Generator::fib().take(14),
        Generator::seq().take(14)
    )
    .collect();
    let h: FIFOBuffer = itertools::chain!(
        v.0.into_iter(),
        v.1.into_iter(),
        v.2.into_iter(),
        v.3.into_iter(),
        v.4.into_iter()
    )
    .collect();
    assert_eq!(g, h);
    test_teardown!(state, controller);
}

/// the address given must be different from each other.
pub fn nak_on_addr<A: ValidAddress>(state: &mut State, addr_target: A, addr_ctrl: A) {
    use embedded_hal::i2c::I2c;
    let mut controller = test_setup(state, addr_target, false);

    let samples1: FIFOBuffer = Generator::seq().take(25).collect();
    assert_eq!(
        controller.write(addr_ctrl, &samples1),
        Err(Error::NoAcknowledgeAddress)
    );

    assert_vec_eq!([]);
    test_teardown!(state, controller);
}
