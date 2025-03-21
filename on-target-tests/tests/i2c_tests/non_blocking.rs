use core::{
    cell::RefCell,
    future::Future,
    ops::{Deref, RangeInclusive},
    task::Poll,
};

use fugit::{HertzU32, RateExtU32};
use futures::FutureExt;
use hal::{gpio::FunctionNull, pio::PIOExt};
use heapless::Vec;

use rp2040_hal::{
    self as hal,
    clocks::init_clocks_and_plls,
    gpio::{FunctionI2C, Pin, PullUp},
    pac::{self, PIO0},
    pio::{UninitStateMachine, PIO, PIO0SM0},
    watchdog::Watchdog,
    Clock,
};

use super::{
    Controller, CtrlPinScl, CtrlPinSda, FIFOBuffer, Generator, Target, TargetState, ValidAddress,
};
use i2c_pio::Error;

pub struct State {
    pio: PIO<PIO0>,
    i2c_components: Option<((CtrlPinSda, CtrlPinScl), UninitStateMachine<PIO0SM0>)>,

    target: Option<Target>,
    resets: hal::pac::RESETS,
    ref_clock_freq: HertzU32,
    payload: RefCell<TargetState>,
}

pub fn run_test(mut f: impl Future<Output = ()>) {
    tinywake::run_all([&mut f]);
}
async fn wait_with(payload: &RefCell<TargetState>, mut f: impl FnMut(&TargetState) -> bool) {
    while f(payload.borrow().deref()) {
        let mut done = false;
        core::future::poll_fn(|cx| {
            cx.waker().wake_by_ref();
            if !done {
                done = true;
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);
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

    let i2c_target = hal::I2C::new_peripheral_event_iterator(
        pac.I2C1,
        trg_sda_pin,
        trg_scl_pin,
        &mut pac.RESETS,
        addr,
    );
    let (pio, sm, ..) = pac.PIO0.split(&mut pac.RESETS);

    unsafe {
        pac::NVIC::unpend(hal::pac::Interrupt::PIO0_IRQ_0);
        pac::NVIC::unmask(hal::pac::Interrupt::PIO0_IRQ_0);
        pac::NVIC::unpend(hal::pac::Interrupt::I2C1_IRQ);
        pac::NVIC::unmask(hal::pac::Interrupt::I2C1_IRQ);
    }

    State {
        pio,
        i2c_components: Some(((ctrl_sda_pin, ctrl_scl_pin), sm)),

        target: Some(i2c_target),
        resets: pac.RESETS,
        ref_clock_freq: clocks.system_clock.freq(),
        payload: RefCell::new(TargetState::new()),
    }
}

pub fn test_setup<'s, T: ValidAddress>(
    state: &'s mut State,
    addr: T,
    _throttling: bool,
) -> (Controller<'s>, &'s RefCell<TargetState>, &'s mut Target) {
    let ((sda, scl), sm) = state
        .i2c_components
        .take()
        .expect("I2C components are available");

    let (i2c_t, (sda_t, scl_t)) = state
        .target
        .take()
        .expect("target's missing")
        .free(&mut state.resets);

    state.payload.replace(Default::default());

    state.target = Some(hal::I2C::new_peripheral_event_iterator(
        i2c_t,
        sda_t,
        scl_t,
        &mut state.resets,
        addr,
    ));

    (
        i2c_pio::I2C::new(
            &mut state.pio,
            sda,
            scl,
            sm,
            200.kHz(),
            state.ref_clock_freq.clone(),
        ),
        &state.payload,
        state.target.as_mut().expect("target is available"),
    )
}

macro_rules! test_teardown {
    ($state:expr, $controller:expr) => {
        $state.i2c_components = Some($controller.free());
    };
}

pub async fn target_handler(payload: &RefCell<TargetState>, target: &mut Target) -> (u32, u32) {
    loop {
        let evt = target.wait_next().await;

        super::target_handler(target, evt, &mut *payload.borrow_mut(), false);
    }
}

async fn embedded_hal_case<A: ValidAddress>(
    controller: &mut Controller<'_>,
    addr: A,
    v: &mut ([u8; 25], [u8; 25], [u8; 25], [u8; 14], [u8; 14]),
    payload: &RefCell<TargetState>,
) -> Result<(), Error> {
    use embedded_hal_async::i2c::I2c;
    let sample1: FIFOBuffer = Generator::seq().take(25).collect();
    let sample2: FIFOBuffer = Generator::fib().take(14).collect();

    // we need to wait for stop to be registered between each operations otherwise we have no
    // way to know when the Target side has finished processing the last request.
    controller.write(addr, &sample1).await?;
    wait_with(payload, |p| p.stop_cnt != 1).await;

    controller.read(addr, &mut v.0).await?;
    wait_with(payload, |p| p.stop_cnt != 2).await;

    controller.read(addr, &mut v.1).await?;
    wait_with(payload, |p| p.stop_cnt != 3).await;

    controller.read(addr, &mut v.2).await?;
    wait_with(payload, |p| p.stop_cnt != 4).await;

    controller.write_read(addr, &sample2, &mut v.3).await?;
    wait_with(payload, |p| p.stop_cnt != 5).await;

    controller.write(addr, &sample2).await?;
    wait_with(payload, |p| p.stop_cnt != 6).await;

    controller.write(addr, &sample1).await?;
    wait_with(payload, |p| p.stop_cnt != 7).await;

    controller.write_read(addr, &sample1, &mut v.4).await?;
    wait_with(payload, |p| p.stop_cnt != 8).await;
    Ok::<(), Error>(())
}
pub async fn embedded_hal<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    // Throttling is important for this test as it also ensures that the Target implementation
    // does not "waste" bytes that would be discarded otherwise.
    //
    // One down side of this is that the Target implementation is unable to detect restarts
    // between consicutive write operations
    let (mut controller, payload, target) = test_setup(state, addr, true);

    // Test
    let mut v = Default::default();
    let ctrl = embedded_hal_case(&mut controller, addr, &mut v, &payload);
    let trgt = target_handler(payload, target);
    futures::select_biased! {
        r = ctrl.fuse() => r.expect("Controller test success"),
        _ = trgt.fuse() => {}
    }

    // Validate

    // There are 14restarts in this sequence but because of latency in the target handling, it
    // may only detect 7.
    let actual_restart_count = payload.borrow().restart_cnt;
    assert!(
        restart_count.contains(&actual_restart_count),
        "restart count out of range {} ∉ {:?}",
        actual_restart_count,
        restart_count
    );

    // assert writes
    let sample1: FIFOBuffer = Generator::seq().take(25).collect();
    let sample2: FIFOBuffer = Generator::fib().take(14).collect();
    let e: FIFOBuffer = itertools::chain!(
        sample1.iter(),
        sample2.iter(),
        sample2.iter(),
        sample1.iter(),
        sample1.iter(),
    )
    .cloned()
    .collect();
    assert_eq!(payload.borrow().vec, e);
    // assert reads
    let g: FIFOBuffer = itertools::chain!(
        Generator::fib().take(25),
        Generator::fib().skip(25 + 7).take(25),
        Generator::fib().skip(2 * (25 + 7)).take(25),
        Generator::seq().take(14),
        Generator::fib().take(14)
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

pub async fn transaction<A: ValidAddress>(
    state: &mut State,
    addr: A,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::Operation;
    use embedded_hal_async::i2c::I2c;
    let (mut controller, payload, target) = test_setup(state, addr, true);

    // Throttling is important for this test as it also ensures that the Target implementation
    // does not "waste" bytes that would be discarded otherwise.
    //
    // One down side of this is that the Target implementation is unable to detect restarts
    // between consicutive write operations
    let sample1: Vec<u8, 25> = Generator::seq().take(25).collect();
    let sample2: Vec<u8, 14> = Generator::fib().take(14).collect();

    // Test
    let mut v: ([u8; 25], [u8; 25], [u8; 25], [u8; 14], [u8; 14]) = Default::default();
    let mut ops = [
        Operation::Write(&sample1), // goes to v2
        Operation::Read(&mut v.0),
        Operation::Read(&mut v.1),
        Operation::Read(&mut v.2),
        Operation::Write(&sample2), // goes to v3
        Operation::Read(&mut v.3),
        Operation::Write(&sample2), // goes to v4
        Operation::Write(&sample1), // remains in buffer
        Operation::Write(&sample1), // remains in buffer
        Operation::Read(&mut v.4),
    ];

    let case = async {
        controller
            .transaction(addr, &mut ops)
            .await
            .expect("Controller test success");
        wait_with(&payload, |p| p.stop_cnt != 1).await;
    };
    futures::select_biased! {
        _ = case.fuse() => {}
        _ = target_handler(
            payload,
            target,
        ).fuse() => {}
    }

    // Validate

    // There are 14restarts in this sequence but because of latency in the target handling, it
    // may only detect 7.
    let actual_restart_count = payload.borrow().restart_cnt;
    assert!(
        restart_count.contains(&actual_restart_count),
        "restart count out of range {} ∉ {:?}",
        actual_restart_count,
        restart_count
    );
    // assert writes
    let e: FIFOBuffer = itertools::chain!(
        Generator::seq().take(25),
        Generator::fib().take(14),
        Generator::fib().take(14),
        Generator::seq().take(25),
        Generator::seq().take(25),
    )
    .collect();
    assert_eq!(e, payload.borrow().vec);
    // assert reads
    let g: FIFOBuffer = itertools::chain!(
        Generator::fib().take(25),
        Generator::fib().skip(32).take(25),
        Generator::fib().skip(64).take(25),
        Generator::fib().skip(96).take(14),
        Generator::fib().skip(112).take(14),
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

pub async fn transaction_iter<A: ValidAddress>(state: &mut State, addr: A) {
    let (mut controller, payload, target) = test_setup(state, addr, true);

    use i2c_write_iter::non_blocking::I2cIter;
    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    let case = async {
        I2cIter::transaction_iter(
            &mut controller,
            addr,
            [i2c_write_iter::Operation::WriteIter(samples.clone())],
        )
        .await
        .expect("Controller test success");
        wait_with(&payload, |p| p.stop_cnt != 1).await;
    };

    futures::select_biased! {
        _ = case.fuse() => {}
        _ = target_handler(
            &payload,
            target
        ).fuse() => {}
    }

    assert_eq!(samples, payload.borrow().vec);
    test_teardown!(state, controller);
}

/// the address given must be different from each other.
pub async fn nak_on_addr<A: ValidAddress>(state: &mut State, addr_target: A, addr_ctrl: A) {
    use embedded_hal_async::i2c::I2c;
    let (mut controller, payload, _target) = test_setup(state, addr_target, true);

    let samples1: FIFOBuffer = Generator::seq().take(25).collect();
    assert_eq!(
        controller.write(addr_ctrl, &samples1).await,
        Err(Error::NoAcknowledgeAddress)
    );

    assert_eq!(payload.borrow().vec, [], "FIFO");
    test_teardown!(state, controller);
}
