# Target tests for i2c-pio

This project is for running tests of i2c-pio against real hardware via knurling-rs tools.
It is based or rp2040-hal own on-target-tests.

Adding a test:  
- Add a new Rust program to tests (eg tests/my_new_test.rs)
- Add a new [[test]] to the Cargo.toml

Running all tests:  
Linux (and any other Unix-likes where probe-rs are supported):
```system
./run_tests.sh
```
Windows
```system
run_tests.bat
```

To run a specific test (to make developing tests faster)

```system
cargo test -p on-target-tests --test my_new_test
```

## Prerequisites

Some of the tests need connections between specific pins.

Currently, the following connections are required:

- Connect GPIO 0 to GPIO 2 (pins 1 and 4 on a Pico) and
  connect GPIO 1 to GPIO 3 (pins 2 and 5 on a Pico) for the I2C loopback tests

If you add tests that need some hardware setup, make sure that they are
compatible to the existing on-target tests, so all tests can be run with
a single configuration.
