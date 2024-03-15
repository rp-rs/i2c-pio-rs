@rem Keep running tests even if one of them fails
@rem We need to specify environment variables here to control build since we aren't able to override them in Cargo.toml

cargo test -p on-target-tests --no-fail-fast
