#!/bin/sh

# Keep running tests even if one of them fails
# "$@" passes any extra args given to ran
cargo test -p on-target-tests --no-fail-fast "$@"
