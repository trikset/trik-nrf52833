#!/bin/bash

# Refresh bin/ artifacts from the current sysbuild output.
#
# Maps build outputs to the tracked files under bin/:
#   mcuboot bootloader   build/mcuboot/zephyr/zephyr.bin              -> bin/mcuboot.bin
#   application (signed) build/trik-nrf52833/zephyr/zephyr.signed.bin -> bin/trik-nrf52833.bin
#   full-chip image      build/merged.hex                             -> bin/merged.bin
#
# merged.hex is an Intel HEX text file, but the bin/ copy intentionally
# keeps the historical ".bin" name so callers do not need to change.
#
# Usage: ./scripts/refresh_bin.sh [BUILD_DIR]
#   BUILD_DIR defaults to <repo-root>/build.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${1:-$ROOT/build}"

copy_one() {
	local src="$1"
	local dst="$2"

	if [[ ! -f "$src" ]]; then
		echo "skip (not found): $src" >&2
		return 1
	fi

	cp -f "$src" "$dst"
	echo "updated: $dst"
}

failed=0

copy_one "$BUILD/mcuboot/zephyr/zephyr.bin" "$ROOT/bin/mcuboot.bin" || failed=1
copy_one "$BUILD/trik-nrf52833/zephyr/zephyr.signed.bin" "$ROOT/bin/trik-nrf52833.bin" || failed=1
copy_one "$BUILD/merged.hex" "$ROOT/bin/merged.bin" || failed=1

exit "$failed"
