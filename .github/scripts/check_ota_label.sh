#!/usr/bin/env bash
# OTA-page board label check, run per board after the build.
#
# The /update page names the board the running firmware was built for. It is
# the last human check before an image is flashed (the 4 MB and 8 MB Heltec
# builds are not interchangeable in either direction; V2.6.34+ pin maps need
# the new carriers). Until V2.8.10 the name came from an #if/#elif chain in
# main/http_server.c, and a port that forgot its branch built fine and showed
# "(unknown board)"; that happened three times (XIAO, SparkFun S3, TFT
# Feather). check_board_lists.sh cannot catch it: the names are keyed on
# BOARD_* macros, not on the board names it compares.
#
# Since V2.8.11 each hal.h board block defines HAL_BOARD_LABEL and hal.h stops
# the build with #error if one is missing, so the compiler does the real work.
# This script is the tripwire that keeps it that way. It fails if:
#   1. hal.h no longer has the `#ifndef HAL_BOARD_LABEL` / #error check,
#   2. http_server.c no longer builds UPLOAD_PROMPT_BOARD from HAL_BOARD_LABEL,
#   3. the binary does not contain the rendered label sentence.
#
# Usage: .github/scripts/check_ota_label.sh <board> <app .bin>
# Exit:  0 = all three hold, 1 = one of them failed, 2 = bad arguments /
#        missing binary.

set -u

BOARD=${1:-}
BIN=${2:-}
# Located from the script, not the cwd, so a relative <app .bin> keeps
# meaning what the caller meant.
ROOT="$(dirname "$0")/../.."
HAL="$ROOT/main/hal.h"
SRC="$ROOT/main/http_server.c"

if [ -z "$BOARD" ] || [ -z "$BIN" ]; then
  echo "usage: $0 <board> <app .bin>" >&2
  exit 2
fi
if [ ! -f "$BIN" ]; then
  echo "::error::$BOARD: binary not found at $BIN"
  exit 2
fi

if ! grep -qx '#ifndef HAL_BOARD_LABEL' "$HAL" ||
   ! grep -qF '#error "hal.h: this board branch does not define HAL_BOARD_LABEL"' "$HAL"; then
  echo "::error::$HAL no longer refuses a board without HAL_BOARD_LABEL; restore the #ifndef/#error check (or update $0 to match it)"
  exit 1
fi

if ! grep -qF '#define UPLOAD_PROMPT_BOARD "<b style=\"color:red\">" HAL_BOARD_LABEL "</b>"' "$SRC"; then
  echo "::error::UPLOAD_PROMPT_BOARD in $SRC is no longer built from HAL_BOARD_LABEL; keep the board label in hal.h (or update $0 to match it)"
  exit 1
fi

if ! strings "$BIN" | grep -qE 'Select a firmware \.bin for <b style="color:red">[^<]+</b>\.'; then
  echo "::error::$BOARD: the OTA page board label is missing from the binary"
  exit 1
fi

echo "::notice::$BOARD: OTA page carries a board label"
exit 0
