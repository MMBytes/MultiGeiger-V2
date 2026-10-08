#!/usr/bin/env bash
# OTA-page board label check, run per board after the build.
#
# The /update page names the board the running firmware was built for, from
# the UPLOAD_PROMPT_BOARD #if/#elif chain in main/http_server.c. It is the
# last human check before an image is flashed (the 4 MB and 8 MB Heltec builds
# are not interchangeable in either direction; V2.6.34+ pin maps need the new
# carriers). A port that forgets its branch still builds and falls through to
# the "(unknown board)" fallback; that happened three times (XIAO, SparkFun
# S3, TFT Feather) and each was noticed only by looking at the served page.
# This script fails the build instead. check_board_lists.sh cannot catch it:
# the chain is keyed on BOARD_* macros, not on the board names it compares.
#
# The fallback text must also still exist in http_server.c, so that rewording
# it breaks this check loudly instead of letting it pass forever.
#
# Usage: .github/scripts/check_ota_label.sh <board> <app .bin>
# Exit:  0 = the binary carries a real board label, 1 = fallback found or the
#        fallback text moved, 2 = bad arguments / missing binary.

set -u

BOARD=${1:-}
BIN=${2:-}
FALLBACK='(unknown board)'
# Located from the script, not the cwd, so a relative <app .bin> keeps
# meaning what the caller meant.
SRC="$(dirname "$0")/../../main/http_server.c"

if [ -z "$BOARD" ] || [ -z "$BIN" ]; then
  echo "usage: $0 <board> <app .bin>" >&2
  exit 2
fi
if [ ! -f "$BIN" ]; then
  echo "::error::$BOARD: binary not found at $BIN"
  exit 2
fi

if ! grep -qF "#define UPLOAD_PROMPT_BOARD \"<b style=\\\"color:red\\\">$FALLBACK</b>\"" "$SRC"; then
  echo "::error::the UPLOAD_PROMPT_BOARD fallback '$FALLBACK' is no longer in $SRC; update $0 to match it"
  exit 1
fi

if strings "$BIN" | grep -qF "$FALLBACK"; then
  echo "::error::$BOARD: the OTA page labels this build '$FALLBACK'; add an #elif BOARD_<NAME> branch to UPLOAD_PROMPT_BOARD in main/http_server.c"
  exit 1
fi

echo "::notice::$BOARD: OTA page carries a board label"
exit 0
