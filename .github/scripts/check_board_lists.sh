#!/usr/bin/env bash
# Board-list consistency check.
#
# The eleven build targets are named in nine places that nothing ties
# together: the CMake selector, both reusable workflow matrices, the release
# artefact count, the per-board sdkconfig overlays, the web-flasher manifests
# and its <select>, the README board table and the .gitignore list of
# generated sdkconfig caches. Adding or renaming a board has repeatedly
# missed one of them (the XIAO cache was committed in V2.5.20 because the
# .gitignore list was not extended; the OTA label chain in http_server.c
# missed three ports). This script takes CMakeLists.txt as the source of
# truth and fails if any other list disagrees.
#
# Usage: .github/scripts/check_board_lists.sh      (from the repo root)
# Exit:  0 = every list matches, 1 = at least one mismatch (printed).

set -u
cd "$(dirname "$0")/../.." || exit 2

fail=0
err() { echo "::error::$*"; fail=1; }

# ---------------------------------------------------------------------------
# Source of truth: every `(if|elseif)(BOARD STREQUAL "<name>")` in CMake.
# ---------------------------------------------------------------------------
mapfile -t CMAKE < <(sed -n 's/^\s*\(if\|elseif\)(BOARD STREQUAL "\([a-z0-9_]\+\)")/\2/p' CMakeLists.txt | sort)
if [ "${#CMAKE[@]}" -eq 0 ]; then
  echo "::error::no boards found in CMakeLists.txt — selector pattern changed?"
  exit 2
fi
expected=$(printf '%s\n' "${CMAKE[@]}")

# compare <label> <newline-separated list>
compare() {
  local label=$1 actual
  actual=$(printf '%s\n' "$2" | sed '/^$/d' | sort)
  if [ "$actual" != "$expected" ]; then
    err "$label disagrees with CMakeLists.txt"
    diff <(echo "$expected") <(echo "$actual") | sed 's/^/    /' | sed 's/^    </    only in CMake:   /; s/^    >/    only in '"$label"': /'
  fi
}

# 1. _build-boards.yml matrix (the `board:` list under strategy.matrix).
compare "_build-boards.yml matrix" \
  "$(awk '/^\s*board:\s*$/{f=1;next} f&&/^\s*-\s*[a-z0-9_]+\s*$/{sub(/^\s*-\s*/,"");print;next} f&&!/^\s*-/{f=0}' .github/workflows/_build-boards.yml)"

# 2. _cppcheck.yml matrix (`- board: <name>` include entries).
compare "_cppcheck.yml matrix" \
  "$(sed -n 's/^\s*- board:\s*\([a-z0-9_]\+\)\s*$/\1/p' .github/workflows/_cppcheck.yml)"

# 3. Per-board sdkconfig overlays (sdkconfig.defaults.<board>; .psram is shared).
compare "sdkconfig.defaults.<board> files" \
  "$(ls sdkconfig.defaults.* | sed 's/^sdkconfig\.defaults\.//' | grep -v '^psram$')"

# 4. Web-flasher manifests.
compare "docs/manifests/*.json" \
  "$(ls docs/manifests/*.json | xargs -n1 basename | sed 's/\.json$//')"

# 5. Web-flasher <select> options.
compare "docs/index.html <option> list" \
  "$(sed -n 's/.*value="manifests\/\([a-z0-9_]\+\)\.json".*/\1/p' docs/index.html)"

# 6. README board table (first column, backticked target name).
compare "README.md board table" \
  "$(sed -n 's/^| `\([a-z0-9_]\+\)` | .*/\1/p' README.md)"

# 7. .gitignore list of generated per-board sdkconfig caches.
compare ".gitignore sdkconfig.<board> entries" \
  "$(sed -n 's/^sdkconfig\.\([a-z0-9_]\+\)$/\1/p' .gitignore | grep -v '^old$')"

# 8. release.yml artefact count.
n_expected=$(sed -n 's/^\s*EXPECTED_BOARDS=\([0-9]\+\).*/\1/p' .github/workflows/release.yml)
if [ "$n_expected" != "${#CMAKE[@]}" ]; then
  err "release.yml EXPECTED_BOARDS=$n_expected but CMakeLists.txt defines ${#CMAKE[@]} boards"
fi

# 9. The two human-readable lists inside CMakeLists.txt itself (CACHE STRING
#    help text and the FATAL_ERROR message) must name every board.
for b in "${CMAKE[@]}"; do
  grep -q "Target board (.*\b$b\b" CMakeLists.txt || err "CMakeLists.txt CACHE STRING help text lacks $b"
  grep -q "Unknown BOARD.*Valid: .*\b$b\b" CMakeLists.txt || err "CMakeLists.txt FATAL_ERROR 'Valid:' list lacks $b"
done

# 10. _build-boards.yml's chip-target ternary must agree with each board's
#     CMake IDF_TARGET: every esp32s3 / esp32c5 board must be named in it,
#     every plain esp32 board must not be.
ternary=$(awk '/target: \$\{\{/{f=1} f{print} f&&/\}\}\s*$/{exit}' .github/workflows/_build-boards.yml)
while read -r b t; do
  case "$t" in
    esp32) if grep -q "'$b'" <<<"$ternary"; then err "_build-boards.yml target ternary names $b but CMake sets IDF_TARGET esp32 (default branch)"; fi ;;
    *)     if ! grep -q "matrix.board == '$b'" <<<"$ternary"; then err "_build-boards.yml target ternary lacks $b (CMake IDF_TARGET $t)"; fi ;;
  esac
done < <(awk '
  /^\s*(if|elseif)\(BOARD STREQUAL "/ { match($0, /"[a-z0-9_]+"/); b=substr($0, RSTART+1, RLENGTH-2) }
  /set\(IDF_TARGET "/ && b != "" { match($0, /"[a-z0-9]+"/); print b, substr($0, RSTART+1, RLENGTH-2); b="" }
' CMakeLists.txt)

if [ "$fail" -eq 0 ]; then
  echo "board lists consistent: ${#CMAKE[@]} boards in all nine places"
fi
exit "$fail"
