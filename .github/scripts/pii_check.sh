#!/usr/bin/env bash
# Private-data scan for a public repository.
#
# This repo is public. Deployment details of a home sensor network — LAN
# addresses, MAC addresses, device hostnames / SSIDs, local user paths — do
# not belong in it, and several have slipped in through code comments,
# example payloads, CHANGELOG prose and commit messages. This script scans
# only the lines a change ADDS (plus the accompanying commit / tag messages)
# for the patterns that have actually leaked, so existing accepted history
# never trips it.
#
# Usage:
#   pii_check.sh --staged              pre-commit: scan `git diff --cached`
#   pii_check.sh --range <rev-range>   CI: scan the diff and the commit
#                                      messages of e.g. origin/main...HEAD
#   pii_check.sh --stdin <label>       scan arbitrary text (a tag message,
#                                      an extracted release-notes section)
# Exit: 0 = clean, 1 = findings printed, 2 = usage.
#
# Patterns are deliberately generic (private-IPv4 ranges, MAC shape, chip-ID
# SSID shape, OS user-profile paths) — listing the actual values to catch
# would itself publish them. Place names and coordinates cannot be matched
# generically; those still rely on the human reading the diff.

set -u

# One ERE per line. Matched against added lines and message text.
PATTERNS=(
  # RFC 1918 private IPv4 (10/8, 172.16/12, 192.168/16).
  '(^|[^0-9.])(10\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}|172\.(1[6-9]|2[0-9]|3[01])\.[0-9]{1,3}\.[0-9]{1,3}|192\.168\.[0-9]{1,3}\.[0-9]{1,3})([^0-9.]|$)'
  # Bare last-octet shorthand for a node, e.g. " .196" — a lone ".1xx" token.
  '(^|[[:space:]])\.1[0-9]{2}([^0-9.]|$)'
  # MAC address (boundary classes allow a following ':' so "MAC xx:..:xx:"
  # at the end of a clause still matches).
  '(^|[^0-9A-Fa-f])([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}([^0-9A-Fa-f]|$)'
  # Default AP SSID / hostname built from the chip ID.
  'esp32-[0-9]{2,}'
  # Legacy upstream-style hostnames.
  'MultiGeiger[0-9]{5,}'
  # OS user-profile paths carrying a username.
  '[A-Za-z]:\\+Users\\+[A-Za-z]'
  '/(Users|home)/[a-z][a-z0-9_-]+/'
)

# Strings that match a pattern above but are NOT private data. Each is a
# fixed string; a line containing one is re-checked with it blanked out.
ALLOW=(
  '192.168.4.1'        # ESP-IDF's default soft-AP address (first-boot setup page)
  '192.0.2.'           # RFC 5737 documentation range
  'esp32-1234567'      # the fictional chip ID used in docs/index.html and examples
  'aa:bb:cc:dd:ee:ff'  # placeholder MAC
  'AA:BB:CC:DD:EE:FF'
  '/home/runner/'      # GitHub Actions workspace
)

# Paths never scanned: 3D models hold float tuples that look like anything,
# and this script's own allow-list would flag itself.
SKIP_PATHS='(^|/)3d-Files/|^\.github/scripts/pii_check\.sh$'

findings=0

# scan_text <label> : reads text on stdin, prints hits as "label:lineno: text".
scan_text() {
  local label=$1 n=0 line stripped a p
  while IFS= read -r line || [ -n "$line" ]; do
    n=$((n + 1))
    stripped=$line
    for a in "${ALLOW[@]}"; do stripped=${stripped//"$a"/}; done
    # bash's own ERE engine: no grep process per line, which on a Windows
    # git-bash turns a 2 000-line diff into minutes.
    for p in "${PATTERNS[@]}"; do
      if [[ $stripped =~ $p ]]; then
        printf '%s:%d: %s\n' "$label" "$n" "$line"
        findings=$((findings + 1))
        break
      fi
    done
  done
}

# scan_diff : reads a unified diff on stdin, scans only added lines, reports
# them against the destination file path.
scan_diff() {
  local file="" skip=0 line
  while IFS= read -r line || [ -n "$line" ]; do
    case "$line" in
      '+++ b/'*)
        file=${line#+++ b/}
        if [[ $file =~ $SKIP_PATHS ]]; then skip=1; else skip=0; fi
        continue ;;
      '+++ '*|'--- '*|'diff --git'*|'index '*|'Binary files'*) continue ;;
      '+'*)
        [ "$skip" -eq 1 ] && continue
        scan_text "$file" <<<"${line#+}" ;;
    esac
  done
}

case "${1:-}" in
  --staged)
    git diff --cached --no-color --unified=0 --diff-filter=AMRC -- . | scan_diff
    ;;
  --range)
    [ -n "${2:-}" ] || { echo "usage: $0 --range <rev-range>"; exit 2; }
    git diff --no-color --unified=0 --diff-filter=AMRC "$2" -- . | scan_diff
    # Commit messages in the range, each labelled by its short hash.
    while read -r sha; do
      git log -1 --format=%B "$sha" | scan_text "commit $sha"
    done < <(git rev-list "$2" 2>/dev/null)
    ;;
  --stdin)
    scan_text "${2:-stdin}"
    ;;
  *)
    echo "usage: $0 --staged | --range <rev-range> | --stdin <label>"; exit 2 ;;
esac

if [ "$findings" -gt 0 ]; then
  echo "::error::$findings line(s) look like private data (LAN/MAC/hostname/user path). Replace with a role description ('the deployed field node') or a documentation value (192.0.2.x, esp32-1234567)."
  exit 1
fi
echo "pii check: clean"
