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
# Exit: 0 = clean, 1 = findings printed, 2 = usage / bad range.
#
# Patterns are deliberately generic (private-IPv4 ranges, MAC shape, chip-ID
# SSID shape, OS user-profile paths) — listing the actual values to catch
# would itself publish them. Place names and coordinates cannot be matched
# generically; those still rely on the human reading the diff.
#
# Implementation notes: matching uses bash's own `=~` (no grep process per
# line — on a Windows git-bash that turns a 2 000-line diff into minutes), and
# the scan functions are fed by redirection, never by a pipe, because a
# function at the end of a pipeline runs in a subshell and its findings
# counter would be lost (the first version of this script printed hits and
# then exited 0 for exactly that reason).

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

# Values that match a pattern above but are NOT private data. Each is an ERE
# anchored so that only the exact token matches (a trailing boundary class
# stops "192.168.4.1" from also excusing "192.168.4.17"); every match is
# replaced by a space before the patterns run, so the boundary classes in
# PATTERNS still see a separator.
ALLOW=(
  '192\.168\.4\.1([^0-9]|$)'        # ESP-IDF's default soft-AP address (first-boot setup page)
  '192\.0\.2\.[0-9]{1,3}([^0-9]|$)' # RFC 5737 documentation range
  'esp32-1234567([^0-9]|$)'         # the fictional chip ID used in docs/index.html and examples
  '[Aa][Aa]:[Bb][Bb]:[Cc][Cc]:[Dd][Dd]:[Ee][Ee]:[Ff][Ff]'  # placeholder MAC
  '/home/runner/'                   # GitHub Actions workspace
)

# Paths never scanned: 3D models hold float tuples that look like anything,
# and this script's own allow-list would flag itself.
SKIP_PATHS='(^|/)3d-Files/|^\.github/scripts/pii_check\.sh$'

findings=0

# report <label> <lineno> <text>
report() {
  printf '%s:%s: %s\n' "$1" "$2" "$3"
  findings=$((findings + 1))
}

# matches <text> : 0 if the text (after allow-list blanking) hits a pattern.
matches() {
  local stripped=$1 a p
  for a in "${ALLOW[@]}"; do
    while [[ $stripped =~ $a ]]; do stripped=${stripped/"${BASH_REMATCH[0]}"/ }; done
  done
  for p in "${PATTERNS[@]}"; do
    [[ $stripped =~ $p ]] && return 0
  done
  return 1
}

# scan_text <label> : reads text on stdin, reports hits as "label:lineno: text".
scan_text() {
  local label=$1 n=0 line
  while IFS= read -r line || [ -n "$line" ]; do
    n=$((n + 1))
    line=${line%$'\r'}
    matches "$line" && report "$label" "$n" "$line"
  done
}

# scan_diff : reads a unified diff on stdin, scans only added lines, reports
# them against the destination file path and its line number in the new file
# (tracked from the @@ hunk headers).
scan_diff() {
  local file="" skip=0 lineno=0 line
  while IFS= read -r line || [ -n "$line" ]; do
    line=${line%$'\r'}
    case "$line" in
      'diff --git '*)
        file=""; skip=0 ;;
      '+++ b/'*)
        # git appends a TAB after a path that contains a space.
        file=${line#+++ b/}; file=${file%$'\t'}
        if [[ $file =~ $SKIP_PATHS ]]; then skip=1; else skip=0; fi ;;
      '+++ '*)
        # Any other +++ shape (a quoted path would land here if the diff were
        # produced without core.quotepath=off) is a parser gap: count it, so a
        # gap can never read as "clean".
        report "diff-header" 0 "unrecognised header, file not scanned: $line" ;;
      '--- '*|'index '*|'Binary files'*|'old mode'*|'new mode'*|'similarity'*|'rename '*|'new file'*|'deleted file'*)
        ;;
      '@@ '*)
        # "@@ -a,b +c,d @@" (or "+c @@" when d == 1): c is the first new line.
        if [[ $line =~ \+([0-9]+) ]]; then lineno=${BASH_REMATCH[1]}; else lineno=0; fi ;;
      '+'*)
        if [ "$skip" -eq 0 ] && [ -n "$file" ]; then
          matches "${line#+}" && report "$file" "$lineno" "${line#+}"
        fi
        lineno=$((lineno + 1)) ;;
      ' '*)
        lineno=$((lineno + 1)) ;;
    esac
  done
}

case "${1:-}" in
  --staged)
    # core.quotepath=off: with the default (on), a path with non-ASCII
    # characters is printed quoted and octal-escaped, which the header
    # parser would not recognise.
    scan_diff < <(git -c core.quotepath=off diff --cached --no-color --unified=0 --diff-filter=AMRC -- .)
    ;;
  --range)
    [ -n "${2:-}" ] || { echo "usage: $0 --range <rev-range>"; exit 2; }
    range=$2
    # Both ends must resolve: an unfetched base would otherwise diff against
    # nothing and report a silent "clean". Cut at the first two-dot run, not
    # the first dot — refs like V2.8.4 contain dots.
    base=${range%%..*}
    if ! git rev-parse --verify --quiet "${base}^{commit}" >/dev/null; then
      echo "::error::cannot resolve the base of range '$range' — fetch it first"
      exit 2
    fi
    scan_diff < <(git -c core.quotepath=off diff --no-color --unified=0 --diff-filter=AMRC "$range" -- .)
    # Commit messages: only the commits reachable from the tip and not from
    # the base. `A...B` would be the symmetric difference and drag in every
    # commit on the base branch since the merge base (e.g. a PR failing for
    # a message already on main), so the three dots become two here. Merge
    # commits (GitHub's synthetic refs/pull/N/merge) carry no authored text.
    while read -r sha; do
      scan_text "commit ${sha:0:9}" < <(git log -1 --format=%B "$sha")
    done < <(git rev-list --no-merges "${range/.../..}")
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
