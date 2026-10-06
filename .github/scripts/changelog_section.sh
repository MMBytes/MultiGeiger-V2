#!/usr/bin/env bash
# Print the CHANGELOG.md section for one release tag.
#
# Usage: .github/scripts/changelog_section.sh <tag> [CHANGELOG.md]
# Exit:  0 and the section body on stdout; 1 (nothing printed) if there is
#        no "## <tag>" heading.
#
# This is the ONE definition of what "the section for a tag" means. The
# release preflight, the private-data scan of the release notes and the
# release-body extraction all call it, so the preflight passes if and only
# if the extractor will find a non-empty body — three hand-synchronised
# copies of the same awk program used to live in release.yml.
#
# Heading shapes accepted:
#   "## V2.4.2"            bare
#   "## V2.4.2 - title"    separator + title
# The [^0-9A-Za-z] boundary stops V2.4.1 from matching V2.4.10. The body is
# everything up to the next "## " heading, with any trailing "---" rule and
# blank lines dropped so they don't look like noise under a release body.

set -u
tag=${1:?usage: changelog_section.sh <tag> [file]}
file=${2:-CHANGELOG.md}

awk -v ver="$tag" '
  /^## / {
    if (in_section) exit
    if ($0 == "## " ver || $0 ~ "^## " ver "[^0-9A-Za-z]") { in_section = 1; next }
  }
  in_section { print }
' "$file" | sed -e :a -e '/^[[:space:]-]*$/{$d;N;ba' -e '}' > "${TMPDIR:-/tmp}/changelog_section.$$"

if [ -s "${TMPDIR:-/tmp}/changelog_section.$$" ]; then
  cat "${TMPDIR:-/tmp}/changelog_section.$$"
  rm -f "${TMPDIR:-/tmp}/changelog_section.$$"
  exit 0
fi
rm -f "${TMPDIR:-/tmp}/changelog_section.$$"
exit 1
