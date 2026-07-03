# Agent 1: CLAUDE.md / house-style compliance

**Status: done**

## House-style compliance review — MAX17048 fuel gauge (f26f85e..HEAD)

**Rule 1 (snprintf accumulation):** No issues. `http_server.c`'s `format_battery()` uses `append_safe(out, sz, n, ...)` (the established helper, same as the adjacent `format_als()`). `mqtt.c`'s new lines use the pre-existing `APPEND(...)` macro (`mqtt.c:453-460`), which bounds-checks via `rem`/`_w` before advancing `n`. Neither is a bare `n += snprintf(...)`.

**Rule 2 (comment quality) / Rule 6 (Doxygen-lite completeness):** No issues. `fuel_gauge.h`'s file-header explains real non-obvious reasoning (charger-IC float voltage, why `fuel_gauge_present()` is unreliable on USB power, why `fuel_gauge_ready()` exists as a separate honest signal) — matches the bar set by `als.h`. Per-function blocks document param order and return semantics identically in style to `als_read()`'s doc block (prose-style return description, no bare `@return` tag on init functions — this matches `als_init()`'s existing style, not a deviation).

**Rule 3 (helper functions):** No issues. `format_battery()` in `http_server.c:708-732` is a parallel static helper exactly like `format_als()`, called once from `status_get()`. The `transmission.c:1435-1442` log block is inlined directly, but that matches the immediately preceding syslog-stats block (`transmission.c:1416-1421`), which is the local convention for per-cycle TX log entries — not a violation.

**Rule 4 (`#if HAL_HAS_X` gating pattern):** No issues. `fuel_gauge.c` mirrors `als.c` precisely: unconditional file header include, `#if HAL_HAS_FUEL_GAUGE` real implementation, `#else` stub with byte-identical signatures (`fuel_gauge_init/ready/vbus_present/present/read`), all four `hal.h` board blocks get an explicit `HAL_HAS_FUEL_GAUGE` define (1 for FeatherS3-D, 0 elsewhere). `main/CMakeLists.txt` adds `fuel_gauge.c` unconditionally to the shared source list, same as `als.c`/`veml7700.c`.

**Rule 5 (version sync):** No issues. `version.h` bumped once, V2.6.5→V2.6.6, in the feature commit (`c683204`); the follow-up commit (`e125c26`) correctly made no further version.h change since no `CMakeLists.txt`/build-relevant behavior warranted a second bump. The `CMakeLists.txt` change is only a new source file in the existing list — no interaction with the top-level `CMAKE_CONFIGURE_DEPENDS` mechanism, no contradiction.

**Rule 7 (dead/unused code):** No issues. `fuel_gauge_ready()` is called exactly once (`transmission.c:1435`, gating the log block) — intentional per the header's stated design ("honest signal... use to gate output that should always show power-supply data"). `fuel_gauge_vbus_present()` is called exactly once (`transmission.c:1440`, inside that same log line) — also matches its documented purpose. Both are fully wired, nothing orphaned.

## Overall verdict

This change meets the project's house-style bar cleanly. It follows the `als.c` template pattern exactly (including the stub branch), uses only the approved safe-append idioms for string building, documents the non-obvious VCELL/charger-float reasoning at WHY-level rather than restating code, and keeps the version/build metadata internally consistent. No Critical, Important, or Minor findings.
