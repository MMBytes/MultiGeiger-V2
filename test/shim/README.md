# Host-test shim headers

Minimal stand-ins for the ESP-IDF / FreeRTOS headers that firmware `.c` files
in `main/` include, so those files can be compiled into the host test binary
(`test/test_main.c`) unchanged. Added in V2.8.8 (codebase review 2.1 B).

Only what the compiled units actually use is declared. A shim provides
declarations, types and inert behaviour — never firmware logic — so a test
exercises the real `main/*.c` code, not a copy of it.

| Header | Provides | Used by |
|---|---|---|
| `freertos/FreeRTOS.h` | `TickType_t`, `BaseType_t`, `pdTRUE`/`pdFALSE`, `portMAX_DELAY` | `history.c` |
| `freertos/semphr.h` | `SemaphoreHandle_t`, mutex create/take/give (single-threaded no-ops; create can be made to fail) | `history.c` |
| `esp_log.h` | `ESP_LOGx` macros that format-check their arguments but print nothing | `history.c` |
| `esp_err.h` | `esp_err_t`, common `ESP_*` codes, `esp_err_to_name()` | `config.h`, `transmission.h` (planned) |
| `driver/i2c_master.h` | opaque I2C bus / device handle types | `pm_sensor.h` via `transmission.h` (planned) |

Include order matters: the build passes `-I test/shim -I main`. A quoted
`#include "x.h"` is resolved from the including file's own directory first, so
for a firmware file in `main/` its `main/` headers (e.g. `tube.h`, `hal.h`) are
always the real ones. `test/test_main.c` lives in `test/`, though, so for it
`test/shim` is searched BEFORE `main/`: **never give a shim the file name of a
header in `main/`**, or the tests would silently compile against the stand-in.
`hal.h` needs a board, so the host build defines `-DBOARD_HELTEC_V2=1`; it only
selects pin and capability macros, none of which the tested code depends on.

Known limits:
- `esp_err.h` and `driver/i2c_master.h` are not used yet; they are there for
  the planned payload / config tests. `esp_err_to_name()` returns only
  `"ESP_OK"` or `"ESP_ERR"`, so no test should compare its text.
- `esp_log.h` uses the GNU `##__VA_ARGS__` form, as ESP-IDF's own header and
  zero-argument calls such as `ESP_LOGE(TAG, "text")` in firmware code need.
  The host build therefore cannot add `-Wpedantic`.
