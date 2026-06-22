#pragma once

/** @file
 *  @brief Geiger-Müller tube characteristics — the cps→µSv/h dose conversion
 *  factor and the human labels per tube type.
 *
 *  V2.6.1: split out of `transmission.h` so the tube table is a small,
 *  dependency-light leaf module (only <stdint.h>). The dose factor is the ONLY
 *  tube-dependent term in the radiation pipeline — pulse counting, the dead-time
 *  gate and the HV charge loop are all tube-agnostic — so this header carries no
 *  networking / sensor includes and `config.c`, `http_server.c`, `transmission.c`
 *  and `main.c` can all share it cheaply.
 *
 *  The enumerator values ARE the config indices (`config_t.tube_type`, see
 *  `config_fields.def`). `k_tubes[]` in `tube_types.c` is the SINGLE source of
 *  truth for the supported-tube list: the `/config` dropdown, the `/status`
 *  Radiation card and the boot config dump all derive their text from the
 *  accessors below, so adding a tube means editing exactly one table.
 *
 *  Factors are the upstream MultiGeiger calibration (ecocurious2 / t-pi
 *  `tube.cpp`): SBM-20 / SBM-19 from datasheet, Si22G empirical vs odlinfo.bfs.de.
 *  See `reference_radiation_data_analysis`.
 */

#include <stdint.h>

typedef enum {
    TUBE_TYPE_UNKNOWN = 0,   // no conversion — dose forced to 0.0
    TUBE_TYPE_SBM20   = 1,   // cps→µSv/h = 1/2.47
    TUBE_TYPE_SBM19   = 2,   // cps→µSv/h = 1/9.81888
    TUBE_TYPE_SI22G   = 3,   // cps→µSv/h = 1/12.2792 (default — the dose baseline)
    TUBE_TYPE_COUNT
} tube_type_t;

/** @brief cps→µSv/h conversion factor for a tube_type index (0..3).
 *         Out-of-range or Unknown → 0.0f (dose suppressed). */
float tube_cps_to_usvph(uint32_t tube_type);

/** @brief Short human tube label ("Si22G", "SBM-19", …). Never NULL;
 *         out-of-range → "Unknown". */
const char *tube_type_name(uint32_t tube_type);

/** @brief HTML suffix appended after the name in the `/config` tube dropdown
 *         (e.g. " (1/12.2792) &mdash; default"). Never NULL; out-of-range → "". */
const char *tube_type_menu_suffix(uint32_t tube_type);
