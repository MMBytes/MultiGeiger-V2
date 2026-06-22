#include "tube_types.h"

// V2.6.1: single source of truth for the supported Geiger-Müller tubes. Index =
// tube_type_t = config_t.tube_type. `cps_to_usvph` is the only tube-dependent term
// in the dose pipeline; `name` + `menu_suffix` feed the /config dropdown, the
// /status Radiation card and the boot config dump — so the tube list is defined in
// exactly ONE place. Factors: upstream MultiGeiger (ecocurious2 / t-pi tube.cpp) —
// SBM-20/SBM-19 from datasheet, Si22G empirical vs odlinfo.bfs.de. TUBE_TYPE_SI22G
// keeps the exact 1/12.2792 the old SI22G_CPS_TO_USVPH #define carried, so the
// default config (tube_type = 3) is a byte-for-byte no-op change.
static const struct {
    const char *name;
    const char *menu_suffix;   // /config dropdown suffix appended after `name` (HTML)
    float       cps_to_usvph;
} k_tubes[TUBE_TYPE_COUNT] = {
    [TUBE_TYPE_UNKNOWN] = { "Unknown", " &mdash; no dose conversion (&micro;Sv/h = 0)", 0.0f            },
    [TUBE_TYPE_SBM20]   = { "SBM-20",  " (1/2.47)",                                     1.0f / 2.47f    },
    [TUBE_TYPE_SBM19]   = { "SBM-19",  " (1/9.81888)",                                  1.0f / 9.81888f },
    [TUBE_TYPE_SI22G]   = { "Si22G",   " (1/12.2792) &mdash; default",                  1.0f / 12.2792f },
};

float tube_cps_to_usvph(uint32_t tube_type) {
    return (tube_type < TUBE_TYPE_COUNT) ? k_tubes[tube_type].cps_to_usvph : 0.0f;
}

const char *tube_type_name(uint32_t tube_type) {
    return (tube_type < TUBE_TYPE_COUNT) ? k_tubes[tube_type].name : "Unknown";
}

const char *tube_type_menu_suffix(uint32_t tube_type) {
    return (tube_type < TUBE_TYPE_COUNT) ? k_tubes[tube_type].menu_suffix : "";
}
