#include "telemetry.h"

#include "esp_log.h"

static const char *TAG = "telemetry";

static telemetry_desc_t s_cols[TELEMETRY_MAX_COLUMNS];
static size_t           s_count;

void telemetry_register(const char *header, telemetry_read_fn read, void *arg) {
    if (header == NULL || read == NULL) {
        ESP_LOGE(TAG, "rejecting NULL registration");
        return;
    }
    if (s_count >= TELEMETRY_MAX_COLUMNS) {
        // Deliberately non-fatal: losing one CSV column beats faulting a
        // field node at boot. The error lands in /log + serial.
        ESP_LOGE(TAG, "registry full (%d) — dropping column '%s'",
                 TELEMETRY_MAX_COLUMNS, header);
        return;
    }
    s_cols[s_count].header = header;
    s_cols[s_count].read   = read;
    s_cols[s_count].arg    = arg;
    s_count++;
}

size_t telemetry_count(void) {
    return s_count;
}

const telemetry_desc_t *telemetry_get(size_t idx) {
    return (idx < s_count) ? &s_cols[idx] : NULL;
}
