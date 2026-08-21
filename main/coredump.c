#include "coredump.h"

#include <string.h>
#include <stdio.h>
#include "esp_core_dump.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "util.h"   // V2.7.3: safe_strcpy

static const char *TAG = "coredump";

// Cached state from coredump_init(). All reads happen on the HTTP server
// task; the only writer is coredump_init() (boot) and coredump_erase()
// (HTTP POST), so a single writer-at-a-time invariant holds and no lock
// is needed for the volatile-bool / size_t / static-char-array reads.
static bool   s_have_dump = false;
static size_t s_dump_size = 0;

// Summary cache. esp_core_dump_get_summary() takes ~150 ms on a 64 KB
// dump (ELF parse + checksum re-verify) and allocates ~6 KB on the
// stack — too heavy to run on every /status hit. Parse once at boot,
// reuse the result. The 16-byte task name field comes straight from
// the IDF struct; the panic reason from IDF is bounded at ~200 bytes
// in practice (the docs example uses a 200-byte buffer).
#define COREDUMP_REASON_MAX 200
static char     s_task_name[16] = {0};
static uint32_t s_exc_pc = 0;
static char     s_panic_reason[COREDUMP_REASON_MAX] = {0};
static bool     s_have_summary = false;

void coredump_init(void) {
    esp_err_t err = esp_core_dump_image_check();
    if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "no core dump present");
        return;
    }
    if (err != ESP_OK) {
        // Partition corrupted or unreadable. Surface explicitly — the
        // user can still GET /coredump.elf for raw bytes if they want
        // to see what's in there, but the summary is unparseable.
        ESP_LOGW(TAG, "core dump partition check failed: %s",
                 esp_err_to_name(err));
        return;
    }

    // Valid dump exists. Read size for status-page display.
    size_t addr = 0;
    if (esp_core_dump_image_get(&addr, &s_dump_size) != ESP_OK) {
        ESP_LOGW(TAG, "image_get failed despite image_check OK");
        return;
    }
    s_have_dump = true;
    ESP_LOGI(TAG, "core dump present: %u bytes at flash 0x%x",
             (unsigned)s_dump_size, (unsigned)addr);

    // Parse summary. Heap allocation rather than stack — the struct is
    // ~700 bytes including the backtrace array, comfortable on heap
    // but tight on a default 4 KB httpd task stack if we ever moved
    // this out of init.
    esp_core_dump_summary_t *summary = malloc(sizeof(*summary));
    if (!summary) {
        ESP_LOGW(TAG, "summary parse skipped: out of heap");
        return;
    }
    err = esp_core_dump_get_summary(summary);
    if (err == ESP_OK) {
        // V2.7.3: was a raw `strncpy(..., n - 1)` + manual terminator. Same
        // copy, but through the shared helper — that idiom is what GCC 16
        // flags as -Wstringop-truncation, and leaving one hand-rolled copy
        // behind just waits for the next toolchain bump to find it.
        safe_strcpy(s_task_name, summary->exc_task, sizeof(s_task_name));
        s_exc_pc = summary->exc_pc;
        s_have_summary = true;
        ESP_LOGW(TAG, "panicked in task '%s' at PC=0x%08x",
                 s_task_name, (unsigned)s_exc_pc);
    } else {
        ESP_LOGW(TAG, "get_summary failed: %s", esp_err_to_name(err));
    }
    free(summary);

    // Best-effort panic reason. Separate API since older dumps may
    // have the summary but not the reason string. Not fatal if absent.
    if (esp_core_dump_get_panic_reason(s_panic_reason,
                                        sizeof(s_panic_reason)) == ESP_OK) {
        ESP_LOGW(TAG, "panic reason: %s", s_panic_reason);
    }
}

bool coredump_have_dump(void) {
    return s_have_dump;
}

size_t coredump_get_size(void) {
    return s_dump_size;
}

void coredump_get_summary_html(char *out, size_t sz) {
    if (!out || sz == 0) return;
    if (!s_have_dump) {
        snprintf(out, sz, "none");
        return;
    }
    // Compose three optional segments — only the ones we actually have.
    // task+PC come from get_summary(); reason comes from
    // get_panic_reason(). Either can be absent on partial dumps.
    char task_seg[80] = "";
    char reason_seg[COREDUMP_REASON_MAX + 16] = "";
    if (s_have_summary) {
        snprintf(task_seg, sizeof(task_seg),
                 " &middot; task=<code>%s</code> PC=<code>0x%08x</code>",
                 s_task_name, (unsigned)s_exc_pc);
    }
    if (s_panic_reason[0]) {
        // Caller is responsible for HTML-escaping the panic reason if
        // it might contain markup. The IDF-generated reason strings
        // are stock ASCII (e.g. "Guru Meditation Error: ..."), so a
        // plain inline render is acceptable for V2.4.18. If we ever
        // surface user-controlled strings here, escape first.
        snprintf(reason_seg, sizeof(reason_seg), " &middot; %s",
                 s_panic_reason);
    }
    snprintf(out, sz, "yes &middot; %u bytes%s%s",
             (unsigned)s_dump_size, task_seg, reason_seg);
}

// Chunk size for partition reads. 4 KB matches the flash erase sector
// and is large enough that the per-chunk httpd overhead is negligible,
// small enough that the buffer fits comfortably on the httpd task stack
// (which is 8192 bytes after V2.4.x bumps).
#define COREDUMP_STREAM_CHUNK 4096

esp_err_t coredump_stream_to_http(httpd_req_t *req) {
    if (!s_have_dump) return ESP_ERR_NOT_FOUND;

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
    if (!part) {
        ESP_LOGE(TAG, "coredump partition not found at stream time");
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t buf[COREDUMP_STREAM_CHUNK];
    size_t remaining = s_dump_size;
    size_t offset = 0;
    while (remaining > 0) {
        size_t n = remaining < sizeof(buf) ? remaining : sizeof(buf);
        esp_err_t r = esp_partition_read(part, offset, buf, n);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "partition read at +%u failed: %s",
                     (unsigned)offset, esp_err_to_name(r));
            return r;
        }
        r = httpd_resp_send_chunk(req, (const char *)buf, n);
        if (r != ESP_OK) {
            // Client gone or TCP error — log and bail. The httpd
            // framework will tear down the response on our return.
            ESP_LOGW(TAG, "send_chunk at +%u failed: %s",
                     (unsigned)offset, esp_err_to_name(r));
            return r;
        }
        offset    += n;
        remaining -= n;
    }
    // Terminator chunk.
    return httpd_resp_send_chunk(req, NULL, 0);
}

esp_err_t coredump_erase(void) {
    esp_err_t err = esp_core_dump_image_erase();
    if (err == ESP_OK) {
        s_have_dump    = false;
        s_dump_size    = 0;
        s_have_summary = false;
        s_task_name[0]    = '\0';
        s_panic_reason[0] = '\0';
        s_exc_pc          = 0;
        ESP_LOGI(TAG, "core dump partition erased");
    } else {
        ESP_LOGE(TAG, "image_erase failed: %s", esp_err_to_name(err));
    }
    return err;
}
