// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

/** @file applog_asm.h
 *  @brief Log-line reassembly for the applog vprintf hook (V2.8.32).
 *
 *  ESP-IDF v6 hands one ESP_LOGx to the vprintf hook in 3 fragments
 *  (prefix, text, a lone "\n"; components/log/src/log_format_text.c). applog.c
 *  collects them in one shared buffer and forwards only whole lines to the
 *  /log ring and syslog. The applog mutex is held per fragment, so another
 *  task can log between two fragments of a line; the half line is then
 *  flushed first so the two never mix. Up to V2.8.31 that early flush carried
 *  no newline, and the ring (and so the syslog boot replay and /log) glued
 *  the next line onto it:
 *  "... 4 min from nowI (...) esp_netif_lwip: DHCP server started ...".
 *  Now the early flush ends the line with "\n" and the interrupted task's
 *  lone "\n" that follows is dropped, so both lines come out whole. The same
 *  applies to a line that overruns the buffer. Up to APPLOG_ASM_CUTS tasks
 *  are remembered at once (two tasks taking turns cut each other).
 *
 *  Only a literal lone "\n" is dropped: with CONFIG_LOG_COLORS on, the last
 *  fragment would be "\033[0m\n" and an empty line would come back (colours
 *  are off on every board).
 *
 *  Pure C (no IDF): the caller passes its task handle (non-NULL) as an opaque
 *  pointer and formats each fragment into buf + len itself (no second
 *  buffer), so the host tests in test/test_main.c drive it directly.
 *
 *  Use, under the caller's mutex:
 *      applog_asm_begin(&a, me, emit);
 *      size_t start = a.len;
 *      int n = vsnprintf(a.buf + start, a.sz - start, fmt, args);
 *      applog_asm_end(&a, me, start, n, emit);
 */

#include <stddef.h>

/** Interrupted tasks remembered at once; when all slots are taken, one is overwritten in turn. */
#define APPLOG_ASM_CUTS 4

/** Called with each whole line (ends in "\n", NUL-terminated, len > 0); may
 *  modify the text in place. The buffer is reset after it returns. */
typedef void (*applog_asm_emit_fn)(char *line, size_t len);

typedef struct {
    char       *buf;                   // line buffer, sz bytes (sz >= 2)
    size_t      sz;
    size_t      len;                   // bytes of the pending line, buf[len] == 0
    const void *owner;                 // task that wrote the pending fragment(s)
    const void *cut[APPLOG_ASM_CUTS];  // tasks whose line was ended early: their lone "\n" is dropped
    unsigned    cut_next;              // slot the next cut overwrites when all are taken
} applog_asm_t;

/** @brief Forward the pending line and reset (no-op when empty). */
static inline void applog_asm_emit(applog_asm_t *a, applog_asm_emit_fn emit) {
    if (a->len > 0) emit(a->buf, a->len);
    a->len    = 0;
    a->buf[0] = 0;
    a->owner  = NULL;
}

/** @brief Remember @p who as interrupted (once; a free slot first, else one in turn). */
static inline void applog_asm_cut_note(applog_asm_t *a, const void *who) {
    if (!who) return;
    for (unsigned i = 0; i < APPLOG_ASM_CUTS; i++) {
        if (a->cut[i] == who) return;
    }
    for (unsigned i = 0; i < APPLOG_ASM_CUTS; i++) {
        if (!a->cut[i]) { a->cut[i] = who; return; }
    }
    a->cut[a->cut_next] = who;
    a->cut_next = (a->cut_next + 1) % APPLOG_ASM_CUTS;
}

/** @brief True (and forgotten) if @p me was interrupted: only its first
 *         fragment after the cut can be the "\n" to drop. */
static inline int applog_asm_cut_take(applog_asm_t *a, const void *me) {
    if (!me) return 0;
    for (unsigned i = 0; i < APPLOG_ASM_CUTS; i++) {
        if (a->cut[i] == me) { a->cut[i] = NULL; return 1; }
    }
    return 0;
}

/** @brief End the pending line early with "\n" (appended, or replacing the
 *         last byte when the buffer is full), remember whose it was, forward it. */
static inline void applog_asm_cut(applog_asm_t *a, applog_asm_emit_fn emit) {
    if (a->buf[a->len - 1] != '\n') {
        if (a->len < a->sz - 1) {
            a->buf[a->len++] = '\n';
            a->buf[a->len]   = 0;
        } else {
            a->buf[a->len - 1] = '\n';
        }
    }
    applog_asm_cut_note(a, a->owner);
    applog_asm_emit(a, emit);
}

/** @brief Before task @p me appends a fragment: another task's half line is
 *         ended and forwarded first. */
static inline void applog_asm_begin(applog_asm_t *a, const void *me, applog_asm_emit_fn emit) {
    if (a->len > 0 && a->owner != me) applog_asm_cut(a, emit);
    a->owner = me;
}

/** @brief After the caller formatted a fragment at buf + @p start (@p n =
 *         vsnprintf's return, the untruncated length). Forwards the line on
 *         "\n"; a line that fills the buffer is ended early like an
 *         interrupted one. A lone "\n" from a task whose line was ended
 *         early is dropped (it would only add an empty line). */
static inline void applog_asm_end(applog_asm_t *a, const void *me, size_t start, int n,
                                  applog_asm_emit_fn emit) {
    if (n <= 0) {
        a->buf[start] = 0;   // a failed vsnprintf may leave bytes behind
        if (a->len == 0) a->owner = NULL;
        return;
    }
    const size_t remain = a->sz - start;
    a->len = start + (((size_t)n >= remain) ? remain - 1 : (size_t)n);
    if (applog_asm_cut_take(a, me) && start == 0 && a->len == 1 && a->buf[0] == '\n') {
        a->len    = 0;
        a->buf[0] = 0;
        a->owner  = NULL;
        return;
    }
    if (a->buf[a->len - 1] == '\n') {
        applog_asm_emit(a, emit);
    } else if (a->len >= a->sz - 1) {
        applog_asm_cut(a, emit);
    }
}
