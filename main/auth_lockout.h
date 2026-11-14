#pragma once

/** @file
 *  @brief Admin login lockout rule (V2.8.26).
 *  Pure functions, free of ESP-IDF, so the host tests can cover them;
 *  http_server.c keeps the state in RTC no-init RAM and passes in the time
 *  from the RTC timer (esp_rtc_get_time_us), which keeps counting across
 *  every reset except a power-on.
 *
 *  Rule: ONE counter for the whole node. AUTH_LOCK_FAILS wrong passwords in a
 *  row lock the login; while locked every protected request is refused, the
 *  correct password too. Lock n lasts AUTH_LOCK_BASE_S << n seconds, capped at
 *  level AUTH_LOCK_LEVEL_MAX (60 s, 2 min, 4 min … 8 h 32 min). A correct
 *  login resets everything. Every wrong attempt counts, repeats included.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AUTH_LOCK_FAILS     5            ///< wrong passwords in a row that lock
#define AUTH_LOCK_BASE_S    60u          ///< first lock, seconds
#define AUTH_LOCK_LEVEL_MAX 9u           ///< 60 << 9 = 30720 s = 8 h 32 min (cap)
#define AUTH_LOCK_MAGIC     0x41554C4Bu  ///< "AULK": RTC RAM holds a valid state

/** @brief Lock state. Lives in RTC no-init RAM, so every field is validated at boot.
 *  The lock duration is never stored; it is derived from `level`. */
typedef struct {
    uint32_t magic;          ///< AUTH_LOCK_MAGIC once initialised
    uint8_t  fails;          ///< wrong passwords since the last lock / success (0..FAILS-1)
    uint8_t  level;          ///< locks since the last success (1..LEVEL_MAX+1 while locked)
    uint8_t  locked;         ///< 1 while a lock is active
    uint8_t  _pad;
    uint32_t refused;        ///< requests refused during the current lock
    uint64_t lock_start_us;  ///< RTC-timer time the current lock started
} auth_lockout_t;

/** @brief Result of auth_lock_check(). */
typedef struct {
    uint32_t remaining_s;    ///< >0: still locked for this many seconds (rounded up)
    bool     just_unlocked;  ///< a lock ended before this check (report it once)
    uint32_t lock_s;         ///< duration of that lock (with just_unlocked)
    uint32_t refused;        ///< refused requests: so far (locked) or in total (just_unlocked)
} auth_lock_verdict_t;

/** @brief Duration of lock number `level` (0-based), capped at AUTH_LOCK_LEVEL_MAX. */
static inline uint32_t auth_lock_duration_s(uint8_t level) {
    uint8_t l = (level > AUTH_LOCK_LEVEL_MAX) ? (uint8_t)AUTH_LOCK_LEVEL_MAX : level;
    return AUTH_LOCK_BASE_S << l;
}

/** @brief Duration of the current lock; only meaningful while `locked` (level >= 1). */
static inline uint32_t auth_lock_current_s(const auth_lockout_t *s) {
    return auth_lock_duration_s((uint8_t)(s->level - 1u));
}

/** @brief Seconds the lock still lasts at `now_us`, rounded up; 0 if not locked or
 *         ended. RTC time before the lock start reads as the full duration.
 *         No side effects (used for the boot log line). */
static inline uint32_t auth_lock_remaining_s(const auth_lockout_t *s, uint64_t now_us) {
    if (!s->locked) return 0;
    const uint64_t dur_us = (uint64_t)auth_lock_current_s(s) * 1000000u;
    if (now_us < s->lock_start_us) return auth_lock_current_s(s);
    const uint64_t elapsed = now_us - s->lock_start_us;
    if (elapsed >= dur_us) return 0;
    return (uint32_t)((dur_us - elapsed + 999999u) / 1000000u);
}

/** @brief Validate the RTC-RAM state at boot. Clears it on a power-on or when
 *         anything is impossible; restarts an active lock in full when the RTC
 *         time is earlier than its start (the counter was reset, so the time
 *         the node was off is unknown). Always resets `refused`. */
static inline void auth_lock_on_boot(auth_lockout_t *s, uint64_t now_us, bool power_on) {
    const bool bad = power_on
                  || s->magic != AUTH_LOCK_MAGIC
                  || s->level > AUTH_LOCK_LEVEL_MAX + 1u
                  || s->fails >= AUTH_LOCK_FAILS
                  || s->locked > 1u
                  || (s->locked && s->level == 0u);
    if (bad) {
        memset(s, 0, sizeof *s);
        s->magic = AUTH_LOCK_MAGIC;
        return;
    }
    if (s->locked && now_us < s->lock_start_us) s->lock_start_us = now_us;
    s->refused = 0;
}

/** @brief Check the lock before a protected request. Counts the request as
 *         refused while locked; ends an expired lock (reported once via
 *         just_unlocked). */
static inline auth_lock_verdict_t auth_lock_check(auth_lockout_t *s, uint64_t now_us) {
    auth_lock_verdict_t v = {0, false, 0, 0};
    if (!s->locked) return v;
    if (now_us < s->lock_start_us) s->lock_start_us = now_us;   // time went backwards
    const uint32_t rem = auth_lock_remaining_s(s, now_us);
    if (rem > 0) {
        s->refused++;
        v.remaining_s = rem;
        v.refused = s->refused;
        return v;
    }
    // Lock over. `fails` is already 0: it was reset when the lock started.
    v.just_unlocked = true;
    v.lock_s = auth_lock_current_s(s);
    v.refused = s->refused;
    s->locked = 0;
    s->refused = 0;
    return v;
}

/** @brief Count one wrong password. Returns the new lock's duration in seconds
 *         when this attempt starts a lock, else 0. */
static inline uint32_t auth_lock_on_fail(auth_lockout_t *s, uint64_t now_us) {
    s->fails++;
    if (s->fails < AUTH_LOCK_FAILS) return 0;
    const uint32_t lock_s = auth_lock_duration_s(s->level);
    if (s->level < AUTH_LOCK_LEVEL_MAX + 1u) s->level++;
    s->fails = 0;
    s->refused = 0;
    s->locked = 1;
    s->lock_start_us = now_us;
    return lock_s;
}

/** @brief A correct login: forget every failure, level and lock. */
static inline void auth_lock_on_success(auth_lockout_t *s) {
    s->fails = 0;
    s->level = 0;
    s->locked = 0;
    s->refused = 0;
    s->lock_start_us = 0;
}

#ifdef __cplusplus
}
#endif
