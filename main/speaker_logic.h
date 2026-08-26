#pragma once

/** @file
 *  @brief Pure tick-defer decision logic for the speaker/LED pulse tick.
 *
 *  Header-only and dependency-free (only <stdint.h>/<stdbool.h>) for the same
 *  reason as tube_logic.h and env_api.h: the host-side runner under `test/`
 *  includes it directly, so the state machine can be exercised without ESP-IDF,
 *  FreeRTOS or hardware. speaker.c owns the side effects (LEDC, GPIO); this
 *  file owns only the decision.
 *
 *  V2.7.4 (tick defer): every counted pulse schedules a piezo tick, and the
 *  1 kHz audio timer dispatched it 0-1000 us later -- so for ~19% of counts
 *  (190/1000) the tick STARTED within 190 us of the count, while the count node
 *  was still recovering. A 4 ms tick begun anywhere in that window is still
 *  switching through the vulnerable part of the recovery: the pad is blind
 *  until the ramp recrosses the input's switching point at ~148 us, and only
 *  then regains a noise margin that grows with the ramp (tube.h). The tick's
 *  transient tips that marginal crossing and manufactures a phantom edge, which
 *  lands in the 50-190 us edge-spacing bucket. Note the distinction the prose
 *  here originally blurred: 190 us is the window the TICK must start inside,
 *  while 50-190 us is the bucket the resulting PHANTOM is recorded in.
 *
 *  Holding the tick one whole audio period moves its start to 1000-2000 us:
 *  clear of the 190 us dead-time gate by more than 5x, and roughly five time
 *  constants into a 190.4 us RC recovery, where the margin is no longer
 *  marginal.
 *
 *  THE SEEDING RULE, which is the whole design: the countdown is seeded at
 *  REQUEST time (in the GM ISR), never inside the poll. A prototype that seeded
 *  in the poll used "left == 0" as its needs-seeding test, so the poll that
 *  counted the counter down to zero re-seeded it on the very next period and the
 *  tick never fired at all. That bug is invisible downstream -- a silent tick
 *  and a working defer produce the SAME phantom count -- so it must be
 *  prevented structurally rather than detected. Seeding at request time makes
 *  "left == 0" mean only "finished".
 *
 *  CONCURRENCY. The live instance is written by the GM count ISR and read by
 *  the esp_timer task. Both run on core 0 -- the GPIO ISR is installed from
 *  app_main's core, and the timer task is pinned by the IDF default
 *  CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0 (verified set, no board overrides it).
 *  So compiler-level ordering is sufficient and no explicit barrier is needed;
 *  if that Kconfig ever changes, this assumption is what breaks.
 *
 *  BOTH members are volatile, so the defer_left-before-pending store order is
 *  guaranteed between them. Do NOT read this as equivalent to the melody-pointer
 *  publish in speaker.c: that one orders NON-volatile stores behind a single
 *  volatile pointer write, which the standard does not actually guarantee. This
 *  is the stronger construction, and copying the weaker one here on the strength
 *  of an analogy would be a mistake.
 *
 *  The qualifier sits on the MEMBERS rather than the instance so speaker.c can
 *  pass a plain &s_tick with no cast-away-volatile at any call site. On the host
 *  test runner the qualifier is simply inert.
 */

#include <stdint.h>
#include <stdbool.h>

/** @brief One pending pulse tick and how many audio periods it still waits.
 *
 *  @c pending is a FLAG, not a queue: counts arriving during a defer restart
 *  the countdown rather than stacking, matching the pre-V2.7.4 behaviour of the
 *  single @c s_pending_ticks flag it replaces.
 *
 *  RESTART, NOT EXTEND-ONCE -- and the real consequence. Because every request
 *  re-seeds, a tick fires only after one quiet audio period. The invariant this
 *  protects is "no tick inside the recovery window of the MOST RECENT count".
 *
 *  A pending request is NEVER LOST -- @c pending persists, so a burst yields
 *  exactly one tick at its first millisecond lull. The tick therefore fires at
 *  the first opportunity with probability exp(-rate * 1 ms), and otherwise is
 *  DELAYED by an expected 1 ms * (exp(rate * 1 ms) - 1):
 *
 *      90 CPM    (ambient)     99.85% fire at once   +1.5 us mean delay
 *      5 000 CPM (hot source)  92.0%                 +87 us
 *      60 000 CPM (extreme)    36.8%                 +1.7 ms
 *
 *  Silence needs deterministic sub-millisecond spacing, which Poisson arrivals
 *  never sustain. Note the honest baseline too: the pre-V2.7.4 path already
 *  merged bursts (flag, not queue; 4 ms sounding; early return) at a ceiling of
 *  ~200 ticks/s, so at 60 000 CPM this is a ~200/s buzz becoming a ~130-150/s
 *  buzz -- a thinning of something already unresolvable, not a loss of signal.
 *
 *  WHY RESTART RATHER THAN SEED-ONLY-WHEN-IDLE. Seed-when-idle bounds tick
 *  latency and would ALSO be an acceptable fix: its fires are decorrelated from
 *  later counts, giving a residual in-window probability of about
 *  rate * 190 us per tick -- roughly 0.03% at ambient, against the ~19%
 *  in-window rate this change removes (same metric, three orders of magnitude).
 *  Restart is kept because (a) it is the semantics the release soak actually
 *  measured, and the whole warrant for this change is that it was measured;
 *  (b) it is two stores with no branch in the ISR; (c) "no tick within the
 *  recovery window of the most recent count" is the cleanest invariant to state
 *  and test. Revisit only for a HANDHELD target, where tick density is the
 *  primary display rather than a courtesy -- this node is fixed-installation
 *  and its real output is the upload path.
 */
typedef struct {
    volatile uint32_t pending;      /**< 1 = a tick is waiting to fire, 0 = idle. */
    volatile uint32_t defer_left;   /**< Audio periods still to wait before firing. */
} tick_defer_state_t;

/** @brief Request a tick, seeding its defer countdown. Called from the GM ISR.
 *
 *  @param s        State block shared with tick_defer_poll().
 *  @param periods  Audio periods to hold the tick; 0 = fire on the next poll
 *                  (the pre-V2.7.4 behaviour).
 *
 *  Write order matters: @c defer_left is stored BEFORE @c pending so the audio
 *  callback can never observe a pending tick alongside a stale countdown.
 *
 *  TWO narrow races are accepted rather than locked out. Both windows are a few
 *  instructions wide, at most once per poll, at <=1 kHz.
 *
 *  1. A request landing between poll's @c defer_left==0 read and its
 *     @c pending=0 write is ABSORBED by the fire already in flight: that tick
 *     sounds within microseconds of the NEW count and the new request is lost.
 *     No worse than the pre-V2.7.4 behaviour it replaces, which did exactly
 *     that on every count.
 *  2. A request landing inside poll's @c defer_left-- (a load/sub/store, so
 *     same-core preemption can split it) has its re-seed OVERWRITTEN by the
 *     store. The tick then fires one period after the newest count instead of
 *     one-to-two -- still past the 190 us gate by >5x, so harmless.
 */
__attribute__((always_inline)) static inline void
tick_defer_request(tick_defer_state_t *s, uint32_t periods) {
    s->defer_left = periods;
    s->pending    = 1;
}

/** @brief Advance one audio period. Returns true when the tick fires NOW.
 *
 *  @param s  State block seeded by tick_defer_request().
 *  @return   true exactly once per request, on the (periods+1)th poll, provided
 *            no further request restarts the countdown first.
 *
 *  Never seeds. @c defer_left == 0 therefore means only "finished waiting",
 *  which is what makes the re-seed bug described in the @file block
 *  unrepresentable here.
 */
__attribute__((always_inline)) static inline bool
tick_defer_poll(tick_defer_state_t *s) {
    if (!s->pending) return false;
    if (s->defer_left > 0) {
        s->defer_left--;
        return false;
    }
    s->pending = 0;
    return true;
}

/** @brief Drop any queued tick and its countdown together.
 *
 *  Used where the pre-V2.7.4 code cleared @c s_pending_ticks alone: the melody
 *  end-of-sequence path and speaker_set_modes(). Clearing the flag without the
 *  countdown would leave a stale counter to delay the NEXT tick.
 */
__attribute__((always_inline)) static inline void
tick_defer_clear(tick_defer_state_t *s) {
    s->pending    = 0;
    s->defer_left = 0;
}
