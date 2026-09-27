#pragma once

// AIM WRITE -- engine-free helpers for the STATE-WRITE actuation rung (Actuation::WriteState).
//
// A state write replaces the game's own control angles every tick, so it has failure modes a stick
// loop does not: a yaw written on a different 2*pi branch than the game's own value, a pitch past the
// game's clamp, and -- the dangerous one -- writing through an address that was never proven, or has
// stopped being right (level change, reload). These helpers hold the game-agnostic half of those
// guards, so each provider only supplies facts (angles, "does the record agree with the aim?", "did
// the writer see a different record?") and the policy is one tested body of code.
//
// Zero engine/game dependencies; unit-tested in tests/aim_solve_tests.cpp alongside aim_solve_logic.h.

#include <cmath>
#include <cstdint>

namespace aim_write {

constexpr float kTwoPi = 6.28318530717958647692f;

// Put `target` (radians) on the same 2*pi branch as `current`, the value the game holds now. A write
// that jumps by 2*pi between ticks is the same direction to the sim, but anything interpolating
// between snapshots renders it as a full spin. Non-finite `current` -> `target` unchanged.
inline float unwrap_to_branch(float target, float current) {
    if (!std::isfinite(current)) return target;
    return current + std::remainder(target - current, kTwoPi);
}

// Keep a written pitch inside the game's own clamp (radians).
inline float clamp_pitch(float pitch, float limit) {
    return pitch < -limit ? -limit : (pitch > limit ? limit : pitch);
}

// ARM GATE -- never write through an address that has not been proven, and stop the moment it may
// be wrong. The provider feeds one step per tick:
//   agree     -- this tick's ValueAgreement: the candidate record holds the SAME angles the game's
//                derived aim shows (only consulted while disarmed; the aim is derived from the record
//                every tick, so a wrong address cannot keep agreeing);
//   record    -- WHICH record agreed (its address; 0 = none). A streak must be about ONE record: if the
//                candidate changes mid-streak, the count restarts on the new one;
//   mismatch  -- the WRITER's cumulative count of ticks where it resolved a different record than the
//                one armed (a level change, a checkpoint reload, a second instance);
//   now_ms    -- a monotonic clock.
// Policy: arm after `agree_needed` consecutive agreements about the same record (then `record` holds
// what was proven); while armed, ANY new mismatch disarms at once and backs off `backoff_ms` before
// re-validating; after `max_disarms` disarms, give up for the session (the two ways of locating the
// record disagree by design on this build -- fail closed and say so).
struct ArmGate {
    int      agree_needed = 3;
    uint64_t backoff_ms   = 5000;
    int      max_disarms  = 3;

    bool     armed      = false;
    bool     given_up   = false;
    int      streak     = 0;
    int      disarms    = 0;
    uint64_t record     = 0;    // the record the current streak -- or the arm -- is about
    unsigned seen_mismatch = 0;
    uint64_t retry_at_ms   = 0;

    enum class Event { None, Armed, Disarmed, GaveUp };

    // True when this tick's evidence can change anything (disarmed, not given up, out of backoff), so
    // the caller can skip gathering it -- evidence is usually the expensive half.
    bool wants_evidence(uint64_t now_ms) const { return !armed && !given_up && now_ms >= retry_at_ms; }

    Event step(bool agree, uint64_t candidate, unsigned mismatch, uint64_t now_ms) {
        if (given_up) return Event::None;
        if (armed) {
            if (mismatch != seen_mismatch) {
                seen_mismatch = mismatch;
                armed = false;
                streak = 0;
                ++disarms;
                retry_at_ms = now_ms + backoff_ms;
                if (disarms >= max_disarms) { given_up = true; return Event::GaveUp; }
                return Event::Disarmed;
            }
            return Event::None;
        }
        seen_mismatch = mismatch;   // mismatches seen while disarmed are not the armed record's fault
        if (now_ms < retry_at_ms) return Event::None;
        if (!agree || candidate == 0) { streak = 0; return Event::None; }
        if (candidate != record) { record = candidate; streak = 0; }
        if (++streak < agree_needed) return Event::None;
        armed = true;
        streak = 0;
        return Event::Armed;
    }

    // Leave the armed state without counting a failure (the mode was switched off, the provider stopped
    // wanting a state write). A later re-arm must re-prove the address from scratch.
    void release() { armed = false; streak = 0; record = 0; }
};

} // namespace aim_write
