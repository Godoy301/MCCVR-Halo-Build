#pragma once
// THE VIEW'S NEUTRAL HEIGHT FOLLOWS THE CONFIRMED STANDING HEIGHT UP.
//
// A recenter captures the head as the view's neutral position. With the physical crouch on it adds how far the
// head is below the player's standing height (VR_RecenterStandingLiftM), so the neutral height is the standing
// one whatever the head was doing. That lift exists only once the standing height is CONFIRMED (calibrated, or a
// minute of standing near it): an unconfirmed one may still fall back, and the lift would outlive it. So every
// recenter in the first minute of a session, the automatic one at a level's start among them, takes the head as
// it is. A player who was crouched, bent over the desk or still sitting down at that moment kept a neutral height
// that low, and then stood that much above their character, until the next manual recenter.
//
// This is the same rule applied late: once the standing height is confirmed, a neutral height more than kStartM
// below it glides up to it. Up only. A neutral height above the standing one (a recenter on tiptoe, a reference
// that later fell back) is left alone: it is small, and lowering the reference raises the view, which is the
// direction that puts a camera through a ceiling.
//
// Pure: the caller supplies the head height and the reference in ITS frame and the standing-height deficit from
// the crouch's own continuous frame (baseline - height, which is a difference and so carries between frames that
// differ by a vertical translation).
#include <cmath>
#include <cstdint>

namespace standing_follow
{
struct State
{
    bool active = false;
    uint64_t lastMs = 0;
};

constexpr float kStartM = 0.05f;       // a neutral height this far below standing starts a glide
constexpr float kDoneM = 0.002f;       // and this close ends it
constexpr float kRateMps = 0.30f;      // 30 cm in a second: seen, not felt as a drop
constexpr float kMaxBelowM = 1.0f;     // |baseline - height| past this is not a standing person's sample
constexpr uint64_t kMaxGapMs = 250;    // calls further apart than this are not one motion

// How far to raise the reference on this call (metres, >= 0). `eligible`: the switch is on, the physical crouch
// is on, the player is on foot and the standing height is confirmed and fresh. `below` = baseline - height
// (positive while the head is under the standing height). `startedOut`, when given, is set on the call that
// starts a glide (for one log line).
inline float Step(State& state, bool eligible, float headY, float below, float referenceY, uint64_t nowMs,
                  bool* startedOut = nullptr) noexcept
{
    if (startedOut)
        *startedOut = false;
    const uint64_t last = state.lastMs;
    state.lastMs = nowMs;
    if (!eligible || !std::isfinite(headY) || !std::isfinite(below) || !std::isfinite(referenceY) ||
        std::fabs(below) > kMaxBelowM)
    {
        state.active = false;
        return 0.0f;
    }
    const float gap = headY + below - referenceY;   // the standing height, in the caller's frame, above the reference
    if (!state.active)
    {
        if (!(gap > kStartM))
            return 0.0f;
        state.active = true;
        if (startedOut)
            *startedOut = true;
    }
    if (!(gap > kDoneM))
    {
        state.active = false;
        return 0.0f;
    }
    // The first call of a glide, and one after a break in the calls, moves nothing: there is no interval to
    // scale by.
    if (!last || nowMs <= last || nowMs - last > kMaxGapMs)
        return 0.0f;
    const float step = kRateMps * static_cast<float>(nowMs - last) * 0.001f;
    return step < gap ? step : gap;
}
} // namespace standing_follow
