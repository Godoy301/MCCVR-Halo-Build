#pragma once
// THE F8 / F9 PITCH TRIM IS A FLAT-SCREEN CONTROL. IN STEREO VR IT IS ZERO.
//
// The trim dates from the mod's first milestone: head tracking on a flat screen, where the player has no
// absolute "level" and nudges the view up or down until it sits right. Every camera path adds it to the HEAD's
// pitch only. The hands, the shot and the compositor reticle never had it.
//
// In stereo VR that is not a comfort setting, it is a fault: the head's pose is the player's real one, so a trim
// tilts the whole view against everything that is tracked or drawn in the headset's own space. The reticle is a
// compositor quad placed along the controller's direction from the real centre eye; with a trim of T the world
// point that ray hits is seen T lower (or higher), so the reticle floats above (or below) every shot by T.
//
// INCIDENT (2026-10-07, Halo: Reach, a player session): the game window received F9 twice, 14 s apart, while the
// player was in the headset. Nothing on screen says so. The trim went to 2.0 and then 4.0 degrees, and the report
// was "my aim reticle is higher than my shot by a good bit": about 1.4 m at 20 m. The session before it had
// taken three such presses (6 degrees). The trim is not saved, so each launch began at zero again, which is why
// it looked like something that "broke" part-way through a session.
//
// So: a stored trim applies only while stereo is off, and a key press in stereo is ignored (it must not bank a
// trim that appears the next time stereo goes off). Pure and unit-tested (tests/pitch_trim_tests.cpp).
#include <cmath>

namespace pitch_trim
{
inline constexpr float kStepRad = 0.035f;   // one key press, about 2 degrees
inline constexpr float kLimitRad = 0.8f;    // about 46 degrees either way

// The trim a head camera adds to the tracked pitch right now.
inline float Effective(float storedRad, bool stereo) noexcept
{
    return stereo || !std::isfinite(storedRad) ? 0.0f : storedRad;
}

// One F8 (dir < 0) or F9 (dir > 0) press. Returns the stored trim afterwards; `ignored` is true when the press
// changed nothing because stereo VR is on.
inline float Step(float storedRad, int dir, bool stereo, bool& ignored) noexcept
{
    ignored = stereo;
    if (stereo)
        return storedRad;
    const float from = std::isfinite(storedRad) ? storedRad : 0.0f;
    const float to = from + (dir > 0 ? kStepRad : dir < 0 ? -kStepRad : 0.0f);
    return to < -kLimitRad ? -kLimitRad : to > kLimitRad ? kLimitRad : to;
}
} // namespace pitch_trim
