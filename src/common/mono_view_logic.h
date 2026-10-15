#pragma once

// MONO RENDERING: one picture of the world a frame, drawn from between the eyes and shown to both.
//
// The game's own work for a frame is close to proportional to what it draws, and stereo draws it twice. On a map
// with much in view (a large battle, little culling) the second eye's pass is what takes a frame past the
// headset's period. With this on the world is drawn ONCE, from the midpoint of the two eyes, and each eye is
// shown its own part of that one picture. Nothing in the picture has binocular disparity, so it all sits at
// infinity: flat, but still head tracked, and with one pass of the game's drawing instead of two (what a frame
// costs that is not per pass stays, so a frame is not halved).
//
// THE PICTURE. One camera at the midpoint of the eyes, looking the way the head looks, with a symmetric frustum
// wide enough for either eye (MonoCover): an eye sees further outward than inward, so the cover is the larger of
// the two eyes' outward angles. Each eye's own frustum is then a sub-rectangle of the cover, and the submit path
// crops it out as it already does from an eye's own symmetric raster. A direction in the picture is the same
// direction in both eyes.
//
// WHICH PASS. A title draws its two eyes in an order the player can set, and does its once-a-frame work (the
// crosshair's capture, for one) in the first of them. The one pass of a mono frame is drawn AS that first eye, so
// all of that work happens in it unchanged, and it is the title's LAST pass too: whatever the engine does once a
// frame after its final view, it does after this one.
//
// WHAT IS LAID OVER THE PICTURE. A layer placed at its true distance (the crosshair on what it marks, the scope's
// pane on the weapon) would be the only thing in view WITH disparity: eye separation over distance, more than a
// degree at 3 m, and with the eyes converged on the picture it doubles and no longer sits on what it marks. So
// while the picture is mono such a layer slides out along its own ray from the midpoint of the eyes to
// kMonoFlatDistanceM, its size scaled to keep its angle (FlattenQuad): direction and apparent size unchanged,
// disparity under a twentieth of a degree. Panels that are not registered to anything in the world (the menus,
// subtitles) keep their own depth: they float in front of the picture, as they should.
//
// WHERE IT IS REFUSED (MonoRefusal): displays whose eyes do not look the same way (canted: an eye's frustum is
// then not a sub-rectangle of a straight-ahead picture), while a cutscene is shown as a stereo theatre, while an
// upscaler reconstructs each eye from its own history, and in a title whose renderer has turned down a run of
// single passes (MonoRejections: the player gets the stereo picture back instead of none). Refused, the frame is
// drawn in stereo as before.
//
// No Windows or OpenXR calls here; the unit test is tests/mono_view_tests.cpp.

#include <cmath>

namespace mono_view
{
inline constexpr float kMonoFlatDistanceM = 100.0f;
inline constexpr float kMaxEyeCantDeg = 0.5f;

// Angles as OpenXR gives them, radians: left and down negative, right and up positive.
struct Fov
{
    float left = 0.0f, right = 0.0f, up = 0.0f, down = 0.0f;
};
inline bool FovValid(const Fov& f)
{
    return std::isfinite(f.left) && std::isfinite(f.right) && std::isfinite(f.up) && std::isfinite(f.down) &&
        f.left < 0.0f && f.right > 0.0f && f.up > 0.0f && f.down < 0.0f && f.left > -1.55f && f.right < 1.55f &&
        f.up < 1.55f && f.down > -1.55f;
}

// The symmetric cover of both eyes, as tangents of its half angles. False: an eye's angles are not usable.
inline bool MonoCover(const Fov& a, const Fov& b, float& tanX, float& tanY)
{
    if (!FovValid(a) || !FovValid(b))
        return false;
    const auto larger = [](float p, float q) { return p > q ? p : q; };
    const float halfX = larger(larger(-a.left, a.right), larger(-b.left, b.right));
    const float halfY = larger(larger(a.up, -a.down), larger(b.up, -b.down));
    tanX = std::tan(halfX);
    tanY = std::tan(halfY);
    return std::isfinite(tanX) && std::isfinite(tanY) && tanX > 0.0f && tanY > 0.0f;
}

// The angle between two eyes' orientations (x, y, z, w), degrees. 180 when either is not a rotation.
inline float EyeCantDeg(const float a[4], const float b[4])
{
    const float la = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
    const float lb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2] + b[3] * b[3]);
    if (!std::isfinite(la) || !std::isfinite(lb) || la < 1.0e-4f || lb < 1.0e-4f)
        return 180.0f;
    float dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]) / (la * lb);
    dot = dot < 0.0f ? -dot : dot;   // q and -q are one rotation
    if (dot > 1.0f)
        dot = 1.0f;
    return 2.0f * std::acos(dot) * 57.29578f;
}

enum class MonoRefusal
{
    None,          // the frame is drawn mono
    Off,           // not asked for
    Title,         // this title's stereo is not built from one render a frame here
    NoViews,       // the headset's eyes are not located yet
    Canted,        // the eyes do not look the same way
    Theatre,       // a cutscene is being shown as a stereo theatre
    Upscaler,      // each eye is being reconstructed from its own history
    Rejected,      // this title's renderer turned down a run of single passes
    Count
};
inline const char* MonoRefusalName(MonoRefusal r)
{
    switch (r)
    {
    case MonoRefusal::None: return "drawn mono";
    case MonoRefusal::Off: return "off";
    case MonoRefusal::Title: return "not available in this title";
    case MonoRefusal::NoViews: return "the headset's eyes are not located yet";
    case MonoRefusal::Canted: return "this headset's eyes do not look the same way (canted displays)";
    case MonoRefusal::Theatre: return "a cutscene is being shown as a stereo theatre";
    case MonoRefusal::Upscaler: return "the upscaler is reconstructing each eye from its own history";
    case MonoRefusal::Rejected: return "this game's renderer did not accept a single pass, so it is left in stereo";
    default: return "?";
    }
}
struct MonoInput
{
    bool asked = false;
    bool titleSupported = false;
    bool viewsLocated = false;
    float eyeCantDeg = 180.0f;
    bool theatre = false;
    bool upscaler = false;
    bool rejected = false;
};
inline MonoRefusal DecideMono(const MonoInput& in)
{
    if (!in.asked)
        return MonoRefusal::Off;
    if (!in.titleSupported)
        return MonoRefusal::Title;
    if (!in.viewsLocated)
        return MonoRefusal::NoViews;
    if (!(in.eyeCantDeg <= kMaxEyeCantDeg))
        return MonoRefusal::Canted;
    if (in.theatre)
        return MonoRefusal::Theatre;
    if (in.upscaler)
        return MonoRefusal::Upscaler;
    if (in.rejected)
        return MonoRefusal::Rejected;
    return MonoRefusal::None;
}

// A title's renderer can turn a single pass down (its own checks on a frame fail). One such frame is a frame
// dropped, as in stereo; a run of them is a title that cannot be drawn this way, and it is left in stereo until
// the setting is switched off and on again or the title is loaded anew. A frame that is drawn ends a run.
inline constexpr unsigned kMonoRejectionsToRefuse = 30;
struct MonoRejections
{
    unsigned run = 0;
    bool refused = false;
    void Rendered() noexcept { run = 0; }
    // True on the rejection that makes the refusal.
    bool Rejected() noexcept
    {
        if (refused)
            return false;
        if (++run < kMonoRejectionsToRefuse)
            return false;
        refused = true;
        return true;
    }
    void Reset() noexcept { run = 0; refused = false; }
};

// A layer registered to a point of the world, moved to where it has no disparity: out along the ray from the
// midpoint of the eyes (`centre`) through its own position, to `farM`, its size scaled by the same factor.
// False (nothing changed): it is already that far, it sits on the midpoint, or a number is not finite.
inline bool FlattenQuad(const float centre[3], float position[3], float size[2], float farM = kMonoFlatDistanceM)
{
    const float d[3]{position[0] - centre[0], position[1] - centre[1], position[2] - centre[2]};
    const float distance = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (!std::isfinite(distance) || !std::isfinite(farM) || !std::isfinite(size[0]) || !std::isfinite(size[1]) ||
        distance < 0.01f || farM <= distance)
        return false;
    const float k = farM / distance;
    for (int axis = 0; axis < 3; ++axis)
        position[axis] = centre[axis] + d[axis] * k;
    size[0] *= k;
    size[1] *= k;
    return true;
}
} // namespace mono_view
