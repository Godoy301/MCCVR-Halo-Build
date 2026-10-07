// REACH: NAVIGATION MARKERS SIT ON THEIR TARGETS, the pure half (no game, engine or OpenXR includes). Objective
// waypoints, marked players, name tags and every other HUD element anchored to a point in the world.
// The write-up is docs/REACH-NAV-MARKERS-2026-10-07.md; the runtime half is src/dll/reach_nav_markers.inl.
//
// WHAT REACH DOES (HREK first, then matched in retail).
// HREK reach_tag_test.exe, chud_draw.cpp: ONE function turns a world point into a place on the HUD (+0x92D980;
// retail haloreach.dll +0x2E1430, eleven direct callers). Every world-anchored element reaches it: the targets a
// user's HUD tracks, the object / player / scripted-object anchors, the flag markers, the navpoint list, the
// damage arrows. It does not use the view being drawn. It BUILDS a camera of its own:
//     camera     = render_camera_from_observer_camera(observer_get_camera(user))   ; position, forward, up and
//                                                                                 ; the vertical field of view
//     projection = render_projection_from_render_camera(camera, the user's window)
//     pixel      = render_camera_view_to_screen(projection, point - camera.position)
// and the widget is then drawn at that pixel, in the same pass that draws the rest of the HUD over the eye.
//
// In this mod the eye is rasterised from the HEAD with a field of view that covers the headset's (the compact
// camera's +0x28, written by ReachApplyEyeOffset), while the observer camera is the one aim steering points down
// the controller ray. The existing observer-camera hook hands the marker projection the eye's position, forward
// and up at its call site (the "CHUD site"), which took the markers off the hand; but it left the FIELD OF VIEW
// the game's own, about half the eye's. A marker was therefore right only on the eye's axis and was pushed
// outward everywhere else by tan(eye half-angle) / tan(game half-angle): turn the head ten degrees away from an
// objective and its marker moved twenty-odd.
//
// Halo 3 does not have the fault, and it is the model: its marker projection (halo3.dll +0x2F2424) builds no
// camera. It reads the CURRENT VIEW's own rasteriser camera and projection, the pair this mod rebuilds per eye.
//
// THE SEAM, part 1 (this header's first half): at the CHUD site the camera is also given the eye's vertical field
// of view. Reach derives its horizontal scale from that one angle and the window's aspect, and rasterises a
// frustum that is symmetric about the camera's axis in both directions (it has no other kind: a headset's
// asymmetric or canted eye is covered by a symmetric frustum about the eye's own axis and cropped at submission),
// so with the same position, forward, up and vertical field of view the game builds, for the marker, the very
// projection the eye was rasterised with. The marker then lands on its target's own pixel in each eye: on the
// target in both eyes, at the target's depth, with no lag, and in the image the compositor reprojects.
// Part 1 also leaves world-anchored elements out of the HUD height shift, which moved them off their targets by
// the player's HUD height setting.
//
// Nothing here is read by a shot, an aim or the network: the camera is the marker projection's private copy, and
// the anchor basis is HUD presentation.
#pragma once

#include <cmath>
#include <cstdint>

namespace reach_nav_markers
{
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kDegToRad = kPi / 180.0f;
inline constexpr float kRadToDeg = 180.0f / kPi;

// ---- the camera's vertical field of view -----------------------------------------------------------------------
// ADDR-HYGIENE: structural + guarded. The render camera's vertical field of view is at +0x28: the game's own
// converter stores it there (retail +0x287E6E / +0x287F1B `movss [rbx + 0x28], xmm0`, the copy block
// reach_render_logic.h already documents), the frustum builder reads it from there (+0x2882CF
// `movss xmm0, [r8 + 0x28]` then tanf), the marker projection reads the same field of the camera it built for
// its off-screen ring (+0x2E172B `movss xmm1, [rbp + 0xA8]`, the camera being at rbp + 0x80), and this mod's own
// per-eye camera writes the eye's angle there (ReachApplyEyeOffset). tools/verify-reach-nav-markers.py decodes
// all three game sites from the pinned module. The observer-camera hook this rides on is installed only on that
// pinned image. At every use both values must also lie in the range the game itself asserts for this field
// (HREK render_camera.cpp: > 0.0001 and < 3.14149), or nothing is written.
inline constexpr unsigned kCameraVerticalFovOffset = 0x28;
inline constexpr float kMinVerticalFov = 0.0001f;
inline constexpr float kMaxVerticalFov = 3.14149f;

inline bool FovPlausible(float verticalFov)
{
    return std::isfinite(verticalFov) && verticalFov > kMinVerticalFov && verticalFov < kMaxVerticalFov;
}

// What the CHUD site did with the camera's field of view. Identity only: never refused for being large or small
// inside the game's own range.
enum class Fov : uint8_t
{
    Used,       // the camera now carries the eye's vertical field of view
    SwitchOff,  // the player's switch is off: the game's own field of view stands
    NoGameFov,  // the camera's field is not a field of view (not finite, or outside the game's own range)
    NoEyeFov,   // the eye's camera has no usable field of view
    Count
};
inline const char* FovName(Fov use)
{
    switch (use)
    {
    case Fov::Used: return "used";
    case Fov::SwitchOff: return "switch off";
    case Fov::NoGameFov: return "the game's camera has no field of view where it should";
    case Fov::NoEyeFov: return "the eye's camera has no field of view";
    default: return "?";
    }
}
// `gameFov`: what the game's converter left in the marker camera. `eyeFov`: the same field of the camera this
// eye was rasterised with. Used = `out` is the eye's; anything else leaves `out` untouched.
inline Fov EyeFov(bool switchOn, float gameFov, float eyeFov, float& out)
{
    if (!switchOn)
        return Fov::SwitchOff;
    if (!FovPlausible(gameFov))
        return Fov::NoGameFov;
    if (!FovPlausible(eyeFov))
        return Fov::NoEyeFov;
    out = eyeFov;
    return Fov::Used;
}

// HOW FAR OFF THE STOCK PLACEMENT WAS. A point at `offAxis` radians from the eye's axis belongs at
// tan(offAxis) / tan(eye / 2) of the raster's half extent; projected with the game's angle it was drawn at
// tan(offAxis) / tan(game / 2). The ratio of the two is the same everywhere: the magnification.
inline float Magnification(float gameFov, float eyeFov)
{
    if (!FovPlausible(gameFov) || !FovPlausible(eyeFov))
        return 1.0f;
    return std::tan(eyeFov * 0.5f) / std::tan(gameFov * 0.5f);
}
// Where a marker whose target is `offAxis` radians off the eye's axis was SEEN, in radians off that axis.
inline float StockSeenOffAxis(float offAxis, float gameFov, float eyeFov)
{
    return std::atan(std::tan(offAxis) * Magnification(gameFov, eyeFov));
}
// The same error in pixels of a raster `rasterExtentPx` high (or wide, with the horizontal half-angles).
inline float StockErrorPixels(float offAxis, float gameFov, float eyeFov, float rasterExtentPx)
{
    if (!FovPlausible(eyeFov))
        return 0.0f;
    return std::tan(offAxis) * (Magnification(gameFov, eyeFov) - 1.0f) / std::tan(eyeFov * 0.5f) * rasterExtentPx *
           0.5f;
}

// ---- the game's projection, as a model for the tests -----------------------------------------------------------
// retail +0x2881CC (frustum from a camera, no bounds given: -1..1 in both axes), +0x2884BC (the projection) and
// +0x288C10 (a view-space vector to a pixel), read from the pinned module:
//     t = tan(verticalFov / 2);   aspect = window width / window height
//     x = 0.5 + 0.5 * (d . right) / ((d . forward) * aspect * t)         ; of the window's width
//     y = 0.5 - 0.5 * (d . up)    / ((d . forward) * t)                  ; of its height, down the screen
// with d = point - camera.position and right = forward x up. A point at or behind the camera's plane has no pixel.
struct Camera
{
    float position[3]{};
    float forward[3]{};
    float up[3]{};
    float verticalFov = 0.0f;
};
inline float Dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline void Cross(const float a[3], const float b[3], float out[3])
{
    const float x = a[1] * b[2] - a[2] * b[1];
    const float y = a[2] * b[0] - a[0] * b[2];
    const float z = a[0] * b[1] - a[1] * b[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}
// False (out untouched) when the point is not in front of the camera or an input is not usable.
inline bool ProjectToRaster(const Camera& camera, const float point[3], float width, float height, float out[2])
{
    if (!FovPlausible(camera.verticalFov) || !(width > 0.0f) || !(height > 0.0f))
        return false;
    const float d[3]{point[0] - camera.position[0], point[1] - camera.position[1], point[2] - camera.position[2]};
    float right[3];
    Cross(camera.forward, camera.up, right);
    const float depth = Dot(d, camera.forward);
    if (!(depth > 0.0001f))
        return false;
    const float t = std::tan(camera.verticalFov * 0.5f);
    const float aspect = width / height;
    const float x = (0.5f + 0.5f * Dot(d, right) / (depth * aspect * t)) * width;
    const float y = (0.5f - 0.5f * Dot(d, camera.up) / (depth * t)) * height;
    if (!std::isfinite(x) || !std::isfinite(y))
        return false;
    out[0] = x;
    out[1] = y;
    return true;
}

// ---- the anchors -----------------------------------------------------------------------------------------------
// HREK chud_draw.cpp: the anchor basis producer (reach_tag_test.exe +0x91C490; retail +0x2DB54C, the function the
// HUD height hook already wraps) switches on a widget's authored anchor type. The official enum table
// (+0x20875C0) names the types; the producer's own jump table (+0x91D1A4, 51 entries) says what each does.
// These end at a point in the WORLD: their case reaches the marker projection, directly or through the object
// path (+0x91DB80 -> +0x925000 -> +0x92D980), or copies a point that projection produced for the user's HUD
// earlier in the same pass (+0x91D270, from the targets update +0x91EFD0):
//     15 tracked target      16 <tracking object>     21 weapon target (when the weapon has one)
//     22 ghost reticule      23 hologram target       24 airstrike target
//     25 <player>            the name tags and markers over other players
//     26 <scripted object>   the waypoints game types and scripts put on objects, flags and points
//     40..49                 fireteam and objective objects (enemy / friendly / neutral objective object)
// Every one of them, when it succeeds, returns true and clears the producer's sixth output (the "native anchor"
// byte). Anchor 21 falls back to the crosshair's own place when the weapon has no target, and then leaves that
// byte set; so the byte tells the two apart. The scoreboard and metagame anchors (27..33) clear it too, which is
// why it cannot stand in for the type.
// ADDR-HYGIENE: structural. The numbers are the tag format's enum, the same in the kit and in retail maps, read
// from HREK's own table and switch (tools/verify-reach-nav-markers.py re-derives the set from both images).
inline constexpr int kAnchorTrackedTarget = 15, kAnchorTrackingObject = 16, kAnchorWeaponTarget = 21,
                     kAnchorGhostReticule = 22, kAnchorHologramTarget = 23, kAnchorAirstrikeTarget = 24,
                     kAnchorPlayer = 25, kAnchorScriptedObject = 26, kAnchorFirstObjectAnchor = 40,
                     kAnchorLastObjectAnchor = 49;
inline constexpr bool AnchorIsWorld(int anchorType)
{
    return anchorType == kAnchorTrackedTarget || anchorType == kAnchorTrackingObject ||
           (anchorType >= kAnchorWeaponTarget && anchorType <= kAnchorScriptedObject) ||
           (anchorType >= kAnchorFirstObjectAnchor && anchorType <= kAnchorLastObjectAnchor);
}
// Did this call of the producer put its basis on a point in the world? `produced`: its return value.
// `flagRead` / `nativeAnchor`: its sixth output, when the caller gave one and it could be read.
inline constexpr bool WorldAnchorProduced(int anchorType, bool produced, bool flagRead, bool nativeAnchor)
{
    return produced && AnchorIsWorld(anchorType) && flagRead && !nativeAnchor;
}
} // namespace reach_nav_markers
