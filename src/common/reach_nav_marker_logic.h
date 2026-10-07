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
#include <cstddef>
#include <cstdint>
#include <cstring>

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

// ====================================================================================================================
// THE MARKER PROJECTION ITSELF (HREK reach_tag_test.exe +0x92D980, chud_draw.cpp; retail +0x2E1430)
//
//     void project(record*, int user, bool ring, bool behind, bool worldPitch)
// The record: a valid byte at +0, the world point at +4, the pixel it lands on (relative to the user's window) at
// +0x10, the same place in the HUD's virtual canvas at +0x18, and at +0x20 how far outside the game's off-screen
// ring the point is (0 = drawn on the point itself). tools/audit_caller_area.py haloreach.dll 2E1430:9:v is
// clean: the function reads no caller slot past its fifth argument (entry+0x28) and none of its eleven native
// callers keeps a volatile register across the call or reads a return value.
//
// ADDR-HYGIENE: resolved. No address ships. Five patterns, each matched exactly once in the image, all inside the
// one function the first begins; the call the second carries must land on the function the observer-camera hook
// hooks and return to the address that hook knows as its CHUD site (so the function bound here is the one whose
// camera part 1 corrects); the vertical field of view the ring reads must be the camera's +0x28 (the ring's
// displacement minus the camera buffer's); the ring's fraction must be 0.3 and its two other constants pi and
// 0.0001 (ValueAgreement); the record's offsets are read from the matched instructions and must follow one
// another the way the record does. Any link failing binds nothing.
// ====================================================================================================================

// The head (retail +0x2E1430): the frame, the cookie, the HUD's window count and index, the record in rsi, the two
// bool arguments spilled (r9b, r8b), the user in r12, the record's valid byte tested at [rcx].
inline constexpr const char* kProjectionHeadPattern =
    "48 8B C4 48 89 58 18 55 56 57 41 54 41 55 41 56 41 57 48 8D A8 28 FE FF FF 48 81 EC A0 02 00 00 "
    "0F 29 70 B8 0F 29 78 A8 44 0F 29 40 98 44 0F 29 48 88 44 0F 29 90 78 FF FF FF "
    "44 0F 29 98 68 FF FF FF 44 0F 29 A0 58 FF FF FF 44 0F 29 B0 48 FF FF FF "
    "48 8B 05 ?? ?? ?? ?? 48 33 C4 48 89 85 10 01 00 00 8B 3D ?? ?? ?? ?? 45 33 ED 48 8B F1 "
    "8B 1D ?? ?? ?? ?? 44 88 4C 24 31 44 88 44 24 30 4C 63 E2 44 38 29 0F 84";
inline constexpr size_t kProjectionHeadBytes = 135;
// The camera it builds (retail +0x2E14FC): the user's observer camera (engine TLS slot, +0x154, 0x410 apart)
// converted into the camera at rbp + CAMERA; that camera's bounds; the projection; then the record's point minus
// the camera's position.
//   mov ecx, SLOT / imul rdx, r12, STRIDE / mov r9, [rax + r8*8] / mov rax, [r9 + rcx] / lea rcx, [rbp + CAMERA]
//   add rax, BASE / add rdx, rax / call CONVERTER / lea rcx, [rbp + CAMERA] / call / lea r9, [rbp + CAMERA+0x40]
//   mov edx, edi / lea r8, [rbp + CAMERA+0x38] / mov ecx, ebx / call / xorps xmm3, xmm3 / lea rcx, [rbp + CAMERA]
//   xor edx, edx / lea r8, [rbp - 0x60] / call PROJECTION / movss xmm0, [rsi + POINT] / lea rcx, [rsp + 0x50]
//   subss xmm0, [rbp + CAMERA] / movss xmm1, [rsi + POINT+4] / subss xmm1, [rbp + CAMERA+4]
inline constexpr const char* kProjectionCameraPattern =
    "B9 88 06 00 00 49 69 D4 10 04 00 00 4E 8B 0C C0 49 8B 04 09 48 8D 8D 80 00 00 00 "
    "48 05 54 01 00 00 48 03 D0 E8 ?? ?? ?? ?? 48 8D 8D 80 00 00 00 E8 ?? ?? ?? ?? "
    "4C 8D 8D C0 00 00 00 8B D7 4C 8D 85 B8 00 00 00 8B CB E8 ?? ?? ?? ?? 0F 57 DB "
    "48 8D 8D 80 00 00 00 33 D2 4C 8D 45 A0 E8 ?? ?? ?? ?? F3 0F 10 46 04 48 8D 4C 24 50 "
    "F3 0F 5C 85 80 00 00 00 F3 0F 10 4E 08 F3 0F 5C 8D 84 00 00 00";
inline constexpr size_t kProjectionCameraBytes = 128;
inline constexpr size_t kCameraBufferDisp = 23, kCameraConverterCall = 36, kCameraConverterReturn = 41,
                        kCameraPointX = 101, kCameraPositionXDisp = 111, kCameraPointY = 119,
                        kCameraPositionYDisp = 124;
// The off-screen ring (retail +0x2E172B): the camera's vertical field of view times a fraction is the ring's
// half-height in radians, and that times the HUD's width over its height its half-width.
//   movss xmm1, [rbp + CAMERA+FOV] / movaps xmm2, xmm11 / mulss xmm1, [rip + FRACTION] / movaps xmm7, xmm1 /
//   divss xmm2, xmm1 / mulss xmm7, xmm12 / movaps xmm1, xmm8 / mulss xmm2, xmm2 / divss xmm7, xmm14
inline constexpr const char* kProjectionRingPattern =
    "F3 0F 10 8D ?? ?? ?? ?? 41 0F 28 D3 F3 0F 59 0D ?? ?? ?? ?? 0F 28 F9 F3 0F 5E D1 F3 41 0F 59 FC "
    "41 0F 28 C8 F3 0F 59 D2 F3 41 0F 5E FE";
inline constexpr size_t kProjectionRingBytes = 45;
inline constexpr size_t kRingFovDisp = 4, kRingFractionDisp = 16, kRingFractionNext = 20;
// How far outside the ring (retail +0x2E1817): half-width / pi * (ratio - 1), at least 0.0001, into the record.
//   divss xmm7, [rip + PI] / subss xmm12, xmm10 / mulss xmm7, xmm12 / maxss xmm7, [rip + FLOOR] /
//   movss [rsi + OFFSCREEN], xmm7
inline constexpr const char* kProjectionOffscreenPattern =
    "F3 0F 5E 3D ?? ?? ?? ?? F3 45 0F 5C E2 F3 41 0F 59 FC F3 0F 5F 3D ?? ?? ?? ?? F3 0F 11 7E 20";
inline constexpr size_t kProjectionOffscreenBytes = 31;
inline constexpr size_t kOffscreenPiDisp = 4, kOffscreenPiNext = 8, kOffscreenFloorDisp = 22, kOffscreenFloorNext = 26,
                        kOffscreenRecord = 30;
// The pixel (retail +0x2E15BC): lea r14, [rsi + PIXEL] / movss xmm10, [rip + 1.0] / mov [r14], r13d /
// mov [rsi + PIXEL+4], r13d -- both cleared before the point is projected into them.
inline constexpr const char* kProjectionPixelPattern = "4C 8D 76 10 F3 44 0F 10 15 ?? ?? ?? ?? 45 89 2E 44 89 6E 14";
inline constexpr size_t kProjectionPixelBytes = 20;
inline constexpr size_t kPixelX = 3, kPixelY = 19;

// The game's own numbers for the ring: 0.3 of the vertical field of view, and the floor of "outside".
inline constexpr float kRingFraction = 0.3f;
inline constexpr float kOffscreenFloor = 0.0001f;

inline int32_t ReadI32(const uint8_t* at)
{
    int32_t value = 0;
    std::memcpy(&value, at, sizeof(value));
    return value;
}
inline float ReadF32(const uint8_t* at)
{
    float value = 0.0f;
    std::memcpy(&value, at, sizeof(value));
    return value;
}

struct ProjectionSites
{
    int64_t headRva = 0;             // the projection: the hook
    int64_t converterRva = 0;        // what its camera call lands on: render_camera_from_observer_camera
    int64_t converterReturnRva = 0;  // and where that call returns: the observer-camera hook's CHUD site
    int64_t fractionRva = 0, piRva = 0, floorRva = 0;  // the ring's three constants
    uint32_t cameraFovOffset = 0;    // the vertical field of view inside the camera it builds
    uint32_t recordPoint = 0;        // the record's world point
    uint32_t recordPixel = 0;        // its pixel, relative to the user's window
    uint32_t recordOffscreen = 0;    // how far outside the ring
};
enum class ProjectionBind : uint8_t
{
    Bound = 0,
    HeadMissing,       // the projection's head: no match, more than one, or not a function's beginning
    CameraMissing,     // the camera it builds
    RingMissing,       // the off-screen ring
    OffscreenMissing,  // the store of how far outside
    PixelMissing,      // the pixel's place in the record
    OutsideFunction,   // a site is not inside the function whose head matched, or they are out of order
    FovMismatch,       // the ring does not read the camera's vertical field of view where part 1 writes it
    RecordMismatch,    // the record's offsets do not follow one another as the record does
    ConverterMismatch, // the camera call is not the observer-camera hook's function at its CHUD site
    ConstantMismatch,  // the ring's fraction is not 0.3, or its pi / floor are not
    Count
};
inline const char* ProjectionBindName(ProjectionBind why)
{
    switch (why)
    {
    case ProjectionBind::Bound: return "bound";
    case ProjectionBind::HeadMissing: return "the marker projection's head is missing, ambiguous or not a function";
    case ProjectionBind::CameraMissing: return "the marker projection's camera build is missing or ambiguous";
    case ProjectionBind::RingMissing: return "the marker projection's off-screen ring is missing or ambiguous";
    case ProjectionBind::OffscreenMissing: return "the marker projection's off-screen store is missing or ambiguous";
    case ProjectionBind::PixelMissing: return "the marker projection's pixel store is missing or ambiguous";
    case ProjectionBind::OutsideFunction: return "a site is not inside the matched marker projection, in order";
    case ProjectionBind::FovMismatch: return "the ring does not read the camera's field of view at +0x28";
    case ProjectionBind::RecordMismatch: return "the marker record's offsets do not follow one another";
    case ProjectionBind::ConverterMismatch:
        return "the projection's camera call is not the observer-camera hook's call site";
    case ProjectionBind::ConstantMismatch: return "the ring's constants are not 0.3, pi and 0.0001";
    default: return "?";
    }
}
// Each pointer: the bytes at that pattern's unique match (null = missing or ambiguous), with its offset from the
// module base. `headIsFunction`: the image's function table begins a function at the head. `*FunctionRva`: the
// beginning of the function each other site lies in, from the same table. The caller then checks the converter
// against the observer-camera hook and reads the three constants for CheckProjectionConstants.
inline ProjectionBind DecodeProjectionSites(const uint8_t* head, int64_t headRva, bool headIsFunction,
                                            const uint8_t* camera, int64_t cameraRva, int64_t cameraFunctionRva,
                                            const uint8_t* ring, int64_t ringRva, int64_t ringFunctionRva,
                                            const uint8_t* offscreen, int64_t offscreenRva,
                                            int64_t offscreenFunctionRva, const uint8_t* pixel, int64_t pixelRva,
                                            int64_t pixelFunctionRva, ProjectionSites& out)
{
    if (!head || !headIsFunction)
        return ProjectionBind::HeadMissing;
    if (!camera)
        return ProjectionBind::CameraMissing;
    if (!ring)
        return ProjectionBind::RingMissing;
    if (!offscreen)
        return ProjectionBind::OffscreenMissing;
    if (!pixel)
        return ProjectionBind::PixelMissing;
    if (cameraFunctionRva != headRva || ringFunctionRva != headRva || offscreenFunctionRva != headRva ||
        pixelFunctionRva != headRva || !(headRva < cameraRva && cameraRva < pixelRva && pixelRva < ringRva &&
                                         ringRva < offscreenRva))
        return ProjectionBind::OutsideFunction;
    const int32_t cameraBuffer = ReadI32(camera + kCameraBufferDisp);
    const int32_t ringFov = ReadI32(ring + kRingFovDisp);
    // The camera's position is its first field: the point is subtracted from the buffer itself.
    if (ReadI32(camera + kCameraPositionXDisp) != cameraBuffer ||
        ReadI32(camera + kCameraPositionYDisp) != cameraBuffer + 4 ||
        ringFov - cameraBuffer != static_cast<int32_t>(kCameraVerticalFovOffset))
        return ProjectionBind::FovMismatch;
    const uint32_t point = camera[kCameraPointX], pointY = camera[kCameraPointY];
    const uint32_t pixelX = pixel[kPixelX], pixelY = pixel[kPixelY], off = offscreen[kOffscreenRecord];
    // valid byte (+0), then the point (three floats), then the pixel (two), the virtual place (two), then this.
    if (point < 4 || (point & 3) || pointY != point + 4 || pixelX != point + 12 || pixelY != pixelX + 4 ||
        off != pixelX + 16 || off >= 0x80)
        return ProjectionBind::RecordMismatch;
    ProjectionSites sites;
    sites.headRva = headRva;
    sites.converterRva = cameraRva + static_cast<int64_t>(kCameraConverterReturn) +
                         ReadI32(camera + kCameraConverterCall + 1);
    sites.converterReturnRva = cameraRva + static_cast<int64_t>(kCameraConverterReturn);
    sites.fractionRva = ringRva + static_cast<int64_t>(kRingFractionNext) + ReadI32(ring + kRingFractionDisp);
    sites.piRva = offscreenRva + static_cast<int64_t>(kOffscreenPiNext) + ReadI32(offscreen + kOffscreenPiDisp);
    sites.floorRva =
        offscreenRva + static_cast<int64_t>(kOffscreenFloorNext) + ReadI32(offscreen + kOffscreenFloorDisp);
    sites.cameraFovOffset = static_cast<uint32_t>(ringFov - cameraBuffer);
    sites.recordPoint = point;
    sites.recordPixel = pixelX;
    sites.recordOffscreen = off;
    if (camera[kCameraConverterCall] != 0xE8 || sites.converterRva <= 0 || sites.fractionRva <= 0 ||
        sites.piRva <= 0 || sites.floorRva <= 0)
        return ProjectionBind::ConverterMismatch;
    out = sites;  // a refusal fills nothing
    return ProjectionBind::Bound;
}
// `observerHookRva` / `chudSiteReturnRva`: the function the observer-camera hook is installed on and the return
// address it classifies as its CHUD site, both as offsets from the module base.
inline ProjectionBind CheckProjectionConverter(const ProjectionSites& sites, int64_t observerHookRva,
                                               int64_t chudSiteReturnRva)
{
    return observerHookRva > 0 && sites.converterRva == observerHookRva &&
                   sites.converterReturnRva == chudSiteReturnRva
               ? ProjectionBind::Bound
               : ProjectionBind::ConverterMismatch;
}
// The four bytes at each of sites.fractionRva, piRva and floorRva.
inline ProjectionBind CheckProjectionConstants(const uint8_t* fraction, const uint8_t* pi, const uint8_t* floor)
{
    if (!fraction || !pi || !floor)
        return ProjectionBind::ConstantMismatch;
    return std::fabs(ReadF32(fraction) - kRingFraction) <= 1.0e-6f && std::fabs(ReadF32(pi) - kPi) <= 1.0e-6f &&
                   std::fabs(ReadF32(floor) - kOffscreenFloor) <= 1.0e-9f
               ? ProjectionBind::Bound
               : ProjectionBind::ConstantMismatch;
}

// ---- the witness (dev builds): the game's pixel against the eye's own picture ----------------------------------
// The eye's raster as the mod committed it: the camera it was rasterised from, the two scales of the projection
// matrix the game derived for it (|m00|, |m11|: 1 / (aspect * t) and 1 / t), and the window it fills.
struct EyeRaster
{
    Camera camera;
    float xScale = 0.0f, yScale = 0.0f;
    float width = 0.0f, height = 0.0f;
};
// Where the eye's own picture shows `point`, by the projection matrix it was rasterised with (not by a field of
// view). False (out untouched) when the point is not in front of the eye or the raster is not usable.
inline bool EyeRasterPixel(const EyeRaster& eye, const float point[3], float out[2])
{
    if (!(eye.xScale > 0.0f) || !(eye.yScale > 0.0f) || !(eye.width > 0.0f) || !(eye.height > 0.0f) ||
        !std::isfinite(eye.xScale) || !std::isfinite(eye.yScale))
        return false;
    const float d[3]{point[0] - eye.camera.position[0], point[1] - eye.camera.position[1],
                     point[2] - eye.camera.position[2]};
    float right[3];
    Cross(eye.camera.forward, eye.camera.up, right);
    const float depth = Dot(d, eye.camera.forward);
    if (!(depth > 0.0001f))
        return false;
    const float x = (0.5f + 0.5f * eye.xScale * Dot(d, right) / depth) * eye.width;
    const float y = (0.5f - 0.5f * eye.yScale * Dot(d, eye.camera.up) / depth) * eye.height;
    if (!std::isfinite(x) || !std::isfinite(y))
        return false;
    out[0] = x;
    out[1] = y;
    return true;
}
// The angle between the camera's axis and the point, in radians; 0 for a point at the camera.
inline float OffAxis(const Camera& camera, const float point[3])
{
    const float d[3]{point[0] - camera.position[0], point[1] - camera.position[1], point[2] - camera.position[2]};
    const float length = std::sqrt(Dot(d, d));
    if (!(length > 1.0e-6f))
        return 0.0f;
    float c = Dot(d, camera.forward) / length;
    c = c > 1.0f ? 1.0f : c < -1.0f ? -1.0f : c;
    return std::acos(c);
}
// What the witness made of one projection. Counted by reason.
enum class Witness : uint8_t
{
    Compared,     // drawn on the point itself: the game's pixel was compared with the eye's own
    OnRing,       // the game put it on its off-screen ring: nothing to compare
    NotValid,     // the record's valid byte is clear: the game projected nothing
    NoEyeScope,   // not inside an eye's own render
    NotInFront,   // the point is not in front of the eye
    NoRaster,     // the eye's raster is not usable
    Fault,        // the record could not be read
    Count
};
inline const char* WitnessName(Witness what)
{
    switch (what)
    {
    case Witness::Compared: return "compared";
    case Witness::OnRing: return "on the off-screen ring";
    case Witness::NotValid: return "not a valid record";
    case Witness::NoEyeScope: return "not inside an eye's render";
    case Witness::NotInFront: return "not in front of the eye";
    case Witness::NoRaster: return "no eye raster";
    case Witness::Fault: return "record read fault";
    default: return "?";
    }
}
} // namespace reach_nav_markers
