// HEAD-RELATIVE MOTION TRACKER, Halo 3 (and ODST, which carries the same code): the pure half of
// src/dll/halo3_head_relative.inl. No game, engine or OpenXR includes.
//
// WHAT THE GAME DOES (retail halo3.dll 1.3528, read offline 2026-10-06; no editing kit on this machine, so every
// name below is ours; tools/verify-halo3-motion-tracker.py re-derives the addresses from the patterns here).
//
//   The HUD update (0x2CF7F8) calls the motion tracker's per-user update (0x32DA98) with the user's tracker state
//   (inside the per-user HUD block, engine TLS +0xF0, stride 0x28C8; the output user index at state +0xCB0). That
//   update takes ONE heading: the player-control facing yaw of its user,
//       yaw = [engine TLS +0xC0] + user * 0x80 + 0x514          (ODST: [TLS +0xD8] + user * 0x230 + 0xBD4)
//   -- the angle direct drive steers with the hand, on foot and in every seat alike (no vehicle branch, no camera
//   read) -- and builds
//       a = -(yaw + pi/2);   S = sinf(a);   C = cosf(a)
//   which it hands to the two builders that place a contact:
//       0x32D9A8  a world point (objective marks):  arguments 5 and 6 = S, C
//       0x32D680  a tracked object:                 arguments 6 and 7 = S, C
//   Each computes, for the horizontal offset d = contact - unit (clamped to the range radially first),
//       blip.x = (d.y * S - d.x * C) / range        blip.y = (d.x * S + d.y * C) / range
//   With S = -cos(yaw) and C = -sin(yaw) that is blip.x = d . right, blip.y = -(d . forward), where
//   forward = (cos yaw, sin yaw) and right = (sin yaw, -cos yaw): the tracker's "up" is the control yaw's heading.
//   The draw (0x2F3904) only paints what the update stored.
//
// THE SEAM: the builders' (S, C) arguments, and nothing else. The detours on the two builders (their only caller
// is the update) replace the pair with the pair of the HEAD's heading. Nothing is read from or written to player
// control, the aim record, the camera or the observer; the builders' own outputs are the tracker's blips.
//
// THE SIGN is not a guess here: the substitution hands the game the same quantity it built (a world heading in
// its own convention), so a contact straight ahead of the head lands where a contact straight ahead of the aim
// landed before. tracker_head_sign / tracker_head_offset_deg stay as the player's safety values.
#pragma once

#include "head_relative_logic.h"

#include <cmath>
#include <cstdint>

namespace halo3_head_relative
{
// Blam's world up.
inline constexpr float kWorldUp[3]{0.0f, 0.0f, 1.0f};

// ---- the signatures (halo3.dll and halo3odst.dll: each matches exactly once in both; the verifier checks) ----
// ADDR-HYGIENE: resolved. No address or offset below is a constant of ours: every one is decoded from a pattern
// that must match exactly once, and each hook target is proven twice (the update's call operand AND the builder's
// own entry pattern at that address), with ValueAgreement on the rest (the install lists them).
//
// The tracker update's entry: ... mov ecx,[rcx+USER]; mov rsi,rdx; ...; call <unit of output user>; cmp eax,-1.
// USER (the output user index in the tracker state) is the disp32 at +41; the call's rel32 at +53.
inline constexpr char kUpdateEntryPattern[] =
    "48 8B C4 48 89 58 08 48 89 68 10 48 89 70 18 48 89 78 20 41 54 41 56 41 57 48 81 EC 80 00 00 00 48 8B F9 "
    "0F 29 70 D8 8B 89 ?? ?? 00 00 48 8B F2 0F 29 78 C8 E8 ?? ?? ?? ?? 83 F8 FF";
inline constexpr int kUpdateUserDisp = 41, kUpdateUnitCall = 52;
// The heading's trig, inside the update: addss xmm6,[pi/2]; xorps xmm6,[sign mask]; movaps xmm0,xmm6; call A;
// movaps xmm7,xmm0; movaps xmm0,xmm6; call B; ...; movaps xmm6,xmm0. So xmm7 = A(a), xmm6 = B(a), a = -(yaw+pi/2).
// pi/2 is the rel32 at +4, the mask at +11, call A at +18, call B at +29.
inline constexpr char kTrigPattern[] =
    "F3 0F 58 35 ?? ?? ?? ?? 0F 57 35 ?? ?? ?? ?? 0F 28 C6 E8 ?? ?? ?? ?? 0F 28 F8 0F 28 C6 E8 ?? ?? ?? ?? "
    "49 63 C4 48 8D 9F ?? ?? 00 00 48 69 C8 ?? ?? 00 00 0F 28 F0";
inline constexpr int kTrigHalfPi = 4, kTrigMask = 11, kTrigCallA = 18, kTrigCallB = 29;
// The update's call of the point builder: [rsp+0x28] = xmm6 (argument 6), [rsp+0x20] = xmm7 (argument 5).
inline constexpr char kPointCallPattern[] =
    "F3 0F 11 74 24 28 48 8D 56 ?? F3 0F 11 7C 24 20 4C 8D 4C 24 ?? 48 8B CF E8 ?? ?? ?? ?? 84 C0";
inline constexpr int kPointCall = 24;
// Its two calls of the object builder: [rsp+0x30] = xmm6 (argument 7), [rsp+0x28] = xmm7 (argument 6); argument 5
// is null (a tracked object) or a world point (an object the HUD globals list).
inline constexpr char kObjectCallAPattern[] =
    "F3 0F 11 74 24 30 4C 03 C3 F3 0F 11 7C 24 28 48 83 64 24 20 00 48 8B CF E8 ?? ?? ?? ?? 84 C0";
inline constexpr int kObjectCallA = 24;
inline constexpr char kObjectCallBPattern[] =
    "F3 0F 11 74 24 30 4C 03 C3 F3 0F 11 7C 24 28 83 CA FF 48 89 44 24 20 48 8B CF E8 ?? ?? ?? ?? 84 C0";
inline constexpr int kObjectCallB = 26;
// The point builder's entry, and its placement (argument 5 at [rsp+0x60], argument 6 at [rsp+0x68]).
inline constexpr char kPointEntryPattern[] =
    "48 8B C4 48 89 58 08 48 89 70 10 57 48 83 EC 30 F3 0F 10 12 49 8B F8 F3 41 0F 5C 11 F3 0F 10 5A 04 48 8B F1 "
    "F3 41 0F 5C 59 04";
inline constexpr char kPointPlacePattern[] =
    "0F 28 C2 0F 28 CB F3 0F 59 4C 24 60 B0 01 32 D8 F3 0F 59 54 24 60 F3 0F 59 44 24 68 8D 4B 05 02 D8 "
    "F3 0F 59 5C 24 68 F3 0F 5C C8 F3 0F 58 D3";
// The object builder's entry, and its placement (argument 6 at [rsp+0xA8], argument 7 at [rsp+0xB0]).
inline constexpr char kObjectEntryPattern[] =
    "48 8B C4 48 89 58 08 48 89 68 18 48 89 70 20 89 50 10 57 41 54 41 55 41 56 41 57 48 83 EC 50 4C 63 E2 33 DB "
    "41 83 FC FF 0F 29 70 C8 0F 29 78 B8 4D 8B F8 4C 8B F1";
inline constexpr char kObjectPlacePattern[] =
    "0F 28 C6 F3 0F 59 B4 24 A8 00 00 00 F3 0F 59 84 24 B0 00 00 00 0F 28 CF F3 0F 59 8C 24 A8 00 00 00 "
    "F3 0F 59 BC 24 B0 00 00 00 F3 0F 5C C8 F3 0F 58 F7";
// How far past its entry a site of the same function may lie (the update is 0x2DB bytes, the builders 0xEF and
// 0x327; a site further away is in another function).
inline constexpr uint32_t kUpdateExtent = 0x400, kPointExtent = 0x100, kObjectExtent = 0x340;

// ---- the model of the game's arithmetic ----
// The pair the update builds for a heading yaw (0x32DB31..0x32DB57).
inline void NativePair(float yaw, float& sinA, float& cosA)
{
    const float a = -(yaw + head_relative::kPi * 0.5f);
    sinA = std::sin(a);
    cosA = std::cos(a);
}
// The heading a pair stands for: forward = (-S, -C, 0). False (out untouched) when the pair is not finite or is
// not a unit pair (it is sin and cos of one angle, or it is not the tracker's pair).
inline bool PairForward(float sinA, float cosA, float out[3])
{
    if (!out || !std::isfinite(sinA) || !std::isfinite(cosA))
        return false;
    const float length = std::sqrt(sinA * sinA + cosA * cosA);
    if (!(std::fabs(length - 1.0f) <= 1.0e-3f))
        return false;
    out[0] = -sinA / length;
    out[1] = -cosA / length;
    out[2] = 0.0f;
    return true;
}
// The pair of a horizontal heading: S = -x, C = -y, normalised. False when it has no horizontal part.
inline bool ForwardPair(const float heading[3], float& sinA, float& cosA)
{
    if (!head_relative::Finite3(heading))
        return false;
    const float length = std::sqrt(heading[0] * heading[0] + heading[1] * heading[1]);
    if (!(length > 1.0e-6f))
        return false;
    sinA = -heading[0] / length;
    cosA = -heading[1] / length;
    return true;
}
// Where the builders put a contact at horizontal offset (dx, dy) from the unit (0x32D8FA, 0x32DA35): x to the
// right on the tracker, y DOWN (a contact straight ahead has y = -distance / range).
inline void Place(float dx, float dy, float sinA, float cosA, float invRange, float out[2])
{
    out[0] = (dy * sinA - dx * cosA) * invRange;
    out[1] = (dx * sinA + dy * cosA) * invRange;
}

// ---- the substitution ----
// The last heading the head had (looking straight up or down leaves none), per title generation.
struct Hold
{
    head_relative::HeadingHold heading;
    uint32_t generation = 0;
};

struct Request
{
    bool switchOn = false;        // g_config.tracker_follows_head, read live
    int32_t user = -1;            // the output user the tracker state belongs to
    bool sampleRead = false;      // a coherent head sample was read
    bool gameplay = false;        // the frame that published it was player-controlled (no cinematic)
    head_relative::HeadSample sample{};
    uint32_t generation = 0;      // the title generation now
    uint64_t nowUs = 0;           // the sight clock now
    int sign = 1;                 // g_config.tracker_head_sign
    float offsetRad = 0.0f;       // g_config.tracker_head_offset_deg, in radians
};

// The verdict on one contact placement and, when it is Use::Ok, the pair to hand the builder in sinA / cosA:
// the game's own pair turned about world up so its heading is the head's. Any other verdict leaves sinA / cosA
// exactly as the game made them. `nativeForward` / `usedForward` (optional, written only on Ok) are the two
// horizontal headings, for the log.
inline head_relative::Use Substitute(const Request& in, Hold& hold, float& sinA, float& cosA,
                                     float nativeForward[3] = nullptr, float usedForward[3] = nullptr)
{
    using head_relative::Use;
    if (!in.switchOn)
        return Use::SwitchOff;
    if (in.user != 0)
        return Use::NotLocalUser;
    if (!in.sampleRead)
        return Use::NoSample;
    const Use sampleUse = head_relative::SampleUse(in.sample, in.generation, in.nowUs);
    if (sampleUse != Use::Ok)
        return sampleUse;
    if (!in.gameplay)
        return Use::NotGameplay;
    float native[3];
    if (!PairForward(sinA, cosA, native))
        return Use::NoHeading;
    if (hold.generation != in.generation)
    {
        hold.heading = head_relative::HeadingHold{};
        hold.generation = in.generation;
    }
    float head[3], used[3];
    if (!head_relative::HeldHeading(hold.heading, in.sample.forward, kWorldUp, head) ||
        !head_relative::WithHeadYaw(native, head, kWorldUp, in.sign, in.offsetRad, used))
        return Use::NoHeading;
    float s = 0.0f, c = 0.0f;
    if (!ForwardPair(used, s, c))
        return Use::NoHeading;
    sinA = s;
    cosA = c;
    if (nativeForward)
    {
        nativeForward[0] = native[0];
        nativeForward[1] = native[1];
        nativeForward[2] = 0.0f;
    }
    if (usedForward)
    {
        usedForward[0] = used[0];
        usedForward[1] = used[1];
        usedForward[2] = 0.0f;
    }
    return Use::Ok;
}
} // namespace halo3_head_relative
