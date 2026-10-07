// REACH: THE MOTION TRACKER FOLLOWS THE HEAD, the pure half (no game, engine or OpenXR includes).
//
// WHAT REACH DOES (HREK first, then matched in retail; docs/REACH-HEAD-RELATIVE-2026-10-06.md).
// HREK reach_tag_test.exe, chud_motion_sensor.cpp: the per-user tracker object's update (+0x954DE0) asks
// player_mapping for the output user's unit, copies the unit's position as the tracker's origin, and takes its
// heading from ONE place -- player_control.cpp's accessor of `&player_control_output->output.desired_angles`
// (+0x1E60D0, the assert names the field):
//     heading = desired_angles.yaw + pi/2          -> stored in the tracker object (+0x13D4)
// It then calls three functions in a row with the tracker as their only object; the first and the third read
// that stored heading and turn every contact's world offset by -heading (cosf / sinf), on foot and in a seat
// alike -- there is no vehicle branch. Nothing at draw time reads a heading again: a drawn contact is the stored
// point times a scale.
// Retail haloreach.dll has the same function at +0x2ED0C8 with the accessor inlined (engine TLS + 0xD8, the
// per-output-user stride 0xB0, the yaw at +0x694) and the same three calls (+0x2EC164, +0x2EC404, +0x2ECEB0).
//
// desired_angles is what this mod's direct drive writes the HAND's direction into, so the stock tracker turns
// with the gun. THE SEAM: a detour on the first of those three calls. It runs after the game stored the
// heading and before anything read it, and it rewrites that one float in the tracker's own object. The aim
// record, the camera evaluators, the rendered view and everything replicated are never touched or even read.
//
// ADDR-HYGIENE: resolved -- no address and no offset ships. The store site is matched uniquely in the whole
// module; the hook target is the call that follows it, and must be the uniquely matched head; the heading's
// offset is read from the store instruction and must be the displacement both readers use (ValueAgreement, two
// independent sites); the constant the game adds must be pi/2; the user index offset is read from the update's
// own head. Any link failing binds nothing: the game's own tracker, with the reason in the log.
#pragma once

#include "head_relative_logic.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace reach_head_relative
{
inline constexpr float kHalfPi = head_relative::kPi * 0.5f;
inline constexpr float kTwoPi = head_relative::kPi * 2.0f;
inline constexpr float kWorldUp[3]{0.0f, 0.0f, 1.0f};

// ---- the sites -------------------------------------------------------------------------------------------------
// The update's read-add-store and the three calls after it (retail +0x2ED187):
//   imul rcx, r9, 0xB0 / mov rax, [rdx + rbx] / movss xmm0, [rax + rcx + 0x694]   ; output desired_angles.yaw
//   mov rcx, rdi / addss xmm0, [rip + HALF PI] / movss [rdi + HEADING], xmm0
//   call FIRST / mov rcx, rdi / call SECOND / mov rdx, r15 / mov rcx, rdi / call THIRD
inline constexpr const char* kTrackerHeadingSitePattern =
    "49 69 C9 B0 00 00 00 48 8B 04 1A F3 0F 10 84 08 94 06 00 00 48 8B CF F3 0F 58 05 ?? ?? ?? ?? "
    "F3 0F 11 87 ?? ?? ?? ?? E8 ?? ?? ?? ?? 48 8B CF E8 ?? ?? ?? ?? 49 8B D7 48 8B CF E8";
inline constexpr size_t kSiteBytes = 0x3F;  // through the third call's rel32
inline constexpr size_t kSiteHalfPiDisp = 0x1B, kSiteHalfPiNext = 0x1F, kSiteHeadingDisp = 0x23,
                        kSiteCallFirst = 0x27, kSiteCallSecond = 0x2F, kSiteCallThird = 0x3A;
// The update's head (retail +0x2ED0C8): prologue / movsxd r9, [rcx + USER] / mov rdi, rcx / mov ecx, r9d /
// mov r15, r8 / call (the output user's player).
inline constexpr const char* kTrackerUpdateHeadPattern =
    "48 89 5C 24 08 48 89 6C 24 10 48 89 74 24 18 57 41 54 41 57 48 83 EC 20 4C 63 89 ?? ?? ?? ?? 48 8B F9 "
    "41 8B C9 4D 8B F8 E8";
inline constexpr size_t kUpdateHeadBytes = 0x29;
inline constexpr size_t kUpdateUserDisp = 0x1B;
// The first call's head (retail +0x2EC164, the hook): prologue / mov edx, [rip + TLS index] /
// lea rbx, [rcx + 0x1204] / movaps [rax - 0x38], xmm6 / mov rdi, rcx. One argument, the tracker, in rcx.
inline constexpr const char* kTrackerFirstReaderHeadPattern =
    "48 8B C4 48 89 58 08 48 89 70 10 48 89 78 18 55 41 54 41 55 41 56 41 57 48 8D 68 A1 48 81 EC C0 00 00 00 "
    "8B 15 ?? ?? ?? ?? 48 8D 99 04 12 00 00 0F 29 70 C8 48 8B F9";
inline constexpr size_t kFirstReaderHeadBytes = 0x37;
// Its read of the heading: movss xmm10, [rdi + HEADING] (F3 44 0F 10 97 disp32).
inline constexpr size_t kFirstReaderRead = 0xD6, kFirstReaderReadDisp = 0xDB, kFirstReaderBytes = 0xDF;
inline constexpr uint8_t kFirstReaderReadOpcode[5]{0xF3, 0x44, 0x0F, 0x10, 0x97};
// The third call's read: movss xmm7, [rcx + HEADING] (F3 0F 10 B9 disp32), before it has moved rcx.
inline constexpr size_t kThirdReaderRead = 0x35, kThirdReaderReadDisp = 0x39, kThirdReaderBytes = 0x3D;
inline constexpr uint8_t kThirdReaderReadOpcode[4]{0xF3, 0x0F, 0x10, 0xB9};

inline uint32_t ReadU32(const uint8_t* at)
{
    uint32_t value = 0;
    std::memcpy(&value, at, sizeof(value));
    return value;
}
inline int32_t ReadI32(const uint8_t* at)
{
    int32_t value = 0;
    std::memcpy(&value, at, sizeof(value));
    return value;
}
// Where the E8 at `site + call` lands, as an offset from the module base (`siteRva` is the site's own).
inline int64_t CallRva(const uint8_t* site, size_t call, int64_t siteRva)
{
    return siteRva + static_cast<int64_t>(call) + 5 + ReadI32(site + call + 1);
}
inline int64_t RipRva(const uint8_t* site, size_t disp, size_t next, int64_t siteRva)
{
    return siteRva + static_cast<int64_t>(next) + ReadI32(site + disp);
}

struct TrackerSites
{
    uint32_t headingOffset = 0;  // the tracker object's heading (retail and HREK 0x13D4)
    uint32_t userOffset = 0;     // its output user index (retail and HREK 0x13B8)
    int64_t firstReaderRva = 0;  // the hook: called right after the store, before any read
    int64_t thirdReaderRva = 0;  // the other reader of the heading
    int64_t halfPiRva = 0;       // the constant the game adds to the yaw
};
enum class TrackerBind : uint8_t
{
    Bound = 0,
    SiteMissing,        // the heading's read-add-store: no match, or more than one
    UpdateHeadMissing,  // the update's head: no match, or more than one
    SiteOutsideUpdate,  // the store is not inside the function whose head matched
    FirstHeadMissing,   // the first reader's head: no match, or more than one
    CallMismatch,       // the call after the store does not land on the matched head
    ReaderMismatch,     // a reader does not read the heading where it should
    FieldMismatch,      // the store and the readers disagree on the heading's offset
    Implausible,        // an offset out of range or misaligned, or the user index is not before the heading
    ConstantMismatch,   // what the game adds to the yaw is not pi/2
    Count
};
inline const char* TrackerBindName(TrackerBind why)
{
    switch (why)
    {
    case TrackerBind::Bound: return "bound";
    case TrackerBind::SiteMissing: return "the tracker's heading store is missing or ambiguous";
    case TrackerBind::UpdateHeadMissing: return "the tracker update's head is missing or ambiguous";
    case TrackerBind::SiteOutsideUpdate: return "the heading store is not inside the matched tracker update";
    case TrackerBind::FirstHeadMissing: return "the first reader's head is missing or ambiguous";
    case TrackerBind::CallMismatch: return "the call after the heading store does not land on the matched reader";
    case TrackerBind::ReaderMismatch: return "a reader does not read the heading where it should";
    case TrackerBind::FieldMismatch: return "the store and the readers disagree on the heading's offset";
    case TrackerBind::Implausible: return "the decoded offsets are implausible";
    case TrackerBind::ConstantMismatch: return "the game does not add pi/2 to the yaw";
    default: return "?";
    }
}
// `site`, `updateHead`, `firstHead`: the unique matches (null = missing or ambiguous) with their offsets from the
// module base. `updateEndRva`: the end of the function `site` lies in, from the image's own function table, and
// `siteFunctionRva` its beginning. Fills the two call targets and the constant's place; the caller then reads
// the third reader's bytes and the constant, and finishes with CheckTrackerReaders.
inline TrackerBind DecodeTrackerSites(const uint8_t* site, int64_t siteRva, const uint8_t* updateHead,
                                      int64_t updateHeadRva, int64_t siteFunctionRva, const uint8_t* firstHead,
                                      int64_t firstHeadRva, TrackerSites& out)
{
    if (!site)
        return TrackerBind::SiteMissing;
    if (!updateHead)
        return TrackerBind::UpdateHeadMissing;
    if (siteFunctionRva != updateHeadRva || siteRva <= updateHeadRva)
        return TrackerBind::SiteOutsideUpdate;
    if (!firstHead)
        return TrackerBind::FirstHeadMissing;
    if (CallRva(site, kSiteCallFirst, siteRva) != firstHeadRva)
        return TrackerBind::CallMismatch;
    const uint32_t heading = ReadU32(site + kSiteHeadingDisp), user = ReadU32(updateHead + kUpdateUserDisp);
    if (!heading || heading >= 0x100000 || (heading & 3) || !user || user >= 0x100000 || (user & 3) ||
        user + sizeof(int32_t) > heading)
        return TrackerBind::Implausible;
    out.headingOffset = heading;
    out.userOffset = user;
    out.firstReaderRva = firstHeadRva;
    out.thirdReaderRva = CallRva(site, kSiteCallThird, siteRva);
    out.halfPiRva = RipRva(site, kSiteHalfPiDisp, kSiteHalfPiNext, siteRva);
    if (out.thirdReaderRva <= 0 || out.thirdReaderRva == firstHeadRva || out.halfPiRva <= 0)
        return TrackerBind::Implausible;
    return TrackerBind::Bound;
}
// `first`: kFirstReaderBytes at the first reader's head; `third`: kThirdReaderBytes at the third's; `halfPi`:
// the four bytes at sites.halfPiRva.
inline TrackerBind CheckTrackerReaders(const uint8_t* first, const uint8_t* third, const uint8_t* halfPi,
                                       const TrackerSites& sites)
{
    if (!first || !third || !halfPi)
        return TrackerBind::ReaderMismatch;
    if (std::memcmp(first + kFirstReaderRead, kFirstReaderReadOpcode, sizeof(kFirstReaderReadOpcode)) != 0 ||
        std::memcmp(third + kThirdReaderRead, kThirdReaderReadOpcode, sizeof(kThirdReaderReadOpcode)) != 0)
        return TrackerBind::ReaderMismatch;
    if (ReadU32(first + kFirstReaderReadDisp) != sites.headingOffset ||
        ReadU32(third + kThirdReaderReadDisp) != sites.headingOffset)
        return TrackerBind::FieldMismatch;
    float value = 0.0f;
    std::memcpy(&value, halfPi, sizeof(value));
    if (!(std::fabs(value - kHalfPi) <= 1.0e-6f))
        return TrackerBind::ConstantMismatch;
    return TrackerBind::Bound;
}

// ---- the heading -----------------------------------------------------------------------------------------------
// The stored field is the yaw plus pi/2; a yaw is counter-clockwise from +X seen from above (+Z up), the angle
// the direct drive writes as atan2(direction.y, direction.x).
inline float FieldFromYaw(float yaw) { return yaw + kHalfPi; }
inline float YawFromField(float field) { return field - kHalfPi; }
inline void ForwardOfYaw(float yaw, float out[3])
{
    out[0] = std::cos(yaw);
    out[1] = std::sin(yaw);
    out[2] = 0.0f;
}
// A yaw in [0, 2 pi), the range the game's own accessor asserts for desired_angles.yaw.
inline float WrapYaw(float yaw)
{
    yaw = std::fmod(yaw, kTwoPi);
    if (yaw < 0.0f)
        yaw += kTwoPi;
    return yaw < kTwoPi ? yaw : 0.0f;
}

// WHERE THE GAME PUTS A CONTACT, from its own arithmetic (retail +0x2EC27B..+0x2EC35F, the scale left out):
//   a = -field;  x = sinf(a) * dy - cosf(a) * dx;  y = sinf(a) * dx + cosf(a) * dy
// With field = yaw + pi/2 that is x = offset . right, y = -(offset . forward): a contact straight ahead lands at
// (0, -r), which the tracker draws at its top; a contact to the right lands at (+r, 0).
inline void TrackerPoint(const float offset[3], float field, float out[2])
{
    const float a = -field;
    const float c = std::cos(a), s = std::sin(a);
    out[0] = s * offset[1] - c * offset[0];
    out[1] = s * offset[0] + c * offset[1];
}
// The angle of a tracker point from the tracker's top, positive toward its LEFT (the same sense as
// head_relative::Bearing: a contact to the left of the heading is positive).
inline float TrackerPointBearing(const float point[2]) { return std::atan2(-point[0], -point[1]); }

struct TrackerRequest
{
    bool switchOn = false;    // g_config.tracker_follows_head, read at this use
    int32_t outputUser = -1;  // the tracker object's own output user
    bool gameplay = false;    // the frame the head sample came from was not a cinematic's
    uint32_t generation = 0;  // the title generation now
    uint64_t nowUs = 0;       // the sight clock now
    int sign = 1;             // g_config.tracker_head_sign
    float offsetDeg = 0.0f;   // g_config.tracker_head_offset_deg
};
// The field the tracker should place its contacts by. Ok = `fieldOut` is the game's field with the head's yaw
// (in the game's own range, [pi/2, 5 pi/2)); anything else names why the game's own field stands, and
// `fieldOut` is untouched. `hold` keeps the last heading the head had, for a head looking straight up or down.
inline head_relative::Use TrackerField(const TrackerRequest& request, const head_relative::HeadSample& sample,
                                       head_relative::HeadingHold& hold, float nativeField, float& fieldOut,
                                       float* headYawOut = nullptr)
{
    using head_relative::Use;
    if (!request.switchOn)
        return Use::SwitchOff;
    if (request.outputUser != 0)
        return Use::NotLocalUser;
    const Use use = head_relative::SampleUse(sample, request.generation, request.nowUs);
    if (use != Use::Ok)
        return use;
    if (!request.gameplay)
        return Use::NotGameplay;
    if (!std::isfinite(nativeField))
        return Use::NoHeading;
    float headHeading[3], nativeForward[3];
    if (!head_relative::HeldHeading(hold, sample.forward, kWorldUp, headHeading))
        return Use::NoHeading;
    const float nativeYaw = YawFromField(nativeField);
    ForwardOfYaw(nativeYaw, nativeForward);
    float turn = 0.0f;
    if (!head_relative::YawToHead(nativeForward, headHeading, kWorldUp, request.sign,
                                  request.offsetDeg * head_relative::kDegToRad, turn))
        return Use::NoHeading;
    const float yaw = WrapYaw(nativeYaw + turn);
    if (!std::isfinite(yaw))
        return Use::NoHeading;
    fieldOut = FieldFromYaw(yaw);
    if (headYawOut)
        *headYawOut = std::atan2(headHeading[1], headHeading[0]);
    return Use::Ok;
}
} // namespace reach_head_relative
