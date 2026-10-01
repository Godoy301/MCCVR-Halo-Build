#pragma once

// HALO 4 SEATED AIM -- the engine-free half of Halo 4's seated trace-and-reaim (stages 0 and 1).
//
// What the game does in a seat (static RE of the retail module, 2026-10-01): the fire routine 0x6176B8
// turns a seated shot onto the CROSSHAIR point. Its assist 0x1DCB8C traces 128 wu from the player's aim
// camera 0x2061A8 (the first-person eye 0x5F8A1C, or the slaved turret's marker for seat flag bit 7) along
// the unit's aim and turns the projectile from the BARREL toward what that ray hits -- by at most
// max(21 deg, the weapon's own angle) from the barrel's forward. It runs only while the aiming unit
// 0x5F1ABC(weapon owner) carries a player. So a seat converges exactly like Halo 3's: bend the aim so the
// game's eye line passes through the point the intent ray hits, and the native assist does the shot side --
// as long as the barrel can reach that point within its cap.
//
// This header holds the pure decisions, unit-tested in tests/halo4_seat_aim_logic_tests.cpp:
//   * the seat CLASS from the seat flags and the unit the seat controls (0x5F9694);
//   * the EYE AGREEMENT between the game's own aim-camera sample (simulation tick) and the eye the render
//     transaction hands the solve, and the exact first-person witness that must back it;
//   * the seated trace's IGNORE list and its LOOK-PAST of our own vehicle (three own-vehicle passes = a MISS);
//   * the BARREL CLAIM: converged only while the barrel reaches the point inside its cap (minus a margin);
//     otherwise the honest reticle sits on the line the capped shot really flies;
//   * the RECORD-FRAME GATE: until the record rebase 0x5F97DC is ported, a seat is converged only while its
//     record is in world angles (flag bit 4) or its vehicle is level.
//
// Zero engine dependencies.

#include <cmath>
#include <cstdint>

namespace halo4_seat_aim {

// ---- seat flags: the vehicle definition's seat block flags dword, read natively by 0x5FB2D4(vehicle, seat,
// bit). The names are the measured USES in the retail module, not H4EK names.
inline constexpr uint32_t kSeatFlagControlsParent = 1u << 0;  // 0x5F9694: the seat controls its parent
inline constexpr uint32_t kSeatFlagDriver = 1u << 2;          // 0x5FB178 and the dispatcher (0xA88F1)
inline constexpr uint32_t kSeatFlagGunner = 1u << 3;          // 0x5FB448 makes the occupant the controller
inline constexpr uint32_t kSeatFlagWorldAngles = 1u << 4;     // selector 0x10D268 picks its chase seat; 0x5F97DC
                                                              // skips the record rebase (world angles)
inline constexpr uint32_t kSeatFlagAllowsWeapons = 1u << 5;   // inferred, not measured
inline constexpr uint32_t kSeatFlagSlavedTurret = 1u << 7;    // 0x124DE6: the aim camera rides turret marker 0x165
// The bits the simulation sample reads (one native call each).
inline constexpr uint8_t kSeatFlagBitsRead[]{0, 2, 3, 4, 5, 7};

// What 0x5F9694 named as the unit this seat controls, relative to the occupant and its vehicle.
enum class Controlled : uint8_t { Unknown, Self, Parent, Child };

inline Controlled ClassifyControlled(int32_t controlled, int32_t unit, int32_t parent)
{
    if (controlled == -1 || unit == -1)
        return Controlled::Unknown;
    if (controlled == unit)
        return Controlled::Self;
    if (parent != -1 && controlled == parent)
        return Controlled::Parent;
    return Controlled::Child;   // a child of the vehicle: a turret that rides it
}

enum class SeatClass : uint8_t { Unknown, NoWeapon, PersonalWeapon, VehicleWeapon, SlavedTurret };

inline const char* SeatClassName(SeatClass c)
{
    switch (c)
    {
    case SeatClass::NoWeapon: return "no weapon";
    case SeatClass::PersonalWeapon: return "personal weapon";
    case SeatClass::VehicleWeapon: return "vehicle weapon";
    case SeatClass::SlavedTurret: return "slaved turret";
    default: return "unknown";
    }
}

// The seat's class. Anything not measured is Unknown, and an Unknown seat is never converged.
// A seat whose controlled unit holds no weapon fires nothing (a Warthog or Mongoose driver): NoWeapon.
inline SeatClass ClassifySeat(bool flagsKnown, uint32_t flags, Controlled controlled, bool weaponKnown,
                              bool hasWeapon)
{
    if (!flagsKnown || controlled == Controlled::Unknown || !weaponKnown)
        return SeatClass::Unknown;
    if (!hasWeapon)
        return SeatClass::NoWeapon;
    if (flags & kSeatFlagSlavedTurret)
        return SeatClass::SlavedTurret;
    return controlled == Controlled::Self ? SeatClass::PersonalWeapon : SeatClass::VehicleWeapon;
}

// ---- eye agreement ------------------------------------------------------------------------------------
// The solve's eye E is the render transaction's (the controller root plus the native eye's offset from the
// placed head, same frame as the rendered head H). The game's own aim camera is sampled on the simulation
// tick. They are compared only as a coarse rail -- the two clocks part them by the vehicle's speed times up
// to a tick -- while WHICH eye the game aims from is the exact first-person witness's job: a sample the
// selector proves was the chase camera unproves at once.

// Metres. The base absorbs interpolation; the speed term a tick and a half of travel (0.05 s).
inline constexpr float kEyeToleranceBaseM = 0.30f;
inline constexpr float kEyeToleranceWindowS = 0.05f;
inline constexpr float kEyeMaxSpeedMps = 40.0f;

inline float EyeToleranceM(float speedMps)
{
    float s = std::isfinite(speedMps) ? speedMps : kEyeMaxSpeedMps;
    s = s < 0.0f ? 0.0f : (s > kEyeMaxSpeedMps ? kEyeMaxSpeedMps : s);
    return kEyeToleranceBaseM + s * kEyeToleranceWindowS;
}

enum class EyeEvidence : uint8_t { None, Agree, Disagree, Chase };

// One simulation sample against the solve's eye. witness: 1 = the selector proved the first-person aim
// camera, 0 = it proved the chase camera, -1 = no proof (not called, called more than once, another unit,
// or the unit's camera override bit). sameSeat: the sample is this occupant, vehicle and seat. speedMps:
// the camera's speed, or NaN when the sample could not measure it (the first of a stint, or after a pause):
// then the gap is no evidence either way -- neither the widest rail (which would let a 2 m error agree) nor
// the rest rail (which would fail a fast vehicle). The witness still counts: a chase sample unproves.
inline EyeEvidence JudgeEye(int witness, bool sameSeat, float gapM, float speedMps)
{
    if (!sameSeat)
        return EyeEvidence::Disagree;
    if (witness == 0)
        return EyeEvidence::Chase;
    if (witness < 0)
        return EyeEvidence::Disagree;
    if (!std::isfinite(speedMps))
        return EyeEvidence::None;
    return std::isfinite(gapM) && gapM <= EyeToleranceM(speedMps) ? EyeEvidence::Agree : EyeEvidence::Disagree;
}

// Proven after `needed` consecutive agreeing samples; unproven after `drop_ms` of disagreement (or of no
// sample at all, which the caller reports as Disagree), and at once on a chase sample.
struct SeatEyeAgreement {
    unsigned needed = 10;
    unsigned long long drop_ms = 250;

    bool proven = false;
    unsigned count = 0;
    unsigned long long bad_since_ms = 0;

    void reset()
    {
        proven = false;
        count = 0;
        bad_since_ms = 0;
    }

    bool step(EyeEvidence e, unsigned long long now_ms)
    {
        switch (e)
        {
        case EyeEvidence::None:
            break;
        case EyeEvidence::Agree:
            bad_since_ms = 0;
            if (count < needed)
                ++count;
            if (count >= needed)
                proven = true;
            break;
        case EyeEvidence::Chase:
            count = 0;
            proven = false;
            bad_since_ms = now_ms ? now_ms : 1;
            break;
        case EyeEvidence::Disagree:
            count = 0;
            if (!bad_since_ms)
                bad_since_ms = now_ms ? now_ms : 1;
            if (proven && now_ms >= bad_since_ms && now_ms - bad_since_ms >= drop_ms)
                proven = false;
            break;
        }
        return proven;
    }
};

// ---- the seated trace -----------------------------------------------------------------------------------
// The intent ray starts at the rendered eye INSIDE the vehicle. Ignore its root first (the slot the game's
// own crosshair trace fills), then the occupant, the seat's parent when it is not the root, and the unit the
// seat controls (a turret riding the vehicle). Duplicates and -1 are skipped. Returns the count.
inline int BuildSeatIgnores(int32_t root, int32_t unit, int32_t parent, int32_t controlled, int32_t out[4])
{
    int n = 0;
    const int32_t candidates[4]{root, unit, parent, controlled};
    for (const int32_t c : candidates)
    {
        if (c == -1)
            continue;
        bool seen = false;
        for (int i = 0; i < n; ++i)
            seen = seen || out[i] == c;
        if (!seen)
            out[n++] = c;
    }
    return n;
}

// A hit is our own vehicle when its root is our root; when the hit's chain cannot be walked, when it is one
// of the chain's own handles.
inline bool IsOwnVehicleHit(int32_t object, int32_t hitRoot, int32_t root, const int32_t* chain, int chainCount)
{
    if (object == -1)
        return false;
    if (hitRoot != -1)
        return hitRoot == root;
    for (int i = 0; i < chainCount; ++i)
        if (chain[i] == object)
            return true;
    return false;
}

struct TracePass {
    bool completed = false;   // the native call returned (no fault)
    bool hit = false;
    float fraction = 0.0f;    // along this pass
    int32_t object = -1;      // the object hit, or -1 (the structure)
};

struct SeatTraceResult {
    bool completed = true;    // false: a pass faulted (the trace latches off)
    bool hit = false;         // false: a MISS (the far end), also after looking past our own vehicle every pass
    float rangeWu = 0.0f;     // from the ray's origin
    int passes = 0;
    int selfHits = 0;
    bool exhausted = false;   // every pass met our own vehicle: published as a MISS
};

// The look-past loop. trace(alongWu, remainingWu) runs one pass from origin + dir * along for `remaining`;
// rootOf(object) walks a hit object's parent chain (-1 when it cannot). A pass that meets our own vehicle
// continues `stepWu` past the contact, at most `maxPasses` passes; running out on our own vehicle is a MISS,
// as Halo 3's and Reach's seated traces publish it.
template <class Trace, class RootOf>
SeatTraceResult TraceLookingPast(float lengthWu, float skipWu, float stepWu, int maxPasses, int32_t root,
                                 const int32_t* chain, int chainCount, Trace&& trace, RootOf&& rootOf)
{
    SeatTraceResult r{};
    float alongWu = skipWu;
    for (int pass = 0; pass < maxPasses; ++pass)
    {
        const float remainingWu = lengthWu - (alongWu - skipWu);
        if (!(remainingWu > 0.0f))
            break;
        const TracePass t = trace(alongWu, remainingWu);
        ++r.passes;
        if (!t.completed)
        {
            r.completed = false;
            r.hit = false;
            return r;
        }
        const bool hit = t.hit && std::isfinite(t.fraction) && t.fraction >= 0.0f && t.fraction <= 1.0f;
        if (!hit)
            return r;   // a miss: the far end
        const float rangeWu = alongWu + t.fraction * remainingWu;
        if (!IsOwnVehicleHit(t.object, rootOf(t.object), root, chain, chainCount))
        {
            r.hit = true;
            r.rangeWu = rangeWu;
            return r;
        }
        ++r.selfHits;
        alongWu = rangeWu + stepWu;
    }
    r.exhausted = r.selfHits > 0;
    return r;
}

// ---- the barrel claim -----------------------------------------------------------------------------------
struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;
};

inline float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 Add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 Scale(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline bool Finite(Vec3 a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
inline bool Normalize(Vec3 a, Vec3& out)
{
    const float l = std::sqrt(Dot(a, a));
    if (!Finite(a) || !std::isfinite(l) || !(l > 1.0e-6f))
        return false;
    out = Scale(a, 1.0f / l);
    return true;
}

// The seated assist's minimum limit, 0.36651915 rad (module constant 0x...D954F4; the weapon's own angle can
// only widen it, so judging against this one is the conservative side).
inline constexpr float kSeatAssistCapRad = 0.36651915f;
// Converged is claimed only this far inside the cap.
inline constexpr float kSeatClaimMarginRad = 2.0f * 0.017453293f;

struct BarrelVerdict {
    bool valid = false;       // the geometry resolved
    bool claim = false;       // the shot reaches the point: the converged aim may be claimed
    float angleRad = 0.0f;    // barrel forward to (point - barrel origin)
    Vec3 shot{};              // the direction the shot really leaves the barrel along
};

// The shot leaves the barrel origin B. With the assist (`assisted`: the aiming unit carries the player) it
// turns from the barrel's forward f toward the point P, by at most `capRad`; without it, it flies along f.
// Claimed only when assisted and the turn needed is inside capRad - marginRad. At range (P far beyond B) the
// angle is the eye->point angle the design names; near, B's offset from the eye matters, so it is measured
// from B.
inline BarrelVerdict JudgeBarrel(Vec3 barrelForward, Vec3 barrelOrigin, Vec3 point, bool assisted,
                                 float capRad = kSeatAssistCapRad, float marginRad = kSeatClaimMarginRad)
{
    BarrelVerdict v{};
    Vec3 f{}, t{};
    if (!Normalize(barrelForward, f) || !Finite(barrelOrigin) || !Normalize(Sub(point, barrelOrigin), t))
        return v;
    float c = Dot(f, t);
    c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
    v.angleRad = std::acos(c);
    v.valid = true;
    if (!assisted)
    {
        v.shot = f;
        return v;
    }
    v.claim = v.angleRad < capRad - marginRad;
    if (v.angleRad <= capRad)
    {
        v.shot = t;
        return v;
    }
    // Rotate f toward t by capRad, in their common plane.
    Vec3 perp{};
    if (!Normalize(Sub(t, Scale(f, c)), perp))
    {
        v.shot = f;   // t opposite f: no plane; the barrel's own line is the honest answer
        return v;
    }
    v.shot = Add(Scale(f, std::cos(capRad)), Scale(perp, std::sin(capRad)));
    return v;
}

// Where the honest reticle points from the rendered eye H: at the point the real shot reaches at the range
// of the converged point, Q = B + shot * |P - B|.
inline bool ShotReticleDirection(Vec3 head, Vec3 barrelOrigin, Vec3 shot, Vec3 point, Vec3& out)
{
    const Vec3 toPoint = Sub(point, barrelOrigin);
    const float range = std::sqrt(Dot(toPoint, toPoint));
    Vec3 s{};
    if (!std::isfinite(range) || !Normalize(shot, s))
        return false;
    return Normalize(Sub(Add(barrelOrigin, Scale(s, range)), head), out);
}

// ---- the record-frame gate --------------------------------------------------------------------------------
// In a seat WITHOUT flag bit 4 the game's record is rotated by R = [normalize(Y x u), u x f, u] (0x5F97DC,
// u the vehicle's up): world angles only while u is +Z. Until that rebase is ported, a seat converges only
// with bit 4, or while its vehicle is level within 2 deg. (u.z, not |u.z|: an upturned vehicle's R is a half
// turn, not the identity.)
inline constexpr float kSeatLevelCos = 0.99939083f;   // cos(2 deg)

inline bool SeatRecordIsWorld(bool flagsKnown, uint32_t flags, bool upKnown, Vec3 up)
{
    if (!flagsKnown)
        return false;
    if (flags & kSeatFlagWorldAngles)
        return true;
    Vec3 u{};
    return upKnown && Normalize(up, u) && u.z > kSeatLevelCos;
}

} // namespace halo4_seat_aim
