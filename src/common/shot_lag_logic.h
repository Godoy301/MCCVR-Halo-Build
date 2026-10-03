#pragma once

// SHOT LAG -- aim at a WORLD POINT, recomposed against each shot's own origin (the two-clocks rule).
//
// The converged aim is solved on the input thread from an engine eye E sampled at RENDER time, bent through the
// point P the intent ray hits, and published as ANGLES. The simulation writes those angles a frame or more later,
// and the shot leaves from the FIRE tick's eye. Angles are a direction, not a line: the shot line is the intended
// line moved by everything the eye did in between (v * dt, measured in Reach at a mean 93 mm, p90 187 mm, max 307 mm
// over 236 shots, dt 25-40 ms). No angle can fix a parallel offset; a point can. So the aim path publishes its
// intent as the point P itself, and every consumer that knows its own origin O aims along normalize(P - O):
//
//   * the shot: the title's own query or firing call, at the fire tick, from the shot's own origin -- exact;
//   * the record (HUD, autoaim, melee, network): the stick call, from the best estimate of the eye at the tick
//     that consumes it -- never from a P rebuilt at the current eye, which would recreate the very error.
//
// P is built from ONE frame-coherent pair: the hand direction and the camera of the same prepared OpenXR frame
// (a hand from frame k with a camera from frame k-1 puts P off by the eye's motion in one frame).
//
// Pure functions and plain structs only: unit-tested in tests/shot_lag_tests.cpp. World units are Blam's
// (1 wu = 3.048 m); times are microseconds of one monotonic clock (QPC in the module).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace shot_lag
{
inline constexpr float kMetresPerWu = 3.048f;
inline constexpr float kMmPerWu = 3048.0f;
inline constexpr float kWuPerMetre = 1.0f / 3.048f;
inline constexpr float kRadToDeg = 57.295779513082320f;

inline float Dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline float Length(const float a[3]) { return std::sqrt(Dot(a, a)); }
inline bool Finite3(const float a[3])
{
    return a && std::isfinite(a[0]) && std::isfinite(a[1]) && std::isfinite(a[2]);
}

// ---- 1. re-aim through the point ---------------------------------------------------------------------------
// Why a re-aim was refused (the caller leaves its query or shot exactly as the game built it).
enum class Reaim : uint8_t
{
    Applied = 0,
    NonFinite,      // any input not finite
    Degenerate,     // a zero-length query vector
    PointBehind,    // P is not ahead of the start along the query: not the line this query traces
    PointTooClose,  // P is within minRangeWu of the start: it defines no direction
    BendTooLarge,   // P is off the line by more than any eye travel plus camera angle explains (ReaimRails)
    AlreadyThrough, // the query already passes within minOffsetWu of P: left exactly as the game built it
    Count
};

inline const char* ReaimName(Reaim why)
{
    switch (why)
    {
    case Reaim::Applied: return "applied";
    case Reaim::NonFinite: return "non-finite";
    case Reaim::Degenerate: return "degenerate";
    case Reaim::PointBehind: return "point behind";
    case Reaim::PointTooClose: return "point too close";
    case Reaim::BendTooLarge: return "bend too large";
    case Reaim::AlreadyThrough: return "already through";
    default: return "?";
    }
}

// The rails. They exist to catch OUR bugs -- a point from the wrong frame, a line that is not this query's -- and
// must never switch the correction off for a natural cause: speed, range, or the game's own camera. Whose query it
// is and how old P is are decided upstream, by gates that say what they refuse (QueryIsLocal: a start within 1.5 m
// of our own eye; FireMark: our fire; ShouldUseIntent: P under 150 ms old, this generation, converged). What is
// left for the line itself is an ENVELOPE of everything the game can legitimately put between its line and P:
//
//   offset of P from the line  <=  maxParallelWu  +  ahead * tan(maxAngleDeg)
//
//   * a PARALLEL part, the same at every range: the eye's travel between the solve and the shot plus the camera's
//     offset from our eye. Bounded by the gates: 30 m/s for the 150 ms a point may live is 4.5 m, plus the 1.5 m a
//     local start may sit from the eye = 6 m. Worst seen (Halo 3 Forge Hornet, 2026-10-03): 21.8 m/s over the 30 ms
//     the shot trailed the solve (0.65 m) beside a camera start ~1.1 m from the eye.
//   * an ANGULAR part, growing with range: the game's line may leave the record by an angle. The Hornet's crosshair
//     query does, by close to atan(v_sideways / ~200 m/s), as if the shooter's velocity were taken off the aim (a
//     fit, mechanism unproven: over 112 seated queries it cuts the query-vs-record angle from 3.1 deg mean to 0.4,
//     0.2 median) -- 5.65 deg at a 15-18 m/s strafe; a 30 m/s strafe would make 8.5. 30 deg is 5x the worst seen
//     and 3.5x that extrapolation.
//
// THE INCIDENT: the rail was `offset <= 3 m and bend <= 30 deg`. An absolute metres rail turns a constant ANGLE
// into a range-and-speed cliff: the Hornet's ~4 deg at 50 m is 3.5-4 m, so from a sideways ~8 m/s up, half or more
// of the queries were refused and the shots landed 2-6 m behind along travel ("accurate until a certain strafe
// speed, then suddenly a good bit off horizontally", user, 2026-10-03). The envelope has no cliff inside physics:
// at close range the parallel part admits the near-90 deg bends a few metres of travel make, at long range the
// angle does. A point beyond it is refused (BendTooLarge); behind the start (PointBehind) bends 90 deg or more.
// Nothing is clamped. A query that already passes within minOffsetWu of P is LEFT ALONE (AlreadyThrough): where the
// game's line is already coherent -- Halo 3 campaign vehicles, user-confirmed 2026-10-02 -- the re-aim must not move
// it by the noise in P.
struct ReaimRails
{
    float minOffsetWu = 0.010f * kWuPerMetre;
    float minRangeWu = 0.3f * kWuPerMetre;
    float maxParallelWu = 6.0f * kWuPerMetre;
    float maxAngleDeg = 30.0f;
};

// out = normalize(P - start) * |vector|: the same query, from its own start, through the point. `out` is written
// only on Applied. `bendDeg`, when given, receives the angle between the old and the new line on Applied.
inline Reaim ReaimThroughPoint(const float start[3], const float vector[3], const float point[3],
                               const ReaimRails& rails, float out[3], float* bendDeg = nullptr)
{
    if (!Finite3(start) || !Finite3(vector) || !Finite3(point) || !out)
        return Reaim::NonFinite;
    const float length = Length(vector);
    if (!std::isfinite(length) || !(length > 1.0e-6f))
        return Reaim::Degenerate;
    const float toPoint[3]{point[0] - start[0], point[1] - start[1], point[2] - start[2]};
    const float distance = Length(toPoint);
    if (!std::isfinite(distance))
        return Reaim::NonFinite;
    const float d[3]{vector[0] / length, vector[1] / length, vector[2] / length};
    const float ahead = Dot(toPoint, d);
    if (!(ahead > 0.0f))
        return Reaim::PointBehind;
    if (!(distance > rails.minRangeWu))
        return Reaim::PointTooClose;
    // How far P sits off the old line, and the angle between the two lines -- from the perpendicular VECTOR, never
    // as distance^2 - ahead^2: that difference of two large squares cancels catastrophically in float (at 1 km its
    // rounding alone is ~0.5 m of "offset", so a query already through P read as off it), and acos near 1 is as bad.
    // toPoint - d * ahead keeps an absolute error of a few ulps of the range (well under a millimetre at 1 km).
    const float perpendicular[3]{toPoint[0] - d[0] * ahead, toPoint[1] - d[1] * ahead, toPoint[2] - d[2] * ahead};
    const float offset = Length(perpendicular);
    const float bend = std::atan2(offset, ahead) * kRadToDeg;
    if (offset < rails.minOffsetWu)
        return Reaim::AlreadyThrough;
    const float allowed = rails.maxParallelWu + ahead * std::tan(rails.maxAngleDeg / kRadToDeg);
    if (!(offset <= allowed))
        return Reaim::BendTooLarge;
    const float scale = length / distance;
    out[0] = toPoint[0] * scale;
    out[1] = toPoint[1] * scale;
    out[2] = toPoint[2] * scale;
    if (bendDeg)
        *bendDeg = bend;
    return Reaim::Applied;
}

// ---- 2. the published intent ----------------------------------------------------------------------------------
// What the aim path publishes: P and how it was built. Every field the shot site needs to decide to use it.
struct IntentPoint
{
    uint32_t generation = 0;
    uint64_t stampUs = 0;      // when the solve built P
    uint64_t serial = 0;       // the prepared OpenXR frame of the coherent pair (0 = none)
    bool coherent = false;     // hand and camera from the same prepared frame
    bool railHeld = false;     // the solve held the aim at the pitch rail: the record does not pass through P
    bool seated = false;
    float point[3]{};          // P = head + dir * rangeWu
    float dir[3]{};            // u, unit
    float head[3]{};           // H, the rendered eye of the pair
    float eye[3]{};            // E, the engine eye the solve used
    uint64_t eyeStampUs = 0;   // when E was sampled (the render that published it)
    float eyeVel[3]{};         // the engine eye's velocity, wu/s, over ~100 ms before eyeStampUs
    bool eyeVelValid = false;
    float rangeWu = 0.0f;
    float solvedYaw = 0.0f, solvedPitch = 0.0f;   // the angles the solve published from it (game basis)
};

// The shot site's verdict on a published point. `maxAgeUs`: older is stale (the player no longer means it).
enum class IntentUse : uint8_t
{
    Use = 0,
    Missing,       // nothing published, or for another title generation
    Stale,         // older than maxAgeUs, or from the future
    RailHeld,      // the solve did not pass through it (held at the pitch rail)
    NaturalHeld,   // a natural cause holds the game's aim off it (a seat stop, a turret's slew)
    NotConverged,  // the converged aim is not in effect
    Bad,           // a non-finite or degenerate point
    OwnShotLine,   // the shot is another gun's own line (an off-hand gun, a barrel shot): never the aim hand's P
    Count
};

inline const char* IntentUseName(IntentUse why)
{
    switch (why)
    {
    case IntentUse::Use: return "used";
    case IntentUse::Missing: return "missing";
    case IntentUse::Stale: return "stale";
    case IntentUse::RailHeld: return "rail-held";
    case IntentUse::NaturalHeld: return "natural-held";
    case IntentUse::NotConverged: return "not converged";
    case IntentUse::Bad: return "bad";
    case IntentUse::OwnShotLine: return "own shot line";
    default: return "?";
    }
}

// THE CROSSHAIR QUERY'S GATE: before any intent is consulted, may the re-aim touch this query at all? Only the local
// player's query at a resolved crosshair site, and only while no other shot line owns the fire. Independent dual aim
// (the off-hand gun) and barrel / muzzle shots run the fire with that GUN's own camera ray -- the firing assist's
// camera is swapped for it (Halo 3 0x13BC0D, ODST 0x161204, Reach 0x10F954) -- so its crosshair query traces that
// gun's line, and re-aiming it through the AIM HAND's point would steer the off-hand or barrel shot onto the other
// hand's target.
//
// THE LOCAL PLAYER'S FIRE. A query's ignores and start cannot tell two players in one vehicle apart (Reach names only
// the vehicle's root; Halo 3's 1.5 m start guard admits an adjacent seat), and in networked co-op another player's
// assist runs on this machine too (the assists take a player index: Halo 3 0x13BAD0, Reach's aim_projectile). So the
// fire itself is tagged on the fire thread: a native fire's entry opens it as kFireOther, and its shot line -- the one
// call that names the shooting unit (Halo 3 0x3524B0 from 0x368B92: the OCCUPANT after the gunner chain; ODST
// 0x396B7C from 0x3AEB26; Reach's unit-adjust from 0x4C303A) -- marks it kFireLocal when that unit is the local
// player's. The fire's exit restores the enclosing value. Only a kFireLocal fire's queries are ever re-aimed.
inline constexpr int8_t kFireNone = -1;    // not inside a native fire on this thread
inline constexpr int8_t kFireOther = 0;    // inside a fire whose shooter is not (yet) the local player
inline constexpr int8_t kFireLocal = 1;    // inside the local player's fire
inline int8_t FireMark(int8_t current, bool shooterIsLocal)
{
    return current == kFireNone ? kFireNone : (shooterIsLocal ? kFireLocal : kFireOther);
}

enum class QueryGate : uint8_t
{
    Try = 0,       // consult the intent (ShouldUseIntent)
    NotOurs,       // not the site, not the local player's fire, or not his query: left alone and not counted
    OwnShotLine,   // the local fire's line belongs to another gun: left alone, counted (IntentUse::OwnShotLine)
};
inline QueryGate GateCrosshairQuery(bool atSite, int8_t fire, bool queryLocal, bool ownShotLine)
{
    if (!atSite || fire != kFireLocal || !queryLocal)
        return QueryGate::NotOurs;
    if (ownShotLine)
        return QueryGate::OwnShotLine;
    return QueryGate::Try;
}

inline IntentUse ShouldUseIntent(bool published, const IntentPoint& p, uint32_t generation, uint64_t nowUs,
                                 uint64_t maxAgeUs, bool convergedLive, bool naturalHeld)
{
    if (!published || !generation || p.generation != generation || !p.stampUs)
        return IntentUse::Missing;
    if (nowUs < p.stampUs || nowUs - p.stampUs > maxAgeUs)
        return IntentUse::Stale;
    if (!convergedLive)
        return IntentUse::NotConverged;
    if (naturalHeld)
        return IntentUse::NaturalHeld;
    if (p.railHeld)
        return IntentUse::RailHeld;
    if (!Finite3(p.point) || !Finite3(p.dir) || !(std::isfinite(p.rangeWu) && p.rangeWu > 0.0f))
        return IntentUse::Bad;
    return IntentUse::Use;
}

// ---- 3. the eye's velocity ------------------------------------------------------------------------------------
// The engine eye's track on ONE thread (the publishing thread): samples with their own clock, the velocity over
// the newest ~windowUs. An eye only moves when the game renders a new frame, so a sample identical to the last is
// news only once it has stood still for kStillUs (a player who stops must read 0, not his last pace). A gap, a
// clock running backwards, or a jump faster than any vehicle (a teleport, a respawn, a seat change) starts afresh.
struct EyeTrack
{
    static constexpr int kSamples = 16;
    static constexpr uint64_t kGapUs = 250000;          // no sample for this long: start afresh
    static constexpr uint64_t kStillUs = 40000;         // an unchanged eye counts as a sample after this long
    static constexpr float kTeleportWuPerS = 150.0f * kWuPerMetre;
    uint64_t t[kSamples]{};
    float p[kSamples][3]{};
    int count = 0;
    int head = 0;   // next write

    void Reset() { count = 0; head = 0; }
    const float* Newest(uint64_t* at = nullptr) const
    {
        if (!count)
            return nullptr;
        const int i = (head + kSamples - 1) % kSamples;
        if (at)
            *at = t[i];
        return p[i];
    }
    void Push(uint64_t tUs, const float eye[3])
    {
        if (!Finite3(eye) || !tUs)
            return;
        uint64_t lastAt = 0;
        if (const float* last = Newest(&lastAt))
        {
            if (tUs <= lastAt)
            {
                if (tUs < lastAt)
                    Reset();   // the clock ran backwards: nothing before is comparable
                else
                    return;    // the same instant twice
            }
            else if (tUs - lastAt > kGapUs)
                Reset();
            else
            {
                const float d[3]{eye[0] - last[0], eye[1] - last[1], eye[2] - last[2]};
                const float moved = Length(d);
                const float seconds = float(tUs - lastAt) * 1.0e-6f;
                if (moved == 0.0f && tUs - lastAt < kStillUs)
                    return;    // a repeat of the last render's eye: no news yet
                if (!(moved <= kTeleportWuPerS * seconds))
                    Reset();
            }
        }
        t[head] = tUs;
        p[head][0] = eye[0];
        p[head][1] = eye[1];
        p[head][2] = eye[2];
        head = (head + 1) % kSamples;
        if (count < kSamples)
            ++count;
    }
    // Velocity (wu/s) from the newest sample back to the oldest one within windowUs of it, needing at least
    // minSpanUs between them. False: not enough track (the caller treats the speed as unknown, never as 0).
    bool Velocity(uint64_t windowUs, uint64_t minSpanUs, float out[3]) const
    {
        uint64_t newestAt = 0;
        const float* newest = Newest(&newestAt);
        if (!newest || count < 2)
            return false;
        int oldest = -1;
        for (int k = 1; k < count; ++k)
        {
            const int i = (head + kSamples - 1 - k) % kSamples;
            if (newestAt - t[i] > windowUs)
                break;
            oldest = i;
        }
        if (oldest < 0 || newestAt - t[oldest] < minSpanUs)
            return false;
        const float seconds = float(newestAt - t[oldest]) * 1.0e-6f;
        for (int axis = 0; axis < 3; ++axis)
            out[axis] = (newest[axis] - p[oldest][axis]) / seconds;
        return Finite3(out);
    }
};

// ---- 3b. the game-eye proof -----------------------------------------------------------------------------------
// A ValueAgreement over a claim: "this evaluator's answer IS the engine eye". Compared against the engine camera's own
// position at rest (moving, a rendered camera may lead or trail the simulation). The comparison means something only
// where the two are the SAME point by construction -- on foot, in first person -- so a sample from a seat, a third-
// person or scripted camera, a carried turret or a death camera is no evidence either way (`eligible` false). One
// disagreeing sample proves nothing (a respawn, a teleport, a frame of transition), so disproof takes a STREAK of
// kDisagreeToDisprove consecutive disagreeing at-rest samples, as proof takes kAgreeToProve agreeing ones. A
// disproof holds for the unit's on-foot stint; a new unit (respawn) or a new stint re-opens the question. A new
// title generation starts over.
struct EyeProof
{
    enum class State : uint8_t { Unproven = 0, Proven, Disproven };
    static constexpr uint32_t kAgreeToProve = 30;
    static constexpr uint32_t kDisagreeToDisprove = 15;
    static constexpr float kAgreeMm = 50.0f;
    static constexpr float kDisagreeMm = 250.0f;

    State state = State::Unproven;
    uint32_t agree = 0, disagree = 0;
    uint32_t generation = 0;
    int32_t unit = -1;
    bool inStint = false;

    // One sample from the publisher. `offsetMm` is read only when eligible and at rest.
    void Note(uint32_t gen, int32_t unitNow, bool eligible, bool atRest, float offsetMm)
    {
        if (gen != generation)
        {
            *this = EyeProof{};
            generation = gen;
            unit = unitNow;
        }
        else if (unitNow != unit)
        {
            unit = unitNow;
            inStint = false;   // a new unit's stint starts below
        }
        if (!eligible)
        {
            inStint = false;
            return;
        }
        if (!inStint)
        {
            inStint = true;
            agree = disagree = 0;
            if (state == State::Disproven)
                state = State::Unproven;
        }
        if (!atRest || !std::isfinite(offsetMm))
            return;
        if (offsetMm < kAgreeMm)
        {
            disagree = 0;
            if (++agree >= kAgreeToProve && state != State::Disproven)
                state = State::Proven;
        }
        else if (offsetMm > kDisagreeMm)
        {
            agree = 0;
            if (++disagree >= kDisagreeToDisprove)
                state = State::Disproven;
        }
        else
            agree = disagree = 0;   // neither: no streak survives it
    }
};

// ---- 4. the eye now, and the angles toward the point --------------------------------------------------------
// The interim estimate of an eye at a later tick: E + v * dt, dt clamped to [0, maxDtS]. False (out untouched)
// without a finite eye and velocity.
inline bool ExtrapolateEye(const float eye[3], const float velocity[3], float dtS, float maxDtS, float out[3])
{
    if (!Finite3(eye) || !Finite3(velocity) || !std::isfinite(dtS) || !out)
        return false;
    const float dt = dtS < 0.0f ? 0.0f : (dtS > maxDtS ? maxDtS : dtS);
    for (int axis = 0; axis < 3; ++axis)
        out[axis] = eye[axis] + velocity[axis] * dt;
    return true;
}

// The crosshair query's own evidence that it is the local player's (with GateCrosshairQuery's fire tag). On foot its
// first ignore is the local unit itself. Seated it is the vehicle's root (the published seat chain's top: `seatC`,
// else `seatB`) or the seat's parent (`seatB`), which another occupant's query names too, so it must also START
// within maxStartWu of the eye the intent was built from, carried forward by that eye's velocity to now (at most
// 100 ms): an AI gunner's camera sits elsewhere. Not enough alone for a co-op player in the next seat -- hence the
// fire tag.
inline bool QueryIsLocal(bool seated, int32_t ignoredA, int32_t localUnit, int32_t seatB, int32_t seatC,
                         const IntentPoint& intent, const float* start, uint64_t nowUs, float maxStartWu)
{
    if (ignoredA == -1)
        return false;
    if (!seated)
        return ignoredA == localUnit;
    const int32_t root = seatC != -1 ? seatC : seatB;
    if (ignoredA != root && ignoredA != seatB)
        return false;
    if (!start || !Finite3(start) || !Finite3(intent.eye))
        return false;
    float eye[3]{intent.eye[0], intent.eye[1], intent.eye[2]};
    if (intent.eyeVelValid && intent.eyeStampUs && nowUs > intent.eyeStampUs)
        (void)ExtrapolateEye(intent.eye, intent.eyeVel, float(nowUs - intent.eyeStampUs) * 1.0e-6f, 0.1f, eye);
    const float d[3]{start[0] - eye[0], start[1] - eye[1], start[2] - eye[2]};
    const float distance = Length(d);
    return std::isfinite(distance) && distance <= maxStartWu;
}

// The game-basis angles of normalize(P - origin) (yaw about +Z from +X, pitch up), pitch clamped to
// +-maxPitchRad. Yaw is atan2's (-pi, pi]: the caller wraps it as it wraps every published angle. False when P is
// within minRangeWu of the origin or anything is non-finite.
inline bool AnglesTowardPoint(const float origin[3], const float point[3], float minRangeWu, float maxPitchRad,
                              float& yaw, float& pitch)
{
    if (!Finite3(origin) || !Finite3(point))
        return false;
    const float d[3]{point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
    const float length = Length(d);
    if (!std::isfinite(length) || !(length > minRangeWu))
        return false;
    float s = d[2] / length;
    s = s > 1.0f ? 1.0f : (s < -1.0f ? -1.0f : s);
    float p = std::asin(s);
    p = p > maxPitchRad ? maxPitchRad : (p < -maxPitchRad ? -maxPitchRad : p);
    const float y = std::atan2(d[1], d[0]);
    if (!std::isfinite(y) || !std::isfinite(p))
        return false;
    yaw = y;
    pitch = p;
    return true;
}

// The identity the record recomposition checks: the setpoint it is about to write IS the solve that built P
// (the same angles, wrap-insensitive), so a setpoint from any other path (the fixed-distance steer, a seat author)
// is never replaced.
inline bool SameAngles(float yawA, float pitchA, float yawB, float pitchB, float toleranceRad = 1.0e-4f)
{
    if (!std::isfinite(yawA) || !std::isfinite(pitchA) || !std::isfinite(yawB) || !std::isfinite(pitchB))
        return false;
    float dy = std::fmod(yawA - yawB, 6.283185307179586f);
    if (dy > 3.141592653589793f) dy -= 6.283185307179586f;
    if (dy < -3.141592653589793f) dy += 6.283185307179586f;
    return std::fabs(dy) <= toleranceRad && std::fabs(pitchA - pitchB) <= toleranceRad;
}

// ---- 4b. the record recomposed through the point (Halo 3 on foot) ---------------------------------------------
// The stick call's setpoint -- the record the HUD, autoaim, melee and the network read -- recomposed as the angles of
// normalize(P - E'): P the published intent point, E' the game's own eye at the stick call carried one simulation
// tick ahead (the record written now is first consumed by the next tick). Never P rebuilt from the current eye. Kept
// as published (fail closed) for every reason below, in this order; `yaw`/`pitch`/`bendDeg` are written only on
// Recomposed.
enum class Recompose : uint8_t
{
    Recomposed = 0,
    NotOnFoot,      // not Halo 3's on-foot stick call, its publication invalid or seated, or the converged aim off
    NoIntent,       // nothing published
    OtherSolve,     // the setpoint is not the solve that built P (the fixed-distance steer, a seat author)
    PointUnusable,  // another generation, stale (or from the future), seated, held at the pitch rail, or non-finite
    NoGameEye,      // the game's eye at the tick is unknown or stale
    EyeUnproven,    // that eye is not (yet) proven to be the engine eye (EyeProof)
    Geometry,       // P within 30 cm of the eye, or the result bends over 30 deg from the setpoint
    Count
};
struct RecomposeInput
{
    bool onFoot = false;
    bool intentRead = false;
    IntentPoint intent{};
    float setYaw = 0.0f, setPitch = 0.0f;   // the setpoint about to be written
    uint32_t generation = 0;                // the title's current generation (0 = none)
    uint64_t nowUs = 0, maxAgeUs = 0;
    bool eyeRead = false;                   // the game's eye at the tick, fresh
    float eye[3]{}, eyeVel[3]{};
    bool eyeVelValid = false;
    bool eyeProven = false;
    float tickS = 0.0f;                     // the measured simulation tick; the carry is clamped to 50 ms
};
inline Recompose RecomposeRecord(const RecomposeInput& in, float& yaw, float& pitch, float& bendDeg)
{
    if (!in.onFoot)
        return Recompose::NotOnFoot;
    if (!in.intentRead)
        return Recompose::NoIntent;
    const IntentPoint& p = in.intent;
    if (!SameAngles(in.setYaw, in.setPitch, p.solvedYaw, p.solvedPitch))
        return Recompose::OtherSolve;
    if (!in.generation || p.generation != in.generation || p.seated || p.railHeld || !p.stampUs ||
        in.nowUs < p.stampUs || in.nowUs - p.stampUs > in.maxAgeUs || !Finite3(p.point))
        return Recompose::PointUnusable;
    if (!in.eyeRead || !Finite3(in.eye))
        return Recompose::NoGameEye;
    if (!in.eyeProven)
        return Recompose::EyeUnproven;
    float fire[3]{in.eye[0], in.eye[1], in.eye[2]};
    if (in.eyeVelValid)
        (void)ExtrapolateEye(in.eye, in.eyeVel, in.tickS, 0.05f, fire);
    float newYaw = 0.0f, newPitch = 0.0f;
    if (!AnglesTowardPoint(fire, p.point, 0.3f * kWuPerMetre, 1.45f, newYaw, newPitch))
        return Recompose::Geometry;
    const float a[3]{std::cos(in.setPitch) * std::cos(in.setYaw), std::cos(in.setPitch) * std::sin(in.setYaw),
                     std::sin(in.setPitch)};
    const float b[3]{std::cos(newPitch) * std::cos(newYaw), std::cos(newPitch) * std::sin(newYaw), std::sin(newPitch)};
    float c = Dot(a, b);
    c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
    const float bend = std::acos(c) * kRadToDeg;
    if (!(bend <= 30.0f))
        return Recompose::Geometry;
    yaw = newYaw;
    pitch = newPitch;
    bendDeg = bend;
    return Recompose::Recomposed;
}

// ---- 5. the frame-coherent pair --------------------------------------------------------------------------------
// A render publishes the eyes it rendered from, keyed by the tracking-space head position its head look used; the
// aim path keeps the hand directions it computed, keyed by the head position of the sample the hand came from.
// The same head sample on both sides is the same prepared frame. The pair is the NEWEST hand whose key matches an
// eye exactly (the newest such eye when several share it: a frozen head).
struct EyeFrame
{
    bool valid = false;
    float key[3]{};
    float head[3]{};   // H: the rendered eye
    float eye[3]{};    // E: the engine eye under it
    uint64_t stampUs = 0;
    uint64_t serial = 0;
};
struct HandFrame
{
    bool valid = false;
    float key[3]{};
    float dir[3]{};    // u, world
    uint64_t stampUs = 0;
    uint64_t serial = 0;
};

inline bool SameKey(const float a[3], const float b[3])
{
    return Finite3(a) && Finite3(b) && a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

// Indices into the two arrays, or false. `maxSkewUs`: a hand newer than its eye by more than this is refused (a
// key that repeats after a long freeze is not the same frame). `maxHandAgeUs` (with `nowUs`; 0 = unbounded): a hand
// older than this at `nowUs`, or stamped after it, is refused -- during a render stall the newest hand whose frame
// rendered can be several polls old, and a pair built from it is the player's aim of that moment, not of now. The
// caller stamps the published point with the chosen HAND's stamp, so its consumers age it from when it was meant.
inline bool SelectCoherentPair(const EyeFrame* eyes, size_t eyeCount, const HandFrame* hands, size_t handCount,
                               uint64_t maxSkewUs, size_t& eyeIndex, size_t& handIndex, uint64_t nowUs = 0,
                               uint64_t maxHandAgeUs = 0)
{
    bool found = false;
    uint64_t bestHand = 0, bestEye = 0;
    for (size_t h = 0; h < handCount; ++h)
    {
        if (!hands[h].valid || !hands[h].stampUs || (found && hands[h].stampUs < bestHand))
            continue;
        if (maxHandAgeUs && (hands[h].stampUs > nowUs || nowUs - hands[h].stampUs > maxHandAgeUs))
            continue;
        for (size_t e = 0; e < eyeCount; ++e)
        {
            if (!eyes[e].valid || !SameKey(eyes[e].key, hands[h].key))
                continue;
            const uint64_t a = eyes[e].stampUs, b = hands[h].stampUs;
            const uint64_t skew = a > b ? a - b : b - a;
            if (skew > maxSkewUs)
                continue;
            if (!found || hands[h].stampUs > bestHand || (hands[h].stampUs == bestHand && a > bestEye))
            {
                found = true;
                bestHand = hands[h].stampUs;
                bestEye = a;
                eyeIndex = e;
                handIndex = h;
            }
        }
    }
    return found;
}

// ---- 6. the witness: where did the shot go, relative to P, and why ------------------------------------------
// One shot: its line (origin O, direction d) against P. The miss m = (closest point of the line to P) - P, split
// along the player's travel (v with its component along the shot removed: the axis a parallel shift moves the
// line along) and across it. The stale-eye term: the shot's origin minus the eye the solve used, perpendicular to
// the shot and signed along that same travel -- what a solve at the wrong eye costs -- and the dt it implies.
struct ShotLagSample
{
    bool valid = false;
    float missMm = 0.0f;
    float alongMm = 0.0f, acrossMm = 0.0f;   // along: signed (+ = ahead of where the player is going)
    float rangeM = 0.0f;                     // |P - O|
    float speedMps = 0.0f;
    bool travel = false;                     // the velocity had a usable component across the shot
    bool velocityKnown = false;
    float staleAlongMm = 0.0f;               // valid with `stale`
    float impliedDtMs = 0.0f;                // valid with `stale && travel`
    bool stale = false;
    int bucket = -1;                         // ShotLagBucket(speedMps), -1 without a velocity
};

inline constexpr int kShotLagBuckets = 4;
inline const char* ShotLagBucketName(int bucket)
{
    switch (bucket)
    {
    case 0: return "0-1 m/s";
    case 1: return "1-4";
    case 2: return "4-10";
    case 3: return "10+";
    default: return "?";
    }
}
inline int ShotLagBucket(float speedMps)
{
    if (!std::isfinite(speedMps) || speedMps < 0.0f)
        return -1;
    return speedMps < 1.0f ? 0 : speedMps < 4.0f ? 1 : speedMps < 10.0f ? 2 : 3;
}

// `velocity` (wu/s) may be null (unknown); `solveEye` may be null (no stale-eye term).
inline ShotLagSample MeasureShotLag(const float origin[3], const float direction[3], const float point[3],
                                    const float* solveEye, const float* velocity)
{
    ShotLagSample s{};
    if (!Finite3(origin) || !Finite3(direction) || !Finite3(point))
        return s;
    const float dl = Length(direction);
    if (!std::isfinite(dl) || !(dl > 1.0e-6f))
        return s;
    const float d[3]{direction[0] / dl, direction[1] / dl, direction[2] / dl};
    const float w[3]{point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
    const float t = Dot(w, d);
    const float m[3]{origin[0] + d[0] * t - point[0], origin[1] + d[1] * t - point[1],
                     origin[2] + d[2] * t - point[2]};
    s.missMm = Length(m) * kMmPerWu;
    s.rangeM = Length(w) * kMetresPerWu;
    s.acrossMm = s.missMm;
    float a[3]{};
    if (velocity && Finite3(velocity))
    {
        s.velocityKnown = true;
        s.speedMps = Length(velocity) * kMetresPerWu;
        s.bucket = ShotLagBucket(s.speedMps);
        const float along = Dot(velocity, d);
        const float vp[3]{velocity[0] - d[0] * along, velocity[1] - d[1] * along, velocity[2] - d[2] * along};
        const float vpl = Length(vp);
        // Below 5 cm/s across the shot there is no travel axis to sign against.
        if (std::isfinite(vpl) && vpl * kMetresPerWu > 0.05f)
        {
            s.travel = true;
            a[0] = vp[0] / vpl; a[1] = vp[1] / vpl; a[2] = vp[2] / vpl;
            const float ma = Dot(m, a);
            const float r[3]{m[0] - a[0] * ma, m[1] - a[1] * ma, m[2] - a[2] * ma};
            s.alongMm = ma * kMmPerWu;
            s.acrossMm = Length(r) * kMmPerWu;
            if (solveEye && Finite3(solveEye))
            {
                const float se[3]{origin[0] - solveEye[0], origin[1] - solveEye[1], origin[2] - solveEye[2]};
                s.stale = true;
                s.staleAlongMm = Dot(se, a) * kMmPerWu;   // the along-shot part is orthogonal to a already
                s.impliedDtMs = Dot(se, a) / vpl * 1000.0f;
            }
        }
    }
    s.valid = std::isfinite(s.missMm) && std::isfinite(s.rangeM) && std::isfinite(s.alongMm) &&
              std::isfinite(s.acrossMm);
    return s;
}

// A report window (one per title): plain sums, owned by whoever holds the title's witness lock.
struct ShotLagWindow
{
    uint32_t shots = 0, unmeasured = 0;
    uint32_t n[kShotLagBuckets]{};
    float missSum[kShotLagBuckets]{}, alongSum[kShotLagBuckets]{}, acrossSum[kShotLagBuckets]{};
    uint32_t noVelocity = 0;
    float noVelocityMissSum = 0.0f;
    uint32_t travelShots = 0;
    float alongTravelSum = 0.0f;
    uint32_t staleShots = 0;
    float staleAlongSum = 0.0f;
    uint32_t dtShots = 0;
    float impliedDtSum = 0.0f;
    float rangeSum = 0.0f;
    bool seated = false;
    int32_t seat = -1;

    void Add(const ShotLagSample& s, bool seatedNow, int32_t seatId)
    {
        if (!s.valid)
        {
            ++unmeasured;
            return;
        }
        ++shots;
        rangeSum += s.rangeM;
        seated = seatedNow;
        seat = seatId;
        if (s.bucket >= 0 && s.bucket < kShotLagBuckets)
        {
            ++n[s.bucket];
            missSum[s.bucket] += s.missMm;
            alongSum[s.bucket] += s.alongMm;
            acrossSum[s.bucket] += s.acrossMm;
        }
        else
        {
            ++noVelocity;
            noVelocityMissSum += s.missMm;
        }
        if (s.travel)
        {
            ++travelShots;
            alongTravelSum += s.alongMm;
        }
        if (s.stale)
        {
            ++staleShots;
            staleAlongSum += s.staleAlongMm;
            // A dt implied by under 1 m/s of travel is noise.
            if (s.travel && s.speedMps >= 1.0f && std::isfinite(s.impliedDtMs))
            {
                ++dtShots;
                impliedDtSum += s.impliedDtMs;
            }
        }
    }

    // "SHOT LAG <title> (dev): ..." -- the whole line. Returns the characters written (0 when there is nothing to
    // report).
    size_t Format(const char* title, char* out, size_t size) const
    {
        if (!out || !size || (!shots && !unmeasured))
            return 0;
        size_t used = 0;
        const auto put = [&](const char* fmt, auto... args) {
            if (used >= size)
                return;
            const int w = std::snprintf(out + used, size - used, fmt, args...);
            if (w > 0)
                used += static_cast<size_t>(w) < size - used ? static_cast<size_t>(w) : size - used - 1;
        };
        put("SHOT LAG %s (dev): %u shots", title, shots);
        if (unmeasured)
            put(" (+%u unmeasured: no fresh intent point)", unmeasured);
        for (int b = 0; b < kShotLagBuckets; ++b)
        {
            put(b == 0 ? "; speed %s: %u" : "; %s: %u", ShotLagBucketName(b), n[b]);
            if (n[b])
                put(", miss %.0f mm (along %+.0f, across %.0f)", double(missSum[b] / float(n[b])),
                    double(alongSum[b] / float(n[b])), double(acrossSum[b] / float(n[b])));
        }
        if (noVelocity)
            put("; speed unknown: %u, miss %.0f mm", noVelocity, double(noVelocityMissSum / float(noVelocity)));
        if (travelShots)
            put("; mean lag axis along travel %+.0f mm", double(alongTravelSum / float(travelShots)));
        else
            put("; mean lag axis along travel n/a");
        if (staleShots)
        {
            put("; stale eye along travel %+.0f mm", double(staleAlongSum / float(staleShots)));
            if (dtShots)
                put(" (implied dt %.0f ms)", double(impliedDtSum / float(dtShots)));
            else
                put(" (implied dt n/a)");
        }
        else
            put("; stale eye n/a");
        if (shots)
            put("; at range %.1f m", double(rangeSum / float(shots)));
        put("; seated=%d seat=%d", seated ? 1 : 0, int(seat));
        return used;
    }
};

} // namespace shot_lag
