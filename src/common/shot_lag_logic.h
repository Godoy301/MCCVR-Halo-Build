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
    BendTooLarge,   // the new line leaves the old by more than the rails allow: not our query, or a stale point
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

// The rails. The lag being corrected is the eye's motion between the solve and the shot: at most a few metres
// (a Hornet at 30 m/s over 80 ms is 2.4 m), so a bend that moves the line by more than maxOffsetWu where it passes
// P, or turns it by more than maxBendDeg, is not this error -- a query from another camera, or a point the player
// no longer means. Both refuse; nothing is clamped. A query that already passes within minOffsetWu of P is LEFT
// ALONE (AlreadyThrough): where the game's line is already coherent -- Halo 3 campaign vehicles, user-confirmed
// 2026-10-02 -- the re-aim must not move it by the noise in P.
struct ReaimRails
{
    float minOffsetWu = 0.010f * kWuPerMetre;
    float minRangeWu = 0.3f * kWuPerMetre;
    float maxBendDeg = 30.0f;
    float maxOffsetWu = 3.0f * kWuPerMetre;
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
    // How far P sits off the old line, and the angle between the two lines.
    const float offsetSq = distance * distance - ahead * ahead;
    const float offset = offsetSq > 0.0f ? std::sqrt(offsetSq) : 0.0f;
    float cosine = ahead / distance;
    cosine = cosine > 1.0f ? 1.0f : (cosine < -1.0f ? -1.0f : cosine);
    const float bend = std::acos(cosine) * kRadToDeg;
    if (offset < rails.minOffsetWu)
        return Reaim::AlreadyThrough;
    if (!(offset <= rails.maxOffsetWu) || !(bend <= rails.maxBendDeg))
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
enum class QueryGate : uint8_t
{
    Try = 0,       // consult the intent (ShouldUseIntent)
    NotOurs,       // not the site, or not the local player's query: left alone and not counted
    OwnShotLine,   // the local fire's line belongs to another gun: left alone, counted (IntentUse::OwnShotLine)
};
inline QueryGate GateCrosshairQuery(bool atSite, bool queryLocal, bool ownShotLine)
{
    if (!atSite || !queryLocal)
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
// key that repeats after a long freeze is not the same frame).
inline bool SelectCoherentPair(const EyeFrame* eyes, size_t eyeCount, const HandFrame* hands, size_t handCount,
                               uint64_t maxSkewUs, size_t& eyeIndex, size_t& handIndex)
{
    bool found = false;
    uint64_t bestHand = 0, bestEye = 0;
    for (size_t h = 0; h < handCount; ++h)
    {
        if (!hands[h].valid || !hands[h].stampUs || (found && hands[h].stampUs < bestHand))
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
