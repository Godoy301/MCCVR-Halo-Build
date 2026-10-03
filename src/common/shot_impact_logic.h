#pragma once

// SHOT IMPACT -- where a round ACTUALLY goes, against the point P the player meant (dev witness, 2026-10-03).
//
// The SHOT LAG witness (shot_lag_logic.h section 6) measures the shot LINE we hand the game. With shots from the
// muzzle that line is built as normalize(P - M), so it passes through P by construction: its 0 mm is circular. This
// witness measures what the game made of it, from the game's own objects:
//
//   * at SPAWN: the projectile object's position, forward and velocity, read the moment the fire routine's
//     object_new returns. The forward carries everything the native did after our line (the firing assist's turn,
//     spread); the velocity carries what it inherited from the shooter (a Blam projectile flies along its velocity,
//     not its forward). Their line against P is where the round goes;
//   * a tick or two LATER: the same object's position and velocity -- a projectile moved or re-aimed after spawn;
//   * its IMPACT: the hit point of the projectile's own collision query (matched by its start at the object's
//     position), against P.
//
// Every offset is split like the SHOT LAG line: DEPTH along the view, ALONG the player's travel (the eye's velocity
// with its view component removed; + = ahead of where the player is going) and ACROSS (the rest). The REQUESTED
// direction is the line we handed the game; (forward - requested) is the native's turn, and the velocity's part off
// the forward is inheritance.
//
// Pure functions and plain structs only: unit-tested in tests/shot_impact_tests.cpp. World units are Blam's
// (1 wu = 3.048 m); times are microseconds of the sight layer's clock.

#include "shot_lag_logic.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace shot_impact
{
using shot_lag::Dot;
using shot_lag::Finite3;
using shot_lag::kMetresPerWu;
using shot_lag::kMmPerWu;
using shot_lag::kRadToDeg;
using shot_lag::kWuPerMetre;
using shot_lag::Length;

// ---- 1. an offset in the player's frame --------------------------------------------------------------------------
struct Axes
{
    bool valid = false;
    bool travel = false;     // the velocity had a usable component across the view (> 5 cm/s)
    float depthMm = 0.0f;    // along the view (+ = beyond)
    float alongMm = 0.0f;    // along the travel axis (+ = ahead of where the player is going); 0 without travel
    float acrossMm = 0.0f;   // the remaining lateral magnitude (all of it without travel)
};

// `offset` (wu), `view` (any length), `velocity` (wu/s, may be null).
inline Axes Decompose(const float offset[3], const float view[3], const float* velocity)
{
    Axes a{};
    if (!Finite3(offset) || !Finite3(view))
        return a;
    const float vl = Length(view);
    if (!std::isfinite(vl) || !(vl > 1.0e-6f))
        return a;
    const float f[3]{view[0] / vl, view[1] / vl, view[2] / vl};
    const float depth = Dot(offset, f);
    float lateral[3]{offset[0] - f[0] * depth, offset[1] - f[1] * depth, offset[2] - f[2] * depth};
    a.depthMm = depth * kMmPerWu;
    a.acrossMm = Length(lateral) * kMmPerWu;
    if (velocity && Finite3(velocity))
    {
        const float along = Dot(velocity, f);
        const float vp[3]{velocity[0] - f[0] * along, velocity[1] - f[1] * along, velocity[2] - f[2] * along};
        const float vpl = Length(vp);
        if (std::isfinite(vpl) && vpl * kMetresPerWu > 0.05f)
        {
            a.travel = true;
            const float t[3]{vp[0] / vpl, vp[1] / vpl, vp[2] / vpl};
            const float la = Dot(lateral, t);
            for (int axis = 0; axis < 3; ++axis)
                lateral[axis] -= t[axis] * la;
            a.alongMm = la * kMmPerWu;
            a.acrossMm = Length(lateral) * kMmPerWu;
        }
    }
    a.valid = std::isfinite(a.depthMm) && std::isfinite(a.alongMm) && std::isfinite(a.acrossMm);
    return a;
}

inline float AngleDeg(const float a[3], const float b[3])
{
    const float la = Length(a), lb = Length(b);
    if (!std::isfinite(la) || !std::isfinite(lb) || !(la > 1.0e-9f) || !(lb > 1.0e-9f))
        return NAN;
    // From the cross product's length and the dot: well conditioned for the tiny angles this measures.
    const float c[3]{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    return std::atan2(Length(c), Dot(a, b)) * kRadToDeg;
}

// ---- 2. the spawn proof ------------------------------------------------------------------------------------------
// The object fields are read at offsets DECODED from object_new's own copy of the placement block; this is the
// per-shot ValueAgreement that they are the right fields: position and forward bitwise equal to the placement's,
// and the velocity equal to the placement's plus something ALONG the forward (projectile_new adds forward times the
// projectile's initial speed) -- or still equal to it, if the type's constructor has not run yet.
inline bool SameBits3(const float a[3], const float b[3])
{
    return std::memcmp(a, b, sizeof(float) * 3) == 0;
}
inline bool SpawnProven(const float placementPos[3], const float placementFwd[3], const float placementVel[3],
                        const float objPos[3], const float objFwd[3], const float objVel[3])
{
    if (!Finite3(placementPos) || !Finite3(placementFwd) || !Finite3(placementVel) || !Finite3(objPos) ||
        !Finite3(objFwd) || !Finite3(objVel))
        return false;
    if (!SameBits3(placementPos, objPos) || !SameBits3(placementFwd, objFwd))
        return false;
    const float fl = Length(objFwd);
    if (!(fl > 0.5f) || !(fl < 2.0f))
        return false;
    const float delta[3]{objVel[0] - placementVel[0], objVel[1] - placementVel[1], objVel[2] - placementVel[2]};
    const float dl = Length(delta);
    if (dl <= 1.0e-4f)
        return true;   // the constructor has not added the initial speed yet
    const float along = Dot(delta, objFwd) / fl;
    const float off[3]{delta[0] - objFwd[0] / fl * along, delta[1] - objFwd[1] / fl * along,
                       delta[2] - objFwd[2] / fl * along};
    return along > 0.0f && Length(off) <= 1.0e-3f * dl + 1.0e-4f;
}

// ---- 3. what the game made of our shot line -----------------------------------------------------------------------
struct Spawn
{
    bool valid = false;
    float rangeM = 0.0f;        // |P - spawn|
    float turnDeg = 0.0f;       // angle(requested, forward): the native's turn after our line (assist, spread)
    Axes turn{};                // line(S, forward) minus line(S, requested) where they reach P's range
    float speedMps = 0.0f;      // |velocity|
    float lateralMps = 0.0f;    // |velocity off the forward|: what the shooter's motion added sideways
    float lateralAlongMps = 0.0f;   // that, along the travel axis (valid with lateralTravel)
    bool lateralTravel = false;
    float inheritRatio = 0.0f;  // lateralAlong / the eye's speed across the shot (1 = all of it inherited)
    bool inheritKnown = false;
    float driftAngleDeg = 0.0f; // angle(forward, velocity)
    // (velocity - requested * |velocity|): everything the game did to our line, as a velocity -- the native's turn
    // and the inheritance together -- along the travel axis taken across OUR direction (valid with deviationTravel).
    float deviationAlongMps = 0.0f;
    bool deviationTravel = false;
    shot_lag::ShotLagSample line{};   // line(S, velocity) against P: WHERE THE ROUND GOES
    float requestedMissMm = 0.0f;     // line(S, requested) against P: the circular check (~0 for muzzle shots)
    float tofMs = 0.0f;               // range / speed
};

// `requested`: the direction we handed the game (any length). `forward`, `velocity` (wu/s), `spawn`: the object's.
// `point`: P. `eyeVel`: the engine eye's velocity (wu/s), may be null.
inline Spawn MeasureSpawn(const float requested[3], const float forward[3], const float velocity[3],
                          const float spawn[3], const float point[3], const float* eyeVel)
{
    Spawn s{};
    if (!Finite3(requested) || !Finite3(forward) || !Finite3(velocity) || !Finite3(spawn) || !Finite3(point))
        return s;
    const float rl = Length(requested), fl = Length(forward), vl = Length(velocity);
    if (!(rl > 1.0e-6f) || !(fl > 1.0e-6f) || !(vl > 1.0e-6f))
        return s;
    const float d[3]{requested[0] / rl, requested[1] / rl, requested[2] / rl};
    const float f[3]{forward[0] / fl, forward[1] / fl, forward[2] / fl};
    const float w[3]{point[0] - spawn[0], point[1] - spawn[1], point[2] - spawn[2]};
    const float range = Length(w);
    if (!std::isfinite(range) || !(range > 1.0e-4f))
        return s;
    s.rangeM = range * kMetresPerWu;
    s.turnDeg = AngleDeg(d, f);
    const float turnOffset[3]{(f[0] - d[0]) * range, (f[1] - d[1]) * range, (f[2] - d[2]) * range};
    s.turn = Decompose(turnOffset, d, eyeVel);
    s.speedMps = vl * kMetresPerWu;
    const float va = Dot(velocity, f);
    const float lateral[3]{velocity[0] - f[0] * va, velocity[1] - f[1] * va, velocity[2] - f[2] * va};
    s.lateralMps = Length(lateral) * kMetresPerWu;
    s.driftAngleDeg = AngleDeg(f, velocity);
    if (eyeVel && Finite3(eyeVel))
    {
        const float ed = Dot(eyeVel, d);
        const float edp[3]{eyeVel[0] - d[0] * ed, eyeVel[1] - d[1] * ed, eyeVel[2] - d[2] * ed};
        const float edpl = Length(edp);
        if (std::isfinite(edpl) && edpl * kMetresPerWu > 0.05f)
        {
            const float deviation[3]{velocity[0] - d[0] * vl, velocity[1] - d[1] * vl, velocity[2] - d[2] * vl};
            s.deviationTravel = true;
            s.deviationAlongMps = Dot(deviation, edp) / edpl * kMetresPerWu;
        }
        const float ea = Dot(eyeVel, f);
        const float ep[3]{eyeVel[0] - f[0] * ea, eyeVel[1] - f[1] * ea, eyeVel[2] - f[2] * ea};
        const float epl = Length(ep);
        if (std::isfinite(epl) && epl * kMetresPerWu > 0.05f)
        {
            s.lateralTravel = true;
            const float t[3]{ep[0] / epl, ep[1] / epl, ep[2] / epl};
            s.lateralAlongMps = Dot(lateral, t) * kMetresPerWu;
            // A ratio from under 1 m/s of travel is noise.
            if (epl * kMetresPerWu >= 1.0f)
            {
                s.inheritKnown = true;
                s.inheritRatio = Dot(lateral, t) / epl;
            }
        }
    }
    s.line = shot_lag::MeasureShotLag(spawn, velocity, point, nullptr, eyeVel);
    const shot_lag::ShotLagSample requestedLine = shot_lag::MeasureShotLag(spawn, d, point, nullptr, nullptr);
    s.requestedMissMm = requestedLine.valid ? requestedLine.missMm : NAN;
    s.tofMs = s.speedMps > 0.0f ? s.rangeM / s.speedMps * 1000.0f : 0.0f;
    s.valid = s.line.valid && std::isfinite(s.turnDeg) && s.turn.valid && std::isfinite(s.lateralMps) &&
              std::isfinite(s.requestedMissMm);
    return s;
}

// Distance (mm) of `position` from the line (origin, direction): a projectile moved off its spawn line.
inline float OffLineMm(const float origin[3], const float direction[3], const float position[3])
{
    if (!Finite3(origin) || !Finite3(direction) || !Finite3(position))
        return NAN;
    const float l = Length(direction);
    if (!(l > 1.0e-9f))
        return NAN;
    const float u[3]{direction[0] / l, direction[1] / l, direction[2] / l};
    const float w[3]{position[0] - origin[0], position[1] - origin[1], position[2] - origin[2]};
    const float t = Dot(w, u);
    const float m[3]{w[0] - u[0] * t, w[1] - u[1] * t, w[2] - u[2] * t};
    return Length(m) * kMmPerWu;
}

// ---- 4. the projectile's own collision query ---------------------------------------------------------------------
// A collision query is the tracked projectile's when it STARTS at the object's current position (within startWu)
// and runs along its velocity (within maxDeg). Nothing else starts there and runs that way.
inline bool QueryIsProjectiles(const float objectPos[3], const float objectVel[3], const float start[3],
                               const float vector[3], float startWu, float maxDeg)
{
    if (!Finite3(objectPos) || !Finite3(objectVel) || !Finite3(start) || !Finite3(vector))
        return false;
    const float d[3]{start[0] - objectPos[0], start[1] - objectPos[1], start[2] - objectPos[2]};
    if (!(Length(d) <= startWu))
        return false;
    const float angle = AngleDeg(objectVel, vector);
    return std::isfinite(angle) && angle <= maxDeg;
}

// ---- 5. the tracked projectiles ----------------------------------------------------------------------------------
// A small fixed table, owned by the witness's lock. A projectile is armed at spawn with the P it was fired at; the
// per-tick poll samples it (up to kSamples), the collision hook records its hits, and it is retired when its object
// is gone (impact, or vanished) or after maxAgeUs. Full: the oldest is evicted (counted).
struct Tracked
{
    bool armed = false;
    uint32_t handle = UINT32_MAX;
    uint8_t group = 0;
    uint8_t samples = 0;
    bool impact = false;
    uint32_t impactCaller = 0;
    uint64_t armedUs = 0;
    float point[3]{}, view[3]{}, eyeVel[3]{};
    bool eyeVelValid = false;
    float spawnPos[3]{}, spawnVel[3]{};
    float lastPos[3]{}, lastVel[3]{};
    float impactPos[3]{};
};

struct Tracker
{
    static constexpr int kSlots = 16;
    static constexpr int kSamples = 2;
    Tracked slot[kSlots]{};
    uint32_t evicted = 0;

    int Armed() const
    {
        int n = 0;
        for (const auto& t : slot)
            n += t.armed ? 1 : 0;
        return n;
    }
    // The slot to arm: a free one, else the oldest (evicted).
    Tracked& Arm(uint64_t nowUs)
    {
        int oldest = 0;
        for (int i = 0; i < kSlots; ++i)
        {
            if (!slot[i].armed)
            {
                slot[i] = Tracked{};
                slot[i].armed = true;
                slot[i].armedUs = nowUs;
                return slot[i];
            }
            if (slot[i].armedUs < slot[oldest].armedUs)
                oldest = i;
        }
        ++evicted;
        slot[oldest] = Tracked{};
        slot[oldest].armed = true;
        slot[oldest].armedUs = nowUs;
        return slot[oldest];
    }
};

// ---- 6. the report windows ---------------------------------------------------------------------------------------
inline constexpr int kBuckets = shot_lag::kShotLagBuckets;

// One window per shot GROUP: how the shot was aimed (muzzle / native) x the fire's predicted flag.
struct Window
{
    // fires and projectiles
    uint32_t fires = 0, projectiles = 0, maxPerFire = 0, firesWithout = 0;
    uint32_t unproven = 0, noPoint = 0, unmeasured = 0;
    uint32_t noInitialSpeed = 0;   // the object's velocity had no initial speed yet at spawn (lines from ticks only)
    // spawn: the line(S, velocity) against P, per eye-speed bucket
    uint32_t n[kBuckets]{};
    float missSum[kBuckets]{}, alongSum[kBuckets]{}, acrossSum[kBuckets]{};
    uint32_t noVelocity = 0;
    float noVelocityMissSum = 0.0f;
    uint32_t travelShots = 0;
    float alongTravelSum = 0.0f;
    // spawn: what the native did after our line
    uint32_t spawnShots = 0;
    float turnDegSum = 0.0f, turnDegMax = 0.0f;
    uint32_t turnTravel = 0;
    float turnAlongSum = 0.0f;
    float speedSum = 0.0f, tofSum = 0.0f, rangeSum = 0.0f, requestedMissSum = 0.0f;
    float lateralSum = 0.0f, lateralMax = 0.0f;
    uint32_t lateralTravelShots = 0;
    float lateralAlongSum = 0.0f;
    uint32_t inheritShots = 0;
    float inheritSum = 0.0f;
    uint32_t deviationShots = 0;
    float deviationAlongSum = 0.0f;
    // later ticks
    uint32_t tickSamples = 0;
    float tickMissSum = 0.0f, tickAlongSum = 0.0f, tickOffLineMax = 0.0f;
    uint32_t tickTravel = 0;
    // impacts
    uint32_t impacts = 0, impactTravel = 0, goneWithoutImpact = 0, goneBeforeTick = 0, expired = 0;
    float impactDepthSum = 0.0f, impactAlongSum = 0.0f, impactAcrossSum = 0.0f;
    uint32_t impactCaller = 0;

    void AddFire(uint32_t projectilesThisFire)
    {
        ++fires;
        projectiles += projectilesThisFire;
        if (projectilesThisFire > maxPerFire)
            maxPerFire = projectilesThisFire;
        if (!projectilesThisFire)
            ++firesWithout;
    }
    void AddSpawn(const Spawn& s)
    {
        if (!s.valid)
        {
            ++unmeasured;
            return;
        }
        ++spawnShots;
        const shot_lag::ShotLagSample& l = s.line;
        if (l.bucket >= 0 && l.bucket < kBuckets)
        {
            ++n[l.bucket];
            missSum[l.bucket] += l.missMm;
            alongSum[l.bucket] += l.alongMm;
            acrossSum[l.bucket] += l.acrossMm;
        }
        else
        {
            ++noVelocity;
            noVelocityMissSum += l.missMm;
        }
        if (l.travel)
        {
            ++travelShots;
            alongTravelSum += l.alongMm;
        }
        turnDegSum += s.turnDeg;
        if (s.turnDeg > turnDegMax)
            turnDegMax = s.turnDeg;
        if (s.turn.travel)
        {
            ++turnTravel;
            turnAlongSum += s.turn.alongMm;
        }
        speedSum += s.speedMps;
        tofSum += s.tofMs;
        rangeSum += s.rangeM;
        requestedMissSum += s.requestedMissMm;
        lateralSum += s.lateralMps;
        if (s.lateralMps > lateralMax)
            lateralMax = s.lateralMps;
        if (s.lateralTravel)
        {
            ++lateralTravelShots;
            lateralAlongSum += s.lateralAlongMps;
        }
        if (s.inheritKnown)
        {
            ++inheritShots;
            inheritSum += s.inheritRatio;
        }
        if (s.deviationTravel)
        {
            ++deviationShots;
            deviationAlongSum += s.deviationAlongMps;
        }
    }
    void AddTick(const shot_lag::ShotLagSample& l, float offLineMm)
    {
        if (!l.valid)
            return;
        ++tickSamples;
        tickMissSum += l.missMm;
        if (l.travel)
        {
            ++tickTravel;
            tickAlongSum += l.alongMm;
        }
        if (std::isfinite(offLineMm) && offLineMm > tickOffLineMax)
            tickOffLineMax = offLineMm;
    }
    void AddImpact(const Axes& a, uint32_t caller)
    {
        if (!a.valid)
            return;
        ++impacts;
        impactDepthSum += a.depthMm;
        impactAcrossSum += a.acrossMm;
        if (a.travel)
        {
            ++impactTravel;
            impactAlongSum += a.alongMm;
        }
        impactCaller = caller;
    }

    bool Empty() const { return !fires && !projectiles && !unproven && !impacts && !goneWithoutImpact && !expired; }

    // "SHOT IMPACT <label> (dev): ..." -- the whole line. Returns the characters written (0: nothing to report).
    size_t Format(const char* label, char* out, size_t size) const
    {
        if (!out || !size || Empty())
            return 0;
        size_t used = 0;
        const auto put = [&](const char* fmt, auto... args) {
            if (used >= size)
                return;
            const int w = std::snprintf(out + used, size - used, fmt, args...);
            if (w > 0)
                used += static_cast<size_t>(w) < size - used ? static_cast<size_t>(w) : size - used - 1;
        };
        const auto mean = [](float sum, uint32_t count) { return count ? double(sum / float(count)) : 0.0; };
        put("SHOT IMPACT %s (dev): %u fires, %u projectiles (max %u per fire, %u fires none)", label, fires,
            projectiles, maxPerFire, firesWithout);
        if (unproven || noPoint || unmeasured || noInitialSpeed)
            put(" (+%u object unproven, %u no fresh P, %u unmeasured, %u not yet given their speed)", unproven,
                noPoint, unmeasured, noInitialSpeed);
        put("; ROUND LINE vs P");
        for (int b = 0; b < kBuckets; ++b)
        {
            put(b == 0 ? " speed %s: %u" : "; %s: %u", shot_lag::ShotLagBucketName(b), n[b]);
            if (n[b])
                put(", miss %.0f mm (along %+.0f, across %.0f)", mean(missSum[b], n[b]), mean(alongSum[b], n[b]),
                    mean(acrossSum[b], n[b]));
        }
        if (noVelocity)
            put("; speed unknown: %u, miss %.0f mm", noVelocity, mean(noVelocityMissSum, noVelocity));
        put("; along travel %s", travelShots ? "" : "n/a");
        if (travelShots)
            put("%+.0f mm", mean(alongTravelSum, travelShots));
        if (spawnShots)
        {
            put("; native turn after our line mean %.2f max %.2f deg", mean(turnDegSum, spawnShots),
                double(turnDegMax));
            if (turnTravel)
                put(" (along travel %+.0f mm at P)", mean(turnAlongSum, turnTravel));
            put("; our line vs P %.0f mm", mean(requestedMissSum, spawnShots));
            put("; velocity off the forward mean %.2f max %.2f m/s", mean(lateralSum, spawnShots),
                double(lateralMax));
            if (lateralTravelShots)
                put(" (along travel %+.2f m/s", mean(lateralAlongSum, lateralTravelShots));
            if (lateralTravelShots && inheritShots)
                put(", %.2f of the eye's", mean(inheritSum, inheritShots));
            if (lateralTravelShots)
                put(")");
            if (deviationShots)
                put("; velocity minus our direction x speed, along travel %+.2f m/s",
                    mean(deviationAlongSum, deviationShots));
            put("; speed %.0f m/s, flight %.1f ms, range %.1f m", mean(speedSum, spawnShots), mean(tofSum, spawnShots),
                mean(rangeSum, spawnShots));
        }
        if (tickSamples)
        {
            put("; later ticks %u: miss %.0f mm", tickSamples, mean(tickMissSum, tickSamples));
            if (tickTravel)
                put(" (along %+.0f)", mean(tickAlongSum, tickTravel));
            put(", off the spawn line max %.0f mm", double(tickOffLineMax));
        }
        else
            put("; later ticks 0");
        put("; IMPACT vs P %u", impacts);
        if (impacts)
        {
            put(": depth %+.0f, across %.0f mm", mean(impactDepthSum, impacts), mean(impactAcrossSum, impacts));
            if (impactTravel)
                put(", along travel %+.0f mm", mean(impactAlongSum, impactTravel));
            put(" (query +0x%X)", impactCaller);
        }
        put("; gone without a matched hit %u (%u before a tick), expired %u", goneWithoutImpact, goneBeforeTick,
            expired);
        return used;
    }
};

// What the player SAW when he fired, against P, and how old P was.
struct ReticleWindow
{
    uint32_t fires = 0, noReticle = 0, noFrame = 0, noPoint = 0;
    uint32_t n = 0, travel = 0;
    float alongSum = 0.0f, acrossSum = 0.0f, depthSum = 0.0f, lateralMax = 0.0f;
    float reticleAgeSum = 0.0f, reticleAgeMax = 0.0f;
    uint32_t pOlderFrame = 0;
    float frameLagSum = 0.0f;   // in prepared frames, over pOlderFrame
    float handAgeSum = 0.0f, handAgeMax = 0.0f, eyeAgeSum = 0.0f;
    uint32_t eyeAges = 0;
    float eyeKeySum = 0.0f;     // the reticle's eye vs its camera frame's head sample, mm (sanity)
    uint32_t triggers = 0;
    float triggerSum = 0.0f, triggerMax = 0.0f;

    void AddOffset(const Axes& a, float reticleAgeMs, float eyeKeyMm)
    {
        if (!a.valid)
            return;
        ++n;
        depthSum += a.depthMm;
        acrossSum += a.acrossMm;
        if (a.travel)
        {
            ++travel;
            alongSum += a.alongMm;
        }
        const float lateral = std::sqrt(a.alongMm * a.alongMm + a.acrossMm * a.acrossMm);
        if (lateral > lateralMax)
            lateralMax = lateral;
        reticleAgeSum += reticleAgeMs;
        if (reticleAgeMs > reticleAgeMax)
            reticleAgeMax = reticleAgeMs;
        eyeKeySum += eyeKeyMm;
    }
    void AddPointAge(float handAgeMs, bool eyeKnown, float eyeAgeMs, int64_t framesOlder)
    {
        handAgeSum += handAgeMs;
        if (handAgeMs > handAgeMax)
            handAgeMax = handAgeMs;
        if (eyeKnown)
        {
            ++eyeAges;
            eyeAgeSum += eyeAgeMs;
        }
        if (framesOlder > 0)
        {
            ++pOlderFrame;
            frameLagSum += float(framesOlder);
        }
    }
    void AddTrigger(float latencyMs)
    {
        ++triggers;
        triggerSum += latencyMs;
        if (latencyMs > triggerMax)
            triggerMax = latencyMs;
    }

    size_t Format(const char* title, char* out, size_t size) const
    {
        if (!out || !size || (!fires && !triggers))
            return 0;
        size_t used = 0;
        const auto put = [&](const char* fmt, auto... args) {
            if (used >= size)
                return;
            const int w = std::snprintf(out + used, size - used, fmt, args...);
            if (w > 0)
                used += static_cast<size_t>(w) < size - used ? static_cast<size_t>(w) : size - used - 1;
        };
        const auto mean = [](float sum, uint32_t count) { return count ? double(sum / float(count)) : 0.0; };
        put("SHOT RETICLE %s (dev): %u local fires", title, fires);
        if (noPoint || noReticle || noFrame)
            put(" (%u no fresh P, %u no fresh reticle, %u reticle frame not in the camera ring)", noPoint, noReticle,
                noFrame);
        put("; RETICLE vs P %u", n);
        if (n)
        {
            put(": along travel %s", travel ? "" : "n/a");
            if (travel)
                put("%+.0f mm", mean(alongSum, travel));
            put(", across %.0f, depth %+.0f mm, max lateral %.0f mm; reticle frame %.1f ms old (max %.1f); its eye vs "
                "its camera frame's head %.0f mm",
                mean(acrossSum, n), mean(depthSum, n), double(lateralMax), mean(reticleAgeSum, n),
                double(reticleAgeMax), mean(eyeKeySum, n));
        }
        const uint32_t pointFires = fires > noPoint ? fires - noPoint : 0;
        if (pointFires)
        {
            put("; P at fire: hand %.1f ms old (max %.1f)", mean(handAgeSum, pointFires), double(handAgeMax));
            if (eyeAges)
                put(", camera %.1f ms old", mean(eyeAgeSum, eyeAges));
            put(", from a frame older than the reticle's %u (by %.1f frames)", pOlderFrame,
                mean(frameLagSum, pOlderFrame));
        }
        put("; trigger (RT fed to the game) -> fire %u", triggers);
        if (triggers)
            put(": mean %.1f max %.1f ms", mean(triggerSum, triggers), double(triggerMax));
        return used;
    }
};

// The game's RT as fed to it: a rising edge opens a latency; the first local fire after it closes it. A fire with
// no open edge (an automatic weapon's later rounds) is not a latency.
struct TriggerLatch
{
    bool pressed = false;
    uint64_t edgeUs = 0;   // 0 = no open edge

    void Note(bool nowPressed, uint64_t nowUs)
    {
        if (nowPressed && !pressed)
            edgeUs = nowUs ? nowUs : 1;
        if (!nowPressed)
            edgeUs = 0;   // released before any fire: that press fired nothing
        pressed = nowPressed;
    }
    // A local fire at nowUs: the latency (ms) when an edge was open, else -1.
    float Fire(uint64_t nowUs)
    {
        if (!edgeUs || nowUs < edgeUs)
            return -1.0f;
        const float ms = float(nowUs - edgeUs) / 1000.0f;
        edgeUs = 0;
        return ms;
    }
};

// ---- 7. the reticle's ray in the world ---------------------------------------------------------------------------
// The aim path's own map from a tracking-space direction (OpenXR: -Z forward, +Y up) to the game's world (Z up):
// yaw = gameYawRef + yawSign * wrap(atan2(x, -z) - headYawRef), pitch = pitchSign * asin(y) clamped to +-1.45 rad
// (game.cpp, the on-foot sight ray). The reticle must be judged through the SAME map the aim used, so that only what
// differs between them -- which frame, which pose, which range -- shows up.
inline float WrapPi(float a)
{
    const float twoPi = 6.283185307179586f;
    a = std::fmod(a + 3.141592653589793f, twoPi);
    if (a < 0.0f)
        a += twoPi;
    return a - 3.141592653589793f;
}
inline bool TrackingDirToWorld(const float dir[3], float gameYawRef, float headYawRef, float yawSign,
                               float pitchSign, float out[3])
{
    if (!Finite3(dir) || !std::isfinite(gameYawRef) || !std::isfinite(headYawRef) || !std::isfinite(yawSign) ||
        !std::isfinite(pitchSign))
        return false;
    const float l = Length(dir);
    if (!(l > 1.0e-6f))
        return false;
    const float x = dir[0] / l, y = dir[1] / l, z = dir[2] / l;
    const float yaw = gameYawRef + yawSign * WrapPi(std::atan2(x, -z) - headYawRef);
    float s = y < -1.0f ? -1.0f : (y > 1.0f ? 1.0f : y);
    float pitch = pitchSign * std::asin(s);
    pitch = pitch > 1.45f ? 1.45f : (pitch < -1.45f ? -1.45f : pitch);
    out[0] = std::cos(pitch) * std::cos(yaw);
    out[1] = std::cos(pitch) * std::sin(yaw);
    out[2] = std::sin(pitch);
    return Finite3(out);
}

} // namespace shot_impact
