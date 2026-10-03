#pragma once

// AS SEEN -- an impact against the crosshair the player is LOOKING AT when the impact first appears (dev, 2026-10-03).
//
// The SHOT IMPACT witness (shot_impact_logic.h) proved where the rounds go: onto P, the point the player meant at the
// fire. That is not what the player judges. He judges the impact against the crosshair ON SCREEN when the impact
// appears, and by then he has strafed on. This file is that measurement, written once and used by two hosts so the
// two numbers are computed identically:
//
//   * the VR dev build (game.cpp halo3_impact_witness.inl): the reticle quad the compositor drew in the first
//     prepared frame whose render follows the impact, mapped into the world through that frame's own camera; and the
//     game's own (stock) crosshair line in that frame: the engine camera's position and aim forward;
//   * the flat probe (src/probe/flat_probe.cpp, HaloMCCFlatProbe.dll): stock Halo 3 with nothing of ours in the
//     game, the camera of the first presented frame rendered after the impact, along its forward.
//
// For each impact:
//   SEEN    the impact against the crosshair line of that frame, from that frame's eye: the offset off the line at
//           the impact's own depth, split ALONG the travel (the eye's velocity across the line; + = ahead of where
//           the player is going) and UP (the rest of it, + = up), in mm and in degrees, and in RETICLE RADII (the
//           host's own reticle size, see Window::Add);
//   STOCK   the same against the stock crosshair line of that frame (in flat it IS the seen line);
//   AT FIRE the same against the aim line at the fire (VR: P's line, its rendered eye through P; flat: the crosshair
//           of the newest frame rendered before the fire) -- where the round went against what was aimed at;
//   CLOCKS  the aim's camera sample -> the fire -> the impact -> the first frame's camera -> its present, in ms of one
//           clock, the sim ticks between the fire and the impact, and how far the CAMERA EYE moved along the travel
//           between the aim's camera sample and the frame's camera. When the seen offset is the camera's own advance,
//           "eye moved" is minus "along", and the ms say which clock difference made it.
//
// Every metric is binned by strafe speed and by range, so flat and VR are compared bin for bin without anyone
// knowing their distance to the wall.
//
// Pure functions and plain structs only: unit-tested in tests/as_seen_tests.cpp. World units are Blam's (1 wu =
// 3.048 m, Z up); velocities wu/s; times microseconds of the host's one clock.

#include "shot_lag_logic.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace as_seen
{
using shot_lag::Dot;
using shot_lag::Finite3;
using shot_lag::kMetresPerWu;
using shot_lag::kMmPerWu;
using shot_lag::kRadToDeg;
using shot_lag::kWuPerMetre;
using shot_lag::Length;

// ---- 1. the bins -------------------------------------------------------------------------------------------------
inline constexpr int kRangeBins = 5;
inline constexpr float kRangeEdgesM[kRangeBins - 1]{5.0f, 10.0f, 20.0f, 40.0f};
inline constexpr int kSpeedBins = 3;
inline constexpr float kSpeedEdgesMps[kSpeedBins - 1]{1.0f, 4.0f};

inline int RangeBin(float metres)
{
    if (!std::isfinite(metres))
        return kRangeBins - 1;
    int bin = 0;
    while (bin < kRangeBins - 1 && metres >= kRangeEdgesM[bin])
        ++bin;
    return bin;
}
inline int SpeedBin(float mps)
{
    if (!std::isfinite(mps))
        return 0;
    int bin = 0;
    while (bin < kSpeedBins - 1 && mps >= kSpeedEdgesMps[bin])
        ++bin;
    return bin;
}
inline const char* RangeLabel(int bin)
{
    static const char* const kLabels[kRangeBins]{"0-5 m", "5-10 m", "10-20 m", "20-40 m", "40+ m"};
    return bin >= 0 && bin < kRangeBins ? kLabels[bin] : "?";
}
inline const char* SpeedLabel(int bin)
{
    static const char* const kLabels[kSpeedBins]{"under 1 m/s", "1-4 m/s", "4+ m/s"};
    return bin >= 0 && bin < kSpeedBins ? kLabels[bin] : "?";
}

// ---- 2. the axes across a line -----------------------------------------------------------------------------------
// The plane across `dir`: TRAVEL (the velocity's part across the line, when it is over 5 cm/s; else the horizontal
// right of the line, and `travel` false) and UP (dir x travel, turned to point up). Halo's world is Z up.
struct Across
{
    bool valid = false;
    bool travel = false;
    float d[3]{};      // the line's unit direction
    float t[3]{};      // the travel axis (or the horizontal right)
    float n[3]{};      // the up axis across the line
    float speedMps = 0.0f;   // the velocity across the line (game metres per second); 0 without travel
};

inline Across AxesAcross(const float dir[3], const float* velocity)
{
    Across a{};
    if (!Finite3(dir))
        return a;
    const float dl = Length(dir);
    if (!std::isfinite(dl) || !(dl > 1.0e-6f))
        return a;
    for (int axis = 0; axis < 3; ++axis)
        a.d[axis] = dir[axis] / dl;
    bool haveT = false;
    if (velocity && Finite3(velocity))
    {
        const float along = Dot(velocity, a.d);
        const float vp[3]{velocity[0] - a.d[0] * along, velocity[1] - a.d[1] * along, velocity[2] - a.d[2] * along};
        const float vpl = Length(vp);
        if (std::isfinite(vpl) && vpl * kMetresPerWu > 0.05f)
        {
            for (int axis = 0; axis < 3; ++axis)
                a.t[axis] = vp[axis] / vpl;
            a.speedMps = vpl * kMetresPerWu;
            a.travel = true;
            haveT = true;
        }
    }
    if (!haveT)
    {
        // The horizontal right of the line: d x up.
        const float r[3]{a.d[1], -a.d[0], 0.0f};
        const float rl = Length(r);
        if (!(rl > 1.0e-6f))
        {
            a.t[0] = 1.0f;   // looking straight up or down: any horizontal axis
            a.t[1] = a.t[2] = 0.0f;
        }
        else
            for (int axis = 0; axis < 3; ++axis)
                a.t[axis] = r[axis] / rl;
    }
    // n = d x t, then made to point up (or, for a vertical travel axis, kept as it is).
    a.n[0] = a.d[1] * a.t[2] - a.d[2] * a.t[1];
    a.n[1] = a.d[2] * a.t[0] - a.d[0] * a.t[2];
    a.n[2] = a.d[0] * a.t[1] - a.d[1] * a.t[0];
    if (a.n[2] < 0.0f)
        for (float& c : a.n)
            c = -c;
    a.valid = Finite3(a.t) && Finite3(a.n);
    return a;
}

// ---- 3. a point against a drawn line -----------------------------------------------------------------------------
struct Offset
{
    bool valid = false;
    bool travel = false;
    float rangeM = 0.0f;     // |point - origin|
    float depthM = 0.0f;     // along the line
    float angleDeg = 0.0f;   // between (point - origin) and the line
    float alongMm = 0.0f;    // off the line at the point's depth, along the travel (+ = ahead of the player's motion)
    float upMm = 0.0f;       // off the line at the point's depth, along the up axis across it (+ = up)
    float alongDeg = 0.0f;   // atan(along / depth)
    float upDeg = 0.0f;      // atan(up / depth)
    float speedMps = 0.0f;   // the eye's velocity across the line
};

// `point` (the impact), the line from `origin` (the eye that drew it) along `dir`, `velocity` the eye's (wu/s; null
// = no travel axis: "along" is then the horizontal right).
inline Offset Measure(const float point[3], const float origin[3], const float dir[3], const float* velocity)
{
    Offset o{};
    if (!Finite3(point) || !Finite3(origin))
        return o;
    const Across a = AxesAcross(dir, velocity);
    if (!a.valid)
        return o;
    const float w[3]{point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
    const float range = Length(w);
    const float depth = Dot(w, a.d);
    if (!std::isfinite(range) || !(range > 1.0e-4f) || !std::isfinite(depth))
        return o;
    const float lateral[3]{w[0] - a.d[0] * depth, w[1] - a.d[1] * depth, w[2] - a.d[2] * depth};
    const float along = Dot(lateral, a.t), up = Dot(lateral, a.n);
    o.travel = a.travel;
    o.speedMps = a.speedMps;
    o.rangeM = range * kMetresPerWu;
    o.depthM = depth * kMetresPerWu;
    o.angleDeg = std::atan2(Length(lateral), depth) * kRadToDeg;
    o.alongMm = along * kMmPerWu;
    o.upMm = up * kMmPerWu;
    o.alongDeg = std::atan2(along, depth) * kRadToDeg;
    o.upDeg = std::atan2(up, depth) * kRadToDeg;
    o.valid = std::isfinite(o.angleDeg) && std::isfinite(o.alongMm) && std::isfinite(o.upMm) && depth > 0.0f;
    return o;
}

// A displacement (wu) along the travel axis of the line `dir` with `velocity`, in mm (0 when there is no travel).
inline float AlongTravelMm(const float delta[3], const float dir[3], const float* velocity, bool* known = nullptr)
{
    const Across a = AxesAcross(dir, velocity);
    const bool ok = a.valid && a.travel && delta && Finite3(delta);
    if (known)
        *known = ok;
    return ok ? Dot(delta, a.t) * kMmPerWu : 0.0f;
}

// ---- 4. one impact -----------------------------------------------------------------------------------------------
struct Sample
{
    Offset seen{};          // vs the crosshair drawn in the first frame showing the impact
    bool stockValid = false;
    Offset stock{};         // vs the stock crosshair line of that frame
    bool atFireValid = false;
    Offset atFire{};        // vs the aim line at the fire
    // One clock (us); 0 = unknown.
    uint64_t aimCamUs = 0, fireUs = 0, impactUs = 0, frameCamUs = 0, presentUs = 0;
    int32_t ticksFireToImpact = -1;   // sim ticks from the fire to the impact
    int32_t ticksAimToFrame = -1;     // sim ticks from the aim's camera sample to the frame's camera
    bool eyeMovedKnown = false;
    float eyeMovedAlongMm = 0.0f;     // the camera eye, aim sample -> frame, along `seen`'s travel axis
};

inline float MsBetween(uint64_t fromUs, uint64_t toUs)
{
    if (!fromUs || !toUs)
        return NAN;
    return toUs >= fromUs ? float(toUs - fromUs) / 1000.0f : -float(fromUs - toUs) / 1000.0f;
}

// ---- 5. the window: per speed bin and range bin ------------------------------------------------------------------
struct Bin
{
    uint32_t n = 0;
    double rangeM = 0, speedMps = 0;
    double seenAlongMm = 0, seenUpMm = 0, seenAlongDeg = 0, seenUpDeg = 0, seenAngleDeg = 0;
    uint32_t nRadii = 0;
    double seenAlongRadii = 0, seenAngleRadii = 0;
    uint32_t nImplied = 0;
    double impliedMs = 0;              // -seen along / the travel speed: the offset as time at the strafe speed
    uint32_t nStock = 0;
    double stockAlongMm = 0, stockAlongDeg = 0, stockUpMm = 0;
    uint32_t nAtFire = 0;
    double atFireAlongMm = 0, atFireUpMm = 0;
    uint32_t nEye = 0;
    double eyeMovedMm = 0, eyeMovedMs = 0;
    uint32_t nAimFire = 0, nFireImpact = 0, nImpactFrame = 0, nFramePresent = 0;
    double aimToFireMs = 0, fireToImpactMs = 0, impactToFrameMs = 0, frameToPresentMs = 0;
    uint32_t nTicksFire = 0, nTicksAim = 0;
    double ticksFireToImpact = 0, ticksAimToFrame = 0;
};

struct Window
{
    Bin bin[kSpeedBins][kRangeBins]{};
    uint32_t impacts = 0;       // impacts that reached a frame
    uint32_t unframed = 0;      // impacts no frame showed in time (host counts)
    uint32_t noLine = 0;        // impacts whose frame had no crosshair line (host counts)
    uint32_t refused = 0;       // a sample that did not measure

    // `radiusDeg`: the host's reticle radius as drawn (<= 0: no radii column).
    void Add(const Sample& s, float radiusDeg)
    {
        if (!s.seen.valid)
        {
            ++refused;
            return;
        }
        ++impacts;
        Bin& b = bin[SpeedBin(s.seen.travel ? s.seen.speedMps : 0.0f)][RangeBin(s.seen.rangeM)];
        ++b.n;
        b.rangeM += s.seen.rangeM;
        b.speedMps += s.seen.speedMps;
        b.seenAlongMm += s.seen.alongMm;
        b.seenUpMm += s.seen.upMm;
        b.seenAlongDeg += s.seen.alongDeg;
        b.seenUpDeg += s.seen.upDeg;
        b.seenAngleDeg += s.seen.angleDeg;
        if (std::isfinite(radiusDeg) && radiusDeg > 0.0f)
        {
            ++b.nRadii;
            b.seenAlongRadii += s.seen.alongDeg / radiusDeg;
            b.seenAngleRadii += s.seen.angleDeg / radiusDeg;
        }
        if (s.seen.travel && s.seen.speedMps > 0.5f)
        {
            ++b.nImplied;
            b.impliedMs += -s.seen.alongMm / s.seen.speedMps;   // mm / (m/s) = ms
        }
        if (s.stockValid && s.stock.valid)
        {
            ++b.nStock;
            b.stockAlongMm += s.stock.alongMm;
            b.stockAlongDeg += s.stock.alongDeg;
            b.stockUpMm += s.stock.upMm;
        }
        if (s.atFireValid && s.atFire.valid)
        {
            ++b.nAtFire;
            b.atFireAlongMm += s.atFire.alongMm;
            b.atFireUpMm += s.atFire.upMm;
        }
        if (s.eyeMovedKnown && std::isfinite(s.eyeMovedAlongMm))
        {
            ++b.nEye;
            b.eyeMovedMm += s.eyeMovedAlongMm;
            if (s.seen.travel && s.seen.speedMps > 0.5f)
                b.eyeMovedMs += s.eyeMovedAlongMm / s.seen.speedMps;
        }
        const auto add = [](uint32_t& n, double& sum, float ms) {
            if (std::isfinite(ms))
            {
                ++n;
                sum += ms;
            }
        };
        add(b.nAimFire, b.aimToFireMs, MsBetween(s.aimCamUs, s.fireUs));
        add(b.nFireImpact, b.fireToImpactMs, MsBetween(s.fireUs, s.impactUs));
        add(b.nImpactFrame, b.impactToFrameMs, MsBetween(s.impactUs, s.frameCamUs));
        add(b.nFramePresent, b.frameToPresentMs, MsBetween(s.frameCamUs, s.presentUs));
        if (s.ticksFireToImpact >= 0)
        {
            ++b.nTicksFire;
            b.ticksFireToImpact += s.ticksFireToImpact;
        }
        if (s.ticksAimToFrame >= 0)
        {
            ++b.nTicksAim;
            b.ticksAimToFrame += s.ticksAimToFrame;
        }
    }

    bool Any() const { return impacts || unframed || noLine || refused; }

    // One line: "AS SEEN <label> (dev): ..." with every non-empty bin. False when there is nothing to say.
    bool Format(const char* label, char* out, size_t size) const
    {
        if (!out || !size)
            return false;
        out[0] = '\0';
        if (!Any())
            return false;
        size_t used = 0;
        const auto put = [&](const char* fmt, auto... args) {
            if (used + 1 >= size)
                return;
            const int w = std::snprintf(out + used, size - used, fmt, args...);
            if (w > 0)
                used = used + static_cast<size_t>(w) >= size ? size - 1 : used + static_cast<size_t>(w);
        };
        const auto mean = [](double sum, uint32_t n) { return n ? sum / static_cast<double>(n) : 0.0; };
        put("AS SEEN %s (dev): %u impacts shown", label ? label : "?", impacts);
        if (unframed || noLine || refused)
            put(" (%u never reached a frame, %u frames without a crosshair line, %u refused)", unframed, noLine,
                refused);
        for (int s = 0; s < kSpeedBins; ++s)
            for (int r = 0; r < kRangeBins; ++r)
            {
                const Bin& b = bin[s][r];
                if (!b.n)
                    continue;
                put(" | [%s, %s] %u at %.1f m, strafe %.1f m/s: SEEN along %+.0f mm (%+.2f deg", SpeedLabel(s),
                    RangeLabel(r), b.n, mean(b.rangeM, b.n), mean(b.speedMps, b.n), mean(b.seenAlongMm, b.n),
                    mean(b.seenAlongDeg, b.n));
                if (b.nRadii)
                    put(", %+.2f radii", mean(b.seenAlongRadii, b.nRadii));
                if (b.nImplied)
                    put(", = %.0f ms of strafe", mean(b.impliedMs, b.nImplied));
                put(") up %+.0f mm (%+.2f deg), total %.2f deg", mean(b.seenUpMm, b.n), mean(b.seenUpDeg, b.n),
                    mean(b.seenAngleDeg, b.n));
                if (b.nRadii)
                    put(" (%.2f radii)", mean(b.seenAngleRadii, b.nRadii));
                if (b.nStock)
                    put("; STOCK along %+.0f mm (%+.2f deg) up %+.0f mm", mean(b.stockAlongMm, b.nStock),
                        mean(b.stockAlongDeg, b.nStock), mean(b.stockUpMm, b.nStock));
                if (b.nAtFire)
                    put("; AT FIRE along %+.0f up %+.0f mm", mean(b.atFireAlongMm, b.nAtFire),
                        mean(b.atFireUpMm, b.nAtFire));
                if (b.nEye)
                    put("; camera eye aim->frame %+.0f mm (%+.0f ms)", mean(b.eyeMovedMm, b.nEye),
                        mean(b.eyeMovedMs, b.nEye));
                put("; clocks aim cam->fire %.1f, fire->impact %.1f", mean(b.aimToFireMs, b.nAimFire),
                    mean(b.fireToImpactMs, b.nFireImpact));
                if (b.nTicksFire)
                    put(" (%.1f ticks)", mean(b.ticksFireToImpact, b.nTicksFire));
                put(", impact->frame %.1f", mean(b.impactToFrameMs, b.nImpactFrame));
                if (b.nFramePresent)
                    put(", frame->present %.1f", mean(b.frameToPresentMs, b.nFramePresent));
                put(" ms");
                if (b.nTicksAim)
                    put("; ticks aim cam->frame %.1f", mean(b.ticksAimToFrame, b.nTicksAim));
            }
        return true;
    }
};

// ---- 6. is the camera interpolated? (hypothesis a) ---------------------------------------------------------------
// Per drawn frame: did the camera eye move since the previous frame, while the player is moving? A camera rendered
// straight from the 60 Hz simulation repeats its position on every frame drawn between two ticks (at 90 Hz about one
// frame in three) and steps by whole ticks; an interpolated camera moves on every frame by about speed x frame time.
// `stepRatio` = |step| / (speed x frame time): ~1 every frame when interpolated, 0 and ~1.5 alternating when not.
struct CameraCensus
{
    static constexpr int kHistory = 16;
    float pos[kHistory][3]{};
    uint64_t us[kHistory]{};
    int count = 0, head = 0;   // ring: head = the next write
    // Counts (reset by TakeCounts / a window swap).
    uint32_t frames = 0, moving = 0, repeated = 0, uneven = 0, stepN = 0;
    double stepRatioSum = 0, speedSum = 0, frameMsSum = 0;

    void Add(const float p[3], uint64_t nowUs)
    {
        if (!Finite3(p) || !nowUs)
            return;
        ++frames;
        if (count)
        {
            const int prev = (head + kHistory - 1) % kHistory;
            if (nowUs > us[prev])
            {
                // The speed over the history (at least 40 ms back), independent of this one step.
                int oldest = -1;
                for (int k = 1; k <= count; ++k)
                {
                    const int i = (head + kHistory - k) % kHistory;
                    if (nowUs - us[i] >= 40000)
                    {
                        oldest = i;
                        break;
                    }
                }
                if (oldest >= 0)
                {
                    const float span[3]{p[0] - pos[oldest][0], p[1] - pos[oldest][1], p[2] - pos[oldest][2]};
                    const float speedMps = Length(span) * kMetresPerWu / (float(nowUs - us[oldest]) * 1.0e-6f);
                    if (std::isfinite(speedMps) && speedMps > 1.0f && speedMps < 50.0f)
                    {
                        ++moving;
                        const float step[3]{p[0] - pos[prev][0], p[1] - pos[prev][1], p[2] - pos[prev][2]};
                        if (step[0] == 0.0f && step[1] == 0.0f && step[2] == 0.0f)
                            ++repeated;
                        const float dt = float(nowUs - us[prev]) * 1.0e-6f;
                        const float ratio = Length(step) * kMetresPerWu / (speedMps * dt);
                        if (std::isfinite(ratio))
                        {
                            ++stepN;
                            stepRatioSum += ratio;
                            speedSum += speedMps;
                            frameMsSum += dt * 1000.0f;
                            if (ratio < 0.75f || ratio > 1.25f)
                                ++uneven;
                        }
                    }
                }
            }
        }
        std::memcpy(pos[head], p, sizeof(pos[head]));
        us[head] = nowUs;
        head = (head + 1) % kHistory;
        if (count < kHistory)
            ++count;
    }

    void ResetCounts()
    {
        frames = moving = repeated = uneven = stepN = 0;
        stepRatioSum = speedSum = frameMsSum = 0;
    }

    // One clause, or false while nothing moved.
    bool Format(char* out, size_t size) const
    {
        if (!out || !size)
            return false;
        out[0] = '\0';
        if (!moving || !stepN)
            return false;
        std::snprintf(out, size,
                      "%u frames, %u while moving (mean %.1f m/s, frame %.1f ms): the camera eye REPEATED on %u "
                      "(%.0f%%), step / (speed x frame time) mean %.2f, outside 0.75-1.25 on %u (%.0f%%)",
                      frames, moving, speedSum / stepN, frameMsSum / stepN, repeated, 100.0 * repeated / moving,
                      stepRatioSum / stepN, uneven, 100.0 * uneven / stepN);
        return true;
    }
};

// ---- 7. the flat reticle's size, art for art ---------------------------------------------------------------------
// The VR reticle shows the game's own crosshair art captured from the central +-(128 / hudWidth) of the HUD's NDC
// width (vr.cpp: Halo 3's capture viewport is the HUD viewport x4, centred in a 512 texture) on a quad
// crosshair_size_deg wide, so its "radius" (the quad's half-angle) frames exactly that art. The same art in flat
// spans atan(that NDC half-width x the flat projection's horizontal tangent). Radii in both hosts are then fractions
// of the same piece of art -- the assumption being that the HUD lays the crosshair out in NDC (it scales with the
// raster), which the VR capture already relies on.
inline float FlatArtHalfAngleDeg(float vrHudWidthPx, float flatTanHalfX)
{
    if (!std::isfinite(vrHudWidthPx) || !(vrHudWidthPx > 0.0f) || !std::isfinite(flatTanHalfX) ||
        !(flatTanHalfX > 0.0f))
        return NAN;
    return std::atan(128.0f / vrHudWidthPx * flatTanHalfX) * kRadToDeg;
}
} // namespace as_seen
