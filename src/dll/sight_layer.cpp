// The title-agnostic sight layer (see sight_layer.h). Every publication is a seqlock with one writer
// thread per title: the ray from the aim path, the trace result from the title's simulation hook.

#include "sight_layer.h"
#include "title_adapter.h"
#include "../common/aim_solve_logic.h"
#include "../common/config.h"
#include "../common/log.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{
constexpr size_t kTitleSlots = 8;

struct RayPublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint64_t> sampleMs{0};
    std::atomic<float> head[3]{};
    std::atomic<float> dir[3]{};
    std::atomic<float> eye[3]{};
    std::atomic<int32_t> unit{-1}, ignoreB{-1}, ignoreC{-1};
    std::atomic<float> eyeVel[3]{};
    std::atomic<bool> eyeVelValid{false};
    std::atomic<uint64_t> eyeStampUs{0};
};

struct TracePublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint64_t> sampleMs{0};
    std::atomic<uint32_t> hit{0};
    std::atomic<float> rangeWu{0.0f};
    std::atomic<float> convergeRangeWu{0.0f};
    std::atomic<bool> faulted{false};
    std::atomic<uint64_t> lastTraceMs{0};
    // The inverse-range average, owned by the title's trace thread (Sight_ResetTitle runs at install,
    // before that thread traces).
    std::atomic<float> invRangePerWu{0.0f};
    std::atomic<uint64_t> smoothMs{0};
    std::atomic<uint32_t> smoothGeneration{0};
};

struct ConvergedStamp
{
    std::atomic<uint64_t> atMs{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<float> unitsPerVrMetre{0.0f};
    // Sight_MarkAimWritten: the converged aim is the game's aim itself, not a servo's target.
    std::atomic<uint64_t> writtenMs{0};
    std::atomic<uint32_t> writtenGeneration{0};
};

struct Stats
{
    std::atomic<uint64_t> rays{0}, traces{0}, hits{0}, misses{0}, faults{0}, converged{0}, declined{0}, written{0};
    std::atomic<uint64_t> costTicks{0}, costMaxTicks{0};
    // Shots from the muzzle: aimed through the converged point, refused by a rail, or without a point; and
    // (defensive) held off because the gun points elsewhere, naturally -- a seat held at its stop, a turret.
    std::atomic<uint64_t> muzzleAimed{0}, muzzleRefused{0}, muzzleUnconverged{0}, muzzleHeld{0};
};

RayPublication g_rays[kTitleSlots];
// The engine eye's track per title, owned by the title's ray publisher (one aim thread per title; the try-lock only
// keeps a second publisher, should one ever appear, from tearing it -- that publish then carries no velocity).
shot_lag::EyeTrack g_eyeTracks[kTitleSlots];
std::atomic_flag g_eyeTrackBusy[kTitleSlots] = {};
TracePublication g_traces[kTitleSlots];
ConvergedStamp g_converged[kTitleSlots];
Stats g_stats[kTitleSlots];

size_t Slot(GameTitle title) { return static_cast<size_t>(title) % kTitleSlots; }

bool Fresh(uint64_t at, uint64_t now, uint64_t maxAge) { return at && now >= at && now - at <= maxAge; }

// A seqlock over a trivially copyable value, every word atomic (the committed-aim-sample discipline): one writer per
// slot, any reader; a torn read retries once, then reports nothing.
template <class T>
struct SeqBlob
{
    static constexpr size_t kWords = (sizeof(T) + 7) / 8;
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint64_t> words[kWords]{};
    void Publish(const T& value)
    {
        uint64_t raw[kWords]{};
        memcpy(raw, &value, sizeof(T));
        sequence.fetch_add(1, std::memory_order_acq_rel);
        std::atomic_thread_fence(std::memory_order_release);
        for (size_t i = 0; i < kWords; ++i)
            words[i].store(raw[i], std::memory_order_relaxed);
        sequence.fetch_add(1, std::memory_order_release);
    }
    bool Read(T& out) const
    {
        for (int attempt = 0; attempt < 2; ++attempt)
        {
            const uint32_t before = sequence.load(std::memory_order_acquire);
            if (!before || (before & 1u))
                continue;
            uint64_t raw[kWords]{};
            for (size_t i = 0; i < kWords; ++i)
                raw[i] = words[i].load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (sequence.load(std::memory_order_relaxed) != before)
                continue;
            memcpy(&out, raw, sizeof(T));
            return true;
        }
        return false;
    }
};

const char* TitleName(GameTitle title)
{
    switch (title)
    {
    case GameTitle::Halo3: return "Halo 3";
    case GameTitle::Halo3ODST: return "ODST";
    case GameTitle::HaloReach: return "Reach";
    case GameTitle::Halo4: return "Halo 4";
    case GameTitle::HaloCE: return "Halo CE";
    case GameTitle::Halo2: return "Halo 2";
    default: return "?";
    }
}
} // namespace

bool Sight_AimEnabled()
{
    return g_config.converged_aim != 0;
}

uint64_t Sight_NowUs()
{
    static const int64_t s_frequency = [] {
        LARGE_INTEGER f{};
        QueryPerformanceFrequency(&f);
        return f.QuadPart > 0 ? f.QuadPart : 1;
    }();
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);
    const int64_t ticks = now.QuadPart;
    // Split so ticks * 1e6 cannot overflow over a long uptime.
    return static_cast<uint64_t>(ticks / s_frequency) * 1000000ull +
           static_cast<uint64_t>((ticks % s_frequency) * 1000000ll / s_frequency);
}

uint64_t Sight_NowMs()
{
    return Sight_NowUs() / 1000ull;
}

void Sight_PublishRay(GameTitle title, const SightRay& rayIn)
{
    if (!Sight_AimEnabled())
        return;
    SightRay ray = rayIn;
    if (!ray.eyeVelValid)
    {
        // The eye's velocity over ~100 ms, on this (the publishing) thread, on the eye's own clock where the
        // publisher knows it.
        const size_t slot = Slot(title);
        if (!g_eyeTrackBusy[slot].test_and_set(std::memory_order_acquire))
        {
            auto& track = g_eyeTracks[slot];
            static uint32_t s_trackGeneration[kTitleSlots]{};
            if (s_trackGeneration[slot] != ray.generation)
            {
                track.Reset();
                s_trackGeneration[slot] = ray.generation;
            }
            track.Push(ray.eyeStampUs ? ray.eyeStampUs : Sight_NowUs(), ray.eye);
            ray.eyeVelValid = track.Velocity(120000, 30000, ray.eyeVel);
            g_eyeTrackBusy[slot].clear(std::memory_order_release);
        }
    }
    auto& p = g_rays[Slot(title)];
    p.sequence.fetch_add(1, std::memory_order_acq_rel);   // odd: a publish is in progress
    p.generation.store(ray.generation, std::memory_order_relaxed);
    p.sampleMs.store(ray.sampleMs, std::memory_order_relaxed);
    for (int i = 0; i < 3; ++i)
    {
        p.head[i].store(ray.head[i], std::memory_order_relaxed);
        p.dir[i].store(ray.dir[i], std::memory_order_relaxed);
        p.eye[i].store(ray.eye[i], std::memory_order_relaxed);
    }
    p.unit.store(ray.unit, std::memory_order_relaxed);
    p.ignoreB.store(ray.ignoreB, std::memory_order_relaxed);
    p.ignoreC.store(ray.ignoreC, std::memory_order_relaxed);
    for (int i = 0; i < 3; ++i)
        p.eyeVel[i].store(ray.eyeVelValid ? ray.eyeVel[i] : 0.0f, std::memory_order_relaxed);
    p.eyeVelValid.store(ray.eyeVelValid, std::memory_order_relaxed);
    p.eyeStampUs.store(ray.eyeStampUs, std::memory_order_relaxed);
    p.sequence.fetch_add(1, std::memory_order_release);
    g_stats[Slot(title)].rays.fetch_add(1, std::memory_order_relaxed);
}

bool Sight_ReadRay(GameTitle title, uint32_t generation, uint64_t maxAgeMs, SightRay& out)
{
    const auto& p = g_rays[Slot(title)];
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = p.sequence.load(std::memory_order_acquire);
        if (!before || (before & 1u))
            continue;
        SightRay r{};
        r.generation = p.generation.load(std::memory_order_relaxed);
        r.sampleMs = p.sampleMs.load(std::memory_order_relaxed);
        for (int i = 0; i < 3; ++i)
        {
            r.head[i] = p.head[i].load(std::memory_order_relaxed);
            r.dir[i] = p.dir[i].load(std::memory_order_relaxed);
            r.eye[i] = p.eye[i].load(std::memory_order_relaxed);
        }
        r.unit = p.unit.load(std::memory_order_relaxed);
        r.ignoreB = p.ignoreB.load(std::memory_order_relaxed);
        r.ignoreC = p.ignoreC.load(std::memory_order_relaxed);
        for (int i = 0; i < 3; ++i)
            r.eyeVel[i] = p.eyeVel[i].load(std::memory_order_relaxed);
        r.eyeVelValid = p.eyeVelValid.load(std::memory_order_relaxed);
        r.eyeStampUs = p.eyeStampUs.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (p.sequence.load(std::memory_order_relaxed) != before)
            continue;
        if (!generation || r.generation != generation || !Fresh(r.sampleMs, Sight_NowMs(), maxAgeMs))
            return false;
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(r.head[i]) || !std::isfinite(r.dir[i]) || !std::isfinite(r.eye[i]))
                return false;
        out = r;
        return true;
    }
    return false;
}

bool Sight_TraceDue(GameTitle title, uint64_t nowMs)
{
    if (!Sight_AimEnabled())
        return false;
    auto& t = g_traces[Slot(title)];
    const uint64_t last = t.lastTraceMs.load(std::memory_order_relaxed);
    if (last && nowMs >= last && nowMs - last < kSightTraceIntervalMs)
        return false;
    t.lastTraceMs.store(nowMs, std::memory_order_relaxed);
    return true;
}

void Sight_PublishTrace(GameTitle title, uint32_t generation, bool hit, float rangeWu, uint64_t costTicks)
{
    auto& t = g_traces[Slot(title)];
    const uint64_t now = Sight_NowMs();
    // The range the aim converges on (SightSmoothInverseRange): the inverse range averaged over 120 ms, a
    // miss counting as the trace's far end. A new generation, or a gap in the traces (a seat, a menu, a
    // stall), starts afresh from this trace.
    const uint64_t lastMs = t.smoothMs.load(std::memory_order_relaxed);
    const bool continuous = lastMs && now >= lastMs && now - lastMs <= kSightFreshMs &&
        t.smoothGeneration.load(std::memory_order_relaxed) == generation;
    const float dt = continuous ? float(now - lastMs) / 1000.0f : float(kSightTraceIntervalMs) / 1000.0f;
    const float inv = SightSmoothInverseRange(t.invRangePerWu.load(std::memory_order_relaxed), continuous,
        hit, rangeWu, dt);
    t.invRangePerWu.store(inv, std::memory_order_relaxed);
    t.smoothMs.store(now, std::memory_order_relaxed);
    t.smoothGeneration.store(generation, std::memory_order_relaxed);
    t.sequence.fetch_add(1, std::memory_order_acq_rel);
    t.generation.store(generation, std::memory_order_relaxed);
    t.sampleMs.store(now, std::memory_order_relaxed);
    t.hit.store(hit ? 1u : 0u, std::memory_order_relaxed);
    t.rangeWu.store(rangeWu, std::memory_order_relaxed);
    t.convergeRangeWu.store(inv > 0.0f ? 1.0f / inv : 0.0f, std::memory_order_relaxed);
    t.sequence.fetch_add(1, std::memory_order_release);
    auto& s = g_stats[Slot(title)];
    s.traces.fetch_add(1, std::memory_order_relaxed);
    (hit ? s.hits : s.misses).fetch_add(1, std::memory_order_relaxed);
    s.costTicks.fetch_add(costTicks, std::memory_order_relaxed);
    uint64_t prior = s.costMaxTicks.load(std::memory_order_relaxed);
    while (costTicks > prior && !s.costMaxTicks.compare_exchange_weak(prior, costTicks, std::memory_order_relaxed)) {}
}

bool Sight_ReadTrace(GameTitle title, uint32_t generation, uint64_t maxAgeMs, SightTrace& out)
{
    const auto& t = g_traces[Slot(title)];
    if (t.faulted.load(std::memory_order_acquire))
        return false;
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = t.sequence.load(std::memory_order_acquire);
        if (!before || (before & 1u))
            continue;
        SightTrace r{};
        r.generation = t.generation.load(std::memory_order_relaxed);
        r.sampleMs = t.sampleMs.load(std::memory_order_relaxed);
        r.hit = t.hit.load(std::memory_order_relaxed) != 0;
        r.rangeWu = t.rangeWu.load(std::memory_order_relaxed);
        r.convergeRangeWu = t.convergeRangeWu.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (t.sequence.load(std::memory_order_relaxed) != before)
            continue;
        if (!generation || r.generation != generation || !Fresh(r.sampleMs, Sight_NowMs(), maxAgeMs))
            return false;
        r.hit = r.hit && std::isfinite(r.rangeWu) && r.rangeWu > 0.0f;
        // A miss means the target is at least the whole trace away (the reticle parks); the aim's
        // smoothed range (convergeRangeWu) is heading for that far end too.
        if (!r.hit)
            r.rangeWu = kSightTraceLengthWu;
        if (!std::isfinite(r.convergeRangeWu) || !(r.convergeRangeWu > 0.0f))
            r.convergeRangeWu = 0.0f;
        out = r;
        return true;
    }
    return false;
}

void Sight_NoteTraceFault(GameTitle title)
{
    g_traces[Slot(title)].faulted.store(true, std::memory_order_release);
    g_stats[Slot(title)].faults.fetch_add(1, std::memory_order_relaxed);
}

bool Sight_TraceFaulted(GameTitle title)
{
    return g_traces[Slot(title)].faulted.load(std::memory_order_acquire);
}

void Sight_ResetTitle(GameTitle title)
{
    auto& t = g_traces[Slot(title)];
    t.faulted.store(false, std::memory_order_release);
    t.lastTraceMs.store(0, std::memory_order_relaxed);
    t.invRangePerWu.store(0.0f, std::memory_order_relaxed);
    t.smoothMs.store(0, std::memory_order_relaxed);
    g_converged[Slot(title)].atMs.store(0, std::memory_order_release);
    g_converged[Slot(title)].writtenMs.store(0, std::memory_order_release);
}

void Sight_MarkConverged(GameTitle title, float unitsPerVrMetre)
{
    auto& c = g_converged[Slot(title)];
    c.generation.store(TitleAdapter_GetGeneration(title), std::memory_order_relaxed);
    c.unitsPerVrMetre.store(unitsPerVrMetre, std::memory_order_relaxed);
    c.atMs.store(Sight_NowMs(), std::memory_order_release);
    g_stats[Slot(title)].converged.fetch_add(1, std::memory_order_relaxed);
}

bool Sight_ConvergedLive(GameTitle title)
{
    const auto& c = g_converged[Slot(title)];
    const uint64_t at = c.atMs.load(std::memory_order_acquire);
    return Fresh(at, Sight_NowMs(), 100) &&
           c.generation.load(std::memory_order_relaxed) == TitleAdapter_GetGeneration(title);
}

void Sight_MarkAimWritten(GameTitle title)
{
    auto& c = g_converged[Slot(title)];
    c.writtenGeneration.store(TitleAdapter_GetGeneration(title), std::memory_order_relaxed);
    c.writtenMs.store(Sight_NowMs(), std::memory_order_release);
    g_stats[Slot(title)].written.fetch_add(1, std::memory_order_relaxed);
}

bool Sight_AimWrittenLive(GameTitle title)
{
    const auto& c = g_converged[Slot(title)];
    const uint64_t at = c.writtenMs.load(std::memory_order_acquire);
    return Fresh(at, Sight_NowMs(), 100) &&
           c.writtenGeneration.load(std::memory_order_relaxed) == TitleAdapter_GetGeneration(title);
}

bool Sight_ConvergedDirection(GameTitle title, uint32_t generation, const float head[3], const float dir[3],
                              const float eye[3], float out[3], float maxPitchDeg, float* outRangeWu,
                              bool* outHit, converge::ConvergeOff* outWhy, aim_solve::RailHold* railHold)
{
    using converge::ConvergeOff;
    const auto decline = [&](ConvergeOff why) {
        if (outWhy)
            *outWhy = why;
        if (railHold)
            railHold->reset();
        return false;
    };
    SightTrace trace{};
    if (!Sight_AimEnabled())
        return decline(ConvergeOff::AimOff);
    if (!head || !dir || !eye || !out)
        return decline(ConvergeOff::RayStale);
    if (!Sight_ReadTrace(title, generation, kSightFreshMs, trace))
        return decline(Sight_TraceFaulted(title) ? ConvergeOff::TraceFaulted : ConvergeOff::TraceStale);
    if (!(trace.convergeRangeWu > 0.0f))   // no hit yet since the traces (re)started: nothing to converge on
    {
        g_stats[Slot(title)].declined.fetch_add(1, std::memory_order_relaxed);
        return decline(ConvergeOff::NoRange);
    }
    aim_solve::ConvergeRails rails{};
    rails.max_pitch_deg = maxPitchDeg;
    aim_solve::Vec3 bent{};
    if (!aim_solve::converge_direction(aim_solve::Vec3{head[0] - eye[0], head[1] - eye[1], head[2] - eye[2]},
                                       aim_solve::Vec3{dir[0], dir[1], dir[2]}, trace.convergeRangeWu,
                                       kSightCmPerWorldUnit, rails, bent, railHold))
    {
        // The geometry did not resolve: a point inside the range floor, or a degenerate ray. (A steep bend no
        // longer declines: Halo 3 / ODST / Halo 4 hold it at the rail and stay converged.)
        g_stats[Slot(title)].declined.fetch_add(1, std::memory_order_relaxed);
        return decline(ConvergeOff::NoRange);
    }
    out[0] = bent.x;
    out[1] = bent.y;
    out[2] = bent.z;
    if (outRangeWu) *outRangeWu = trace.convergeRangeWu;
    if (outHit) *outHit = trace.hit;
    return true;
}

bool Sight_MuzzleShotsConverge()
{
    return g_config.shots_from_muzzle && !g_config.gun_barrel_aim && Sight_AimEnabled();
}

void Sight_NoteMuzzleShotUnconverged(GameTitle title)
{
    g_stats[Slot(title)].muzzleUnconverged.fetch_add(1, std::memory_order_relaxed);
}

void Sight_NoteMuzzleShotHeld(GameTitle title)
{
    g_stats[Slot(title)].muzzleHeld.fetch_add(1, std::memory_order_relaxed);
}

bool Sight_AimMuzzleShotThrough(GameTitle title, const float point[3], const float rayDir[3], const float eye[3],
                                const float before[3], const float position[3], float direction[3], float up[3])
{
    if (!point || !rayDir || !eye || !before || !position || !direction || !up)
        return false;
    aim_solve::Vec3 out{}, newUp{up[0], up[1], up[2]};
    if (!aim_solve::muzzle_shot_direction(aim_solve::Vec3{before[0], before[1], before[2]},
                                          aim_solve::Vec3{position[0], position[1], position[2]},
                                          aim_solve::Vec3{point[0], point[1], point[2]},
                                          aim_solve::Vec3{rayDir[0], rayDir[1], rayDir[2]},
                                          aim_solve::Vec3{eye[0], eye[1], eye[2]}, kSightCmPerWorldUnit,
                                          aim_solve::MuzzleShotRails{}, out, newUp))
    {
        g_stats[Slot(title)].muzzleRefused.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    direction[0] = out.x; direction[1] = out.y; direction[2] = out.z;
    up[0] = newUp.x; up[1] = newUp.y; up[2] = newUp.z;
    g_stats[Slot(title)].muzzleAimed.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool Sight_AimMuzzleShotFromRay(GameTitle title, uint32_t generation, const float head[3], const float dir[3],
                                const float eye[3], const float before[3], const float position[3],
                                float direction[3], float up[3], float* pointDistance)
{
    // The point the converged aim in effect is bent through: the intent ray at the smoothed range the aim
    // path converged on (SightTrace::AimRangeWu), so the shot from the muzzle lands where the shot from the
    // engine eye would have.
    SightTrace trace{};
    if (!head || !dir || !Sight_ConvergedLive(title) || !Sight_ReadTrace(title, generation, kSightFreshMs, trace) ||
        !(trace.AimRangeWu() > 0.0f))
    {
        Sight_NoteMuzzleShotUnconverged(title);
        return false;
    }
    const float length = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (!std::isfinite(length) || length < 1.0e-3f)
    {
        Sight_NoteMuzzleShotUnconverged(title);
        return false;
    }
    const float range = trace.AimRangeWu();
    float u[3]{}, point[3]{};
    for (int axis = 0; axis < 3; ++axis)
    {
        u[axis] = dir[axis] / length;
        point[axis] = head[axis] + u[axis] * range;
    }
    if (!Sight_AimMuzzleShotThrough(title, point, u, eye, before, position, direction, up))
        return false;
    if (pointDistance)
    {
        float d2 = 0.0f;
        for (int axis = 0; axis < 3; ++axis)
            d2 += (point[axis] - position[axis]) * (point[axis] - position[axis]);
        *pointDistance = std::isfinite(d2) ? std::sqrt(d2) : 0.0f;
    }
    return true;
}

// ---- the intent point and the shot re-aims (shot_lag_logic.h) -------------------------------------------------
namespace
{
SeqBlob<SightIntent> g_intents[kTitleSlots];

struct ReaimStats
{
    std::atomic<uint64_t> applied{0}, through{0};
    std::atomic<uint64_t> refused[static_cast<size_t>(shot_lag::Reaim::Count)]{};
    std::atomic<uint64_t> skipped[static_cast<size_t>(shot_lag::IntentUse::Count)]{};
    std::atomic<uint32_t> bendSumMilliDeg{0}, bendMaxMilliDeg{0};
};
ReaimStats g_reaim[kTitleSlots][static_cast<size_t>(SightShotSite::Count)];

const char* SiteName(SightShotSite site)
{
    switch (site)
    {
    case SightShotSite::CrosshairAssist: return "crosshair-assist query";
    case SightShotSite::UnitAdjust: return "unit-adjust shot";
    case SightShotSite::Muzzle: return "muzzle shot";
    default: return "?";
    }
}
} // namespace

void Sight_PublishIntent(GameTitle title, const SightIntent& intent)
{
    if (!Sight_AimEnabled())
        return;
    g_intents[Slot(title)].Publish(intent);
}

bool Sight_ReadIntent(GameTitle title, SightIntent& out)
{
    return g_intents[Slot(title)].Read(out);
}

shot_lag::IntentUse Sight_IntentForShot(GameTitle title, uint64_t maxAgeMs, bool naturalHeld, SightIntent& out)
{
    out = SightIntent{};
    if (!Sight_AimEnabled())
        return shot_lag::IntentUse::NotConverged;
    const bool read = Sight_ReadIntent(title, out);
    return shot_lag::ShouldUseIntent(read, out.p, TitleAdapter_GetGeneration(title), Sight_NowUs(),
                                     maxAgeMs * 1000ull, Sight_ConvergedLive(title), naturalHeld);
}

bool Sight_ReaimThroughPoint(GameTitle title, SightShotSite site, const float start[3], float vector[3],
                             const float point[3])
{
    const size_t s = static_cast<size_t>(site) % static_cast<size_t>(SightShotSite::Count);
    auto& stats = g_reaim[Slot(title)][s];
    float out[3]{};
    float bend = 0.0f;
    const shot_lag::Reaim why = shot_lag::ReaimThroughPoint(start, vector, point, shot_lag::ReaimRails{}, out, &bend);
    if (why == shot_lag::Reaim::AlreadyThrough)
    {
        stats.through.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if (why != shot_lag::Reaim::Applied)
    {
        stats.refused[static_cast<size_t>(why)].fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    vector[0] = out[0];
    vector[1] = out[1];
    vector[2] = out[2];
    stats.applied.fetch_add(1, std::memory_order_relaxed);
    const uint32_t milli = static_cast<uint32_t>(std::min(bend * 1000.0f, 1.0e9f));
    stats.bendSumMilliDeg.fetch_add(milli, std::memory_order_relaxed);
    uint32_t prior = stats.bendMaxMilliDeg.load(std::memory_order_relaxed);
    while (milli > prior && !stats.bendMaxMilliDeg.compare_exchange_weak(prior, milli, std::memory_order_relaxed)) {}
    return true;
}

void Sight_NoteReaimSkipped(GameTitle title, SightShotSite site, shot_lag::IntentUse why)
{
    const size_t s = static_cast<size_t>(site) % static_cast<size_t>(SightShotSite::Count);
    const size_t w = static_cast<size_t>(why);
    if (w < static_cast<size_t>(shot_lag::IntentUse::Count))
        g_reaim[Slot(title)][s].skipped[w].fetch_add(1, std::memory_order_relaxed);
}

namespace
{
// Cold worker (Sight_Report): one line per title and site that saw a shot in the window. The muzzle site counts
// "used" for a shot that took the intent point (the muzzle's own rails then decide, as before).
void ReportReaims(size_t slot)
{
    for (size_t s = 0; s < static_cast<size_t>(SightShotSite::Count); ++s)
    {
        auto& stats = g_reaim[slot][s];
        const uint64_t applied = stats.applied.exchange(0, std::memory_order_relaxed);
        const uint64_t through = stats.through.exchange(0, std::memory_order_relaxed);
        const uint64_t used = stats.skipped[0].exchange(0, std::memory_order_relaxed);
        uint64_t refused = 0, skipped = 0;
        char reasons[256]{};
        size_t length = 0;
        const auto add = [&](const char* name, uint64_t n) {
            if (!n || length + 1 >= sizeof(reasons))
                return;
            const int w = std::snprintf(reasons + length, sizeof(reasons) - length, "%s%s=%llu", length ? ", " : "",
                                        name, static_cast<unsigned long long>(n));
            if (w > 0)
                length += std::min(static_cast<size_t>(w), sizeof(reasons) - length - 1);
        };
        for (size_t r = 1; r < static_cast<size_t>(shot_lag::Reaim::Count); ++r)
        {
            const uint64_t n = stats.refused[r].exchange(0, std::memory_order_relaxed);
            refused += n;
            add(shot_lag::ReaimName(static_cast<shot_lag::Reaim>(r)), n);
        }
        for (size_t r = 1; r < static_cast<size_t>(shot_lag::IntentUse::Count); ++r)
        {
            const uint64_t n = stats.skipped[r].exchange(0, std::memory_order_relaxed);
            skipped += n;
            add(shot_lag::IntentUseName(static_cast<shot_lag::IntentUse>(r)), n);
        }
        const uint32_t bendSum = stats.bendSumMilliDeg.exchange(0, std::memory_order_relaxed);
        const uint32_t bendMax = stats.bendMaxMilliDeg.exchange(0, std::memory_order_relaxed);
        if (!applied && !through && !refused && !skipped && !used)
            continue;
        if (static_cast<SightShotSite>(s) == SightShotSite::Muzzle)
        {
            LOG("SHOT REAIM %s %s: %llu flew through the frame-coherent intent point, %llu through the ray's point "
                "as before%s%s%s", TitleName(static_cast<GameTitle>(slot)), SiteName(static_cast<SightShotSite>(s)),
                static_cast<unsigned long long>(used), static_cast<unsigned long long>(skipped),
                length ? " (" : "", length ? reasons : "", length ? ")" : "");
            continue;
        }
        LOG("SHOT REAIM %s %s: %llu re-aimed through the intent point from the query's own start (mean %.2f deg, "
            "max %.2f), %llu already through it (left alone), %llu left as the game built them%s%s%s",
            TitleName(static_cast<GameTitle>(slot)), SiteName(static_cast<SightShotSite>(s)),
            static_cast<unsigned long long>(applied),
            applied ? double(bendSum) / 1000.0 / double(applied) : 0.0, double(bendMax) / 1000.0,
            static_cast<unsigned long long>(through), static_cast<unsigned long long>(refused + skipped),
            length ? " (" : "", length ? reasons : "", length ? ")" : "");
    }
}
} // namespace

bool Sight_AimMuzzleShot(GameTitle title, uint32_t generation, const float before[3], const float position[3],
                         float direction[3], float up[3])
{
    SightRay ray{};
    if (!Sight_ReadRay(title, generation, kSightRayFreshMs, ray))
    {
        Sight_NoteMuzzleShotUnconverged(title);
        return false;
    }
    // THE INTENT POINT FIRST (2026-10-02): the point built from one frame-coherent hand/camera pair, through which
    // the shot flies from this muzzle -- the ray's own head may be a camera a frame older than its hand. Without a
    // usable intent, the ray's point as before (Sight_AimMuzzleShotFromRay keeps every gate and count).
    SightIntent intent{};
    const shot_lag::IntentUse use = Sight_IntentForShot(title, kSightIntentShotMaxAgeMs, false, intent);
    Sight_NoteReaimSkipped(title, SightShotSite::Muzzle, use);
    // A seat flies through the point its record was bent through (a seat's line is only ever corrected, never
    // re-targeted: SightIntent::solvePoint); on foot through the frame-coherent point.
    if (use == shot_lag::IntentUse::Use && intent.p.generation == generation)
        return Sight_AimMuzzleShotThrough(title, intent.p.seated ? intent.solvePoint : intent.p.point, intent.p.dir,
                                          ray.eye, before, position, direction, up);
    return Sight_AimMuzzleShotFromRay(title, generation, ray.head, ray.dir, ray.eye, before, position, direction, up);
}

bool Sight_ScopeConvergedLive(GameTitle title)
{
    return g_config.scope_converge && Sight_AimEnabled() && Sight_ConvergedLive(title) &&
           Sight_AimWrittenLive(title);
}

bool Sight_ScopeConvergedDirection(GameTitle title, float outDir[3])
{
    const uint32_t generation = TitleAdapter_GetGeneration(title);
    SightRay ray{};
    if (!outDir || !generation || !Sight_ScopeConvergedLive(title) ||
        !Sight_ReadRay(title, generation, kSightRayFreshMs, ray))
        return false;
    const float length = std::sqrt(ray.dir[0] * ray.dir[0] + ray.dir[1] * ray.dir[1] + ray.dir[2] * ray.dir[2]);
    if (!std::isfinite(length) || length < 1.0e-3f)
        return false;
    for (int axis = 0; axis < 3; ++axis)
        outDir[axis] = ray.dir[axis] / length;
    return true;
}

bool Game_SightReticleRange(float& outRangeM, bool& outHit)
{
    outRangeM = 0.0f;
    outHit = false;
    const GameTitle title = TitleAdapter_GetActiveTitle();
    if (title == GameTitle::HaloReach || title == GameTitle::None || title == GameTitle::Unknown)
        return false;
    if (!Sight_ConvergedLive(title))
        return false;
    const uint32_t generation = TitleAdapter_GetGeneration(title);
    SightTrace trace{};
    if (!Sight_ReadTrace(title, generation, kSightFreshMs, trace))
        return false;
    const float scale = g_converged[Slot(title)].unitsPerVrMetre.load(std::memory_order_relaxed);
    if (!std::isfinite(scale) || !(scale > 0.0f))
        return false;
    outHit = trace.hit;
    outRangeM = trace.hit ? trace.rangeWu / scale : 0.0f;
    return true;
}

#if HALOMCCVR_DEV_DIAGNOSTICS
namespace
{
// The shot witness's per-title sums since the last report (mm), and the last shot's origin offsets.
struct ShotWitness
{
    // One lock keeps a shot's sums and its count in the same report window: NoteShot adds both under it and
    // ReportShots takes both under it. Without it a shot straddling a report split across two windows, and a
    // window whose count was 0 dropped the sums and logged the next window's shot as a perfect +0 mm.
    SRWLOCK lock = SRWLOCK_INIT;
    std::atomic<uint32_t> count{0}, unmeasured{0};
    std::atomic<float> surfaceUp{0.0f}, surfaceRight{0.0f}, pointUp{0.0f}, pointRight{0.0f};
    std::atomic<float> surfaceM{0.0f}, pointM{0.0f};
    std::atomic<float> fromEye[3]{}, fromHead[3]{};
};
ShotWitness g_shots[kTitleSlots];
constexpr float kMmPerWu = 3048.0f;

void AddRelaxed(std::atomic<float>& total, float value)
{
    float current = total.load(std::memory_order_relaxed);
    while (!total.compare_exchange_weak(current, current + value, std::memory_order_relaxed))
    {
    }
}

struct EngineRayEntry
{
    std::atomic<uint32_t> rva{0}, profile{0};
    std::atomic<uint64_t> flags{0};
    std::atomic<uint32_t> count{0}, aligned{0};
    std::atomic<float> lengthM{0.0f}, startMm{0.0f}, angleDeg{0.0f}, missMm{0.0f}, rangeM{0.0f};
};
constexpr size_t kEngineRaySlots = 48;
EngineRayEntry g_engineRays[kTitleSlots][kEngineRaySlots];
std::atomic<uint32_t> g_engineRayTitles{0};
constexpr float kMetresPerWu = 3.048f;
} // namespace
#endif

void Sight_NoteEngineRay(GameTitle title, uint32_t generation, uint32_t callerRva, uint32_t profile,
                         uint64_t flags, const float start[3], const float end[3])
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (!callerRva || !start || !end)
        return;
    const size_t slot = Slot(title);
    g_engineRayTitles.fetch_or(1u << slot, std::memory_order_relaxed);
    EngineRayEntry* entry = nullptr;
    for (auto& candidate : g_engineRays[slot])
    {
        uint32_t expected = 0;
        if (candidate.rva.load(std::memory_order_acquire) == callerRva ||
            candidate.rva.compare_exchange_strong(expected, callerRva, std::memory_order_acq_rel) ||
            expected == callerRva)
        {
            entry = &candidate;
            break;
        }
    }
    if (!entry)
        return;
    entry->count.fetch_add(1, std::memory_order_relaxed);
    entry->profile.store(profile, std::memory_order_relaxed);
    entry->flags.store(flags, std::memory_order_relaxed);
    const float v[3]{end[0] - start[0], end[1] - start[1], end[2] - start[2]};
    const float length = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!std::isfinite(length) || length < 1.0e-6f)
        return;
    entry->lengthM.store(length * kMetresPerWu, std::memory_order_relaxed);
    SightRay ray{};
    SightTrace trace{};
    if (!Sight_ReadRay(title, generation, kSightRayFreshMs, ray) ||
        !Sight_ReadTrace(title, generation, kSightFreshMs, trace))
        return;
    const float aimRange = trace.AimRangeWu();
    const float p[3]{ray.head[0] + ray.dir[0] * aimRange, ray.head[1] + ray.dir[1] * aimRange,
                     ray.head[2] + ray.dir[2] * aimRange};
    const float a[3]{p[0] - ray.eye[0], p[1] - ray.eye[1], p[2] - ray.eye[2]};
    const float al = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (!(al > 1.0e-6f))
        return;
    float dot = (a[0] * v[0] + a[1] * v[1] + a[2] * v[2]) / (al * length);
    dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
    const float angle = std::acos(dot) * 57.29578f;
    const float s[3]{start[0] - ray.eye[0], start[1] - ray.eye[1], start[2] - ray.eye[2]};
    const float startMm = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]) * kMetresPerWu * 1000.0f;
    if (startMm >= 300.0f || angle >= 2.0f)
        return;
    const float d[3]{v[0] / length, v[1] / length, v[2] / length};
    const float w[3]{p[0] - start[0], p[1] - start[1], p[2] - start[2]};
    const float c[3]{w[1] * d[2] - w[2] * d[1], w[2] * d[0] - w[0] * d[2], w[0] * d[1] - w[1] * d[0]};
    entry->aligned.fetch_add(1, std::memory_order_relaxed);
    entry->startMm.store(startMm, std::memory_order_relaxed);
    entry->angleDeg.store(angle, std::memory_order_relaxed);
    entry->missMm.store(std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) * kMetresPerWu * 1000.0f,
                        std::memory_order_relaxed);
    entry->rangeM.store(std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * kMetresPerWu,
                        std::memory_order_relaxed);
#else
    (void)title; (void)generation; (void)callerRva; (void)profile; (void)flags; (void)start; (void)end;
#endif
}

void Sight_NoteShot(GameTitle title, uint32_t generation, const float origin[3], const float direction[3])
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    auto& w = g_shots[Slot(title)];
    SightRay ray{};
    SightTrace trace{};
    const float length = origin && direction
        ? std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
        : 0.0f;
    if (!std::isfinite(length) || length < 1.0e-6f || !Sight_ReadRay(title, generation, kSightRayFreshMs, ray) ||
        !Sight_ReadTrace(title, generation, kSightFreshMs, trace))
    {
        w.unmeasured.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    for (int axis = 0; axis < 3; ++axis)
        if (!std::isfinite(origin[axis]))
        {
            w.unmeasured.fetch_add(1, std::memory_order_relaxed);
            return;
        }
    // The intent ray's own frame: forward along it, right level (Blam is Z-up), up perpendicular to both.
    const float f[3]{ray.dir[0], ray.dir[1], ray.dir[2]};
    float right[3]{f[1], -f[0], 0.0f};
    const float rightLength = std::sqrt(right[0] * right[0] + right[1] * right[1]);
    if (!std::isfinite(rightLength) || rightLength < 1.0e-3f)   // straight up or down: no level right
    {
        w.unmeasured.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    right[0] /= rightLength;
    right[1] /= rightLength;
    const float up[3]{right[1] * f[2] - right[2] * f[1], right[2] * f[0] - right[0] * f[2],
                      right[0] * f[1] - right[1] * f[0]};
    const float u[3]{direction[0] / length, direction[1] / length, direction[2] / length};
    const auto dot = [](const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    // Where the shot's line passes the point `range` along the intent ray, measured from that point.
    const auto passes = [&](float range, float& outUp, float& outRight) {
        const float point[3]{ray.head[0] + f[0] * range, ray.head[1] + f[1] * range, ray.head[2] + f[2] * range};
        const float toPoint[3]{point[0] - origin[0], point[1] - origin[1], point[2] - origin[2]};
        const float t = dot(toPoint, u);
        const float miss[3]{origin[0] + u[0] * t - point[0], origin[1] + u[1] * t - point[1],
                            origin[2] + u[2] * t - point[2]};
        outUp = dot(miss, up) * kMmPerWu;
        outRight = dot(miss, right) * kMmPerWu;
    };
    float surfaceUp = 0.0f, surfaceRight = 0.0f, pointUp = 0.0f, pointRight = 0.0f;
    passes(trace.rangeWu, surfaceUp, surfaceRight);
    passes(trace.AimRangeWu(), pointUp, pointRight);
    if (!std::isfinite(surfaceUp) || !std::isfinite(surfaceRight) || !std::isfinite(pointUp) ||
        !std::isfinite(pointRight))
    {
        w.unmeasured.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const float fromEye[3]{origin[0] - ray.eye[0], origin[1] - ray.eye[1], origin[2] - ray.eye[2]};
    const float fromHead[3]{origin[0] - ray.head[0], origin[1] - ray.head[1], origin[2] - ray.head[2]};
    AcquireSRWLockExclusive(&w.lock);
    AddRelaxed(w.surfaceUp, surfaceUp);
    AddRelaxed(w.surfaceRight, surfaceRight);
    AddRelaxed(w.pointUp, pointUp);
    AddRelaxed(w.pointRight, pointRight);
    w.surfaceM.store(trace.rangeWu * kSightCmPerWorldUnit * 0.01f, std::memory_order_relaxed);
    w.pointM.store(trace.AimRangeWu() * kSightCmPerWorldUnit * 0.01f, std::memory_order_relaxed);
    w.fromEye[0].store(dot(fromEye, right) * kMmPerWu, std::memory_order_relaxed);
    w.fromEye[1].store(dot(fromEye, up) * kMmPerWu, std::memory_order_relaxed);
    w.fromEye[2].store(dot(fromEye, f) * kMmPerWu, std::memory_order_relaxed);
    w.fromHead[0].store(dot(fromHead, right) * kMmPerWu, std::memory_order_relaxed);
    w.fromHead[1].store(dot(fromHead, up) * kMmPerWu, std::memory_order_relaxed);
    w.fromHead[2].store(dot(fromHead, f) * kMmPerWu, std::memory_order_relaxed);
    w.count.fetch_add(1, std::memory_order_relaxed);
    ReleaseSRWLockExclusive(&w.lock);
#else
    (void)title; (void)generation; (void)origin; (void)direction;
#endif
}

#if HALOMCCVR_DEV_DIAGNOSTICS
namespace
{
std::atomic<uint32_t> g_intentPairing[kTitleSlots][4]{};
}
#endif

void Sight_NoteIntentPairing(GameTitle title, int kind)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (kind >= 0 && kind < 4)
        g_intentPairing[Slot(title)][kind].fetch_add(1, std::memory_order_relaxed);
#else
    (void)title; (void)kind;
#endif
}
void Sight_ReportShots()
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    static uint64_t s_last = 0;
    const uint64_t now = GetTickCount64();
    if (now - s_last < 2000)
        return;
    s_last = now;
    for (size_t slot = 0; slot < kTitleSlots; ++slot)
    {
        auto& w = g_shots[slot];
        // Take the window whole (see ShotWitness::lock), last-shot fields included.
        AcquireSRWLockExclusive(&w.lock);
        const uint32_t count = w.count.exchange(0, std::memory_order_relaxed);
        const uint32_t unmeasured = w.unmeasured.exchange(0, std::memory_order_relaxed);
        const float surfaceUp = w.surfaceUp.exchange(0.0f, std::memory_order_relaxed);
        const float surfaceRight = w.surfaceRight.exchange(0.0f, std::memory_order_relaxed);
        const float pointUp = w.pointUp.exchange(0.0f, std::memory_order_relaxed);
        const float pointRight = w.pointRight.exchange(0.0f, std::memory_order_relaxed);
        const float surfaceM = w.surfaceM.load(std::memory_order_relaxed);
        const float pointM = w.pointM.load(std::memory_order_relaxed);
        float fromEye[3], fromHead[3];
        for (int axis = 0; axis < 3; ++axis)
        {
            fromEye[axis] = w.fromEye[axis].load(std::memory_order_relaxed);
            fromHead[axis] = w.fromHead[axis].load(std::memory_order_relaxed);
        }
        ReleaseSRWLockExclusive(&w.lock);
        if (!count && !unmeasured)
            continue;
        const auto title = static_cast<GameTitle>(slot);
        if (!count)
        {
            LOG("SHOT %s: %u shot(s), none measurable (no fresh intent ray or trace, or aimed straight up/down)",
                TitleName(title), unmeasured);
            continue;
        }
        const float n = static_cast<float>(count);
        LOG("SHOT %s: %u shot(s) (+%u unmeasured) pass %+.0f mm up / %+.0f mm right of the surface under the "
            "reticle (%.2f m) and %+.0f / %+.0f mm of the converged point (%.2f m), on average; the last left "
            "%+.0f/%+.0f/%+.0f mm (right/up/along) from the engine eye and %+.0f/%+.0f/%+.0f mm from the "
            "rendered eye", TitleName(title), count, unmeasured, double(surfaceUp / n), double(surfaceRight / n),
            double(surfaceM), double(pointUp / n), double(pointRight / n), double(pointM), double(fromEye[0]),
            double(fromEye[1]), double(fromEye[2]), double(fromHead[0]), double(fromHead[1]),
            double(fromHead[2]));
    }
    // SHOT LAG, per title and path.
    for (size_t slot = 0; slot < kTitleSlots; ++slot)
    {
        uint32_t pairing[4]{};
        for (int k = 0; k < 4; ++k)
            pairing[k] = g_intentPairing[slot][k].exchange(0, std::memory_order_relaxed);
        if (pairing[0] || pairing[1] || pairing[2] || pairing[3])
            LOG("SHOT LAG %s intent point (dev): %u from this poll's hand and the render of its own frame, %u from an "
                "earlier hand and the render of its frame (this poll's frame not rendered yet), %u from the solve's "
                "own ray (no kept hand had a rendered frame), %u without camera frames",
                TitleName(static_cast<GameTitle>(slot)), pairing[0], pairing[1], pairing[2], pairing[3]);
    }
#endif
}

void Sight_ReportEngineRays()
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    static uint64_t s_last = 0;
    const uint64_t now = GetTickCount64();
    if (now - s_last < 5000)
        return;
    s_last = now;
    const uint32_t titles = g_engineRayTitles.load(std::memory_order_relaxed);
    for (size_t slot = 0; slot < kTitleSlots; ++slot)
    {
        if (!(titles & (1u << slot)))
            continue;
        const auto title = static_cast<GameTitle>(slot);
        uint32_t callers = 0, queries = 0;
        for (auto& entry : g_engineRays[slot])
        {
            const uint32_t rva = entry.rva.load(std::memory_order_acquire);
            if (!rva)
                continue;
            const uint32_t count = entry.count.exchange(0, std::memory_order_relaxed);
            const uint32_t aligned = entry.aligned.exchange(0, std::memory_order_relaxed);
            if (count)
            {
                ++callers;
                queries += count;
            }
            if (aligned)
                LOG("SIGHT RAYS %s +0x%X profile 0x%X flags 0x%llX: %u queries, %u ALIGNED (start %.0f mm from the "
                    "eye, %.2f deg from eye->point; the traced point %.0f mm off this line at %.2f m), %.1f m long",
                    TitleName(title), rva, entry.profile.load(), (unsigned long long)entry.flags.load(), count,
                    aligned, double(entry.startMm.load()), double(entry.angleDeg.load()),
                    double(entry.missMm.load()), double(entry.rangeM.load()), double(entry.lengthM.load()));
        }
        LOG("SIGHT RAYS %s: %u callers, %u queries in 5 s", TitleName(title), callers, queries);
    }
#endif
}

void Sight_Report()
{
    static uint64_t s_last = 0;
    const uint64_t now = GetTickCount64();
    if (now - s_last < 2000)
        return;
    s_last = now;
    LARGE_INTEGER freq{};
    QueryPerformanceFrequency(&freq);
    const double usPerTick = freq.QuadPart ? 1.0e6 / double(freq.QuadPart) : 0.0;
    for (size_t i = 0; i < kTitleSlots; ++i)
    {
        auto& s = g_stats[i];
        const uint64_t rays = s.rays.exchange(0, std::memory_order_relaxed);
        const uint64_t traces = s.traces.exchange(0, std::memory_order_relaxed);
        const uint64_t converged = s.converged.exchange(0, std::memory_order_relaxed);
        const uint64_t written = s.written.exchange(0, std::memory_order_relaxed);
        const uint64_t declined = s.declined.exchange(0, std::memory_order_relaxed);
        const uint64_t faults = s.faults.exchange(0, std::memory_order_relaxed);
        const uint64_t muzzleAimed = s.muzzleAimed.exchange(0, std::memory_order_relaxed);
        const uint64_t muzzleRefused = s.muzzleRefused.exchange(0, std::memory_order_relaxed);
        const uint64_t muzzleUnconverged = s.muzzleUnconverged.exchange(0, std::memory_order_relaxed);
        const uint64_t muzzleHeld = s.muzzleHeld.exchange(0, std::memory_order_relaxed);
        ReportReaims(i);
        if (muzzleAimed || muzzleRefused || muzzleUnconverged || muzzleHeld)
            LOG("SHOTS FROM THE MUZZLE %s: %llu aimed through the converged point, %llu refused by a rail (the "
                "muzzle far from the eye, pulled back by the obstruction check, or the point not ahead of it), "
                "%llu without a converged point, %llu held off (a seat held at its stop, or a turret: the gun "
                "points elsewhere); every refusal left from the eye as before",
                TitleName(static_cast<GameTitle>(i)), (unsigned long long)muzzleAimed,
                (unsigned long long)muzzleRefused, (unsigned long long)muzzleUnconverged,
                (unsigned long long)muzzleHeld);
        if (!rays && !traces && !converged && !declined && !faults)
            continue;
        const uint64_t hits = s.hits.exchange(0, std::memory_order_relaxed);
        const uint64_t misses = s.misses.exchange(0, std::memory_order_relaxed);
        const uint64_t cost = s.costTicks.exchange(0, std::memory_order_relaxed);
        const uint64_t costMax = s.costMaxTicks.exchange(0, std::memory_order_relaxed);
        const auto title = static_cast<GameTitle>(i);
        const float convergeWu = g_traces[i].convergeRangeWu.load(std::memory_order_relaxed);
        LOG("SIGHT %s: rays %llu, traces %llu (%llu hits, %llu misses [= the far end]), cost avg %.1f / "
            "max %.1f us; converged aim %llu polls (written, not steered: %llu), declined %llu; converging at "
            "%.2f m (smoothed); faulted=%d%s",
            TitleName(title), (unsigned long long)rays, (unsigned long long)traces, (unsigned long long)hits,
            (unsigned long long)misses, traces ? double(cost) / double(traces) * usPerTick : 0.0,
            double(costMax) * usPerTick, (unsigned long long)converged, (unsigned long long)written,
            (unsigned long long)declined,
            double(convergeWu) * double(kSightCmPerWorldUnit) * 0.01, Sight_TraceFaulted(title) ? 1 : 0,
            faults ? " (a native fault latched the trace off)" : "");
    }
}

// ---- the converge-off witness and the natural-cause counters (dev lines only; converge_policy.h) ----------
#if HALOMCCVR_DEV_DIAGNOSTICS
namespace
{
converge::ConvergeOffWitness g_convergeOff[kTitleSlots];
converge::NaturalWitness g_natural[kTitleSlots];
std::atomic<uint64_t> g_naturalHeldMs[kTitleSlots]{};
} // namespace
#endif

void Sight_NoteConvergeState(GameTitle title, bool live, converge::ConvergeOff why)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    g_convergeOff[Slot(title)].note(live, why);
#else
    (void)title; (void)live; (void)why;
#endif
}

void Sight_NoteNatural(GameTitle title, const converge::NaturalEvents& events)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    g_natural[Slot(title)].note(events);
    if (converge::NaturalHoldsAimOff(events))
        g_naturalHeldMs[Slot(title)].store(Sight_NowMs(), std::memory_order_relaxed);
#else
    (void)title; (void)events;
#endif
}

bool Sight_NaturalHeld(GameTitle title, uint64_t windowMs)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    return Fresh(g_naturalHeldMs[Slot(title)].load(std::memory_order_relaxed), Sight_NowMs(), windowMs);
#else
    (void)title; (void)windowMs;
    return false;
#endif
}

void Sight_ReportConverge()
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    using converge::ConvergeOff;
    static converge::ConvergeEdgeLog s_edges[kTitleSlots];
    static uint64_t s_last = 0;
    const uint64_t now = GetTickCount64();
    const auto reasonName = [](ConvergeOff why) {
        return why == ConvergeOff::None ? "Unclassified" : converge::ConvergeOffName(why);
    };
    // EDGES, every loop of the cold worker (a byte load per title; a line only on a change).
    for (size_t slot = 0; slot < kTitleSlots; ++slot)
    {
        uint8_t previous = converge::kConvergeStateNone;
        const uint8_t current = g_convergeOff[slot].current();
        if (!s_edges[slot].observe(current, now, previous))
            continue;
        const auto title = static_cast<GameTitle>(slot);
        if (converge::ConvergeStateIsOff(current))
            LOG("SIGHT %s converge-off: %s%s", TitleName(title), reasonName(converge::ConvergeStateReason(current)),
                previous == converge::kConvergeStateLive ? " (it was on)" : "");
        else
            LOG("SIGHT %s converge back on (it was off: %s)", TitleName(title),
                reasonName(converge::ConvergeStateReason(previous)));
    }
    if (now - s_last < 2000)
        return;
    s_last = now;
    // SUMMARIES, every 2 s per title: why it was off (our code's reasons only), and the natural causes it stayed
    // on through.
    for (size_t slot = 0; slot < kTitleSlots; ++slot)
    {
        const auto title = static_cast<GameTitle>(slot);
        const converge::ConvergeOffCounts c = g_convergeOff[slot].take();
        const uint32_t suppressed = s_edges[slot].take_suppressed();
        // Off by configuration (F1 Converged aim, reach_direct_drive, reach_vehicle_aim) the whole window: the edge
        // line already said so.
        const bool onlyKeyOff = c.off() == c.why[static_cast<size_t>(ConvergeOff::AimOff)] && !c.offEdges &&
                                !c.onEdges;
        if ((c.off() || c.offEdges || c.onEdges) && !onlyKeyOff)
        {
            char reasons[512]{};
            size_t used = 0;
            for (size_t i = 0; i < static_cast<size_t>(ConvergeOff::Count) && used < sizeof(reasons); ++i)
            {
                if (!c.why[i])
                    continue;
                const int n = std::snprintf(reasons + used, sizeof(reasons) - used, "%s%s=%u", used ? ", " : "",
                                            reasonName(static_cast<ConvergeOff>(i)), c.why[i]);
                if (n <= 0)
                    break;
                used += static_cast<size_t>(n);
            }
            LOG("SIGHT %s converge-off %u/%u polls: %s (went off %u times, back on %u%s)", TitleName(title),
                c.off(), c.polls, used ? reasons : "none", c.offEdges, c.onEdges,
                suppressed ? "; some edge lines suppressed" : "");
        }
        const converge::NaturalCounts n = g_natural[slot].take();
        if (n.latchHeld || n.outcomeMiss || n.stall || n.turretSlew || n.reachLimited || n.yawShortH4)
            LOG("SIGHT %s natural (no switch-off) in %u polls: latchHeld=%u outcomeMiss=%u stall=%u turret=%u "
                "reachLimited=%u yawShortH4=%u (none of these switches the converged aim off)",
                TitleName(title), n.polls, n.latchHeld, n.outcomeMiss, n.stall, n.turretSlew, n.reachLimited,
                n.yawShortH4);
    }
#endif
}

#if HALOMCCVR_DEV_DIAGNOSTICS
// ---- DEV ASSESSMENT RAY and its trace (sight_layer.h). Seqlocks like the intent ray's: one writer each (the aim
// path, the title's trace thread), any reader. Slots of their own, so nothing the aim or the scope reads moves.
namespace
{
struct AssessRayPublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint64_t> sampleMs{0};
    std::atomic<float> origin[3]{};
    std::atomic<float> dir[3]{};
};
struct AssessTracePublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint64_t> sampleMs{0};
    std::atomic<uint32_t> hit{0};
    std::atomic<float> rangeWu{0.0f};
};
AssessRayPublication g_assessRays[kTitleSlots];
AssessTracePublication g_assessTraces[kTitleSlots];
} // namespace

void Sight_PublishAssessRay(GameTitle title, const SightAssessRay& ray)
{
    auto& p = g_assessRays[Slot(title)];
    p.sequence.fetch_add(1, std::memory_order_acq_rel);
    p.generation.store(ray.generation, std::memory_order_relaxed);
    p.sampleMs.store(ray.sampleMs, std::memory_order_relaxed);
    for (int i = 0; i < 3; ++i)
    {
        p.origin[i].store(ray.origin[i], std::memory_order_relaxed);
        p.dir[i].store(ray.dir[i], std::memory_order_relaxed);
    }
    p.sequence.fetch_add(1, std::memory_order_release);
}

bool Sight_ReadAssessRay(GameTitle title, uint32_t generation, uint64_t maxAgeMs, SightAssessRay& out)
{
    const auto& p = g_assessRays[Slot(title)];
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = p.sequence.load(std::memory_order_acquire);
        if (!before || (before & 1u))
            continue;
        SightAssessRay r{};
        r.generation = p.generation.load(std::memory_order_relaxed);
        r.sampleMs = p.sampleMs.load(std::memory_order_relaxed);
        for (int i = 0; i < 3; ++i)
        {
            r.origin[i] = p.origin[i].load(std::memory_order_relaxed);
            r.dir[i] = p.dir[i].load(std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if (p.sequence.load(std::memory_order_relaxed) != before)
            continue;
        if (!generation || r.generation != generation || !Fresh(r.sampleMs, Sight_NowMs(), maxAgeMs))
            return false;
        const float length = std::sqrt(r.dir[0] * r.dir[0] + r.dir[1] * r.dir[1] + r.dir[2] * r.dir[2]);
        if (!std::isfinite(length) || std::fabs(length - 1.0f) > 1.0e-2f)
            return false;
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(r.origin[i]))
                return false;
        out = r;
        return true;
    }
    return false;
}

void Sight_PublishAssessTrace(GameTitle title, uint32_t generation, bool hit, float rangeWu)
{
    auto& t = g_assessTraces[Slot(title)];
    t.sequence.fetch_add(1, std::memory_order_acq_rel);
    t.generation.store(generation, std::memory_order_relaxed);
    t.sampleMs.store(Sight_NowMs(), std::memory_order_relaxed);
    t.hit.store(hit ? 1u : 0u, std::memory_order_relaxed);
    t.rangeWu.store(rangeWu, std::memory_order_relaxed);
    t.sequence.fetch_add(1, std::memory_order_release);
}

bool Sight_ReadAssessTrace(GameTitle title, uint32_t generation, uint64_t maxAgeMs, bool& hit, float& rangeWu)
{
    const auto& t = g_assessTraces[Slot(title)];
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = t.sequence.load(std::memory_order_acquire);
        if (!before || (before & 1u))
            continue;
        const uint32_t g = t.generation.load(std::memory_order_relaxed);
        const uint64_t at = t.sampleMs.load(std::memory_order_relaxed);
        bool h = t.hit.load(std::memory_order_relaxed) != 0;
        float r = t.rangeWu.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (t.sequence.load(std::memory_order_relaxed) != before)
            continue;
        if (!generation || g != generation || !Fresh(at, Sight_NowMs(), maxAgeMs))
            return false;
        h = h && std::isfinite(r) && r > 0.0f;
        if (!h)
            r = kSightTraceLengthWu;   // a miss: the shot meets nothing within the whole trace
        hit = h;
        rangeWu = r;
        return true;
    }
    return false;
}
#endif
