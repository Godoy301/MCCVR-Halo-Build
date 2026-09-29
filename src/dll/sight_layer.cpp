// The title-agnostic sight layer (see sight_layer.h). Every publication is a seqlock with one writer
// thread per title: the ray from the aim path, the trace result from the title's simulation hook.

#include "sight_layer.h"
#include "title_adapter.h"
#include "../common/aim_solve_logic.h"
#include "../common/log.h"

#include <windows.h>
#include <atomic>
#include <cmath>

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
};

struct TracePublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint64_t> sampleMs{0};
    std::atomic<uint32_t> hit{0};
    std::atomic<float> rangeWu{0.0f};
    std::atomic<bool> faulted{false};
    std::atomic<uint64_t> lastTraceMs{0};
};

struct ConvergedStamp
{
    std::atomic<uint64_t> atMs{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<float> unitsPerVrMetre{0.0f};
};

struct Stats
{
    std::atomic<uint64_t> rays{0}, traces{0}, hits{0}, misses{0}, faults{0}, converged{0}, declined{0};
    std::atomic<uint64_t> costTicks{0}, costMaxTicks{0};
};

RayPublication g_rays[kTitleSlots];
TracePublication g_traces[kTitleSlots];
ConvergedStamp g_converged[kTitleSlots];
Stats g_stats[kTitleSlots];

size_t Slot(GameTitle title) { return static_cast<size_t>(title) % kTitleSlots; }

bool Fresh(uint64_t at, uint64_t now, uint64_t maxAge) { return at && now >= at && now - at <= maxAge; }

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

void Sight_PublishRay(GameTitle title, const SightRay& ray)
{
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
        if (p.sequence.load(std::memory_order_acquire) != before)
            continue;
        if (!generation || r.generation != generation || !Fresh(r.sampleMs, GetTickCount64(), maxAgeMs))
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
    t.sequence.fetch_add(1, std::memory_order_acq_rel);
    t.generation.store(generation, std::memory_order_relaxed);
    t.sampleMs.store(GetTickCount64(), std::memory_order_relaxed);
    t.hit.store(hit ? 1u : 0u, std::memory_order_relaxed);
    t.rangeWu.store(rangeWu, std::memory_order_relaxed);
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
        if (t.sequence.load(std::memory_order_acquire) != before)
            continue;
        if (!generation || r.generation != generation || !Fresh(r.sampleMs, GetTickCount64(), maxAgeMs))
            return false;
        r.hit = r.hit && std::isfinite(r.rangeWu) && r.rangeWu > 0.0f;
        // A miss means the target is at least the whole trace away: converge on the trace's far end.
        if (!r.hit)
            r.rangeWu = kSightTraceLengthWu;
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
    g_converged[Slot(title)].atMs.store(0, std::memory_order_release);
}

void Sight_MarkConverged(GameTitle title, float unitsPerVrMetre)
{
    auto& c = g_converged[Slot(title)];
    c.generation.store(TitleAdapter_GetGeneration(title), std::memory_order_relaxed);
    c.unitsPerVrMetre.store(unitsPerVrMetre, std::memory_order_relaxed);
    c.atMs.store(GetTickCount64(), std::memory_order_release);
    g_stats[Slot(title)].converged.fetch_add(1, std::memory_order_relaxed);
}

bool Sight_ConvergedLive(GameTitle title)
{
    const auto& c = g_converged[Slot(title)];
    const uint64_t at = c.atMs.load(std::memory_order_acquire);
    return Fresh(at, GetTickCount64(), 100) &&
           c.generation.load(std::memory_order_relaxed) == TitleAdapter_GetGeneration(title);
}

bool Sight_ConvergedDirection(GameTitle title, uint32_t generation, const float head[3], const float dir[3],
                              const float eye[3], float out[3], float* outRangeWu, bool* outHit)
{
    SightTrace trace{};
    if (!head || !dir || !eye || !out || !Sight_ReadTrace(title, generation, kSightFreshMs, trace))
        return false;
    aim_solve::ConvergeRails rails{};
    aim_solve::Vec3 bent{};
    if (!aim_solve::converge_direction(aim_solve::Vec3{head[0] - eye[0], head[1] - eye[1], head[2] - eye[2]},
                                       aim_solve::Vec3{dir[0], dir[1], dir[2]}, trace.rangeWu,
                                       kSightCmPerWorldUnit, rails, bent))
    {
        g_stats[Slot(title)].declined.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    out[0] = bent.x;
    out[1] = bent.y;
    out[2] = bent.z;
    if (outRangeWu) *outRangeWu = trace.rangeWu;
    if (outHit) *outHit = trace.hit;
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
    const float p[3]{ray.head[0] + ray.dir[0] * trace.rangeWu, ray.head[1] + ray.dir[1] * trace.rangeWu,
                     ray.head[2] + ray.dir[2] * trace.rangeWu};
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
        const uint64_t declined = s.declined.exchange(0, std::memory_order_relaxed);
        const uint64_t faults = s.faults.exchange(0, std::memory_order_relaxed);
        if (!rays && !traces && !converged && !declined && !faults)
            continue;
        const uint64_t hits = s.hits.exchange(0, std::memory_order_relaxed);
        const uint64_t misses = s.misses.exchange(0, std::memory_order_relaxed);
        const uint64_t cost = s.costTicks.exchange(0, std::memory_order_relaxed);
        const uint64_t costMax = s.costMaxTicks.exchange(0, std::memory_order_relaxed);
        const auto title = static_cast<GameTitle>(i);
        LOG("SIGHT %s: rays %llu, traces %llu (%llu hits, %llu misses [far end]), cost avg %.1f / max %.1f us; "
            "converged aim %llu polls, declined %llu; faulted=%d%s",
            TitleName(title), (unsigned long long)rays, (unsigned long long)traces, (unsigned long long)hits,
            (unsigned long long)misses, traces ? double(cost) / double(traces) * usPerTick : 0.0,
            double(costMax) * usPerTick, (unsigned long long)converged, (unsigned long long)declined,
            Sight_TraceFaulted(title) ? 1 : 0, faults ? " (a native fault latched the trace off)" : "");
    }
}
