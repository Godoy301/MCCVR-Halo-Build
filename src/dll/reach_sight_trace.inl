// Included in game.cpp (anonymous namespace) before reach_contact_melee_runtime.inl.
//
// TRACE-AND-REAIM for Halo Reach on foot -- the Halo: Campaign Evolved aim.
//
// Reach fires first-person shots from its own camera (the engine eye). The player points along the
// rendered intent ray (the controller/reticle ray). A direction alone cannot make a shot from the eye
// land on what that ray points at; a direction plus a RANGE can:
//
//      T   = H + u * d                the point the intent ray hits (H origin, u direction)
//      aim = normalize(T - E)         the direction that puts a shot from the engine eye E through it
//          = normalize((H - E) + u * d)
//
// d is MEASURED by tracing the intent ray with Reach's own collision_test_vector, on the simulation
// thread, from inside the per-unit update the contact-melee lane already hooks (a native context in
// which the game itself runs collision queries). The input-thread aim provider reads the published
// range, smooths it, and bends the direct-drive setpoint (src/common/aim_solve_logic.h does the math).
// While a converged setpoint is live, the older on-foot hand shot ray -- which MOVED the projectile's
// origin to the hand -- stands down, so the shot leaves from exactly where the game fires it.
//
// The trace flags are the engine's own collision-test flags (reach_sight_trace_flags; 0 = off). The
// optional census below logs the flags Reach's own callers use, which is how they were identified.

struct ReachSightTracePublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint64_t> sampleMs{0};
    std::atomic<uint32_t> hit{0};
    std::atomic<uint32_t> hitType{0};
    std::atomic<float> rangeWu{0.0f};
    std::atomic<bool> faulted{false};
    std::atomic<uint64_t> traces{0}, hits{0}, misses{0}, skips{0};
    std::atomic<uint32_t> nearestMm{UINT32_MAX}, farthestMm{0};
} g_reachSightTrace;

// Beyond ~60 wu (~180 m) the convergence correction is a small fraction of a degree.
inline constexpr float kReachSightTraceLengthWu = 60.0f;
// Start a few centimetres down the ray so the trace clears the gun in the hand; the range reported
// is still measured from the ray's origin.
inline constexpr float kReachSightTraceSkipWu = 0.05f;

std::atomic<uint64_t> g_reachConvergedShotMs{0};

bool ReachConvergedShotLive(uint64_t nowMs)
{
    const uint64_t at = g_reachConvergedShotMs.load(std::memory_order_acquire);
    return at && nowMs >= at && nowMs - at <= 250;
}

void ReachSightTraceTick(uint32_t unit)
{
    const uint64_t flags = g_config.reach_sight_trace_flags;
    auto& trace = g_reachSightTrace;
    if (!flags || trace.faulted.load(std::memory_order_acquire) ||
        !g_reachCamera.armed.load(std::memory_order_acquire) ||
        g_reachWorldCollision.fourArgumentResolver || !ReachContactMeleeReady())
        return;
    const int32_t localUnit = LegacyCollisionIgnoredObject(GameTitle::HaloReach);
    if (localUnit == -1 || unit != static_cast<uint32_t>(localUnit))
        return;
    const auto original = reinterpret_cast<LegacyCollisionTestVectorFn>(g_reachWorldCollision.original);
    if (!original)
        return;
    const uint32_t generation = g_reachCamera.generation.load(std::memory_order_acquire);
    const uint64_t now = GetTickCount64();
    ReachCompletedReticleTargetSnapshot ray{};
    if (!ReachReadCompletedReticleTarget(ray, g_reachOnFootShotRay) || !generation ||
        ray.generation != generation || !ray.sampleMs || now < ray.sampleMs || now - ray.sampleMs > 100 ||
        ray.key.unitHandle != localUnit)
    {
        trace.skips.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    float u[3]{ray.target[0] - ray.origin[0], ray.target[1] - ray.origin[1], ray.target[2] - ray.origin[2]};
    const float length = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (!std::isfinite(length) || length < 1.0e-3f)
    {
        trace.skips.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    for (float& c : u) c /= length;
    const float start[3]{ray.origin[0] + u[0] * kReachSightTraceSkipWu,
                         ray.origin[1] + u[1] * kReachSightTraceSkipWu,
                         ray.origin[2] + u[2] * kReachSightTraceSkipWu};
    const float vector[3]{u[0] * kReachSightTraceLengthWu, u[1] * kReachSightTraceLengthWu,
                          u[2] * kReachSightTraceLengthWu};
    alignas(16) uint8_t result[0x400]{};
    uint8_t returned = 0;
    // Our query calls the unhooked entry; anything it reaches that is hooked must not schedule
    // world-collision work of its own from inside it.
    const bool previousOwned = g_legacyCollisionOwnedQuery;
    g_legacyCollisionOwnedQuery = true;
    __try
    {
        returned = original(flags, g_config.reach_sight_trace_mode, start, vector, localUnit, -1, -1, result);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        g_legacyCollisionOwnedQuery = previousOwned;
        trace.faulted.store(true, std::memory_order_release);   // fail closed: the old aim comes back
        return;
    }
    g_legacyCollisionOwnedQuery = previousOwned;
    trace.traces.fetch_add(1, std::memory_order_relaxed);
    uint32_t type = 0;
    float fraction = 1.0f;
    memcpy(&type, result, sizeof(type));
    memcpy(&fraction, result + 4, sizeof(fraction));
    const bool hit = returned && std::isfinite(fraction) && fraction >= 0.0f && fraction <= 1.0f;
    const float rangeWu = hit ? kReachSightTraceSkipWu + fraction * kReachSightTraceLengthWu : 0.0f;
    (hit ? trace.hits : trace.misses).fetch_add(1, std::memory_order_relaxed);
    if (hit)
    {
        const uint32_t mm = static_cast<uint32_t>(rangeWu / kReachWorldUnitsPerMeter * 1000.0f);
        uint32_t prior = trace.nearestMm.load(std::memory_order_relaxed);
        while (mm < prior && !trace.nearestMm.compare_exchange_weak(prior, mm, std::memory_order_relaxed)) {}
        prior = trace.farthestMm.load(std::memory_order_relaxed);
        while (mm > prior && !trace.farthestMm.compare_exchange_weak(prior, mm, std::memory_order_relaxed)) {}
    }
    trace.sequence.fetch_add(1, std::memory_order_acq_rel);
    trace.generation.store(generation, std::memory_order_relaxed);
    trace.sampleMs.store(now, std::memory_order_relaxed);
    trace.hit.store(hit ? 1u : 0u, std::memory_order_relaxed);
    trace.hitType.store(type, std::memory_order_relaxed);
    trace.rangeWu.store(rangeWu, std::memory_order_relaxed);
    trace.sequence.fetch_add(1, std::memory_order_release);
}

// ---- shot check: does the native shot actually pass through the traced point? ---------------------
struct ReachSightShotStats
{
    std::atomic<uint32_t> shots{0}, measured{0};
    std::atomic<uint32_t> missSumMm{0}, missMaxMm{0};
    std::atomic<uint32_t> originOffsetMaxMm{0};
    std::atomic<uint32_t> lastRangeMm{0}, lastMissMm{0};
} g_reachSightShots;

void ReachSightMeasureShot(int32_t unitIndex, const float* origin, const float* direction)
{
    auto& stats = g_reachSightShots;
    stats.shots.fetch_add(1, std::memory_order_relaxed);
    if (!origin || !direction)
        return;
    ReachCompletedReticleTargetSnapshot ray{};
    const uint64_t now = GetTickCount64();
    if (!ReachReadCompletedReticleTarget(ray, g_reachOnFootShotRay) || ray.key.unitHandle != unitIndex ||
        !ray.sampleMs || now < ray.sampleMs || now - ray.sampleMs > 150)
        return;
    const uint32_t before = g_reachSightTrace.sequence.load(std::memory_order_acquire);
    if (!before || (before & 1u))
        return;
    const bool hit = g_reachSightTrace.hit.load(std::memory_order_relaxed) != 0;
    const float rangeWu = g_reachSightTrace.rangeWu.load(std::memory_order_relaxed);
    if (g_reachSightTrace.sequence.load(std::memory_order_acquire) != before || !hit ||
        !std::isfinite(rangeWu) || rangeWu <= 0.0f)
        return;
    float u[3]{ray.target[0] - ray.origin[0], ray.target[1] - ray.origin[1], ray.target[2] - ray.origin[2]};
    const float ul = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    const float dl = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    if (!std::isfinite(ul) || ul < 1.0e-3f || !std::isfinite(dl) || dl < 1.0e-6f)
        return;
    // Distance from the traced point to the actual shot line (the central direction, before spread).
    const float target[3]{ray.origin[0] + u[0] / ul * rangeWu, ray.origin[1] + u[1] / ul * rangeWu,
                          ray.origin[2] + u[2] / ul * rangeWu};
    const float w[3]{target[0] - origin[0], target[1] - origin[1], target[2] - origin[2]};
    const float d[3]{direction[0] / dl, direction[1] / dl, direction[2] / dl};
    const float c[3]{w[1] * d[2] - w[2] * d[1], w[2] * d[0] - w[0] * d[2], w[0] * d[1] - w[1] * d[0]};
    const float missWu = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    const float offWu = std::sqrt((origin[0] - ray.base[0]) * (origin[0] - ray.base[0]) +
                                  (origin[1] - ray.base[1]) * (origin[1] - ray.base[1]) +
                                  (origin[2] - ray.base[2]) * (origin[2] - ray.base[2]));
    if (!std::isfinite(missWu) || !std::isfinite(offWu))
        return;
    const auto mm = [](float wu) { return static_cast<uint32_t>(std::min(wu / kReachWorldUnitsPerMeter * 1000.0f, 4.0e9f)); };
    const uint32_t missMm = mm(missWu), offMm = mm(offWu);
    stats.measured.fetch_add(1, std::memory_order_relaxed);
    stats.missSumMm.fetch_add(missMm, std::memory_order_relaxed);
    stats.lastMissMm.store(missMm, std::memory_order_relaxed);
    stats.lastRangeMm.store(mm(rangeWu), std::memory_order_relaxed);
    uint32_t prior = stats.missMaxMm.load(std::memory_order_relaxed);
    while (missMm > prior && !stats.missMaxMm.compare_exchange_weak(prior, missMm, std::memory_order_relaxed)) {}
    prior = stats.originOffsetMaxMm.load(std::memory_order_relaxed);
    while (offMm > prior && !stats.originOffsetMaxMm.compare_exchange_weak(prior, offMm, std::memory_order_relaxed)) {}
}

// ---- census (diagnostic): which flags do Reach's own collision_test_vector callers use? ------------
struct ReachCollisionCensusEntry
{
    uint32_t rva = 0;
    int32_t mode = 0;
    uint64_t flags = 0;
    uint32_t count = 0;
    uint32_t ownerIgnored = 0;   // calls whose first ignored object is the local unit
    float maxLengthWu = 0.0f;
    bool reported = false;
};
inline constexpr uint32_t kReachCollisionCensusSlots = 48;
struct ReachCollisionCensus
{
    std::atomic_flag lock = ATOMIC_FLAG_INIT;
    ReachCollisionCensusEntry entries[kReachCollisionCensusSlots]{};
    uint32_t used = 0;
    uint32_t dropped = 0;
} g_reachCollisionCensus;

void ReachNoteCollisionCensus(uintptr_t caller, uint64_t flags, int32_t mode, const float* vector, int32_t ignoredA)
{
    if (!g_config.reach_collision_census)
        return;
    const uintptr_t base = g_reachCamera.base;
    const uint32_t rva = (base && caller >= base && caller - base < 0x10000000u) ? static_cast<uint32_t>(caller - base) : 0;
    float length = 0.0f;
    if (vector)
    {
        length = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
        if (!std::isfinite(length)) length = -1.0f;
    }
    const int32_t localUnit = g_reachCamera.playerUnitByOutputUser ? LegacyCollisionIgnoredObject(GameTitle::HaloReach) : -1;
    auto& census = g_reachCollisionCensus;
    for (int spin = 0; census.lock.test_and_set(std::memory_order_acquire); ++spin)
        if (spin > 64) return;   // never make an engine thread wait on a diagnostic
    ReachCollisionCensusEntry* entry = nullptr;
    for (uint32_t i = 0; i < census.used; ++i)
        if (census.entries[i].rva == rva && census.entries[i].flags == flags && census.entries[i].mode == mode)
        { entry = &census.entries[i]; break; }
    if (!entry && census.used < kReachCollisionCensusSlots)
    {
        entry = &census.entries[census.used++];
        *entry = {};
        entry->rva = rva;
        entry->flags = flags;
        entry->mode = mode;
    }
    if (entry)
    {
        ++entry->count;
        if (localUnit != -1 && ignoredA == localUnit) ++entry->ownerIgnored;
        if (length > entry->maxLengthWu) entry->maxLengthWu = length;
    }
    else
        ++census.dropped;
    census.lock.clear(std::memory_order_release);
}

void ReportReachSightTrace()
{
    auto& trace = g_reachSightTrace;
    auto& shots = g_reachSightShots;
    const uint32_t leashFrames = g_reachLeashHeldFrames.exchange(0, std::memory_order_relaxed);
    const uint32_t leashMax = g_reachLeashMaxLiftMm.exchange(0, std::memory_order_relaxed);
    if (leashFrames)
        LOG("Reach leash below the character's eye: view held on %u frames, max lift %u mm (limit %.2f m)",
            leashFrames, leashMax, double(g_config.head_leash_below_eye_m));
    if (g_config.reach_sight_trace_flags || trace.faulted.load(std::memory_order_acquire))
    {
        const uint32_t nearest = trace.nearestMm.exchange(UINT32_MAX, std::memory_order_relaxed);
        const uint32_t measured = shots.measured.exchange(0, std::memory_order_relaxed);
        const uint32_t missSum = shots.missSumMm.exchange(0, std::memory_order_relaxed);
        LOG("REACHSIGHT trace: flags=0x%llX mode=%d %s; %llu traces, %llu hits / %llu misses, %llu skipped; "
            "range last %.2f m (nearest %.2f / farthest %.2f m), type %u; converged-shot live=%d; "
            "shots %u (%u measured): miss at the traced point avg %u / max %u mm, last %u mm at %.2f m; "
            "native origin vs engine eye max %u mm",
            g_config.reach_sight_trace_flags, g_config.reach_sight_trace_mode,
            trace.faulted.load(std::memory_order_acquire) ? "FAULTED (off)" : "ok",
            (unsigned long long)trace.traces.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.hits.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.misses.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.skips.exchange(0, std::memory_order_relaxed),
            double(trace.rangeWu.load(std::memory_order_relaxed) / kReachWorldUnitsPerMeter),
            nearest == UINT32_MAX ? 0.0 : nearest / 1000.0,
            trace.farthestMm.exchange(0, std::memory_order_relaxed) / 1000.0,
            trace.hitType.load(std::memory_order_relaxed),
            ReachConvergedShotLive(GetTickCount64()) ? 1 : 0,
            shots.shots.exchange(0, std::memory_order_relaxed), measured,
            measured ? missSum / measured : 0u, shots.missMaxMm.exchange(0, std::memory_order_relaxed),
            shots.lastMissMm.load(std::memory_order_relaxed),
            shots.lastRangeMm.load(std::memory_order_relaxed) / 1000.0,
            shots.originOffsetMaxMm.exchange(0, std::memory_order_relaxed));
    }
    if (!g_config.reach_collision_census)
        return;
    auto& census = g_reachCollisionCensus;
    ReachCollisionCensusEntry copy[kReachCollisionCensusSlots]{};
    uint32_t used = 0, dropped = 0;
    for (int spin = 0; census.lock.test_and_set(std::memory_order_acquire); ++spin)
        if (spin > 100000) return;
    used = census.used;
    dropped = census.dropped;
    memcpy(copy, census.entries, sizeof(copy));
    for (uint32_t i = 0; i < census.used; ++i) census.entries[i].reported = true;
    census.lock.clear(std::memory_order_release);
    for (uint32_t i = 0; i < used; ++i)
        LOG("CENSUS collision_test_vector caller=+0x%X flags=0x%llX mode=%d count=%u ownerIgnored=%u maxLen=%.2f wu%s",
            copy[i].rva, static_cast<unsigned long long>(copy[i].flags), copy[i].mode, copy[i].count,
            copy[i].ownerIgnored, double(copy[i].maxLengthWu), copy[i].reported ? "" : " (NEW)");
    if (dropped)
        LOG("CENSUS collision_test_vector: %u calls not recorded (table full)", dropped);
}
