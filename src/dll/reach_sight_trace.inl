// Included in game.cpp (anonymous namespace) before reach_contact_melee_runtime.inl.
//
// TRACE-AND-REAIM for Halo Reach on foot -- the Halo: Campaign Evolved aim.
//
// The player points along the rendered intent ray (the controller/reticle ray). Reach decides where a
// shot goes from its own camera: on the fire tick it traces the unit's aim ray 128 wu into the world
// (HREK aim_assist.cpp:986, aim_projectile_internal_result) and sends the projectile from its native
// origin at the point that trace hit. A direction alone cannot put that point where the intent ray
// points; a direction plus a RANGE can:
//
//      T   = H + u * d                the point the intent ray hits (H origin, u direction)
//      aim = normalize(T - E)         the direction whose ray from the engine eye E passes through T
//          = normalize((H - E) + u * d)
//
// d is MEASURED by tracing the intent ray with Reach's own collision_test_vector and the engine's own
// crosshair-trace flags (below), on the simulation thread, from inside the per-unit update the contact-
// melee lane already hooks (a native context in which the game itself runs collision queries). The
// input-thread aim provider reads the published range, smooths it, and bends the direct-drive setpoint
// (src/common/aim_solve_logic.h does the math). The engine's crosshair trace then finds T and the shot
// leaves from exactly where the game fires it. While a converged setpoint is live, the older on-foot
// hand shot ray -- which MOVED the projectile's origin to the hand -- stands down.

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
    // Why a tick did not trace (reported each window, so "0 traces" always names its cause).
    std::atomic<uint64_t> blockedUnresolved{0}, blockedFaulted{0}, blockedIdle{0}, blockedUnhooked{0},
        blockedUnarmed{0}, blockedRate{0};
    std::atomic<uint32_t> nearestMm{UINT32_MAX}, farthestMm{0};
#if HALOMCCVR_DEV_DIAGNOSTICS
    std::atomic<uint64_t> traceTicksSum{0}, traceTicksMax{0}, timed{0};
    std::atomic<uint32_t> objectHits{0};   // type 4: bipeds, vehicles, crates, scenery ...
#endif
} g_reachSightTrace;

// Beyond ~60 wu (~180 m) the convergence correction is a small fraction of a degree.
inline constexpr float kReachSightTraceLengthWu = 60.0f;
// Start 5 cm down the ray so the trace clears the gun in the hand; the range reported is still
// measured from the ray's origin.
inline constexpr float kReachSightTraceSkipWu = 0.05f * kReachWorldUnitsPerMeter;
// The range is smoothed over 120 ms in the provider, so ~30 Hz measures it fully.
inline constexpr uint64_t kReachSightTraceIntervalMs = 30;

// True while the angles the direct-drive hook last wrote into desired_angles were converged ones,
// written within the last 100 ms (the hook runs every sim tick). Recorded by the hook itself on the
// sim thread (g_reachWrittenConvergedMs), which is the thread the firing site runs on, so the stand-
// down follows the write that is actually in effect -- a decline or a release clears it at once.
bool ReachConvergedShotLive(uint64_t nowMs)
{
    const uint64_t at = g_reachWrittenConvergedMs.load(std::memory_order_acquire);
    return at && nowMs >= at && nowMs - at <= 100;
}

#if HALOMCCVR_DEV_DIAGNOSTICS
// The exact sample + smoothed range the provider last actuated as a converged aim, so the shot check
// measures the solver against what it aimed, not against a later sample (dev builds only).
struct ReachSightActuatedPublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint64_t> atMs{0};
    std::atomic<float> ox{0}, oy{0}, oz{0}, dx{0}, dy{0}, dz{0}, ex{0}, ey{0}, ez{0}, rangeWu{0};
} g_reachSightActuated;
#endif

// ---- the trace flags: RESOLVED from this build and VALUE-CHECKED, never baked in ------------------
//
// Reach's own crosshair trace (HREK reach_tag_test.exe: weapon fire -> aim_assist.cpp:986 ->
// collision_test_vector) passes
//     flags = [collision-test flags global] | 0x202000,   second argument 0,   ignoring the unit's root.
// The global is set once, at startup, by a CRT initializer, to 0x0000000100024809: collision flags
// 0x24809 = structure, instanced geometry, front faces, ignore invisible surfaces, try to keep the
// location valid; object flags 1 = "objects" with no type bits, which collision_test_vector widens to
// EVERY object type -- bipeds, vehicles, crates, scenery. 0x202000 adds ignore-two-sided surfaces and
// bit 21 (unnamed in HREK). A range measured with these flags is the range the engine converges its
// own shot on. (The flags the game issues ~100x a second, 0x2985000xD809, are AI line of sight: their
// object mask omits bipeds, so a ray traced with them passes straight through enemies.)
//
// Neither the value nor the address is trusted from one build. At install, on the cold path, each rung
// fails CLOSED (the trace stays off and the provider keeps the fixed-distance steer):
//   1. ANCHOR  `or ecx, 0x202000` (81 C9 00 20 20 00) must occur exactly once in .text. It does in HREK
//              reach_tag_test, reach_tag_play and the retail build.
//   2. GLOBAL  the nearest `mov rax, [rip+disp32]` before it must address an aligned qword inside the
//              image, with `mov ecx, eax` between them and `shr rax, 32` after (the flags' two halves).
//   3. CALLEE  a `call rel32` within 0x100 bytes after it must reach the collision_test_vector this
//              build's install already signature-matched at its HREK-mapped address and hooked.
//   4. VALUE   the global must hold exactly HREK's initial value (ValueAgreement). A patch that re-meant
//              a bit, or moved the load to a neighbouring global, fails here.
// The value is re-read by the cold worker every report; any change withdraws the flags.
// ADDR-HYGIENE: resolved + guarded. Both constants are HREK measurements (the CRT initializer that
// builds the global, reach_tag_test.exe 0x17B90, and the `or` at aim_assist 0x4E3C29); the retail
// module is only the match target (docs/EDITING-KIT-EVIDENCE.md).
inline constexpr uint64_t kReachCollisionTestFlagsInit = 0x0000000100024809ull;
inline constexpr uint32_t kReachAimTraceFlagsOr = 0x00202000u;
inline constexpr int32_t kReachAimTraceMode = 0;

struct ReachSightFlagsState
{
    std::atomic<uint64_t> flags{0};                        // 0 = unresolved: the trace stays off
    std::atomic<uintptr_t> global{0};                      // the value-checked global (re-read each report)
    std::atomic<const char*> why{"Reach is not installed"};   // why it is unresolved (static strings)
    std::atomic<uint32_t> anchorRva{0}, globalRva{0};
} g_reachSightFlags;

// The provider asks for a sample on every input poll while direct drive may converge; the sim thread
// traces only while someone is asking, so a trace nobody consumes never costs the sim thread anything.
std::atomic<uint64_t> g_reachSightDemandMs{0};
inline constexpr uint64_t kReachSightDemandFreshMs = 250;

// Guarded qword read (kept free of C++ objects so __try is legal).
bool ReachGuardedReadQword(uintptr_t address, uint64_t& value)
{
    __try
    {
        value = *reinterpret_cast<const volatile uint64_t*>(address);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Rungs 2-3 on the code around the anchor. nullptr = both hold (global set); otherwise the reason.
const char* ReachInspectAimTraceSite(uintptr_t anchor, uintptr_t textBegin, uintptr_t textEnd,
                                     uintptr_t base, size_t size, uintptr_t collisionTest, uintptr_t& global)
{
    global = 0;
    __try
    {
        const uint8_t* code = reinterpret_cast<const uint8_t*>(anchor);
        const uint8_t* load = nullptr;
        for (uintptr_t back = 7; back <= 0x40 && anchor - back >= textBegin && !load; ++back)
        {
            const uint8_t* p = code - back;
            if (p[0] == 0x48 && p[1] == 0x8B && p[2] == 0x05)
                load = p;
        }
        if (!load)
            return "no `mov rax, [rip+disp32]` in the 64 bytes before the anchor";
        bool split = false;
        for (const uint8_t* p = load + 7; p + 1 < code && !split; ++p)
            split = p[0] == 0x8B && p[1] == 0xC8;
        if (!split)
            return "no `mov ecx, eax` between the flags load and the anchor";
        int32_t disp = 0;
        memcpy(&disp, load + 3, sizeof(disp));
        const uintptr_t target = reinterpret_cast<uintptr_t>(load) + 7 + static_cast<intptr_t>(disp);
        if (size < sizeof(uint64_t) || target < base || target > base + size - sizeof(uint64_t) || (target & 7u))
            return "the flags load does not address an aligned qword inside the module";
        bool high = false;
        for (uintptr_t fwd = 6; fwd <= 0x40 && anchor + fwd + 4 <= textEnd && !high; ++fwd)
            high = code[fwd] == 0x48 && code[fwd + 1] == 0xC1 && code[fwd + 2] == 0xE8 && code[fwd + 3] == 0x20;
        if (!high)
            return "no `shr rax, 32` after the anchor";
        bool calls = false;
        for (uintptr_t fwd = 6; fwd <= 0x100 && anchor + fwd + 5 <= textEnd && !calls; ++fwd)
        {
            if (code[fwd] != 0xE8)
                continue;
            int32_t rel = 0;
            memcpy(&rel, code + fwd + 1, sizeof(rel));
            calls = anchor + fwd + 5 + static_cast<intptr_t>(rel) == collisionTest;
        }
        if (!calls)
            return "the anchor's function does not call the hooked collision_test_vector";
        global = target;
        return nullptr;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return "a fault while reading the anchor's code";
    }
}

void ReachClearSightFlags(const char* why)
{
    g_reachSightFlags.flags.store(0, std::memory_order_release);
    g_reachSightFlags.global.store(0, std::memory_order_release);
    g_reachSightFlags.anchorRva.store(0, std::memory_order_relaxed);
    g_reachSightFlags.globalRva.store(0, std::memory_order_relaxed);
    g_reachSightFlags.why.store(why, std::memory_order_release);
}

// Install path (cold, once per Reach install, after the collision hook). Clears first, so a failed
// resolve on a new build never inherits an earlier answer.
void ReachResolveSightFlags(uintptr_t base, size_t size)
{
    ReachClearSightFlags("resolving");
    const uintptr_t collisionTest = reinterpret_cast<uintptr_t>(g_reachWorldCollision.target);
    static constexpr char kAnchor[] = "81 C9 00 20 20 00";   // or ecx, 0x202000
    uintptr_t textBegin = 0, textEnd = 0, anchor = 0, global = 0;
    uint32_t stamp = 0;
    size_t imageSize = 0;
    uint64_t value = 0;
    const char* why = nullptr;
    if (!g_reachWorldCollision.installed.load(std::memory_order_acquire) ||
        !g_reachWorldCollision.originalIsVectorTest || !collisionTest)
        why = "collision_test_vector is not hooked on this build";
    else if (!ResolveOdstTextRange(base, size, textBegin, textEnd, stamp, imageSize))
        why = "the module has no .text section";
    else if (!(anchor = sig::Find(textBegin, textEnd - textBegin, kAnchor)))
        why = "the anchor `or ecx, 0x202000` is missing";
    else if (sig::Find(anchor + 1, textEnd - anchor - 1, kAnchor))
        why = "the anchor `or ecx, 0x202000` is ambiguous";
    else if ((why = ReachInspectAimTraceSite(anchor, textBegin, textEnd, base, size, collisionTest, global)) != nullptr)
        ;
    else if (!ReachGuardedReadQword(global, value))
        why = "the flags global is unreadable";
    else if (value != kReachCollisionTestFlagsInit)
        why = "the flags global disagrees with HREK's initial value";
    if (why)
    {
        ReachClearSightFlags(why);
        LOG("REACHSIGHT aim-trace flags NOT resolved: %s (anchor +0x%llX, global +0x%llX = 0x%llX, HREK 0x%llX); "
            "the aim trace stays off and aim keeps the fixed-distance steer",
            why, anchor ? static_cast<unsigned long long>(anchor - base) : 0ull,
            global ? static_cast<unsigned long long>(global - base) : 0ull, static_cast<unsigned long long>(value),
            static_cast<unsigned long long>(kReachCollisionTestFlagsInit));
        return;
    }
    const uint64_t flags = value | kReachAimTraceFlagsOr;
    g_reachSightFlags.anchorRva.store(static_cast<uint32_t>(anchor - base), std::memory_order_relaxed);
    g_reachSightFlags.globalRva.store(static_cast<uint32_t>(global - base), std::memory_order_relaxed);
    g_reachSightFlags.global.store(global, std::memory_order_release);
    g_reachSightFlags.why.store(nullptr, std::memory_order_release);
    g_reachSightFlags.flags.store(flags, std::memory_order_release);
    LOG("REACHSIGHT aim-trace flags resolved: anchor +0x%llX, global +0x%llX = 0x%llX (agrees with HREK), its "
        "function calls collision_test_vector +0x%llX; tracing with 0x%llX, second byte %d (the engine's own "
        "crosshair trace: every object type, bipeds included)",
        static_cast<unsigned long long>(anchor - base), static_cast<unsigned long long>(global - base),
        static_cast<unsigned long long>(value), static_cast<unsigned long long>(collisionTest - base),
        static_cast<unsigned long long>(flags), kReachAimTraceMode);
}

// The flags this tick traces with (0 = off). Resolved flags, or in a dev build an explicit override.
uint64_t ReachSightActiveFlags(int32_t& mode)
{
    mode = kReachAimTraceMode;
    uint64_t flags = g_reachSightFlags.flags.load(std::memory_order_acquire);
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (g_config.reach_sight_trace_flags)   // an experiment; logged at load as a DEV OVERRIDE
    {
        flags = g_config.reach_sight_trace_flags;
        mode = g_config.reach_sight_trace_mode;
    }
    if (g_config.reach_sight_fault & 2)      // fault injection: behave as if the resolve failed
        flags = 0;
#endif
    return flags;
}

void ReachSightTraceTick(uint32_t unit)
{
    auto& trace = g_reachSightTrace;
    // Every unit comes through this hook each tick, so the cheap gates come first.
    const uint64_t now = GetTickCount64();
    const uint64_t demand = g_reachSightDemandMs.load(std::memory_order_relaxed);
    if (!demand || now < demand || now - demand > kReachSightDemandFreshMs)
    {
        trace.blockedIdle.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    int32_t mode = 0;
    const uint64_t flags = ReachSightActiveFlags(mode);
    if (!flags)
    {
        trace.blockedUnresolved.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (trace.faulted.load(std::memory_order_acquire))
    {
        trace.blockedFaulted.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    // The call below is the 8-argument collision_test_vector trampoline -- proven by the install that
    // hooked the scheduler entry, not inferred -- and contact melee must be live (its per-unit hook is
    // the one we run inside, and its teardown drains before `original` is cleared).
    if (!g_reachWorldCollision.installed.load(std::memory_order_acquire) ||
        !g_reachWorldCollision.originalIsVectorTest || !ReachContactMeleeReady())
    {
        trace.blockedUnhooked.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (!g_reachCamera.armed.load(std::memory_order_acquire))
    {
        trace.blockedUnarmed.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    // Rate-limit first, then find the local unit through a cache so a busy level does not cost a native
    // lookup per unit.
    static uint64_t s_lastTraceMs = 0, s_localAtMs = 0;
    static int32_t s_localUnit = -1;
    if (s_lastTraceMs && now >= s_lastTraceMs && now - s_lastTraceMs < kReachSightTraceIntervalMs)
    {
        trace.blockedRate.fetch_add(1, std::memory_order_relaxed);
        return;
    }
#if HALOMCCVR_DEV_DIAGNOSTICS
    // Fault injection (dev): prove the fallbacks run. Bit 0 = behave as if the native trace faulted
    // (off for the rest of the Reach install); bit 1 (in ReachSightActiveFlags) = as if the flags never
    // resolved.
    if (g_config.reach_sight_fault & 1)
    {
        trace.faulted.store(true, std::memory_order_release);
        return;
    }
#endif
    if (!s_localAtMs || now < s_localAtMs || now - s_localAtMs > 100)
    {
        s_localUnit = LegacyCollisionIgnoredObject(GameTitle::HaloReach);
        s_localAtMs = now;
    }
    const int32_t localUnit = s_localUnit;
    if (localUnit == -1 || unit != static_cast<uint32_t>(localUnit))
        return;
    s_lastTraceMs = now;
    const auto original = reinterpret_cast<LegacyCollisionTestVectorFn>(g_reachWorldCollision.original);
    if (!original)
    {
        trace.blockedUnhooked.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint32_t generation = g_reachCamera.generation.load(std::memory_order_acquire);
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
    // Result: type (int32 +0) and fraction (float +4), the layout HREK's collisions.cpp asserts name and
    // this repo's Reach contact-melee path already reads; the buffer is generously sized.
    alignas(16) uint8_t result[0x400]{};
    uint8_t returned = 0;
    // Our query calls the unhooked entry; anything it reaches that is hooked must not schedule
    // world-collision work of its own from inside it.
    const bool previousOwned = g_legacyCollisionOwnedQuery;
    g_legacyCollisionOwnedQuery = true;
#if HALOMCCVR_DEV_DIAGNOSTICS
    LARGE_INTEGER qpcStart{};
    QueryPerformanceCounter(&qpcStart);
#endif
    __try
    {
        // Ignored object: the local unit, as the engine's crosshair trace ignores the unit's root (on
        // foot, the unit itself).
        returned = original(flags, mode, start, vector, localUnit, -1, -1, result);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        g_legacyCollisionOwnedQuery = previousOwned;
        trace.faulted.store(true, std::memory_order_release);   // fail closed: the old aim comes back
        return;
    }
    g_legacyCollisionOwnedQuery = previousOwned;
#if HALOMCCVR_DEV_DIAGNOSTICS
    LARGE_INTEGER qpcEnd{};
    QueryPerformanceCounter(&qpcEnd);
    const uint64_t spent = static_cast<uint64_t>(qpcEnd.QuadPart - qpcStart.QuadPart);
    trace.traceTicksSum.fetch_add(spent, std::memory_order_relaxed);
    trace.timed.fetch_add(1, std::memory_order_relaxed);
    uint64_t priorTicks = trace.traceTicksMax.load(std::memory_order_relaxed);
    while (spent > priorTicks && !trace.traceTicksMax.compare_exchange_weak(priorTicks, spent, std::memory_order_relaxed)) {}
#endif
    trace.traces.fetch_add(1, std::memory_order_relaxed);
    uint32_t type = 0;
    float fraction = 1.0f;
    memcpy(&type, result, sizeof(type));
    memcpy(&fraction, result + 4, sizeof(fraction));
    const bool hit = returned && std::isfinite(fraction) && fraction >= 0.0f && fraction <= 1.0f;
    const float rangeWu = hit ? kReachSightTraceSkipWu + fraction * kReachSightTraceLengthWu : 0.0f;
    (hit ? trace.hits : trace.misses).fetch_add(1, std::memory_order_relaxed);
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (hit && type == 4)
        trace.objectHits.fetch_add(1, std::memory_order_relaxed);
#endif
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

// ---- shot check (dev): does the native shot actually pass through the traced point? ---------------
#if HALOMCCVR_DEV_DIAGNOSTICS
struct ReachSightShotStats
{
    std::atomic<uint32_t> shots{0}, measured{0};
    std::atomic<uint32_t> missSumMm{0}, missMaxMm{0};
    std::atomic<uint32_t> originOffsetMaxMm{0};
    std::atomic<uint32_t> lastRangeMm{0}, lastMissMm{0};
} g_reachSightShots;
#endif

void ReachSightMeasureShot(int32_t unitIndex, const float* origin, const float* direction)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    auto& stats = g_reachSightShots;
    stats.shots.fetch_add(1, std::memory_order_relaxed);
    (void)unitIndex;
    if (!origin || !direction)
        return;
    // What the provider AIMED at: the ray, eye and smoothed range it actuated (not a later sample).
    auto& a = g_reachSightActuated;
    const uint64_t now = GetTickCount64();
    float o[3]{}, u[3]{}, e[3]{}, rangeWu = 0.0f;
    uint64_t atMs = 0;
    bool coherent = false;
    for (int attempt = 0; attempt < 2 && !coherent; ++attempt)
    {
        const uint32_t before = a.sequence.load(std::memory_order_acquire);
        if (!before || (before & 1u)) continue;
        atMs = a.atMs.load(std::memory_order_relaxed);
        o[0] = a.ox.load(std::memory_order_relaxed); o[1] = a.oy.load(std::memory_order_relaxed); o[2] = a.oz.load(std::memory_order_relaxed);
        u[0] = a.dx.load(std::memory_order_relaxed); u[1] = a.dy.load(std::memory_order_relaxed); u[2] = a.dz.load(std::memory_order_relaxed);
        e[0] = a.ex.load(std::memory_order_relaxed); e[1] = a.ey.load(std::memory_order_relaxed); e[2] = a.ez.load(std::memory_order_relaxed);
        rangeWu = a.rangeWu.load(std::memory_order_relaxed);
        coherent = a.sequence.load(std::memory_order_acquire) == before;
    }
    if (!coherent || !atMs || now < atMs || now - atMs > 150 || !std::isfinite(rangeWu) || rangeWu <= 0.0f)
        return;
    const float dl = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    if (!std::isfinite(dl) || dl < 1.0e-6f)
        return;
    // Distance from the aimed point to the actual shot line (the central direction, before spread; it
    // may include Reach's own autoaim bend, which is native behaviour, not solver error).
    const float target[3]{o[0] + u[0] * rangeWu, o[1] + u[1] * rangeWu, o[2] + u[2] * rangeWu};
    const float w[3]{target[0] - origin[0], target[1] - origin[1], target[2] - origin[2]};
    const float d[3]{direction[0] / dl, direction[1] / dl, direction[2] / dl};
    const float c[3]{w[1] * d[2] - w[2] * d[1], w[2] * d[0] - w[0] * d[2], w[0] * d[1] - w[1] * d[0]};
    const float missWu = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    // The native origin's offset from the eye the solver assumed, PERPENDICULAR to the shot (an
    // along-ray barrel offset does not move the impact).
    const float oe[3]{origin[0] - e[0], origin[1] - e[1], origin[2] - e[2]};
    const float along = oe[0] * d[0] + oe[1] * d[1] + oe[2] * d[2];
    const float perp[3]{oe[0] - d[0] * along, oe[1] - d[1] * along, oe[2] - d[2] * along};
    const float offWu = std::sqrt(perp[0] * perp[0] + perp[1] * perp[1] + perp[2] * perp[2]);
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
#else
    (void)unitIndex; (void)origin; (void)direction;
#endif
}

// ---- census (dev): which flags do Reach's own collision_test_vector callers use? ------------------
#if HALOMCCVR_DEV_DIAGNOSTICS
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
#endif

// Compiled in only with HALOMCCVR_DEV_DIAGNOSTICS; an empty stub otherwise, so a player build carries
// none of it whatever its config says (the Halo: Campaign Evolved dev-tool rule).
void ReachNoteCollisionCensus(uintptr_t caller, uint64_t flags, int32_t mode, const float* vector, int32_t ignoredA)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
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
    // The second argument is a byte (upper bits are whatever the register held): key on the low byte.
    const int32_t modeByte = static_cast<int32_t>(static_cast<uint32_t>(mode) & 0xFFu);
    ReachCollisionCensusEntry* entry = nullptr;
    for (uint32_t i = 0; i < census.used; ++i)
        if (census.entries[i].rva == rva && census.entries[i].flags == flags && census.entries[i].mode == modeByte)
        { entry = &census.entries[i]; break; }
    if (!entry && census.used < kReachCollisionCensusSlots)
    {
        entry = &census.entries[census.used++];
        *entry = {};
        entry->rva = rva;
        entry->flags = flags;
        entry->mode = modeByte;
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
#else
    (void)caller; (void)flags; (void)mode; (void)vector; (void)ignoredA;
#endif
}

void ReportReachSightTrace()
{
    auto& trace = g_reachSightTrace;
    const uint32_t leashFrames = g_reachLeashHeldFrames.exchange(0, std::memory_order_relaxed);
    const uint32_t leashMax = g_reachLeashMaxLiftMm.exchange(0, std::memory_order_relaxed);
    const uint32_t crouchFrames = g_reachCrouchLiftFrames.exchange(0, std::memory_order_relaxed);
    const uint32_t crouchMax = g_reachCrouchMaxLiftMm.exchange(0, std::memory_order_relaxed);
    if (leashFrames || crouchFrames)
        LOG("Reach view over the character's eye: leash held it on %u frames (max lift %u mm, limit %.2f m); "
            "the physical-crouch correction gave back his crouch drop on %u frames (max %u mm); the hands and "
            "reticle ray rose with it", leashFrames, leashMax, double(g_config.head_leash_below_eye_m),
            crouchFrames, crouchMax);
    ReportReachCachedCrouch();

    // VALUE AGREEMENT, continued: the engine reads this global on every crosshair trace, so if anything
    // ever re-wrote it our trace would measure a different ray than the engine's. Withdraw at once.
    const uintptr_t global = g_reachSightFlags.global.load(std::memory_order_acquire);
    uint64_t globalValue = 0;
    if (global && (!ReachGuardedReadQword(global, globalValue) || globalValue != kReachCollisionTestFlagsInit))
    {
        ReachClearSightFlags("the flags global changed after it was resolved");
        LOG("REACHSIGHT aim-trace flags WITHDRAWN: the global now reads 0x%llX (HREK 0x%llX); the aim trace is "
            "off and aim keeps the fixed-distance steer", static_cast<unsigned long long>(globalValue),
            static_cast<unsigned long long>(kReachCollisionTestFlagsInit));
    }

    // Player-facing state, logged when it changes: whether the aim trace is running and, if not, why.
    int32_t mode = 0;
    const uint64_t flags = ReachSightActiveFlags(mode);
    const uint64_t traces = trace.traces.exchange(0, std::memory_order_relaxed);
    const uint64_t hits = trace.hits.exchange(0, std::memory_order_relaxed);
    const uint64_t misses = trace.misses.exchange(0, std::memory_order_relaxed);
    const uint64_t skips = trace.skips.exchange(0, std::memory_order_relaxed);
    const uint64_t unresolved = trace.blockedUnresolved.exchange(0, std::memory_order_relaxed);
    const uint64_t faulted = trace.blockedFaulted.exchange(0, std::memory_order_relaxed);
    const uint64_t idle = trace.blockedIdle.exchange(0, std::memory_order_relaxed);
    const uint64_t unhooked = trace.blockedUnhooked.exchange(0, std::memory_order_relaxed);
    const uint64_t unarmed = trace.blockedUnarmed.exchange(0, std::memory_order_relaxed);
    (void)trace.blockedRate.exchange(0, std::memory_order_relaxed);
    const char* state =
        !flags ? "OFF: the engine's crosshair-trace flags are not resolved on this build; aim keeps the "
                 "fixed-distance steer" :
        trace.faulted.load(std::memory_order_acquire) ? "OFF: the native trace faulted; aim keeps the "
                                                          "fixed-distance steer" :
        unhooked && !traces ? "OFF: the collision hook or contact melee is not live; aim keeps the "
                              "fixed-distance steer" :
        traces ? "running" : "idle (direct drive not converging: not on foot, not armed, or not in use)";
    static const char* s_lastState = nullptr;
    if (state != s_lastState)
    {
        const char* why = g_reachSightFlags.why.load(std::memory_order_acquire);
        LOG("REACHSIGHT aim trace %s (flags 0x%llX, second byte %d%s%s)", state,
            static_cast<unsigned long long>(flags), mode & 0xFF, !flags && why ? "; " : "",
            !flags && why ? why : "");
        s_lastState = state;
    }
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (flags || trace.faulted.load(std::memory_order_acquire))
    {
        auto& shots = g_reachSightShots;
        const uint32_t nearest = trace.nearestMm.exchange(UINT32_MAX, std::memory_order_relaxed);
        const uint32_t measured = shots.measured.exchange(0, std::memory_order_relaxed);
        const uint32_t missSum = shots.missSumMm.exchange(0, std::memory_order_relaxed);
        const uint64_t timed = trace.timed.exchange(0, std::memory_order_relaxed);
        const uint64_t sumTicks = trace.traceTicksSum.exchange(0, std::memory_order_relaxed);
        const uint64_t maxTicks = trace.traceTicksMax.exchange(0, std::memory_order_relaxed);
        LARGE_INTEGER freq{};
        QueryPerformanceFrequency(&freq);
        const double usPerTick = freq.QuadPart ? 1.0e6 / double(freq.QuadPart) : 0.0;
        const uint64_t now = GetTickCount64();
        LOG("REACHSIGHT trace: %llu traces (%llu hits, %u on objects / %llu misses, %llu skipped; blocked "
            "unresolved %llu faulted %llu idle %llu unhooked %llu unarmed %llu); cost avg %.1f / max %.1f us; "
            "range last %.2f m (nearest %.2f / farthest %.2f m), type %u; converged-shot live=%d; shots %u "
            "(%u measured): miss at the traced point avg %u / max %u mm, last %u mm at %.2f m; native origin "
            "vs engine eye max %u mm",
            (unsigned long long)traces, (unsigned long long)hits,
            trace.objectHits.exchange(0, std::memory_order_relaxed), (unsigned long long)misses,
            (unsigned long long)skips, (unsigned long long)unresolved, (unsigned long long)faulted,
            (unsigned long long)idle, (unsigned long long)unhooked, (unsigned long long)unarmed,
            timed ? double(sumTicks) / double(timed) * usPerTick : 0.0, double(maxTicks) * usPerTick,
            double(trace.rangeWu.load(std::memory_order_relaxed) / kReachWorldUnitsPerMeter),
            nearest == UINT32_MAX ? 0.0 : nearest / 1000.0,
            trace.farthestMm.exchange(0, std::memory_order_relaxed) / 1000.0,
            trace.hitType.load(std::memory_order_relaxed),
            ReachConvergedShotLive(now) ? 1 : 0,
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
#else
    (void)hits; (void)misses; (void)skips; (void)unresolved; (void)faulted; (void)idle; (void)unarmed;
#endif
}
