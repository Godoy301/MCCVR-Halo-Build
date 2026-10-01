// Included in game.cpp (anonymous namespace) before reach_contact_melee_runtime.inl.
//
// TRACE-AND-REAIM for Halo Reach, on foot and seated -- the Halo: Campaign Evolved aim.
//
// SEATED (2026-09-28): the same solve with the engine eye E moved. In a vehicle Reach does not fire from
// the character's eye: its aim_assist (HREK 0x4E32A0, after the unit-adjust hook) traces the crosshair
// from its OWN camera along that camera's forward -- the chase camera in a third-person seat (HREK
// 0x4E5B60 -> the following camera), the unit camera in a first-person one -- and turns the barrel's shot
// at whatever the trace hits. So E is a point of that camera line (the one nearest the seat, which stays
// put as the chase camera orbits), the intent ray still starts at the rendered eye, and the trace ignores
// the player's own vehicle. The seat's limits clamp the record before our write; the hook keeps its write
// inside them (aim_write::LimitLatch) and STILL claims the converged aim (2026-10-01): a seat's stop is the
// game's own, and the barrel fires along it as in flat Reach. Only the muzzle redirect stands down there.
//
// The player points along the intent ray: from their rendered EYE, along the controller's direction --
// Halo: Campaign Evolved's geometry, so the reticle drawn on it reads true at any depth. Reach decides
// where a shot goes from its own camera: on the fire tick it traces the unit's aim ray 128 wu into the
// world (HREK aim_assist.cpp:986, aim_projectile_internal_result) and sends the projectile from its
// native origin at the point that trace hit. A direction alone cannot put that point where the intent
// ray points; a direction plus a RANGE can:
//
//      T   = H + u * d                the point the intent ray hits (H the eye, u the controller dir)
//      aim = normalize(T - E)         the direction whose ray from the engine eye E passes through T
//          = normalize((H - E) + u * d)
//
// d is MEASURED by tracing the intent ray with Reach's own collision_test_vector and the engine's own
// crosshair-trace flags (below), on the simulation thread, from inside the per-unit update the contact-
// melee lane already hooks (a native context in which the game itself runs collision queries). The
// input-thread aim provider reads the published range, smooths it, and bends the direct-drive setpoint
// (src/common/aim_solve_logic.h does the math). The engine's crosshair trace then finds T and the shot
// leaves from exactly where the game fires it.
//
// THIS IS THE AIM PATH, not one option among several: it converges whenever it can resolve -- on a
// hit, on a miss (the far end of a 1 km trace), at any bend. The older on-foot hand shot ray -- which
// MOVED the projectile's origin to the hand, local-only and refused beyond 1.5 m -- is only the
// worst-case fallback for when it cannot: the trace is off, faulted or has no result within
// kReachSightFreshMs, the geometry is degenerate, or the aim is steeper than the control record
// accepts. While a converged setpoint is live, that fallback stands down.

struct ReachSightTracePublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint64_t> sampleMs{0};
    std::atomic<uint32_t> hit{0};
    std::atomic<uint32_t> hitType{0};
    std::atomic<float> rangeWu{0.0f};
    std::atomic<bool> faulted{false};     // latched by a native fault; cleared by the next Reach install
    std::atomic<uint64_t> traces{0};      // the player-facing state line reads this ("running")
#if HALOMCCVR_DEV_DIAGNOSTICS
    // Why a tick did not trace, and what the traces found. Dev builds only: the player-facing state
    // line needs none of it, and the hook runs for every unit on every sim tick.
    std::atomic<uint64_t> hits{0}, misses{0}, skips{0};
    std::atomic<uint64_t> blockedUnresolved{0}, blockedFaulted{0}, blockedIdle{0}, blockedUnhooked{0},
        blockedUnarmed{0}, blockedRate{0};
    std::atomic<uint32_t> nearestMm{UINT32_MAX}, farthestMm{0};
    std::atomic<uint64_t> traceTicksSum{0}, traceTicksMax{0}, timed{0};
    std::atomic<uint32_t> objectHits{0};   // type 4: bipeds, vehicles, crates, scenery ...
    std::atomic<uint32_t> bipedHits{0};    // ...of which the hit object's kind is 0 (a biped)
    std::atomic<uint64_t> selfHits{0};     // seated: a pass met our own vehicle and looked past it
#endif
} g_reachSightTrace;

// A dev-build counter on the trace's gates; compiles to nothing (argument unevaluated) in a player build.
#if HALOMCCVR_DEV_DIAGNOSTICS
#define REACH_SIGHT_DEV_COUNT(counter) (counter).fetch_add(1, std::memory_order_relaxed)
#else
#define REACH_SIGHT_DEV_COUNT(counter) ((void)0)
#endif

// The intent ray is traced 1 km out, like Halo: Campaign Evolved's vehicle intent ray (the user's call,
// 2026-09-28). A miss therefore means the target is at least 1 km away, and the aim converges on the
// ray's far end, where the eye-to-hand offset is a small fraction of a degree. (It was 60 wu, ~180 m,
// and a miss went to the fallback.) Cost measured in FA: docs/REACH-AIM-CONVERGENCE-2026-09-28.md.
inline constexpr float kReachSightTraceLengthWu = 1000.0f * kReachWorldUnitsPerMeter;
// A trace result older than this is not used: the trace runs every ~30 ms, so no fresh result means
// the simulation stalled or the trace is off, and the shot genuinely cannot be resolved (the
// worst-case fallback). A miss is a fresh result: it converges on the far end.
inline constexpr uint64_t kReachSightFreshMs = 150;
// The effective length: 1 km, or in a dev build an A/B override (reach_sight_trace_length_m).
float ReachSightTraceLength()
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (g_config.reach_sight_trace_length_m > 0.0f)
        return g_config.reach_sight_trace_length_m * kReachWorldUnitsPerMeter;
#endif
    return kReachSightTraceLengthWu;
}
// Start 5 cm down the ray so the trace clears the head it starts in; the range reported is still
// measured from the ray's origin.
inline constexpr float kReachSightTraceSkipWu = 0.05f * kReachWorldUnitsPerMeter;
// Seated, a pass that meets our own vehicle continues 5 cm past the contact, at most this many passes.
inline constexpr int kReachSightSelfPasses = 3;
inline constexpr float kReachSightSelfStepWu = 0.05f * kReachWorldUnitsPerMeter;

// The root of an object's parent chain (the vehicle a seated unit rides; on foot the unit itself), or -1.
// Sim thread; guarded, and kept free of C++ objects so __try is legal.
int32_t ReachSightRootOf(int32_t object)
{
    const auto ultimateParent = g_reachCamera.objectUltimateParent;
    if (object == -1 || !ultimateParent)
        return -1;
    __try
    {
        return ultimateParent(object);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return -1;
    }
}

// One guarded native collision_test_vector pass (the unhooked trampoline). Our query must not schedule
// world-collision work of its own from anything hooked it reaches, so it runs as an owned query. False on
// a native fault (the caller latches the trace off).
bool ReachSightGuardedTrace(LegacyCollisionTestVectorFn original, uint64_t flags, int32_t mode,
                            const float* start, const float* vector, int32_t ignoreA, int32_t ignoreB,
                            int32_t ignoreC, uint8_t* result, uint8_t& returned, bool injectFault)
{
    const bool previousOwned = g_legacyCollisionOwnedQuery;
    g_legacyCollisionOwnedQuery = true;
    __try
    {
#if HALOMCCVR_DEV_DIAGNOSTICS
        if (injectFault)
            RaiseException(0xE0FA0001u, 0, 0, nullptr);
#else
        (void)injectFault;
#endif
        returned = original(flags, mode, start, vector, ignoreA, ignoreB, ignoreC, result);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        g_legacyCollisionOwnedQuery = previousOwned;
        return false;
    }
    g_legacyCollisionOwnedQuery = previousOwned;
    return true;
}
// The range is smoothed over 120 ms in the provider, so ~30 Hz measures it fully.
inline constexpr uint64_t kReachSightTraceIntervalMs = 30;

// True while the angles the direct-drive hook last wrote into desired_angles were converged ones,
// written within the last 100 ms (the hook runs every sim tick). Recorded by the hook itself on the
// sim thread (g_reachWrittenConvergedMs), which is the thread the firing site runs on, so the stand-
// down follows the write that is actually in effect -- a decline or a release clears it at once. A seat
// limit does NOT clear it (converge::ReachStampUpdate): natural, as in every other title.
bool ReachConvergedShotLive(uint64_t nowMs)
{
    return converge::StampLive(g_reachWrittenConvergedMs.load(std::memory_order_acquire), nowMs, 100);
}

// True while the last stick update wrote an aim the seat-limit latch held short of the setpoint (within
// 100 ms). DEFENSIVE ONLY: the muzzle redirect refuses and the dev engine-trace witness skips. The converged
// claim above is untouched by it.
bool ReachHeldAtLimit(uint64_t nowMs)
{
    return converge::StampLive(g_reachHeldAtLimitMs.load(std::memory_order_acquire), nowMs, 100);
}

// The exact sample + smoothed range the provider last actuated as a converged aim. The shot check (dev)
// measures the solver against what it aimed, not a later sample; shots from the muzzle (every build) fly
// through its point.
struct ReachSightActuatedPublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint64_t> atMs{0};
    std::atomic<float> ox{0}, oy{0}, oz{0}, dx{0}, dy{0}, dz{0}, ex{0}, ey{0}, ez{0}, hx{0}, hy{0}, hz{0}, rangeWu{0};
} g_reachSightActuated;

#if HALOMCCVR_DEV_DIAGNOSTICS

// Set while a render transaction has the HEAD camera committed into the game's own camera state
// (between ReachMainRenderView's commit and its restore), with that thread's id. The shot check
// records both, so a shot that leaves from the head can be told apart from a render/sim race.
std::atomic<uint32_t> g_reachDevHeadCommitted{0};
std::atomic<uint32_t> g_reachDevRenderTid{0};

// One record per measured converged shot: where the native origin N sits relative to every candidate
// eye, in the shot's own frame (along the shot, horizontal right, up; mm). The cold worker prints them.
struct ReachSightShotRecord
{
    uint64_t atMs = 0;
    float missMm = 0.0f, rangeM = 0.0f;
    float fromEye[3]{}, fromHead[3]{}, fromHand[3]{}, fromUnit[3]{}, fromCamera[3]{};
    uint8_t unitValid = 0, cameraValid = 0, firesFromCamera = 0, useUnitAim = 0, headCommitted = 0;
    uint32_t fireTid = 0, renderTid = 0;
};
inline constexpr uint32_t kReachSightShotRingSize = 32;
struct ReachSightShotRing
{
    std::atomic<uint32_t> written{0};
    ReachSightShotRecord records[kReachSightShotRingSize];
    uint32_t printed = 0;   // cold worker only
} g_reachSightShotRing;

// The stock unit camera-position evaluator for this unit (the trampoline, never our hook), and a
// guarded vector read. Kept free of C++ objects so __try is legal.
bool ReachDevUnitCameraPosition(int32_t unit, float* out)
{
    if (!g_origReachUnitCameraPosition || !out)
        return false;
    __try
    {
        g_origReachUnitCameraPosition(unit, out);
        return std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}
bool ReachDevReadVec3(const float* source, float* out)
{
    if (!source || !out)
        return false;
    __try
    {
        out[0] = source[0]; out[1] = source[1]; out[2] = source[2];
        return std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}
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

#if HALOMCCVR_DEV_DIAGNOSTICS
// The kind of the object a trace hit (result +0x40 = object index, HREK collisions.cpp), 0 = biped,
// -1 = unknown. Sim thread, SEH-guarded (dev builds only).
int ReachSightHitObjectKind(const uint8_t* result)
{
    __try
    {
        int32_t object = -1;
        memcpy(&object, result + 0x40, sizeof(object));
        uint8_t kind = 255;
        return object != -1 && ReachVehicleObjectData(object, kind) ? kind : -1;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return -1;
    }
}
#endif

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
    // A native fault latches the trace off for the rest of THIS install; a new install tries again.
    g_reachSightTrace.faulted.store(false, std::memory_order_release);
    uint64_t expected = kReachCollisionTestFlagsInit;
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (g_config.reach_sight_fault & 4)   // fault injection, bit 2: the value check must refuse
        expected ^= 1ull;
#endif
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
    else if (value != expected)
        why = "the flags global disagrees with HREK's initial value";
    if (why)
    {
        ReachClearSightFlags(why);
        LOG("REACHSIGHT aim-trace flags NOT resolved: %s (anchor +0x%llX, global +0x%llX = 0x%llX, expected "
            "0x%llX); the aim trace stays off and aim keeps the fixed-distance steer",
            why, anchor ? static_cast<unsigned long long>(anchor - base) : 0ull,
            global ? static_cast<unsigned long long>(global - base) : 0ull, static_cast<unsigned long long>(value),
            static_cast<unsigned long long>(expected));
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
    if (g_config.reach_sight_fault & 2)      // fault injection, bit 1: withhold the resolved flags
        flags = 0;
#endif
    return flags;
}

void ReachSightTraceTick(uint32_t unit)
{
    // Every unit comes through this hook on every sim tick, so the cheapest gate comes first: plain config
    // loads. Only direct drive (mode 3) consumes the trace, and only with F1's Converged aim on.
    if (g_config.reach_direct_drive != 3 || !g_config.converged_aim)
        return;
    auto& trace = g_reachSightTrace;
    const uint64_t now = GetTickCount64();
    const uint64_t demand = g_reachSightDemandMs.load(std::memory_order_relaxed);
    if (!demand || now < demand || now - demand > kReachSightDemandFreshMs)
    {
        REACH_SIGHT_DEV_COUNT(trace.blockedIdle);
        return;
    }
    int32_t mode = 0;
    const uint64_t flags = ReachSightActiveFlags(mode);
    if (!flags)
    {
        REACH_SIGHT_DEV_COUNT(trace.blockedUnresolved);
        return;
    }
    if (trace.faulted.load(std::memory_order_acquire))
    {
        REACH_SIGHT_DEV_COUNT(trace.blockedFaulted);
        return;
    }
    // The call below is the 8-argument collision_test_vector trampoline -- proven by the install that
    // hooked the scheduler entry, not inferred -- and contact melee must be live (its per-unit hook is
    // the one we run inside, and its teardown drains before `original` is cleared).
    if (!g_reachWorldCollision.installed.load(std::memory_order_acquire) ||
        !g_reachWorldCollision.originalIsVectorTest || !ReachContactMeleeReady())
    {
        REACH_SIGHT_DEV_COUNT(trace.blockedUnhooked);
        return;
    }
    if (!g_reachCamera.armed.load(std::memory_order_acquire))
    {
        REACH_SIGHT_DEV_COUNT(trace.blockedUnarmed);
        return;
    }
    // Rate-limit first, then find the local unit through a cache so a busy level does not cost a native
    // lookup per unit.
    static uint64_t s_lastTraceMs = 0, s_localAtMs = 0;
    static int32_t s_localUnit = -1;
    if (s_lastTraceMs && now >= s_lastTraceMs && now - s_lastTraceMs < kReachSightTraceIntervalMs)
    {
        REACH_SIGHT_DEV_COUNT(trace.blockedRate);
        return;
    }
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
        REACH_SIGHT_DEV_COUNT(trace.blockedUnhooked);
        return;
    }
    const uint32_t generation = g_reachCamera.generation.load(std::memory_order_acquire);
    ReachCompletedReticleTargetSnapshot ray{};
    if (!ReachReadCompletedReticleTarget(ray, g_reachLocalShotRay) || !generation ||
        ray.generation != generation || !ray.sampleMs || now < ray.sampleMs || now - ray.sampleMs > 100 ||
        ray.key.unitHandle != localUnit)
    {
        REACH_SIGHT_DEV_COUNT(trace.skips);
        return;
    }
    float u[3]{ray.target[0] - ray.origin[0], ray.target[1] - ray.origin[1], ray.target[2] - ray.origin[2]};
    const float length = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (!std::isfinite(length) || length < 1.0e-3f || !std::isfinite(ray.head[0]) ||
        !std::isfinite(ray.head[1]) || !std::isfinite(ray.head[2]))
    {
        REACH_SIGHT_DEV_COUNT(trace.skips);
        return;
    }
    for (float& c : u) c /= length;
    // Ignored objects. On foot: the local unit -- the engine's crosshair trace ignores the unit's root, and
    // on foot that is the unit itself. Seated (the ray carries the seat key): the engine ignores the root,
    // the vehicle, but its trace starts out past the vehicle, while ours starts at the rendered eye INSIDE
    // it; so ours ignores the root, the occupant and the seat's own parent (a Warthog's turret is a
    // vehicle of its own), and looks past any other part of the same vehicle it still meets.
    const bool seated = ray.key.directParent != -1;
    int32_t ignoreA = localUnit, ignoreB = -1, ignoreC = -1, root = -1;
    if (seated)
    {
        root = ReachSightRootOf(localUnit);
        if (root == -1)
        {
            REACH_SIGHT_DEV_COUNT(trace.skips);
            return;
        }
        ignoreA = root;
        ignoreB = localUnit;
        ignoreC = ray.key.directParent != root ? ray.key.directParent : -1;
    }
    // From the player's EYE along the controller's direction (see Game_ReachSightSample): the ray the
    // aim converges on and the reticle is drawn along.
    const float traceLengthWu = ReachSightTraceLength();
    // Result: type (int32 +0), fraction (float +4) and the hit object (int32 +0x40), the layout HREK's
    // collisions.cpp asserts name and this repo's Reach contact-melee path already reads; the buffer is
    // generously sized.
    alignas(16) uint8_t result[0x400]{};
#if HALOMCCVR_DEV_DIAGNOSTICS
    // Fault injection (dev), bit 0: raise INSIDE the guarded native call, so the genuine __except runs
    // (the owned-query flag restore and the fault latch) -- the exact path a real native fault takes.
    const bool injectFault = (g_config.reach_sight_fault & 1) != 0;
    LARGE_INTEGER qpcStart{};
    QueryPerformanceCounter(&qpcStart);
#else
    const bool injectFault = false;
#endif
    bool hit = false;
    uint32_t type = 0;
    float rangeWu = 0.0f;
    float alongWu = kReachSightTraceSkipWu;   // where this pass starts, measured from the eye
    for (int pass = 0; pass < (seated ? kReachSightSelfPasses : 1); ++pass)
    {
        const float remainingWu = traceLengthWu - (alongWu - kReachSightTraceSkipWu);
        if (!(remainingWu > 0.0f))
            break;
        const float start[3]{ray.head[0] + u[0] * alongWu, ray.head[1] + u[1] * alongWu,
                             ray.head[2] + u[2] * alongWu};
        const float vector[3]{u[0] * remainingWu, u[1] * remainingWu, u[2] * remainingWu};
        memset(result, 0, sizeof(result));
        uint8_t returned = 0;
        if (!ReachSightGuardedTrace(original, flags, mode, start, vector, ignoreA, ignoreB, ignoreC, result,
                                    returned, injectFault))
        {
            trace.faulted.store(true, std::memory_order_release);   // fail closed: the old aim comes back
            return;
        }
        trace.traces.fetch_add(1, std::memory_order_relaxed);
        float fraction = 1.0f;
        memcpy(&type, result, sizeof(type));
        memcpy(&fraction, result + 4, sizeof(fraction));
        hit = returned && std::isfinite(fraction) && fraction >= 0.0f && fraction <= 1.0f;
        if (!hit)
            break;
        rangeWu = alongWu + fraction * remainingWu;
        if (!seated)
            break;
        int32_t object = -1;
        memcpy(&object, result + 0x40, sizeof(object));
        if (object == -1 || ReachSightRootOf(object) != root)
            break;
        // A part of our own vehicle the ignores did not cover (another occupant, an attached gun):
        // not a target. Look past it.
        REACH_SIGHT_DEV_COUNT(trace.selfHits);
        hit = false;
        rangeWu = 0.0f;
        alongWu += fraction * remainingWu + kReachSightSelfStepWu;
    }
#if HALOMCCVR_DEV_DIAGNOSTICS
    LARGE_INTEGER qpcEnd{};
    QueryPerformanceCounter(&qpcEnd);
    const uint64_t spent = static_cast<uint64_t>(qpcEnd.QuadPart - qpcStart.QuadPart);
    trace.traceTicksSum.fetch_add(spent, std::memory_order_relaxed);
    trace.timed.fetch_add(1, std::memory_order_relaxed);
    uint64_t priorTicks = trace.traceTicksMax.load(std::memory_order_relaxed);
    while (spent > priorTicks && !trace.traceTicksMax.compare_exchange_weak(priorTicks, spent, std::memory_order_relaxed)) {}
    (hit ? trace.hits : trace.misses).fetch_add(1, std::memory_order_relaxed);
    if (hit && type == 4)
    {
        trace.objectHits.fetch_add(1, std::memory_order_relaxed);
        if (ReachSightHitObjectKind(result) == 0)
            trace.bipedHits.fetch_add(1, std::memory_order_relaxed);
    }
    if (hit)
    {
        const uint32_t mm = static_cast<uint32_t>(rangeWu / kReachWorldUnitsPerMeter * 1000.0f);
        uint32_t prior = trace.nearestMm.load(std::memory_order_relaxed);
        while (mm < prior && !trace.nearestMm.compare_exchange_weak(prior, mm, std::memory_order_relaxed)) {}
        prior = trace.farthestMm.load(std::memory_order_relaxed);
        while (mm > prior && !trace.farthestMm.compare_exchange_weak(prior, mm, std::memory_order_relaxed)) {}
    }
#endif
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

// What the provider AIMED at: the ray (o, u), the engine eye e, the rendered head h and the smoothed
// range it actuated -- not a later sample. False unless one coherent record younger than 150 ms exists.
bool ReachSightReadActuated(uint64_t now, float o[3], float u[3], float e[3], float h[3], float& rangeWu)
{
    auto& a = g_reachSightActuated;
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
        h[0] = a.hx.load(std::memory_order_relaxed); h[1] = a.hy.load(std::memory_order_relaxed); h[2] = a.hz.load(std::memory_order_relaxed);
        rangeWu = a.rangeWu.load(std::memory_order_relaxed);
        coherent = a.sequence.load(std::memory_order_acquire) == before;
    }
    return coherent && atMs && now >= atMs && now - atMs <= 150 && std::isfinite(rangeWu) && rangeWu > 0.0f;
}

// SHOTS FROM THE MUZZLE (sight_layer.h), Reach: the converged point is the one the direct drive last wrote
// (ReachConvergedShotLive), the intent ray at the range the provider actuated. Firing-site thread.
bool ReachAimMuzzleShot(const float before[3], const float position[3], float direction[3], float up[3])
{
    const uint64_t now = GetTickCount64();
    float o[3]{}, u[3]{}, e[3]{}, h[3]{}, rangeWu = 0.0f;
    if (!ReachConvergedShotLive(now) || !ReachSightReadActuated(now, o, u, e, h, rangeWu))
    {
        Sight_NoteMuzzleShotUnconverged(GameTitle::HaloReach);
        return false;
    }
    // Defensive (Reach publishes no muzzle palette in a seat today): a seat held at its stop points the barrel
    // elsewhere, so a shot bent through the point from it would follow neither line. Left as the game fires it.
    if (ReachHeldAtLimit(now))
    {
        Sight_NoteMuzzleShotHeld(GameTitle::HaloReach);
        return false;
    }
    const float point[3]{o[0] + u[0] * rangeWu, o[1] + u[1] * rangeWu, o[2] + u[2] * rangeWu};
    return Sight_AimMuzzleShotThrough(GameTitle::HaloReach, point, u, e, before, position, direction, up);
}

void ReachSightMeasureShot(int32_t unitIndex, const float* origin, const float* direction,
                           const float* cameraPoint, uint8_t firesFromCamera, uint8_t useUnitAim)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    auto& stats = g_reachSightShots;
    stats.shots.fetch_add(1, std::memory_order_relaxed);
    if (!origin || !direction)
        return;
    const uint64_t now = GetTickCount64();
    float o[3]{}, u[3]{}, e[3]{}, h[3]{}, rangeWu = 0.0f;
    if (!ReachSightReadActuated(now, o, u, e, h, rangeWu))
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

    // WHERE DOES THE NATIVE SHOT LEAVE FROM? N relative to each candidate eye, in the shot's frame:
    // along the shot, horizontal right (d x world up), up. The candidate it sits on is the answer.
    float right[3]{d[1] * 1.0f - d[2] * 0.0f, d[2] * 0.0f - d[0] * 1.0f, d[0] * 0.0f - d[1] * 0.0f};
    float rl = std::sqrt(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
    if (!(rl > 1.0e-4f)) { right[0] = 1.0f; right[1] = 0.0f; right[2] = 0.0f; rl = 1.0f; }
    for (float& c2 : right) c2 /= rl;
    const float up[3]{right[1] * d[2] - right[2] * d[1], right[2] * d[0] - right[0] * d[2], right[0] * d[1] - right[1] * d[0]};
    const float toMm = 1000.0f / kReachWorldUnitsPerMeter;
    const auto frame = [&](const float* from, float* outMm)
    {
        const float v[3]{origin[0] - from[0], origin[1] - from[1], origin[2] - from[2]};
        outMm[0] = (v[0] * d[0] + v[1] * d[1] + v[2] * d[2]) * toMm;
        outMm[1] = (v[0] * right[0] + v[1] * right[1] + v[2] * right[2]) * toMm;
        outMm[2] = (v[0] * up[0] + v[1] * up[1] + v[2] * up[2]) * toMm;
    };
    auto& ring = g_reachSightShotRing;
    const uint32_t slot = ring.written.load(std::memory_order_relaxed) % kReachSightShotRingSize;
    ReachSightShotRecord& r = ring.records[slot];
    r = {};
    r.atMs = now;
    r.missMm = missWu * toMm;
    r.rangeM = rangeWu / kReachWorldUnitsPerMeter;
    frame(e, r.fromEye);
    frame(h, r.fromHead);
    frame(o, r.fromHand);
    float unitEye[3]{}, camera[3]{};
    r.unitValid = ReachDevUnitCameraPosition(unitIndex, unitEye) ? 1 : 0;
    if (r.unitValid) frame(unitEye, r.fromUnit);
    r.cameraValid = ReachDevReadVec3(cameraPoint, camera) ? 1 : 0;
    if (r.cameraValid) frame(camera, r.fromCamera);
    r.firesFromCamera = firesFromCamera;
    r.useUnitAim = useUnitAim;
    r.headCommitted = static_cast<uint8_t>(g_reachDevHeadCommitted.load(std::memory_order_acquire));
    r.fireTid = GetCurrentThreadId();
    r.renderTid = g_reachDevRenderTid.load(std::memory_order_relaxed);
    ring.written.fetch_add(1, std::memory_order_release);
#else
    (void)unitIndex; (void)origin; (void)direction; (void)cameraPoint; (void)firesFromCamera; (void)useUnitAim;
#endif
}

// ---- engine-trace check (dev): does REACH'S OWN crosshair trace pass through the traced point? ------
// This is where a seated shot is decided: Reach's aim_assist (HREK 0x4E32A0, called from the projectile
// builder AFTER the unit-adjust hook the shot check above runs in) traces the crosshair from its camera
// along the aim and turns the barrel's shot at whatever that trace hits. So the check for a vehicle is the
// trace itself, seen in the collision hook: its start and direction against what the solver aimed.
#if HALOMCCVR_DEV_DIAGNOSTICS
struct ReachEngineTraceRecord
{
    uint64_t atMs = 0;
    float missMm = 0.0f, rangeM = 0.0f, dirVsAimDeg = 0.0f;
    float startFromEye[3]{};    // the trace's start minus the solver's eye, (along the aim, right, up) mm
    int32_t ignored[3]{-1, -1, -1};
    int32_t unit = -1, root = -1;
};
inline constexpr uint32_t kReachEngineTraceRingSize = 16;
struct ReachEngineTraceRing
{
    std::atomic<uint32_t> written{0};
    ReachEngineTraceRecord records[kReachEngineTraceRingSize];
    uint32_t printed = 0;   // cold worker only
    // Crosshair traces skipped while a seat held the write at its stop: the converged claim stays on there
    // (2026-10-01), but Reach's line is off the point by design, so they are not misses of the solve.
    std::atomic<uint32_t> heldSkips{0};
} g_reachEngineTraceRing;
#endif

void ReachSightNoteEngineTrace(uint64_t flags, const float* start, const float* vector,
                               int32_t ignoredA, int32_t ignoredB, int32_t ignoredC)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    // Cheapest gate first: this runs for every collision query the game makes.
    const uint64_t resolved = g_reachSightFlags.flags.load(std::memory_order_acquire);
    if (!resolved || flags != resolved || !start || !vector)
        return;
    const uint64_t now = GetTickCount64();
    if (!ReachConvergedShotLive(now))
        return;
    if (ReachHeldAtLimit(now))
    {
        g_reachEngineTraceRing.heldSkips.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    float o[3]{}, u[3]{}, e[3]{}, h[3]{}, rangeWu = 0.0f;
    if (!ReachSightReadActuated(now, o, u, e, h, rangeWu))
        return;
    const float vl = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    if (!std::isfinite(vl) || vl < 1.0e-6f)
        return;
    const float d[3]{vector[0] / vl, vector[1] / vl, vector[2] / vl};
    // The traced point the solver aimed at, and its distance from the engine's crosshair line.
    const float target[3]{o[0] + u[0] * rangeWu, o[1] + u[1] * rangeWu, o[2] + u[2] * rangeWu};
    const float w[3]{target[0] - start[0], target[1] - start[1], target[2] - start[2]};
    const float c[3]{w[1] * d[2] - w[2] * d[1], w[2] * d[0] - w[0] * d[2], w[0] * d[1] - w[1] * d[0]};
    // The direction the hook wrote (desired_angles), against the direction Reach actually traced.
    const float yaw = g_reachDriveYaw.load(std::memory_order_relaxed);
    const float pitch = g_reachDrivePitch.load(std::memory_order_relaxed);
    const float a[3]{std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
    float dot = a[0] * d[0] + a[1] * d[1] + a[2] * d[2];
    dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
    // The start relative to the solver's eye, in the aim's frame (along, horizontal right, up).
    float right[3]{d[1], -d[0], 0.0f};
    float rl = std::sqrt(right[0] * right[0] + right[1] * right[1]);
    if (!(rl > 1.0e-4f)) { right[0] = 1.0f; right[1] = 0.0f; rl = 1.0f; }
    right[0] /= rl; right[1] /= rl;
    const float up[3]{right[1] * d[2] - right[2] * d[1], right[2] * d[0] - right[0] * d[2],
                      right[0] * d[1] - right[1] * d[0]};
    const float se[3]{start[0] - e[0], start[1] - e[1], start[2] - e[2]};
    const float toMm = 1000.0f / kReachWorldUnitsPerMeter;
    auto& ring = g_reachEngineTraceRing;
    ReachEngineTraceRecord& r = ring.records[ring.written.load(std::memory_order_relaxed) % kReachEngineTraceRingSize];
    r = {};
    r.atMs = now;
    r.missMm = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) * toMm;
    r.rangeM = rangeWu / kReachWorldUnitsPerMeter;
    r.dirVsAimDeg = std::acos(dot) * 57.29578f;
    r.startFromEye[0] = (se[0] * d[0] + se[1] * d[1] + se[2] * d[2]) * toMm;
    r.startFromEye[1] = (se[0] * right[0] + se[1] * right[1] + se[2] * right[2]) * toMm;
    r.startFromEye[2] = (se[0] * up[0] + se[1] * up[1] + se[2] * up[2]) * toMm;
    r.ignored[0] = ignoredA; r.ignored[1] = ignoredB; r.ignored[2] = ignoredC;
    r.unit = LegacyCollisionIgnoredObject(GameTitle::HaloReach);
    r.root = ReachSightRootOf(r.unit);
    ring.written.fetch_add(1, std::memory_order_release);
#else
    (void)flags; (void)start; (void)vector; (void)ignoredA; (void)ignoredB; (void)ignoredC;
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
    uint64_t expected = kReachCollisionTestFlagsInit;
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (g_config.reach_sight_fault & 8)   // fault injection, bit 3: this re-check must withdraw
        expected ^= 1ull;
#endif
    const uintptr_t global = g_reachSightFlags.global.load(std::memory_order_acquire);
    uint64_t globalValue = 0;
    if (global && (!ReachGuardedReadQword(global, globalValue) || globalValue != expected))
    {
        ReachClearSightFlags("the flags global changed after it was resolved");
        LOG("REACHSIGHT aim-trace flags WITHDRAWN: the global now reads 0x%llX (expected 0x%llX); the aim "
            "trace is off and aim keeps the fixed-distance steer", static_cast<unsigned long long>(globalValue),
            static_cast<unsigned long long>(expected));
    }

    // Player-facing state, logged when it changes: whether the aim trace is running and, if not, why.
    int32_t mode = 0;
    const uint64_t flags = ReachSightActiveFlags(mode);
    const uint64_t traces = trace.traces.exchange(0, std::memory_order_relaxed);
    // The trace runs inside contact melee's per-unit hook and calls the collision hook's trampoline;
    // without both it cannot run at all, and "idle" would hide why.
    const bool hostLive = g_reachWorldCollision.installed.load(std::memory_order_acquire) &&
                          g_reachWorldCollision.originalIsVectorTest && ReachContactMeleeReady();
    const char* state =
#if HALOMCCVR_DEV_DIAGNOSTICS
        // A fault-injected OFF must never read like a genuine one.
        (g_config.reach_sight_fault & 2) ? "OFF: DEV fault injection (reach_sight_fault bit 1) withholds the "
                                           "resolved flags; aim keeps the fixed-distance steer" :
        (g_config.reach_sight_fault & 1) && trace.faulted.load(std::memory_order_acquire)
            ? "OFF: DEV fault injection (reach_sight_fault bit 0) faulted the trace; aim keeps the "
              "fixed-distance steer" :
#endif
        !flags ? "OFF: the engine's crosshair-trace flags are not resolved on this build; aim keeps the "
                 "fixed-distance steer" :
        trace.faulted.load(std::memory_order_acquire) ? "OFF: the native trace faulted; aim keeps the "
                                                          "fixed-distance steer" :
        !hostLive ? "OFF: the collision hook or contact melee (the per-unit hook the trace runs in) is not "
                    "live; aim keeps the fixed-distance steer" :
        traces ? "running" : "idle (direct drive not converging: not on foot or in an aimed seat, not armed, "
                             "or not in use)";
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
    if (flags || trace.faulted.load(std::memory_order_acquire) || g_config.reach_sight_fault)
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
        LOG("REACHSIGHT trace: %llu traces (%llu hits, %u on objects, %u bipeds / %llu misses, %llu skipped, %llu "
            "passes past our own vehicle; blocked "
            "unresolved %llu faulted %llu idle %llu unhooked %llu unarmed %llu); cost avg %.1f / max %.1f us; "
            "range last %.2f m (nearest %.2f / farthest %.2f m), type %u; converged-shot live=%d; shots %u "
            "(%u measured): miss at the traced point avg %u / max %u mm, last %u mm at %.2f m; native origin "
            "vs engine eye max %u mm",
            (unsigned long long)traces,
            (unsigned long long)trace.hits.exchange(0, std::memory_order_relaxed),
            trace.objectHits.exchange(0, std::memory_order_relaxed),
            trace.bipedHits.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.misses.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.skips.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.selfHits.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.blockedUnresolved.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.blockedFaulted.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.blockedIdle.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.blockedUnhooked.exchange(0, std::memory_order_relaxed),
            (unsigned long long)trace.blockedUnarmed.exchange(0, std::memory_order_relaxed),
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
        (void)trace.blockedRate.exchange(0, std::memory_order_relaxed);
    }
    // One line per measured shot (at most 8 per report): the native origin N relative to each
    // candidate eye, as (along the shot, right, up) mm. The candidate N sits on reads (~0, ~0, ~0).
    {
        auto& ring = g_reachSightShotRing;
        const uint32_t written = ring.written.load(std::memory_order_acquire);
        if (written - ring.printed > kReachSightShotRingSize)
            ring.printed = written - kReachSightShotRingSize;
        uint32_t shown = 0;
        for (; ring.printed != written && shown < 8; ++ring.printed, ++shown)
        {
            const ReachSightShotRecord r = ring.records[ring.printed % kReachSightShotRingSize];
            LOG("REACHSIGHT shot %u: miss %.0f mm at %.2f m; N minus [along,right,up mm] eye(solve) %.0f,%.0f,%.0f "
                "| head %.0f,%.0f,%.0f | ray-origin %.0f,%.0f,%.0f | unit-eval%s %.0f,%.0f,%.0f | camera-arg%s "
                "%.0f,%.0f,%.0f; firesFromCamera=%u useUnitAim=%u; head-committed=%u fire-tid=%u render-tid=%u",
                ring.printed, double(r.missMm), double(r.rangeM),
                double(r.fromEye[0]), double(r.fromEye[1]), double(r.fromEye[2]),
                double(r.fromHead[0]), double(r.fromHead[1]), double(r.fromHead[2]),
                double(r.fromHand[0]), double(r.fromHand[1]), double(r.fromHand[2]),
                r.unitValid ? "" : "(n/a)", double(r.fromUnit[0]), double(r.fromUnit[1]), double(r.fromUnit[2]),
                r.cameraValid ? "" : "(n/a)", double(r.fromCamera[0]), double(r.fromCamera[1]), double(r.fromCamera[2]),
                unsigned(r.firesFromCamera), unsigned(r.useUnitAim), unsigned(r.headCommitted),
                unsigned(r.fireTid), unsigned(r.renderTid));
        }
        if (ring.printed != written)
            ring.printed = written;   // more than 8 this window: the rest are summarized above
    }
    // Reach's own crosshair traces while a converged aim was live: the check that decides a SEATED shot
    // (the barrel fires at what this line hits). Pass = the point aimed at sits ~0 mm off the line.
    {
        auto& ring = g_reachEngineTraceRing;
        const uint32_t written = ring.written.load(std::memory_order_acquire);
        if (written - ring.printed > kReachEngineTraceRingSize)
            ring.printed = written - kReachEngineTraceRingSize;
        uint32_t shown = 0;
        for (; ring.printed != written && shown < 8; ++ring.printed, ++shown)
        {
            const ReachEngineTraceRecord r = ring.records[ring.printed % kReachEngineTraceRingSize];
            LOG("REACHSIGHT engine trace %u: the point aimed at is %.0f mm off Reach's crosshair line at %.2f m; "
                "that line is %.2f deg from the written aim; its start minus eye(solve) [along,right,up mm] "
                "%.0f,%.0f,%.0f; ignores %d/%d/%d (unit %d, root %d)",
                ring.printed, double(r.missMm), double(r.rangeM), double(r.dirVsAimDeg),
                double(r.startFromEye[0]), double(r.startFromEye[1]), double(r.startFromEye[2]),
                r.ignored[0], r.ignored[1], r.ignored[2], r.unit, r.root);
        }
        if (ring.printed != written)
            ring.printed = written;
        const uint32_t heldSkips = ring.heldSkips.exchange(0, std::memory_order_relaxed);
        if (heldSkips)
            LOG("REACHSIGHT engine trace: %u more skipped (a seat held the write at its stop: still converged, "
                "and Reach's line follows the gun there, not the point)", heldSkips);
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
#endif
}
