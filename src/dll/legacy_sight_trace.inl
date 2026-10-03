// Included in game.cpp (anonymous namespace) BEFORE halo3_contact_melee_runtime.inl and
// odst_contact_melee_runtime.inl, whose per-unit update hooks call LegacySightTraceTick (each hands it its
// own object-table reader, which is defined there).
//
// TRACE-AND-REAIM for Halo 3 and ODST, on foot and in C20 seats (sight_layer.h) -- Reach's aim
// (reach_sight_trace.inl) for two titles whose bent aim is written by direct drive (aim_direct_drive, default 1;
// legacy_direct_drive.inl) or, with it off or not yet armed, steered by the right-stick servo.
//
// The player points along the intent ray: from the rendered centre eye (g_cam*, the camera after head
// look) along the controller. The engine fires from its own eye (g_baseCam*, the camera before head look:
// the seat re-origin in Game_ComputeAimStick already treats it as the shot origin), so a servo that chases
// the controller's direction leaves every shot parallel to the sight line, off by the head's offset. The
// servo now chases the direction from the engine eye through the point the intent ray hits:
//
//      aim = normalize((H - E) + u * d)        H the rendered eye, u the controller, E the engine eye
//
// d is measured by tracing the intent ray with the title's own collision_test_vector and the engine's own
// crosshair-trace flags, on the simulation thread, from inside the per-unit update the contact-melee lane
// already hooks (a native context in which the game itself runs collision queries). The input thread
// publishes the ray (Sight_PublishRay) and bends the servo target with the latest fresh range.
//
// THE FLAGS, resolved per build and value-checked (never baked in). Both titles trace their crosshair the
// way Reach does (halo3 0x13BAD0, the firing-assist function the dual-aim lane hooks; odst 0x161488 and
// 0x3A9128): `mov rax, [flags global]; mov ecx, eax; or ecx, 0x202000; shr rax, 32` feeding a call to
// collision_test_vector, mode 0, ignoring the unit's root. The global is built by a CRT initializer from
// 0x809 (| 0x24000, high | 1) -- exactly Reach's HREK value, 0x0000000100024809 (read statically from both
// retail modules: halo3 0x5714 -> 0x5730 -> 0x5778, odst 0x5A38). The rungs, each failing CLOSED (no trace;
// the servo keeps its old target):
//   1. ANCHORS  every `or ecx, 0x202000` in .text passes ReachInspectAimTraceSite against the hooked
//               collision_test_vector (Halo 3 has one; ODST two), and all of them name one global;
//   2. PINNED   that global is the flags global the contact-melee install pinned independently, from its
//               own call site (halo3 0x35B265, odst 0x39FE8E);
//   3. VALUE    it holds 0x0000000100024809 (ValueAgreement).
// ADDR-HYGIENE: resolved + guarded (the anchors and the value are the same HREK-derived facts Reach uses;
// the retail offsets above are only where the anchors matched, recorded for the next reader).

struct LegacySightState
{
    std::atomic<uint64_t> flags{0};                       // 0 = unresolved: the trace stays off
    std::atomic<const char*> why{"the title is not installed"};
    std::atomic<uint32_t> anchors{0}, globalRva{0};
    std::atomic<uint64_t> converged{0}, fallback{0};      // servo targets, counted on the input thread
    // Seated traces (sim thread): how many ran, how many passes met a part of our own vehicle and looked
    // past it, how many seats were refused because the aim path named no vehicle, and how many fell back
    // to the published seat chain because the object table could not be walked.
    std::atomic<uint64_t> seatedTraces{0}, selfHits{0}, seatUnnamed{0}, rootFallbacks{0};
    // Dev: the return address of each crosshair trace's collision call (the engine-crosshair check).
    std::atomic<uintptr_t> siteReturn[4]{};
};
// [0] Halo 3, [1] ODST, [2] Halo 4 (Halo 4's query is PhysicsRayCast with the contact lane's pinned melee
// flags, see halo4_contact_melee_runtime.inl: Halo4SightTraceTick; its `flags` is only "available").
LegacySightState g_legacySight[3];

LegacySightState& LegacySight(GameTitle title)
{
    return g_legacySight[title == GameTitle::Halo3ODST ? 1 : title == GameTitle::Halo4 ? 2 : 0];
}

const char* LegacySightTitleName(GameTitle title)
{
    return title == GameTitle::Halo3ODST ? "ODST" : title == GameTitle::Halo4 ? "Halo 4" : "Halo 3";
}

// Install path (cold, once per title install, after the contact install). Clears first, so a failed resolve
// on a new build never inherits an earlier answer.
void LegacyResolveSightFlags(GameTitle title, uintptr_t base, size_t size,
                             const LegacyWorldCollisionFeature& collision, const uint64_t* pinnedGlobal)
{
    auto& state = LegacySight(title);
    state.flags.store(0, std::memory_order_release);
    state.anchors.store(0, std::memory_order_relaxed);
    state.globalRva.store(0, std::memory_order_relaxed);
    for (auto& site : state.siteReturn)
        site.store(0, std::memory_order_relaxed);
    Sight_ResetTitle(title);
    static constexpr char kAnchor[] = "81 C9 00 20 20 00";   // or ecx, 0x202000
    const uintptr_t collisionTest = reinterpret_cast<uintptr_t>(collision.target);
    uintptr_t textBegin = 0, textEnd = 0, global = 0;
    uint32_t stamp = 0, anchors = 0;
    size_t imageSize = 0;
    uint64_t value = 0;
    const char* why = nullptr;
    if (!collision.installed.load(std::memory_order_acquire) || !collision.originalIsVectorTest || !collisionTest)
        why = "collision_test_vector is not hooked on this build";
    else if (!pinnedGlobal)
        why = "the contact-melee install did not pin a collision-flags global";
    else if (!ResolveOdstTextRange(base, size, textBegin, textEnd, stamp, imageSize))
        why = "the module has no .text section";
    else
    {
        for (uintptr_t at = sig::Find(textBegin, textEnd - textBegin, kAnchor); at && !why;
             at = at + 1 < textEnd ? sig::Find(at + 1, textEnd - at - 1, kAnchor) : 0)
        {
            uintptr_t named = 0;
            if ((why = ReachInspectAimTraceSite(at, textBegin, textEnd, base, size, collisionTest, named)) != nullptr)
                break;
            if (global && named != global)
                why = "two crosshair traces name different flags globals";
            global = named;
            // The collision call the inspector matched: its return address identifies the site's queries.
            if (anchors < 4)
            {
                const auto* code = reinterpret_cast<const uint8_t*>(at);
                for (uintptr_t fwd = 6; fwd <= 0x100 && at + fwd + 5 <= textEnd; ++fwd)
                {
                    int32_t rel = 0;
                    if (code[fwd] != 0xE8)
                        continue;
                    memcpy(&rel, code + fwd + 1, sizeof(rel));
                    if (at + fwd + 5 + static_cast<intptr_t>(rel) == collisionTest)
                    {
                        state.siteReturn[anchors].store(at + fwd + 5, std::memory_order_relaxed);
                        break;
                    }
                }
            }
            if (++anchors > 4)
                why = "more crosshair-trace anchors than any known build has";
        }
        if (!why && !anchors)
            why = "the anchor `or ecx, 0x202000` is missing";
        else if (!why && global != reinterpret_cast<uintptr_t>(pinnedGlobal))
            why = "the crosshair trace's flags global is not the one the contact install pinned";
        else if (!why && !ReachGuardedReadQword(global, value))
            why = "the flags global is unreadable";
        else if (!why && value != kReachCollisionTestFlagsInit)
            why = "the flags global disagrees with the editing kit's initial value";
    }
    if (why)
    {
        state.why.store(why, std::memory_order_release);
        LOG("SIGHT %s aim-trace flags NOT resolved: %s (%u anchors, global +0x%llX = 0x%llX, expected 0x%llX); "
            "no trace, the aim servo keeps its old target",
            LegacySightTitleName(title), why, anchors,
            global ? static_cast<unsigned long long>(global - base) : 0ull, static_cast<unsigned long long>(value),
            static_cast<unsigned long long>(kReachCollisionTestFlagsInit));
        return;
    }
    const uint64_t flags = value | kReachAimTraceFlagsOr;
    state.anchors.store(anchors, std::memory_order_relaxed);
    state.globalRva.store(static_cast<uint32_t>(global - base), std::memory_order_relaxed);
    state.why.store(nullptr, std::memory_order_release);
    state.flags.store(flags, std::memory_order_release);
    LOG("SIGHT %s aim-trace flags resolved: %u crosshair-trace anchor(s), global +0x%llX = 0x%llX (the editing "
        "kit's value, and the global the contact install pinned); tracing with 0x%llX, mode %d",
        LegacySightTitleName(title), anchors, static_cast<unsigned long long>(global - base),
        static_cast<unsigned long long>(value), static_cast<unsigned long long>(flags), kReachAimTraceMode);
}

// Seated, a pass that meets our own vehicle continues 5 cm past the contact, at most this many passes
// (Reach's kReachSightSelfPasses / kReachSightSelfStepWu).
inline constexpr int kLegacySightSelfPasses = 3;
inline constexpr float kLegacySightSelfStepWu = 0.05f * kReachWorldUnitsPerMeter;
// The parent link in an object's data: +0x10 in both titles (kHalo3ObjectParentOffset,
// kOdstObjectParentOffset; the seat samplers already walk it).
inline constexpr size_t kLegacyObjectParentOffset = 0x10;
using LegacyObjectDataFn = const uint8_t* (*)(uint32_t handle);

// The root of an object's parent chain (the vehicle a seated unit rides; on foot the unit itself), walked
// through the title's own object table: objectData maps a handle the table still holds to that object's
// data, or null. -1 when any link is stale or the chain is deeper than any vehicle. Simulation thread;
// guarded, and kept free of C++ objects so __try is legal.
int32_t LegacySightRootOf(LegacyObjectDataFn objectData, int32_t object)
{
    if (object == -1 || !objectData)
        return -1;
    __try
    {
        int32_t current = object;
        for (int depth = 0; depth < 8; ++depth)
        {
            const uint8_t* data = objectData(static_cast<uint32_t>(current));
            if (!data)
                return -1;
            int32_t parent = -1;
            memcpy(&parent, data + kLegacyObjectParentOffset, sizeof(parent));
            if (parent == -1)
                return current;
            current = parent;
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
    }
    return -1;
}

// Simulation thread, inside the title's per-unit contact update (counted by that hook's callbacks, whose
// teardown drains before the collision trampoline is freed). Traces the local unit's published intent ray
// at most every 30 ms: on foot, or in a seat whose vehicle the aim path named.
//
// SEATED, the ray starts at the rendered eye INSIDE the vehicle, so it must not stop on the vehicle itself
// (user report 2026-09-30: "in the Banshee it felt like my intent ray was colliding with the vehicle"). It
// ignores the vehicle's ROOT in the first slot -- the slot both engines' own crosshair traces fill (Reach's
// engine query is (root, -1, -1); Halo 3's ignores the unit's root) -- then the occupant and the seat's own
// parent when that is not the root (a Warthog's turret is a vehicle of its own); and any other part of
// the same vehicle the ray still meets (another occupant, an attached gun) is looked past, as Reach's trace
// does. Before this the vehicle sat only in the second slot and the hit object was never read.
void LegacySightTraceTick(GameTitle title, uint32_t unit, int32_t localUnit, bool onFoot,
                          const LegacyWorldCollisionFeature& collision, LegacyObjectDataFn objectData)
{
    auto& state = LegacySight(title);
    const uint64_t flags = state.flags.load(std::memory_order_acquire);
    if (!flags || localUnit == -1 || unit != static_cast<uint32_t>(localUnit) ||
        TitleAdapter_GetActiveTitle() != title)
        return;
    const uint64_t now = GetTickCount64();
    if (Sight_TraceFaulted(title) || !Sight_TraceDue(title, now))
        return;
    const uint32_t generation = TitleAdapter_GetGeneration(title);
    SightRay ray{};
    if (!Sight_ReadRay(title, generation, kSightRayFreshMs, ray))
        return;
    // Seated is the unit's own parent link. unit_in_vehicle (the caller's onFoot) answers false in a
    // stationary turret, which then traced as on foot -- with nothing but the unit ignored, into the turret.
    // When the table cannot be walked, the caller's answer stands.
    int32_t root = LegacySightRootOf(objectData, static_cast<int32_t>(unit));
    const bool seated = root != -1 ? root != static_cast<int32_t>(unit) : !onFoot;
    if (seated && ray.ignoreB == -1)
    {
        // A seat the aim path did not name: no trace (the aim keeps its fixed-distance steer).
        state.seatUnnamed.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const auto original = reinterpret_cast<LegacyCollisionTestVectorFn>(collision.original);
    if (!original || !collision.originalIsVectorTest || !collision.installed.load(std::memory_order_acquire))
        return;
    int32_t ignoreA = static_cast<int32_t>(unit), ignoreB = -1, ignoreC = -1;
    if (seated)
    {
        if (root == -1 || root == static_cast<int32_t>(unit))
        {
            // The walk failed: the published seat chain's top (the carrier when the seat's parent is
            // attached to one, else the parent).
            root = ray.ignoreC != -1 ? ray.ignoreC : ray.ignoreB;
            state.rootFallbacks.fetch_add(1, std::memory_order_relaxed);
        }
        ignoreA = root;
        ignoreB = static_cast<int32_t>(unit);
        ignoreC = ray.ignoreB != root ? ray.ignoreB : -1;
        state.seatedTraces.fetch_add(1, std::memory_order_relaxed);
    }
    const float length = kSightTraceLengthWu;
    // A hit object that is part of our own vehicle: its root is our root, or -- when that walk fails, as it did for
    // the unit on the fallback path above -- it is one of the seat chain's own handles.
    const auto isOurs = [&](int32_t object) {
        const int32_t hitRoot = LegacySightRootOf(objectData, object);
        return object != -1 && (hitRoot != -1 ? hitRoot == root
            : object == root || object == ignoreB || object == ignoreC || object == ray.ignoreB ||
              object == ray.ignoreC);
    };
    // Native collision initializes fields through +63 (type +0, fraction +4, point +8, object +0x40).
    alignas(16) uint8_t result[0x80]{};
    bool hit = false;
    float rangeWu = 0.0f;
    float alongWu = kSightTraceSkipWu;   // where this pass starts, measured from the eye
    LARGE_INTEGER before{}, after{};
    QueryPerformanceCounter(&before);
    for (int pass = 0; pass < (seated ? kLegacySightSelfPasses : 1); ++pass)
    {
        const float remainingWu = length - (alongWu - kSightTraceSkipWu);
        if (!(remainingWu > 0.0f))
            break;
        const float start[3]{ray.head[0] + ray.dir[0] * alongWu, ray.head[1] + ray.dir[1] * alongWu,
                             ray.head[2] + ray.dir[2] * alongWu};
        const float vector[3]{ray.dir[0] * remainingWu, ray.dir[1] * remainingWu, ray.dir[2] * remainingWu};
        memset(result, 0, sizeof(result));
        uint8_t returned = 0;
        const bool completed = ReachSightGuardedTrace(original, flags, kReachAimTraceMode, start, vector,
                                                      ignoreA, ignoreB, ignoreC, result, returned, false);
        if (!completed)
        {
            Sight_NoteTraceFault(title);
            LOG("SIGHT %s: the aim trace raised a native fault; it stays off until the title reinstalls",
                LegacySightTitleName(title));
            return;
        }
        float fraction = 0.0f;
        memcpy(&fraction, result + 4, sizeof(fraction));
        hit = returned && std::isfinite(fraction) && fraction >= 0.0f && fraction <= 1.0f;
        if (!hit)
            break;
        rangeWu = alongWu + fraction * remainingWu;
        if (!seated)
            break;
        int32_t object = -1;
        memcpy(&object, result + 0x40, sizeof(object));
        if (!isOurs(object))
            break;
        // A part of our own vehicle the ignores did not cover: not a target. Look past it.
        state.selfHits.fetch_add(1, std::memory_order_relaxed);
        hit = false;
        rangeWu = 0.0f;
        alongWu += fraction * remainingWu + kLegacySightSelfStepWu;
    }
    QueryPerformanceCounter(&after);
    Sight_PublishTrace(title, generation, hit, hit ? rangeWu : 0.0f,
                       static_cast<uint64_t>(after.QuadPart - before.QuadPart));
#if HALOMCCVR_DEV_DIAGNOSTICS
    // [assess-only begin]
    // DEV ASSESSMENT (honest_reticle_assess): the game's own aim ray from its shot origin (the engine eye, or the
    // rendered eye for an ODST passenger whose shot O6 moves there), in this same tick, with these same ignores.
    // Only while the key published one; nothing the aim reads moves.
    SightAssessRay assess{};
    if (Sight_ReadAssessRay(title, generation, kSightRayFreshMs, assess))
    {
        bool assessHit = false;
        float assessRangeWu = 0.0f;
        if (!SightAssessTraceVector(original, flags, kReachAimTraceMode, assess, length, kSightTraceSkipWu,
                                    kLegacySightSelfStepWu, seated ? kLegacySightSelfPasses : 1, ignoreA, ignoreB,
                                    ignoreC, isOurs, assessHit, assessRangeWu))
        {
            Sight_NoteTraceFault(title);
            LOG("SIGHT %s: the assessment trace raised a native fault; the trace stays off until the title "
                "reinstalls", LegacySightTitleName(title));
            return;
        }
        Sight_PublishAssessTrace(title, generation, assessHit, assessHit ? assessRangeWu : 0.0f);
    }
    // [assess-only end]
#endif
}

// ---- engine-crosshair check (dev): does the TITLE'S OWN crosshair trace pass through the traced point? ----
// Halo 3 and ODST trace their crosshair with the resolved flags. Our own trace calls the ORIGINAL through
// the trampoline, so every query the collision hook sees with those flags is the game's. While our aim is
// converged each one is compared with the point the intent ray hit (H + u * range): how far that point sits
// off the game's line, where the line starts relative to the engine eye we solved with, and its angle from
// the eye -> point direction. Pass = the point ~0 mm off the line.
#if HALOMCCVR_DEV_DIAGNOSTICS
struct LegacyEngineTraceWindow
{
    std::atomic<uint32_t> siteCalls{0};
    std::atomic<uint64_t> siteFlags{0};
    std::atomic<uint32_t> seen{0}, compared{0};
    // Converged queries skipped because a natural cause (a seat's latch, a turret's slew, a stall) held the game's
    // aim off the point: the mark stays on through those since 2026-10-01, so they are not misses of the solve.
    std::atomic<uint32_t> heldOff{0};
    std::atomic<float> missMm{0.0f}, maxMissMm{0.0f}, rangeM{0.0f}, angleDeg{0.0f};
    std::atomic<float> startAlongMm{0.0f}, startRightMm{0.0f}, startUpMm{0.0f};
};
LegacyEngineTraceWindow g_legacyEngineTrace[2];   // [0] Halo 3, [1] ODST

// ---- the turret's own shots (dev, Halo 3): does the GUN's shot line pass through the traced point? ----------------
// In a gun-slaved seat (flag bit 7) the camera is the gun's marker, so the solve's eye is that marker -- but the shot
// leaves the BARREL, along the gun's own aim. The firing call's final (origin, direction) is compared with the point
// the intent ray hit: the miss off the shot's line, the range, the shot's start relative to the solve's eye (a large
// offset means a parallel miss no aim angle can fix -- the case for a muzzle-origin solve), and the shot's angle off
// the eye -> point direction. Shots while the gun is still slewing onto the writes are counted apart: those miss by
// design. The input thread publishes the seat (the turret vehicle that fires) every gun-slaved poll.
struct LegacyTurretShotWindow
{
    std::atomic<uint32_t> shots{0}, settled{0}, noPoint{0};
    std::atomic<uint32_t> maxMissSettledBits{0}, maxMissSlewingBits{0};
    std::atomic<float> missMm{0.0f}, rangeM{0.0f}, angleDeg{0.0f};
    std::atomic<float> startAlongMm{0.0f}, startRightMm{0.0f}, startUpMm{0.0f};
};
LegacyTurretShotWindow g_legacyTurretShots;
std::atomic<int32_t> g_legacyTurretShotParent{-1};
std::atomic<uint64_t> g_legacyTurretShotSeatMs{0};
std::atomic<bool> g_legacyTurretShotGunSettled{false};
#endif

// Input thread, every poll of a Halo 3 gun-slaved seat: the turret vehicle (it fires the gun) and whether the gun has
// settled on the writes.
void LegacySightPublishTurretSeat(int32_t parentHandle, bool gunSettled)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    g_legacyTurretShotParent.store(parentHandle, std::memory_order_relaxed);
    g_legacyTurretShotGunSettled.store(gunSettled, std::memory_order_relaxed);
    g_legacyTurretShotSeatMs.store(GetTickCount64(), std::memory_order_release);
#else
    (void)parentHandle; (void)gunSettled;
#endif
}

// The firing call (Halo 3's native weapon fire, after its final ray): one shot of `unit`.
void LegacySightNoteTurretShot(GameTitle title, uint32_t unit, const float* origin, const float* direction)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    if (title != GameTitle::Halo3 || !origin || !direction)
        return;
    const uint64_t at = g_legacyTurretShotSeatMs.load(std::memory_order_acquire);
    const uint64_t now = GetTickCount64();
    const int32_t parent = g_legacyTurretShotParent.load(std::memory_order_relaxed);
    if (!at || now < at || now - at > 250 || parent == -1 || static_cast<int32_t>(unit) != parent)
        return;
    auto& w = g_legacyTurretShots;
    w.shots.fetch_add(1, std::memory_order_relaxed);
    const uint32_t generation = TitleAdapter_GetGeneration(title);
    SightRay ray{};
    SightTrace trace{};
    if (!Sight_ReadRay(title, generation, kSightRayFreshMs, ray) ||
        !Sight_ReadTrace(title, generation, kSightFreshMs, trace))
    {
        w.noPoint.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const float dl = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    if (!std::isfinite(dl) || dl < 1.0e-6f)
        return;
    const float d[3]{direction[0] / dl, direction[1] / dl, direction[2] / dl};
    const float range = trace.AimRangeWu();
    const float p[3]{ray.head[0] + ray.dir[0] * range, ray.head[1] + ray.dir[1] * range,
                     ray.head[2] + ray.dir[2] * range};
    const float v[3]{p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]};
    const float c[3]{v[1] * d[2] - v[2] * d[1], v[2] * d[0] - v[0] * d[2], v[0] * d[1] - v[1] * d[0]};
    const float toMm = 3048.0f;   // 1 world unit = 3.048 m
    const float miss = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) * toMm;
    const bool settled = g_legacyTurretShotGunSettled.load(std::memory_order_relaxed);
    if (settled)
        w.settled.fetch_add(1, std::memory_order_relaxed);
    aim_caller::AtomicMaxFloat(settled ? w.maxMissSettledBits : w.maxMissSlewingBits, miss);
    w.missMm.store(miss, std::memory_order_relaxed);
    w.rangeM.store(std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) * 3.048f, std::memory_order_relaxed);
    const float a[3]{p[0] - ray.eye[0], p[1] - ray.eye[1], p[2] - ray.eye[2]};
    const float al = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float dot = al > 1.0e-6f ? (a[0] * d[0] + a[1] * d[1] + a[2] * d[2]) / al : 1.0f;
    dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
    w.angleDeg.store(std::acos(dot) * 57.29578f, std::memory_order_relaxed);
    float right[3]{d[1], -d[0], 0.0f};
    float rl = std::sqrt(right[0] * right[0] + right[1] * right[1]);
    if (!(rl > 1.0e-4f)) { right[0] = 1.0f; right[1] = 0.0f; rl = 1.0f; }
    right[0] /= rl; right[1] /= rl;
    const float up[3]{right[1] * d[2] - right[2] * d[1], right[2] * d[0] - right[0] * d[2],
                      right[0] * d[1] - right[1] * d[0]};
    const float se[3]{origin[0] - ray.eye[0], origin[1] - ray.eye[1], origin[2] - ray.eye[2]};
    w.startAlongMm.store((se[0] * d[0] + se[1] * d[1] + se[2] * d[2]) * toMm, std::memory_order_relaxed);
    w.startRightMm.store((se[0] * right[0] + se[1] * right[1] + se[2] * right[2]) * toMm, std::memory_order_relaxed);
    w.startUpMm.store((se[0] * up[0] + se[1] * up[1] + se[2] * up[2]) * toMm, std::memory_order_relaxed);
#else
    (void)title; (void)unit; (void)origin; (void)direction;
#endif
}

void LegacySightNoteEngineTrace(GameTitle title, uint64_t flags, const float* start, const float* vector,
                                uintptr_t caller)
{
#if HALOMCCVR_DEV_DIAGNOSTICS
    // Cheapest gate first: this runs for every collision query the game makes.
    if (title != GameTitle::Halo3 && title != GameTitle::Halo3ODST)
        return;
    auto& state = LegacySight(title);
    auto& window = g_legacyEngineTrace[title == GameTitle::Halo3ODST ? 1 : 0];
    for (const auto& site : state.siteReturn)
    {
        const uintptr_t at = site.load(std::memory_order_relaxed);
        if (at && at == caller)
        {
            window.siteCalls.fetch_add(1, std::memory_order_relaxed);
            window.siteFlags.store(flags, std::memory_order_relaxed);
            break;
        }
    }
    const uint64_t resolved = state.flags.load(std::memory_order_acquire);
    if (!resolved || flags != resolved || !start || !vector)
        return;
    window.seen.fetch_add(1, std::memory_order_relaxed);
    if (TitleAdapter_GetActiveTitle() != title || !Sight_ConvergedLive(title))
        return;
    if (ConvergeNaturalHeld(title))
    {
        window.heldOff.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint32_t generation = TitleAdapter_GetGeneration(title);
    SightRay ray{};
    SightTrace trace{};
    if (!Sight_ReadRay(title, generation, kSightRayFreshMs, ray) ||
        !Sight_ReadTrace(title, generation, kSightFreshMs, trace))
        return;
    const float vl = std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    if (!std::isfinite(vl) || vl < 1.0e-6f)
        return;
    const float d[3]{vector[0] / vl, vector[1] / vl, vector[2] / vl};
    const float aimRange = trace.AimRangeWu();
    const float p[3]{ray.head[0] + ray.dir[0] * aimRange, ray.head[1] + ray.dir[1] * aimRange,
                     ray.head[2] + ray.dir[2] * aimRange};
    const float w[3]{p[0] - start[0], p[1] - start[1], p[2] - start[2]};
    const float c[3]{w[1] * d[2] - w[2] * d[1], w[2] * d[0] - w[0] * d[2], w[0] * d[1] - w[1] * d[0]};
    const float toMm = 3048.0f;   // 1 world unit = 3.048 m
    // The eye -> point direction the solve aimed along, against the line the game traced.
    const float a[3]{p[0] - ray.eye[0], p[1] - ray.eye[1], p[2] - ray.eye[2]};
    const float al = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float dot = al > 1.0e-6f ? (a[0] * d[0] + a[1] * d[1] + a[2] * d[2]) / al : 1.0f;
    dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
    // The line's start relative to the solve's eye, in the line's frame (along, horizontal right, up).
    float right[3]{d[1], -d[0], 0.0f};
    float rl = std::sqrt(right[0] * right[0] + right[1] * right[1]);
    if (!(rl > 1.0e-4f)) { right[0] = 1.0f; right[1] = 0.0f; rl = 1.0f; }
    right[0] /= rl; right[1] /= rl;
    const float up[3]{right[1] * d[2] - right[2] * d[1], right[2] * d[0] - right[0] * d[2],
                      right[0] * d[1] - right[1] * d[0]};
    const float se[3]{start[0] - ray.eye[0], start[1] - ray.eye[1], start[2] - ray.eye[2]};
    const float miss = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) * toMm;
    window.missMm.store(miss, std::memory_order_relaxed);
    if (miss > window.maxMissMm.load(std::memory_order_relaxed))
        window.maxMissMm.store(miss, std::memory_order_relaxed);
    window.rangeM.store(std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * 3.048f, std::memory_order_relaxed);
    window.angleDeg.store(std::acos(dot) * 57.29578f, std::memory_order_relaxed);
    window.startAlongMm.store((se[0] * d[0] + se[1] * d[1] + se[2] * d[2]) * toMm, std::memory_order_relaxed);
    window.startRightMm.store((se[0] * right[0] + se[1] * right[1] + se[2] * right[2]) * toMm,
                              std::memory_order_relaxed);
    window.startUpMm.store((se[0] * up[0] + se[1] * up[1] + se[2] * up[2]) * toMm, std::memory_order_relaxed);
    window.compared.fetch_add(1, std::memory_order_release);
#else
    (void)title; (void)flags; (void)start; (void)vector; (void)caller;
#endif
}

// Cold worker, every 5 s: how the servo target was chosen per title, and why a title does not trace.
void LegacySightReport()
{
    static uint64_t s_last = 0;
    const uint64_t now = GetTickCount64();
    if (now - s_last < 5000)
        return;
    s_last = now;
    for (const GameTitle title : {GameTitle::Halo3, GameTitle::Halo3ODST, GameTitle::Halo4})
    {
        auto& state = LegacySight(title);
        const uint64_t converged = state.converged.exchange(0, std::memory_order_relaxed);
        const uint64_t fallback = state.fallback.exchange(0, std::memory_order_relaxed);
        const uint64_t seatedTraces = state.seatedTraces.exchange(0, std::memory_order_relaxed);
        const uint64_t selfHits = state.selfHits.exchange(0, std::memory_order_relaxed);
        const uint64_t seatUnnamed = state.seatUnnamed.exchange(0, std::memory_order_relaxed);
        const uint64_t rootFallbacks = state.rootFallbacks.exchange(0, std::memory_order_relaxed);
        if (!converged && !fallback && !seatedTraces && !seatUnnamed)
            continue;
        const char* why = state.why.load(std::memory_order_acquire);
        LOG("SIGHT %s aim servo: %llu targets converged on the traced point, %llu on the fixed-distance steer "
            "(no fresh trace)%s%s; seated traces %llu (looked past our own vehicle %llu times, %llu from the "
            "published seat chain), %llu seats untraced (no vehicle named)", LegacySightTitleName(title),
            static_cast<unsigned long long>(converged), static_cast<unsigned long long>(fallback),
            why ? "; trace off: " : "", why ? why : "", static_cast<unsigned long long>(seatedTraces),
            static_cast<unsigned long long>(selfHits), static_cast<unsigned long long>(rootFallbacks),
            static_cast<unsigned long long>(seatUnnamed));
    }
#if HALOMCCVR_DEV_DIAGNOSTICS
    for (int slot = 0; slot < 2; ++slot)
    {
        auto& window = g_legacyEngineTrace[slot];
        const uint32_t sites = window.siteCalls.exchange(0, std::memory_order_relaxed);
        const uint32_t seen = window.seen.exchange(0, std::memory_order_relaxed);
        const uint32_t compared = window.compared.exchange(0, std::memory_order_acquire);
        const uint32_t heldOff = window.heldOff.exchange(0, std::memory_order_relaxed);
        if (!seen && !sites)
            continue;
        LOG("SIGHT %s engine crosshair: %u queries from the crosshair call site(s) (last flags 0x%llX); %u "
            "with the resolved flags, %u while converged (%u more skipped: a seat limit, turret or stall held the "
            "aim off the point); the point aimed at is %.0f mm off the game's line "
            "(max %.0f) at %.2f m; line vs eye->point %.2f deg; its start minus the solve's eye "
            "[along,right,up] %.0f,%.0f,%.0f mm", slot ? "ODST" : "Halo 3", sites,
            static_cast<unsigned long long>(window.siteFlags.load()), seen, compared, heldOff,
            double(window.missMm.load()), double(window.maxMissMm.exchange(0.0f)), double(window.rangeM.load()),
            double(window.angleDeg.load()), double(window.startAlongMm.load()),
            double(window.startRightMm.load()), double(window.startUpMm.load()));
    }
    {
        auto& w = g_legacyTurretShots;
        const uint32_t shots = w.shots.exchange(0, std::memory_order_relaxed);
        if (shots)
            LOG("SIGHT Halo 3 turret shots (dev): %u shots of the gun-slaved seat's gun (%u with the gun settled on "
                "the writes, %u with no fresh traced point); the point is %.0f mm off the shot's line (max settled "
                "%.0f, max while slewing %.0f) at %.2f m; the shot's line vs eye->point %.2f deg; its start minus "
                "the solve's eye (the gun's camera marker) [along,right,up] %.0f,%.0f,%.0f mm",
                shots, w.settled.exchange(0, std::memory_order_relaxed),
                w.noPoint.exchange(0, std::memory_order_relaxed), double(w.missMm.load()),
                double(aim_caller::BitsToFloat(w.maxMissSettledBits.exchange(0, std::memory_order_relaxed))),
                double(aim_caller::BitsToFloat(w.maxMissSlewingBits.exchange(0, std::memory_order_relaxed))),
                double(w.rangeM.load()), double(w.angleDeg.load()), double(w.startAlongMm.load()),
                double(w.startRightMm.load()), double(w.startUpMm.load()));
    }
#endif
}
