// Included in game.cpp (inside its anonymous namespace) right after halo3_assist_witness.inl: it reads the fire lane's
// thread state (t_halo3Fire, t_legacyFirePredicted, g_halo3FiringWeapon, g_halo3IndependentShot) and object access
// (Halo3MeleeSelectionObject), and is fed from halo3_independent_shots.inl, halo3_collision_dispatch.inl,
// legacy_sight_trace.inl, the compositor (Game_DevNoteTracedReticle) and the XInput hook (Game_DevNoteFireTrigger).
//
// THE SHOT IMPACT WITNESS, Halo 3 on foot (dev builds only; 2026-10-03). The SHOT LAG [muzzle] line is circular: a
// muzzle shot's direction is BUILT as normalize(P - muzzle), so it passes through P by construction. This witness
// asks the game's own objects where each local infantry round went (shot_impact_logic.h):
//
//   SHOT IMPACT Halo 3 [<muzzle|native>; <predicted|authoritative>] (dev): fires, projectiles; ROUND LINE vs P (the
//   spawned object's position and VELOCITY against P, per eye-speed bin, along/across travel); the native's turn after
//   our line (forward vs the direction we handed it); velocity off the forward (what the shooter's motion added, and
//   how much of the eye's speed that is); speed, flight time, range; later ticks (the same object re-read: moved off
//   its spawn line?); IMPACT vs P (the hit point of the projectile's own collision query)
//
//   SHOT RETICLE Halo 3 (dev): the reticle the compositor last drew (its ray mapped to the world through the aim
//   path's own map, from the rendered eye of its frame, at the range it was drawn for) against P; how old P was at
//   the fire (its hand, its camera), whether P's frame was older than the reticle's; and RT-to-fire latency.
//
// HOW THE GAME SPAWNS A ROUND (pinned retail halo3.dll; read with capstone, every RVA checked against the bytes):
//   * the fire routine 0x3683A0 (pdata to 0x369FB9) builds a placement block on its frame (rbp+0xD0) through
//     0x341508, writes position (+0x1C), forward (+0x28, after spread 0x109648) and translational velocity (+0x40),
//     then calls object_new 0x341AE8 at 0x369720 (return 0x369725) and keeps the handle at [rsp+0x4C];
//   * object_new copies the block into the object: +0x1C -> +0x50 (position), +0x28 -> +0x5C (forward), +0x34 -> +0x68
//     (up), +0x40 -> +0x74 (velocity) (0x341D19..0x341D56); the projectile type (index 5 of the type table at
//     0x815160, named "projectile") constructs at 0x384D3C, which adds forward * initial speed into +0x74
//     (0x384FBE..0x384FE6);
//   * THE INHERITED VELOCITY: the placement velocity is forward * max(0, V.forward) with V the shot line's own
//     velocity output (0x3524B0's fourth argument, [rbp+0x28]; 0x368BD6..0x368C14), PLUS -- when the owner's object
//     flag +0xA8 bit 3 is set (0x368B97) and 0x397E4C answers -- the velocity of the physics body the owner rides
//     (0x397EA8: v + w x r) with its forward part removed (0x369419..0x3694AF, flag [rsp+0x45]). So a gun carried by a
//     vehicle (its owner is the vehicle) inherits the vehicle's full sideways velocity; whether an on-foot biped ever
//     carries the flag is not static -- this witness measures it ("velocity off the forward").
//   * the firing assist 0x13BAD0 (called at 0x368DFE) traces 128 wu along the CAMERA ray (0x212198 at 0x13BC08; the
//     muzzle lane substitutes the muzzle ray there, Halo3IndependentCameraDetour) and turns the shot toward the hit
//     minus the shot origin; on foot that turn is uncapped. Its no-target path does not subtract the shooter's
//     velocity; its target path leads the TARGET's velocity (0x13BDBC..0x13BE48).
//
// ANCHORS (each unique in the module, matched where pdata says it belongs; offsets decoded from the bytes, never
// assumed), and the per-shot proof:
//   * object_new's prologue -> its entry; its placement copy (four movsd/mov pairs) -> the six offsets above, and
//     RtlLookupFunctionEntry must place the copy inside object_new;
//   * the fire routine's call `lea rcx, [rbp+..]; call object_new; ...; mov [rsp+..], eax` -> the call must target
//     object_new and lie inside the dual lane's fire routine (its pinned entry; pdata);
//   * object_new's type-table load `lea rbx, [image base]; ...; mov rbx, [rbx+rax*8+table]` -> the table; entry 5's
//     name must read "projectile";
//   * every shot: the new object's position and forward must equal the placement's bitwise, and its velocity the
//     placement's plus something along the forward (shot_impact::SpawnProven) -- else the shot is "unproven" and
//     measured not at all.
// The hook forwards its one argument exactly: `py -3 tools/audit_caller_area.py halo3.dll 341AE8:1:i` is clean (no
// caller-area slot touched; 47 native callers, none relies on a volatile register across the call). It filters by
// return address and touches nothing it does not read. ADDR-HYGIENE: dev-only (a witness; decides nothing).

#if HALOMCCVR_DEV_DIAGNOSTICS
using Halo3ObjectNewFn = int32_t(__fastcall*)(void*);

struct Halo3ImpactWitness
{
    void* target = nullptr;
    Halo3ObjectNewFn original = nullptr;
    std::atomic<uint32_t> callbacks{0};
    uintptr_t base = 0;
    uintptr_t fireReturn = 0;
    // Decoded from object_new's copy of the placement block.
    uint32_t placePos = 0, placeFwd = 0, placeVel = 0, objPos = 0, objFwd = 0, objVel = 0;
    uint8_t projectileType = 0xFF;
    std::atomic<bool> ready{false};
    std::atomic<int> armed{0};
    std::atomic<uint32_t> faults{0};
    // Everything below is under `lock`.
    SRWLOCK lock = SRWLOCK_INIT;
    shot_impact::Tracker tracker;
    shot_impact::Window windows[4];   // [muzzle][predicted]
    shot_impact::ReticleWindow reticle;
    shot_impact::TriggerLatch trigger;
    uint32_t nested = 0, ourLineThroughP = 0, ourLineOffP = 0;
};
Halo3ImpactWitness g_halo3Impact;

// The reticle the compositor last drew (Game_DevNoteTracedReticle): tracking space, its prepared frame.
struct Halo3ImpactReticle
{
    SRWLOCK lock = SRWLOCK_INIT;
    bool valid = false;
    uint64_t serial = 0, stampUs = 0;
    float center[3]{}, dir[3]{};
    float rangeM = 0.0f;
    bool hit = false;
};
Halo3ImpactReticle g_halo3ImpactReticle;

// One native fire on this thread: opened by the fire detour, filled by its shot line and its object_new.
struct Halo3ImpactFire
{
    bool open = false;
    bool lineSeen = false;
    bool local = false;
    bool muzzle = false;
    int predicted = -1;
    uint32_t projectiles = 0;
    float lineOrigin[3]{}, lineDir[3]{};
    bool pointKnown = false;
    shot_lag::IntentPoint point{};
};
thread_local Halo3ImpactFire t_halo3ImpactFire;

namespace
{
int Halo3ImpactGroup(const Halo3ImpactFire& fire)
{
    return (fire.muzzle ? 2 : 0) + (fire.predicted > 0 ? 1 : 0);
}

// Guarded reads (no C++ objects with destructors here: SEH only).
bool Halo3ImpactCopy3(const void* from, float out[3])
{
    __try
    {
        memcpy(out, from, sizeof(float) * 3);
        return shot_lag::Finite3(out);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// The projectile object's position / forward / velocity, or false when the handle no longer names a live projectile.
bool Halo3ImpactReadObject(uint32_t handle, float pos[3], float fwd[3], float vel[3])
{
    const auto& w = g_halo3Impact;
    __try
    {
        const auto* object = Halo3MeleeSelectionObject(handle, w.projectileType);
        if (!object)
            return false;
        memcpy(pos, object + w.objPos, sizeof(float) * 3);
        memcpy(fwd, object + w.objFwd, sizeof(float) * 3);
        memcpy(vel, object + w.objVel, sizeof(float) * 3);
        return shot_lag::Finite3(pos) && shot_lag::Finite3(fwd) && shot_lag::Finite3(vel);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        g_halo3Impact.faults.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
}

// The rendered eye of a prepared frame (the camera ring, PublishCamFrame), by serial.
bool Halo3ImpactFrameBySerial(uint64_t serial, shot_lag::EyeFrame& out)
{
    if (!serial)
        return false;
    shot_lag::EyeFrame frames[kCamFrameSlots]{};
    const size_t count = ReadCamFrames(frames);
    for (size_t i = 0; i < count; ++i)
        if (frames[i].valid && frames[i].serial == serial)
        {
            out = frames[i];
            return true;
        }
    return false;
}

// The local fire's P (the intent the shot sites read), fresh and this generation's.
bool Halo3ImpactReadPoint(uint64_t nowUs, shot_lag::IntentPoint& out)
{
    SightIntent intent{};
    const uint32_t generation = TitleAdapter_GetGeneration(GameTitle::Halo3);
    if (!Sight_ReadIntent(GameTitle::Halo3, intent) || !generation || intent.p.generation != generation ||
        !intent.p.stampUs || nowUs < intent.p.stampUs || nowUs - intent.p.stampUs > 250000ull ||
        !shot_lag::Finite3(intent.p.point))
        return false;
    out = intent.p;
    return true;
}

// The reticle the player saw at this fire, against P (and P's age), into the reticle window. Under the lock.
void Halo3ImpactMeasureReticle(const shot_lag::IntentPoint& p, uint64_t nowUs)
{
    auto& w = g_halo3Impact;
    auto& r = w.reticle;
    Halo3ImpactReticle seen{};
    AcquireSRWLockShared(&g_halo3ImpactReticle.lock);
    seen.valid = g_halo3ImpactReticle.valid;
    seen.serial = g_halo3ImpactReticle.serial;
    seen.stampUs = g_halo3ImpactReticle.stampUs;
    memcpy(seen.center, g_halo3ImpactReticle.center, sizeof(seen.center));
    memcpy(seen.dir, g_halo3ImpactReticle.dir, sizeof(seen.dir));
    seen.rangeM = g_halo3ImpactReticle.rangeM;
    seen.hit = g_halo3ImpactReticle.hit;
    ReleaseSRWLockShared(&g_halo3ImpactReticle.lock);

    const float handAgeMs = float(nowUs - p.stampUs) / 1000.0f;
    const bool eyeKnown = p.eyeStampUs && nowUs >= p.eyeStampUs;
    const float eyeAgeMs = eyeKnown ? float(nowUs - p.eyeStampUs) / 1000.0f : 0.0f;
    const int64_t framesOlder = (seen.valid && seen.serial && p.serial)
        ? static_cast<int64_t>(seen.serial) - static_cast<int64_t>(p.serial) : 0;
    r.AddPointAge(handAgeMs, eyeKnown, eyeAgeMs, framesOlder);

    const float scale = Sight_UnitsPerVrMetre(GameTitle::Halo3);
    if (!seen.valid || !seen.hit || !seen.stampUs || nowUs < seen.stampUs || nowUs - seen.stampUs > 100000ull ||
        !std::isfinite(scale) || !(scale > 0.0f) || !(seen.rangeM > 0.0f))
    {
        ++r.noReticle;
        return;
    }
    shot_lag::EyeFrame frame{};
    if (!Halo3ImpactFrameBySerial(seen.serial, frame))
    {
        ++r.noFrame;
        return;
    }
    float dirWorld[3]{};
    if (!shot_impact::TrackingDirToWorld(seen.dir, g_gameYawRef, g_headYawRef, g_yawSign.load(),
                                         g_pitchSign.load(), dirWorld))
    {
        ++r.noReticle;
        return;
    }
    const float rangeWu = seen.rangeM * scale;
    const float offset[3]{frame.head[0] + dirWorld[0] * rangeWu - p.point[0],
                          frame.head[1] + dirWorld[1] * rangeWu - p.point[1],
                          frame.head[2] + dirWorld[2] * rangeWu - p.point[2]};
    const float keyDelta[3]{seen.center[0] - frame.key[0], seen.center[1] - frame.key[1],
                            seen.center[2] - frame.key[2]};
    r.AddOffset(shot_impact::Decompose(offset, p.dir, p.eyeVelValid ? p.eyeVel : nullptr),
                float(nowUs - seen.stampUs) / 1000.0f, shot_lag::Length(keyDelta) * 1000.0f);
}
} // namespace

// The fire detour's entry and exit (Halo3IndependentFireDetour). A fire inside a fire is not measured.
void Halo3ImpactFireBegin(uint8_t predicted)
{
    if (!g_halo3Impact.ready.load(std::memory_order_acquire))
        return;
    auto& fire = t_halo3ImpactFire;
    if (fire.open)
    {
        AcquireSRWLockExclusive(&g_halo3Impact.lock);
        ++g_halo3Impact.nested;
        ReleaseSRWLockExclusive(&g_halo3Impact.lock);
        return;
    }
    fire = Halo3ImpactFire{};
    fire.open = true;
    fire.predicted = predicted ? 1 : 0;
}

void Halo3ImpactFireEnd()
{
    auto& fire = t_halo3ImpactFire;
    if (!fire.open)
        return;
    if (fire.local && fire.lineSeen)
    {
        AcquireSRWLockExclusive(&g_halo3Impact.lock);
        g_halo3Impact.windows[Halo3ImpactGroup(fire)].AddFire(fire.projectiles);
        ReleaseSRWLockExclusive(&g_halo3Impact.lock);
    }
    fire = Halo3ImpactFire{};
}

// The fire's shot line as it leaves the shot-line call (0x368B97): what we handed the game. `muzzle`: the muzzle
// lane's ray. Only the local player's own weapon on foot (the gun's owner is the unit itself) is measured.
void Halo3ImpactNoteShotLine(uint32_t unit, const float origin[3], const float direction[3], bool muzzle)
{
    auto& fire = t_halo3ImpactFire;
    if (!fire.open || fire.lineSeen || !origin || !direction)
        return;
    fire.lineSeen = true;
    fire.local = t_halo3Fire == shot_lag::kFireLocal && unit != UINT32_MAX &&
                 Halo3WeaponOwner(g_halo3FiringWeapon) == unit;
    if (!fire.local || !shot_lag::Finite3(origin) || !shot_lag::Finite3(direction))
    {
        fire.local = false;
        return;
    }
    fire.muzzle = muzzle;
    memcpy(fire.lineOrigin, origin, sizeof(fire.lineOrigin));
    memcpy(fire.lineDir, direction, sizeof(fire.lineDir));
    const uint64_t nowUs = Sight_NowUs();
    fire.pointKnown = Halo3ImpactReadPoint(nowUs, fire.point);
    auto& w = g_halo3Impact;
    AcquireSRWLockExclusive(&w.lock);
    const float latency = w.trigger.Fire(nowUs);
    if (latency >= 0.0f)
        w.reticle.AddTrigger(latency);
    ++w.reticle.fires;
    if (!fire.pointKnown)
        ++w.reticle.noPoint;
    else
    {
        Halo3ImpactMeasureReticle(fire.point, nowUs);
        // The circular check: does the line we handed the game pass through this P (the muzzle lane built it so)?
        if (muzzle)
        {
            const shot_lag::ShotLagSample ours =
                shot_lag::MeasureShotLag(origin, direction, fire.point.point, nullptr, nullptr);
            if (ours.valid && ours.missMm <= 1.0f)
                ++w.ourLineThroughP;
            else
                ++w.ourLineOffP;
        }
    }
    ReleaseSRWLockExclusive(&w.lock);
}

// object_new returned `handle` for a projectile the fire routine placed (`placement*`: the block before the call).
void Halo3ImpactNoteSpawn(int32_t handle, const float placementPos[3], const float placementFwd[3],
                          const float placementVel[3])
{
    auto& fire = t_halo3ImpactFire;
    auto& w = g_halo3Impact;
    ++fire.projectiles;
    float pos[3]{}, fwd[3]{}, vel[3]{};
    const bool read = handle != -1 && Halo3ImpactReadObject(static_cast<uint32_t>(handle), pos, fwd, vel);
    const bool proven = read && shot_impact::SpawnProven(placementPos, placementFwd, placementVel, pos, fwd, vel);
    auto& window = w.windows[Halo3ImpactGroup(fire)];
    // A velocity still equal to the placement's has not had the projectile's initial speed added (its constructor
    // has not run inside object_new): its direction is not the round's. The tick reads give the line instead.
    const float added[3]{vel[0] - placementVel[0], vel[1] - placementVel[1], vel[2] - placementVel[2]};
    const bool constructed = shot_lag::Length(added) > 1.0e-4f;
    AcquireSRWLockExclusive(&w.lock);
    if (!proven)
        ++window.unproven;
    else if (!fire.pointKnown)
        ++window.noPoint;
    else
    {
        const shot_lag::IntentPoint& p = fire.point;
        const float* eyeVel = p.eyeVelValid ? p.eyeVel : nullptr;
        if (constructed)
            window.AddSpawn(shot_impact::MeasureSpawn(fire.lineDir, fwd, vel, pos, p.point, eyeVel));
        else
            ++window.noInitialSpeed;
        shot_impact::Tracked& t = w.tracker.Arm(Sight_NowUs());
        t.handle = static_cast<uint32_t>(handle);
        t.group = static_cast<uint8_t>(Halo3ImpactGroup(fire));
        memcpy(t.point, p.point, sizeof(t.point));
        memcpy(t.view, p.dir, sizeof(t.view));
        t.eyeVelValid = p.eyeVelValid;
        if (p.eyeVelValid)
            memcpy(t.eyeVel, p.eyeVel, sizeof(t.eyeVel));
        memcpy(t.spawnPos, pos, sizeof(t.spawnPos));
        memcpy(t.spawnVel, vel, sizeof(t.spawnVel));
        memcpy(t.lastPos, pos, sizeof(t.lastPos));
        memcpy(t.lastVel, vel, sizeof(t.lastVel));
    }
    w.armed.store(w.tracker.Armed(), std::memory_order_release);
    ReleaseSRWLockExclusive(&w.lock);
}

__declspec(noinline) int32_t __fastcall Halo3ImpactObjectNewDetour(void* placement)
{
    auto& w = g_halo3Impact;
    w.callbacks.fetch_add(1, std::memory_order_acq_rel);
    int32_t result = -1;
    __try
    {
        const auto& fire = t_halo3ImpactFire;
        const bool ours = reinterpret_cast<uintptr_t>(_ReturnAddress()) == w.fireReturn && fire.open &&
                          fire.local && placement && w.ready.load(std::memory_order_acquire);
        float placementPos[3]{}, placementFwd[3]{}, placementVel[3]{};
        const bool placed = ours && Halo3ImpactCopy3(static_cast<const uint8_t*>(placement) + w.placePos, placementPos) &&
                            Halo3ImpactCopy3(static_cast<const uint8_t*>(placement) + w.placeFwd, placementFwd) &&
                            Halo3ImpactCopy3(static_cast<const uint8_t*>(placement) + w.placeVel, placementVel);
        if (w.original)
            result = w.original(placement);
        if (placed)
            Halo3ImpactNoteSpawn(result, placementPos, placementFwd, placementVel);
    }
    __finally
    {
        w.callbacks.fetch_sub(1, std::memory_order_acq_rel);
    }
    return result;
}

// The collision scheduler's detour, after the native query (Halo3CollisionResolveDetour): a tracked projectile's
// own query -- starting at its object's position, along its velocity -- that hit records its impact point.
void Halo3ImpactWitnessNoteQuery(uintptr_t caller, const float* start, const float* vector, const void* result,
                                 uint8_t hit)
{
    auto& w = g_halo3Impact;
    if (!w.armed.load(std::memory_order_acquire) || !start || !vector || !result)
        return;
    float s[3]{}, v[3]{};
    if (!Halo3ImpactCopy3(start, s) || !Halo3ImpactCopy3(vector, v))
        return;
    float fraction = NAN, point[3]{};
    bool landed = false;
    if (hit)
    {
        __try
        {
            memcpy(&fraction, static_cast<const uint8_t*>(result) + 4, sizeof(fraction));
            memcpy(point, static_cast<const uint8_t*>(result) + 8, sizeof(point));
            landed = std::isfinite(fraction) && fraction >= 0.0f && fraction <= 1.0f && shot_lag::Finite3(point);
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
            landed = false;
        }
    }
    AcquireSRWLockExclusive(&w.lock);
    for (auto& t : w.tracker.slot)
    {
        if (!t.armed)
            continue;
        float pos[3]{}, fwd[3]{}, vel[3]{};
        if (!Halo3ImpactReadObject(t.handle, pos, fwd, vel))
            continue;
        // Within ~3 cm of the object and 5 deg of its velocity: this projectile's own segment.
        if (!shot_impact::QueryIsProjectiles(pos, vel, s, v, 0.01f, 5.0f))
            continue;
        if (landed && !t.impact)
        {
            t.impact = true;
            memcpy(t.impactPos, point, sizeof(t.impactPos));
            t.impactCaller = static_cast<uint32_t>(caller - w.base);
        }
        break;
    }
    ReleaseSRWLockExclusive(&w.lock);
}

// The local unit's simulation tick (LegacySightTraceTick): re-read each tracked projectile; retire the gone.
void Halo3ImpactWitnessTick()
{
    auto& w = g_halo3Impact;
    if (!w.armed.load(std::memory_order_acquire))
        return;
    const uint64_t nowUs = Sight_NowUs();
    AcquireSRWLockExclusive(&w.lock);
    for (auto& t : w.tracker.slot)
    {
        if (!t.armed)
            continue;
        auto& window = w.windows[t.group & 3];
        const float* eyeVel = t.eyeVelValid ? t.eyeVel : nullptr;
        float pos[3]{}, fwd[3]{}, vel[3]{};
        if (!Halo3ImpactReadObject(t.handle, pos, fwd, vel))
        {
            if (t.impact)
            {
                const float offset[3]{t.impactPos[0] - t.point[0], t.impactPos[1] - t.point[1],
                                      t.impactPos[2] - t.point[2]};
                window.AddImpact(shot_impact::Decompose(offset, t.view, eyeVel), t.impactCaller);
            }
            else
            {
                ++window.goneWithoutImpact;
                if (!t.samples)
                    ++window.goneBeforeTick;
            }
            t.armed = false;
            continue;
        }
        if (t.samples < shot_impact::Tracker::kSamples &&
            (pos[0] != t.lastPos[0] || pos[1] != t.lastPos[1] || pos[2] != t.lastPos[2]))
        {
            ++t.samples;
            window.AddTick(shot_lag::MeasureShotLag(pos, vel, t.point, nullptr, eyeVel),
                           shot_impact::OffLineMm(t.spawnPos, t.spawnVel, pos));
        }
        memcpy(t.lastPos, pos, sizeof(t.lastPos));
        memcpy(t.lastVel, vel, sizeof(t.lastVel));
        if (nowUs - t.armedUs > 3000000ull)
        {
            ++window.expired;
            t.armed = false;
        }
    }
    w.armed.store(w.tracker.Armed(), std::memory_order_release);
    ReleaseSRWLockExclusive(&w.lock);
}

void Halo3ImpactNoteReticle(uint64_t serial, const float center[3], const float dir[3], float rangeM, bool hit)
{
    if (!g_halo3Impact.ready.load(std::memory_order_acquire) || !center || !dir)
        return;
    auto& r = g_halo3ImpactReticle;
    AcquireSRWLockExclusive(&r.lock);
    r.valid = true;
    r.serial = serial;
    r.stampUs = Sight_NowUs();
    memcpy(r.center, center, sizeof(r.center));
    memcpy(r.dir, dir, sizeof(r.dir));
    r.rangeM = rangeM;
    r.hit = hit;
    ReleaseSRWLockExclusive(&r.lock);
}

void Halo3ImpactNoteTrigger(bool pressed)
{
    if (!g_halo3Impact.ready.load(std::memory_order_acquire))
        return;
    AcquireSRWLockExclusive(&g_halo3Impact.lock);
    g_halo3Impact.trigger.Note(pressed, Sight_NowUs());
    ReleaseSRWLockExclusive(&g_halo3Impact.lock);
}

bool RemoveHalo3ImpactWitness()
{
    auto& w = g_halo3Impact;
    w.ready.store(false, std::memory_order_release);
    if (!w.target)
        return true;
    const auto status = MCCVR_DisableHookForRetirement(w.target);
    if (status != MH_OK && status != MH_ERROR_DISABLED && status != MH_ERROR_NOT_CREATED)
    {
        LOG("SHOT IMPACT Halo 3 witness CleanupRequired: the hook did not disable");
        return false;
    }
    const void* functions[]{reinterpret_cast<const void*>(&Halo3ImpactObjectNewDetour)};
    const void* trampolines[]{reinterpret_cast<const void*>(w.original)};
    if (!WaitForNativeDetourQuiescence(functions, trampolines, 1, w.callbacks))
    {
        LOG("SHOT IMPACT Halo 3 witness CleanupRequired: the witness is busy");
        return false;
    }
    const auto removed = MH_RemoveHook(w.target);
    if (removed != MH_OK && removed != MH_ERROR_NOT_CREATED)
    {
        LOG("SHOT IMPACT Halo 3 witness CleanupRequired: the hook was not removed");
        return false;
    }
    w.target = nullptr;
    w.original = nullptr;
    AcquireSRWLockExclusive(&w.lock);
    w.tracker = shot_impact::Tracker{};
    w.armed.store(0, std::memory_order_release);
    ReleaseSRWLockExclusive(&w.lock);
    return true;
}

namespace
{
// The function pdata places `address` in (its entry), or 0.
uintptr_t Halo3ImpactFunctionOf(uintptr_t address)
{
    DWORD64 imageBase = 0;
    const auto* entry = RtlLookupFunctionEntry(static_cast<DWORD64>(address), &imageBase, nullptr);
    return entry ? static_cast<uintptr_t>(imageBase) + entry->BeginAddress : 0;
}

uintptr_t Halo3ImpactUnique(uintptr_t base, size_t size, const char* pattern)
{
    const uintptr_t hit = sig::Find(base, size, pattern);
    if (!hit || sig::Find(hit + 1, base + size - hit - 1, pattern))
        return 0;
    return hit;
}

// Does the type-table slot at `slot` point at a definition whose first field names `name`?
bool Halo3ImpactTypeNamed(uintptr_t slot, const char* name)
{
    __try
    {
        const auto* definition = *reinterpret_cast<const uint8_t* const*>(slot);
        const char* text = definition ? *reinterpret_cast<const char* const*>(definition) : nullptr;
        return text && strncmp(text, name, 16) == 0;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}
} // namespace

// After the dual lane (its fire routine's entry is the one this call site must lie in). Its own transaction: failing
// it costs only the two lines.
void InstallHalo3ImpactWitness(uintptr_t base, size_t size)
{
    auto& w = g_halo3Impact;
    if (w.target)
    {
        LOG("SHOT IMPACT Halo 3 witness: a previous hook has not retired; no impact line");
        return;
    }
    w.ready.store(false, std::memory_order_release);
    static constexpr char kObjectNew[] =
        "48 8B C4 53 55 56 57 41 54 41 55 41 56 41 57 48 83 EC 78 83 CD FF 45 33 E4 F6 41 18 10 48 8B F1 8B FD 44 88 "
        "60 10";
    // object_new's copy of the placement block: movsd/mov pairs for position, forward, up and velocity.
    static constexpr char kPlacementCopy[] =
        "F2 0F 10 46 ?? F2 0F 11 43 ?? 8B 46 ?? 89 43 ?? F2 0F 10 46 ?? F2 0F 11 43 ?? 8B 46 ?? 89 43 ?? F2 0F 10 46 ?? "
        "F2 0F 11 43 ?? 8B 46 ?? 89 43 ?? F2 0F 10 46 ?? F2 0F 11 43 ?? 8B 46 ?? 89 43 ??";
    // The fire routine's projectile creation: lea rcx, [rbp+placement]; call object_new; ...; mov [rsp+..], eax.
    static constexpr char kFireCall[] =
        "48 8D 8D ?? ?? 00 00 E8 ?? ?? ?? ?? F3 44 0F 10 4C 24 ?? 41 83 C9 FF 89 44 24 ?? 41 3B C1 0F 84";
    // object_new's type-table load: lea rbx, [image base]; movsx rax, word [r15]; mov r13, r12; mov rbx, [rbx+rax*8+T].
    static constexpr char kTypeTable[] = "48 8D 1D ?? ?? ?? ?? 49 0F BF 07 4D 8B EC 48 8B 9C C3 ?? ?? ?? ??";
    const uintptr_t entry = Halo3ImpactUnique(base, size, kObjectNew);
    const uintptr_t copy = Halo3ImpactUnique(base, size, kPlacementCopy);
    const uintptr_t call = Halo3ImpactUnique(base, size, kFireCall);
    const uintptr_t table = Halo3ImpactUnique(base, size, kTypeTable);
    const uintptr_t fireEntry = reinterpret_cast<uintptr_t>(g_halo3Dual.fireTarget);
    if (!entry || !copy || !call || !table || !fireEntry)
    {
        LOG("SHOT IMPACT Halo 3 witness: an anchor did not prove (object_new %d, its placement copy %d, the fire's "
            "call %d, the type table %d, the dual lane's fire routine %d); no impact line",
            entry ? 1 : 0, copy ? 1 : 0, call ? 1 : 0, table ? 1 : 0, fireEntry ? 1 : 0);
        return;
    }
    int32_t rel = 0, tableRel = 0;
    uint32_t tableRva = 0;
    memcpy(&rel, reinterpret_cast<const void*>(call + 8), sizeof(rel));
    memcpy(&tableRel, reinterpret_cast<const void*>(table + 3), sizeof(tableRel));
    memcpy(&tableRva, reinterpret_cast<const void*>(table + 18), sizeof(tableRva));
    const auto* c = reinterpret_cast<const uint8_t*>(copy);
    const uint32_t placePos = c[4], objPos = c[9], placePosZ = c[0x0C], objPosZ = c[0x0F];
    const uint32_t placeFwd = c[0x14], objFwd = c[0x19];
    const uint32_t placeVel = c[0x34], objVel = c[0x39];
    const bool copyCoherent = placePosZ == placePos + 8 && objPosZ == objPos + 8 && placeFwd == placePos + 0xC &&
                              objFwd == objPos + 0xC && placeVel == placePos + 0x24 && objVel == objPos + 0x24;
    const bool inObjectNew = Halo3ImpactFunctionOf(copy) == entry && Halo3ImpactFunctionOf(table) == entry;
    const bool callsObjectNew = call + 12 + static_cast<intptr_t>(rel) == entry;
    const bool inFire = Halo3ImpactFunctionOf(call) == fireEntry;
    const bool imageBase = table + 7 + static_cast<intptr_t>(tableRel) == base && tableRva && tableRva + 6 * 8 <= size;
    if (!copyCoherent || !inObjectNew || !callsObjectNew || !inFire || !imageBase)
    {
        LOG("SHOT IMPACT Halo 3 witness: the decoded layout did not prove (placement copy coherent %d, inside "
            "object_new %d, the fire's call targets it %d, inside the fire routine %d, type table %d); no impact line",
            copyCoherent ? 1 : 0, inObjectNew ? 1 : 0, callsObjectNew ? 1 : 0, inFire ? 1 : 0, imageBase ? 1 : 0);
        return;
    }
    // The projectile type: the table entry whose definition is named "projectile" (index 5 in this build).
    uint8_t projectileType = 0xFF;
    for (uint8_t index = 0; index < 16 && projectileType == 0xFF; ++index)
        if (Halo3ImpactTypeNamed(base + tableRva + index * 8u, "projectile"))
            projectileType = index;
    if (projectileType == 0xFF)
    {
        LOG("SHOT IMPACT Halo 3 witness: no object type named \"projectile\" in the table at +0x%X; no impact line",
            tableRva);
        return;
    }
    w.base = base;
    w.fireReturn = call + 12;
    w.placePos = placePos;
    w.placeFwd = placeFwd;
    w.placeVel = placeVel;
    w.objPos = objPos;
    w.objFwd = objFwd;
    w.objVel = objVel;
    w.projectileType = projectileType;
    AcquireSRWLockExclusive(&w.lock);
    w.tracker = shot_impact::Tracker{};
    for (auto& window : w.windows)
        window = shot_impact::Window{};
    w.reticle = shot_impact::ReticleWindow{};
    w.trigger = shot_impact::TriggerLatch{};
    w.armed.store(0, std::memory_order_release);
    ReleaseSRWLockExclusive(&w.lock);
    void* const target = reinterpret_cast<void*>(entry);
    if (MH_CreateHook(target, reinterpret_cast<void*>(&Halo3ImpactObjectNewDetour),
                      reinterpret_cast<void**>(&w.original)) != MH_OK)
    {
        w.original = nullptr;
        LOG("SHOT IMPACT Halo 3 witness: the hook was not created; no impact line");
        return;
    }
    w.target = target;
    if (MH_EnableHook(target) != MH_OK)
    {
        LOG("SHOT IMPACT Halo 3 witness: the hook did not enable; no impact line");
        (void)RemoveHalo3ImpactWitness();
        return;
    }
    w.ready.store(true, std::memory_order_release);
    LOG("SHOT IMPACT Halo 3 witness (dev): hooked object_new +0x%llX for the fire routine's call +0x%llX (return "
        "+0x%llX); placement +0x%X/+0x%X/+0x%X -> object +0x%X/+0x%X/+0x%X (position/forward/velocity); projectile "
        "type %u (table +0x%X)", static_cast<unsigned long long>(entry - base),
        static_cast<unsigned long long>(call + 7 - base), static_cast<unsigned long long>(call + 12 - base), placePos,
        placeFwd, placeVel, objPos, objFwd, objVel, projectileType, tableRva);
}

// Cold worker (GunAimReport, every 2 s): the windows.
void Halo3ImpactWitnessReport()
{
    auto& w = g_halo3Impact;
    shot_impact::Window windows[4];
    shot_impact::ReticleWindow reticle;
    uint32_t nested = 0, through = 0, off = 0, evicted = 0;
    AcquireSRWLockExclusive(&w.lock);
    for (int g = 0; g < 4; ++g)
    {
        windows[g] = w.windows[g];
        w.windows[g] = shot_impact::Window{};
    }
    reticle = w.reticle;
    w.reticle = shot_impact::ReticleWindow{};
    nested = w.nested;
    through = w.ourLineThroughP;
    off = w.ourLineOffP;
    evicted = w.tracker.evicted;
    w.nested = w.ourLineThroughP = w.ourLineOffP = 0;
    w.tracker.evicted = 0;
    ReleaseSRWLockExclusive(&w.lock);
    static const char* const kLabels[4]{"Halo 3 [native; authoritative]", "Halo 3 [native; predicted]",
                                        "Halo 3 [muzzle; authoritative]", "Halo 3 [muzzle; predicted]"};
    char line[1536]{};
    for (int g = 0; g < 4; ++g)
        if (windows[g].Format(kLabels[g], line, sizeof(line)))
            LOG("%s", line);
    if (reticle.Format("Halo 3", line, sizeof(line)))
        LOG("%s; our muzzle line through P %u, off it %u%s", line, through, off,
            nested || evicted ? " (nested fires or evicted projectiles: see below)" : "");
    const uint32_t faults = w.faults.exchange(0, std::memory_order_relaxed);
    if (nested || evicted || faults)
        LOG("SHOT IMPACT Halo 3 (dev): %u fires inside a fire (not measured), %u tracked projectiles evicted, %u read "
            "faults", nested, evicted, faults);
}
#endif
