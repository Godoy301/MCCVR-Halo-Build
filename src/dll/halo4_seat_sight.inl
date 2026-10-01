// Included in game.cpp (anonymous namespace) right after the Halo 4 muzzle lane, whose view, aim and fire
// detours call into it; the contact lane's per-unit update calls its seated trace (forward-declared there).
//
// HALO 4 SEATED CONVERGENCE, stages 0 and 1. DEV BUILDS ONLY: a player build compiles none of this file.
// Stage 0 only measures (the dev lines below); stage 1 is the dev key halo4_seat_converge, off by default.
// The mechanism and every pure decision are in src/common/halo4_seat_aim_logic.h; this file is the engine
// half:
//
//   ANCHORS (InstallHalo4SeatSight, after the muzzle lane). Each is a unique signature plus the native call
//   edge that proves its arguments, each its own feature bit, each failing closed with one LOG line:
//     seat flags    0x5FB2D4 (seat, body) + the dispatcher's call 0xA88F1        -> class Unknown, no convergence
//     controlled    0x5F9694 + 0x1DC9E4's own call 0x1DCA0E                      -> class Unknown, no convergence
//     object frame  0x5D84D0 + the record rebase's call 0x5F98DB (0x5F98C2)      -> bit-4 seats only
//     barrel        0x6121B8 + the fire routine's marker block 0x61797F          -> no claim (honest reticle)
//     aim unit      the fire routine's aim-unit block 0x617DA7 (+0x49C)          -> no claim (honest reticle)
//     aim camera    0x2061A8 whole (selector call, +0x490 bit 31) + 0x1DCA25     -> no first-person witness, no convergence
//     assist exit   0x1DCB8C + the fire routine's call 0x618103 (a dev hook)     -> no SHOT Halo 4 seated line
//   The weapon-slot, owner, aiming-unit and barrel-marker natives (0x5FA1C4, 0x612238, 0x5F1ABC, 0x610F6C) and
//   the marker query 0x5D5B74 are the muzzle lane's own proven bindings; without that lane nothing here runs.
//
//   THE SIMULATION SAMPLE (Halo4SeatSightNoteAimCamera, from the muzzle's view detour on the per-tick
//   targeting calls 0x1DB9A6 / 0x1DBDB6, never on the mod's own query or a barrel shot -- verifier item 6):
//   the game's own aim camera for the local unit, the exact first-person witness (item 7), the seat flags,
//   the controlled unit (item 15), the vehicle's up, whether the aiming unit carries the player (item 12) and
//   the barrel's forward and its offset from the eye (item 11). Seqlock, published every tick.
//
//   THE SOLVE'S EYE (Halo4PublishSeatEyeRoot, from the stereo transaction): the controller root plus the
//   native seat eye's render-time offset (NativeVehicleFirstPerson_SeatEyeDelta) -- the game's aim-line
//   origin in the same frame as the rendered head, never a simulation-time world position.
//
//   THE SEATED TRACE (Halo4SightTraceSeated, the contact lane's per-unit update): the published seated intent
//   ray, a cheap pre-gate before the rate limit (item 10), identity from this thread's own seat read, the
//   vehicle's root, the occupant, the seat's parent and the controlled unit ignored, our own vehicle looked
//   past (three passes; exhaustion is a MISS). Stage 0 keeps the result to itself (the shot witness measures
//   against it); stage 1 publishes it to the sight layer.
//
//   THE SHOT WITNESS (Halo4SeatAssistDetour, 0x1DCB8C filtered to the fire routine's return 0x618108 --
//   item 9): the shot's direction before and after the native seated assist, against the converged point.
#if HALOMCCVR_DEV_DIAGNOSTICS

using Halo4SeatFlagFn = uint8_t(__fastcall*)(uint32_t, int16_t, uint32_t);       // 0x5FB2D4(vehicle, seat, bit)
using Halo4SeatHandleFn = uint32_t(__fastcall*)(uint32_t);                         // 0x5F9694 0x6121B8 0x612238 0x5F1ABC
using Halo4SeatObjectFrameFn = void(__fastcall*)(uint32_t, float*, float*);        // 0x5D84D0(object, forward, up)
using Halo4SeatWeaponInSlotFn = uint32_t(__fastcall*)(uint32_t, int32_t);          // 0x5FA1C4(unit, role)
using Halo4SeatBarrelMarkerFn = uint32_t(__fastcall*)(uint32_t, int32_t);          // 0x610F6C(weapon, barrel)
using Halo4SeatAssistFn = uint64_t(__fastcall*)(uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t,
                                                uint64_t, uint64_t);               // 0x1DCB8C, 8 arguments

enum : uint32_t
{
    kH4SeatFlags = 1u << 0,
    kH4SeatControlled = 1u << 1,
    kH4SeatObjectFrame = 1u << 2,
    kH4SeatBarrel = 1u << 3,
    kH4SeatAimUnit = 1u << 4,
    kH4SeatAimCamera = 1u << 5,
    kH4SeatAssistWitness = 1u << 6,
};

// The simulation sample (one per tick for the local unit). World units; plain data, copied whole.
struct Halo4SeatAimSample
{
    uint32_t sequence = 0, generation = 0;
    uint64_t ms = 0;
    uint32_t unit = UINT32_MAX, parent = UINT32_MAX;
    int16_t seat = -1;
    bool seated = false;
    float origin[3]{}, camera[3]{}, dir[3]{};   // 0x1DC9E4's outputs: the eye (chase: pushed), the camera, its aim
    int8_t witness = -1;                        // 1 exact first person, 0 the selector chose chase, -1 no proof
    bool pushed = false;                        // the chase push moved the eye off the camera
    float speedMps = -1.0f;                     // the camera's speed over the last sample, metres/s (-1 unknown)
    bool flagsKnown = false;
    uint32_t flags = 0;
    uint32_t controlled = UINT32_MAX;
    halo4_seat_aim::Controlled controlledKind = halo4_seat_aim::Controlled::Unknown;
    bool upKnown = false;
    float up[3]{};
    bool weaponKnown = false, hasWeapon = false;
    bool aimUnitKnown = false, assisted = false;   // the aiming unit is this unit and carries its player
    bool barrelKnown = false;
    float barrelForward[3]{}, barrelOffset[3]{};   // the barrel marker's forward, and its origin minus `camera`
};

struct Halo4SeatSightRuntime
{
    uintptr_t base = 0;
    uint32_t generation = 0;
    std::atomic<uint32_t> ready{0};
    Halo4SeatFlagFn seatFlag = nullptr;
    Halo4SeatHandleFn controlled = nullptr, markerOwner = nullptr, weaponOwner = nullptr, aimingUnit = nullptr;
    Halo4SeatObjectFrameFn objectFrame = nullptr;
    Halo4SeatWeaponInSlotFn weaponInSlot = nullptr;
    Halo4SeatBarrelMarkerFn barrelMarker = nullptr;
    Halo4MuzzleMarkersFn markers = nullptr;
    // The assist-exit witness hook (dev).
    void* assistTarget = nullptr;
    Halo4SeatAssistFn assistOriginal = nullptr;
    std::atomic<uint32_t> assistCallbacks{0};
    // The sample: a seqlock with a try-lock writer (two per-tick targeting sites, one thread in practice).
    std::atomic_flag sampleWriting = ATOMIC_FLAG_INIT;
    std::atomic<uint32_t> sampleVersion{0};
    Halo4SeatAimSample sample{};   // written only between the odd and even version stores
    Halo4SeatAimSample last{};     // the writer's own previous sample (the speed)
    uint64_t lastQpc = 0;
    // Counters since the last report (dev lines).
    std::atomic<uint64_t> samples{0}, samplesSeated{0}, firstPerson{0}, chase{0}, noWitness{0}, pushes{0},
        sampleFaults{0}, writerBusy{0};
    std::atomic<uint32_t> lastFlags{0}, lastClass{0}, lastControlledKind{0}, lastAssisted{2}, lastBarrel{0},
        lastUpZ1e4{0};
    // The seated trace.
    std::atomic<uint64_t> seatedUpdates{0}, traces{0}, traceHits{0}, traceMisses{0}, lookPasts{0}, exhausted{0},
        unnamed{0}, mismatched{0}, rootFallbacks{0}, published{0}, costTicks{0}, costMaxTicks{0}, faults{0};
} g_h4SeatSight;

// ---- the solve's eye (render transaction -> input thread) ---------------------------------------------------
struct Halo4SeatEyeRootPublication
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0}, parent{UINT32_MAX};
    std::atomic<int32_t> seat{-1};
    std::atomic<uint64_t> ms{0};
    std::atomic<float> position[3]{}, delta[3]{};
};
Halo4SeatEyeRootPublication g_halo4SeatEyeRoot;
// A render-time offset older than this is not this frame's camera any more.
inline constexpr uint64_t kHalo4SeatEyeDeltaMaxAgeMs = 50;

// Stereo transaction (render thread): the game's own aim-line origin for this frame's seat, as the controller
// root (the observer before head pose: head plus trims in an owned seat) plus the native eye's offset from
// that placed point. Nothing when the seat stamp is not seated or the offset is not this seat's or is stale.
void Halo4PublishSeatEyeRoot(bool seatRead, const Halo4VehicleInputState& seat, const float rootPosition[3])
{
    NativeVehicleSeatEyeDelta d{};
    const uint64_t now = GetTickCount64();
    if (!seatRead || !seat.seated || seat.parent == UINT32_MAX || seat.seat < 0 ||
        !NativeVehicleFirstPerson_SeatEyeDelta(GameTitle::Halo4, d) || d.parent != seat.parent ||
        d.seat != seat.seat || now < d.ms || now - d.ms > kHalo4SeatEyeDeltaMaxAgeMs)
        return;
    float position[3]{};
    for (int axis = 0; axis < 3; ++axis)
    {
        position[axis] = rootPosition[axis] + d.delta[axis];
        if (!std::isfinite(position[axis]))
            return;
    }
    auto& p = g_halo4SeatEyeRoot;
    p.sequence.fetch_add(1, std::memory_order_acq_rel);
    p.generation.store(d.generation, std::memory_order_relaxed);
    p.parent.store(d.parent, std::memory_order_relaxed);
    p.seat.store(d.seat, std::memory_order_relaxed);
    for (int axis = 0; axis < 3; ++axis)
    {
        p.position[axis].store(position[axis], std::memory_order_relaxed);
        p.delta[axis].store(d.delta[axis], std::memory_order_relaxed);
    }
    p.ms.store(now, std::memory_order_relaxed);
    p.sequence.fetch_add(1, std::memory_order_release);
}

// Any thread: the solve's eye for exactly this seat, at most maxAgeMs old.
bool Halo4LoadSeatEyeRoot(uint32_t parent, int seat, uint64_t maxAgeMs, float out[3], float delta[3])
{
    const auto& p = g_halo4SeatEyeRoot;
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = p.sequence.load(std::memory_order_acquire);
        if (!before || (before & 1u))
            continue;
        float position[3]{}, d[3]{};
        for (int axis = 0; axis < 3; ++axis)
        {
            position[axis] = p.position[axis].load(std::memory_order_relaxed);
            d[axis] = p.delta[axis].load(std::memory_order_relaxed);
        }
        const uint32_t generation = p.generation.load(std::memory_order_relaxed);
        const uint32_t publishedParent = p.parent.load(std::memory_order_relaxed);
        const int32_t publishedSeat = p.seat.load(std::memory_order_relaxed);
        const uint64_t at = p.ms.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (p.sequence.load(std::memory_order_relaxed) != before)
            continue;
        const uint64_t now = GetTickCount64();
        if (!at || now < at || now - at > maxAgeMs || publishedParent != parent || publishedSeat != seat ||
            generation != TitleAdapter_GetGeneration(GameTitle::Halo4))
            return false;
        memcpy(out, position, sizeof(position));
        if (delta)
            memcpy(delta, d, sizeof(d));
        return true;
    }
    return false;
}

// ---- the simulation sample -------------------------------------------------------------------------------
bool Halo4ReadSeatAimSample(Halo4SeatAimSample& out)
{
    const auto& f = g_h4SeatSight;
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = f.sampleVersion.load(std::memory_order_acquire);
        if (!before || (before & 1u))
            continue;
        Halo4SeatAimSample s;
        memcpy(&s, &f.sample, sizeof(s));
        std::atomic_thread_fence(std::memory_order_acquire);
        if (f.sampleVersion.load(std::memory_order_relaxed) != before)
            continue;
        out = s;
        return true;
    }
    return false;
}

bool Halo4SeatFinite3(const float* v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// Simulation thread, inside the view detour's callbacks lease. Every native here is one the anchors proved,
// called only with a handle the salt-checked object table still holds (they validate nothing themselves).
// Guarded; plain data only, so __try is legal.
bool Halo4SeatSightBuildSample(uint32_t unit, const Halo4VehicleInputState& seat, const float* origin,
                               const float* dir, const float* camera, const NativeVehicleSelectWitness& witness,
                               uint32_t ready, Halo4SeatAimSample& s)
{
    auto& f = g_h4SeatSight;
    __try
    {
        memcpy(s.origin, origin, sizeof(s.origin));
        memcpy(s.camera, camera, sizeof(s.camera));
        memcpy(s.dir, dir, sizeof(s.dir));
        if (!Halo4SeatFinite3(s.origin) || !Halo4SeatFinite3(s.camera) || !Halo4SeatFinite3(s.dir))
            return false;
        s.unit = unit;
        s.parent = seat.parent;
        s.seat = seat.seat;
        s.seated = seat.seated;
        const float pushed[3]{s.origin[0] - s.camera[0], s.origin[1] - s.camera[1], s.origin[2] - s.camera[2]};
        s.pushed = pushed[0] * pushed[0] + pushed[1] * pushed[1] + pushed[2] * pushed[2] > 1.0e-8f;
        const uint8_t* unitData = Halo4MuzzleObject(unit, 1);
        if (!unitData)
            return false;
        // THE FIRST-PERSON WITNESS: exactly one selector call, for this unit, nested in this view call; and
        // 0x2061A8 then took the selector's branch, not the unit's camera override (+0x490 bit 31, which it
        // tests AFTER the selector call: the aim-camera anchor proves the offset and the order).
        if (witness.calls == 1 && witness.unit == unit)
        {
            uint32_t override490 = 0;
            memcpy(&override490, unitData + 0x490, sizeof(override490));
            s.witness = (override490 >> 31) ? int8_t(-1) : (witness.result == 0 ? int8_t(1) : int8_t(0));
        }
        if (!s.seated)
            return true;
        if (ready & kH4SeatFlags)
        {
            for (const uint8_t bit : halo4_seat_aim::kSeatFlagBitsRead)
                if (f.seatFlag(seat.parent, seat.seat, bit))
                    s.flags |= 1u << bit;
            s.flagsKnown = true;
        }
        if (ready & kH4SeatObjectFrame)
        {
            float up[3]{};
            f.objectFrame(seat.parent, nullptr, up);
            const float l = up[0] * up[0] + up[1] * up[1] + up[2] * up[2];
            if (Halo4SeatFinite3(up) && l > 0.81f && l < 1.21f)
            {
                memcpy(s.up, up, sizeof(up));
                s.upKnown = true;
            }
        }
        if (!(ready & kH4SeatControlled))
            return true;
        s.controlled = f.controlled(unit);
        const uint8_t* controlledData =
            s.controlled != UINT32_MAX ? Halo4MuzzleObject(s.controlled, 0x2003) : nullptr;
        if (!controlledData)
        {
            s.controlled = UINT32_MAX;
            return true;
        }
        s.controlledKind = halo4_seat_aim::ClassifyControlled(static_cast<int32_t>(s.controlled),
                                                              static_cast<int32_t>(unit),
                                                              static_cast<int32_t>(seat.parent));
        // The controlled unit's current weapon: its role byte, as the selector (0x10D3BB) and the muzzle lane
        // read it, then the native slot lookup. 0xFF is the native "no weapon".
        const int8_t role = static_cast<int8_t>(controlledData[0x63A]);
        if (role != -1 && (role < 0 || role > 3))
            return true;
        const uint32_t weapon = f.weaponInSlot(s.controlled, role);
        if (weapon != UINT32_MAX && !Halo4MuzzleObject(weapon, 4))
            return true;
        s.weaponKnown = true;
        s.hasWeapon = weapon != UINT32_MAX;
        if (!s.hasWeapon)
            return true;
        if (ready & kH4SeatAimUnit)
        {
            // THE SHOT'S AIMING UNIT, exactly as the fire routine names it (0x617DAA): the weapon's owner, then
            // its controlling-unit chain. The native seated assist runs only when that unit carries a player.
            const uint32_t owner = f.weaponOwner(weapon);
            if (owner != UINT32_MAX && Halo4MuzzleObject(owner, UINT32_MAX))
            {
                const uint32_t aiming = f.aimingUnit(owner);
                s.aimUnitKnown = true;
                if (aiming == unit)
                {
                    int32_t player = -1;
                    memcpy(&player, unitData + 0x49C, sizeof(player));
                    s.assisted = player != -1;
                }
            }
        }
        if (ready & kH4SeatBarrel)
        {
            // THE BARREL, exactly as the fire routine reads it for barrel 0 (0x617986 -> 0x6179B2): the barrel's
            // marker name on the object that carries the weapon's markers. A missing marker is refused (the
            // native fills a fallback transform; for names 0 and 0x22D it even counts it).
            const uint32_t name = f.barrelMarker(weapon, 0);
            const uint32_t carrier = f.markerOwner(weapon);
            if (name && name != 0x22D && carrier != UINT32_MAX && Halo4MuzzleObject(carrier, UINT32_MAX))
            {
                alignas(16) uint8_t record[0x100]{};
                const int16_t count = f.markers(carrier, name, record, 1, 0, 0, 0);
                float forward[3]{}, position[3]{};
                memcpy(forward, record + 0x3C, sizeof(forward));
                memcpy(position, record + 0x60, sizeof(position));
                const float l = forward[0] * forward[0] + forward[1] * forward[1] + forward[2] * forward[2];
                if (count >= 1 && Halo4SeatFinite3(forward) && Halo4SeatFinite3(position) && l > 0.81f && l < 1.21f)
                {
                    memcpy(s.barrelForward, forward, sizeof(forward));
                    for (int axis = 0; axis < 3; ++axis)
                        s.barrelOffset[axis] = position[axis] - s.camera[axis];
                    s.barrelKnown = true;
                }
            }
        }
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        f.sampleFaults.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
}

// The muzzle's view detour, before the native call: arm the first-person witness for a per-tick targeting
// call that is neither the mod's own query nor a barrel shot (verifier item 6).
bool Halo4SeatSightArmAimCamera(uintptr_t caller)
{
    const auto& f = g_h4SeatSight;
    const uintptr_t base = g_halo4Barrel.base;
    if (!(f.ready.load(std::memory_order_acquire) & kH4SeatAimCamera) || !base ||
        (caller != base + 0x1DB9A6 && caller != base + 0x1DBDB6) || g_halo4MuzzleOwnedQuery ||
        g_halo4BarrelShot.active || TitleAdapter_GetActiveTitle() != GameTitle::Halo4)
        return false;
    NativeVehicleFirstPerson_ArmSelectWitness();
    return true;
}

// The muzzle's view detour, after the native call and BEFORE any barrel substitution touches the outputs.
void Halo4SeatSightNoteAimCamera(uint32_t unit, const float* origin, const float* dir, const float* camera)
{
    const NativeVehicleSelectWitness witness = NativeVehicleFirstPerson_TakeSelectWitness();
    auto& f = g_h4SeatSight;
    const uint32_t ready = f.ready.load(std::memory_order_acquire);
    if (!(ready & kH4SeatAimCamera) || !origin || !dir || !camera ||
        f.generation != TitleAdapter_GetGeneration(GameTitle::Halo4))
        return;
    // The local unit only: a remote co-op unit is never sampled.
    Halo4VehicleInputState seat{};
    if (!Halo4ReadVehicleInput(seat) || seat.unit != unit)
        return;
    Halo4SeatAimSample s{};
    s.generation = f.generation;
    s.ms = GetTickCount64();
    if (!Halo4SeatSightBuildSample(unit, seat, origin, dir, camera, witness, ready, s))
        return;
    LARGE_INTEGER qpc{}, frequency{};
    QueryPerformanceCounter(&qpc);
    QueryPerformanceFrequency(&frequency);
    if (f.sampleWriting.test_and_set(std::memory_order_acquire))
    {
        f.writerBusy.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    // The camera's speed since this writer's previous sample of the same occupant and seat (same clock).
    if (f.lastQpc && f.last.unit == s.unit && f.last.parent == s.parent && f.last.seat == s.seat &&
        frequency.QuadPart > 0 && qpc.QuadPart > static_cast<LONGLONG>(f.lastQpc))
    {
        const double dt = double(qpc.QuadPart - static_cast<LONGLONG>(f.lastQpc)) / double(frequency.QuadPart);
        const float d[3]{s.camera[0] - f.last.camera[0], s.camera[1] - f.last.camera[1],
                         s.camera[2] - f.last.camera[2]};
        if (dt > 0.001 && dt < 0.5)
            s.speedMps = static_cast<float>(std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) * 3.048 / dt);
    }
    f.last = s;
    f.lastQpc = static_cast<uint64_t>(qpc.QuadPart);
    s.sequence = f.sampleVersion.load(std::memory_order_relaxed) / 2u + 1u;
    f.sampleVersion.fetch_add(1, std::memory_order_acq_rel);   // odd: writing
    memcpy(&f.sample, &s, sizeof(s));
    f.sampleVersion.fetch_add(1, std::memory_order_release);   // even: published
    f.sampleWriting.clear(std::memory_order_release);
    f.samples.fetch_add(1, std::memory_order_relaxed);
    if (s.seated)
    {
        f.samplesSeated.fetch_add(1, std::memory_order_relaxed);
        (s.witness > 0 ? f.firstPerson : s.witness == 0 ? f.chase : f.noWitness).fetch_add(1, std::memory_order_relaxed);
        if (s.pushed)
            f.pushes.fetch_add(1, std::memory_order_relaxed);
        f.lastFlags.store(s.flagsKnown ? s.flags : 0xFFFFFFFFu, std::memory_order_relaxed);
        f.lastClass.store(static_cast<uint32_t>(halo4_seat_aim::ClassifySeat(s.flagsKnown, s.flags, s.controlledKind,
            s.weaponKnown, s.hasWeapon)), std::memory_order_relaxed);
        f.lastControlledKind.store(static_cast<uint32_t>(s.controlledKind), std::memory_order_relaxed);
        f.lastAssisted.store(s.aimUnitKnown ? (s.assisted ? 1u : 0u) : 2u, std::memory_order_relaxed);
        f.lastBarrel.store(s.barrelKnown ? 1u : 0u, std::memory_order_relaxed);
        f.lastUpZ1e4.store(s.upKnown ? static_cast<uint32_t>(std::lround((s.up[2] + 1.0f) * 10000.0f)) : 0u,
                           std::memory_order_relaxed);
    }
}

// ---- the seated trace (simulation thread, the contact lane's per-unit update) ------------------------------
// The seated trace's own last result, beside (stage 1) or instead of (stage 0) the sight layer's: the shot
// witness measures against it when no seated trace is published. Single writer (the simulation thread).
struct Halo4SeatTraceRecord
{
    std::atomic<uint32_t> sequence{0};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint64_t> ms{0};
    std::atomic<bool> hit{false};
    std::atomic<float> rangeWu{0.0f}, head[3]{}, dir[3]{}, eye[3]{};
} g_halo4SeatTrace;

void Halo4PublishSeatTrace(uint32_t generation, bool hit, float rangeWu, const SightRay& ray)
{
    auto& r = g_halo4SeatTrace;
    r.sequence.fetch_add(1, std::memory_order_acq_rel);
    r.generation.store(generation, std::memory_order_relaxed);
    r.hit.store(hit, std::memory_order_relaxed);
    r.rangeWu.store(hit ? rangeWu : kSightTraceLengthWu, std::memory_order_relaxed);
    for (int axis = 0; axis < 3; ++axis)
    {
        r.head[axis].store(ray.head[axis], std::memory_order_relaxed);
        r.dir[axis].store(ray.dir[axis], std::memory_order_relaxed);
        r.eye[axis].store(ray.eye[axis], std::memory_order_relaxed);
    }
    r.ms.store(GetTickCount64(), std::memory_order_relaxed);
    r.sequence.fetch_add(1, std::memory_order_release);
}

// The last seated trace as a ray and a range, at most maxAgeMs old and of this generation.
bool Halo4ReadSeatTrace(uint32_t generation, uint64_t maxAgeMs, SightRay& ray, float& rangeWu)
{
    const auto& r = g_halo4SeatTrace;
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = r.sequence.load(std::memory_order_acquire);
        if (!before || (before & 1u))
            continue;
        SightRay out{};
        for (int axis = 0; axis < 3; ++axis)
        {
            out.head[axis] = r.head[axis].load(std::memory_order_relaxed);
            out.dir[axis] = r.dir[axis].load(std::memory_order_relaxed);
            out.eye[axis] = r.eye[axis].load(std::memory_order_relaxed);
        }
        const float range = r.rangeWu.load(std::memory_order_relaxed);
        const uint32_t g = r.generation.load(std::memory_order_relaxed);
        const uint64_t at = r.ms.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (r.sequence.load(std::memory_order_relaxed) != before)
            continue;
        const uint64_t now = GetTickCount64();
        if (!generation || g != generation || !at || now < at || now - at > maxAgeMs || !std::isfinite(range))
            return false;
        ray = out;
        rangeWu = range;
        return true;
    }
    return false;
}

// The root of an object's parent chain (+0x24, the link 0x5D84D0, 0x6121B8 and the seat readers walk), through
// the salt-checked object table. -1 on a stale link or a chain deeper than any vehicle.
int32_t Halo4SightRootOf(int32_t object)
{
    if (object == -1)
        return -1;
    __try
    {
        int32_t current = object;
        for (int depth = 0; depth < 8; ++depth)
        {
            const uint8_t* data = Halo4ContactObject(static_cast<uint32_t>(current));
            if (!data)
                return -1;
            int32_t parent = -1;
            memcpy(&parent, data + 0x24, sizeof(parent));
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

// One native ray, guarded. False: a fault (the trace latches off).
bool Halo4SeatGuardedRayCast(Halo4PhysicsRayCastInput* input, Halo4PhysicsRayCastResult* output, bool& returned)
{
    __try
    {
        returned = g_halo4WorldCollision.originalRayCast(input, output);
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Whether this update is the local occupant's own, read on THIS thread (never the stamp's or the first-person
// body's), and the unit it controls (0x5F9694, item 15). Guarded.
bool Halo4SeatSightLocalSeated(uint32_t unit, void* instance, Halo4VehicleInputState& seat, uint32_t& controlled)
{
    controlled = UINT32_MAX;
    __try
    {
        if (!Halo4ReadVehicleInput(seat) || !seat.seated || seat.unit != unit ||
            Halo4ContactBiped(unit) != instance)
            return false;
        if (g_h4SeatSight.ready.load(std::memory_order_acquire) & kH4SeatControlled)
        {
            controlled = g_h4SeatSight.controlled(unit);
            if (controlled != UINT32_MAX && !Halo4ContactObject(controlled))
                controlled = UINT32_MAX;
        }
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

void Halo4SightTraceSeated(uint32_t unit, void* instance)
{
    constexpr GameTitle kTitle = GameTitle::Halo4;
    auto& f = g_h4SeatSight;
    if (!LegacySight(kTitle).flags.load(std::memory_order_acquire) || !g_halo4Contact.flags ||
        !g_halo4WorldCollision.originalRayCast || TitleAdapter_GetActiveTitle() != kTitle)
        return;
    // CHEAP PRE-GATE FIRST (verifier item 10): this runs for every unit's update. Only the unit the camera
    // hook's stamp names as seated goes on; the rate limit is taken only by it, so another unit never uses up
    // the 30 ms slot, and the two native calls of this thread's own seat read run at most once per trace.
    if (g_halo4SeatStamp.unit.load(std::memory_order_relaxed) != unit)
        return;   // one relaxed load for every other unit
    Halo4VehicleInputState stamp{};
    if (!Halo4ReadSeatStamp(stamp) || !stamp.seated || stamp.unit != unit)
        return;
    f.seatedUpdates.fetch_add(1, std::memory_order_relaxed);   // open question 1: the seated biped IS updated
    const uint64_t now = GetTickCount64();
    if (Sight_TraceFaulted(kTitle) || !Sight_TraceDue(kTitle, now))
        return;
    Halo4VehicleInputState seat{};
    uint32_t controlled = UINT32_MAX;
    if (!Halo4SeatSightLocalSeated(unit, instance, seat, controlled))
    {
        f.mismatched.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const uint32_t generation = TitleAdapter_GetGeneration(kTitle);
    SightRay ray{};
    if (!Sight_ReadRay(kTitle, generation, kSightRayFreshMs, ray))
        return;
    if (ray.ignoreB != static_cast<int32_t>(seat.parent) || (ray.unit != -1 && ray.unit != static_cast<int32_t>(unit)))
    {
        // A ray the aim path did not publish for this seat: not traced.
        f.unnamed.fetch_add(1, std::memory_order_relaxed);
        LegacySight(kTitle).seatUnnamed.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    int32_t root = Halo4SightRootOf(static_cast<int32_t>(unit));
    if (root == -1 || root == static_cast<int32_t>(unit))
    {
        // The walk failed: the seat's parent (Halo 4 publishes no carrier).
        root = static_cast<int32_t>(seat.parent);
        f.rootFallbacks.fetch_add(1, std::memory_order_relaxed);
        LegacySight(kTitle).rootFallbacks.fetch_add(1, std::memory_order_relaxed);
    }
    int32_t chain[4]{};
    const int chainCount = halo4_seat_aim::BuildSeatIgnores(root, static_cast<int32_t>(unit),
        static_cast<int32_t>(seat.parent), controlled == UINT32_MAX ? -1 : static_cast<int32_t>(controlled), chain);
    uint64_t collisionFlags = g_halo4SightShotFlags.load(std::memory_order_acquire);
    if (!collisionFlags)
        memcpy(&collisionFlags, g_halo4Contact.flags, sizeof(collisionFlags));
    LARGE_INTEGER before{}, after{};
    QueryPerformanceCounter(&before);
    const auto trace = [&](float alongWu, float remainingWu) {
        halo4_seat_aim::TracePass pass{};
        Halo4PhysicsRayCastInput input{};
        Halo4PhysicsRayCastResult output{};
        input.profile = 0x1A;
        memcpy(&input.collisionFlags, &collisionFlags, sizeof(collisionFlags));
        for (int axis = 0; axis < 3; ++axis)
        {
            input.start[axis] = ray.head[axis] + ray.dir[axis] * alongWu;
            input.end[axis] = input.start[axis] + ray.dir[axis] * remainingWu;
        }
        for (int i = 0; i < chainCount; ++i)
            input.ignoredObjects[i] = chain[i];
        input.ignoredObjectCount = static_cast<uint8_t>(chainCount);
        input.resultOptions[0] = 1;
        bool returned = false;
        pass.completed = Halo4SeatGuardedRayCast(&input, &output, returned);
        pass.hit = returned;
        pass.fraction = output.fraction;
        // An object hit (type 4, as the contact backend reads it) names its handle; anything else is structure.
        pass.object = output.type == 4 ? output.objectIndex : -1;
        return pass;
    };
    const auto result = halo4_seat_aim::TraceLookingPast(kSightTraceLengthWu, kSightTraceSkipWu,
        kLegacySightSelfStepWu, kLegacySightSelfPasses, root, chain, chainCount, trace, Halo4SightRootOf);
    QueryPerformanceCounter(&after);
    if (!result.completed)
    {
        Sight_NoteTraceFault(kTitle);
        f.faults.fetch_add(1, std::memory_order_relaxed);
        LOG("SIGHT Halo 4 seated trace: the native ray raised a fault; the trace stays off until the title "
            "reinstalls");
        return;
    }
    const uint64_t cost = static_cast<uint64_t>(after.QuadPart - before.QuadPart);
    f.traces.fetch_add(1, std::memory_order_relaxed);
    (result.hit ? f.traceHits : f.traceMisses).fetch_add(1, std::memory_order_relaxed);
    f.lookPasts.fetch_add(static_cast<uint64_t>(result.selfHits), std::memory_order_relaxed);
    if (result.exhausted)
        f.exhausted.fetch_add(1, std::memory_order_relaxed);
    f.costTicks.fetch_add(cost, std::memory_order_relaxed);
    uint64_t prior = f.costMaxTicks.load(std::memory_order_relaxed);
    while (cost > prior && !f.costMaxTicks.compare_exchange_weak(prior, cost, std::memory_order_relaxed)) {}
    auto& legacy = LegacySight(kTitle);
    legacy.seatedTraces.fetch_add(1, std::memory_order_relaxed);
    legacy.selfHits.fetch_add(static_cast<uint64_t>(result.selfHits), std::memory_order_relaxed);
    // The seat's own record (the shot witness reads it); with the stage-1 key, the sight layer's too: the aim
    // path converges on it only while every proof holds (Halo4SeatSightPrepare), so publishing alone bends
    // nothing.
    Halo4PublishSeatTrace(generation, result.hit, result.rangeWu, ray);
    if (g_config.halo4_seat_converge)
    {
        Sight_PublishTrace(kTitle, generation, result.hit, result.hit ? result.rangeWu : 0.0f, cost);
        f.published.fetch_add(1, std::memory_order_relaxed);
    }
}

// ---- the shot witness (0x1DCB8C's exit, the fire routine's call only) --------------------------------------
// The fire routine's aim call (0x617E95) names the aiming unit: the local seated occupant arms this thread's
// tag, which the assist exit consumes in the same fire. The fire detour clears it.
struct Halo4SeatShotTag
{
    bool active = false;
    uint32_t unit = UINT32_MAX;
};
thread_local Halo4SeatShotTag t_halo4SeatShot{};

struct Halo4SeatShotWindow
{
    SRWLOCK lock = SRWLOCK_INIT;
    uint32_t shots = 0, unmeasured = 0, capBound = 0, unassisted = 0, converged = 0, smoothed = 0;
    float turnSumDeg = 0.0f, turnMaxDeg = 0.0f, missSumMm = 0.0f, missMaxMm = 0.0f;
    float lastRangeM = 0.0f, lastMissMm = 0.0f, lastStart[3]{};   // [along, right, up] mm from the solve's eye
} g_halo4SeatShots;

// Halo4MuzzleAimDetour, the fire routine's call, after the native aim: whose shot is this?
void Halo4SeatSightNoteFiringAim(uint32_t unit, const Halo4VehicleInputState& seat)
{
    if (!(g_h4SeatSight.ready.load(std::memory_order_acquire) & kH4SeatAssistWitness) || !seat.seated)
        return;
    if (seat.unit == unit)
    {
        t_halo4SeatShot = {true, unit};
        return;
    }
    if (seat.parent == unit)
    {
        // The aiming unit is our own vehicle: no player on it, so the native assist is skipped and the shot
        // flies along the barrel (verifier item 12).
        AcquireSRWLockExclusive(&g_halo4SeatShots.lock);
        ++g_halo4SeatShots.unassisted;
        ReleaseSRWLockExclusive(&g_halo4SeatShots.lock);
    }
}

void Halo4SeatSightClearShot()
{
    t_halo4SeatShot = {};
}

void Halo4SeatSightMeasureShot(const float pre[3], const float post[3], const float origin[3])
{
    const auto dot = [](const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    const float preLength = std::sqrt(dot(pre, pre)), postLength = std::sqrt(dot(post, post));
    auto& w = g_halo4SeatShots;
    const uint32_t generation = TitleAdapter_GetGeneration(GameTitle::Halo4);
    SightRay ray{};
    SightTrace trace{};
    // The point: the published seated trace's smoothed range, the one the aim was bent through (stage 1);
    // otherwise the seated trace's own last result along the ray it traced (stage 0: nothing is bent, and the
    // miss is the unconverged aim's).
    float range = 0.0f;
    bool smoothed = false;
    bool measurable = std::isfinite(preLength) && preLength > 1.0e-6f && std::isfinite(postLength) &&
        postLength > 1.0e-6f && Halo4SeatFinite3(origin);
    if (measurable)
    {
        if (Sight_ReadRay(GameTitle::Halo4, generation, kSightRayFreshMs, ray) && ray.ignoreB != -1 &&
            Sight_ReadTrace(GameTitle::Halo4, generation, kSightFreshMs, trace))
        {
            range = trace.AimRangeWu();
            smoothed = true;
        }
        else
            measurable = Halo4ReadSeatTrace(generation, kSightFreshMs, ray, range);
    }
    if (!measurable)
    {
        AcquireSRWLockExclusive(&w.lock);
        ++w.unmeasured;
        ReleaseSRWLockExclusive(&w.lock);
        return;
    }
    const float d[3]{post[0] / postLength, post[1] / postLength, post[2] / postLength};
    float c = dot(pre, post) / (preLength * postLength);
    c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
    const float turnDeg = std::acos(c) * 57.29578f;
    // The point under the reticle's ray at that range.
    const float p[3]{ray.head[0] + ray.dir[0] * range, ray.head[1] + ray.dir[1] * range,
                     ray.head[2] + ray.dir[2] * range};
    const float toPoint[3]{p[0] - origin[0], p[1] - origin[1], p[2] - origin[2]};
    const float t = dot(toPoint, d);
    const float miss[3]{origin[0] + d[0] * t - p[0], origin[1] + d[1] * t - p[1], origin[2] + d[2] * t - p[2]};
    const float missMm = std::sqrt(dot(miss, miss)) * 3048.0f;
    // The shot's start relative to the solve's eye, in the line's frame (along, level right, up).
    float right[3]{d[1], -d[0], 0.0f};
    float rl = std::sqrt(right[0] * right[0] + right[1] * right[1]);
    if (!(rl > 1.0e-4f)) { right[0] = 1.0f; right[1] = 0.0f; rl = 1.0f; }
    right[0] /= rl;
    right[1] /= rl;
    const float up[3]{right[1] * d[2] - right[2] * d[1], right[2] * d[0] - right[0] * d[2],
                      right[0] * d[1] - right[1] * d[0]};
    const float se[3]{origin[0] - ray.eye[0], origin[1] - ray.eye[1], origin[2] - ray.eye[2]};
    AcquireSRWLockExclusive(&w.lock);
    ++w.shots;
    if (smoothed)
        ++w.smoothed;
    if (turnDeg >= halo4_seat_aim::kSeatAssistCapRad * 57.29578f - 0.05f)
        ++w.capBound;
    if (Sight_ConvergedLive(GameTitle::Halo4))
        ++w.converged;
    w.turnSumDeg += turnDeg;
    w.turnMaxDeg = std::fmax(w.turnMaxDeg, turnDeg);
    if (std::isfinite(missMm))
    {
        w.missSumMm += missMm;
        w.missMaxMm = std::fmax(w.missMaxMm, missMm);
    }
    w.lastMissMm = missMm;
    w.lastRangeM = std::sqrt(dot(toPoint, toPoint)) * 3.048f;
    w.lastStart[0] = dot(se, d) * 3048.0f;
    w.lastStart[1] = dot(se, right) * 3048.0f;
    w.lastStart[2] = dot(se, up) * 3048.0f;
    ReleaseSRWLockExclusive(&w.lock);
}

// 0x1DCB8C: forwards every caller exactly (all eight arguments as 64-bit values, plus the spare slots;
// tools/audit_caller_area.py halo4.dll 1DCB8C:12:i is clean: highest caller-area slot entry+0x40 = argument 8,
// no native caller relies on a volatile register across the call). Measures only the fire routine's call
// (return 0x618108) of a shot the local seated occupant aims.
__declspec(noinline) uint64_t __fastcall Halo4SeatAssistDetour(uint64_t a1, uint64_t a2, uint64_t a3, uint64_t a4,
    uint64_t a5, uint64_t a6, uint64_t a7, uint64_t a8)
{
    auto& f = g_h4SeatSight;
    f.assistCallbacks.fetch_add(1, std::memory_order_acq_rel);
    uint64_t result = 0;
    __try
    {
        bool measure = false;
        float pre[3]{};
        auto* direction = reinterpret_cast<float*>(a8);
        const auto* origin = reinterpret_cast<const float*>(a6);
        if (reinterpret_cast<uintptr_t>(_ReturnAddress()) == f.base + 0x618108 && t_halo4SeatShot.active &&
            direction && origin)
        {
            memcpy(pre, direction, sizeof(pre));
            measure = true;
        }
        if (f.assistOriginal)
            result = CallNativeWithSpareSlots(f.assistOriginal, a1, a2, a3, a4, a5, a6, a7, a8);
        if (measure)
        {
            t_halo4SeatShot = {};
            float post[3]{}, from[3]{};
            memcpy(post, direction, sizeof(post));
            memcpy(from, origin, sizeof(from));
            Halo4SeatSightMeasureShot(pre, post, from);
        }
    }
    __finally
    {
        f.assistCallbacks.fetch_sub(1, std::memory_order_acq_rel);
    }
    return result;
}

// ---- the input thread (Game_ComputeAimStick) -------------------------------------------------------------------
// What the aim path needs this poll for a Halo 4 seat. Seated, the on-foot eyes are never used: the controller
// root is the placed head there, not the point the game's aim line starts from.
struct Halo4SeatSightPoll
{
    bool seated = false;   // the camera hook's stamp names the local unit seated (fresh)
    bool ray = false;      // publish the seated intent ray: our seat view, the rendered head known, seats aimed
    bool solve = false;    // stage 1: every pre-bend proof holds (bend the aim through the traced point)
    bool turret = false;   // stage 1: a slaved-turret seat (the reticle rides the barrel, never claimed)
    uint32_t unit = UINT32_MAX, parent = UINT32_MAX;
    int16_t seat = -1;
    float head[3]{}, eye[3]{};          // H (the rendered centre eye) and the ray's eye (E when it is known)
    bool eyeKnown = false;
    bool assistKnown = false, assisted = false;   // the shot's aiming unit carries the player (the native assist)
    bool barrelKnown = false;
    float barrelForward[3]{}, barrelOrigin[3]{};  // the barrel at E + the sample's offset (render frame)
};

// Why a seat did not solve this poll: the first proof that failed (counted for the dev line).
enum Halo4SeatSightWhy : unsigned
{
    kH4SeatWhyView, kH4SeatWhyHead, kH4SeatWhySwitch, kH4SeatWhyEye, kH4SeatWhySample, kH4SeatWhyWitness,
    kH4SeatWhyClass, kH4SeatWhyRecord, kH4SeatWhyKey, kH4SeatWhySolved, kH4SeatWhyCount
};

// Why a converged seat aim was not CLAIMED (Game_ComputeAimStick, stage 1).
enum Halo4SeatClaimWhy : unsigned
{
    kH4ClaimYes, kH4ClaimTurret, kH4ClaimUnknown, kH4ClaimUnassisted, kH4ClaimCap, kH4ClaimCount
};

struct Halo4SeatEyeWitness
{
    std::atomic<uint64_t> polls{0}, provenPolls{0}, why[kH4SeatWhyCount]{}, claim[kH4ClaimCount]{};
    std::atomic<float> maxGapMm{0.0f}, speedMps{0.0f}, eyeFromHead[3]{}, maxDirDeg{0.0f}, maxBarrelDeg{0.0f};
    std::atomic<uint32_t> evidence[4]{};   // halo4_seat_aim::EyeEvidence counts over new samples
} g_h4SeatEyeWitness;

void Halo4SeatSightStoreMax(std::atomic<float>& slot, float value)
{
    if (std::isfinite(value) && value > slot.load(std::memory_order_relaxed))
        slot.store(value, std::memory_order_relaxed);
}

// Input thread. True while seated (out filled); false on foot or with no fresh stamp (out untouched beyond
// its defaults), which leaves the on-foot path exactly as it was.
bool Halo4SeatSightPrepare(Halo4SeatSightPoll& out, const float observerForward[3])
{
    struct State
    {
        uint32_t unit = UINT32_MAX, parent = UINT32_MAX, generation = 0, sampleSequence = 0;
        int16_t seat = -1;
        halo4_seat_aim::SeatEyeAgreement agreement{};
    };
    static State s_state;   // Game_ComputeAimStick's thread only, like the servo's own statics
    Halo4VehicleInputState stamp{};
    if (!Halo4ReadSeatStamp(stamp) || !stamp.seated || stamp.parent == UINT32_MAX || stamp.seat < 0)
        return false;
    out.seated = true;
    out.unit = stamp.unit;
    out.parent = stamp.parent;
    out.seat = stamp.seat;
    const uint32_t generation = TitleAdapter_GetGeneration(GameTitle::Halo4);
    if (s_state.unit != stamp.unit || s_state.parent != stamp.parent || s_state.seat != stamp.seat ||
        s_state.generation != generation)
    {
        // A new seat stint: nothing proven in the last one carries over.
        s_state = {};
        s_state.unit = stamp.unit;
        s_state.parent = stamp.parent;
        s_state.seat = stamp.seat;
        s_state.generation = generation;
    }
    auto& w = g_h4SeatEyeWitness;
    w.polls.fetch_add(1, std::memory_order_relaxed);
    const uint64_t now = GetTickCount64();
    const bool viewOwned = NativeVehicleFirstPerson_SeatViewOwned(GameTitle::Halo4, stamp.parent, stamp.seat);
    BoneMatrix center{}, body{};
    const bool headKnown = Halo4LoadCenterRoot(1.0f, center);
    float delta[3]{};
    out.eyeKnown = Halo4LoadSeatEyeRoot(stamp.parent, stamp.seat, 100, out.eye, delta);
    if (headKnown)
    {
        memcpy(out.head, center.translation, sizeof(out.head));
        if (!out.eyeKnown)
        {
            // The ray's eye only (the census's and the shot witness's reference): the controller root.
            if (Halo4LoadControllerRoot(1.0f, body))
                memcpy(out.eye, body.translation, sizeof(out.eye));
            else
                memcpy(out.eye, out.head, sizeof(out.eye));
        }
    }
    out.ray = viewOwned && headKnown && g_config.reach_vehicle_aim > 0;
    // The simulation's aim-camera sample for this seat.
    Halo4SeatAimSample sample{};
    const bool fresh = Halo4ReadSeatAimSample(sample) && sample.generation == generation && sample.seated &&
        sample.ms && now >= sample.ms && now - sample.ms <= kSightFreshMs;
    const bool sameSeat = fresh && sample.unit == stamp.unit && sample.parent == stamp.parent &&
        sample.seat == stamp.seat;
    if (fresh && sample.sequence != s_state.sampleSequence)
    {
        s_state.sampleSequence = sample.sequence;
        float gapM = std::numeric_limits<float>::quiet_NaN();
        if (out.eyeKnown)
        {
            const float g[3]{sample.camera[0] - out.eye[0], sample.camera[1] - out.eye[1],
                             sample.camera[2] - out.eye[2]};
            gapM = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]) * 3.048f;
        }
        const float speed = sample.speedMps >= 0.0f ? sample.speedMps : std::numeric_limits<float>::quiet_NaN();
        const auto evidence = halo4_seat_aim::JudgeEye(sample.witness, sameSeat && out.eyeKnown, gapM, speed);
        s_state.agreement.step(evidence, now);
        w.evidence[static_cast<unsigned>(evidence) & 3u].fetch_add(1, std::memory_order_relaxed);
        if (sameSeat && out.eyeKnown)
        {
            Halo4SeatSightStoreMax(w.maxGapMm, gapM * 1000.0f);
            w.speedMps.store(sample.speedMps, std::memory_order_relaxed);
        }
        // The game's aim direction on the tick against the observer forward the render transaction read.
        if (sameSeat && observerForward)
        {
            const float a = std::sqrt(sample.dir[0] * sample.dir[0] + sample.dir[1] * sample.dir[1] +
                                      sample.dir[2] * sample.dir[2]);
            const float b = std::sqrt(observerForward[0] * observerForward[0] +
                                      observerForward[1] * observerForward[1] +
                                      observerForward[2] * observerForward[2]);
            if (a > 1.0e-4f && b > 1.0e-4f)
            {
                float c = (sample.dir[0] * observerForward[0] + sample.dir[1] * observerForward[1] +
                           sample.dir[2] * observerForward[2]) / (a * b);
                c = c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
                Halo4SeatSightStoreMax(w.maxDirDeg, std::acos(c) * 57.29578f);
            }
        }
    }
    else if (!fresh)
    {
        // No sample: the proof ages out (a seat with no weapon never gets one -- its view call is gated by the
        // same weapon test as the shot's assist).
        s_state.agreement.step(halo4_seat_aim::EyeEvidence::Disagree, now);
    }
    if (headKnown && out.eyeKnown)
    {
        // E relative to the rendered head, in the head's frame (forward, right, up), for the dev line.
        const float e[3]{out.eye[0] - out.head[0], out.eye[1] - out.head[1], out.eye[2] - out.head[2]};
        const float* forward = center.rotation;
        const float* left = center.rotation + 3;
        const float* up = center.rotation + 6;
        w.eyeFromHead[0].store((e[0] * forward[0] + e[1] * forward[1] + e[2] * forward[2]) * 3048.0f,
                               std::memory_order_relaxed);
        w.eyeFromHead[1].store(-(e[0] * left[0] + e[1] * left[1] + e[2] * left[2]) * 3048.0f,
                               std::memory_order_relaxed);
        w.eyeFromHead[2].store((e[0] * up[0] + e[1] * up[1] + e[2] * up[2]) * 3048.0f, std::memory_order_relaxed);
    }
    const auto seatClass = sameSeat
        ? halo4_seat_aim::ClassifySeat(sample.flagsKnown, sample.flags, sample.controlledKind, sample.weaponKnown,
                                       sample.hasWeapon)
        : halo4_seat_aim::SeatClass::Unknown;
    const bool recordWorld = sameSeat && halo4_seat_aim::SeatRecordIsWorld(sample.flagsKnown, sample.flags,
        sample.upKnown, {sample.up[0], sample.up[1], sample.up[2]});
    if (s_state.agreement.proven)
        w.provenPolls.fetch_add(1, std::memory_order_relaxed);
    // THE PROOFS, every one required (any failure: this seat aims exactly as before). The kill switches are in
    // them: vehicle_first_person = 0 (the view is never ours), reach_vehicle_aim = 0 and converged_aim = 0 (no
    // seated ray), and the dev key itself.
    const bool key = g_config.halo4_seat_converge != 0;
    unsigned why = kH4SeatWhySolved;
    if (!viewOwned) why = kH4SeatWhyView;
    else if (!headKnown) why = kH4SeatWhyHead;
    else if (!out.ray || !Sight_AimEnabled()) why = kH4SeatWhySwitch;
    else if (!out.eyeKnown) why = kH4SeatWhyEye;
    else if (!sameSeat) why = kH4SeatWhySample;
    else if (!s_state.agreement.proven) why = kH4SeatWhyWitness;
    else if (seatClass == halo4_seat_aim::SeatClass::Unknown || seatClass == halo4_seat_aim::SeatClass::NoWeapon)
        why = kH4SeatWhyClass;
    else if (!recordWorld) why = kH4SeatWhyRecord;
    else if (!key) why = kH4SeatWhyKey;   // every proof holds; only the stage-1 key is off
    w.why[why].fetch_add(1, std::memory_order_relaxed);
    out.solve = why == kH4SeatWhySolved;
    // A slaved turret's aim camera IS its barrel's line (0x124DE6): the reticle rides the barrel, whatever the
    // eye proofs say, and nothing is ever claimed there (verifier item 14).
    out.turret = key && out.ray && Sight_AimEnabled() && seatClass == halo4_seat_aim::SeatClass::SlavedTurret;
    // The shot side's facts for the claim (stage 1).
    out.assistKnown = sameSeat && sample.aimUnitKnown;
    out.assisted = out.assistKnown && sample.assisted;
    out.barrelKnown = sameSeat && out.eyeKnown && sample.barrelKnown;
    if (out.barrelKnown)
    {
        memcpy(out.barrelForward, sample.barrelForward, sizeof(out.barrelForward));
        for (int axis = 0; axis < 3; ++axis)
            out.barrelOrigin[axis] = out.eye[axis] + sample.barrelOffset[axis];
    }
    return true;
}

// Stage 1, Game_ComputeAimStick, after the aim was bent through `point` (the intent ray at the converged range):
// may the converged aim be CLAIMED (verifier items 11 and 12)? The native seated assist turns the shot from the
// barrel toward the crosshair point by at most 21 deg, and only while the shot's aiming unit carries the player;
// claimed only with 2 deg to spare. Otherwise honestDir is the world direction from the rendered head to where
// the real shot is at the point's range, when that is known.
bool Halo4SeatSightClaim(const Halo4SeatSightPoll& seat, const float point[3], bool& honestValid,
                         float honestDir[3])
{
    using halo4_seat_aim::Vec3;
    honestValid = false;
    auto& w = g_h4SeatEyeWitness;
    unsigned why = kH4ClaimYes;
    if (seat.turret)
        why = kH4ClaimTurret;
    else if (!seat.barrelKnown || !seat.assistKnown)
        why = kH4ClaimUnknown;
    else
    {
        const Vec3 barrelOrigin{seat.barrelOrigin[0], seat.barrelOrigin[1], seat.barrelOrigin[2]};
        const Vec3 target{point[0], point[1], point[2]};
        const auto verdict = halo4_seat_aim::JudgeBarrel(
            {seat.barrelForward[0], seat.barrelForward[1], seat.barrelForward[2]}, barrelOrigin, target,
            seat.assisted);
        if (!verdict.valid)
            why = kH4ClaimUnknown;
        else
        {
            Halo4SeatSightStoreMax(w.maxBarrelDeg, verdict.angleRad * 57.29578f);
            if (!seat.assisted)
                why = kH4ClaimUnassisted;
            else if (!verdict.claim)
                why = kH4ClaimCap;
            Vec3 dir{};
            if (why != kH4ClaimYes &&
                halo4_seat_aim::ShotReticleDirection({seat.head[0], seat.head[1], seat.head[2]}, barrelOrigin,
                                                     verdict.shot, target, dir))
            {
                honestDir[0] = dir.x;
                honestDir[1] = dir.y;
                honestDir[2] = dir.z;
                honestValid = true;
            }
        }
    }
    w.claim[why].fetch_add(1, std::memory_order_relaxed);
    return why == kH4ClaimYes;
}

// Cold worker, every 2 s: the three dev lines.
void Halo4SeatSightReport()
{
    static uint64_t s_last = 0;
    const uint64_t now = GetTickCount64();
    if (now - s_last < 2000)
        return;
    s_last = now;
    auto& f = g_h4SeatSight;
    if (!f.ready.load(std::memory_order_acquire) && !f.assistTarget)
        return;
    auto& w = g_h4SeatEyeWitness;
    const uint64_t samples = f.samplesSeated.exchange(0, std::memory_order_relaxed);
    const uint64_t polls = w.polls.exchange(0, std::memory_order_relaxed);
    f.samples.exchange(0, std::memory_order_relaxed);
    if (samples || polls)
    {
        static const char* const kControlled[]{"unknown", "the occupant (own weapon)", "the vehicle", "a turret"};
        const uint32_t flags = f.lastFlags.load(std::memory_order_relaxed);
        const uint32_t assisted = f.lastAssisted.load(std::memory_order_relaxed);
        const uint32_t upZ = f.lastUpZ1e4.load(std::memory_order_relaxed);
        uint64_t why[kH4SeatWhyCount]{};
        for (unsigned i = 0; i < kH4SeatWhyCount; ++i)
            why[i] = w.why[i].exchange(0, std::memory_order_relaxed);
        uint32_t evidence[4]{};
        for (unsigned i = 0; i < 4; ++i)
            evidence[i] = w.evidence[i].exchange(0, std::memory_order_relaxed);
        char flagText[16], upText[16];
        if (flags == 0xFFFFFFFFu)
            snprintf(flagText, sizeof(flagText), "unknown");
        else
            snprintf(flagText, sizeof(flagText), "0x%02X", flags);
        if (upZ)
            snprintf(upText, sizeof(upText), "%.4f", double(upZ) / 10000.0 - 1.0);
        else
            snprintf(upText, sizeof(upText), "unknown");
        LOG("SIGHT Halo 4 seat eye: %llu sim samples (first person %llu, chase %llu, no witness %llu; chase pushes "
            "%llu; faults %llu, writer busy %llu); seat flags %s, class %s, controls %s, aiming unit carries the "
            "player %s, barrel %s, vehicle up.z %s; eye vs sim: agree %u disagree %u chase %u, max |E_sim - "
            "E_render| %.0f mm at %.1f m/s, E_render - head [along,right,up] %.0f,%.0f,%.0f mm, sim aim vs "
            "observer forward max %.2f deg; %llu polls, eye proven in %llu; not solved: view %llu, head %llu, "
            "switches %llu, eye %llu, sample %llu, witness %llu, class %llu, tilted record %llu, key off %llu; "
            "SOLVED %llu; claimed %llu, not claimed: turret %llu, shot unknown %llu, no assist %llu, beyond the cap "
            "%llu (max barrel->point %.1f deg)",
            static_cast<unsigned long long>(samples), static_cast<unsigned long long>(f.firstPerson.exchange(0)),
            static_cast<unsigned long long>(f.chase.exchange(0)), static_cast<unsigned long long>(f.noWitness.exchange(0)),
            static_cast<unsigned long long>(f.pushes.exchange(0)), static_cast<unsigned long long>(f.sampleFaults.exchange(0)),
            static_cast<unsigned long long>(f.writerBusy.exchange(0)), flagText,
            halo4_seat_aim::SeatClassName(static_cast<halo4_seat_aim::SeatClass>(f.lastClass.load())),
            kControlled[f.lastControlledKind.load() & 3u], assisted == 2 ? "unknown" : assisted ? "yes" : "NO",
            f.lastBarrel.load() ? "known" : "unknown", upText, evidence[1], evidence[2], evidence[3],
            double(w.maxGapMm.exchange(0.0f)), double(w.speedMps.load()), double(w.eyeFromHead[0].load()),
            double(w.eyeFromHead[1].load()), double(w.eyeFromHead[2].load()), double(w.maxDirDeg.exchange(0.0f)),
            static_cast<unsigned long long>(polls), static_cast<unsigned long long>(w.provenPolls.exchange(0)),
            static_cast<unsigned long long>(why[kH4SeatWhyView]), static_cast<unsigned long long>(why[kH4SeatWhyHead]),
            static_cast<unsigned long long>(why[kH4SeatWhySwitch]),
            static_cast<unsigned long long>(why[kH4SeatWhyEye]), static_cast<unsigned long long>(why[kH4SeatWhySample]),
            static_cast<unsigned long long>(why[kH4SeatWhyWitness]), static_cast<unsigned long long>(why[kH4SeatWhyClass]),
            static_cast<unsigned long long>(why[kH4SeatWhyRecord]), static_cast<unsigned long long>(why[kH4SeatWhyKey]),
            static_cast<unsigned long long>(why[kH4SeatWhySolved]),
            static_cast<unsigned long long>(w.claim[kH4ClaimYes].exchange(0)),
            static_cast<unsigned long long>(w.claim[kH4ClaimTurret].exchange(0)),
            static_cast<unsigned long long>(w.claim[kH4ClaimUnknown].exchange(0)),
            static_cast<unsigned long long>(w.claim[kH4ClaimUnassisted].exchange(0)),
            static_cast<unsigned long long>(w.claim[kH4ClaimCap].exchange(0)), double(w.maxBarrelDeg.exchange(0.0f)));
    }
    const uint64_t seatedUpdates = f.seatedUpdates.exchange(0, std::memory_order_relaxed);
    const uint64_t traces = f.traces.exchange(0, std::memory_order_relaxed);
    const uint64_t unnamed = f.unnamed.exchange(0, std::memory_order_relaxed);
    const uint64_t mismatched = f.mismatched.exchange(0, std::memory_order_relaxed);
    if (seatedUpdates || traces || unnamed || mismatched)
    {
        LARGE_INTEGER frequency{};
        QueryPerformanceFrequency(&frequency);
        const double usPerTick = frequency.QuadPart ? 1.0e6 / double(frequency.QuadPart) : 0.0;
        const uint64_t cost = f.costTicks.exchange(0, std::memory_order_relaxed);
        LOG("SIGHT Halo 4 seated trace: %llu updates of the seated local biped; %llu traces (%llu hits, %llu misses, "
            "%llu of them after three passes on our own vehicle), looked past our own vehicle %llu times; %llu seats "
            "untraced (the ray named no vehicle), %llu not this thread's seat, %llu root fallbacks; cost avg %.1f / "
            "max %.1f us; published to the sight layer %llu (halo4_seat_converge)",
            static_cast<unsigned long long>(seatedUpdates), static_cast<unsigned long long>(traces),
            static_cast<unsigned long long>(f.traceHits.exchange(0)), static_cast<unsigned long long>(f.traceMisses.exchange(0)),
            static_cast<unsigned long long>(f.exhausted.exchange(0)), static_cast<unsigned long long>(f.lookPasts.exchange(0)),
            static_cast<unsigned long long>(unnamed), static_cast<unsigned long long>(mismatched),
            static_cast<unsigned long long>(f.rootFallbacks.exchange(0)),
            traces ? double(cost) / double(traces) * usPerTick : 0.0,
            double(f.costMaxTicks.exchange(0)) * usPerTick, static_cast<unsigned long long>(f.published.exchange(0)));
    }
    auto& s = g_halo4SeatShots;
    AcquireSRWLockExclusive(&s.lock);
    const Halo4SeatShotWindow shots = [&] {
        Halo4SeatShotWindow copy{};
        copy.shots = s.shots; copy.unmeasured = s.unmeasured; copy.capBound = s.capBound;
        copy.unassisted = s.unassisted; copy.converged = s.converged; copy.smoothed = s.smoothed;
        copy.turnSumDeg = s.turnSumDeg;
        copy.turnMaxDeg = s.turnMaxDeg; copy.missSumMm = s.missSumMm; copy.missMaxMm = s.missMaxMm;
        copy.lastRangeM = s.lastRangeM; copy.lastMissMm = s.lastMissMm;
        memcpy(copy.lastStart, s.lastStart, sizeof(copy.lastStart));
        return copy;
    }();
    s.shots = s.unmeasured = s.capBound = s.unassisted = s.converged = s.smoothed = 0;
    s.turnSumDeg = s.turnMaxDeg = s.missSumMm = s.missMaxMm = 0.0f;
    ReleaseSRWLockExclusive(&s.lock);
    if (shots.shots || shots.unmeasured || shots.unassisted)
    {
        const float n = shots.shots ? float(shots.shots) : 1.0f;
        LOG("SHOT Halo 4 seated: %u shot(s) through the native seated assist (+%u unmeasured; %u while converged; "
            "%u against the published smoothed range, the rest against the seated trace's own last hit), turned "
            "avg %.2f / max %.2f deg from the barrel, %u at the 21 deg cap; the final line misses the point under "
            "the reticle by avg %.0f / max %.0f mm (last %.0f mm at %.2f m); the last left [along,right,up] "
            "%+.0f,%+.0f,%+.0f mm from the solve's eye; %u seated fire(s) aimed by our vehicle (no assist: the "
            "barrel's own line)",
            shots.shots, shots.unmeasured, shots.converged, shots.smoothed, double(shots.turnSumDeg / n),
            double(shots.turnMaxDeg),
            shots.capBound, double(shots.missSumMm / n), double(shots.missMaxMm), double(shots.lastMissMm),
            double(shots.lastRangeM), double(shots.lastStart[0]), double(shots.lastStart[1]),
            double(shots.lastStart[2]), shots.unassisted);
    }
}

// ---- install / remove ----------------------------------------------------------------------------------------
bool Halo4SeatSightUnique(uintptr_t base, size_t size, uintptr_t rva, const char* pattern)
{
    const uintptr_t hit = sig::Find(base, size, pattern);
    return hit == base + rva && !sig::Find(hit + 1, base + size - hit - 1, pattern);
}

bool Halo4SeatSightCalls(uintptr_t base, uintptr_t call, uintptr_t target)
{
    const auto* code = reinterpret_cast<const uint8_t*>(base + call);
    return code[0] == 0xE8 && base + call + 5 + *reinterpret_cast<const int32_t*>(code + 1) == base + target;
}

bool RemoveHalo4SeatSight()
{
    auto& f = g_h4SeatSight;
    f.ready.store(0, std::memory_order_release);
    if (!f.assistTarget)
        return true;
    const auto status = MCCVR_DisableHookForRetirement(f.assistTarget);
    if (status != MH_OK && status != MH_ERROR_DISABLED && status != MH_ERROR_NOT_CREATED)
    {
        LOG("SIGHT Halo 4 seats CleanupRequired: the assist witness hook did not disable");
        return false;
    }
    const void* functions[]{reinterpret_cast<const void*>(&Halo4SeatAssistDetour)};
    const void* trampolines[]{reinterpret_cast<const void*>(f.assistOriginal)};
    if (!WaitForNativeDetourQuiescence(functions, trampolines, 1, f.assistCallbacks))
    {
        LOG("SIGHT Halo 4 seats CleanupRequired: the assist witness is busy");
        return false;
    }
    const auto removed = MH_RemoveHook(f.assistTarget);
    if (removed != MH_OK && removed != MH_ERROR_NOT_CREATED)
    {
        LOG("SIGHT Halo 4 seats CleanupRequired: the assist witness hook was not removed");
        return false;
    }
    f.assistTarget = nullptr;
    f.assistOriginal = nullptr;
    return true;
}

// After the muzzle lane (its bindings are this lane's too). Every anchor its own feature bit; whatever fails
// says so once and leaves that piece -- and therefore seated convergence -- off.
void InstallHalo4SeatSight(uintptr_t base, size_t size, uint32_t generation)
{
    auto& f = g_h4SeatSight;
    f.ready.store(0, std::memory_order_release);
    if (f.assistTarget)
    {
        LOG("SIGHT Halo 4 seats: a previous assist witness has not retired; seated witnesses off");
        return;
    }
    if (!g_halo4BarrelBindingsReady.load(std::memory_order_acquire) || g_halo4Barrel.base != base)
    {
        LOG("SIGHT Halo 4 seats: the muzzle lane is not installed (its view detour carries the sample); seated "
            "witnesses and seated convergence off");
        return;
    }
    f.base = base;
    f.generation = generation;
    f.sampleVersion.store(0, std::memory_order_release);
    f.lastQpc = 0;
    f.last = {};
    g_halo4SeatEyeRoot.ms.store(0, std::memory_order_release);
    uint32_t ready = 0;
    // The muzzle lane proved these entries at exactly these addresses.
    f.weaponInSlot = reinterpret_cast<Halo4SeatWeaponInSlotFn>(base + 0x5FA1C4);
    f.weaponOwner = reinterpret_cast<Halo4SeatHandleFn>(base + 0x612238);
    f.aimingUnit = reinterpret_cast<Halo4SeatHandleFn>(base + 0x5F1ABC);
    f.barrelMarker = reinterpret_cast<Halo4SeatBarrelMarkerFn>(base + 0x610F6C);
    f.markers = reinterpret_cast<Halo4MuzzleMarkersFn>(base + 0x5D5B74);
    // ADDR-HYGIENE: resolved -- each entry below is a unique signature at its address AND the native call edge
    // that proves how the game calls it; nothing is assumed on another build.
    if (Halo4SeatSightUnique(base, size, 0x5FB2D4, "48 89 5C 24 08 57 48 83 EC 20 0F B7 DA 41 8B F8 BA 03 20 00 00 "
            "E8 ?? ?? ?? ?? 48 85 C0 74 5A") &&
        Halo4SeatSightUnique(base, size, 0x5FB31D, "42 3B 84 82 10 05 00 00 7D 26 42 8B 94 82 14 05 00 00 48 0F BF "
            "C3 48 6B C8 5B 48 03 CA 48 C1 EA 1C 49 8B 04 D2 8B 04 88 0F A3 F8 0F 92 C0") &&
        Halo4SeatSightUnique(base, size, 0xA88E4, "0F B7 55 CB 41 B8 02 00 00 00 8B 4D C7 E8 ?? ?? ?? ?? 84 C0 74 3B") &&
        Halo4SeatSightCalls(base, 0xA88F1, 0x5FB2D4))
    {
        f.seatFlag = reinterpret_cast<Halo4SeatFlagFn>(base + 0x5FB2D4);
        ready |= kH4SeatFlags;
    }
    else
        LOG("SIGHT Halo 4 seats: the seat-flag accessor (0x5FB2D4, its body, the dispatcher's call 0xA88F1) did "
            "not prove; every seat is class Unknown and none converges");
    if (Halo4SeatSightUnique(base, size, 0x5F9694, "48 89 5C 24 10 48 89 6C 24 18 48 89 74 24 20 57 41 54 41 56 48 83 "
            "EC 20 41 83 CE FF 8B F9 41 3B CE 0F 84 ?? ?? ?? ?? 8B 15 ?? ?? ?? ?? 65 48 8B 04 25 58 00 00 00 BE 18 00 "
            "00 00 0F B7 CF 48 8B 04 D0 48 8B 34 06 48 8D 04 49 48 8B 56 50 4C 8B 44 C2 10") &&
        Halo4SeatSightCalls(base, 0x1DCA0E, 0x5F9694))
    {
        f.controlled = reinterpret_cast<Halo4SeatHandleFn>(base + 0x5F9694);
        ready |= kH4SeatControlled;
    }
    else
        LOG("SIGHT Halo 4 seats: the controlled-unit query (0x5F9694, the aim view's call 0x1DCA0E) did not prove; "
            "seats are class Unknown, none converges, and the seated trace ignores no turret");
    if (Halo4SeatSightUnique(base, size, 0x5D84D0, "48 83 EC 28 44 8B 0D ?? ?? ?? ?? 4D 8B D0 65 48 8B 04 25 58 00 00 "
            "00 4C 8B C2 BA 18 00 00 00 0F B7 C9 4A 8B 04 C8 4C 8B 0C 02 48 8D 04 49 49 8B 51 50 4C 8B 4C C2 10 41 83 "
            "79 24 FF 75 38") &&
        Halo4SeatSightUnique(base, size, 0x5F98C2, "41 C1 E8 04 41 F6 C0 01 0F 85 ?? ?? ?? ?? 4C 8D 45 97 41 8B CE 48 "
            "8D 55 A7 E8 ?? ?? ?? ??") &&
        Halo4SeatSightCalls(base, 0x5F98DB, 0x5D84D0))
    {
        f.objectFrame = reinterpret_cast<Halo4SeatObjectFrameFn>(base + 0x5D84D0);
        ready |= kH4SeatObjectFrame;
    }
    else
        LOG("SIGHT Halo 4 seats: the object frame (0x5D84D0, the record rebase's call 0x5F98DB) did not prove; "
            "only seats whose record is in world angles (flag bit 4) can converge");
    if (Halo4SeatSightUnique(base, size, 0x6121B8, "48 83 EC 28 65 48 8B 04 25 58 00 00 00 44 8B C9 8B 0D ?? ?? ?? ?? "
            "41 B8 18 00 00 00 48 8B 04 C8 41 0F B7 C9 4D 8B 04 00 48 8D 14 49 49 8B 40 50 4C 8B 54 D0 10 48 8B 05 ?? "
            "?? ?? ??") &&
        Halo4SeatSightUnique(base, size, 0x61797F, "8B 54 24 78 41 8B CD E8 ?? ?? ?? ?? 41 8B CD 8B D8 E8 ?? ?? ?? ?? "
            "33 C9 4C 8D 85 E0 06 00 00 88 4C 24 30 8B D3 88 4C 24 28 88 4C 24 20 44 8D 49 40 8B C8 E8 ?? ?? ?? ??") &&
        Halo4SeatSightCalls(base, 0x617986, 0x610F6C) && Halo4SeatSightCalls(base, 0x617990, 0x6121B8) &&
        Halo4SeatSightCalls(base, 0x6179B2, 0x5D5B74))
    {
        f.markerOwner = reinterpret_cast<Halo4SeatHandleFn>(base + 0x6121B8);
        ready |= kH4SeatBarrel;
    }
    else
        LOG("SIGHT Halo 4 seats: the barrel marker read (0x6121B8, the fire routine's marker block 0x61797F) did "
            "not prove; no seat claims convergence (the reticle stays honest)");
    if (Halo4SeatSightUnique(base, size, 0x617DA7, "41 8B CC E8 ?? ?? ?? ?? 8B C8 8B D8 E8 ?? ?? ?? ?? 48 8B 48 10 8B 81 "
            "9C 04 00 00 44 8B A1 84 04 00 00 89 44 24 6C 48 8D 81 A8 05 00 00") &&
        Halo4SeatSightCalls(base, 0x617DAA, 0x5F1ABC))
        ready |= kH4SeatAimUnit;
    else
        LOG("SIGHT Halo 4 seats: the fire routine's aiming unit (0x617DA7: 0x5F1ABC, its +0x49C player) did not "
            "prove; no seat claims convergence (the reticle stays honest)");
    if (Halo4SeatSightUnique(base, size, 0x2061A8, "48 89 5C 24 10 48 89 6C 24 18 56 57 41 56 48 83 EC 20 48 8B FA 8B F1 "
            "48 8D 54 24 40 45 8B F1 49 8B D8 E8 ?? ?? ?? ?? 44 8B 1D ?? ?? ?? ?? 8B E8 65 4C 8B 14 25 58 00 00 00 44 0F "
            "B7 C6 B9 18 00 00 00 4B 8B 04 DA 4B 8D 14 40 48 8B 0C 01 48 8B 49 50 4C 8B 44 D1 10 41 8B 88 90 04 00 00 "
            "48 C1 E9 1F F6 C1 01 74 3C B9 08 00 00 00 48 8B 0C 01 41 0F B7 80 9C 04 00 00 48 69 D0 18 0F 00 00 48 03 "
            "51 50 F2 0F 10 42 40 F2 0F 11 07 8B 42 48 89 47 08 F2 0F 10 42 4C F2 0F 11 03 8B 42 54 89 43 08 EB 23 45 "
            "33 C9 4C 8B C3 48 8B D7 8B CE 85 ED 75 0F 41 83 FE 01 41 0F 94 C1 E8 ?? ?? ?? ?? EB 05 E8 ?? ?? ?? ?? 48 "
            "8B 5C 24 48 8B C5 48 8B 6C 24 50 48 83 C4 20 41 5E 5F 5E C3") &&
        Halo4SeatSightCalls(base, 0x2061CA, 0x10D268) && Halo4SeatSightCalls(base, 0x1DCA25, 0x2061A8))
        ready |= kH4SeatAimCamera;
    else
        LOG("SIGHT Halo 4 seats: the aim camera (0x2061A8 with its selector call and +0x490 test, the aim view's "
            "call 0x1DCA25) did not prove; no first-person witness, no sample, no seated convergence");
    if (Halo4SeatSightUnique(base, size, 0x1DCB8C, "48 8B C4 48 89 58 08 48 89 70 10 48 89 78 18 55 41 56 41 57 48 8D 68 "
            "C1 48 81 EC E0 00 00 00 48 8B 5D 7F 33 FF 0F 29 70 D8 45 85 C9 44 8A 4D 67 0F 29 78 C8 44 0F 29 40 B8 41 8B "
            "00 4C 8D 45 5F 89 45 5F 0F 94 C0") &&
        Halo4SeatSightCalls(base, 0x618103, 0x1DCB8C))
    {
        void* const target = reinterpret_cast<void*>(base + 0x1DCB8C);
        if (MH_CreateHook(target, reinterpret_cast<void*>(&Halo4SeatAssistDetour),
                reinterpret_cast<void**>(&f.assistOriginal)) == MH_OK)
        {
            f.assistTarget = target;
            if (MH_EnableHook(target) == MH_OK)
                ready |= kH4SeatAssistWitness;
            else
                LOG("SIGHT Halo 4 seats: the assist witness hook did not enable; no SHOT Halo 4 seated line");
        }
        else
        {
            f.assistOriginal = nullptr;
            LOG("SIGHT Halo 4 seats: the assist witness hook was not created; no SHOT Halo 4 seated line");
        }
    }
    else
        LOG("SIGHT Halo 4 seats: the seated assist (0x1DCB8C, the fire routine's call 0x618103) did not prove; no "
            "SHOT Halo 4 seated line");
    f.ready.store(ready, std::memory_order_release);
    LOG("SIGHT Halo 4 seats (dev): witnesses %s -- seat flags %d, controlled unit %d, object frame %d, barrel %d, "
        "aiming unit %d, aim camera %d, assist exit %d",
        ready == 0x7Fu ? "all proven" : "PARTIAL", (ready & kH4SeatFlags) ? 1 : 0, (ready & kH4SeatControlled) ? 1 : 0,
        (ready & kH4SeatObjectFrame) ? 1 : 0, (ready & kH4SeatBarrel) ? 1 : 0, (ready & kH4SeatAimUnit) ? 1 : 0,
        (ready & kH4SeatAimCamera) ? 1 : 0, (ready & kH4SeatAssistWitness) ? 1 : 0);
}

#endif   // HALOMCCVR_DEV_DIAGNOSTICS
