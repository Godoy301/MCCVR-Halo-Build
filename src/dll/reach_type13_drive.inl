// Included in game.cpp at file scope, right after type13_flight_log.inl (which only a dev build includes).
//
// THE SABRE'S DIRECT DRIVE. Reach's per-user control dispatcher 0x6350C calls a dedicated attitude controller,
// 0x64828, for the pilot of a physics type-13 craft (the Sabre, the Seraph) and jumps past the desired-angles
// accumulator the direct drive's hook sits on. So that pilot was never written and stayed on the stick servo. This
// detour runs the game's controller unchanged and then, for the first local player, hands the same record to the
// direct drive's own apply (ReachPcApply): the hand's yaw and pitch go into player control's desired angles after
// the controller has written them and before the dispatcher derives its deltas from them, exactly as on foot and in
// every other seat. Roll stays the game's (the controller's own roll rate and auto-level), and so does everything
// that limits the craft: the hull chases the desired facing through its own gain and torque limit, downstream of
// the write.
//
// THE RECORD IS IN WORLD ANGLES here. The controller builds its attitude from yaw, pitch and roll and writes the
// Euler angles of the result back, so forward(yaw, pitch) is the craft's desired forward at any roll, and the craft's
// desired facing (vehicle +0x1E4, the vector the hull steers by) is that forward, one tick later. No seat frame
// applies and there is no seat limit to respect.
//
// WHAT A FLIGHT WAITS FOR (a stint: one craft flown without a break):
//   * THE LINK. That the record written here is what the hull steers by is measured, not assumed: on each new game
//     tick the craft's desired facing has to equal the forward of the record as the previous call left it, for
//     kReachSabreLinkTicks ticks running. Until then nothing is written and the drive's heartbeat is not stamped,
//     so the aim provider keeps the stick servo (a drive that reads ready and writes nothing would hold the stick
//     neutral: dead aim). kReachSabreLinkMissTicks ticks out of agreement end the proof, and the servo takes over
//     again.
//   * THE GAME'S OWN STEERING. Past the play space's edge the game stamps a direction on the craft (vehicle +0xABC
//     the tick, +0xAC0 the direction) and the controller turns it there for the tag's window. While a stamp is
//     that fresh the update writes nothing (the apply's no-write action), the stick stays neutral, and the craft is
//     the game's to turn.
// THE WRITE'S RATE. The hull steers on the angle between the craft and its desired facing with a stiff gain (the nose
// trails a turning facing by a few degrees), so a hand flicked across the cockpit would whip the craft. The written
// direction moves at most kReachSabreMaxTurnDegPerSecond, the fastest the game's own stick turned the record
// (78 degrees a second at full deflection, measured).
//
// Keys: the seat direct drive's own (aim_direct_drive, seat_direct_drive, and reach_vehicle_aim = 2 for a pilot,
// who is a look-steered driver). With any of them off the detour only runs the game's controller.
//
// The detour is the one hook on the controller in every build. In a dev build with type13_log = 1 it also feeds
// the log-only observer (type13_flight_log.inl) around the native call, before anything here is written.

#if HALOMCCVR_EXPERIMENTAL_REACH_RENDER_CANDIDATE
namespace
{
// ADDR-HYGIENE: guarded. Every RVA and offset below is proven by the install before a hook exists: the entry and
// the dispatcher's type-13 call are unique in the module, the call's rel32 lands on the entry, the dispatcher's
// frame is proven from its unwind info and prologue, its stick-update caller calls the dispatcher, and the rest
// match byte for byte at their RVAs (each is the instruction that reads the field this file reads). A miss: no
// hook, one line naming what failed, the pilot keeps the stick servo. The whole Reach image is hash-pinned before
// the camera core installs as well.
struct ReachSabreAnchor
{
    const char* name;
    uint32_t rva;
    bool unique;
    const char* pattern;
};
constexpr uint32_t kReachSabreEntryRva = 0x64828;
constexpr uint32_t kReachSabreCallRva = 0x63607;        // the dispatcher's call of the controller
constexpr uint32_t kReachSabreDispatcherRva = 0x6350C;
constexpr ReachSabreAnchor kReachSabreAnchors[] = {
    {"ENTRY", 0x64828, true,
     "48 8B C4 55 53 56 57 41 54 41 55 41 56 41 57 48 8D A8 18 FF FF FF 48 81 EC A8 01 00 00 0F 29 70 A8 0F 29 78 98 "
     "44 0F 29 40 88 44 0F 29 88 78 FF FF FF 44 0F 29 90 68 FF FF FF 44 0F 29 98 58 FF FF FF 44 0F 29 A0 48 FF FF FF "
     "44 0F 29 A8 38 FF FF FF 44 0F 29 B0 28 FF FF FF 44 0F 29 B8 18 FF FF FF 48 8B 05 ?? ?? ?? ?? 48 33 C4 48 89 45 F0 "
     "65 48 8B 04 25 58 00 00 00 48 8D 35 ?? ?? ?? ?? 44 8B 05 ?? ?? ?? ?? 45 0F 57 C9 41 B9 10 00 00 00 41 BD D8 00 00 00"},
    // The dispatcher: physics type == 13 -> the controller -> jmp past the accumulator's call.
    {"CALL", 0x635D5, true,
     "8B 08 E8 ?? ?? ?? ?? 83 F8 0D 75 ?? F3 0F 10 45 7F 0F 28 DF F3 0F 10 4D 77 0F 28 D6 8B 55 C7 8B CF "
     "F3 0F 11 44 24 30 F3 0F 11 4C 24 28 4C 89 6C 24 20 E8 ?? ?? ?? ?? EB ??"},
    // The record: player control = *(TLS + 0xD8) + user * 0x180; the controller works on +0x80 of it.
    {"REC", 0x648B3, false,
     "41 B9 10 00 00 00 41 BD D8 00 00 00 F3 0F 11 54 24 48 4E 8B 04 C0 48 63 C1 0F B7 CA F3 0F 11 5C 24 50 4F 8B 2C 28 "
     "4C 8D 24 40 4C 89 45 A8 4B 8B 04 01 48 8D 14 49 49 C1 E4 07 48 89 54 24 68 49 8D 9D 80 00 00 00 49 03 DC"},
    {"ANGPTR", 0x64C88, false, "4C 8D 7B 14"},   // the Euler output: player control + 0x94 (yaw, pitch, roll)
    // The forced steer's stamp: direction vehicle +0xAC0 / +0xAC8, tick vehicle +0xABC.
    {"STAMP", 0x64D6A, false, "F2 41 0F 10 85 C0 0A 00 00 41 8B 85 C8 0A 00 00 45 8B B5 BC 0A 00 00"},
    // Its window: the tag block's +0xAC against game time (TLS + 0xA0: the tick at +0xC, seconds per tick at +8).
    {"WIN", 0x64E58, false,
     "F3 0F 10 84 BE AC 00 00 00 B9 A0 00 00 00 4A 8B 0C 21 8B 41 0C 41 2B C6 66 0F 6E C8 0F 5B C9 F3 0F 59 49 08 "
     "0F 2F C1 0F 86 ?? ?? ?? ??"},
    // The hull (0x51DE3C) reading the desired facing it steers by, vehicle +0x1E4 / +0x1E8 / +0x1EC ...
    {"HX", 0x51E551, false,
     "F3 0F 10 B6 E8 01 00 00 F3 0F 10 BE E4 01 00 00 0F 28 C6 0F 28 CF E8 ?? ?? ?? ?? F3 0F 59 FF F3 0F 11 45 C0 "
     "F3 0F 59 F6 F3 0F 58 FE 0F 28 C7 E8 ?? ?? ?? ?? 0F 28 C8 F3 0F 10 86 EC 01 00 00 E8 ?? ?? ?? ?? F3 0F 10 B6 50 02 00 00"},
    // ... and the craft's own forward, vehicle +0x50 / +0x54 / +0x58 (the nose, for the report).
    {"HFWD", 0x51DF73, false,
     "F3 0F 10 4E 54 F3 0F 10 46 50 F3 0F 59 C0 F3 0F 59 C9 F3 0F 58 C1 E8 ?? ?? ?? ?? 0F 28 C8 F3 0F 10 46 58"},
};
// The dispatcher's own callers: [0] the action / kit consumer (an extra call), [1] the stick update, [2] the path
// with no unit. [1] must be proven (it is the update); the others read as unknown when they are not.
constexpr uint32_t kReachSabreCallerRvas[3] = {0x5EE43, 0x62467, 0x628A3};

// The tag's forced-steer window (the type-13 block's +0xAC) as measured in the retail Sabre: one second. The game
// re-stamps while the craft stays outside, so a craft whose tag gave another window is only handed back a little
// late or early; nothing else follows from the constant.
constexpr float kReachSabreSteerWindowSeconds = 1.0f;
constexpr uint32_t kReachSabreLinkTicks = 20;
constexpr uint32_t kReachSabreLinkMissTicks = 3;
constexpr float kReachSabreLinkAgreeDeg = 0.5f;
constexpr float kReachSabreMaxTurnDegPerSecond = 80.0f;

using ReachSabreControllerFn = void (*)(int, uint32_t, float, float, const void*, float, float);

struct ReachSabreDrive
{
    // Install (worker). Read by the hook only while it is enabled.
    uintptr_t base = 0;
    uint32_t dispatcherFrame = 0;     // the dispatcher's own return address: controller entry rsp + this many bytes
    uintptr_t callerRet[3]{};
    std::atomic<uint32_t> generation{0};
    std::atomic<uint32_t> resetSeq{0};
    // Simulation thread.
    uint32_t seenReset = 0;
    uint32_t vehicle = UINT32_MAX;    // the stint's craft
    int32_t lastTick = INT32_MIN;
    uint32_t agree = 0, misses = 0;
    bool proven = false;
    bool haveEnd = false;
    float endRecord[2]{};             // the record as the last call left it: what the next tick steers by
    // Simulation thread -> worker.
    std::atomic<uint32_t> calls{0}, ticks{0}, stints{0}, gated{0}, unread{0}, waiting{0}, linkMisses{0}, yielded{0},
        updates{0}, extras{0}, noUnit{0}, unknownCaller{0}, capped{0};
    std::atomic<uint32_t> state{0};   // 0 keys off / no flight, 1 waiting for the link, 2 proven
    std::atomic<float> noseMaxDeg{0.0f}, linkMaxDeg{0.0f};
} g_reachSabre;

bool ReachSabreWanted()
{
    const auto& d = g_reachSabre;
    return g_reachT13Enabled.load(std::memory_order_acquire) && g_config.reach_direct_drive == 3 &&
           g_config.seat_direct_drive != 0 && g_config.reach_vehicle_aim == 2 &&
           !g_reachCamera.teardownRequested.load(std::memory_order_acquire) &&
           g_reachCamera.generation.load(std::memory_order_acquire) == d.generation.load(std::memory_order_acquire);
}

struct ReachSabreState
{
    const uint8_t* pc;    // the first local player's player control
    float facing[3];      // the craft's desired facing (what the hull steers by)
    float nose[3];        // the craft's own forward
    int32_t stamp;        // the forced steer's tick (-1: none)
    int32_t tick;
    float secondsPerTick;
};

// Sim thread, guarded: everything the decision needs, from the calling thread's own TLS and the craft's datum.
bool ReachSabreRead(uint32_t vehicle, ReachSabreState& s)
{
    __try
    {
        auto** slots = reinterpret_cast<const uint8_t**>(__readgsqword(0x58));
        const uint32_t index = *reinterpret_cast<const uint32_t*>(g_reachCamera.base + kReachEngineTlsIndexRva);
        const uint8_t* tls = (slots && index < 0x200) ? slots[index] : nullptr;
        if (!tls)
            return false;
        const uintptr_t globals = *reinterpret_cast<const uintptr_t*>(tls + kReachPlayerControlTlsOffset);   // REC
        const auto* time = *reinterpret_cast<const uint8_t* const*>(tls + 0xA0);                             // WIN
        if (!globals || !time)
            return false;
        s.pc = reinterpret_cast<const uint8_t*>(globals);
        memcpy(&s.tick, time + 0xC, sizeof(s.tick));
        memcpy(&s.secondsPerTick, time + 8, sizeof(s.secondsPerTick));
        uint8_t kind = 0;
        const unsigned char* craft = ReachVehicleObjectData(static_cast<int32_t>(vehicle), kind);
        if (!craft || kind != kReachObjectKindVehicle)
            return false;
        memcpy(s.facing, craft + 0x1E4, sizeof(s.facing));            // HX
        memcpy(s.nose, craft + 0x50, sizeof(s.nose));                 // HFWD
        memcpy(&s.stamp, craft + 0xABC, sizeof(s.stamp));             // STAMP
        return std::isfinite(s.facing[0]) && std::isfinite(s.facing[1]) && std::isfinite(s.facing[2]) &&
               std::isfinite(s.nose[0]) && std::isfinite(s.nose[1]) && std::isfinite(s.nose[2]) &&
               s.secondsPerTick > 0.001f && s.secondsPerTick < 0.2f;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Sim thread, guarded: the record's yaw and pitch as they stand now (ANGPTR).
bool ReachSabreRecordNow(const uint8_t* pc, float out[2])
{
    __try
    {
        memcpy(out, pc + 0x94, 2 * sizeof(float));
        return std::isfinite(out[0]) && std::isfinite(out[1]);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// The angle between two vectors of any length, degrees; 180 when either is undefined.
float ReachSabreAngleDeg(const float a[3], const float b[3])
{
    const float la = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const float lb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    if (!(la > 1.0e-4f) || !(lb > 1.0e-4f) || !std::isfinite(la) || !std::isfinite(lb))
        return 180.0f;
    const float dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb);
    return std::acos((std::max)(-1.0f, (std::min)(1.0f, dot))) * 57.29578f;
}

void ReachSabreMax(std::atomic<float>& slot, float value)
{
    float prior = slot.load(std::memory_order_relaxed);
    while (value > prior && !slot.compare_exchange_weak(prior, value, std::memory_order_relaxed)) {}
}

// Sim thread, after the game's controller has written the record. `entry`: the controller's own entry rsp;
// `dtA`: the controller's own time step for this update.
void ReachSabreAfterController(int user, uint32_t vehicle, float dtA, const uintptr_t* entry)
{
    auto& d = g_reachSabre;
    d.calls.fetch_add(1, std::memory_order_relaxed);
    if (user != 0)
        return;
    if (!ReachSabreWanted())
    {
        d.gated.fetch_add(1, std::memory_order_relaxed);
        d.vehicle = UINT32_MAX;   // the next wanted call starts a stint afresh
        d.state.store(0, std::memory_order_relaxed);
        return;
    }
    const uint32_t reset = d.resetSeq.load(std::memory_order_acquire);
    if (reset != d.seenReset)
    {
        d.seenReset = reset;
        d.vehicle = UINT32_MAX;
    }
    ReachSabreState s{};
    if (!ReachSabreRead(vehicle, s))
    {
        d.unread.fetch_add(1, std::memory_order_relaxed);
        d.vehicle = UINT32_MAX;
        d.state.store(0, std::memory_order_relaxed);
        return;
    }
    if (vehicle != d.vehicle)
    {
        d.vehicle = vehicle;
        d.lastTick = s.tick;      // judged from the next tick on: nothing is known of the record before this call
        d.agree = d.misses = 0;
        d.proven = false;
        d.haveEnd = false;
        d.stints.fetch_add(1, std::memory_order_relaxed);
    }
    // The link, once a game tick: the tick that just ran steered by the record as the previous call left it.
    if (s.tick != d.lastTick)
    {
        d.lastTick = s.tick;
        d.ticks.fetch_add(1, std::memory_order_relaxed);
        if (d.haveEnd)
        {
            const float cp = std::cos(d.endRecord[1]);
            const float forward[3]{cp * std::cos(d.endRecord[0]), cp * std::sin(d.endRecord[0]),
                                   std::sin(d.endRecord[1])};
            const float off = ReachSabreAngleDeg(forward, s.facing);
            ReachSabreMax(d.linkMaxDeg, off);
            if (off < kReachSabreLinkAgreeDeg)
            {
                d.misses = 0;
                if (d.agree < kReachSabreLinkTicks && ++d.agree == kReachSabreLinkTicks)
                    d.proven = true;
            }
            else
            {
                d.linkMisses.fetch_add(1, std::memory_order_relaxed);
                if (++d.misses >= kReachSabreLinkMissTicks)
                {
                    d.agree = 0;
                    d.proven = false;
                }
            }
        }
        ReachSabreMax(d.noseMaxDeg, ReachSabreAngleDeg(s.nose, s.facing));
    }
    d.state.store(d.proven ? 2u : 1u, std::memory_order_relaxed);
    if (d.proven)
    {
        // Which of the dispatcher's callers this is.
        const uintptr_t dispatcherReturn = d.dispatcherFrame ? entry[d.dispatcherFrame / sizeof(uintptr_t)] : 0;
        ReachApplySite site{};
        const float step = std::isfinite(dtA) && dtA > 0.0f ? (std::min)(dtA, 0.1f) : s.secondsPerTick;
        site.maxStepRad = kReachSabreMaxTurnDegPerSecond * 0.01745329f * step;
        site.capped = &d.capped;
        if (dispatcherReturn && dispatcherReturn == d.callerRet[1])
        {
            // THE UPDATE -- unless the game is steering the craft itself (a stamp younger than its window).
            const bool steering = s.stamp != -1 && s.tick >= s.stamp &&
                static_cast<float>(s.tick - s.stamp) * s.secondsPerTick < kReachSabreSteerWindowSeconds;
            site.action = steering ? aim_caller::Action::EndUpdate : aim_caller::Action::StepAndWrite;
            (steering ? d.yielded : d.updates).fetch_add(1, std::memory_order_relaxed);
        }
        else if (dispatcherReturn && dispatcherReturn == d.callerRet[2])
        {
            site.action = aim_caller::Action::EndUpdate;
            d.noUnit.fetch_add(1, std::memory_order_relaxed);
        }
        else
        {
            // An extra call in the same update (or a caller this build does not know): the last write is put back.
            site.action = aim_caller::Action::RestoreOnly;
            (dispatcherReturn && dispatcherReturn == d.callerRet[0] ? d.extras : d.unknownCaller)
                .fetch_add(1, std::memory_order_relaxed);
        }
        ReachPcApply(0, nullptr, &site);
    }
    else
        d.waiting.fetch_add(1, std::memory_order_relaxed);   // no apply, so no heartbeat: the stick servo aims
    d.haveEnd = ReachSabreRecordNow(s.pc, d.endRecord);
}
} // namespace

// MinHook's relay jumps here, so _AddressOfReturnAddress() is the native's own entry rsp. The callback count spans
// the whole call, the native included: the teardown's quiescence proof and its ingress scan cover it. The native
// takes seven arguments, three of them on the stack, and reads them at entry +0x28 / +0x30 / +0x38, so it is
// forwarded through CallNativeWithSpareSlots (native_forward.h).
__declspec(noinline) void ReachType13Detour(int user, uint32_t vehicle, float yawD, float pitchD, const void* input,
                                            float dtA, float dtB)
{
    const auto* entry = static_cast<const uintptr_t*>(_AddressOfReturnAddress());
#if HALOMCCVR_DEV_DIAGNOSTICS
    volatile bool observing = false;
#endif
    g_reachCamera.activeCallbacks.fetch_add(1, std::memory_order_acq_rel);
    __try
    {
        const auto native = reinterpret_cast<ReachSabreControllerFn>(g_reachT13Orig);
        if (!native)
            return;
#if HALOMCCVR_DEV_DIAGNOSTICS
        T13Call call{};
        observing = T13ReachBefore(user, vehicle, yawD, pitchD, input, dtA, dtB, entry, call);
#endif
        CallNativeWithSpareSlots(native, user, vehicle, yawD, pitchD, input, dtA, dtB);
#if HALOMCCVR_DEV_DIAGNOSTICS
        if (observing)
        {
            observing = false;
            T13ReachAfter(call);   // releases the observer's busy flag; it reads what the native wrote, before ours
        }
#endif
        ReachSabreAfterController(user, vehicle, dtA, entry);
    }
    __finally
    {
#if HALOMCCVR_DEV_DIAGNOSTICS
        if (observing)
            T13Release(g_t13Reach);
#endif
        g_reachCamera.activeCallbacks.fetch_sub(1, std::memory_order_acq_rel);
    }
}

// Worker, inside InstallReachCameraCore (whole-image-hashed module, g_reachCamera.base / generation set).
bool ReachType13Install(uintptr_t base, size_t size, uint32_t generation)
{
    auto& d = g_reachSabre;
    if (g_reachT13Target)
    {
        if (d.base == base)
        {
            d.generation.store(generation, std::memory_order_release);
            d.resetSeq.fetch_add(1, std::memory_order_acq_rel);
            g_reachT13Enabled.store(true, std::memory_order_release);
            return true;
        }
        LOG("Reach Sabre direct drive: the pilot controller's hook is still installed on another module; not "
            "reinstalled");
        return false;
    }
    char failed[96]{};
    size_t used = 0;
    bool anchorsOk = size > kReachEngineTlsIndexRva + sizeof(uint32_t);
    for (const ReachSabreAnchor& a : kReachSabreAnchors)
    {
        const bool ok = ReachColdExecutableAddress(base + a.rva) &&
            (a.unique ? LegacyDriveUniqueAt(base, size, a.rva, a.pattern) : LegacyDriveBytesAt(base, size, a.rva, a.pattern));
        if (ok)
            continue;
        anchorsOk = false;
        const int n = snprintf(failed + used, sizeof(failed) - used, "%s%s", used ? "," : "", a.name);
        if (n > 0 && used + static_cast<size_t>(n) < sizeof(failed))
            used += static_cast<size_t>(n);
    }
    const bool callOk = anchorsOk && AimCallerCallReturn(base, size, kReachSabreCallRva, kReachSabreEntryRva) != 0;
    // The frame through which the dispatcher's own return address is read from the controller's entry rsp: from the
    // dispatcher's unwind info and its prologue, which must agree.
    uint32_t frame = 0;
    const char* frameWhy = nullptr;
    const bool frameOk = callOk &&
        AimCallerProveDispatcher(base, size, kReachSabreDispatcherRva, kReachSabreCallRva, frame, frameWhy) && frame;
    uintptr_t callerRet[3]{};
    for (int i = 0; frameOk && i < 3; ++i)
        callerRet[i] = AimCallerCallReturn(base, size, kReachSabreCallerRvas[i], kReachSabreDispatcherRva);
    if (!anchorsOk || !callOk || !frameOk || !callerRet[1])
    {
        LOG("Reach Sabre direct drive: the pilot's controller +0x%X was NOT hooked (anchors failed: %s; dispatcher "
            "call=%d frame=%d%s%s stick-update caller=%d); a Sabre's pilot keeps the stick servo, nothing else changes",
            kReachSabreEntryRva, failed[0] ? failed : "none", callOk, frameOk, frameWhy ? ": " : "",
            frameWhy ? frameWhy : "", callerRet[1] ? 1 : 0);
        return false;
    }
    void* target = reinterpret_cast<void*>(base + kReachSabreEntryRva);
    const MH_STATUS created = MH_CreateHook(target, reinterpret_cast<void*>(&ReachType13Detour), &g_reachT13Orig);
    if (created != MH_OK)
    {
        g_reachT13Orig = nullptr;
        LOG("Reach Sabre direct drive: the pilot controller's hook FAILED (create=%d); a Sabre's pilot keeps the "
            "stick servo", static_cast<int>(created));
        return false;
    }
    // Everything the hook reads is set before it can run.
    d.base = base;
    d.dispatcherFrame = frame;
    for (int i = 0; i < 3; ++i)
        d.callerRet[i] = callerRet[i];
    d.generation.store(generation, std::memory_order_release);
    d.resetSeq.fetch_add(1, std::memory_order_acq_rel);
    if (MH_EnableHook(target) != MH_OK)
    {
        MH_RemoveHook(target);
        g_reachT13Orig = nullptr;
        LOG("Reach Sabre direct drive: the pilot controller's hook FAILED to enable; removed, a Sabre's pilot keeps "
            "the stick servo");
        return false;
    }
    g_reachT13Target = target;
    g_reachT13Enabled.store(true, std::memory_order_release);
    LOG("Reach Sabre direct drive: the pilot's controller +0x%X is hooked (dispatcher return at entry rsp+0x%X; "
        "callers: extra %d, stick update 1, no unit %d). After the game's own controller the hand's yaw and pitch "
        "are written to the record every other seat is written through (keys: aim_direct_drive, seat_direct_drive, "
        "reach_vehicle_aim 2), once a flight has shown that record to be what the craft steers by; at most %.0f "
        "deg/s; roll, and the game's own turn-back at the play space's edge, stay the game's",
        kReachSabreEntryRva, frame, callerRet[0] ? 1 : 0, callerRet[2] ? 1 : 0,
        static_cast<double>(kReachSabreMaxTurnDegPerSecond));
    return true;
}

// Cold worker, with the direct drive's own report: what the pilot's detour did in the last window.
void ReachSabreDriveReport()
{
    auto& d = g_reachSabre;
    static unsigned long long s_last = 0;
    const unsigned long long now = GetTickCount64();
    if (now - s_last < 2000)
        return;
    s_last = now;
    const uint32_t calls = d.calls.exchange(0, std::memory_order_relaxed);
    const uint32_t ticks = d.ticks.exchange(0, std::memory_order_relaxed);
    const uint32_t stints = d.stints.exchange(0, std::memory_order_relaxed);
    const uint32_t gated = d.gated.exchange(0, std::memory_order_relaxed);
    const uint32_t unread = d.unread.exchange(0, std::memory_order_relaxed);
    const uint32_t waiting = d.waiting.exchange(0, std::memory_order_relaxed);
    const uint32_t misses = d.linkMisses.exchange(0, std::memory_order_relaxed);
    const uint32_t yielded = d.yielded.exchange(0, std::memory_order_relaxed);
    const uint32_t updates = d.updates.exchange(0, std::memory_order_relaxed);
    const uint32_t extras = d.extras.exchange(0, std::memory_order_relaxed);
    const uint32_t noUnit = d.noUnit.exchange(0, std::memory_order_relaxed);
    const uint32_t unknown = d.unknownCaller.exchange(0, std::memory_order_relaxed);
    const uint32_t capped = d.capped.exchange(0, std::memory_order_relaxed);
    const float nose = d.noseMaxDeg.exchange(0.0f, std::memory_order_relaxed);
    const float link = d.linkMaxDeg.exchange(0.0f, std::memory_order_relaxed);
    if (!calls)
        return;
    const uint32_t state = d.state.load(std::memory_order_relaxed);
    LOG("REACHDRIVE Sabre: %u controller calls in %u ticks (new flights %u); the record %s (the craft's facing "
        "stood at most %.2f deg off it, %u ticks out of agreement); updates handed to the drive %u (%u of its "
        "writes held to the turn rate), left to the game's own turn-back %u, waiting for the link %u, keys off %u, "
        "unreadable %u; other callers: extra %u, no unit %u, unknown %u; the craft's nose trailed its facing by at "
        "most %.1f deg",
        calls, ticks, stints,
        state == 2 ? "is PROVEN to be what the craft steers by" :
            state == 1 ? "is not yet proven to be what the craft steers by" : "is not being judged",
        static_cast<double>(link), misses, updates, capped, yielded, waiting, gated, unread, extras, noUnit, unknown,
        static_cast<double>(nose));
}
#endif // HALOMCCVR_EXPERIMENTAL_REACH_RENDER_CANDIDATE
