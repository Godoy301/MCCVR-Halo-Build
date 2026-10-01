// Included in game.cpp at file scope, just before Game_ComputeAimStick.
//
// HALO 3 / ODST / HALO 4 DIRECT DRIVE (legacy_direct_drive = 1). Reach's direct drive (reach_direct_drive
// = 3) for the three sibling engines, which otherwise aim with the right-stick servo: the servo can only
// ask the game to turn at its own turn rate, so the aim lags the hand, and it needs the game to accept
// stick input at all. Here the aim is written where the game keeps it.
//
// Each title has Reach's per-user control update -- the ACCUMULATOR of player_control.desired_angles
// (desired += stick delta, then wrap and clamp) -- found 2026-09-29 by structure in each retail module
// and matched field for field against Reach's 0x6375C:
//
//      title    control update   TLS slot   per-user stride   desired yaw / pitch
//      Reach    0x6375C          +0xD8      0x180             +0x94 / +0x98       (reach_direct_drive)
//      Halo 3   0xF7DF0          +0xC0      0x110             +0xD4 / +0xD8
//      ODST     0x113AB8         +0xD8      0x2C0             +0xD4 / +0xD8
//      Halo 4   0xA8B34          +0xB0      0x1F0             +0x138 / +0x13C
//
// The detour (legacy_player_control_hook.asm) runs the native update, then LegacyPcApplyForUser rewrites
// the pair with the servo's target (the converged aim when trace-and-reaim resolves) on the SIM thread,
// through that thread's own TLS: exactly the record the native just updated.
//
// ADDR-HYGIENE: guarded. Each RVA is a measurement of one build. Three exact-byte anchors must match at
// their RVAs before a hook exists -- the entry prologue (so MinHook relocates known instructions), the
// TLS/stride block (the slot and stride constants above) and the accumulator itself -- and the TLS/stride
// block and the accumulator must each be unique in the module (Halo 4's prologue is shared by two other
// functions, so it is matched at its address only). The engine TLS index is read from the update's own
// `mov e?x, [rip+disp32]`, never assumed. Any mismatch: no hook, said once; the servo keeps aiming.
// ARM GATE (ValueAgreement): installed disarmed; the hook writes only after the record it resolved has
// held the game's own aim (the camera forward the servo closes on) within 1.7 deg for 30 consecutive
// updates, and only while every later update resolves the same record. A different record disarms it.
// ON FOOT, and (seat_direct_drive, Halo 3 / ODST) in a seat that fires the occupant's own weapon: the input
// thread publishes such a seat's setpoint flagged `seated`, and the write goes through Reach's seat-limit
// latch (aim_write::LimitLatch) so a passenger's cone holds it at the edge instead of the aim running past.
// An arm earned on foot does not carry into a seat: every seat stint first passes the same agreement on
// its own record, under the servo, before its first write. Every other seat is never written; it keeps
// the servo.

namespace
{
struct LegacyDriveSite
{
    GameTitle title;
    const char* name;
    uint32_t entryRva;
    const char* prologue;
    bool prologueUnique;
    uint32_t tlsRva;
    const char* tlsPattern;
    uint32_t accRva;
    const char* accPattern;
    uint32_t tlsIndexLoadRva;   // `mov e?x, [rip+disp32]` (8B 0D / 8B 15) loading the engine TLS index
    uint32_t tlsSlot;
    uint32_t stride;
    uint32_t angles;
};

constexpr LegacyDriveSite kLegacyDriveSites[3] = {
    {GameTitle::Halo3, "Halo 3", 0xF7DF0,
     "48 8B C4 48 89 58 08 48 89 70 10 48 89 78 18 F3 0F 11 58 20 55 41 54 41 55 41 56 41 57 48 8D 6C 24 80 "
     "48 81 EC 80 01 00 00 8B 15 ?? ?? ?? ??", true,
     0xF7E65, "65 48 8B 04 25 58 00 00 00 4C 63 E1 49 69 DC 10 01 00 00 48 8B 34 D0 B8 C0 00 00 00 "
              "48 89 74 24 30 48 8B 3C 30",
     0xF8146, "F3 0F 58 B4 3B D4 00 00 00 4C 8B 3D ?? ?? ?? ?? 41 83 C8 FF 4C 8B 35 ?? ?? ?? ?? "
              "B8 38 00 00 00 F3 0F 11 B4 3B D4 00 00 00",
     0xF7E19, 0xC0, 0x110, 0xD4},
    {GameTitle::Halo3ODST, "ODST", 0x113AB8,
     "48 8B C4 48 89 58 08 48 89 70 10 48 89 78 18 F3 0F 11 58 20 55 41 54 41 55 41 56 41 57 48 8D A8 48 FF "
     "FF FF 48 81 EC 90 01 00 00 8B 15 ?? ?? ?? ??", true,
     0x113B2F, "65 48 8B 04 25 58 00 00 00 4C 63 E1 BF D8 00 00 00 49 69 DC C0 02 00 00 4C 8B 2C D0 "
               "48 8B 05 ?? ?? ?? ?? 4A 8B 3C 2F",
     0x113E0A, "F3 0F 58 B4 1F D4 00 00 00 4C 8B 3D ?? ?? ?? ?? 83 CE FF 4C 8B 35 ?? ?? ?? ?? "
               "B8 20 00 00 00 F3 0F 11 B4 1F D4 00 00 00",
     0x113AE3, 0xD8, 0x2C0, 0xD4},
    {GameTitle::Halo4, "Halo 4", 0xA8B34,
     "48 8B C4 48 89 58 10 48 89 70 18 48 89 78 20 55 41 54 41 55 41 56 41 57 48 8D A8 08 FF FF FF "
     "48 81 EC D0 01 00 00", false,
     0xA8BD0, "4C 63 F1 44 0F 28 D9 8B 0D ?? ?? ?? ?? 49 69 DE F0 01 00 00 F3 0F 11 5D 88 4C 8B 3C C8 "
              "48 8B 05 ?? ?? ?? ?? BF B0 00 00 00",
     0xA92A6, "F3 0F 58 84 1F 38 01 00 00 41 0F 28 CD E8 ?? ?? ?? ?? 44 0F 2F D0 F3 0F 11 84 1F 38 01 00 00",
     0xA8BD7, 0xB0, 0x1F0, 0x138},
};

struct LegacyDriveState
{
    void* target = nullptr;
    uintptr_t tlsIndexAddress = 0;
    std::atomic<bool> installed{false};
    std::atomic<uint32_t> callbacks{0};
    // Arm gate: written on the sim thread (the hook), read by the input thread.
    std::atomic<bool> armed{false};
    std::atomic<uintptr_t> expected{0};
    uint32_t agree = 0;                      // sim thread only
    std::atomic<uint64_t> liveMs{0};         // last update that ran armed on the validated record
    // Setpoint and the game's aim, from the input thread (seqlock).
    std::atomic<uint32_t> seq{0};
    std::atomic<float> yaw{0.0f}, pitch{0.0f}, aimYaw{0.0f}, aimPitch{0.0f};
    std::atomic<uint64_t> setMs{0};
    std::atomic<bool> setValid{false};
    std::atomic<bool> setSeated{false};      // the setpoint is a seat's (seat_direct_drive)
    // Seated writes (sim thread): the seat-limit latch, and when a seat write last ran / last held short.
    aim_write::LimitLatch seatLatch{};
    uint32_t seatTicks = 0;                  // consecutive seat writes since the latch last reset (capped)
    uint32_t seatAgree = 0;                  // this seat stint: consecutive updates its record held the aim
    bool seatVerified = false;               // this seat stint passed its own agreement
    std::atomic<uint64_t> seatWriteMs{0}, seatLimitedMs{0};
    // Status (sim thread counts, cold worker logs).
    std::atomic<uint32_t> calls{0}, writes{0}, arms{0}, disarms{0}, seatedSkips{0}, faults{0};
    std::atomic<uint32_t> seatWrites{0}, seatHeld{0}, seatVerifies{0}, seatUnverified{0};
    std::atomic<float> nativeYaw{0.0f}, nativePitch{0.0f};
};
LegacyDriveState g_legacyDrive[3];

int LegacyDriveSlot(GameTitle title)
{
    return title == GameTitle::Halo3 ? 0 : title == GameTitle::Halo3ODST ? 1 : title == GameTitle::Halo4 ? 2 : -1;
}

float LegacyDriveAngleDiff(float a, float b)
{
    return std::remainder(a - b, 6.28318531f);
}

// ValueAgreement: the record holds the game's own aim within 1.7 deg for 30 consecutive updates.
constexpr float kLegacyDriveAgreeRad = 0.03f;
constexpr uint32_t kLegacyDriveAgreeUpdates = 30;
// The first writes of a seat stint read no limit off the record. Until the input thread sees the seat
// written (LegacyDriveSeatLive) the servo may still be deflecting the stick, and the game's next update
// moves our first write by that deflection, which the latch would take for a seat limit.
constexpr uint32_t kLegacyDriveSeatPrimeUpdates = 2;

bool LegacyDriveAgrees(bool fresh, float nativeYaw, float nativePitch, float aimYaw, float aimPitch)
{
    return fresh && std::isfinite(nativeYaw) && std::isfinite(nativePitch) && std::isfinite(aimYaw) &&
           std::isfinite(aimPitch) && std::fabs(LegacyDriveAngleDiff(nativeYaw, aimYaw)) < kLegacyDriveAgreeRad &&
           std::fabs(nativePitch - aimPitch) < kLegacyDriveAgreeRad;
}

// Sim thread: the seat stint is over (left the seat, the seat stopped qualifying, a disarm or a fault). The
// next seat proves its record again.
void LegacyDriveForgetSeat(LegacyDriveState& st)
{
    st.seatAgree = 0;
    st.seatVerified = false;
    // The input thread reads the seat as written while these are fresh (LegacyDriveSeatLive): a stint that
    // is over stops claiming it at once, or the stick stays zeroed and the reticle reads "written" for up to
    // 150 ms over an aim nothing writes any more.
    st.seatWriteMs.store(0, std::memory_order_release);
    st.seatLimitedMs.store(0, std::memory_order_release);
}

// The pattern matches at exactly base + rva (not merely somewhere).
bool LegacyDriveBytesAt(uintptr_t base, size_t size, uint32_t rva, const char* pattern)
{
    size_t bytes = 0;
    for (const char* p = pattern; *p; ++p)
        if (*p != ' ' && (p == pattern || p[-1] == ' '))
            ++bytes;
    if (!bytes || rva + bytes > size)
        return false;
    return sig::Find(base + rva, bytes, pattern) == base + rva;
}

bool LegacyDriveUniqueAt(uintptr_t base, size_t size, uint32_t rva, const char* pattern)
{
    const uintptr_t first = sig::Find(base, size, pattern);
    if (first != base + rva)
        return false;
    const uintptr_t next = first + 1;
    return next >= base + size || !sig::Find(next, static_cast<size_t>(base + size - next), pattern);
}

// Whether the local player's unit is on foot, from the title's own state (sim thread). Unknown = not.
bool LegacyDriveLocalOnFoot(int slot)
{
    __try
    {
        if (slot == 0)
        {
            if (!g_halo3PlayerUnitGetter || !g_halo3UnitInVehicle)
                return false;
            const int32_t unit = g_halo3PlayerUnitGetter(0);
            return unit != -1 && !g_halo3UnitInVehicle(unit);
        }
        if (slot == 1)
        {
            if (!g_odstPlayerUnitGetter || !g_odstUnitInVehicle)
                return false;
            const int32_t unit = g_odstPlayerUnitGetter(0);
            return unit != -1 && !g_odstUnitInVehicle(unit);
        }
#if HALOMCCVR_EXPERIMENTAL_HALO4_CAMERA
        if (slot == 2)
        {
            const uint64_t at = g_halo4SightUnitAtMs.load(std::memory_order_acquire);
            const uint64_t now = GetTickCount64();
            if (!at || now < at || now - at > 250)
                return false;
            const auto* biped = Halo4ContactBiped(static_cast<uint32_t>(g_halo4SightUnit.load(std::memory_order_relaxed)));
            return biped && *reinterpret_cast<const int32_t*>(biped + 0x24) == -1;
        }
#endif
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
    }
    return false;
}
} // namespace

bool LegacyDriveInstall(GameTitle title, uintptr_t base, size_t size)
{
    const int slot = LegacyDriveSlot(title);
    // Installed whatever the setting, so F1's Direct drive switches it live: the hook writes only while
    // legacy_direct_drive is on, and disarms (to re-validate) whenever it is off.
    if (slot < 0)
        return false;
    auto& st = g_legacyDrive[slot];
    const auto& site = kLegacyDriveSites[slot];
    if (st.target)
        return true;
    const bool entryOk = site.prologueUnique
        ? LegacyDriveUniqueAt(base, size, site.entryRva, site.prologue)
        : LegacyDriveBytesAt(base, size, site.entryRva, site.prologue);
    const bool tlsOk = LegacyDriveUniqueAt(base, size, site.tlsRva, site.tlsPattern);
    const bool accOk = LegacyDriveUniqueAt(base, size, site.accRva, site.accPattern);
    const auto* load = reinterpret_cast<const uint8_t*>(base + site.tlsIndexLoadRva);
    const bool loadOk = site.tlsIndexLoadRva + 6 <= size && load[0] == 0x8B && (load[1] == 0x0D || load[1] == 0x15);
    if (!entryOk || !tlsOk || !accOk || !loadOk)
    {
        LOG("DRIVE %s: control update NOT hooked (anchors entry=%d tls=%d accumulator=%d index-load=%d at "
            "+0x%X); the stick servo keeps aiming", site.name, entryOk, tlsOk, accOk, loadOk, site.entryRva);
        return false;
    }
    int32_t disp = 0;
    memcpy(&disp, load + 2, sizeof(disp));
    const uintptr_t indexAddress = base + site.tlsIndexLoadRva + 6 + static_cast<intptr_t>(disp);
    if (indexAddress < base || indexAddress + sizeof(uint32_t) > base + size)
    {
        LOG("DRIVE %s: the engine TLS index is outside the module; not hooked", site.name);
        return false;
    }
    void* detour = slot == 0 ? reinterpret_cast<void*>(&Halo3PcHook)
                 : slot == 1 ? reinterpret_cast<void*>(&OdstPcHook)
                             : reinterpret_cast<void*>(&Halo4PcHook);
    void** orig = slot == 0 ? &g_halo3PcOrig : slot == 1 ? &g_odstPcOrig : &g_halo4PcOrig;
    void* target = reinterpret_cast<void*>(base + site.entryRva);
    st.armed.store(false, std::memory_order_release);
    st.expected.store(0, std::memory_order_release);
    st.agree = 0;
    st.liveMs.store(0, std::memory_order_release);
    st.setValid.store(false, std::memory_order_release);
    st.seatLatch.reset();
    st.seatTicks = 0;
    LegacyDriveForgetSeat(st);
    st.seatWriteMs.store(0, std::memory_order_release);
    st.seatLimitedMs.store(0, std::memory_order_release);
    st.tlsIndexAddress = indexAddress;
    const MH_STATUS created = MH_CreateHook(target, detour, orig);
    if (created != MH_OK || MH_EnableHook(target) != MH_OK)
    {
        if (created == MH_OK)
            MH_RemoveHook(target);
        *orig = nullptr;
        LOG("DRIVE %s: control-update hook FAILED (create=%d); the stick servo keeps aiming", site.name, (int)created);
        return false;
    }
    st.target = target;
    st.installed.store(true, std::memory_order_release);
    LOG("DRIVE %s: control-update hook INSTALLED at +0x%X (TLS index +0x%llX, player_control TLS+0x%X, "
        "stride 0x%X, desired angles +0x%X); disarmed until the record agrees with the game's aim",
        site.name, site.entryRva, static_cast<unsigned long long>(indexAddress - base), site.tlsSlot, site.stride,
        site.angles);
    return true;
}

bool LegacyDriveRemove(GameTitle title)
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0)
        return true;
    auto& st = g_legacyDrive[slot];
    st.installed.store(false, std::memory_order_release);
    st.armed.store(false, std::memory_order_release);
    if (!st.target)
        return true;
    const MH_STATUS disabled = MCCVR_DisableHookForRetirement(st.target);
    if (disabled != MH_OK && disabled != MH_ERROR_DISABLED && disabled != MH_ERROR_NOT_CREATED)
        return false;
    void* detour = slot == 0 ? reinterpret_cast<void*>(&Halo3PcHook)
                 : slot == 1 ? reinterpret_cast<void*>(&OdstPcHook)
                             : reinterpret_cast<void*>(&Halo4PcHook);
    void** orig = slot == 0 ? &g_halo3PcOrig : slot == 1 ? &g_odstPcOrig : &g_halo4PcOrig;
    const void* functions[]{detour, reinterpret_cast<const void*>(&LegacyPcApplyForUser)};
    const void* trampolines[]{*orig, nullptr};
    if (!WaitForNativeDetourQuiescence(functions, trampolines, 2, st.callbacks))
        return false;
    const MH_STATUS removed = MH_RemoveHook(st.target);
    if (removed != MH_OK && removed != MH_ERROR_NOT_CREATED)
        return false;
    st.target = nullptr;
    *orig = nullptr;
    return true;
}

// Input thread: the servo's target (the converged aim when it resolves) and the game's own aim. `seated`:
// the setpoint belongs to a seat the hook may write (seat_direct_drive); on foot it is false.
void LegacyDrivePublish(GameTitle title, float yaw, float pitch, float aimYaw, float aimPitch, bool valid,
                        bool seated)
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0)
        return;
    auto& st = g_legacyDrive[slot];
    st.seq.fetch_add(1, std::memory_order_acq_rel);
    st.yaw.store(yaw, std::memory_order_relaxed);
    st.pitch.store(pitch, std::memory_order_relaxed);
    st.aimYaw.store(aimYaw, std::memory_order_relaxed);
    st.aimPitch.store(aimPitch, std::memory_order_relaxed);
    st.setMs.store(GetTickCount64(), std::memory_order_relaxed);
    st.setValid.store(valid && std::isfinite(yaw) && std::isfinite(pitch), std::memory_order_relaxed);
    st.setSeated.store(seated, std::memory_order_relaxed);
    st.seq.fetch_add(1, std::memory_order_release);
}

// Input thread: the hook is armed on a validated record and ran within 150 ms -- the servo stands down.
bool LegacyDriveLive(GameTitle title)
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0)
        return false;
    const auto& st = g_legacyDrive[slot];
    const uint64_t live = st.liveMs.load(std::memory_order_acquire);
    const uint64_t now = GetTickCount64();
    return st.armed.load(std::memory_order_acquire) && live && now >= live && now - live <= 150;
}

// Input thread: the hook wrote a SEAT's setpoint within 150 ms (seat_direct_drive) -- the seat's servo stands
// down -- and whether that write was held short at a seat limit (then the aim is not the converged one).
bool LegacyDriveSeatLive(GameTitle title)
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0)
        return false;
    const auto& st = g_legacyDrive[slot];
    const uint64_t at = st.seatWriteMs.load(std::memory_order_acquire);
    const uint64_t now = GetTickCount64();
    return st.armed.load(std::memory_order_acquire) && at && now >= at && now - at <= 150;
}

bool LegacyDriveSeatLimited(GameTitle title)
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0)
        return false;
    const uint64_t at = g_legacyDrive[slot].seatLimitedMs.load(std::memory_order_acquire);
    const uint64_t now = GetTickCount64();
    return at && now >= at && now - at <= 150;
}

extern "C" void LegacyPcApplyForUser(int slot, int user)
{
    if (slot < 0 || slot > 2 || user != 0)
        return;
    auto& st = g_legacyDrive[slot];
    const auto& site = kLegacyDriveSites[slot];
    st.callbacks.fetch_add(1, std::memory_order_acq_rel);
    st.calls.fetch_add(1, std::memory_order_relaxed);
    bool seatWrote = false;   // this update ended in a seat write (the latch keeps its last value only then)
    bool seatKept = false;    // this update continued a seat stint (its agreement count survives)
    __try
    {
        do
        {
            if (!g_config.legacy_direct_drive)
            {
                // Switched off live: stop now, and make switching back on re-validate the record first.
                if (st.armed.exchange(false, std::memory_order_acq_rel))
                    st.disarms.fetch_add(1, std::memory_order_relaxed);
                st.agree = 0;
                break;
            }
            if (!st.installed.load(std::memory_order_acquire) || !st.tlsIndexAddress)
                break;
            // This (sim) thread's TLS -> the record the native update just wrote.
            auto** slots = reinterpret_cast<void**>(__readgsqword(0x58));
            const uint32_t index = *reinterpret_cast<const uint32_t*>(st.tlsIndexAddress);
            auto* tls = (slots && index < 0x200) ? reinterpret_cast<uint8_t*>(slots[index]) : nullptr;
            const uintptr_t globals = tls ? *reinterpret_cast<const uintptr_t*>(tls + site.tlsSlot) : 0;
            const uintptr_t pc = globals ? globals + static_cast<uintptr_t>(user) * site.stride : 0;
            if (!pc)
                break;
            float* pair = reinterpret_cast<float*>(pc + site.angles);
            const float nativeYaw = pair[0], nativePitch = pair[1];
            st.nativeYaw.store(nativeYaw, std::memory_order_relaxed);
            st.nativePitch.store(nativePitch, std::memory_order_relaxed);
            float yaw = 0.0f, pitch = 0.0f, aimYaw = 0.0f, aimPitch = 0.0f;
            uint64_t setMs = 0;
            bool valid = false, seatedSet = false, coherent = false;
            for (int attempt = 0; attempt < 2 && !coherent; ++attempt)
            {
                const uint32_t before = st.seq.load(std::memory_order_acquire);
                if (before & 1u)
                    continue;
                yaw = st.yaw.load(std::memory_order_relaxed);
                pitch = st.pitch.load(std::memory_order_relaxed);
                aimYaw = st.aimYaw.load(std::memory_order_relaxed);
                aimPitch = st.aimPitch.load(std::memory_order_relaxed);
                setMs = st.setMs.load(std::memory_order_relaxed);
                valid = st.setValid.load(std::memory_order_relaxed);
                seatedSet = st.setSeated.load(std::memory_order_relaxed);
                std::atomic_thread_fence(std::memory_order_acquire);   // the loads above stay before the re-read
                coherent = st.seq.load(std::memory_order_relaxed) == before;
            }
            const uint64_t now = GetTickCount64();
            const bool fresh = coherent && valid && setMs && now >= setMs && now - setMs <= 200;
            // ValueAgreement (the arm gate, and each seat stint's own check below): the record holds the
            // game's own aim.
            const bool agrees = LegacyDriveAgrees(fresh, nativeYaw, nativePitch, aimYaw, aimPitch);
            // Seated: only a seat the input thread flagged (seat_direct_drive, a seat that fires the occupant's
            // own weapon; Halo 3 / ODST) is written, through the seat-limit latch below. Any other seat keeps
            // the servo, exactly as before.
            const bool onFoot = LegacyDriveLocalOnFoot(slot);
            const bool seatWrite = !onFoot && slot != 2 && g_config.seat_direct_drive && fresh && seatedSet;
            seatKept = seatWrite;
            if (!onFoot && !seatWrite)
            {
                st.seatedSkips.fetch_add(1, std::memory_order_relaxed);
                st.agree = 0;
                break;
            }
            // SEATED VALUE AGREEMENT. A seat's record need not hold the aim the shot follows the way an
            // on-foot record does (a driver seat measured 1.8 deg between them), and an arm earned on foot says
            // nothing about it. So each seat stint proves its own, counted here whether or not the hook is
            // armed yet: the record holds the game's aim, under the servo, for the arm gate's 30 updates before
            // the first seat write. A seat that never agrees is never written; it keeps the servo, and the
            // report counts the updates it waited.
            if (seatWrite && !st.seatVerified)
            {
                st.seatAgree = agrees ? st.seatAgree + 1 : 0;
                if (st.seatAgree >= kLegacyDriveAgreeUpdates)
                {
                    st.seatVerified = true;
                    st.seatVerifies.fetch_add(1, std::memory_order_relaxed);
                }
            }
            if (!st.armed.load(std::memory_order_acquire))
            {
                // ValueAgreement: the record must hold the game's own aim before it is ever written.
                st.agree = agrees ? st.agree + 1 : 0;
                if (st.agree >= kLegacyDriveAgreeUpdates)
                {
                    st.expected.store(pc, std::memory_order_release);
                    st.armed.store(true, std::memory_order_release);
                    st.arms.fetch_add(1, std::memory_order_relaxed);
                }
                break;
            }
            if (pc != st.expected.load(std::memory_order_acquire))
            {
                st.armed.store(false, std::memory_order_release);
                st.agree = 0;
                seatKept = false;
                st.disarms.fetch_add(1, std::memory_order_relaxed);
                break;
            }
            st.liveMs.store(now, std::memory_order_release);
            if (seatWrite && !st.seatVerified)
            {
                st.seatUnverified.fetch_add(1, std::memory_order_relaxed);
                break;
            }
            if (fresh && std::isfinite(yaw) && std::isfinite(pitch))
            {
                // Stay on the branch the game's yaw is on (never a 2*pi jump) and inside its pitch clamp.
                float y = aim_write::unwrap_to_branch(yaw, nativeYaw);
                float p = aim_write::clamp_pitch(pitch, 1.45f);
                bool limited = false;
                if (seatWrite)
                {
                    // The game clamped the record to the seat's limits before this write; the latch reads
                    // that clamp off last tick's write and keeps this one at the limit (one probe step past
                    // it), so a passenger's cone holds the aim at its edge instead of the write running on.
                    // The stint's first writes read no limit (kLegacyDriveSeatPrimeUpdates).
                    if (st.seatTicks < kLegacyDriveSeatPrimeUpdates)
                        st.seatLatch.reset();
                    const float native[2]{nativeYaw, nativePitch}, target[2]{y, p};
                    float held[2]{};
                    limited = st.seatLatch.apply(native, target, held);
                    y = aim_write::unwrap_to_branch(held[0], nativeYaw);
                    p = aim_write::clamp_pitch(held[1], 1.45f);
                }
                if (std::isfinite(y) && std::isfinite(p))
                {
                    pair[0] = y;
                    pair[1] = p;
                    if (seatWrite)
                    {
                        const float written[2]{y, p};
                        st.seatLatch.wrote(written);
                        seatWrote = true;
                        if (st.seatTicks < kLegacyDriveSeatPrimeUpdates)
                            ++st.seatTicks;
                        st.seatWriteMs.store(now, std::memory_order_release);
                        st.seatLimitedMs.store(limited ? now : 0, std::memory_order_release);
                        st.seatWrites.fetch_add(1, std::memory_order_relaxed);
                        if (limited)
                            st.seatHeld.fetch_add(1, std::memory_order_relaxed);
                    }
                    st.writes.fetch_add(1, std::memory_order_relaxed);
                }
            }
        } while (false);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        st.armed.store(false, std::memory_order_release);
        seatWrote = false;
        seatKept = false;
        st.faults.fetch_add(1, std::memory_order_relaxed);
    }
    // The latch's contract is "nothing written last tick: forget everything". Every update that did not end
    // in a seat write -- on foot, a skipped or unverified seat, disarmed, switched off, a fault -- resets it
    // here, so a later seat write never reads a limit off a value this hook did not write.
    if (!seatWrote)
    {
        st.seatLatch.reset();
        st.seatTicks = 0;
    }
    if (!seatKept)
        LegacyDriveForgetSeat(st);
    st.callbacks.fetch_sub(1, std::memory_order_acq_rel);
}

// Cold worker, every 2 s, per title with a hook that ran.
void LegacyDriveReport()
{
    static uint64_t s_last = 0;
    const uint64_t now = GetTickCount64();
    if (now - s_last < 2000)
        return;
    s_last = now;
    for (int slot = 0; slot < 3; ++slot)
    {
        auto& st = g_legacyDrive[slot];
        const uint32_t calls = st.calls.exchange(0, std::memory_order_relaxed);
        if (!calls)
            continue;
        LOG("DRIVE %s: armed=%d updates %u, wrote %u; arms %u, disarms %u, seated %u, faults %u; seat writes %u "
            "(held at a seat limit %u), seat checks passed %u, seat updates held for the check %u; record yaw "
            "%.3f pitch %.3f vs game aim yaw %.3f pitch %.3f",
            kLegacyDriveSites[slot].name, st.armed.load() ? 1 : 0, calls, st.writes.exchange(0), st.arms.exchange(0),
            st.disarms.exchange(0), st.seatedSkips.exchange(0), st.faults.exchange(0), st.seatWrites.exchange(0),
            st.seatHeld.exchange(0), st.seatVerifies.exchange(0), st.seatUnverified.exchange(0),
            st.nativeYaw.load(), st.nativePitch.load(), st.aimYaw.load(), st.aimPitch.load());
    }
}
