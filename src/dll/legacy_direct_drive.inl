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
//
// SEATS (seat_direct_drive). Every seat of these titles keeps its aim in this same record, updated by this
// same accumulator from the same call site as on foot, with the seat's limits applied inside it (mapped
// 2026-10-01; the one exception, Halo 4's type-13 driver, runs a different controller and never reaches
// the hook, so it stays on the servo). The input thread publishes a seat's setpoint flagged `seated`, with
// the AXES the hand owns there (seat_drive::Decide: a passenger, gunner or turret both; a look-steered
// driver both at reach_vehicle_aim 2, the pitch only while the stick or wheel steers, none at 1; Halo 4 seats
// the heading only (game.cpp); nothing in a seat whose view is not ours). Each seat stint then proves its own
// record axis by axis before that axis is written (seat_drive::StintCheck: yaw on 30 agreeing updates, pitch
// on 30 SETTLED ones within 3 s of yaw passing, never before yaw -- a Halo 3 driver+gunner's pitch
// sits ~13 deg off the gun and stays on the servo). The write goes through Reach's seat-limit latch
// (aim_write::LimitLatch), pitch excepted for a LOOK-STEERED driver, whose pitch the accumulator levels as the vehicle
// moves. A seat may also ARM the hook, on yaw agreement alone: a checkpoint that loads in a vehicle.
//
// THE STICK CALL IS THE UPDATE (aim_caller_logic.h). The accumulator does not only run for the stick. Halo 3
// and ODST call it directly from three sites -- the stick update, the no-unit path and a weapon's pitch kick
// -- so this detour's own return address names the caller. Halo 4 calls it from ONE site inside the
// dispatcher 0xA8828, which one control pass calls up to four times: [an effect camera impulse 0xA349E] ->
// [the weapon's aim sway 0xA7751] -> the stick update 0xA7774, or [impulse] -> the no-unit path 0xA7A01.
// Every call returns to the same place, so there the DISPATCHER's return address names the caller, read at
// the accumulator's entry rsp + 0x100 (the dispatcher's fixed 0xF8-byte frame, proven at install from its
// unwind info and its prologue). Only the stick call counts updates, checks seats, steps the arm gate and
// feeds the latch. An extra call (impulse, sway, kick, or any caller the map does not know, once the stick is
// proven) has the last stick write put back over what it added; Halo 4's no-unit call ends the update
// (nothing was written: the latch and the seat stint are over). Halo 3's and ODST's every non-stick call keeps
// the put-back it always had -- this frame's write, the same record, no context or plausibility test -- and
// one thing changed, the window: a KICK (it runs in this frame's ticks, after the stick call) is put back while
// no stick call has run since, under the 250 ms QPC cap, where it was 50 ms of GetTickCount64 (a catch-up burst
// of ticks after a hitch outlasted that and left the kick for the seat latch); the NO-UNIT call and any caller
// the map does not know run with no stick call at all while the player is dead, so they keep the 50 ms window,
// now on QPC (kLegacyDrivePutBackNoStickMs). An unproven stick site leaves every call counting as the update,
// as before.
// "Updates" are therefore stick calls, and the control pass runs once per MAIN-LOOP ITERATION -- a rendered
// frame -- ahead of that frame's 0..N sim ticks: ~120 per second at 120 fps, not the sim's 60 Hz.

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
    // The `call entry` (E8 rel32) of the stick update (Halo 3, ODST), or of the dispatcher that every Halo 4
    // caller goes through (its one call site). ADDR-HYGIENE: guarded (the rel32 must land on entryRva, and it
    // is the first rung of the caller map below; otherwise every caller counts, as before).
    uint32_t primaryCallRva;
};

constexpr LegacyDriveSite kLegacyDriveSites[3] = {
    {GameTitle::Halo3, "Halo 3", 0xF7DF0,
     "48 8B C4 48 89 58 08 48 89 70 10 48 89 78 18 F3 0F 11 58 20 55 41 54 41 55 41 56 41 57 48 8D 6C 24 80 "
     "48 81 EC 80 01 00 00 8B 15 ?? ?? ?? ??", true,
     0xF7E65, "65 48 8B 04 25 58 00 00 00 4C 63 E1 49 69 DC 10 01 00 00 48 8B 34 D0 B8 C0 00 00 00 "
              "48 89 74 24 30 48 8B 3C 30",
     0xF8146, "F3 0F 58 B4 3B D4 00 00 00 4C 8B 3D ?? ?? ?? ?? 41 83 C8 FF 4C 8B 35 ?? ?? ?? ?? "
              "B8 38 00 00 00 F3 0F 11 B4 3B D4 00 00 00",
     0xF7E19, 0xC0, 0x110, 0xD4, 0xF6D11},
    {GameTitle::Halo3ODST, "ODST", 0x113AB8,
     "48 8B C4 48 89 58 08 48 89 70 10 48 89 78 18 F3 0F 11 58 20 55 41 54 41 55 41 56 41 57 48 8D A8 48 FF "
     "FF FF 48 81 EC 90 01 00 00 8B 15 ?? ?? ?? ??", true,
     0x113B2F, "65 48 8B 04 25 58 00 00 00 4C 63 E1 BF D8 00 00 00 49 69 DC C0 02 00 00 4C 8B 2C D0 "
               "48 8B 05 ?? ?? ?? ?? 4A 8B 3C 2F",
     0x113E0A, "F3 0F 58 B4 1F D4 00 00 00 4C 8B 3D ?? ?? ?? ?? 83 CE FF 4C 8B 35 ?? ?? ?? ?? "
               "B8 20 00 00 00 F3 0F 11 B4 1F D4 00 00 00",
     0x113AE3, 0xD8, 0x2C0, 0xD4, 0x112890},
    {GameTitle::Halo4, "Halo 4", 0xA8B34,
     "48 8B C4 48 89 58 10 48 89 70 18 48 89 78 20 55 41 54 41 55 41 56 41 57 48 8D A8 08 FF FF FF "
     "48 81 EC D0 01 00 00", false,
     0xA8BD0, "4C 63 F1 44 0F 28 D9 8B 0D ?? ?? ?? ?? 49 69 DE F0 01 00 00 F3 0F 11 5D 88 4C 8B 3C C8 "
              "48 8B 05 ?? ?? ?? ?? BF B0 00 00 00",
     0xA92A6, "F3 0F 58 84 1F 38 01 00 00 41 0F 28 CD E8 ?? ?? ?? ?? 44 0F 2F D0 F3 0F 11 84 1F 38 01 00 00",
     0xA8BD7, 0xB0, 0x1F0, 0x138, 0xA8988},
};

// THE CALLER MAP (aim_caller_logic.h). Each site is an exact-byte pattern that must be UNIQUE in the module
// and match at its RVA, and whose E8 at callOff must land on the accumulator (direct mode: Halo 3, ODST) or
// on the dispatcher (Halo 4); the caller's return address is patRva + callOff + 5. Halo 4's dispatcher mode
// also needs the accumulator's one call site (accCallPat: its E8 is primaryCallRva and lands on the entry),
// the dispatcher's prologue (unique) and its frame, proven from its .pdata/unwind info and its prologue
// (AimCallerProveDispatcher). ADDR-HYGIENE: guarded. The PROOF LADDER: the stick site (Halo 3 / ODST: the
// primaryCallRva check above, as before) is rung 1, and without it every call counts as the update; each
// other site is its own proof, and one that fails is simply absent (its calls read "unknown", put back).
// Measured 2026-10-01 on the retail Steam modules; the Store modules are byte-identical (MCC-EDITIONS-EVIDENCE).
struct LegacyCallerSite
{
    uint32_t patRva;
    const char* pat;
    uint8_t callOff;
    aim_caller::Caller role;
};

struct LegacyCallerAnchors
{
    uint32_t accCallPatRva;          // dispatcher mode only (0: direct mode)
    const char* accCallPat;
    uint8_t accCallOff;
    uint32_t dispatcherRva;
    const char* dispatcherPrologue;
    LegacyCallerSite sites[4];
    aim_caller::Action noUnitAction;
};

constexpr LegacyCallerAnchors kLegacyCallerAnchors[3] = {
    // Halo 3: the stick update in the dispatcher 0xF66AC; the no-unit path (exclusive with it, 0xF6938);
    // a weapon's pitch kick from 0x366858 during the tick. Only the census and the counters read the last two:
    // every caller that is not the stick is put back, exactly as before.
    {0, nullptr, 0, 0, nullptr,
     {{0xF6D00, "F3 0F 10 54 24 5C 0F 28 DE F3 0F 10 4C 24 58 8B CB E8 ?? ?? ?? ??", 0x11, aim_caller::Caller::Stick},
      {0xF6F59, "F3 0F 10 54 24 5C 0F 28 DE F3 0F 10 4C 24 58 41 8B CF E8 ?? ?? ?? ??", 0x12,
       aim_caller::Caller::NoUnit},
      {0x3675A4, "41 0F 28 C8 F3 0F 5C C6 8B CE F3 0F 59 D0 F3 0F 58 D6 E8 ?? ?? ?? ??", 0x12, aim_caller::Caller::Kick},
      {}},
     aim_caller::Action::RestoreOnly},
    // ODST: the same shape (stick update in 0x112184, no-unit path, the weapon's pitch kick).
    {0, nullptr, 0, 0, nullptr,
     {{0x112881, "F3 0F 10 55 8B 0F 28 DE F3 0F 10 4D 87 8B CB E8 ?? ?? ?? ?? 41 83 CA FF", 0x0F,
       aim_caller::Caller::Stick},
      {0x112B12, "8B 7D 67 0F 28 DE F3 0F 10 55 8B 8B CF F3 0F 10 4D 87 E8 ?? ?? ?? ??", 0x12,
       aim_caller::Caller::NoUnit},
      {0x3ADA90, "41 0F 28 C8 F3 0F 5C C6 8B CF F3 0F 59 D0 F3 0F 58 D6 E8 ?? ?? ?? ??", 0x12, aim_caller::Caller::Kick},
      {}},
     aim_caller::Action::RestoreOnly},
    // Halo 4: the accumulator's one call 0xA8988 in the dispatcher 0xA8828 (frame 0xF8: 5 pushes + 0xD0, no
    // frame register), and the dispatcher's four callers, all in the per-user pass 0xA6BA8 / 0xA2670:
    //   0xA7774 the stick update (the real input &[rbp-0x20]);
    //   0xA349E an effect camera impulse (INFERRED from 0x1456EC's per-player effect slots: damage, explosions);
    //   0xA7751 the weapon's aim SWAY -- not a recoil kick: 0xA5B90 advances two phases from the weapon tag's
    //           per-zoom table (+0x408/+0x40C) and returns sin(phase) x amplitude, every pass it is non-zero;
    //   0xA7A01 the no-unit path (exclusive with the stick, `cmp [pc],-1` at 0xA6FF4).
    {0xA897C, "0F 28 DE 41 0F 28 D0 0F 28 CF 8B CE E8 ?? ?? ?? ?? F2 0F 10 43 18 0F 57 ED", 0x0C, 0xA8828,
     "48 8B C4 48 89 58 20 55 56 41 54 41 56 41 57 48 8D 68 B1 48 81 EC D0 00 00 00 0F 29 70 C8 0F 29 78 B8 "
     "44 0F 29 40 A8",
     {{0xA7756, "F3 0F 10 55 F8 4C 8D 4D E0 F3 0F 10 4D F4 8B 4C 24 60 F3 0F 11 7C 24 28 F3 0F 11 74 24 20 "
                "E8 ?? ?? ?? ??", 0x1E, aim_caller::Caller::Stick},
      {0xA348D, "4C 8D 0D ?? ?? ?? ?? F3 0F 10 95 94 00 00 00 8B CE E8 ?? ?? ?? ??", 0x11, aim_caller::Caller::Impulse},
      {0xA7733, "0F 57 0D ?? ?? ?? ?? 4C 8D 0D ?? ?? ?? ?? 8B 4C 24 60 F3 0F 11 7C 24 28 F3 0F 11 74 24 20 "
                "E8 ?? ?? ?? ??", 0x1E, aim_caller::Caller::Sway},
      {0xA79E5, "F3 0F 10 55 F8 4C 8D 4D E0 F3 0F 10 4D F4 8B CB F3 0F 11 7C 24 28 F3 0F 11 74 24 20 "
                "E8 ?? ?? ?? ??", 0x1C, aim_caller::Caller::NoUnit}},
     aim_caller::Action::EndUpdate},
};

#if HALOMCCVR_DEV_DIAGNOSTICS
// Dev census: each title's sim tick counter (AimCallerTickProof). Halo 3 / ODST [[TLS+0xC8]]+0xC, Halo 4
// [[TLS+0x90]]+0xC: the getter, then the increment inside the per-tick function.
constexpr AimCallerTickAnchors kLegacyTickAnchors[3] = {
    {0xF33B0, "8B 0D ?? ?? ?? ?? 65 48 8B 04 25 58 00 00 00 BA C8 00 00 00 48 8B 04 C8 48 8B 0C 10 8B 41 0C C3",
     0xEF60B, "48 8D 54 24 38 48 8D 4C 24 50 E8 ?? ?? ?? ?? 44 84 64 24 54 74 08 48 8B 14 3E 44 01 62 0C", 0xFF, 29},
    {0x10EB40, "8B 0D ?? ?? ?? ?? 65 48 8B 04 25 58 00 00 00 BA C8 00 00 00 48 8B 04 C8 48 8B 0C 10 8B 41 0C C3",
     0x10A34A, "48 8D 54 24 38 48 8D 4C 24 50 E8 ?? ?? ?? ?? 8A 44 24 54 24 01 74 07 49 8B 14 1E FF 42 0C", 0xFF, 29},
    {0xA0438, "8B 0D ?? ?? ?? ?? 65 48 8B 04 25 58 00 00 00 BA 90 00 00 00 48 8B 04 C8 48 8B 0C 10 8B 41 0C C3",
     0x9ADB9, "44 84 7D 44 74 0D B8 90 00 00 00 48 8B 0C 38 44 01 79 0C", 7, 18},
};
#endif

// Halo 4: the record's own unit, player_control +0x120 (0xA8C15 `mov ecx, [rdi+rbx+0x120]`, rdi = TLS+0xB0,
// rbx = user * 0x1F0), from which the hook tells on foot from seated on the sim thread itself.
// ADDR-HYGIENE: guarded (the exact load must sit at its RVA, or Halo 4 seats are never written).
constexpr uint32_t kHalo4RecordUnitOffset = 0x120;
constexpr uint32_t kHalo4RecordUnitLoadRva = 0xA8C15;
constexpr const char* kHalo4RecordUnitLoad = "8B 8C 1F 20 01 00 00";

struct LegacyDriveState
{
    void* target = nullptr;
    uintptr_t base = 0;
    uintptr_t tlsIndexAddress = 0;
    // Which caller ran the accumulator (aim_caller_logic.h): written at install before the hook is enabled,
    // rewritten only after LegacyDriveRemove's quiescence. Not proven = every caller counts, as before.
    aim_caller::CallerMap callers{};
    std::atomic<uint8_t> callerMode{0};      // for the cold worker: 0 not proven, 1 by return, 2 by dispatcher
    bool halo4RecordUnit = false;           // Halo 4: the record's unit load is proven (seats may be written)
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
    std::atomic<uint32_t> setAxes{0};        // seated: bit 0 yaw, bit 1 pitch -- the axes the hand owns there
    std::atomic<bool> setPitchLatch{true};   // seated: the latch may read pitch limits (not a look-steered driver's)
    std::atomic<uint32_t> setSeatKey{0};     // seated: which seat (seat_drive::SeatKey)
    // Seated writes (sim thread): the seat-limit latch, this stint's per-axis check, and when each axis was
    // last written / last held short (read by the input thread).
    aim_write::LimitLatch seatLatch{};
    uint32_t axisTicks[2]{};                 // per axis: consecutive seat writes of it (capped at the prime)
    seat_drive::StintCheck stint{};
    bool stintActive = false;
    uint32_t stintKey = 0;
    bool stintPitchFailCounted = false;
    std::atomic<uint64_t> seatYawWriteMs{0}, seatPitchWriteMs{0}, seatLimitedMs{0};
    // The last stick call's write (sim thread), put back over what a later extra call adds (aim_caller::
    // PutBack: no stick call since, same record, a QPC cap; Halo 4 also the same publication context and a
    // plausible record; Halo 3 / ODST same_frame, as they always were).
    aim_caller::PutBack putBack{};
    uint64_t putBackNoStickCap = 0;          // Halo 3 / ODST no-unit and unknown callers: 50 ms in QPC counts
    uint32_t stickSeq = 0;                   // sim thread: steps on every stick call
    // Status (sim thread counts, cold worker logs). `calls`: stick calls (the updates).
    std::atomic<uint32_t> calls{0}, writes{0}, arms{0}, disarms{0}, seatedSkips{0}, faults{0};
    std::atomic<uint32_t> seatYawWrites{0}, seatPitchWrites{0}, seatHeld{0}, seatUnverified{0};
    std::atomic<uint32_t> seatStints{0}, seatYawChecks{0}, seatPitchChecks{0}, seatPitchFails{0};
    std::atomic<uint32_t> byRole[aim_caller::kCallerCount]{};   // every call, by caller
    std::atomic<uint32_t> restores{0}, restoresRefused{0};
    std::atomic<uintptr_t> lastUnknownRet{0};                   // Halo 4: the dispatcher's return
#if HALOMCCVR_DEV_DIAGNOSTICS
    // Dev census (aim_caller::TickCensus): calls per sim tick and per caller, on foot vs seated.
    aim_caller::TickCensus census{};
    uint32_t tickSlot = 0, tickField = 0;    // proven at install (AimCallerTickProof); 0 = unreadable
    std::atomic<uint32_t> censusPutBacks{0};
    std::atomic<uint32_t> refusals[aim_caller::kRefusalCount]{};
    // Max |what an extra call added| (put back) and max |native - last write| on a stick call, deg as float
    // bits, [seated][axis].
    std::atomic<uint32_t> extraDeltaMax[2][2]{}, stickDriftMax[2][2]{};
#endif
    uint64_t stintSampleMs = 0;              // the publication the stint last judged (a repeat is no evidence)
    uint32_t stintStaleUpdates = 0;          // consecutive updates of this stint with no fresh publication
    bool stintTitleSeated = false;           // Halo 3 / ODST: unit_in_vehicle said seated when the stint began
                                             // (a walk-up turret reads on foot: its word decides nothing then)
    // The last two seat writes (seqlock), for the input thread's honest-reticle gate: the game's aim lags the
    // latest write by up to a tick, so the gate measures it against whichever of the two is nearer.
    std::atomic<uint32_t> wroteSeq{0};
    std::atomic<float> wroteYaw[2]{}, wrotePitch[2]{};
    std::atomic<bool> wroteValid[2]{};
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

// ValueAgreement: the record holds the game's own aim within 1.7 deg for 30 consecutive updates (stick calls,
// one per rendered frame: ~0.25 s at 120 fps, ~0.5 s at 60).
constexpr float kLegacyDriveAgreeRad = 0.03f;
constexpr uint32_t kLegacyDriveAgreeUpdates = 30;
// The first writes of a seat stint read no limit off the record. Until the input thread sees the seat
// written (LegacyDriveSeatAxes) the servo may still be deflecting the stick, and the game's next update
// moves our first write by that deflection, which the latch would take for a seat limit.
constexpr uint32_t kLegacyDriveSeatPrimeUpdates = 2;
// A stint survives this many consecutive updates with no fresh publication of its seat (the input thread stops
// publishing through a pause's fade and a tracking dropout): ~1 s at 120 fps, ~2 s at 60. Longer, it ends.
constexpr uint32_t kLegacyDriveStintStaleUpdates = 120;
constexpr uint32_t kLegacyDriveAxisYaw = 1u, kLegacyDriveAxisPitch = 2u;
// Halo 3 / ODST: the put-back window of a call that runs with NO stick call since the write -- the no-unit path
// while the player is dead, and any caller the map does not know (the no-unit path reads that way if its own
// proof fails). "No stick call since" never ends those, so only this window does: the 50 ms it always was,
// which closes long before the servo takes the aim back (LegacyDriveLive, 150 ms) and so never erases the
// death camera's stick input. A kick keeps the shared cap (kAimCallerPutBackCapMs): "no stick call since"
// scopes it to this frame's ticks.
constexpr uint64_t kLegacyDrivePutBackNoStickMs = 50;

bool LegacyDriveYawAgrees(bool fresh, float nativeYaw, float aimYaw)
{
    return fresh && std::isfinite(nativeYaw) && std::isfinite(aimYaw) &&
           std::fabs(LegacyDriveAngleDiff(nativeYaw, aimYaw)) < kLegacyDriveAgreeRad;
}

bool LegacyDriveAgrees(bool fresh, float nativeYaw, float nativePitch, float aimYaw, float aimPitch)
{
    return LegacyDriveYawAgrees(fresh, nativeYaw, aimYaw) && std::isfinite(nativePitch) &&
           std::isfinite(aimPitch) && std::fabs(nativePitch - aimPitch) < kLegacyDriveAgreeRad;
}

// Sim thread: the seat stint is over (left the seat, the seat stopped qualifying, a disarm or a fault). The
// next seat proves its record again.
void LegacyDriveForgetSeat(LegacyDriveState& st)
{
    st.stint.reset();
    st.stintActive = false;
    st.stintKey = 0;
    st.stintPitchFailCounted = false;
    st.stintStaleUpdates = 0;
    st.stintTitleSeated = false;
    // The input thread reads an axis as written while these are fresh (LegacyDriveSeatAxes): a stint that
    // is over stops claiming it at once, or the stick stays zeroed and the reticle reads "written" for up to
    // 150 ms over an aim nothing writes any more.
    st.seatYawWriteMs.store(0, std::memory_order_release);
    st.seatPitchWriteMs.store(0, std::memory_order_release);
    st.seatLimitedMs.store(0, std::memory_order_release);
    // ...and its writes are no reference for the next stint's outcome gate.
    st.wroteSeq.fetch_add(1, std::memory_order_acq_rel);
    st.wroteValid[0].store(false, std::memory_order_relaxed);
    st.wroteValid[1].store(false, std::memory_order_relaxed);
    st.wroteSeq.fetch_add(1, std::memory_order_release);
    st.stintSampleMs = 0;
}

#if HALOMCCVR_EXPERIMENTAL_HALO4_CAMERA
// Halo 4, sim thread: the record's own unit (+0x120) through this thread's TLS -- the local output unit,
// proven by Halo4ReadVehicleInputMemory's unique input mapping. -1: unknown (fails closed: never seated, and
// on foot falls back to the first-person stamp); 0: on foot; 1: seated.
int LegacyDriveHalo4RecordSeat(const uint8_t* tls, uintptr_t pc)
{
    __try
    {
        if (!tls || !pc)
            return -1;
        const uint32_t unit = *reinterpret_cast<const uint32_t*>(pc + kHalo4RecordUnitOffset);
        Halo4VehicleInputState seat{};
        if (!Halo4ReadVehicleInputMemory(tls, unit, seat))
            return -1;
        return seat.seated ? 1 : 0;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return -1;
    }
}
#endif

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

// Builds the title's caller map (kLegacyCallerAnchors) and logs what it proved. Returns rung 1: the stick
// site is proven. Otherwise the map stays empty and every caller counts as the update (as before).
bool LegacyDriveBuildCallerMap(int slot, uintptr_t base, size_t size, aim_caller::CallerMap& map)
{
    const auto& site = kLegacyDriveSites[slot];
    const auto& anchors = kLegacyCallerAnchors[slot];
    map.clear();
    const char* why = nullptr;
    uint32_t callTarget = site.entryRva;
    if (anchors.accCallPat)
    {
        // DISPATCHER MODE: the accumulator's one call site, the dispatcher around it, and its frame.
        const uint32_t accCall = anchors.accCallPatRva + anchors.accCallOff;
        const uintptr_t accRet = (accCall == site.primaryCallRva &&
                                  LegacyDriveUniqueAt(base, size, anchors.accCallPatRva, anchors.accCallPat))
            ? AimCallerCallReturn(base, size, accCall, site.entryRva) : 0;
        uint32_t frame = 0;
        if (!accRet)
            why = "the accumulator's call site";
        else if (!LegacyDriveUniqueAt(base, size, anchors.dispatcherRva, anchors.dispatcherPrologue))
            why = "the dispatcher prologue";
        else
            (void)AimCallerProveDispatcher(base, size, anchors.dispatcherRva, accCall, frame, why);
        if (!frame)
        {
            map.clear();
            LOG("DRIVE %s: caller map NOT proven (%s); every call counts as the update (as before)", site.name,
                why ? why : "dispatcher frame");
            return false;
        }
        map.acc_ret = accRet;
        map.dispatcher_frame = frame;
        callTarget = anchors.dispatcherRva;
    }
    else
    {
        // DIRECT MODE: rung 1 is the stick update's own call (primaryCallRva), as before the caller map.
        const uintptr_t primary = AimCallerCallReturn(base, size, site.primaryCallRva, site.entryRva);
        if (primary)
            map.add(primary, aim_caller::Caller::Stick);
    }
    map.no_unit_action = anchors.noUnitAction;
    char sites[256] = "";
    size_t used = 0;
    for (const auto& s : anchors.sites)
    {
        if (!s.pat)
            continue;
        const uintptr_t ret = LegacyDriveUniqueAt(base, size, s.patRva, s.pat)
            ? AimCallerCallReturn(base, size, s.patRva + s.callOff, callTarget) : 0;
        const bool ok = ret && map.add(ret, s.role);
        // Direct mode already holds the stick from primaryCallRva; its pattern is then a cross-check only.
        const bool stickStands = !ok && s.role == aim_caller::Caller::Stick && map.has(aim_caller::Caller::Stick);
        const int n = ok
            ? snprintf(sites + used, sizeof(sites) - used, "%s%s +0x%llX", used ? ", " : "",
                       aim_caller::CallerName(s.role), static_cast<unsigned long long>(ret - base))
            : snprintf(sites + used, sizeof(sites) - used, "%s%s NOT verified (%s)", used ? ", " : "",
                       aim_caller::CallerName(s.role),
                       stickStands ? "the update's own call stands" : "reads unknown");
        if (n > 0)
            used = (std::min)(sizeof(sites) - 1, used + static_cast<size_t>(n));
    }
    if (!map.stick_proven())
    {
        map.clear();
        LOG("DRIVE %s: caller map NOT proven (the stick update's call); every call counts as the update (as "
            "before)", site.name);
        return false;
    }
    if (map.dispatcher_mode())
        LOG("DRIVE %s: caller map proven by the dispatcher's return (accumulator call +0x%X in dispatcher +0x%X, "
            "frame 0x%X from its unwind info and prologue): %s. Only the stick call is the update; an extra call "
            "has the last stick write put back, the no-unit call ends the update",
            site.name, site.primaryCallRva, anchors.dispatcherRva, map.dispatcher_frame, sites);
    else
        LOG("DRIVE %s: caller map proven by the return address: %s. Only the stick call is the update; any other "
            "caller has the last stick write put back (as before)", site.name, sites);
    return true;
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
    // Which caller ran the accumulator (the caller map). Not proven: every caller counts as the update (the
    // old behaviour), and the log says so.
    aim_caller::CallerMap callers;
    (void)LegacyDriveBuildCallerMap(slot, base, size, callers);
#if HALOMCCVR_DEV_DIAGNOSTICS
    uint32_t tickSlot = 0, tickField = 0;
    const bool tickOk = AimCallerTickProof(base, size, indexAddress, kLegacyTickAnchors[slot], tickSlot, tickField);
    LOG("DRIVE %s (dev census): the sim tick %s", site.name,
        tickOk ? "is proven (getter, TLS index and incrementer agree)" : "did NOT verify; the census reads it unreadable");
#endif
    const bool recordUnit = slot == 2 &&
        LegacyDriveBytesAt(base, size, kHalo4RecordUnitLoadRva, kHalo4RecordUnitLoad);
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
    st.axisTicks[0] = st.axisTicks[1] = 0;
    LegacyDriveForgetSeat(st);
    // The hook is not enabled yet (or was removed after its quiescence): the sim-thread state is ours to reset.
    st.putBack = aim_caller::PutBack{};
    st.putBack.max_age = AimCallerClockFromMs(kAimCallerPutBackCapMs);
    // Direct mode (Halo 3 / ODST): the extra calls follow the stick call, so the slot is this frame's write.
    st.putBack.same_frame = !callers.dispatcher_mode();
    st.putBackNoStickCap = AimCallerClockFromMs(kLegacyDrivePutBackNoStickMs);
    st.stickSeq = 0;
    st.callers = callers;
    st.callerMode.store(callers.stick_proven() ? (callers.dispatcher_mode() ? 2 : 1) : 0, std::memory_order_release);
#if HALOMCCVR_DEV_DIAGNOSTICS
    st.census.restart();
    st.tickSlot = tickOk ? tickSlot : 0;
    st.tickField = tickOk ? tickField : 0;
#endif
    st.base = base;
    st.halo4RecordUnit = recordUnit;
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
    if (slot == 2)
        LOG("DRIVE Halo 4: the record's unit load at +0x%X %s", kHalo4RecordUnitLoadRva,
            recordUnit ? "verified (seats may be written)" : "did NOT verify; seats keep the stick servo");
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
// the local player is in a seat; `axes` (seat_drive::Decide) are the axes the hand owns there -- none keeps
// the seat on the servo, exactly as before; `pitchLatch`: the seat-limit latch may read pitch limits;
// `seatKey`: which seat, so a check passed in one seat never carries into the next.
void LegacyDrivePublish(GameTitle title, float yaw, float pitch, float aimYaw, float aimPitch, bool valid,
                        bool seated, seat_drive::Axes axes, bool pitchLatch, uint32_t seatKey)
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
    st.setAxes.store(seated ? (axes.yaw ? kLegacyDriveAxisYaw : 0u) | (axes.pitch ? kLegacyDriveAxisPitch : 0u)
                            : 0u,
                     std::memory_order_relaxed);
    st.setPitchLatch.store(pitchLatch, std::memory_order_relaxed);
    st.setSeatKey.store(seatKey, std::memory_order_relaxed);
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

// Input thread: which axes of a SEAT the hook wrote within 150 ms (seat_direct_drive) -- the servo stands down
// on those. Whether a write was held short at a seat limit: LegacyDriveSeatLimited.
seat_drive::Axes LegacyDriveSeatAxes(GameTitle title)
{
    seat_drive::Axes axes;
    const int slot = LegacyDriveSlot(title);
    if (slot < 0)
        return axes;
    const auto& st = g_legacyDrive[slot];
    if (!st.armed.load(std::memory_order_acquire))
        return axes;
    const uint64_t now = GetTickCount64();
    const auto fresh = [now](uint64_t at) { return at && now >= at && now - at <= 150; };
    axes.yaw = fresh(st.seatYawWriteMs.load(std::memory_order_acquire));
    axes.pitch = fresh(st.seatPitchWriteMs.load(std::memory_order_acquire));
    return axes;
}

// Input thread: the hook's last two seat writes (yaw/pitch as written into the record), newest first. False
// when torn or none.
bool LegacyDriveSeatWrites(GameTitle title, float yaw[2], float pitch[2], bool valid[2])
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0)
        return false;
    auto& st = g_legacyDrive[slot];
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = st.wroteSeq.load(std::memory_order_acquire);
        if (before & 1u)
            continue;
        for (int i = 0; i < 2; ++i)
        {
            yaw[i] = st.wroteYaw[i].load(std::memory_order_relaxed);
            pitch[i] = st.wrotePitch[i].load(std::memory_order_relaxed);
            valid[i] = st.wroteValid[i].load(std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_acquire);
        if (st.wroteSeq.load(std::memory_order_relaxed) == before)
            return valid[0];
    }
    return false;
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

namespace
{
// This (sim) thread's TLS -> the record the native update just wrote. 0 when any link is missing.
uintptr_t LegacyDriveRecord(const LegacyDriveState& st, const LegacyDriveSite& site, int user, const uint8_t** tlsOut)
{
    auto** slots = reinterpret_cast<void**>(__readgsqword(0x58));
    const uint32_t index = *reinterpret_cast<const uint32_t*>(st.tlsIndexAddress);
    auto* tls = (slots && index < 0x200) ? reinterpret_cast<uint8_t*>(slots[index]) : nullptr;
    if (tlsOut)
        *tlsOut = tls;
    const uintptr_t globals = tls ? *reinterpret_cast<const uintptr_t*>(tls + site.tlsSlot) : 0;
    return globals ? globals + static_cast<uintptr_t>(user) * site.stride : 0;
}

// The input thread's latest publication (seqlock): one coherent read, retried once. Torn twice: `coherent`
// false, and the fields hold the last attempt (as the hook always read them).
struct LegacyDrivePublication
{
    float yaw = 0.0f, pitch = 0.0f, aimYaw = 0.0f, aimPitch = 0.0f;
    uint64_t setMs = 0;
    uint32_t axesBits = 0, seatKey = 0;
    bool valid = false, seated = false, pitchLatch = true, coherent = false;
    // What a stick write was made under, for the put-back (aim_caller::Context).
    aim_caller::Context context() const { return {seated, seatKey, axesBits, pitchLatch}; }
};

LegacyDrivePublication LegacyDriveReadPublication(const LegacyDriveState& st)
{
    LegacyDrivePublication p;
    for (int attempt = 0; attempt < 2 && !p.coherent; ++attempt)
    {
        const uint32_t before = st.seq.load(std::memory_order_acquire);
        if (before & 1u)
            continue;
        p.yaw = st.yaw.load(std::memory_order_relaxed);
        p.pitch = st.pitch.load(std::memory_order_relaxed);
        p.aimYaw = st.aimYaw.load(std::memory_order_relaxed);
        p.aimPitch = st.aimPitch.load(std::memory_order_relaxed);
        p.setMs = st.setMs.load(std::memory_order_relaxed);
        p.valid = st.setValid.load(std::memory_order_relaxed);
        p.seated = st.setSeated.load(std::memory_order_relaxed);
        p.axesBits = st.setAxes.load(std::memory_order_relaxed);
        p.pitchLatch = st.setPitchLatch.load(std::memory_order_relaxed);
        p.seatKey = st.setSeatKey.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);   // the loads above stay before the re-read
        p.coherent = st.seq.load(std::memory_order_relaxed) == before;
    }
    return p;
}

// Which caller ran the accumulator this time (the caller map). `dispatcherRet`: Halo 4's dispatcher return,
// read at entry rsp + the proven frame (0 when not in dispatcher mode or not read).
aim_caller::Caller LegacyDriveClassify(const LegacyDriveState& st, uintptr_t ret, const uint8_t* entryRsp,
                                       uintptr_t& dispatcherRet)
{
    dispatcherRet = 0;
    __try
    {
        dispatcherRet = aim_caller::DispatcherReturn(st.callers, entryRsp, ret);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        dispatcherRet = 0;
    }
    return st.callers.classify(ret, dispatcherRet);
}

void LegacyDriveRefused(LegacyDriveState& st, aim_caller::Refusal why)
{
    st.restoresRefused.fetch_add(1, std::memory_order_relaxed);
#if HALOMCCVR_DEV_DIAGNOSTICS
    st.refusals[static_cast<int>(why)].fetch_add(1, std::memory_order_relaxed);
#else
    (void)why;
#endif
}

// Sim thread, an EXTRA call (aim_caller::Action::RestoreOnly -- Halo 4's effect impulse and weapon sway, Halo 3
// / ODST's weapon kick and no-unit path, any caller the map does not know): put the last stick write back over
// what this call accumulated, on the axes that write wrote, so the next stick call reads the record it left --
// no count, no seat check, no arm-gate step, no latch. On foot this keeps what the hook did before (it rewrote
// the setpoint on every call, cancelling the kick); a seat no longer reads an impulse or a sway as a limit.
// Halo 4's extra calls PRECEDE the stick call, so the slot holds the previous frame's write: it is honoured
// only while no stick call has run since, on the same record, under the same publication context, and while
// the record sits within an impulse of it (aim_caller::PutBack::check). Halo 3's / ODST's extra calls FOLLOW
// the stick call (same_frame): this frame's write, on the same record, with no publication, context or
// plausibility test, as before -- only the window is per caller (kLegacyDrivePutBackNoStickMs).
void LegacyDrivePutBack(LegacyDriveState& st, const LegacyDriveSite& site, int user, aim_caller::Caller caller)
{
    if (!g_config.legacy_direct_drive || !st.installed.load(std::memory_order_acquire) ||
        !st.armed.load(std::memory_order_acquire) || !st.tlsIndexAddress)
        return;
    if (!st.putBack.valid)
    {
        LegacyDriveRefused(st, aim_caller::Refusal::Invalid);
        return;
    }
    const uintptr_t pc = LegacyDriveRecord(st, site, user, nullptr);
    if (!pc || pc != st.expected.load(std::memory_order_acquire))
    {
        LegacyDriveRefused(st, aim_caller::Refusal::Record);
        return;
    }
    aim_caller::Context context{};
    uint64_t cap = st.putBack.max_age;
    if (!st.putBack.same_frame)
    {
        // Halo 4: the previous frame's write, honoured only under the publication it was written under.
        const LegacyDrivePublication pub = LegacyDriveReadPublication(st);
        if (!pub.coherent || !pub.valid || (pub.seated && !g_config.seat_direct_drive))
        {
            LegacyDriveRefused(st, aim_caller::Refusal::Context);
            return;
        }
        context = pub.context();
    }
    else if (caller != aim_caller::Caller::Kick)
    {
        cap = st.putBackNoStickCap;   // Halo 3 / ODST: no stick call ends these; only their 50 ms does
    }
    float* pair = reinterpret_cast<float*>(pc + site.angles);
    const float native[2]{pair[0], pair[1]};
    const aim_caller::Refusal why = st.putBack.check(pc, context, st.stickSeq, AimCallerClockNow(), native, cap);
    if (why != aim_caller::Refusal::None)
    {
        LegacyDriveRefused(st, why);
        return;
    }
#if HALOMCCVR_DEV_DIAGNOSTICS
    // On foot or seated as the stick write was (Halo 4: the same as now, or the context check refused it).
    for (int a = 0; a < 2; ++a)
        if (st.putBack.axis[a])
        {
            const float d = a == 0 ? LegacyDriveAngleDiff(native[a], st.putBack.value[a])
                                   : native[a] - st.putBack.value[a];
            aim_caller::AtomicMaxFloat(st.extraDeltaMax[st.putBack.ctx.seated ? 1 : 0][a], std::fabs(d) * 57.29578f);
        }
    st.censusPutBacks.fetch_add(1, std::memory_order_relaxed);
#endif
    if (st.putBack.axis[0])
        pair[0] = st.putBack.value[0];
    if (st.putBack.axis[1])
        pair[1] = st.putBack.value[1];
    st.restores.fetch_add(1, std::memory_order_relaxed);
}

// Sim thread, Halo 4's NO-UNIT call (aim_caller::Action::EndUpdate): the pass had no unit, so nothing is
// written this update -- the epilogue of an update that wrote nothing, and only that: the latch forgets its
// last value, the seat stint is over (a stint must not survive a death), each axis's prime starts again, and
// nothing is left to put back. The arm gate, the stint check and the counters are not stepped.
void LegacyDriveEndUpdate(LegacyDriveState& st)
{
    st.putBack.invalidate();
    st.seatLatch.reset();
    st.axisTicks[0] = st.axisTicks[1] = 0;
    LegacyDriveForgetSeat(st);
}

#if HALOMCCVR_DEV_DIAGNOSTICS
// Dev census: one call, its caller and the sim tick it saw (this thread's TLS).
void LegacyDriveCensusNote(LegacyDriveState& st, const LegacyDriveSite& site, int user, aim_caller::Caller caller)
{
    const uint8_t* tls = nullptr;
    uint32_t tick = 0;
    __try
    {
        if (st.tlsIndexAddress)
            (void)LegacyDriveRecord(st, site, user, &tls);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        tls = nullptr;
    }
    const bool tickOk = AimCallerReadTick(tls, st.tickSlot, st.tickField, tick);
    st.census.note(tickOk, tick, caller, st.setSeated.load(std::memory_order_relaxed));
}
#endif
} // namespace

extern "C" void LegacyPcApplyForUser(int slot, int user, uintptr_t ret, const uint8_t* entryRsp)
{
    if (slot < 0 || slot > 2 || user != 0)
        return;
    auto& st = g_legacyDrive[slot];
    const auto& site = kLegacyDriveSites[slot];
    st.callbacks.fetch_add(1, std::memory_order_acq_rel);
    // WHICH CALLER (aim_caller_logic.h). The stick call is the update; until the stick site is proven every
    // call is, as before.
    uintptr_t dispatcherRet = 0;
    const aim_caller::Caller caller = LegacyDriveClassify(st, ret, entryRsp, dispatcherRet);
    const aim_caller::Action action = aim_caller::ActionFor(st.callers, caller);
    st.byRole[static_cast<int>(caller)].fetch_add(1, std::memory_order_relaxed);
    if (caller == aim_caller::Caller::Unknown && st.callers.stick_proven())
        st.lastUnknownRet.store(st.callers.dispatcher_mode() ? dispatcherRet : ret, std::memory_order_relaxed);
#if HALOMCCVR_DEV_DIAGNOSTICS
    LegacyDriveCensusNote(st, site, user, caller);
#endif
    if (action != aim_caller::Action::StepAndWrite)
    {
        __try
        {
            if (action == aim_caller::Action::RestoreOnly)
                LegacyDrivePutBack(st, site, user, caller);
            else
                LegacyDriveEndUpdate(st);
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
            // A fault disarms like a disarm: the record is proven again from a count of zero.
            if (st.armed.exchange(false, std::memory_order_acq_rel))
                st.disarms.fetch_add(1, std::memory_order_relaxed);
            st.agree = 0;
            st.putBack.invalidate();
            st.faults.fetch_add(1, std::memory_order_relaxed);
        }
        st.callbacks.fetch_sub(1, std::memory_order_acq_rel);
        return;
    }
    // THE UPDATE (the stick call, or every call while the caller map is not proven).
    st.calls.fetch_add(1, std::memory_order_relaxed);
    ++st.stickSeq;            // nothing written before this call may be put back after it
#if HALOMCCVR_DEV_DIAGNOSTICS
    const aim_caller::PutBack previousWrite = st.putBack;
#endif
    st.putBack.invalidate();  // this update writes afresh, or nothing is put back after it
    bool seatWrote = false;   // this update ended in a seat write (the latch keeps its last value only then)
    bool seatKept = false;    // this update continued a seat stint (its checks survive)
    bool wroteAxis[2]{};      // which axes this update's seat write wrote
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
            const uint8_t* tls = nullptr;
            const uintptr_t pc = LegacyDriveRecord(st, site, user, &tls);
            if (!pc)
                break;
            float* pair = reinterpret_cast<float*>(pc + site.angles);
            const float nativeYaw = pair[0], nativePitch = pair[1];
            st.nativeYaw.store(nativeYaw, std::memory_order_relaxed);
            st.nativePitch.store(nativePitch, std::memory_order_relaxed);
            const LegacyDrivePublication pub = LegacyDriveReadPublication(st);
            const float yaw = pub.yaw, pitch = pub.pitch, aimYaw = pub.aimYaw, aimPitch = pub.aimPitch;
            const uint64_t setMs = pub.setMs;
            const uint32_t axesBits = pub.axesBits, seatKey = pub.seatKey;
            const bool valid = pub.valid, seatedSet = pub.seated, pitchLatch = pub.pitchLatch, coherent = pub.coherent;
#if HALOMCCVR_DEV_DIAGNOSTICS
            // What reached the record between our last write and this stick call. Halo 3, ODST and Halo 2 may
            // fold an impulse into the stick call itself, which no caller filter can see (research item 8):
            // non-zero here under damage, with the stick neutral in a written seat, says so.
            if (previousWrite.valid && previousWrite.pc == pc && previousWrite.stick_seq + 1 == st.stickSeq)
            {
                const float nativePair[2]{nativeYaw, nativePitch};
                for (int a = 0; a < 2; ++a)
                    if (previousWrite.axis[a])
                    {
                        const float d = a == 0 ? LegacyDriveAngleDiff(nativePair[a], previousWrite.value[a])
                                               : nativePair[a] - previousWrite.value[a];
                        aim_caller::AtomicMaxFloat(st.stickDriftMax[seatedSet ? 1 : 0][a],
                                                   std::fabs(d) * 57.29578f);
                    }
            }
#endif
            const uint64_t now = GetTickCount64();
            const bool fresh = coherent && valid && setMs && now >= setMs && now - setMs <= 200;
            // ValueAgreement (the arm gate): the record holds the game's own aim.
            const bool agrees = LegacyDriveAgrees(fresh, nativeYaw, nativePitch, aimYaw, aimPitch);
            // ON FOOT OR SEATED. The input thread says which (its seated flag is the first-person seat it
            // composes the aim in, walk-up turrets included, which the game's unit_in_vehicle calls on foot);
            // on foot also needs the title's own word. Halo 4 asks the record's own unit, on this thread.
            bool onFoot = !seatedSet && LegacyDriveLocalOnFoot(slot);
            bool seatConfirmed = seatedSet;
            int recordSeat = -1;   // Halo 4: the record's own unit -- -1 unknown, 0 on foot, 1 seated
#if HALOMCCVR_EXPERIMENTAL_HALO4_CAMERA
            if (slot == 2)
            {
                recordSeat = st.halo4RecordUnit ? LegacyDriveHalo4RecordSeat(tls, pc) : -1;
                onFoot = !seatedSet && (recordSeat == 0 || (recordSeat < 0 && LegacyDriveLocalOnFoot(slot)));
                seatConfirmed = seatedSet && recordSeat == 1;
            }
#else
            if (slot == 2)
                seatConfirmed = false;
#endif
            // A seat is written only on the axes the input thread says the hand owns there; a seat with none
            // keeps the servo, exactly as before.
            const uint32_t wantAxes =
                (!onFoot && seatConfirmed && g_config.seat_direct_drive && fresh) ? axesBits : 0u;
            const bool seatWrite = wantAxes != 0;
            seatKept = seatWrite;
            if (!onFoot && !seatWrite)
            {
                // A GAP in this stint's seat is not the end of it: write nothing, keep the proven checks (no step:
                // a gap is no evidence, and StintCheck counts one at most gap_ms), for a bounded run of updates.
                // A gap is either a stale or torn publication of the same seat (a pause's fade, a tracking
                // dropout: the input thread stops publishing), or a fresh one that lost the seat's VIEW (Halo 4's
                // seat stamp goes stale in the theater) while the title's own word says the seat is still taken.
                // That word ends the stint when it says the unit left -- except where it cannot tell: a Halo 3 /
                // ODST walk-up turret reads "on foot" (unit_in_vehicle 0), and Halo 4's record may be unreadable;
                // there only the publication's own seat keeps the stint. Another seat, or a fresh publication of
                // this seat with no axes, ends it.
                const bool titleKnowsSeat = slot == 2 ? recordSeat >= 0 : st.stintTitleSeated;
                const bool titleSeated = slot == 2 ? recordSeat == 1 : !LegacyDriveLocalOnFoot(slot);
                const bool sameSeatStale = !fresh && (!coherent || (seatedSet && seatKey == st.stintKey));
                const bool seatViewLost = fresh && !seatedSet && titleKnowsSeat && titleSeated;
                if (st.stintActive && (titleKnowsSeat ? titleSeated : true) && (sameSeatStale || seatViewLost) &&
                    ++st.stintStaleUpdates <= kLegacyDriveStintStaleUpdates)
                    seatKept = true;
                st.seatedSkips.fetch_add(1, std::memory_order_relaxed);
                st.agree = 0;
                break;
            }
            // THE SEAT STINT: a new seat starts its own checks; each update of it steps them, armed or not,
            // under the servo (an axis is not written until it passes, so its record is the game's own).
            if (seatWrite)
            {
                if (!st.stintActive || st.stintKey != seatKey)
                {
                    LegacyDriveForgetSeat(st);
                    st.seatLatch.reset();
                    st.axisTicks[0] = st.axisTicks[1] = 0;
                    st.stintActive = true;
                    st.stintKey = seatKey;
                    st.stintTitleSeated = slot != 2 && !LegacyDriveLocalOnFoot(slot);
                    st.seatStints.fetch_add(1, std::memory_order_relaxed);
                }
                st.stintStaleUpdates = 0;
                const bool yawWas = st.stint.yaw_ok, pitchWas = st.stint.pitch_ok;
                const bool newSample = setMs != st.stintSampleMs;
                st.stintSampleMs = setMs;
                st.stint.step(fresh, nativeYaw, nativePitch, aimYaw, aimPitch, now, newSample);
                if (!yawWas && st.stint.yaw_ok)
                    st.seatYawChecks.fetch_add(1, std::memory_order_relaxed);
                if (!pitchWas && st.stint.pitch_ok)
                    st.seatPitchChecks.fetch_add(1, std::memory_order_relaxed);
                // Counted only where the seat wants its pitch (a Halo 4 seat never does).
                if (st.stint.pitch_failed && !st.stintPitchFailCounted && (wantAxes & kLegacyDriveAxisPitch))
                {
                    st.stintPitchFailCounted = true;
                    st.seatPitchFails.fetch_add(1, std::memory_order_relaxed);
                }
            }
            if (!st.armed.load(std::memory_order_acquire))
            {
                // ValueAgreement: the record must hold the game's own aim before it is ever written -- both
                // axes on foot; yaw in a seat, whose pitch need not (its own check decides that axis).
                const bool armAgree = seatWrite ? LegacyDriveYawAgrees(fresh, nativeYaw, aimYaw) : agrees;
                st.agree = armAgree ? st.agree + 1 : 0;
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
            bool writeYaw = true, writePitch = true;
            if (seatWrite)
            {
                writeYaw = (wantAxes & kLegacyDriveAxisYaw) && st.stint.yaw_ok;
                writePitch = (wantAxes & kLegacyDriveAxisPitch) && st.stint.yaw_ok && st.stint.pitch_ok;
                if (!writeYaw && !writePitch)
                {
                    // Waiting on a check only while one is pending; a pitch that failed its check is on the
                    // servo for the stint, which is not a wait.
                    const bool pending = ((wantAxes & kLegacyDriveAxisYaw) && !st.stint.yaw_ok) ||
                        ((wantAxes & kLegacyDriveAxisPitch) && !st.stint.pitch_ok && !st.stint.pitch_failed);
                    if (pending)
                        st.seatUnverified.fetch_add(1, std::memory_order_relaxed);
                    break;
                }
            }
            if (fresh && std::isfinite(yaw) && std::isfinite(pitch))
            {
                // Stay on the branch the game's yaw is on (never a 2*pi jump) and inside its pitch clamp.
                float y = aim_write::unwrap_to_branch(yaw, nativeYaw);
                float p = aim_write::clamp_pitch(pitch, 1.45f);
                bool limited = false;
                if (seatWrite)
                {
                    // The game clamped the record to the seat's limits before this write; the latch reads that
                    // clamp off last tick's write and keeps this one at the limit (one probe step past it), so a
                    // passenger's cone holds the aim at its edge instead of the write running on. Per axis: an
                    // axis's first writes read no limit (kLegacyDriveSeatPrimeUpdates), and a look-steered driver's
                    // pitch is never latched -- the accumulator levels it as the vehicle moves, not a limit.
                    const bool latch[2]{writeYaw && st.axisTicks[0] >= kLegacyDriveSeatPrimeUpdates,
                                        writePitch && pitchLatch && st.axisTicks[1] >= kLegacyDriveSeatPrimeUpdates};
                    const float native[2]{nativeYaw, nativePitch}, target[2]{y, p};
                    float held[2]{};
                    limited = st.seatLatch.apply(native, target, held, latch);
                    if (latch[0])
                        y = aim_write::unwrap_to_branch(held[0], nativeYaw);
                    if (latch[1])
                        p = aim_write::clamp_pitch(held[1], 1.45f);
                }
                if (std::isfinite(y) && std::isfinite(p))
                {
                    if (writeYaw)
                        pair[0] = y;
                    if (writePitch)
                        pair[1] = p;
                    // What a later extra call may put back (aim_caller::PutBack), under this publication.
                    const bool putAxes[2]{writeYaw, writePitch};
                    const float putValues[2]{y, p};
                    st.putBack.wrote(pc, putAxes, putValues, pub.context(), st.stickSeq, AimCallerClockNow());
                    if (seatWrite)
                    {
                        // Recorded: every axis written that the latch reads, its prime included (the next
                        // update reads a limit off this value once the axis is primed).
                        const float written[2]{y, p};
                        const bool recorded[2]{writeYaw, writePitch && pitchLatch};
                        st.seatLatch.wrote(written, recorded);
                        seatWrote = true;
                        wroteAxis[0] = writeYaw;
                        wroteAxis[1] = writePitch;
                        st.seatYawWriteMs.store(writeYaw ? now : 0, std::memory_order_release);
                        st.seatPitchWriteMs.store(writePitch ? now : 0, std::memory_order_release);
                        st.seatLimitedMs.store(limited ? now : 0, std::memory_order_release);
                        st.wroteSeq.fetch_add(1, std::memory_order_acq_rel);
                        st.wroteYaw[1].store(st.wroteYaw[0].load(std::memory_order_relaxed), std::memory_order_relaxed);
                        st.wrotePitch[1].store(st.wrotePitch[0].load(std::memory_order_relaxed),
                                               std::memory_order_relaxed);
                        st.wroteValid[1].store(st.wroteValid[0].load(std::memory_order_relaxed),
                                               std::memory_order_relaxed);
                        st.wroteYaw[0].store(writeYaw ? y : nativeYaw, std::memory_order_relaxed);
                        st.wrotePitch[0].store(writePitch ? p : nativePitch, std::memory_order_relaxed);
                        st.wroteValid[0].store(true, std::memory_order_relaxed);
                        st.wroteSeq.fetch_add(1, std::memory_order_release);
                        if (writeYaw)
                            st.seatYawWrites.fetch_add(1, std::memory_order_relaxed);
                        if (writePitch)
                            st.seatPitchWrites.fetch_add(1, std::memory_order_relaxed);
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
        // A fault disarms like a disarm: the record is proven again from a count of zero.
        if (st.armed.exchange(false, std::memory_order_acq_rel))
            st.disarms.fetch_add(1, std::memory_order_relaxed);
        st.agree = 0;
        st.putBack.invalidate();
        seatWrote = false;
        seatKept = false;
        wroteAxis[0] = wroteAxis[1] = false;
        st.faults.fetch_add(1, std::memory_order_relaxed);
    }
    // The latch's contract is "nothing written last tick: forget everything". Every update that did not end
    // in a seat write -- on foot, a skipped or unverified seat, disarmed, switched off, a fault -- resets it
    // here, so a later seat write never reads a limit off a value this hook did not write. Per axis, an axis
    // this update did not write starts its prime again.
    for (int a = 0; a < 2; ++a)
        st.axisTicks[a] = wroteAxis[a]
            ? (st.axisTicks[a] < kLegacyDriveSeatPrimeUpdates ? st.axisTicks[a] + 1 : st.axisTicks[a])
            : 0;
    if (!seatWrote)
        st.seatLatch.reset();
    if (!seatKept)
    {
        LegacyDriveForgetSeat(st);
    }
    else if (!seatWrote)
    {
        // Still in the seat, but nothing written this update (an axis not yet proven): nothing reads as written.
        st.seatYawWriteMs.store(0, std::memory_order_release);
        st.seatPitchWriteMs.store(0, std::memory_order_release);
        st.seatLimitedMs.store(0, std::memory_order_release);
    }
    st.callbacks.fetch_sub(1, std::memory_order_acq_rel);
}

// Cold worker, every 2 s, per title with a hook that ran. "Updates" are stick calls -- one per rendered frame
// (~240 in 2 s at 120 fps), not per sim tick. The callers list counts every call by its caller.
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
        const char* name = kLegacyDriveSites[slot].name;
        uint32_t roles[aim_caller::kCallerCount]{};
        uint32_t extra = 0;
        for (int r = 0; r < aim_caller::kCallerCount; ++r)
        {
            roles[r] = st.byRole[r].exchange(0, std::memory_order_relaxed);
            if (r != static_cast<int>(aim_caller::Caller::Stick))
                extra += roles[r];
        }
        const uint32_t restores = st.restores.exchange(0, std::memory_order_relaxed);
        const uint32_t refused = st.restoresRefused.exchange(0, std::memory_order_relaxed);
        const uintptr_t unknownRet = st.lastUnknownRet.load(std::memory_order_relaxed);
        const unsigned long long unknownRva = unknownRet >= st.base ? unknownRet - st.base : 0;
        const uint8_t mode = st.callerMode.load(std::memory_order_acquire);
        char callers[224];
        snprintf(callers, sizeof(callers),
                 "callers%s: stick %u, impulse %u, sway %u, kick %u, no-unit %u, unknown %u (last +0x%llX); put back "
                 "%u (refused %u)",
                 mode ? "" : " (map not proven: every call is an update)",
                 roles[static_cast<int>(aim_caller::Caller::Stick)], roles[static_cast<int>(aim_caller::Caller::Impulse)],
                 roles[static_cast<int>(aim_caller::Caller::Sway)], roles[static_cast<int>(aim_caller::Caller::Kick)],
                 roles[static_cast<int>(aim_caller::Caller::NoUnit)],
                 roles[static_cast<int>(aim_caller::Caller::Unknown)], unknownRva, restores, refused);
        const uint32_t calls = st.calls.exchange(0, std::memory_order_relaxed);
        if (!calls)
        {
            // No stick update (dead, no unit, a pass with the stick skipped): only extra calls ran, which is
            // when their count matters.
            if (extra)
                LOG("DRIVE %s: no stick update; %s", name, callers);
        }
        else
        {
            LOG("DRIVE %s: armed=%d updates %u, wrote %u; arms %u, disarms %u, seated %u, faults %u; record yaw %.3f "
                "pitch %.3f vs game aim yaw %.3f pitch %.3f; %s",
                name, st.armed.load() ? 1 : 0, calls, st.writes.exchange(0), st.arms.exchange(0),
                st.disarms.exchange(0), st.seatedSkips.exchange(0), st.faults.exchange(0), st.nativeYaw.load(),
                st.nativePitch.load(), st.aimYaw.load(), st.aimPitch.load(), callers);
            const uint32_t stints = st.seatStints.exchange(0), yawWrites = st.seatYawWrites.exchange(0),
                           pitchWrites = st.seatPitchWrites.exchange(0), unverified = st.seatUnverified.exchange(0),
                           yawChecks = st.seatYawChecks.exchange(0), pitchChecks = st.seatPitchChecks.exchange(0),
                           pitchFails = st.seatPitchFails.exchange(0), held = st.seatHeld.exchange(0);
            if (stints || yawWrites || pitchWrites || unverified || yawChecks || pitchChecks || pitchFails || held)
                LOG("DRIVE %s seats: new stints %u; checks passed yaw %u pitch %u, pitch left on the servo %u; writes "
                    "yaw %u pitch %u (held at a seat limit %u); updates waiting on a check %u; this stint yaw %s, "
                    "pitch %s",
                    name, stints, yawChecks, pitchChecks, pitchFails, yawWrites, pitchWrites, held, unverified,
                    st.stint.yaw_ok ? "proven" : "checking",
                    st.stint.pitch_ok ? "proven" : st.stint.pitch_failed ? "servo" : "checking");
        }
#if HALOMCCVR_DEV_DIAGNOSTICS
        // THE CENSUS: calls per sim tick and per caller. "stick per tick" is frames per tick (2.00 at 120 fps
        // over a 60 Hz sim); ~1.00 at 120 fps would mean the pass runs per tick after all.
        const aim_caller::CensusTotals t = st.census.take();
        uint32_t byRole[aim_caller::kCallerCount]{}, any = 0;
        for (int r = 0; r < aim_caller::kCallerCount; ++r)
        {
            byRole[r] = t.calls[0][r] + t.calls[1][r];
            any += byRole[r];
        }
        if (any)
        {
            const auto role = [](aim_caller::Caller c) { return static_cast<int>(c); };
            LOG("DRIVE %s callers/tick (dev): ticks %u, stick %u (%.2f per tick), impulse %u, sway %u, kick %u, "
                "no-unit %u, unknown %u (last unknown %s +0x%llX); seated of those: stick %u, impulse %u, sway %u, "
                "kick %u, no-unit %u, unknown %u; put back %u; max calls in one tick %u; ticks with an extra call "
                "before a stick %u (stick calls with one ahead %u); ticks with no stick %u; stalled ticks %u; tick "
                "went back %u; tick unreadable %u",
                name, t.ticks, byRole[role(aim_caller::Caller::Stick)],
                t.ticks ? static_cast<double>(t.sticks) / t.ticks : 0.0, byRole[role(aim_caller::Caller::Impulse)],
                byRole[role(aim_caller::Caller::Sway)], byRole[role(aim_caller::Caller::Kick)],
                byRole[role(aim_caller::Caller::NoUnit)], byRole[role(aim_caller::Caller::Unknown)],
                mode == 2 ? "dispatcher return" : "return", unknownRva,
                t.calls[1][role(aim_caller::Caller::Stick)], t.calls[1][role(aim_caller::Caller::Impulse)],
                t.calls[1][role(aim_caller::Caller::Sway)], t.calls[1][role(aim_caller::Caller::Kick)],
                t.calls[1][role(aim_caller::Caller::NoUnit)], t.calls[1][role(aim_caller::Caller::Unknown)],
                st.censusPutBacks.exchange(0, std::memory_order_relaxed), t.max_calls, t.extra_before_stick,
                t.extra_passes, t.no_stick, t.stalled, t.went_back, t.unreadable);
            const auto deg = [](std::atomic<uint32_t>& bits) {
                return aim_caller::BitsToFloat(bits.exchange(0, std::memory_order_relaxed));
            };
            uint32_t why[aim_caller::kRefusalCount]{};
            for (int r = 0; r < aim_caller::kRefusalCount; ++r)
                why[r] = st.refusals[r].exchange(0, std::memory_order_relaxed);
            const float e00 = deg(st.extraDeltaMax[0][0]), e01 = deg(st.extraDeltaMax[0][1]);
            const float e10 = deg(st.extraDeltaMax[1][0]), e11 = deg(st.extraDeltaMax[1][1]);
            const float d00 = deg(st.stickDriftMax[0][0]), d01 = deg(st.stickDriftMax[0][1]);
            const float d10 = deg(st.stickDriftMax[1][0]), d11 = deg(st.stickDriftMax[1][1]);
            LOG("DRIVE %s deltas (dev): an extra call's |put back| max on foot yaw %.2f pitch %.2f, seated yaw %.2f "
                "pitch %.2f deg; a stick call's |record - last write| max on foot yaw %.2f pitch %.2f, seated yaw "
                "%.2f pitch %.2f deg; put-back refused: nothing written %u, record %u, stick since %u, context %u, "
                "age %u, implausible %u",
                name, e00, e01, e10, e11, d00, d01, d10, d11,
                why[static_cast<int>(aim_caller::Refusal::Invalid)], why[static_cast<int>(aim_caller::Refusal::Record)],
                why[static_cast<int>(aim_caller::Refusal::StickSince)],
                why[static_cast<int>(aim_caller::Refusal::Context)], why[static_cast<int>(aim_caller::Refusal::Age)],
                why[static_cast<int>(aim_caller::Refusal::Implausible)]);
        }
#endif
    }
}
