// Included in game.cpp at file scope, just before Game_ComputeAimStick.
//
// HALO 3 / ODST / HALO 4 DIRECT DRIVE (legacy_direct_drive = 1, derived from aim_direct_drive, default 1).
// Reach's direct drive (reach_direct_drive = 3) for the three sibling engines, which otherwise aim with the
// right-stick servo: the servo can only ask the game to turn at its own turn rate, so the aim lags the hand,
// and it needs the game to accept stick input at all. Here the aim is written where the game keeps it.
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
// record axis by axis before that axis is written (seat_drive::StintCheck: yaw on 30 agreeing samples, judged
// with a tolerance that grows with the motion since the last judged sample -- agree under tol + motion, disagree
// over tol + 3 * motion, a jump over 5.7 deg is no evidence -- so a turn, a slewing barrel or flight still proves
// it; pitch the same within 3 s of evidence time after yaw passed, never before yaw. A failed pitch retries
// every 5 s; a Halo 3 driver+gunner's pitch sits ~13 deg off the gun and keeps failing, so it stays on the
// servo). The write goes through Reach's seat-limit latch (aim_write::LimitLatch), pitch excepted for a
// LOOK-STEERED driver, whose pitch the accumulator levels as the vehicle moves. A seat may also ARM the hook,
// on yaw agreement alone: a checkpoint that loads in a vehicle.
//
// THE SEAT'S FRAME (2026-10-01; seat_frame_reader.h, docs/ALL-TITLES-AIM-2026-09-29.md "Seats aim in world
// angles"). The record is NOT always in world angles: each title's rebase (Halo 3 0x355E20, ODST 0x39A600, Halo 4
// 0x5F97DC) rotates it by the vehicle's up frame R(u) unless the seat's live flags carry bit 4. So each seated
// update first reads, on this thread and from the unit the rebase itself will rotate, what the rebase will do this
// tick: WORLD (bit 4, or its own early exit), case (a) (our Halo 3 / ODST skip: the flag is clear only because C20
// cleared it, and the rebase is jumped over -- what an unmodded peer computes), or case (b) (the seat's own data
// lacks bit 4: every peer applies R(u) to the action angles we send). In case (b) the hand's world aim AND the
// game's world aim are taken into the record's frame -- the record holds R^-1 of the aim -- before the stint check,
// the arm gate, the seat-limit latch and the write, so every comparison is like with like. A frame that does not
// read makes NO seated write that update (the servo keeps the seat; the stint survives it like a gap).
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
// of ticks after a hitch outlasted that and left the kick for the seat latch); the NO-UNIT call runs with no
// stick call at all while the player is dead, so it keeps the 50 ms window, now on QPC
// (kLegacyDrivePutBackNoStickMs), as does any caller the map does not know (the no-unit call reads that way if
// its own proof fails). An unproven stick site leaves every call counting as the update, as before.
// "Updates" are therefore stick calls, and the control pass runs once per MAIN-LOOP ITERATION -- a rendered
// frame -- ahead of that frame's 0..N sim ticks: ~120 per second at 120 fps, not the sim's 60 Hz.
//
// GUN-SLAVED SEATS (Halo 3 / ODST seat flag bit 7, gun_slaved_seat_logic.h; 2026-10-02). In a seat whose first-
// person camera is the GUN -- the Warthog turret's gunner -- the game's aim the input thread publishes is the gun's
// marker, and the gun slews toward the record at its own rate. Proving the record against it proved it against a
// quantity that lags by design: every turret entry spent 1-2 s unwritten, short ones never wrote, a pause restarted
// the wait, and a pitch left on the stick servo nodded. So such a seat ALSO proves its record against the
// OCCUPANT's own aim vector -- the field the camera reads in every other seat and on foot, and the one the control
// update re-derives the record from (both found by signature at install, LegacyDriveBindUnitAim). A second
// StintCheck (stintOccupant) runs beside the camera's; an axis is proven when EITHER passes it
// (seat_drive::CombineStints), so the seat is never slower than before and fails closed to the camera check when
// the vector is unbound or disagrees. The arm gate takes either reference on yaw. The occupant's aim (world
// angles, with how long it has agreed with the record per axis) is published for the input thread, whose stick
// servo closes on it instead of the slewing gun once it is trusted (gun_slaved::ServoFeedback). The writes are
// unchanged: world angles, case (a), both axes. The gun stays the outcome -- for the witnesses, and for one guard:
// an axis written on the occupant's proof alone loses it for the stint if the gun comes to REST more than 3 deg off
// the record (gun_slaved::OffsetWatch: lag ends when the gun stops, a real marker offset does not).

// SEATEXIT witness (dev builds only): every accumulator call is recorded into a fixed ring
// (seat_exit_witness.h). The macro's argument is compiled only in a dev build.
#if HALOMCCVR_DEV_DIAGNOSTICS
#define LEGACY_DRIVE_WITNESS(...) do { __VA_ARGS__; } while (0)
#else
#define LEGACY_DRIVE_WITNESS(...) do {} while (0)
#endif

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

// THE OCCUPANT'S AIM VECTOR, Halo 3 and ODST (gun_slaved_seat_logic.h). Two consumers, each a pattern that must be
// UNIQUE in the module (found anywhere: the offsets are decoded from the matched instructions, never assumed):
//   * the first-person camera's forward read, through the seat-flag test that swaps it for the gun's marker
//     (Halo 3 0x17DB44 in 0x17DAEC, ODST 0x1AC6C8 in 0x1AC670): `movsd xmm0,[unit+X]; ...; mov eax,[unit+X+8]`,
//     then the seat block's `imul rdx, rax, stride` and `shr eax, 7; test r?b, al`;
//   * the control update's reset of the record from the same vector (Halo 3 0xF7075 in 0xF66AC, ODST 0x112C50 in
//     0x112184): `movsd xmm0,[unit+X]; ...; mov eax,[unit+X+8]`, atan2/asin, `movss [record+0xD8]` (pitch) and
//     `movss [record+0xD4]` (yaw).
// Measured 2026-10-02 on the retail Steam modules (Halo 3 X = 0x1AC, ODST X = 0x1A4; the Store modules are byte-
// identical, MCC-EDITIONS-EVIDENCE). ADDR-HYGIENE: resolved (unique signatures, decoded and cross-checked:
// gun_slaved::ResolveUnitAimOffset), then guarded per seat stint by the occupant's own ValueAgreement -- the
// record must agree with e(vector) for 30 judged samples per axis before that check proves anything, and the
// vector must read as a unit normal. Not bound: every gun-slaved seat proves against the camera, as before.
struct LegacyUnitAimSite
{
    const char* camera;
    uint8_t camForward, camForwardZ, camStride, camShift;
};
constexpr const char* kLegacyUnitAimControl =
    "44 84 AB ?? ?? 00 00 0F 84 ?? ?? ?? ?? F2 0F 10 83 ?? ?? 00 00 48 8D 54 24 ?? 8B 83 ?? ?? 00 00 41 8B ?? "
    "F2 0F 11 44 24 ?? 89 44 24 ?? E8 ?? ?? ?? ?? F3 0F 10 7C 24 ?? F3 0F 10 74 24 ?? 0F 28 C7 0F 28 D6 F3 0F 59 C7 "
    "F3 0F 59 D6 F3 0F 58 C2 E8 ?? ?? ?? ?? 0F 28 C8 F3 0F 10 44 24 ?? E8 ?? ?? ?? ?? F3 41 0F 11 84 36 ?? ?? 00 00 "
    "0F 28 CF 0F 28 C6 E8 ?? ?? ?? ?? F3 0F 10 35 ?? ?? ?? ?? F3 0F 58 C6 0F 28 CE E8 ?? ?? ?? ?? "
    "F3 41 0F 11 84 36 ?? ?? 00 00";
constexpr uint8_t kLegacyUnitAimCtlForward = 17, kLegacyUnitAimCtlForwardZ = 28, kLegacyUnitAimCtlPitch = 105,
                  kLegacyUnitAimCtlYaw = 146;
constexpr LegacyUnitAimSite kLegacyUnitAimSites[2] = {
    {"F2 0F 10 86 ?? ?? 00 00 F2 0F 11 03 8B 86 ?? ?? 00 00 89 43 08 83 7E 10 FF 0F 84 ?? ?? ?? ?? 0F B7 46 10 "
     "41 B9 01 00 00 00 48 8D 14 40 49 8B 04 2E 4C 8B 40 48 41 8B C1 41 8A 4C D0 03 D3 E0 A9 03 10 00 00 "
     "0F 84 ?? ?? ?? ?? 49 8B 44 D0 10 33 ED 48 8B 15 ?? ?? ?? ?? 0F B7 08 48 8B 05 ?? ?? ?? ?? 44 8B 44 C8 04 "
     "8B C5 45 85 C0 74 04 4A 8D 04 82 8B 88 ?? ?? 00 00 85 C9 75 05 48 8B CD EB 04 48 8D 0C 8A 48 0F BF 86 "
     "?? ?? 00 00 48 69 D0 ?? ?? 00 00 8B 04 0A C1 E8 ?? 41 84 C1 74 ?? 8B 4E 10 4C 8D 44 24 30 40 88 6C 24 28 "
     "BA ?? 00 00 00",
     4, 14, 144, 153},
    {"F2 0F 10 86 ?? ?? 00 00 41 83 CB FF F2 0F 11 03 8B 86 ?? ?? 00 00 89 43 08 44 39 5E 10 0F 84 ?? ?? ?? ?? "
     "0F B7 46 10 45 8D 53 02 4C 8D 04 40 49 8B 04 2E 4C 8B 48 48 41 8B C2 43 8A 4C C1 03 D3 E0 A9 03 20 00 00 "
     "0F 84 ?? ?? ?? ?? 0F B7 96 ?? ?? 00 00 66 41 3B D3 0F 84 ?? ?? ?? ?? 4B 8B 44 C1 10 33 ED 4C 8B 05 "
     "?? ?? ?? ?? 0F B7 08 48 8B 05 ?? ?? ?? ?? 44 8B 4C C8 04 8B C5 45 85 C9 74 04 4B 8D 04 88 8B 88 ?? ?? 00 00 "
     "85 C9 75 05 48 8B CD EB 04 49 8D 0C 88 48 0F BF C2 48 69 D0 ?? ?? 00 00 8B 04 0A C1 E8 ?? 41 84 C2 74 ?? "
     "8B 4E 10 4C 8D 44 24 30 45 8B CA 40 88 6C 24 28 BA ?? 00 00 00",
     4, 18, 159, 168},
};
constexpr int32_t kLegacyUnitAimSeatStride[2] = {static_cast<int32_t>(kHalo3VehicleSeatStride),
                                                 static_cast<int32_t>(kOdstVehicleSeatStride)};
// The occupant's aim, published to the input thread, is current for this long (the servo's feedback otherwise).
constexpr uint64_t kLegacyOccupantAimFreshMs = 100;

// THE RECORD'S EASED PITCH LIMITS, Halo 3 and ODST (seat_limit::ResolvePitchLimits; 2026-10-03). The accumulator eases
// two record fields toward the seat's pitch range every update and clamps the pitch to them; read after the update,
// they are a look-steered driver's pitch stops (that driver's pitch is never latched: the accumulator levels it).
// Each site is the clamp's exact bytes -- the eased low's store, the eased high's load and store, the pitch's
// add/max/store -- UNIQUE in the module and matched at its RVA; the six offsets are DECODED from it (`?? ?? 00 00`)
// and must agree (ResolvePitchLimits). Measured 2026-10-03 on the retail Steam modules (Halo 3 +0x190/+0x194 at
// 0xF86F1, ODST +0x344/+0x348 at 0x114381; the Store modules are byte-identical, MCC-EDITIONS-EVIDENCE).
// ADDR-HYGIENE: resolved (a unique pinned signature, offsets decoded and cross-checked) and guarded per read (finite,
// ordered, inside the pole: seat_limit::RecordPitchInterval). Not bound: a look-steered driver's reticle is never
// clamped in pitch, as before; logged once.
struct LegacyPitchLimitSite
{
    uint32_t rva;
    const char* pattern;
    uint8_t loStore, hiLoad, hiStore, pitchAdd, loMax, pitchStore;   // disp32 positions in the pattern
};
constexpr LegacyPitchLimitSite kLegacyPitchLimitSites[2] = {
    {0xF86F1,
     "F3 0F 11 8C 3B ?? ?? 00 00 F3 0F 10 8C 3B ?? ?? 00 00 F3 0F 5C C1 F3 0F 5F C2 44 0F 2F E0 77 04 41 0F 28 C4 "
     "F3 0F 58 C8 41 0F 28 D6 F3 0F 11 8C 3B ?? ?? 00 00 0F 28 C1 F3 0F 58 94 3B ?? ?? 00 00 F3 0F 5F 94 3B ?? ?? 00 00 "
     "0F 2F C2 77 03 0F 28 D1 45 0F 2E F3 F3 0F 11 94 3B ?? ?? 00 00",
     5, 14, 49, 61, 70, 91},
    {0x114381,
     "F3 0F 11 8C 1F ?? ?? 00 00 F3 0F 10 8C 1F ?? ?? 00 00 F3 0F 5C C1 F3 0F 5F C2 0F 2F D8 77 03 0F 28 C3 "
     "F3 0F 58 C8 41 0F 28 D4 F3 0F 11 8C 1F ?? ?? 00 00 0F 28 C1 F3 0F 58 94 1F ?? ?? 00 00 F3 0F 5F 94 1F ?? ?? 00 00 "
     "0F 2F C2 77 03 0F 28 D1 45 0F 2E E3 F3 0F 11 94 1F ?? ?? 00 00",
     5, 14, 47, 59, 68, 89},
};

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
    std::atomic<bool> setGunSlaved{false};   // seated: the seat's first-person camera is the gun (flag bit 7)
    std::atomic<uint32_t> setAxes{0};        // seated: bit 0 yaw, bit 1 pitch -- the axes the hand owns there
    std::atomic<bool> setPitchLatch{true};   // seated: the latch may read pitch limits (not a look-steered driver's)
    std::atomic<uint32_t> setSeatKey{0};     // seated: which seat (seat_drive::SeatKey)
    // Seated writes (sim thread): the seat-limit latch, this stint's per-axis check, and when each axis was
    // last written / last held short (read by the input thread).
    aim_write::LimitLatch seatLatch{};
    uint32_t axisTicks[2]{};                 // per axis: consecutive seat writes of it (capped at the prime)
    seat_drive::StintCheck stint{};
    // Gun-slaved seats: the same check against the occupant's own aim vector (see the header). `unitAimOffset` is
    // the vector's offset in the unit, bound at install (0: not bound -- the camera check alone, as before).
    seat_drive::StintCheck stintOccupant{};
    bool stintOccupantActive = false;
    uint32_t unitAimOffset = 0;
    std::atomic<uint32_t> seatOccYawChecks{0}, seatOccPitchChecks{0}, seatOccUnread{0}, seatOccUpdates{0};
    // The outcome guard per axis (gun_slaved::OffsetWatch; [0] yaw wraps): an axis written on the occupant's proof
    // alone loses that proof for the stint if the gun comes to rest off the record.
    gun_slaved::OffsetWatch occWatch[2]{{0.0524f, 0.00175f, 30, true}, {}};
    std::atomic<uint32_t> seatOccRevoked[2]{};
    // The occupant's aim for the input thread (seqlock): WORLD angles, the seat it was read in, and how many judged
    // samples in a row it has agreed with the record per axis (low 16 bits yaw, high 16 pitch).
    std::atomic<uint32_t> occSeq{0};
    std::atomic<float> occYaw{0.0f}, occPitch{0.0f};
    std::atomic<uint64_t> occMs{0};
    std::atomic<uint32_t> occKey{0}, occAgree{0};
#if HALOMCCVR_DEV_DIAGNOSTICS
    // Gun-slaved seats, per report window (deg as float bits): the largest |record - occupant's aim| and
    // |record - gun| this stint's updates saw, in the record's frame.
    std::atomic<uint32_t> occRecordMaxBits{0}, gunRecordMaxBits{0};
    // ...and the input thread's turret witnesses (LegacyDriveTurretWitness, LegacyDriveTurretServo): polls, the
    // longest gun settle (ms), the peak gun slew (deg/s), the gun's and the hand's pitch reversals and largest swings
    // (deg), the servo's feedback per axis (occupant / gun) and the pacer's held polls.
    std::atomic<uint32_t> turretPolls{0}, turretSettleMaxMs{0}, turretSettles{0}, turretPeakSlewBits{0};
    std::atomic<uint32_t> turretGunReversals{0}, turretGunSwingBits{0}, turretHandReversals{0},
        turretHandSwingBits{0};
    std::atomic<uint32_t> turretServoOccupant[2]{}, turretServoGun[2]{}, turretLagHolds[2]{};
#endif
    // The record's eased pitch limits (kLegacyPitchLimitSites): the low's offset, bound at install (0: not bound), and
    // how often a look-steered driver's write published them as its pitch stops (and how often a read was refused).
    uint32_t pitchLimitOffset = 0;
    std::atomic<uint32_t> pitchLimitPublished{0}, pitchLimitRefused{0};
    bool stintActive = false;
    uint32_t stintKey = 0;
    bool stintPitchFailCounted = false;
    std::atomic<uint64_t> seatYawWriteMs{0}, seatPitchWriteMs{0}, seatLimitedMs{0};
    // THE SEAT'S STOPS for the reticle (seat_limit_publication.h): after each seat write, the latch's stops on the axes
    // it read, in the record's frame, with that frame. Cleared by every update that did not end in a seat write.
    SeatStopsPublication stops{};
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
    // Writes whose branch-matched yaw fell outside [0, 2*pi) and were wrapped back into it
    // (aim_write::wrap_0_2pi). Before 2026-10-01 each of those reached the game's action packer unwrapped: in
    // Halo 3 / ODST it overflowed the 13-bit yaw into the button field and read as an action-button press
    // (seat exits); in Halo 4 the packer clamped it to 0 or 2*pi (a one-update aim nudge).
    std::atomic<uint32_t> yawWrapped{0};
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
    // The last two seat writes (seqlock), for the input thread's dev outcome witness (it switched the reticle
    // to the gun until 2026-10-01): the game's aim lags the latest write by up to a tick, so the witness
    // measures it against whichever of the two is nearer.
    std::atomic<uint32_t> wroteSeq{0};
    std::atomic<float> wroteYaw[2]{}, wrotePitch[2]{};
    std::atomic<bool> wroteValid[2]{};
    std::atomic<float> nativeYaw{0.0f}, nativePitch{0.0f};
    // The seat's frame (seat_frame_reader.h): bound at install with the hook, read on the sim thread each seated
    // update; its per-case counters feed the seats line (and, dev, the frame check).
    seat_frame_rt::Binding frame{};
    seat_frame_rt::Counters frameCounters{};
};
LegacyDriveState g_legacyDrive[3];

constexpr const seat_frame::ReaderSite* kLegacyFrameSites[3]{&seat_frame::kHalo3Reader, &seat_frame::kOdstReader,
                                                             &seat_frame::kHalo4Reader};

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
    st.stintOccupant.reset();
    st.stintOccupantActive = false;
    st.occWatch[0].reset();
    st.occWatch[1].reset();
    // The input thread's servo stops closing on this stint's occupant at once.
    st.occMs.store(0, std::memory_order_release);
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
    st.stops.clear();
    // ...and its writes are no reference for the next stint's outcome gate.
    st.wroteSeq.fetch_add(1, std::memory_order_acq_rel);
    st.wroteValid[0].store(false, std::memory_order_relaxed);
    st.wroteValid[1].store(false, std::memory_order_relaxed);
    st.wroteSeq.fetch_add(1, std::memory_order_release);
    st.stintSampleMs = 0;
}

// This stint's per-axis verdict: the camera's check, or a gun-slaved seat's occupant check where its outcome guard
// has not revoked it (seat_drive::CombineStints).
seat_drive::StintVerdict LegacyDriveVerdict(const LegacyDriveState& st)
{
    return seat_drive::CombineStints(st.stint, st.stintOccupant, st.stintOccupantActive, st.occWatch[0].revoked,
                                     st.occWatch[1].revoked);
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

// Sim thread: the local player's unit -- the one the title's rebase will rotate this tick. Halo 3 / ODST: the
// player-unit getter (output user 0), as LegacyDriveLocalOnFoot reads it; Halo 4: the record's own unit (+0x120,
// proven at install). kNone when unknown (the seat frame then reads Unknown: no seated write).
uint32_t LegacyDriveSeatUnit(int slot, const LegacyDriveState& st, uintptr_t pc)
{
    __try
    {
        if (slot == 0)
            return g_halo3PlayerUnitGetter ? static_cast<uint32_t>(g_halo3PlayerUnitGetter(0)) : seat_frame::kNone;
        if (slot == 1)
            return g_odstPlayerUnitGetter ? static_cast<uint32_t>(g_odstPlayerUnitGetter(0)) : seat_frame::kNone;
        if (slot == 2 && st.halo4RecordUnit && pc)
            return *reinterpret_cast<const uint32_t*>(pc + kHalo4RecordUnitOffset);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
    }
    return seat_frame::kNone;
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

// The unique match of `pattern` anywhere in the module, or 0 (none, or more than one).
uintptr_t LegacyDriveUniqueAnywhere(uintptr_t base, size_t size, const char* pattern)
{
    const uintptr_t first = sig::Find(base, size, pattern);
    if (!first)
        return 0;
    const uintptr_t next = first + 1;
    if (next < base + size && sig::Find(next, static_cast<size_t>(base + size - next), pattern))
        return 0;
    return first;
}

// Install thread: bind the occupant's aim vector (kLegacyUnitAimSites). Returns the offset, 0 when not bound; `why`
// names the first proof that failed and `cameraRva` / `controlRva` where the two consumers were found.
uint32_t LegacyDriveBindUnitAim(int slot, uintptr_t base, size_t size, const char*& why, uint32_t& cameraRva,
                                uint32_t& controlRva)
{
    why = nullptr;
    cameraRva = controlRva = 0;
    if (slot < 0 || slot > 1)
    {
        why = "no binding for this title";
        return 0;
    }
    const auto& site = kLegacyUnitAimSites[slot];
    const uintptr_t camera = LegacyDriveUniqueAnywhere(base, size, site.camera);
    const uintptr_t control = LegacyDriveUniqueAnywhere(base, size, kLegacyUnitAimControl);
    if (!camera)
    {
        why = "the camera function's forward read is missing or ambiguous";
        return 0;
    }
    if (!control)
    {
        why = "the control update's reset from the vector is missing or ambiguous";
        return 0;
    }
    cameraRva = static_cast<uint32_t>(camera - base);
    controlRva = static_cast<uint32_t>(control - base);
    const auto i32 = [](uintptr_t at) {
        int32_t v = 0;
        memcpy(&v, reinterpret_cast<const void*>(at), sizeof(v));
        return v;
    };
    gun_slaved::UnitAimSites s{};
    s.cameraForward = i32(camera + site.camForward);
    s.cameraForwardZ = i32(camera + site.camForwardZ);
    s.cameraSeatStride = i32(camera + site.camStride);
    s.cameraFlagShift = *reinterpret_cast<const uint8_t*>(camera + site.camShift);
    s.controlForward = i32(control + kLegacyUnitAimCtlForward);
    s.controlForwardZ = i32(control + kLegacyUnitAimCtlForwardZ);
    s.controlPitchWrite = i32(control + kLegacyUnitAimCtlPitch);
    s.controlYawWrite = i32(control + kLegacyUnitAimCtlYaw);
    return gun_slaved::ResolveUnitAimOffset(s, kLegacyUnitAimSeatStride[slot],
                                            static_cast<int32_t>(kLegacyDriveSites[slot].angles), why);
}

// Sim thread: the occupant's aim vector as WORLD angles, read from `unit` through the seat frame binding's own
// salt-checked object walk (the same table the rebase reads). False when unbound, unknown or not a unit normal.
bool LegacyDriveReadOccupantAim(const LegacyDriveState& st, uint32_t unit, float& yaw, float& pitch)
{
    if (!st.unitAimOffset || unit == seat_frame::kNone || !st.frame.ready.load(std::memory_order_acquire))
        return false;
    float v[3]{};
    bool copied = false;
    __try
    {
        const seat_frame_rt::RawMem mem{&st.frame, nullptr};
        const uintptr_t data = mem.Object(unit, 0);
        if (data)
        {
            memcpy(v, reinterpret_cast<const void*>(data + st.unitAimOffset), sizeof(v));
            copied = true;
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        copied = false;
    }
    return copied && gun_slaved::AnglesFromVector(v, yaw, pitch);
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

// Install thread: bind the record's eased pitch limits (kLegacyPitchLimitSites). Returns the low's offset, 0 when not
// bound (`why` names the first proof that failed).
uint32_t LegacyDriveBindPitchLimits(int slot, uintptr_t base, size_t size, const char*& why)
{
    why = nullptr;
    if (slot < 0 || slot > 1)
    {
        why = "no binding for this title";
        return 0;
    }
    const auto& site = kLegacyPitchLimitSites[slot];
    if (!LegacyDriveUniqueAt(base, size, site.rva, site.pattern))
    {
        why = "the accumulator's pitch clamp is missing, moved or ambiguous";
        return 0;
    }
    const auto i32 = [&](uint8_t at) {
        int32_t v = 0;
        memcpy(&v, reinterpret_cast<const void*>(base + site.rva + at), sizeof(v));
        return v;
    };
    seat_limit::PitchLimitSite s{};
    s.loStore = i32(site.loStore);
    s.hiLoad = i32(site.hiLoad);
    s.hiStore = i32(site.hiStore);
    s.pitchAdd = i32(site.pitchAdd);
    s.loMax = i32(site.loMax);
    s.pitchStore = i32(site.pitchStore);
    return seat_limit::ResolvePitchLimits(s, static_cast<int32_t>(kLegacyDriveSites[slot].angles),
                                          static_cast<int32_t>(kLegacyDriveSites[slot].stride), why);
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
    // legacy_direct_drive (derived from aim_direct_drive) is on, and disarms (to re-validate) whenever it is off.
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
    st.census.restart(callers.dispatcher_mode());   // pass order is measured where the extra calls lead
    st.tickSlot = tickOk ? tickSlot : 0;
    st.tickField = tickOk ? tickField : 0;
#endif
    st.base = base;
    st.halo4RecordUnit = recordUnit;
    st.tlsIndexAddress = indexAddress;
    // The seat's frame: what the title's vehicle-tilt rebase reads, proven here (seat_frame_reader.h). Unbound,
    // every seated update reads Unknown and writes nothing (the stick servo keeps every seat).
    const char* frameWhy = nullptr;
    const bool frameBound = seat_frame_rt::Bind(st.frame, *kLegacyFrameSites[slot], base, size, frameWhy);
    st.frameCounters.unknownLogged.store(0, std::memory_order_relaxed);
    // Gun-slaved seats: the occupant's aim vector (Halo 3 and ODST; Halo 4's slaved turrets are its own lane).
    const char* unitAimWhy = nullptr;
    uint32_t unitAimCameraRva = 0, unitAimControlRva = 0;
    st.unitAimOffset = slot < 2 ? LegacyDriveBindUnitAim(slot, base, size, unitAimWhy, unitAimCameraRva,
                                                         unitAimControlRva)
                                : 0;
    // A look-steered driver's pitch stops: the record's eased limits (Halo 3 and ODST).
    const char* pitchLimitWhy = nullptr;
    st.pitchLimitOffset = slot < 2 ? LegacyDriveBindPitchLimits(slot, base, size, pitchLimitWhy) : 0;
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
    if (frameBound)
        LOG("DRIVE %s: seat frame reader bound (the rebase +0x%X: its gate +0x%X and up getter +0x%X proven): a seated "
            "write follows the vehicle-tilt rebase (world / a / b)", site.name, kLegacyFrameSites[slot]->rebaseRva,
            kLegacyFrameSites[slot]->gateRva, kLegacyFrameSites[slot]->getterRva);
    else
        LOG("DRIVE %s: seat frame reader NOT bound (%s at +0x%X); seats are never written (the stick servo keeps "
            "every seat)", site.name, frameWhy ? frameWhy : "unknown", kLegacyFrameSites[slot]->gateRva);
    if (slot < 2 && st.unitAimOffset)
        LOG("DRIVE %s: the occupant's aim vector is unit +0x%X (the first-person camera's read +0x%X and the control "
            "update's reset +0x%X agree): a gun-slaved seat (flag bit 7, a turret whose camera is its gun) proves its "
            "record against it as well as the gun, and its stick servo closes on it once it agrees", site.name,
            st.unitAimOffset, unitAimCameraRva, unitAimControlRva);
    else if (slot < 2)
        LOG("DRIVE %s: the occupant's aim vector is NOT bound (%s); a gun-slaved seat proves against the gun's "
            "camera alone, as before", site.name, unitAimWhy ? unitAimWhy : "unknown");
    if (slot < 2 && st.pitchLimitOffset)
        LOG("DRIVE %s: the record's eased pitch limits are +0x%X / +0x%X (the accumulator's clamp at +0x%X stores, reads "
            "and bounds the pitch +0x%X by them): a look-steered driver's reticle stops at them",
            site.name, st.pitchLimitOffset, st.pitchLimitOffset + 4, kLegacyPitchLimitSites[slot].rva, site.angles + 4);
    else if (slot < 2)
        LOG("DRIVE %s: the record's eased pitch limits are NOT bound (%s at +0x%X); a look-steered driver's reticle is "
            "not clamped in pitch, as before", site.name, pitchLimitWhy ? pitchLimitWhy : "unknown",
            kLegacyPitchLimitSites[slot].rva);
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
    seat_frame_rt::Unbind(st.frame);
    return true;
}

// Input thread: the servo's target (the converged aim when it resolves) and the game's own aim. `seated`:
// the local player is in a seat; `axes` (seat_drive::Decide) are the axes the hand owns there -- none keeps
// the seat on the servo, exactly as before; `pitchLatch`: the seat-limit latch may read pitch limits;
// `seatKey`: which seat, so a check passed in one seat never carries into the next. `gunSlaved`: the seat's
// first-person camera is its gun (seat flag bit 7), so `aim` is the gun and the stint also proves the record against
// the occupant's own aim.
void LegacyDrivePublish(GameTitle title, float yaw, float pitch, float aimYaw, float aimPitch, bool valid,
                        bool seated, seat_drive::Axes axes, bool pitchLatch, uint32_t seatKey, bool gunSlaved)
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
    st.setGunSlaved.store(seated && gunSlaved, std::memory_order_relaxed);
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

// Input thread: the hook's last two seat writes, newest first, as the WORLD angles each put into effect (R e(record)
// in case b, the record itself otherwise; an axis the write did not touch is the record's own). False when torn or
// none.
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

// Input thread, a gun-slaved seat (Halo 3 / ODST): the occupant's own aim in WORLD angles, as the hook last read it
// for seat `seatKey`, and how many judged samples in a row it has agreed with the record per axis ([0] yaw, [1]
// pitch). False when unbound, torn, stale (kLegacyOccupantAimFreshMs) or another seat's: the servo then closes on the
// camera's forward, as before.
bool LegacyDriveOccupantAim(GameTitle title, uint32_t seatKey, float& yaw, float& pitch, unsigned agreeing[2])
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0 || slot > 1 || !seatKey)
        return false;
    const auto& st = g_legacyDrive[slot];
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const uint32_t before = st.occSeq.load(std::memory_order_acquire);
        if (before & 1u)
            continue;
        const uint64_t at = st.occMs.load(std::memory_order_relaxed);
        const uint32_t key = st.occKey.load(std::memory_order_relaxed);
        const uint32_t agree = st.occAgree.load(std::memory_order_relaxed);
        const float y = st.occYaw.load(std::memory_order_relaxed);
        const float p = st.occPitch.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        if (st.occSeq.load(std::memory_order_relaxed) != before)
            continue;
        const uint64_t now = GetTickCount64();
        if (!at || now < at || now - at > kLegacyOccupantAimFreshMs || key != seatKey || !std::isfinite(y) ||
            !std::isfinite(p))
            return false;
        yaw = y;
        pitch = p;
        agreeing[0] = agree & 0xFFFFu;
        agreeing[1] = agree >> 16;
        return true;
    }
    return false;
}

#if HALOMCCVR_DEV_DIAGNOSTICS
// Input thread, dev, every poll of a gun-slaved seat: the gun's settle episodes on the writes (`settle`, stepped by
// the caller), its slew, and the pitch reversals of the gun against the hand's target -- a gun that reverses where the
// hand does not is the loop's own nod. For the 2 s turret line (LegacyDriveReport).
void LegacyDriveTurretWitness(GameTitle title, float gunYaw, float gunPitch, float handPitch, gun_slaved::GunSettle& settle)
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0 || slot > 1)
        return;
    auto& st = g_legacyDrive[slot];
    static gun_slaved::SlewRate s_slew[2];
    static gun_slaved::NodWitness s_gunNod[2], s_handNod[2];
    const uint64_t now = GetTickCount64();
    st.turretPolls.fetch_add(1, std::memory_order_relaxed);
    if (settle.episodeDone)
    {
        settle.episodeDone = false;
        st.turretSettles.fetch_add(1, std::memory_order_relaxed);
        const uint32_t ms = static_cast<uint32_t>((std::min)(settle.lastSettleMs, uint64_t{0xFFFFFFFFu}));
        uint32_t cur = st.turretSettleMaxMs.load(std::memory_order_relaxed);
        while (ms > cur && !st.turretSettleMaxMs.compare_exchange_weak(cur, ms, std::memory_order_relaxed)) {}
    }
    aim_caller::AtomicMaxFloat(st.turretPeakSlewBits, s_slew[slot].step(gunYaw, gunPitch, now));
    s_gunNod[slot].step(gunPitch);
    s_handNod[slot].step(handPitch);
    unsigned reversals = 0;
    float swing = 0.0f;
    s_gunNod[slot].take(reversals, swing);
    st.turretGunReversals.fetch_add(reversals, std::memory_order_relaxed);
    aim_caller::AtomicMaxFloat(st.turretGunSwingBits, swing * gun_slaved::kRadToDeg);
    s_handNod[slot].take(reversals, swing);
    st.turretHandReversals.fetch_add(reversals, std::memory_order_relaxed);
    aim_caller::AtomicMaxFloat(st.turretHandSwingBits, swing * gun_slaved::kRadToDeg);
}

// Input thread, dev: what a gun-slaved seat's stick servo closed on this poll per axis (`feedback`: 1 the occupant,
// 0 the gun, -1 not steered) and the pacer's held polls since the last call.
void LegacyDriveTurretServo(GameTitle title, const int feedback[2], const unsigned held[2])
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0 || slot > 1)
        return;
    auto& st = g_legacyDrive[slot];
    for (int a = 0; a < 2; ++a)
    {
        if (feedback[a] == 1)
            st.turretServoOccupant[a].fetch_add(1, std::memory_order_relaxed);
        else if (feedback[a] == 0)
            st.turretServoGun[a].fetch_add(1, std::memory_order_relaxed);
        st.turretLagHolds[a].fetch_add(held[a], std::memory_order_relaxed);
    }
}
#endif

// Input thread: the stops the hook's last seat write knew (the seat-limit latch's, record frame, with the frame), when
// that write is under 100 ms old -- the seat limit reticle's source (seat_limit_reticle_logic.h). `seatKey`: the seat
// they belong to; `recordYaw`: the record's yaw then. False: no fresh seat write (the reticle is not clamped from it).
bool LegacyDriveSeatStops(GameTitle title, seat_limit::Stops& out, uint32_t& seatKey, float& recordYaw)
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0 || !g_legacyDrive[slot].armed.load(std::memory_order_acquire))
        return false;
    return g_legacyDrive[slot].stops.read(GetTickCount64(), 100, out, seatKey, recordYaw);
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
    bool valid = false, seated = false, pitchLatch = true, coherent = false, gunSlaved = false;
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
        p.gunSlaved = st.setGunSlaved.load(std::memory_order_relaxed);
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
// Any thread (the dev Hornet capture): the record as the last stick call found it and the setpoint it last read.
bool LegacyDriveRecordSnapshot(GameTitle title, float& nativeYaw, float& nativePitch, float& setYaw, float& setPitch)
{
    const int slot = LegacyDriveSlot(title);
    if (slot < 0)
        return false;
    const auto& st = g_legacyDrive[slot];
    nativeYaw = st.nativeYaw.load(std::memory_order_relaxed);
    nativePitch = st.nativePitch.load(std::memory_order_relaxed);
    setYaw = st.yaw.load(std::memory_order_relaxed);
    setPitch = st.pitch.load(std::memory_order_relaxed);
    return st.installed.load(std::memory_order_acquire) && std::isfinite(nativeYaw) && std::isfinite(setYaw);
}

// THE RECORD THROUGH THE INTENT POINT (Halo 3 on foot; shot_lag_logic.h, 2026-10-02). The shot itself is re-aimed
// at the firing assist's crosshair query (legacy_sight_trace.inl). The RECORD -- the angles the HUD, autoaim, melee
// and the network read -- was solved on the input thread from the render-time eye, so it still points the line
// from the tick's eye beside the point by the eye's travel. At the stick call it is recomposed as the angles of
// normalize(P - E): P the published intent point, E the game's OWN eye -- Halo 3's control pass asks the unit's
// camera evaluator for it (the acquisition, 0x13B08C at 0xF4CC8, in the same pass and just before this stick call
// at 0xF6D11: both inside 0xF66AC, read from the pinned module) -- carried one measured simulation tick ahead,
// because the record written now is first consumed by the next tick. Never P rebuilt from the current eye: that
// would recreate the error. ONLY FROM OUR UNIT'S FIRST-PERSON EYE (shot_lag_logic.h 4d, 2026-10-03): our unit on
// foot in control and the director's camera for output user 0 the first-person camera -- in Forge the Monitor's
// own and orbit cameras bent the record by up to 88 deg and flung the held object. Kept as published (fail closed)
// when the setpoint is not the solve that built P (the fixed-distance steer, a seat author), the point is stale,
// seated or held at the pitch rail, the eye is not a first-person eye, the game's eye is not from this tick (Halo
// 3's dual-aim hooks absent, or none published since the previous stick call), or P is within 30 cm of that eye or
// behind it (no magnitude limit: shot_lag::Recompose). When the game skipped its acquisition this tick, the eye is
// asked of the same evaluator directly (Halo3TickEyeFromEvaluator; shot_lag::TickEyeMiss). Yaw is atan2's like the
// solve's: the wrap into [0, 2*pi) below applies to it as to every write. The decision is shot_lag::RecomposeRecord
// (pure, unit-tested); this gathers its inputs.
using LegacyRecomposeWhy = shot_lag::Recompose;
std::atomic<uint32_t> g_legacyRecompose[static_cast<size_t>(LegacyRecomposeWhy::Count)]{};
std::atomic<uint32_t> g_legacyRecomposeMilliDeg{0}, g_legacyRecomposeMaxMilliDeg{0};
// The record's game eye when the acquisition published none (shot_lag::TickEyeMiss / TickEyeAcquisition), for
// the SHOT RECORD line.
std::atomic<uint32_t> g_tickEyeMiss[static_cast<size_t>(shot_lag::TickEyeMiss::Count)]{};
std::atomic<uint32_t> g_tickEyeAcquisition[static_cast<size_t>(shot_lag::TickEyeAcquisition::Count)]{};
// The first-person eye identity's verdicts at the stick calls that had a point to recompose through (4d), and the
// director's camera mode each time it said "not first person" (the last slot: the first-person camera with another
// perspective), for the SHOT RECORD first-person line.
std::atomic<uint32_t> g_firstPersonEye[static_cast<size_t>(shot_lag::FirstPersonEye::Count)]{};
std::atomic<uint32_t> g_firstPersonEyeMode[shot_lag::kHalo3CameraModeCount + 1]{};

// THE FIRST-PERSON EYE (sim thread, the stick call; shot_lag_logic.h 4d). Our unit, on foot (no seat word, the
// vehicle sampler on foot) and player-controlled, and the director's camera for output user 0 the first-person
// camera from the first-person perspective (ReadHalo3DirectorCamera, game.cpp). `unitOut` is the unit read; `modeOut`
// the director's mode when read (-1 otherwise).
shot_lag::FirstPersonEye Halo3FirstPersonEyeNow(uint32_t generation, int32_t& unitOut, int32_t& modeOut,
                                                int32_t& perspectiveOut)
{
    shot_lag::FirstPersonEyeInput in{};
    unitOut = -1;
    __try
    {
        unitOut = g_halo3PlayerUnitGetter ? g_halo3PlayerUnitGetter(0) : -1;
        int32_t scene = -1, shot = -1;
        const uint8_t* unit = unitOut != -1 ? Halo3MeleeSelectionObject(static_cast<uint32_t>(unitOut), 0) : nullptr;
        in.onFootControlled = unit &&
            *reinterpret_cast<const int16_t*>(unit + kHalo3UnitSeatWordOffset) == -1 &&
            Game_Halo3VehicleState().state == Halo3VehicleState::OnFoot &&
            ReadCinematicControl(scene, shot) == CinematicControlState::PlayerControlled;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        in.onFootControlled = false;
    }
    in.directorRead = ReadHalo3DirectorCamera(generation, in.mode, in.perspective);
    modeOut = in.directorRead ? in.mode : -1;
    perspectiveOut = in.directorRead ? in.perspective : -1;
    return shot_lag::ClassifyFirstPersonEye(in);
}

// THE GAME'S EYE, ASKED DIRECTLY (sim thread, the stick call; shot_lag_logic.h 4c). Halo 3 skips its aim
// acquisition -- and with it the evaluator call the eye is normally published from -- by its own branch (0xF48C9).
// The evaluator itself (0x212198, signature-bound by the dual-aim lane, InstallHalo3DualAim; called through its
// trampoline, so our camera detour does not run) is the same eye the acquisition and the firing assist ask for.
// Asked only for a first-person eye (4d), and never when the acquisition ran and declined the unit (an unarmed
// Monitor: the game itself does not aim it). Its answer is published only when it took the first-person path
// (0x17DAEC, whose chain writes nothing but its out-parameters: read offline two calls deep). The lane's callback
// count brackets the call so RemoveHalo3DualAim's quiescence wait covers it. Returns the reason.
shot_lag::TickEyeMiss Halo3TickEyeFromEvaluator(uint32_t generation, int32_t unit, bool firstPersonEye,
                                                shot_lag::TickEyeAcquisition acquisition)
{
    auto& feature = g_halo3Dual;
    feature.callbacks.fetch_add(1, std::memory_order_acq_rel);
    shot_lag::TickEyeMiss why = shot_lag::TickEyeMiss::NoHook;
    __try
    {
        shot_lag::TickEyeFallbackInput in{};
        const Halo3DualCameraFn evaluator = feature.cameraOriginal;
        in.hooked = evaluator && feature.enabled.load(std::memory_order_acquire) &&
            !feature.faulted.load(std::memory_order_acquire);
        in.sameGeneration = generation && feature.generation == generation;
        in.unit = unit;
        in.firstPersonEye = firstPersonEye;
        in.acquisition = acquisition;
        why = shot_lag::TickEyeFallbackGate(in);
        if (why == shot_lag::TickEyeMiss::FromEvaluator)
        {
            float position[3]{}, direction[3]{};
            const int32_t path = evaluator(static_cast<uint32_t>(in.unit), position, direction);
            if (!shot_lag::EvaluatorTookFirstPersonPath(path))
                why = shot_lag::TickEyeMiss::ThirdPersonPath;
            else if (shot_lag::Finite3(position))
                PublishHalo3TickEye(generation, in.unit, position);
            else
                why = shot_lag::TickEyeMiss::EvaluatorFault;
        }
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        why = shot_lag::TickEyeMiss::EvaluatorFault;
    }
    feature.callbacks.fetch_sub(1, std::memory_order_acq_rel);
    return why;
}

LegacyRecomposeWhy LegacyDriveRecomposeOnFoot(int slot, const LegacyDrivePublication& pub, float& yaw, float& pitch)
{
    // Halo 3's stick calls, one per tick: an eye is this tick's only when published since the previous one.
    static uint64_t s_lastStickUs = 0;
    uint64_t previousStickUs = 0;
    const uint64_t nowUs = Sight_NowUs();
    if (slot == 0)
    {
        previousStickUs = s_lastStickUs;
        s_lastStickUs = nowUs;
    }
    shot_lag::RecomposeInput in{};
    in.onFoot = slot == 0 && !pub.seated && pub.valid && Sight_AimEnabled();
    if (!in.onFoot)
        return LegacyRecomposeWhy::NotOnFoot;   // nothing read
    SightIntent intent{};
    in.intentRead = Sight_ReadIntent(GameTitle::Halo3, intent);
    in.intent = intent.p;
    in.setYaw = pub.yaw;
    in.setPitch = pub.pitch;
    in.generation = TitleAdapter_GetGeneration(GameTitle::Halo3);
    in.nowUs = nowUs;
    in.maxAgeUs = kSightIntentShotMaxAgeMs * 1000ull;
    // IDENTITY FIRST: whose eye, from which camera. Nothing below asks the game for an eye that is not ours.
    int32_t unit = -1, mode = -1, perspective = -1;
    const shot_lag::FirstPersonEye identity = in.generation
        ? Halo3FirstPersonEyeNow(in.generation, unit, mode, perspective)
        : shot_lag::FirstPersonEye::NotOnFootControlled;
    in.firstPersonEye = identity == shot_lag::FirstPersonEye::Yes;
    if (in.intentRead)
    {
        g_firstPersonEye[static_cast<size_t>(identity)].fetch_add(1, std::memory_order_relaxed);
        if (identity == shot_lag::FirstPersonEye::NotFirstPersonCamera)
            g_firstPersonEyeMode[mode == shot_lag::kHalo3CameraModeFirstPerson ? shot_lag::kHalo3CameraModeCount
                                                                               : mode]
                .fetch_add(1, std::memory_order_relaxed);
    }
    uint64_t stampUs = 0;
    const bool read = in.generation &&
        ReadHalo3TickEye(in.generation, 100000, in.eye, in.eyeVel, in.eyeVelValid, stampUs);
    in.eyeRead = shot_lag::TickEyeFromThisTick(read, stampUs, previousStickUs);
    if (in.generation && !in.eyeRead && in.intentRead)
    {
        // No eye from this tick's acquisition: count why, then ask the evaluator directly if the eye is ours.
        const uint64_t sinceStickUs = previousStickUs && nowUs > previousStickUs ? nowUs - previousStickUs : 100000;
        const auto acquisition = shot_lag::ClassifyTickEyeAcquisition(
            g_halo3AcquisitionUser0Us.load(std::memory_order_relaxed), nowUs, sinceStickUs);
        g_tickEyeAcquisition[static_cast<size_t>(acquisition)].fetch_add(1, std::memory_order_relaxed);
        const shot_lag::TickEyeMiss miss = Halo3TickEyeFromEvaluator(in.generation, unit, in.firstPersonEye,
                                                                     acquisition);
        const size_t m = static_cast<size_t>(miss);
        if (m < static_cast<size_t>(shot_lag::TickEyeMiss::Count))
            g_tickEyeMiss[m].fetch_add(1, std::memory_order_relaxed);
        if (miss == shot_lag::TickEyeMiss::FromEvaluator)
        {
            const bool asked =
                ReadHalo3TickEye(in.generation, 100000, in.eye, in.eyeVel, in.eyeVelValid, stampUs);
            in.eyeRead = shot_lag::TickEyeFromThisTick(asked, stampUs, previousStickUs);
        }
    }
    in.eyeProven = g_halo3TickEyeProof.load(std::memory_order_relaxed) == static_cast<uint32_t>(TickEyeProof::Proven);
    in.tickS = float(g_legacyTickPeriodUs[0].load(std::memory_order_relaxed)) * 1.0e-6f;
    // Flat parity (sight_layer.h Sight_TransportWanted; every build, the dev key shot_point_transport the A/B): the
    // record through P moved with the eye, so the HUD, autoaim, the crosshair query and a multiplayer host firing from
    // this record agree with the shot sites that move it too. Its E' is this tick's eye carried one tick (the eye the
    // shot leaves from after the tick's move); from E' the line through P' is parallel to the solved one, as flat fires.
    in.transport = Sight_TransportWanted(GameTitle::Halo3, false);
    float bendDeg = 0.0f;
    shot_lag::Transport transported = shot_lag::Transport::Count;
    float shift[3]{};
    const LegacyRecomposeWhy why = shot_lag::RecomposeRecord(in, yaw, pitch, bendDeg, &transported, shift);
    if (why != LegacyRecomposeWhy::Recomposed)
        return why;
    if (in.transport)
        Sight_NoteTransport(GameTitle::Halo3, SightTransportSite::Record, transported, shot_lag::FireEye::CarriedTick,
                            shift, intent.p.dir, intent.p.eyeVelValid ? intent.p.eyeVel : nullptr,
                            nowUs >= stampUs ? float(nowUs - stampUs) / 1000.0f : -1.0f,
                            intent.p.pointEyeStampUs && nowUs >= intent.p.pointEyeStampUs
                                ? float(nowUs - intent.p.pointEyeStampUs) / 1000.0f : -1.0f);
    const uint32_t milli = static_cast<uint32_t>(bendDeg * 1000.0f);
    g_legacyRecomposeMilliDeg.fetch_add(milli, std::memory_order_relaxed);
    uint32_t prior = g_legacyRecomposeMaxMilliDeg.load(std::memory_order_relaxed);
    while (milli > prior && !g_legacyRecomposeMaxMilliDeg.compare_exchange_weak(prior, milli,
                                                                               std::memory_order_relaxed)) {}
    return LegacyRecomposeWhy::Recomposed;
}

// THE SEATED RECORD UPSTREAM OF REPLICATION (Halo 3; src/common/seat_upstream_logic.h, 2026-10-03). In online play
// the record is what the host fires from -- with the stock game, if it runs no mod -- so a correction made only at
// the fire site (the crosshair query re-aimed through P, legacy_sight_trace.inl) is this machine's prediction and
// nothing more. Here, at the stick call, a seat's setpoint is replaced by the record whose STOCK crosshair line at the
// next tick passes through the seat's P (SightIntent::solvePoint): solved from this control pass's own camera (the
// acquisition's evaluator answer, carried one measured tick ahead) through the seat's learned velocity term (the stock
// query runs along normalize(R - g v), g learned from the same evaluator's directions against the record that drove
// them). Kept as published, counted by reason, whenever any input is missing (seat_upstream::SeatRecompose); no
// magnitude limit. Gun-slaved seats are not modelled (their line is the gun's marker; GunOffsetStep owns them).
// THE HOST CAVEAT (read from the module, not testable here): the evaluator chooses the chase-camera path for a seat
// whose LOADED tag flags carry bit 4 (0x1326E8 -> 0x2121D2). C20 clears that bit on this machine; an unmodded host
// keeps it, so in a stock third-person seat its query starts at its chase camera and runs parallel to ours. The dev
// SHOT UPSTREAM line names such seats ("stock chase camera").
using LegacySeatRecomposeWhy = seat_upstream::SeatRecompose;
std::atomic<uint32_t> g_legacySeatRecompose[static_cast<size_t>(LegacySeatRecomposeWhy::Count)]{};
std::atomic<uint32_t> g_legacySeatRecomposeMilliDeg{0}, g_legacySeatRecomposeMaxMilliDeg{0};
// The gain learner's samples since the last report, and its state (sim thread writes; the report reads).
std::atomic<uint32_t> g_legacySeatGainUsed{0}, g_legacySeatGainSlow{0}, g_legacySeatGainUnpaired{0};
std::atomic<uint32_t> g_legacySeatCameraPath[3]{};   // this pass's camera: first person, chase, other/none
std::atomic<uint32_t> g_legacySeatStockCamera[2]{};  // seat_upstream_aim 2: the stock chase camera used / not available
std::atomic<float> g_legacySeatGain{0.0f};
std::atomic<bool> g_legacySeatGainLearned{false};
struct LegacySeatUpstream
{
    seat_upstream::GainFit fit;
    seat_upstream::WriteRing ring;
    uint32_t fitKey = 0;
    uint32_t generation = 0;
    uint64_t lastStickUs = 0;
};
LegacySeatUpstream g_legacySeatUpstream;   // the stick call's thread only

// The local player's unit (Halo 3), guarded; -1 when unknown.
int32_t LegacyHalo3LocalUnit()
{
    __try
    {
        return g_halo3PlayerUnitGetter ? g_halo3PlayerUnitGetter(0) : -1;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return -1;
    }
}

// Sim thread, Halo 3's stick call: what record the call leaves in effect (for the learner's pairing), after it ran.
// `dir` null: this call wrote nothing (the record is whatever the game or the servo made it).
void LegacySeatUpstreamNoteCall(uint32_t seatKey, const float* dir)
{
    g_legacySeatUpstream.ring.Note(g_legacySimTicks[0].load(std::memory_order_acquire), seatKey, dir);
}

LegacySeatRecomposeWhy LegacyDriveRecomposeSeated(int slot, const LegacyDrivePublication& pub, float& yaw,
                                                  float& pitch)
{
    auto& s = g_legacySeatUpstream;
    const uint64_t nowUs = Sight_NowUs();
    const uint64_t previousStickUs = s.lastStickUs;
    s.lastStickUs = nowUs;
    const uint32_t generation = TitleAdapter_GetGeneration(GameTitle::Halo3);
    if (generation != s.generation)
    {
        s.fit.Reset();
        s.ring.Reset();
        s.fitKey = 0;
        s.generation = generation;
    }
    if (slot != 0 || !pub.valid || !pub.seated || !pub.axesBits)
        return LegacySeatRecomposeWhy::NotSeated;   // not counted: not a Halo 3 seat this drive writes
    // THIS PASS'S CAMERA: the acquisition ran just before this stick call, in the same control pass.
    int32_t cameraUnit = -1, path = -1;
    float camera[3]{}, direction[3]{}, velocity[3]{};
    bool velocityValid = false;
    uint64_t stampUs = 0;
    const bool cameraRead = generation &&
        ReadHalo3AcquisitionCamera(generation, 100000, cameraUnit, path, camera, direction, velocity, velocityValid,
                                   stampUs) &&
        shot_lag::TickEyeFromThisTick(true, stampUs, previousStickUs);
    const int32_t localUnit = cameraRead ? LegacyHalo3LocalUnit() : -1;
    const bool ourUnit = cameraRead && localUnit != -1 && cameraUnit == localUnit;
    // Only the first-person path's answer is used (0x17DAEC writes both outputs, read offline); the chase path's is
    // counted and refused like no camera.
    const bool firstPerson = ourUnit && shot_lag::EvaluatorTookFirstPersonPath(path);
    const bool ours = firstPerson;
    g_legacySeatCameraPath[!ourUnit ? 2 : firstPerson ? 0 : 1].fetch_add(1, std::memory_order_relaxed);
    // THE GAIN: the game's answer (this camera's direction) to the record that drove it.
    if (pub.seatKey != s.fitKey)
    {
        s.fit.Reset();
        s.fitKey = pub.seatKey;
    }
    if (ours && !pub.gunSlaved && velocityValid)
    {
        const auto* inEffect = s.ring.InEffect(g_legacySimTicks[0].load(std::memory_order_acquire), pub.seatKey);
        float q[3]{direction[0], direction[1], direction[2]};
        if (!inEffect || !seat_upstream::Normalize(q))
            g_legacySeatGainUnpaired.fetch_add(1, std::memory_order_relaxed);
        else if (s.fit.Add(inEffect->dir, q, velocity, seat_upstream::kGainMinCrossWuPerS, seat_upstream::kGainDecay))
            g_legacySeatGainUsed.fetch_add(1, std::memory_order_relaxed);
        else
            g_legacySeatGainSlow.fetch_add(1, std::memory_order_relaxed);
    }
    const bool learned = s.fit.Learned(seat_upstream::kGainMinWeight);
    g_legacySeatGain.store(s.fit.Gain(), std::memory_order_relaxed);
    g_legacySeatGainLearned.store(learned, std::memory_order_relaxed);
    // seat_upstream_aim 2 (dev): solve for the camera an UNMODDED host queries this seat from -- in a stock
    // third-person seat its chase camera, asked of the evaluator's own chase path now (StockCamera::Chase); the line
    // has the same direction, so only the camera moves. Not reproducible here, or not answered: this pass's camera,
    // counted.
    if (ours && g_config.seat_upstream_aim == 2)
    {
        const auto stock = seat_upstream::StockCameraFor(Halo3OccupiedSeatFlags());
        float chasePosition[3]{}, chaseDirection[3]{};
        if (stock == seat_upstream::StockCamera::Chase &&
            Halo3AskChaseCamera(localUnit, chasePosition, chaseDirection))
        {
            memcpy(camera, chasePosition, sizeof(camera));
            g_legacySeatStockCamera[0].fetch_add(1, std::memory_order_relaxed);
        }
        else if (stock != seat_upstream::StockCamera::SameAsHere)
            g_legacySeatStockCamera[1].fetch_add(1, std::memory_order_relaxed);
    }
    // THE RECORD.
    seat_upstream::SeatRecomposeInput in{};
    in.enabled = g_config.seat_upstream_aim != 0 && Sight_AimEnabled();
    in.seated = true;
    in.gunSlaved = pub.gunSlaved;
    SightIntent intent{};
    in.intentRead = Sight_ReadIntent(GameTitle::Halo3, intent);
    in.intent = intent.p;
    memcpy(in.point, intent.solvePoint, sizeof(in.point));
    in.setYaw = pub.yaw;
    in.setPitch = pub.pitch;
    in.generation = generation;
    in.nowUs = nowUs;
    in.maxAgeUs = kSightIntentShotMaxAgeMs * 1000ull;
    in.cameraRead = ours;
    memcpy(in.camera, camera, sizeof(in.camera));
    memcpy(in.cameraVel, velocity, sizeof(in.cameraVel));
    in.cameraVelValid = velocityValid;
    in.tickS = float(g_legacyTickPeriodUs[0].load(std::memory_order_relaxed)) * 1.0e-6f;
    in.gain = learned ? s.fit.Gain() : 0.0f;
    float bendDeg = 0.0f;
    const LegacySeatRecomposeWhy why = seat_upstream::RecomposeSeat(in, yaw, pitch, bendDeg);
    g_legacySeatRecompose[static_cast<size_t>(why)].fetch_add(1, std::memory_order_relaxed);
    if (why == LegacySeatRecomposeWhy::Recomposed)
    {
        const uint32_t milli = static_cast<uint32_t>((std::min)(bendDeg * 1000.0f, 4.0e9f));
        g_legacySeatRecomposeMilliDeg.fetch_add(milli, std::memory_order_relaxed);
        uint32_t prior = g_legacySeatRecomposeMaxMilliDeg.load(std::memory_order_relaxed);
        while (milli > prior && !g_legacySeatRecomposeMaxMilliDeg.compare_exchange_weak(prior, milli,
                                                                                       std::memory_order_relaxed)) {}
    }
    return why;
}

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
    seat_witness::DriveSample ds{};
    ds.ms = SeatWitness_NowMs();
    ds.slot = static_cast<uint8_t>(slot);
    ds.caller = static_cast<uint8_t>(caller);
    ds.nativeYaw = ds.nativePitch = ds.wroteYaw = ds.wrotePitch = std::numeric_limits<float>::quiet_NaN();
    ds.setYaw = ds.setPitch = ds.aimYaw = ds.aimPitch = std::numeric_limits<float>::quiet_NaN();
#endif
    if (action != aim_caller::Action::StepAndWrite)
    {
#if HALOMCCVR_DEV_DIAGNOSTICS
        const uint32_t restoresBefore = st.restores.load(std::memory_order_relaxed);
        uint32_t refusalsBefore[aim_caller::kRefusalCount];
        for (int r = 0; r < aim_caller::kRefusalCount; ++r)
            refusalsBefore[r] = st.refusals[r].load(std::memory_order_relaxed);
        const uint32_t faultsBefore = st.faults.load(std::memory_order_relaxed);
#endif
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
#if HALOMCCVR_DEV_DIAGNOSTICS
        ds.outcome = action != aim_caller::Action::RestoreOnly ? seat_witness::DriveEndUpdate
            : st.restores.load(std::memory_order_relaxed) != restoresBefore ? seat_witness::DrivePutBack
            : seat_witness::DrivePutBackRefused;
        for (int r = 0; r < aim_caller::kRefusalCount; ++r)
            if (st.refusals[r].load(std::memory_order_relaxed) != refusalsBefore[r])
                ds.refusal = static_cast<uint8_t>(r);
        if (st.faults.load(std::memory_order_relaxed) != faultsBefore)
            ds.outcome = seat_witness::DriveFault;
        if (st.armed.load(std::memory_order_relaxed)) ds.flags |= seat_witness::DriveArmed;
        if (st.putBack.valid)
        {
            ds.wroteYaw = st.putBack.axis[0] ? st.putBack.value[0] : ds.wroteYaw;
            ds.wrotePitch = st.putBack.axis[1] ? st.putBack.value[1] : ds.wrotePitch;
        }
        SeatWitness_RecordDrive(ds);
#endif
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
    bool latchRead[2]{};      // which axes the seat-limit latch read this update (their stops go to the reticle)
    seat_frame_rt::Live seatFrame{};   // the seat's frame this update (World unless read)
    // What record this call leaves in effect, in world angles, for the seated gain learner's pairing (Halo 3).
    bool noteKnown = false;
    uint32_t noteKey = 0;
    float noteDir[3]{};
    __try
    {
        do
        {
            if (!g_config.legacy_direct_drive)
            {
                LEGACY_DRIVE_WITNESS(ds.outcome = seat_witness::DriveOff);
                // Switched off live: stop now, and make switching back on re-validate the record first.
                if (st.armed.exchange(false, std::memory_order_acq_rel))
                    st.disarms.fetch_add(1, std::memory_order_relaxed);
                st.agree = 0;
                break;
            }
            LEGACY_DRIVE_WITNESS(ds.outcome = seat_witness::DriveNoHook);
            if (!st.installed.load(std::memory_order_acquire) || !st.tlsIndexAddress)
                break;
            const uint8_t* tls = nullptr;
            const uintptr_t pc = LegacyDriveRecord(st, site, user, &tls);
            LEGACY_DRIVE_WITNESS(ds.outcome = seat_witness::DriveNoRecord);
            if (!pc)
                break;
            float* pair = reinterpret_cast<float*>(pc + site.angles);
            const float nativeYaw = pair[0], nativePitch = pair[1];
            LEGACY_DRIVE_WITNESS(ds.pcLow = static_cast<uint32_t>(pc), ds.nativeYaw = nativeYaw,
                                 ds.nativePitch = nativePitch);
            st.nativeYaw.store(nativeYaw, std::memory_order_relaxed);
            st.nativePitch.store(nativePitch, std::memory_order_relaxed);
            const LegacyDrivePublication pub = LegacyDriveReadPublication(st);
            // THE RECORD THROUGH THE INTENT POINT (LegacyDriveRecomposeOnFoot): Halo 3 on foot, the solve's angles
            // recomposed from the game's own eye at the tick that will consume them. Every other case keeps the
            // published angles exactly.
            float setYaw = pub.yaw, setPitch = pub.pitch;
            {
                const LegacyRecomposeWhy why = LegacyDriveRecomposeOnFoot(slot, pub, setYaw, setPitch);
                if (why != LegacyRecomposeWhy::NotOnFoot)
                    g_legacyRecompose[static_cast<size_t>(why)].fetch_add(1, std::memory_order_relaxed);
            }
            // THE SEATED RECORD UPSTREAM OF REPLICATION (LegacyDriveRecomposeSeated): a Halo 3 seat's setpoint becomes
            // the record whose stock crosshair line at the fire tick passes through the seat's point. The world angles
            // it returns go through the seat's frame, latch and write below exactly as the published ones did.
            if (slot == 0)
                (void)LegacyDriveRecomposeSeated(slot, pub, setYaw, setPitch);
            // World angles as published; a seat in case (b) takes them into the record's frame below.
            float yaw = setYaw, pitch = setPitch, aimYaw = pub.aimYaw, aimPitch = pub.aimPitch;
            const float handWorldYaw = setYaw;   // a yaw-only write converts at the game's own pitch (below)
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
#if HALOMCCVR_DEV_DIAGNOSTICS
            ds.setYaw = yaw; ds.setPitch = pitch; ds.aimYaw = aimYaw; ds.aimPitch = aimPitch;
            ds.seatKey = seatKey;
            ds.pubAgeMs = static_cast<uint16_t>(setMs && now >= setMs ? (now - setMs > 0xFFFE ? 0xFFFE : now - setMs)
                                                                        : 0xFFFF);
            ds.flags |= (fresh ? seat_witness::DriveFresh : 0) | (coherent ? seat_witness::DriveCoherent : 0) |
                (seatedSet ? seat_witness::DriveSeatedSet : 0);
#endif
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
            // A GUN-SLAVED SEAT (flag bit 7, the header): `aim` is the gun, so the occupant's own aim vector is read
            // as well, from the same unit the rebase rotates, and proven against the record beside it.
            const bool occupantActive = seatWrite && pub.gunSlaved && st.unitAimOffset != 0;
            float occupantYaw = std::numeric_limits<float>::quiet_NaN(), occupantPitch = occupantYaw;
            float occupantWorldYaw = occupantYaw, occupantWorldPitch = occupantYaw;
            bool occupantRead = false;
            // THE SEAT'S FRAME, read now from the unit the rebase will rotate this tick (see the header). In case
            // (b) the hand's aim and the game's aim go into the record's frame; unread, nothing is written.
            if (seatWrite)
            {
                const uint32_t seatUnit = LegacyDriveSeatUnit(slot, st, pc);
                seatFrame = seat_frame_rt::Read(st.frame, nullptr, seatUnit);
                seat_frame_rt::Count(st.frameCounters, seatFrame.kase);
                if (occupantActive)
                {
                    occupantRead = LegacyDriveReadOccupantAim(st, seatUnit, occupantWorldYaw, occupantWorldPitch);
                    st.seatOccUpdates.fetch_add(1, std::memory_order_relaxed);
                    if (!occupantRead)
                        st.seatOccUnread.fetch_add(1, std::memory_order_relaxed);
                }
#if HALOMCCVR_DEV_DIAGNOSTICS
                // THE FRAME CHECK: the record (as the accumulator left it) through R against the game's world aim --
                // in a gun-slaved seat the occupant's, which the record drives (the gun only follows it).
                if (occupantRead)
                    seat_frame_rt::NoteFrameCheck(st.frameCounters, seatFrame, nativeYaw, nativePitch,
                                                  occupantWorldYaw, occupantWorldPitch);
                else
                    seat_frame_rt::NoteFrameCheck(st.frameCounters, seatFrame, nativeYaw, nativePitch, aimYaw,
                                                  aimPitch);
#endif
                if (!seat_frame_rt::ToRecord(seatFrame, yaw, pitch, nativeYaw) ||
                    !seat_frame_rt::ToRecord(seatFrame, aimYaw, aimPitch, nativeYaw))
                {
                    // No frame this update: no seated write, the servo keeps the seat. The stint survives it as it
                    // survives a gap (bounded), so a flicker of an unreadable link does not cost the proven checks.
                    seatKept = st.stintActive && st.stintKey == seatKey &&
                        ++st.stintStaleUpdates <= kLegacyDriveStintStaleUpdates;
                    LEGACY_DRIVE_WITNESS(ds.outcome = seatKept ? seat_witness::DriveGapKept
                                                               : seat_witness::DriveSkipped);
                    st.seatedSkips.fetch_add(1, std::memory_order_relaxed);
                    st.agree = 0;
                    break;
                }
                if (occupantRead)
                {
                    occupantYaw = occupantWorldYaw;
                    occupantPitch = occupantWorldPitch;
                    occupantRead = seat_frame_rt::ToRecord(seatFrame, occupantYaw, occupantPitch, nativeYaw);
                }
            }
#if HALOMCCVR_DEV_DIAGNOSTICS
            ds.axes = static_cast<uint8_t>(wantAxes & 3u);
            ds.flags |= (onFoot ? seat_witness::DriveOnFoot : 0) |
                (seatConfirmed ? seat_witness::DriveSeatConfirmed : 0) |
                (slot != 2 && !LegacyDriveLocalOnFoot(slot) ? seat_witness::DriveTitleSeated : 0);
#endif
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
                LEGACY_DRIVE_WITNESS(ds.outcome = seatKept ? seat_witness::DriveGapKept : seat_witness::DriveSkipped);
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
                st.stintOccupantActive = occupantActive;
                const seat_drive::StintVerdict was =
                    LegacyDriveVerdict(st);
                const bool newSample = setMs != st.stintSampleMs;
                st.stintSampleMs = setMs;
                st.stint.step(fresh, nativeYaw, nativePitch, aimYaw, aimPitch, now, newSample);
                // The occupant's vector is read on this thread, on the record's own clock, every update: each read is
                // a sample (a repeat of the same tick is a still sample, which the check reads as such).
                if (occupantActive)
                    st.stintOccupant.step(fresh && occupantRead, nativeYaw, nativePitch, occupantYaw, occupantPitch,
                                          now, true);
                const seat_drive::StintVerdict v =
                    LegacyDriveVerdict(st);
                if (!was.yaw_ok && v.yaw_ok)
                {
                    st.seatYawChecks.fetch_add(1, std::memory_order_relaxed);
                    if (v.yaw_by_occupant)
                        st.seatOccYawChecks.fetch_add(1, std::memory_order_relaxed);
                }
                if (!was.pitch_ok && v.pitch_ok)
                {
                    st.seatPitchChecks.fetch_add(1, std::memory_order_relaxed);
                    if (v.pitch_by_occupant)
                        st.seatOccPitchChecks.fetch_add(1, std::memory_order_relaxed);
                }
                // Counted only where the seat wants its pitch (a Halo 4 seat never does); each failed attempt
                // once (the check retries every 5 s).
                if (v.pitch_failed && !st.stintPitchFailCounted && (wantAxes & kLegacyDriveAxisPitch))
                {
                    st.stintPitchFailCounted = true;
                    st.seatPitchFails.fetch_add(1, std::memory_order_relaxed);
                }
                else if (!v.pitch_failed)
                    st.stintPitchFailCounted = false;
                // THE OUTCOME GUARD: an axis written on the occupant's proof alone is watched for the gun coming to
                // rest off the record (a real offset, not lag); then the camera's check decides it again.
                if (occupantActive && fresh && newSample)
                {
                    const bool byOccupant[2]{v.yaw_by_occupant, v.pitch_by_occupant};
                    const float record[2]{nativeYaw, nativePitch}, gun[2]{aimYaw, aimPitch};
                    for (int a = 0; a < 2; ++a)
                    {
                        auto& watch = st.occWatch[a];
                        if (watch.revoked)
                            continue;
                        if (!byOccupant[a])
                        {
                            watch.have = false;
                            watch.count = 0;
                            continue;
                        }
                        if (watch.step(record[a], gun[a]))
                            st.seatOccRevoked[a].fetch_add(1, std::memory_order_relaxed);
                    }
                }
                // For the input thread's servo: the occupant's WORLD aim and how long it has agreed per axis.
                if (occupantActive && occupantRead)
                {
                    const uint32_t yawAgree =
                        st.occWatch[0].revoked ? 0u : (std::min)(st.stintOccupant.agreeing(0), 0xFFFFu);
                    const uint32_t pitchAgree =
                        st.occWatch[1].revoked ? 0u : (std::min)(st.stintOccupant.agreeing(1), 0xFFFFu);
                    st.occSeq.fetch_add(1, std::memory_order_acq_rel);
                    st.occYaw.store(occupantWorldYaw, std::memory_order_relaxed);
                    st.occPitch.store(occupantWorldPitch, std::memory_order_relaxed);
                    st.occKey.store(seatKey, std::memory_order_relaxed);
                    st.occAgree.store(yawAgree | pitchAgree << 16, std::memory_order_relaxed);
                    st.occMs.store(now, std::memory_order_relaxed);
                    st.occSeq.fetch_add(1, std::memory_order_release);
                }
#if HALOMCCVR_DEV_DIAGNOSTICS
                // The turret line (dev): the record against the occupant's aim and against the gun, record frame.
                if (occupantActive)
                {
                    if (occupantRead)
                        aim_caller::AtomicMaxFloat(st.occRecordMaxBits, gun_slaved::AngleBetweenDeg(
                            nativeYaw, nativePitch, occupantYaw, occupantPitch));
                    if (fresh)
                        aim_caller::AtomicMaxFloat(st.gunRecordMaxBits, gun_slaved::AngleBetweenDeg(
                            nativeYaw, nativePitch, aimYaw, aimPitch));
                }
#endif
            }
            if (!st.armed.load(std::memory_order_acquire))
            {
                // ValueAgreement: the record must hold the game's own aim before it is ever written -- both
                // axes on foot; yaw in a seat, whose pitch need not (its own check decides that axis).
                // A gun-slaved seat may arm on the occupant's own aim as well as the gun's.
                const bool armAgree = seatWrite
                    ? (LegacyDriveYawAgrees(fresh, nativeYaw, aimYaw) ||
                       (occupantRead && LegacyDriveYawAgrees(fresh, nativeYaw, occupantYaw)))
                    : agrees;
                st.agree = armAgree ? st.agree + 1 : 0;
                if (st.agree >= kLegacyDriveAgreeUpdates)
                {
                    st.expected.store(pc, std::memory_order_release);
                    st.armed.store(true, std::memory_order_release);
                    st.arms.fetch_add(1, std::memory_order_relaxed);
                }
                LEGACY_DRIVE_WITNESS(ds.outcome = seat_witness::DriveArming);
                break;
            }
            if (pc != st.expected.load(std::memory_order_acquire))
            {
                LEGACY_DRIVE_WITNESS(ds.outcome = seat_witness::DriveDisarmed);
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
                // Either reference proves an axis (seat_drive::CombineStints): the camera's, or -- in a gun-slaved
                // seat -- the occupant's.
                const seat_drive::StintVerdict v =
                    LegacyDriveVerdict(st);
                writeYaw = (wantAxes & kLegacyDriveAxisYaw) && v.yaw_ok;
                writePitch = (wantAxes & kLegacyDriveAxisPitch) && v.yaw_ok && v.pitch_ok;
                if (!writeYaw && !writePitch)
                {
                    // Waiting on a check only while one is pending; a pitch that failed its check is on the
                    // servo until its retry, which is not a wait.
                    const bool pending = ((wantAxes & kLegacyDriveAxisYaw) && !v.yaw_ok) ||
                        ((wantAxes & kLegacyDriveAxisPitch) && !v.pitch_ok && !v.pitch_failed);
                    if (pending)
                        st.seatUnverified.fetch_add(1, std::memory_order_relaxed);
                    LEGACY_DRIVE_WITNESS(ds.outcome = seat_witness::DriveUnverified);
                    break;
                }
            }
            // A YAW-ONLY seat write in case (b) keeps the game's record pitch, so its yaw is converted at the pitch
            // the game will aim at (seat_frame::RecordYawForWorldYaw), not at the hand's: until the pitch servo
            // caught up, the hand's pitch put the heading off under a tilted hull.
            if (seatWrite && writeYaw && !writePitch && seatFrame.kase == seat_frame::Case::Rebased)
            {
                float yawOnly = 0.0f;
                if (seat_frame::RecordYawForWorldYaw(seatFrame.frame, handWorldYaw, nativeYaw, nativePitch, yawOnly))
                    yaw = yawOnly;
            }
            LEGACY_DRIVE_WITNESS(ds.outcome = seat_witness::DriveNotFresh);
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
                    latchRead[0] = latch[0];
                    latchRead[1] = latch[1];
                    if (latch[0])
                        y = aim_write::unwrap_to_branch(held[0], nativeYaw);
                    if (latch[1])
                        p = aim_write::clamp_pitch(held[1], 1.45f);
                }
                // All three titles pack the control yaw into the per-tick player action as 13 bits over
                // [0, 2*pi], and keep their own record in [0, 2*pi). The branch-matched value above can lie up
                // to half a turn past the seam, so it is wrapped into that range (aim_write::wrap_0_2pi).
                // Halo 3 / ODST: the quantizer neither clamps nor masks, and the overflow bits press the action
                // button (exit seat in a vehicle). Halo 4: its quantizer clamps (halo4 0x517BEC), so the
                // overflow was a one-update aim nudge to 0 or 2*pi instead.
                if (std::isfinite(y) && !(y >= 0.0f && y < aim_write::kTwoPi))
                {
                    y = aim_write::wrap_0_2pi(y);
                    st.yawWrapped.fetch_add(1, std::memory_order_relaxed);
                }
                if (std::isfinite(y) && std::isfinite(p))
                {
                    if (writeYaw)
                        pair[0] = y;
                    if (writePitch)
                        pair[1] = p;
#if HALOMCCVR_DEV_DIAGNOSTICS
                    ds.outcome = seat_witness::DriveWrote;
                    if (writeYaw) ds.wroteYaw = y;
                    if (writePitch) ds.wrotePitch = p;
                    ds.axes |= static_cast<uint8_t>((writeYaw ? 0x10 : 0) | (writePitch ? 0x20 : 0));
                    if (limited) ds.flags |= seat_witness::DriveLimited;
#endif
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
                        // The stops the reticle clamps against (seat_limit_reticle_logic.h): the latch's, on the axes
                        // it read, in this update's frame. None is a valid publication (written, nothing holds it).
                        {
                            seat_limit::Stops stops{};
                            seat_limit::AddLatch(stops, st.seatLatch, latchRead);
                            // A LOOK-STEERED DRIVER's pitch is never latched (autolevel); its stops are the record's
                            // own eased limits, read now: the update just eased them and clamped the pitch to them.
                            if (writePitch && !pitchLatch && st.pitchLimitOffset)
                            {
                                const float lo = *reinterpret_cast<const float*>(pc + st.pitchLimitOffset);
                                const float hi = *reinterpret_cast<const float*>(pc + st.pitchLimitOffset + 4);
                                if (seat_limit::RecordPitchInterval(lo, hi))
                                {
                                    seat_limit::AddInterval(stops, 1, lo, hi);
                                    st.pitchLimitPublished.fetch_add(1, std::memory_order_relaxed);
                                }
                                else
                                {
                                    st.pitchLimitRefused.fetch_add(1, std::memory_order_relaxed);
                                }
                            }
                            stops.rebased = seatFrame.kase == seat_frame::Case::Rebased;
                            stops.frame = seatFrame.frame;
                            st.stops.publish(seatKey, stops, nativeYaw, now);
                            // Halo 3: a look-steered driver's gun band is current while its updates run
                            // (halo3_vehicle_gun_band.inl; the aim poll checks the vehicle and definition itself).
                            if (slot == 0 && !pitchLatch)
                                Halo3GunBandMarkDriver(now);
                        }
                        st.wroteSeq.fetch_add(1, std::memory_order_acq_rel);
                        st.wroteYaw[1].store(st.wroteYaw[0].load(std::memory_order_relaxed), std::memory_order_relaxed);
                        st.wrotePitch[1].store(st.wrotePitch[0].load(std::memory_order_relaxed),
                                               std::memory_order_relaxed);
                        st.wroteValid[1].store(st.wroteValid[0].load(std::memory_order_relaxed),
                                               std::memory_order_relaxed);
                        // As the WORLD angles the write put into effect (R e(record) in case b): the input thread's
                        // witnesses compare these with the game's world aim.
                        float worldYaw = writeYaw ? y : nativeYaw, worldPitch = writePitch ? p : nativePitch;
                        seat_frame::Angles world{};
                        if (seatFrame.kase == seat_frame::Case::Rebased &&
                            seat_frame::WorldFromRecord(seatFrame.frame, worldYaw, worldPitch, world))
                        {
                            worldYaw = world.yaw;
                            worldPitch = world.pitch;
                        }
                        st.wroteYaw[0].store(worldYaw, std::memory_order_relaxed);
                        st.wrotePitch[0].store(worldPitch, std::memory_order_relaxed);
                        st.wroteValid[0].store(true, std::memory_order_relaxed);
                        st.wroteSeq.fetch_add(1, std::memory_order_release);
                        noteKnown = true;
                        noteKey = seatKey;
                        seat_upstream::DirFromAngles(worldYaw, worldPitch, noteDir);
                        if (writeYaw)
                            st.seatYawWrites.fetch_add(1, std::memory_order_relaxed);
                        if (writePitch)
                            st.seatPitchWrites.fetch_add(1, std::memory_order_relaxed);
                        if (limited)
                        {
                            st.seatHeld.fetch_add(1, std::memory_order_relaxed);
                            noteKnown = false;   // held at the seat's stop: the game's aim there is the clamp's
                        }
                    }
                    else
                    {
                        // On foot: the record left is the written pair (world angles), the unwritten axis the game's.
                        noteKnown = true;
                        noteKey = 0;
                        seat_upstream::DirFromAngles(writeYaw ? y : nativeYaw, writePitch ? p : nativePitch, noteDir);
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
        noteKnown = false;
        st.faults.fetch_add(1, std::memory_order_relaxed);
        LEGACY_DRIVE_WITNESS(ds.outcome = seat_witness::DriveFault);
    }
    // The seated gain learner's pairing (Halo 3): the record this stick call leaves for the ticks that follow it.
    if (slot == 0)
        LegacySeatUpstreamNoteCall(noteKey, noteKnown ? noteDir : nullptr);
    // The latch's contract is "nothing written last tick: forget everything". Every update that did not end
    // in a seat write -- on foot, a skipped or unverified seat, disarmed, switched off, a fault -- resets it
    // here, so a later seat write never reads a limit off a value this hook did not write. Per axis, an axis
    // this update did not write starts its prime again.
    for (int a = 0; a < 2; ++a)
        st.axisTicks[a] = wroteAxis[a]
            ? (st.axisTicks[a] < kLegacyDriveSeatPrimeUpdates ? st.axisTicks[a] + 1 : st.axisTicks[a])
            : 0;
    if (!seatWrote)
    {
        st.seatLatch.reset();
        st.stops.clear();
    }
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
#if HALOMCCVR_DEV_DIAGNOSTICS
    {
        const seat_drive::StintVerdict v =
            LegacyDriveVerdict(st);
        ds.flags |= (st.armed.load(std::memory_order_relaxed) ? seat_witness::DriveArmed : 0) |
            (st.stintActive ? seat_witness::DriveStint : 0) | (v.yaw_ok ? seat_witness::DriveYawOk : 0) |
            (v.pitch_ok ? seat_witness::DrivePitchOk : 0) | (v.pitch_failed ? seat_witness::DrivePitchFailed : 0);
    }
    SeatWitness_RecordDrive(ds);
#endif
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
    {
        uint32_t n[static_cast<size_t>(LegacyRecomposeWhy::Count)]{};
        uint32_t total = 0;
        for (size_t i = 0; i < static_cast<size_t>(LegacyRecomposeWhy::Count); ++i)
            total += (n[i] = g_legacyRecompose[i].exchange(0, std::memory_order_relaxed));
        const uint32_t sum = g_legacyRecomposeMilliDeg.exchange(0, std::memory_order_relaxed);
        const uint32_t max = g_legacyRecomposeMaxMilliDeg.exchange(0, std::memory_order_relaxed);
        if (total)
            LOG("SHOT RECORD Halo 3 on foot: %u stick writes recomposed through the intent point from the game's own "
                "eye a tick ahead (%.1f ms; mean %.2f deg from the solve, max %.2f), %u kept as solved (no intent %u, "
                "another solve %u, point stale/seated/rail-held %u, not a first-person eye %u, no game eye from this "
                "tick %u, game eye unproven %u, point within 30 cm %u, point behind the eye %u)",
                n[0], double(g_legacyTickPeriodUs[0].load(std::memory_order_relaxed)) / 1000.0,
                n[0] ? double(sum) / 1000.0 / double(n[0]) : 0.0, double(max) / 1000.0, total - n[0],
                n[static_cast<size_t>(LegacyRecomposeWhy::NoIntent)],
                n[static_cast<size_t>(LegacyRecomposeWhy::OtherSolve)],
                n[static_cast<size_t>(LegacyRecomposeWhy::PointUnusable)],
                n[static_cast<size_t>(LegacyRecomposeWhy::NotFirstPerson)],
                n[static_cast<size_t>(LegacyRecomposeWhy::NoGameEye)],
                n[static_cast<size_t>(LegacyRecomposeWhy::EyeUnproven)],
                n[static_cast<size_t>(LegacyRecomposeWhy::PointTooClose)],
                n[static_cast<size_t>(LegacyRecomposeWhy::PointBehind)]);
        {
            // THE SEATED RECORD UPSTREAM OF REPLICATION (LegacyDriveRecomposeSeated): how many seat setpoints were
            // replaced by the record whose stock crosshair line passes through P, why the rest were kept, and the
            // learned velocity gain (printed as its speed, 1/g).
            using SR = LegacySeatRecomposeWhy;
            uint32_t sr[static_cast<size_t>(SR::Count)]{};
            uint32_t seatTotal = 0;
            for (size_t i = 0; i < static_cast<size_t>(SR::Count); ++i)
                seatTotal += (sr[i] = g_legacySeatRecompose[i].exchange(0, std::memory_order_relaxed));
            const uint32_t seatSum = g_legacySeatRecomposeMilliDeg.exchange(0, std::memory_order_relaxed);
            const uint32_t seatMax = g_legacySeatRecomposeMaxMilliDeg.exchange(0, std::memory_order_relaxed);
            const uint32_t used = g_legacySeatGainUsed.exchange(0, std::memory_order_relaxed);
            const uint32_t slow = g_legacySeatGainSlow.exchange(0, std::memory_order_relaxed);
            const uint32_t unpaired = g_legacySeatGainUnpaired.exchange(0, std::memory_order_relaxed);
            uint32_t paths[3]{};
            for (int i = 0; i < 3; ++i)
                paths[i] = g_legacySeatCameraPath[i].exchange(0, std::memory_order_relaxed);
            const float gain = g_legacySeatGain.load(std::memory_order_relaxed);
            const bool learned = g_legacySeatGainLearned.load(std::memory_order_relaxed);
            const uint32_t recomposed = sr[static_cast<size_t>(SR::Recomposed)];
            if (seatTotal)
            {
                char kept[320]{};
                size_t length = 0;
                for (size_t i = 1; i < static_cast<size_t>(SR::Count); ++i)
                {
                    if (!sr[i] || length + 1 >= sizeof(kept))
                        continue;
                    const int w = std::snprintf(kept + length, sizeof(kept) - length, "%s%s %u", length ? ", " : "",
                                                seat_upstream::SeatRecomposeName(static_cast<SR>(i)), sr[i]);
                    if (w > 0)
                        length += (std::min)(static_cast<size_t>(w), sizeof(kept) - length - 1);
                }
                LOG("SHOT RECORD Halo 3 seated: %u stick writes recomposed so the stock crosshair line at the fire "
                    "tick passes through the point (what a multiplayer host fires; mean %.2f deg from the solve, max "
                    "%.2f), %u kept as solved%s%s%s; this pass's camera first person %u, chase %u, none %u; velocity "
                    "gain %s (1/g %.0f m/s; samples used %u, too slow %u, unpaired %u); solved for %s camera (stock "
                    "chase camera used %u, not available %u)",
                    recomposed, recomposed ? double(seatSum) / 1000.0 / double(recomposed) : 0.0,
                    double(seatMax) / 1000.0, seatTotal - recomposed, length ? " (" : "", length ? kept : "",
                    length ? ")" : "", paths[0], paths[1], paths[2], learned ? "learned" : "not yet learned (g = 0 used)",
                    std::fabs(gain) > 1.0e-9f ? double(shot_lag::kMetresPerWu / gain) : 0.0, used, slow, unpaired,
                    g_config.seat_upstream_aim == 2 ? "an unmodded host's" : "this machine's",
                    g_legacySeatStockCamera[0].exchange(0, std::memory_order_relaxed),
                    g_legacySeatStockCamera[1].exchange(0, std::memory_order_relaxed));
            }
        }
        {
            // The first-person eye identity (shot_lag_logic.h 4d) at the stick calls with a point: why it said no,
            // and the director's camera mode when the camera was the reason.
            using FE = shot_lag::FirstPersonEye;
            uint32_t fe[static_cast<size_t>(FE::Count)]{};
            uint32_t refused = 0;
            for (size_t i = 0; i < static_cast<size_t>(FE::Count); ++i)
            {
                fe[i] = g_firstPersonEye[i].exchange(0, std::memory_order_relaxed);
                if (i != static_cast<size_t>(FE::Yes))
                    refused += fe[i];
            }
            uint32_t modes[shot_lag::kHalo3CameraModeCount + 1]{};
            for (int i = 0; i <= shot_lag::kHalo3CameraModeCount; ++i)
                modes[i] = g_firstPersonEyeMode[i].exchange(0, std::memory_order_relaxed);
            if (refused)
                LOG("SHOT RECORD Halo 3 first-person eye: %u of %u stick calls not our unit's first-person eye (%s %u, "
                    "%s %u, %s %u: the director's camera %s %u, %s %u, %s %u, %s %u, %s %u, %s %u, %s %u, first person "
                    "from another perspective %u); neither recomposed nor asked for the game's eye",
                    refused, refused + fe[static_cast<size_t>(FE::Yes)],
                    shot_lag::FirstPersonEyeName(FE::NotOnFootControlled),
                    fe[static_cast<size_t>(FE::NotOnFootControlled)],
                    shot_lag::FirstPersonEyeName(FE::DirectorUnread), fe[static_cast<size_t>(FE::DirectorUnread)],
                    shot_lag::FirstPersonEyeName(FE::NotFirstPersonCamera),
                    fe[static_cast<size_t>(FE::NotFirstPersonCamera)],
                    shot_lag::Halo3CameraModeName(0), modes[0], shot_lag::Halo3CameraModeName(1), modes[1],
                    shot_lag::Halo3CameraModeName(2), modes[2], shot_lag::Halo3CameraModeName(4), modes[4],
                    shot_lag::Halo3CameraModeName(5), modes[5], shot_lag::Halo3CameraModeName(6), modes[6],
                    shot_lag::Halo3CameraModeName(7), modes[7], modes[shot_lag::kHalo3CameraModeCount]);
        }
        {
            // The game eye's misses by cause (shot_lag_logic.h 4c), whenever this tick's acquisition published none.
            uint32_t acq[static_cast<size_t>(shot_lag::TickEyeAcquisition::Count)]{};
            uint32_t miss[static_cast<size_t>(shot_lag::TickEyeMiss::Count)]{};
            uint32_t misses = 0;
            for (size_t i = 0; i < static_cast<size_t>(shot_lag::TickEyeAcquisition::Count); ++i)
                misses += (acq[i] = g_tickEyeAcquisition[i].exchange(0, std::memory_order_relaxed));
            for (size_t i = 0; i < static_cast<size_t>(shot_lag::TickEyeMiss::Count); ++i)
                miss[i] = g_tickEyeMiss[i].exchange(0, std::memory_order_relaxed);
            const uint32_t chase = g_halo3TickEyeChasePath.exchange(0, std::memory_order_relaxed);
            using TM = shot_lag::TickEyeMiss;
            if (misses || chase)
                LOG("SHOT RECORD Halo 3 game eye: this tick's acquisition published none at %u stick calls (the game "
                    "skipped its acquisition %u, it ran without reaching the evaluator %u); %s %u (%s %u, %s %u), not "
                    "asked: %s %u, %s %u, %s %u, %s %u; acquisition eyes withheld as chase-camera %u",
                    misses, acq[static_cast<size_t>(shot_lag::TickEyeAcquisition::Skipped)],
                    acq[static_cast<size_t>(shot_lag::TickEyeAcquisition::RanWithoutEvaluator)],
                    shot_lag::TickEyeMissName(TM::FromEvaluator),
                    miss[static_cast<size_t>(TM::FromEvaluator)] + miss[static_cast<size_t>(TM::ThirdPersonPath)] +
                        miss[static_cast<size_t>(TM::EvaluatorFault)],
                    shot_lag::TickEyeMissName(TM::ThirdPersonPath), miss[static_cast<size_t>(TM::ThirdPersonPath)],
                    shot_lag::TickEyeMissName(TM::EvaluatorFault), miss[static_cast<size_t>(TM::EvaluatorFault)],
                    shot_lag::TickEyeMissName(TM::NoHook), miss[static_cast<size_t>(TM::NoHook)],
                    shot_lag::TickEyeMissName(TM::NoUnit), miss[static_cast<size_t>(TM::NoUnit)],
                    shot_lag::TickEyeMissName(TM::NotFirstPersonEye), miss[static_cast<size_t>(TM::NotFirstPersonEye)],
                    shot_lag::TickEyeMissName(TM::GameDeclined), miss[static_cast<size_t>(TM::GameDeclined)], chase);
        }
        // The game-eye proof's state, once per change (the ValueAgreement above PublishHalo3TickEye).
        static uint32_t s_proofLogged = 0;
        const uint32_t proof = g_halo3TickEyeProof.load(std::memory_order_relaxed);
        if (proof != s_proofLogged)
        {
            const uint32_t was = s_proofLogged;
            s_proofLogged = proof;
            if (proof == static_cast<uint32_t>(TickEyeProof::Unproven) &&
                was == static_cast<uint32_t>(TickEyeProof::Disproven))
                LOG("SHOT RECORD Halo 3: a new unit or on-foot stint re-opens the game-eye proof (the record keeps its "
                    "solved angles until it is proven)");
            else if (proof == static_cast<uint32_t>(TickEyeProof::Proven))
                LOG("SHOT RECORD Halo 3: the acquisition's camera evaluator agrees with the engine camera at rest "
                    "(%.0f mm, 30 samples): the record is recomposed from the game's own eye",
                    double(g_halo3TickEyeRestOffsetMm.load(std::memory_order_relaxed)));
            else if (proof == static_cast<uint32_t>(TickEyeProof::Disproven))
                LOG("SHOT RECORD Halo 3: the acquisition's camera evaluator sits %.0f mm from the engine camera at "
                    "rest, on foot in first person, for %u samples in a row; it is not the eye, so the record keeps "
                    "its solved angles for this on-foot stint (the shot re-aim is unaffected)",
                    double(g_halo3TickEyeRestOffsetMm.load(std::memory_order_relaxed)),
                    shot_lag::EyeProof::kDisagreeToDisprove);
        }
    }
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
                "pitch %.3f vs game aim yaw %.3f pitch %.3f; yaw wrapped into [0, 2pi) %u; %s",
                name, st.armed.load() ? 1 : 0, calls, st.writes.exchange(0), st.arms.exchange(0),
                st.disarms.exchange(0), st.seatedSkips.exchange(0), st.faults.exchange(0), st.nativeYaw.load(),
                st.nativePitch.load(), st.aimYaw.load(), st.aimPitch.load(), st.yawWrapped.exchange(0), callers);
            const uint32_t stints = st.seatStints.exchange(0), yawWrites = st.seatYawWrites.exchange(0),
                           pitchWrites = st.seatPitchWrites.exchange(0), unverified = st.seatUnverified.exchange(0),
                           yawChecks = st.seatYawChecks.exchange(0), pitchChecks = st.seatPitchChecks.exchange(0),
                           pitchFails = st.seatPitchFails.exchange(0), held = st.seatHeld.exchange(0);
            char frames[96];
            const bool unread = seat_frame_rt::CaseText(st.frameCounters, frames, sizeof(frames));
            if (stints || yawWrites || pitchWrites || unverified || yawChecks || pitchChecks || pitchFails || held ||
                frames[0])
            {
                const seat_drive::StintVerdict v =
                    LegacyDriveVerdict(st);
                LOG("DRIVE %s seats: new stints %u; checks passed yaw %u pitch %u, pitch checks failed %u (retried after 5 s); writes "
                    "yaw %u pitch %u (held at a seat limit %u); updates waiting on a check %u; this stint yaw %s, "
                    "pitch %s; %s",
                    name, stints, yawChecks, pitchChecks, pitchFails, yawWrites, pitchWrites, held, unverified,
                    v.yaw_ok ? "proven" : "checking",
                    v.pitch_ok ? "proven" : v.pitch_failed ? "servo until its retry" : "checking",
                    frames[0] ? frames : "frame not read");
            }
            // Gun-slaved seats (flag bit 7): what the occupant's own aim proved, and reads that failed.
            const uint32_t occUpdates = st.seatOccUpdates.exchange(0), occYaw = st.seatOccYawChecks.exchange(0),
                           occPitch = st.seatOccPitchChecks.exchange(0), occUnread = st.seatOccUnread.exchange(0),
                           revokedYaw = st.seatOccRevoked[0].exchange(0), revokedPitch = st.seatOccRevoked[1].exchange(0);
            if (occUpdates || occYaw || occPitch || revokedYaw || revokedPitch)
                LOG("DRIVE %s gun-slaved seat (its camera is the gun): %u updates proved against the occupant's own aim "
                    "too; checks it passed alone yaw %u pitch %u; revoked (the gun came to rest > 3 deg off the record) "
                    "yaw %u pitch %u; vector unreadable %u; this stint the occupant's check yaw %s, pitch %s",
                    name, occUpdates, occYaw, occPitch, revokedYaw, revokedPitch, occUnread,
                    st.occWatch[0].revoked ? "revoked" : st.stintOccupant.yaw_ok ? "proven" : "checking",
                    st.occWatch[1].revoked ? "revoked"
                        : st.stintOccupant.pitch_ok ? "proven" : st.stintOccupant.pitch_failed ? "failed" : "checking");
            // A look-steered driver's pitch stops (the record's eased limits), when any were published.
            const uint32_t limitPublished = st.pitchLimitPublished.exchange(0, std::memory_order_relaxed);
            const uint32_t limitRefused = st.pitchLimitRefused.exchange(0, std::memory_order_relaxed);
            if (limitPublished || limitRefused)
                LOG("DRIVE %s driver pitch stops: %u writes published the record's eased limits as the reticle's pitch "
                    "stops; %u reads refused (not a finite, ordered pitch range)", name, limitPublished, limitRefused);
            // Once per binding: a seat whose frame did not read is never written (the servo keeps it).
            if (unread && !st.frameCounters.unknownLogged.exchange(1, std::memory_order_relaxed))
                LOG("DRIVE %s seats: the seat's frame did not read (the rebase's links or the vehicle's up failed "
                    "their checks); those updates write nothing and the stick servo keeps the seat (logged once)",
                    name);
        }
#if HALOMCCVR_DEV_DIAGNOSTICS
        {
            char check[224];
            seat_frame_rt::FrameCheckText(st.frameCounters, check, sizeof(check));
            if (check[0])
                LOG("DRIVE %s seats (dev): %s", name, check);   // frame check: max |R*e(record) - game aim| ...
        }
        {
            // THE TURRET LINE (gun-slaved seats): the record against the occupant's aim and the gun, how the gun
            // landed on the writes, its peak slew, the gun's pitch reversals against the hand's (a nod reverses where
            // the hand does not), and what the stick servo closed on.
            const float occMax = aim_caller::BitsToFloat(st.occRecordMaxBits.exchange(0, std::memory_order_relaxed));
            const float gunMax = aim_caller::BitsToFloat(st.gunRecordMaxBits.exchange(0, std::memory_order_relaxed));
            const uint32_t polls = st.turretPolls.exchange(0, std::memory_order_relaxed);
            const uint32_t settles = st.turretSettles.exchange(0, std::memory_order_relaxed);
            const uint32_t settleMax = st.turretSettleMaxMs.exchange(0, std::memory_order_relaxed);
            const float slew = aim_caller::BitsToFloat(st.turretPeakSlewBits.exchange(0, std::memory_order_relaxed));
            const uint32_t gunRev = st.turretGunReversals.exchange(0, std::memory_order_relaxed);
            const float gunSwing = aim_caller::BitsToFloat(st.turretGunSwingBits.exchange(0, std::memory_order_relaxed));
            const uint32_t handRev = st.turretHandReversals.exchange(0, std::memory_order_relaxed);
            const float handSwing =
                aim_caller::BitsToFloat(st.turretHandSwingBits.exchange(0, std::memory_order_relaxed));
            uint32_t occFb[2], gunFb[2], holds[2];
            for (int a = 0; a < 2; ++a)
            {
                occFb[a] = st.turretServoOccupant[a].exchange(0, std::memory_order_relaxed);
                gunFb[a] = st.turretServoGun[a].exchange(0, std::memory_order_relaxed);
                holds[a] = st.turretLagHolds[a].exchange(0, std::memory_order_relaxed);
            }
            if (polls || occMax > 0.0f || gunMax > 0.0f)
                LOG("DRIVE %s turret (dev): record vs the occupant's aim max %.2f deg, vs the gun max %.2f deg; gun "
                    "settled on the writes %u times, longest %u ms, peak slew %.0f deg/s; pitch reversals gun %u (max "
                    "swing %.1f deg) vs hand %u (max swing %.1f deg) in %u polls; servo closed on the occupant yaw %u "
                    "pitch %u, on the gun yaw %u pitch %u, pacer held yaw %u pitch %u",
                    name, occMax, gunMax, settles, settleMax, slew, gunRev, gunSwing, handRev, handSwing, polls,
                    occFb[0], occFb[1], gunFb[0], gunFb[1], holds[0], holds[1]);
        }
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
            // The pass order is measured only where the extra calls lead their pass (Halo 4's dispatcher); Halo
            // 3's / ODST's kick runs in a tick after its stick call (aim_caller::TickCensus::extras_lead).
            char order[112];
            if (mode == 2)
                snprintf(order, sizeof(order), "ticks with an extra call before a stick %u (stick calls with one "
                         "ahead %u)", t.extra_before_stick, t.extra_passes);
            else
                snprintf(order, sizeof(order), "extra call before a stick n/a (direct callers: a kick follows its "
                         "stick)");
            LOG("DRIVE %s callers/tick (dev): ticks %u, stick %u (%.2f per tick), impulse %u, sway %u, kick %u, "
                "no-unit %u, unknown %u (last unknown %s +0x%llX); seated of those: stick %u, impulse %u, sway %u, "
                "kick %u, no-unit %u, unknown %u; put back %u; max calls in one tick %u; %s; ticks with no stick "
                "%u; stalled ticks %u; tick went back %u; tick unreadable %u",
                name, t.ticks, byRole[role(aim_caller::Caller::Stick)],
                t.ticks ? static_cast<double>(t.sticks) / t.ticks : 0.0, byRole[role(aim_caller::Caller::Impulse)],
                byRole[role(aim_caller::Caller::Sway)], byRole[role(aim_caller::Caller::Kick)],
                byRole[role(aim_caller::Caller::NoUnit)], byRole[role(aim_caller::Caller::Unknown)],
                mode == 2 ? "dispatcher return" : "return", unknownRva,
                t.calls[1][role(aim_caller::Caller::Stick)], t.calls[1][role(aim_caller::Caller::Impulse)],
                t.calls[1][role(aim_caller::Caller::Sway)], t.calls[1][role(aim_caller::Caller::Kick)],
                t.calls[1][role(aim_caller::Caller::NoUnit)], t.calls[1][role(aim_caller::Caller::Unknown)],
                st.censusPutBacks.exchange(0, std::memory_order_relaxed), t.max_calls, order, t.no_stick, t.stalled,
                t.went_back, t.unreadable);
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
