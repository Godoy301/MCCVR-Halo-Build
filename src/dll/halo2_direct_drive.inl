// Included by halo2_observer_6dof.cpp inside its anonymous namespace, after CountPatternMatches. The drive's
// state and the accessors the observer reads (Halo2DriveState, g_halo2Drive, the record offsets) are
// declared earlier in that file.
//
// HALO 2 DIRECT DRIVE (aim_direct_drive, F1 > Weapon & Aim). Halo 2 aimed by writing the UNIT's aiming
// vectors (Halo2NativeAimUpdateDetour, +0x8FDF50), downstream of player control. Two things followed. The
// body's facing and the walk frame, which the game builds from player control's desired angles, stayed
// wherever the thumbstick had left them: legs up to 180 deg off the aim, and walking forward played the
// backpedal (the user, 2026-09-30). And nothing the game NETWORKS carried the aim, because player control's
// angles are what goes into the player_action the simulation submits. So the aim now goes where Reach,
// Halo 3, ODST and Halo 4 put it: into player control's desired angles, right after the native accumulator.
// The unit-aim write stays as well, so the shot keeps following the hand without the unit's own easing.
//
// The accumulator (retail halo2.dll, found 2026-09-30 by structure: the Halo 1 decomp's
// player_control_modify_desired_angles, E-H2-80's player_control.cpp attribution of +0x6C0DF0, and the
// "player control globals" allocation):
//   +0x6C3DF0  void (int user /*ecx*/, float dyaw /*xmm1*/, float dpitch /*xmm2*/): desired += delta, yaw
//              wrapped to [0, 2pi), pitch clamped to the record's eased limits [+0xA8, +0xAC]
//   record     *[the rip slot the prologue loads] + user * 0xB8: desired yaw +0x38, pitch +0x3C, radians;
//              yaw 0 = world +X, positive toward +Y; pitch positive up (forward = cos y cos p, sin y cos p,
//              sin p, the conversion +0x6C0DF0 tail-calls)
//   callers    the per-user control update (+0x6C0A46), the only one written after, and a per-shot recoil
//              kick (+0x6C47A0 tail-jumps in from weapon fire), which stays stock: the next control update
//              rewrites the pitch it kicked, so the aim follows the hand, not the recoil
// After the accumulator returns, the control update builds player_action from +0x38 (action+8) and submits
// it to the simulation; the per-player update then sets the unit's desired facing, aiming and looking from
// it. A write here therefore reaches the network and the body. After the control update it would be too
// late: the action is already built.
//
// ADDR-HYGIENE: guarded. Five exact-byte anchors, each unique in the module and at its RVA: the entry
// prologue (it also carries the record slot's disp32), the stride block, the yaw and the pitch
// accumulators, and the call site, whose rel32 must land on the entry (its return address is the only one
// written after). The slot must resolve inside the module. Any mismatch: no hook, said once, and Halo 2
// keeps its unit aim (the shot follows the hand; the body and the network do not).
// ARM GATE (ValueAgreement): installed disarmed. It writes only after the record has held the stock
// camera's direction (the game builds that camera from the record) within 1.7 deg for 30 updates -- both
// axes on foot, yaw in a seat -- and only while each later update resolves the same record. On foot it
// writes only while the observer publishes the view at its own heading (halo2_foot_view.h): written before
// that, the stock camera would turn the view with the hand, which the headset rejected as C-H2-41. SEATED
// (2026-10-01) it writes, axis by axis, only while the seat's own view is up and the seat's own record has
// passed its checks (the seated branch below).

constexpr uint32_t kHalo2PcEntryRva = 0x6C3DF0;
constexpr char kHalo2PcEntryPattern[] =
    "48 8B C4 F3 0F 11 50 18 89 48 08 55 53 57 48 8D A8 C8 FE FF FF 48 81 EC 20 02 00 00 "
    "48 8B 3D ?? ?? ?? ??";
constexpr uint32_t kHalo2PcSlotLoadOffset = 0x1C;   // `mov rdi, [rip+disp32]`, 7 bytes
constexpr uint32_t kHalo2PcStrideRva = 0x6C3E5E;
constexpr char kHalo2PcStridePattern[] =
    "48 63 C1 48 69 D8 B8 00 00 00 48 8B 05 ?? ?? ?? ?? 48 63 90 F4 00 00 00";
constexpr uint32_t kHalo2PcYawRva = 0x6C3FF8;
constexpr char kHalo2PcYawPattern[] =
    "4C 8D 34 3B F3 44 0F 10 05 ?? ?? ?? ?? 45 0F 57 E4 F2 44 0F 10 0D ?? ?? ?? ?? F3 41 0F 58 76 38 "
    "F3 0F 10 3D ?? ?? ?? ?? 44 0F 29 94 24 B0 01 00 00 44 0F 29 9C 24 A0 01 00 00 F3 41 0F 11 76 38";
constexpr uint32_t kHalo2PcPitchRva = 0x6C4740;
constexpr char kHalo2PcPitchPattern[] =
    "F3 0F 10 8D 50 01 00 00 F3 0F 58 D0 44 0F 28 AC 24 80 01 00 00 F3 0F 11 94 3B AC 00 00 00 "
    "F3 0F 58 4C 3B 3C 0F 2F D9 76 11 F3 0F 11 5C 3B 3C";
constexpr uint32_t kHalo2PcCallSiteRva = 0x6C0A27;
constexpr char kHalo2PcCallSitePattern[] =
    "41 0F BF 4F 28 E8 ?? ?? ?? ?? 84 C0 75 16 41 0F BF 4F 28 F3 0F 10 54 24 44 F3 0F 10 4C 24 40 "
    "E8 ?? ?? ?? ??";
constexpr uint32_t kHalo2PcCallOffset = 0x1F;       // the E8 that calls the accumulator
// ValueAgreement, as the other titles.
constexpr float kHalo2DriveAgreeRad = 0.03f;
constexpr uint32_t kHalo2DriveAgreeUpdates = 30;
// The native's own pitch clip, for a record whose eased limits read unusable.
constexpr float kHalo2DrivePitchClip = 1.4922565f;

// Halo yaw and pitch of a direction (world +Z up): forward = (cos y cos p, sin y cos p, sin p).
bool Halo2DriveYawPitch(const float direction[3], float& yaw, float& pitch) noexcept
{
    const float length = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] +
        direction[2] * direction[2]);
    if (!std::isfinite(length) || length < 1.0e-6f)
        return false;
    yaw = std::atan2(direction[1], direction[0]);
    pitch = std::asin(std::clamp(direction[2] / length, -1.0f, 1.0f));
    return std::isfinite(yaw) && std::isfinite(pitch);
}

// The local player's unit is a biped: -1 unknown (fails closed), 0 on foot, 1 seated (`parent`, `seat` set).
int Halo2DriveOccupancy(uint32_t& parent, int16_t& seat) noexcept
{
    parent = UINT32_MAX;
    seat = -1;
    if (!g_vehicleSeatVerified.load(std::memory_order_acquire))
        return -1;
    const uint32_t owned = Halo2OwnedUnit();
    if (owned == UINT32_MAX)
        return -1;
    const auto* unit = static_cast<const uint8_t*>(Halo2ObjectFromIndex(owned));
    if (!unit || unit[0xAA] != 0)
        return -1;
    seat = *reinterpret_cast<const int16_t*>(unit + kHalo2UnitParentSeatOffset);
    if (seat == -1)
        return 0;
    parent = *reinterpret_cast<const uint32_t*>(unit + 0x14);
    return seat >= 0 && parent != UINT32_MAX ? 1 : -1;
}

// The first writes of a seat's yaw read no limit off the record (the stick servo may still be deflecting the
// stick until the input thread sees the seat written), as the other titles' seats.
constexpr uint32_t kHalo2SeatPrimeUpdates = 2;

// Sim thread: the seat stint is over (left the seat, the seat stopped qualifying, a disarm or a fault).
void Halo2DriveForgetSeat(Halo2DriveState& st) noexcept
{
    st.stint.reset();
    st.stintActive = false;
    st.stintKey = 0;
    st.stintPitchFailCounted = false;
    st.stintSample = 0;
    st.yawLatch.reset();
    st.yawTicks = 0;
    st.seatYawMs.store(0, std::memory_order_release);
    st.seatPitchMs.store(0, std::memory_order_release);
}

void Halo2DriveApply(int user, uint64_t returnAddress) noexcept
{
    auto& st = g_halo2Drive;
    st.callbacks.fetch_add(1, std::memory_order_acq_rel);
    bool seatKept = false;    // this update continued a seat stint (its checks survive)
    bool seatWrote = false;   // ...and wrote it (the yaw latch keeps its last value only then)
    __try
    {
        do
        {
            if (user != static_cast<int>(kOwnedUser))
                break;
            if (returnAddress != st.controlReturn)
            {
                st.kicks.fetch_add(1, std::memory_order_relaxed);
                seatKept = st.stintActive;   // another caller: not an update of the seat, nor its end
                seatWrote = seatKept;
                break;
            }
            st.calls.fetch_add(1, std::memory_order_relaxed);
            if (!g_config.aim_direct_drive || !st.installed.load(std::memory_order_acquire) || !st.slot ||
                !g_armed.load(std::memory_order_acquire) || !g_levelLive.load(std::memory_order_acquire) ||
                g_teardownRequested.load(std::memory_order_acquire))
            {
                // Switched off live (or the observer is down): stop now, and re-validate before writing again.
                if (st.armed.exchange(false, std::memory_order_acq_rel))
                    st.disarms.fetch_add(1, std::memory_order_relaxed);
                st.agree = 0;
                break;
            }
            const uintptr_t globals = *reinterpret_cast<const uintptr_t*>(st.slot);
            if (!globals)
                break;
            const uintptr_t pc = globals + static_cast<uintptr_t>(user) * kHalo2PcStride;
            float* pair = reinterpret_cast<float*>(pc + kHalo2PcDesiredYaw);
            const float nativeYaw = pair[0], nativePitch = pair[1];
            st.nativeYaw.store(nativeYaw, std::memory_order_relaxed);
            st.nativePitch.store(nativePitch, std::memory_order_relaxed);
            uint32_t seatParent = UINT32_MAX;
            int16_t seatIndex = -1;
            const int occupancy = Halo2DriveOccupancy(seatParent, seatIndex);
            const uint64_t now = GetTickCount64();
            // The game's own aim: the stock camera, which the game builds from this record.
            Halo2ObserverPosePublication publication{};
            const uint32_t generation = g_generation.load(std::memory_order_acquire);
            const bool havePublication = generation && Halo2Observer6Dof_ReadPublishedPose(publication) &&
                publication.generation == generation;
            float aimYaw = 0.0f, aimPitch = 0.0f;
            const bool aimValid = havePublication && Halo2DriveYawPitch(publication.stock.forward, aimYaw, aimPitch);
            if (aimValid)
            {
                st.aimYaw.store(aimYaw, std::memory_order_relaxed);
                st.aimPitch.store(aimPitch, std::memory_order_relaxed);
            }
            if (occupancy != 0)
            {
                // SEATED. Every seat aims from this same record, written after this same call (mapped
                // 2026-10-01; the seat's limits are applied inside the accumulator). The hand owns the seat as
                // seat_drive::Decide says -- Halo 2's roles are not mapped, so every seat is taken as a driver
                // whose look steers: both axes at reach_vehicle_aim 2, the servo at 1. Never while the view
                // follows the vehicle: the hand's frame would turn with the hull it steers. And only while the
                // observer publishes THIS seat's own view (the vehicle reference, whose heading never reads the
                // native look): written without it, the stock camera -- built from this record -- would turn
                // the view with the hand.
                st.seated.fetch_add(1, std::memory_order_relaxed);
                const uint64_t seatViewAt = g_seatViewEngagedMs.load(std::memory_order_acquire);
                seat_drive::Seat facts{};
                facts.identified = occupancy > 0;
                // ...and the native first-person seat view placed THIS seat: the vehicle reference keeps the
                // stock camera's POSITION, which in a chase seat is pushed out along this very aim, so a write
                // there would swing the view round the hull (vehicle_first_person = 0 keeps the servo).
                facts.view_owned = havePublication && publication.vehicleReferenceValid &&
                    publication.vehicleUnit == Halo2OwnedUnit() && publication.vehicleParent == seatParent &&
                    publication.vehicleSeat == seatIndex && seatViewAt && now >= seatViewAt &&
                    now - seatViewAt <= 100 &&
                    NativeVehicleFirstPerson_SeatViewOwned(GameTitle::Halo2, seatParent, seatIndex);
                facts.driver = facts.look_steered = true;
                const seat_drive::Axes axes = g_config.vehicle_view_follow ? seat_drive::Axes{}
                    : seat_drive::Decide(facts, g_config.reach_vehicle_aim, g_config.seat_direct_drive != 0);
                if (!axes.any())
                {
                    st.agree = 0;
                    break;
                }
                seatKept = true;
                const uint32_t key = seat_drive::SeatKey(static_cast<int>(seatParent), seatIndex, false);
                if (!st.stintActive || st.stintKey != key)
                {
                    Halo2DriveForgetSeat(st);
                    st.stintActive = true;
                    st.stintKey = key;
                    st.seatStints.fetch_add(1, std::memory_order_relaxed);
                }
                const bool yawWas = st.stint.yaw_ok, pitchWas = st.stint.pitch_ok;
                const bool newSample = !havePublication || publication.index != st.stintSample;
                st.stintSample = havePublication ? publication.index : 0;
                st.stint.step(aimValid, nativeYaw, nativePitch, aimYaw, aimPitch, now, newSample);
                if (!yawWas && st.stint.yaw_ok)
                    st.seatYawChecks.fetch_add(1, std::memory_order_relaxed);
                if (!pitchWas && st.stint.pitch_ok)
                    st.seatPitchChecks.fetch_add(1, std::memory_order_relaxed);
                if (st.stint.pitch_failed && !st.stintPitchFailCounted)
                {
                    st.stintPitchFailCounted = true;
                    st.seatPitchFails.fetch_add(1, std::memory_order_relaxed);
                }
                if (!st.armed.load(std::memory_order_acquire))
                {
                    // A seat may arm the drive on yaw agreement alone (a level that starts in a vehicle).
                    const bool agrees = aimValid && std::isfinite(nativeYaw) &&
                        std::fabs(std::remainder(nativeYaw - aimYaw, 6.28318531f)) < kHalo2DriveAgreeRad;
                    st.agree = agrees ? st.agree + 1 : 0;
                    if (st.agree >= kHalo2DriveAgreeUpdates)
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
                // The hand's aim: the steer from the engine eye through the hand ray at the reticle distance.
                // Never a converged aim here: there is no seated trace yet, and an on-foot one could still be
                // fresh for 150 ms after sitting down.
                float direction[3]{};
                float yaw = 0.0f, pitch = 0.0f;
                const bool handAim = Game_Halo2ControllerAimActive() &&
                    BuildPublishedControllerAimDirection(direction, nullptr, false) &&
                    Halo2DriveYawPitch(direction, yaw, pitch);
                const bool writeYaw = axes.yaw && st.stint.yaw_ok && handAim;
                const bool writePitch = axes.pitch && st.stint.yaw_ok && st.stint.pitch_ok && handAim;
                if (!writeYaw && !writePitch)
                {
                    st.seatWaits.fetch_add(1, std::memory_order_relaxed);
                    break;
                }
                // Yaw: the seat-limit latch reads the seat's yaw clamp (a cone about the seat marker) off last
                // update's write and holds this one at it, after a short prime. Then [0, 2pi), as the native.
                float y = aim_write::unwrap_to_branch(yaw, nativeYaw);
                bool limited = false;
                if (writeYaw)
                {
                    const bool latch[2]{st.yawTicks >= kHalo2SeatPrimeUpdates, false};
                    const float native[2]{nativeYaw, nativePitch}, target[2]{y, nativePitch};
                    float held[2]{};
                    limited = st.yawLatch.apply(native, target, held, latch);
                    if (latch[0])
                        y = aim_write::unwrap_to_branch(held[0], nativeYaw);
                }
                y = std::fmod(y, 6.28318531f);
                if (y < 0.0f)
                    y += 6.28318531f;
                if (!(y < 6.28318531f))
                    y = 0.0f;
                // Pitch: exactly the record's own seat limits, which the accumulator has just eased toward the
                // seat camera's (hull tilt included). Never latched: the accumulator's speed-scaled autolevel
                // moves the pitch as the vehicle drives, which is not a limit.
                const float low = *reinterpret_cast<const float*>(pc + kHalo2PcPitchLow);
                const float high = *reinterpret_cast<const float*>(pc + kHalo2PcPitchHigh);
                const float p = std::isfinite(low) && std::isfinite(high) && low <= high
                    ? std::clamp(pitch, low, high)
                    : std::clamp(pitch, -kHalo2DrivePitchClip, kHalo2DrivePitchClip);
                if (!std::isfinite(y) || !std::isfinite(p))
                    break;
                if (writeYaw)
                    pair[0] = y;
                if (writePitch)
                    pair[1] = p;
                seatWrote = true;
                const float written[2]{y, p};
                const bool recorded[2]{writeYaw, false};
                st.yawLatch.wrote(written, recorded);
                st.yawTicks = writeYaw ? (st.yawTicks < kHalo2SeatPrimeUpdates ? st.yawTicks + 1 : st.yawTicks) : 0;
                st.seatYawMs.store(writeYaw ? now : 0, std::memory_order_release);
                st.seatPitchMs.store(writePitch ? now : 0, std::memory_order_release);
                st.seatWrites.fetch_add(1, std::memory_order_relaxed);
                if (limited)
                    st.seatHeld.fetch_add(1, std::memory_order_relaxed);
                st.wroteYaw.store(y, std::memory_order_relaxed);
                st.wrotePitch.store(p, std::memory_order_relaxed);
                st.writes.fetch_add(1, std::memory_order_relaxed);
                break;
            }
            // ON FOOT. The player's control update runs for an on-foot unit: the observer may take the view's
            // heading.
            st.footControlMs.store(now, std::memory_order_release);
            if (!st.armed.load(std::memory_order_acquire))
            {
                const bool agrees = aimValid && std::isfinite(nativeYaw) && std::isfinite(nativePitch) &&
                    std::fabs(std::remainder(nativeYaw - aimYaw, 6.28318531f)) < kHalo2DriveAgreeRad &&
                    std::fabs(nativePitch - aimPitch) < kHalo2DriveAgreeRad;
                st.agree = agrees ? st.agree + 1 : 0;
                if (st.agree >= kHalo2DriveAgreeUpdates)
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
                st.disarms.fetch_add(1, std::memory_order_relaxed);
                break;
            }
            // The hand's aim, while VR owns Halo 2's aim (Insert switches it off) and a direction can be built
            // (both controllers tracked). Stamped before the view is engaged too: the observer holds the
            // heading, and keeps the right stick from the game, only while the drive has something to write.
            float direction[3]{};
            bool converged = false;
            float yaw = 0.0f, pitch = 0.0f;
            const bool handAim = Game_Halo2ControllerAimActive() &&
                BuildPublishedControllerAimDirection(direction, &converged) &&
                Halo2DriveYawPitch(direction, yaw, pitch);
            if (handAim)
                st.handAimMs.store(now, std::memory_order_release);
            // Only once the view is published at its own heading: before that, the stock camera -- built
            // from this record -- would turn the view with the hand.
            const uint64_t engaged = g_footViewEngagedMs.load(std::memory_order_acquire);
            if (!handAim || !havePublication || !publication.footReferenceValid || !engaged ||
                now < engaged || now - engaged > 100)
            {
                st.waits.fetch_add(1, std::memory_order_relaxed);
                break;
            }
            // Inside the native's own invariants: yaw in [0, 2pi), pitch inside the record's eased limits,
            // and finite -- a NaN makes the simulation drop the whole action.
            yaw = std::fmod(yaw, 6.28318531f);
            if (yaw < 0.0f)
                yaw += 6.28318531f;
            if (!(yaw < 6.28318531f))
                yaw = 0.0f;
            const float low = *reinterpret_cast<const float*>(pc + kHalo2PcPitchLow);
            const float high = *reinterpret_cast<const float*>(pc + kHalo2PcPitchHigh);
            pitch = std::isfinite(low) && std::isfinite(high) && low <= high
                ? std::clamp(pitch, low, high)
                : std::clamp(pitch, -kHalo2DrivePitchClip, kHalo2DrivePitchClip);
            if (!std::isfinite(yaw) || !std::isfinite(pitch))
                break;
            pair[0] = yaw;
            pair[1] = pitch;
            st.wroteYaw.store(yaw, std::memory_order_relaxed);
            st.wrotePitch.store(pitch, std::memory_order_relaxed);
            st.writes.fetch_add(1, std::memory_order_relaxed);
            if (converged)
                st.convergedWrites.fetch_add(1, std::memory_order_relaxed);
            st.liveMs.store(now, std::memory_order_release);
        } while (false);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        // A fault disarms like a disarm: the record is proven again from a count of zero.
        if (st.armed.exchange(false, std::memory_order_acq_rel))
            st.disarms.fetch_add(1, std::memory_order_relaxed);
        st.agree = 0;
        seatKept = false;
        seatWrote = false;
        st.faults.fetch_add(1, std::memory_order_relaxed);
    }
    // A seat stint ends with any update that did not continue it (on foot, a seat the hand does not own, a
    // disarm, a fault); the yaw latch forgets with any update that did not write through it.
    if (!seatKept)
    {
        Halo2DriveForgetSeat(st);
    }
    else if (!seatWrote)
    {
        st.yawLatch.reset();
        st.yawTicks = 0;
        st.seatYawMs.store(0, std::memory_order_release);
        st.seatPitchMs.store(0, std::memory_order_release);
    }
    st.callbacks.fetch_sub(1, std::memory_order_acq_rel);
}

// Management thread, with the observer's install. Installed whatever the setting, so F1's Direct drive
// switches it live: it writes only while aim_direct_drive is on, and disarms whenever it is off.
bool InstallHalo2Drive(uintptr_t base, size_t size) noexcept
{
    auto& st = g_halo2Drive;
    if (st.target)
        return true;
    const auto at = [&](const char* pattern, uint32_t rva) {
        uintptr_t match = 0;
        uint32_t count = 0;
        return CountPatternMatches(base, size, pattern, match, count) && count == 1 && match == base + rva;
    };
    const bool entryOk = at(kHalo2PcEntryPattern, kHalo2PcEntryRva);
    const bool strideOk = at(kHalo2PcStridePattern, kHalo2PcStrideRva);
    const bool yawOk = at(kHalo2PcYawPattern, kHalo2PcYawRva);
    const bool pitchOk = at(kHalo2PcPitchPattern, kHalo2PcPitchRva);
    const bool callSiteOk = at(kHalo2PcCallSitePattern, kHalo2PcCallSiteRva);
    // The call site's rel32 must land on the entry; the instruction after it is the return address the drive
    // writes after. The record slot comes from the prologue's rip load and must lie inside the module.
    uintptr_t callTarget = 0, slot = 0;
    const uintptr_t call = base + kHalo2PcCallSiteRva + kHalo2PcCallOffset;
    const bool callOk = callSiteOk && *reinterpret_cast<const uint8_t*>(call) == 0xE8 &&
        DecodeRipRelative(call, 1, 5, callTarget) && callTarget == base + kHalo2PcEntryRva;
    const uintptr_t load = base + kHalo2PcEntryRva + kHalo2PcSlotLoadOffset;
    const bool slotOk = entryOk && DecodeRipRelative(load, 3, 7, slot) && slot >= base &&
        slot + sizeof(uintptr_t) <= base + size;
    if (!entryOk || !strideOk || !yawOk || !pitchOk || !callOk || !slotOk)
    {
        LOG("Halo 2 DRIVE: control update NOT hooked (anchors entry=%d stride=%d yaw=%d pitch=%d call=%d "
            "slot=%d at +0x%X); Halo 2 keeps its unit aim (the shot follows the hand; the body and the "
            "network do not)", entryOk ? 1 : 0, strideOk ? 1 : 0, yawOk ? 1 : 0, pitchOk ? 1 : 0,
            callOk ? 1 : 0, slotOk ? 1 : 0, kHalo2PcEntryRva);
        return false;
    }
    st.armed.store(false, std::memory_order_release);
    st.expected.store(0, std::memory_order_release);
    st.agree = 0;
    Halo2DriveForgetSeat(st);
    st.footControlMs.store(0, std::memory_order_release);
    st.handAimMs.store(0, std::memory_order_release);
    st.liveMs.store(0, std::memory_order_release);
    st.slot = slot;
    st.controlReturn = call + 5;
    void* const target = reinterpret_cast<void*>(base + kHalo2PcEntryRva);
    const MH_STATUS created = MH_CreateHook(target, reinterpret_cast<void*>(&Halo2PcHook), &g_halo2PcOrig);
    if (created != MH_OK || MH_EnableHook(target) != MH_OK)
    {
        if (created == MH_OK)
            (void)MH_RemoveHook(target);
        g_halo2PcOrig = nullptr;
        LOG("Halo 2 DRIVE: control-update hook FAILED (create=%d); Halo 2 keeps its unit aim",
            static_cast<int>(created));
        return false;
    }
    st.target = target;
    st.installed.store(true, std::memory_order_release);
    LOG("Halo 2 DRIVE: control-update hook INSTALLED at +0x%X (player control globals slot +0x%llX, stride "
        "0x%X, desired angles +0x%X; written after the call at +0x%X only); disarmed until the record agrees "
        "with the game's camera", kHalo2PcEntryRva, static_cast<unsigned long long>(slot - base),
        static_cast<unsigned>(kHalo2PcStride), static_cast<unsigned>(kHalo2PcDesiredYaw),
        kHalo2PcCallSiteRva + kHalo2PcCallOffset);
    return true;
}

bool RemoveHalo2Drive() noexcept
{
    auto& st = g_halo2Drive;
    st.installed.store(false, std::memory_order_release);
    st.armed.store(false, std::memory_order_release);
    if (!st.target)
        return true;
    const MH_STATUS disabled = MCCVR_DisableHookForRetirement(st.target);
    if (disabled != MH_OK && disabled != MH_ERROR_DISABLED && disabled != MH_ERROR_NOT_CREATED)
    {
        LOG("Halo 2 DRIVE: disable failed (%d); ownership retained for cleanup", static_cast<int>(disabled));
        return false;
    }
    // The asm thunk and the body it reaches. Not the extern "C" entry in between: it compiles to a bare tail
    // jump with no unwind entry, which the quiescence scan cannot resolve, so listing it failed every
    // retirement (verified in the built DLL). A thread there has already left the trampoline.
    const void* functions[]{reinterpret_cast<const void*>(&Halo2PcHook),
                            reinterpret_cast<const void*>(&Halo2DriveApply)};
    const void* trampolines[]{g_halo2PcOrig, nullptr};
    if (!WaitForNativeDetourQuiescence(functions, trampolines, 2, st.callbacks))
    {
        LOG("Halo 2 DRIVE: callbacks did not drain; ownership retained for cleanup");
        return false;
    }
    const MH_STATUS removed = MH_RemoveHook(st.target);
    if (removed != MH_OK && removed != MH_ERROR_NOT_CREATED)
    {
        LOG("Halo 2 DRIVE: remove failed (%d); ownership retained for cleanup", static_cast<int>(removed));
        return false;
    }
    st.target = nullptr;
    g_halo2PcOrig = nullptr;
    st.slot = 0;
    st.controlReturn = 0;
    return true;
}

// Cold worker, with the observer's telemetry.
void ReportHalo2Drive() noexcept
{
    auto& st = g_halo2Drive;
    const uint32_t calls = st.calls.exchange(0, std::memory_order_relaxed);
    const uint32_t kicks = st.kicks.exchange(0, std::memory_order_relaxed);
    if (!calls && !kicks)
        return;
    LOG("Halo 2 DRIVE: armed=%d updates %u, wrote %u (%u converged), waited for the view or the hand %u; "
        "recoil kicks left stock %u; arms %u, disarms %u, seated %u, faults %u; record yaw %.3f pitch %.3f, "
        "camera yaw %.3f pitch %.3f, last write yaw %.3f pitch %.3f; view heading taken %u, released %u, "
        "folded %u",
        st.armed.load(std::memory_order_acquire) ? 1 : 0, calls, st.writes.exchange(0),
        st.convergedWrites.exchange(0), st.waits.exchange(0), kicks, st.arms.exchange(0),
        st.disarms.exchange(0), st.seated.exchange(0), st.faults.exchange(0), st.nativeYaw.load(),
        st.nativePitch.load(), st.aimYaw.load(), st.aimPitch.load(), st.wroteYaw.load(), st.wrotePitch.load(),
        g_footViewEngages.exchange(0), g_footViewReleases.exchange(0), g_footViewFolds.exchange(0));
    const uint32_t stints = st.seatStints.exchange(0), seatWrites = st.seatWrites.exchange(0),
                   seatWaits = st.seatWaits.exchange(0), yawChecks = st.seatYawChecks.exchange(0),
                   pitchChecks = st.seatPitchChecks.exchange(0), pitchFails = st.seatPitchFails.exchange(0),
                   held = st.seatHeld.exchange(0);
    if (stints || seatWrites || seatWaits || yawChecks || pitchChecks || pitchFails || held)
        LOG("Halo 2 DRIVE seats: new stints %u; checks passed yaw %u pitch %u, pitch left on the servo %u; "
            "writes %u (yaw held at a seat limit %u), waited for the hand or a check %u",
            stints, yawChecks, pitchChecks, pitchFails, seatWrites, held, seatWaits);
}
