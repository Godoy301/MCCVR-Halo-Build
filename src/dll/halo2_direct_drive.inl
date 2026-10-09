// Included by halo2_observer_6dof.cpp inside its anonymous namespace, after CountPatternMatches. The drive's
// state and the accessors the observer reads (Halo2DriveState, g_halo2Drive, the record offsets) are
// declared earlier in that file.
//
// HALO 2 DIRECT DRIVE (aim_direct_drive, F1 > Weapon & Aim). Writing only the UNIT's aiming
// vectors (Halo2NativeAimUpdateDetour, +0x8FDF50) is downstream of player control, and falls short twice. The
// body's facing and the walk frame, which the game builds from player control's desired angles, stay
// wherever the thumbstick left them: legs up to 180 deg off the aim, and walking forward plays the
// backpedal. And nothing the game NETWORKS carries the aim, because player control's
// angles are what goes into the player_action the simulation submits. So the aim goes where Reach,
// Halo 3, ODST and Halo 4 put it: into player control's desired angles, right after the native accumulator.
// The unit-aim write stays as well, so the shot keeps following the hand without the unit's own easing.
//
// The accumulator (retail halo2.dll, identified by structure: the Halo 1 decomp's
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
// that, the stock camera would turn the view with the hand, which is unacceptable in a headset (C-H2-41). SEATED
// it writes, axis by axis, only while the seat's own view is up and the seat's own record has
// passed its checks (the seated branch below).
//
// THE SEAT'S FRAME (seat_frame_reader.h, docs/ALL-TITLES-AIM-2026-09-29.md "Seats aim in world angles").
// The per-player update hands the action's angles to 0x8F0D20 (from 0x6A40CD), which rotates them by the vehicle's
// up frame R(u) unless the occupied seat's live flags carry bit 4. No patch of ours clears that flag in Halo 2, so a
// seat without it lacks it in its own data, and every peer -- the host included -- applies R(u) to the angles we
// send: case (b). Each seated update therefore reads, on this thread and from the unit the rebase will rotate, the
// seat's flags and the vehicle's up exactly as the rebase does, and takes the game's aim (for the stint check and
// the arm gate) and the hand's aim (for the write) into the record's frame; the yaw latch and the record's own eased
// pitch limits then work in that frame too, as the accumulator's do. A frame that does not read writes nothing.

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

// The seat's frame (seat_frame_reader.h): bound with the hook, read on the sim thread each seated update.
seat_frame_rt::Binding g_halo2SeatFrame;
seat_frame_rt::Counters g_halo2SeatFrameCounters;

// AIM THE SEAT'S GUN THROUGH THE POINT (seat_barrel_logic.h). A vehicle gun's shot leaves along its barrel, so a seat
// aimed from its camera through the point fires PARALLEL to that line, beside the point by the barrel's distance
// from the camera. Each round a gun of the vehicle our unit aims fires shows the barrel's line (where the round
// starts and which way it flies, with the seat's eye from the same fire's camera answer); from those the model
// learns where the barrel sits in the vehicle, and the seat's write becomes the aim for which the barrel's line
// runs through the point. Until the seat's first round, and whenever the model declines, the write is the
// camera's line. The rounds are seen by the projectile witness, which only dev builds install: a player build
// keeps the camera's line.
// Only the direction written to player control changes -- what the title replicates -- and nothing of the shot is
// touched, so a host or a peer simulating the seat fires the same barrel at the same point.
// The fire and the control update both run on the simulation's thread; the try-lock covers the case they do not
// (the loser of a race keeps the camera's line for that update, or learns nothing from that fire).
struct Halo2SeatBarrelState
{
    std::atomic_flag busy = ATOMIC_FLAG_INIT;
    uint32_t stance = 0;
    bool gunner = false;          // this seat aims its vehicle's gun (the control update's read of the seat's flags)
    struct Gun
    {
        uint32_t weapon = UINT32_MAX;
        int16_t barrel = -1;
        seat_barrel::Model model;
        seat_barrel::Pointing pointing;   // the barrel settling a little off the aim: taken up from steady fires
    } guns[2];
    int active = -1;              // the gun fired last: the one the aim is solved for
    float fireEye[3]{};
    bool fireEyeKnown = false;    // this fire's camera answer was our seat's (taken by the fire's line, or dropped)
    float lastCameraAim[3]{};     // the last update's aim along the camera's line, and for how many updates in a
    uint32_t steady = 0;          // row it has kept still: only a fire of a steady aim teaches the pointing
    bool lastSolved = false;      // the last update's write was the barrel's (the residual is then the barrel's own)
    std::atomic<uint32_t> learned{0}, pointed{0}, unsteady{0}, aimed{0}, byEye{0}, declined{0}, noFrame{0};
    // The fire in progress: what the aim call showed, kept for the round that fire creates.
    struct Pending
    {
        bool valid = false, pointKnown = false, steady = false, barrelWrite = false;
        int slot = -1;
        uint32_t steadyUpdates = 0;
        seat_barrel::Frame hull{};
        float eye[3]{}, point[3]{}, aimOrigin[3]{}, aimDirection[3]{};
    } pending;
#if HALOMCCVR_DEV_DIAGNOSTICS
    // The last rounds, for the cold report (dev): what each showed.
    struct FireNote
    {
        float rangeWu = 0, residualRightDeg = 0, residualUpDeg = 0, pushRightDeg = 0, pushUpDeg = 0;
        float aimCallVsRoundDeg = -1, roundFromEyeWu = 0, aimCallFromEyeWu = 0, roundFromAimCallWu = 0, speedWu = 0;
        // The round's first collision step: the way it really flies, against where it was placed to point and
        // against the aim call's direction (-1: no step seen).
        float stepVsForwardDeg = -1, stepVsAimCallDeg = -1, stepStartWu = -1, stepLengthWu = 0;
        uint32_t steady = 0, modelFires = 0;
        int16_t barrel = -1;
        uint8_t taught = 0;       // 1: the pointing learned from it; 2: not steady; 3: no barrel write before it;
    } notes[8];                   // 4: residual past the limit or no frame
    uint32_t noteCount = 0, noteReported = 0;
    // The round waiting for its first collision step.
    std::atomic<bool> awaitingStep{false};
    uint32_t stepNote = 0;
    uint64_t stepRoundUs = 0;
    float stepSpawn[3]{}, stepForward[3]{}, stepAimCall[3]{};
#endif
} g_halo2SeatBarrel;
// The aim counts as steady when it turned less than this between two control updates, for this many in a row
// (about half a second): long enough for a turret to have settled on it.
constexpr float kHalo2SeatBarrelSteadyCos = 0.99999391f;   // cos(0.2 deg)
constexpr uint32_t kHalo2SeatBarrelSteadyUpdates = 24;

// Sim thread, every seated control update: which seat this is and whether it aims the vehicle's gun. Another seat
// forgets the guns.
void Halo2SeatBarrelNoteSeat(uint32_t stance, bool gunner) noexcept
{
    auto& s = g_halo2SeatBarrel;
    if (s.busy.test_and_set(std::memory_order_acquire))
        return;
    if (s.stance != stance)
    {
        for (auto& gun : s.guns)
            gun = {};
        s.active = -1;
        s.fireEyeKnown = false;
        s.steady = 0;
        s.lastSolved = false;
        s.stance = stance;
    }
    s.gunner = gunner;
    s.busy.clear(std::memory_order_release);
}

// Fire thread: the seat's eye as this fire's camera answer gave it (before the re-aim, which turns the line only).
void Halo2SeatBarrelNoteEye(const float* eye) noexcept
{
    auto& s = g_halo2SeatBarrel;
    if (!eye || !seat_barrel::Finite3(eye) || s.busy.test_and_set(std::memory_order_acquire))
        return;
    std::memcpy(s.fireEye, eye, sizeof(s.fireEye));
    s.fireEyeKnown = true;
    s.busy.clear(std::memory_order_release);
}

// Our unit is seated (parent, seat) and the weapon is a gun of that vehicle: held by something other than the unit,
// under the same root.
bool Halo2SeatBarrelSeatedGun(uint32_t unit, uint32_t weapon, uint32_t& parent, int16_t& seat) noexcept
{
    __try
    {
        if (Halo2DriveOccupancy(parent, seat) != 1)
            return false;
        const auto* data = static_cast<const uint8_t*>(Halo2ObjectFromIndex(weapon));
        if (!data)
            return false;
        const uint32_t holder = *reinterpret_cast<const uint32_t*>(data + 0x14);
        const uint32_t root = Halo2UltimateParent(unit);
        return holder != UINT32_MAX && holder != unit && root != UINT32_MAX && Halo2UltimateParent(weapon) == root;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Fire thread: one seated fire's aim call (the line the fire is given before the engine builds the round). It is
// NOT the round's path -- a Scorpion's shell leaves metres from it -- so nothing is learned from it: what the call
// showed is kept for the round this fire creates (Halo2SeatBarrelNoteRound).
void Halo2SeatBarrelLearn(uint32_t weapon, int16_t barrel, const float* origin, const float* direction,
                          const float* entryDirection) noexcept
{
    (void)entryDirection;
    auto& s = g_halo2SeatBarrel;
    const uint32_t unit = Halo2OwnedUnit();
    uint32_t parent = UINT32_MAX;
    int16_t seat = -1;
    float forward[3]{}, up[3]{};
    seat_barrel::Frame hull{};
    const bool usable = unit != UINT32_MAX && weapon != UINT32_MAX && origin && direction &&
        Halo2SeatBarrelSeatedGun(unit, weapon, parent, seat);
    const bool framed = usable &&
        seat_frame_rt::ReadVehicleFrame(g_halo2SeatFrame, &Halo2ObjectFromIndex, unit, forward, up) &&
        seat_barrel::MakeFrame(forward, up, hull);
    if (usable && !framed)
        s.noFrame.fetch_add(1, std::memory_order_relaxed);
    // The point this fire is aimed at (the seat's converged point now), taken before the lock: it solves the seat's
    // aim, which takes the lock itself.
    float point[3]{};
    const bool pointKnown = framed && Halo2SeatShotPoint(unit, point);
    if (s.busy.test_and_set(std::memory_order_acquire))
        return;
    const bool eyeKnown = s.fireEyeKnown;
    s.fireEyeKnown = false;   // one fire's eye serves one fire
    s.pending = {};
    if (framed && eyeKnown && s.gunner && s.stance == seat_converge::StanceKey(parent, seat))
    {
        int slot = -1;
        for (int i = 0; i < 2; ++i)
            if (s.guns[i].weapon == weapon && s.guns[i].barrel == barrel)
                slot = i;
        if (slot < 0)
        {
            slot = s.guns[0].weapon == UINT32_MAX ? 0 : s.guns[1].weapon == UINT32_MAX ? 1 : s.active == 0 ? 1 : 0;
            s.guns[slot] = {};
            s.guns[slot].weapon = weapon;
            s.guns[slot].barrel = barrel;
        }
        auto& p = s.pending;
        p.valid = true;
        p.slot = slot;
        p.hull = hull;
        p.pointKnown = pointKnown;
        p.steadyUpdates = s.steady;
        p.steady = s.steady >= kHalo2SeatBarrelSteadyUpdates;
        // The write before this fire was this gun's barrel: the round's residual is then the barrel's own pointing
        // and not its place, which the model takes.
        p.barrelWrite = s.active == slot && s.lastSolved && s.guns[slot].model.fires() != 0;
        std::memcpy(p.eye, s.fireEye, sizeof(p.eye));
        std::memcpy(p.point, point, sizeof(p.point));
        std::memcpy(p.aimOrigin, origin, sizeof(p.aimOrigin));
        std::memcpy(p.aimDirection, direction, sizeof(p.aimDirection));
    }
    s.busy.clear(std::memory_order_release);
}

// Fire thread: the round a seated fire created -- where it starts and which way it flies. This is the barrel. Its
// pointing is learned first (a steady aim, and the write before it already the barrel's), then its place.
// The caller is the projectile witness (object_new from the fire routine), which only dev builds install.
void Halo2SeatBarrelNoteRound(const float spawn[3], const float forward[3], const float* velocity) noexcept
{
    auto& s = g_halo2SeatBarrel;
    if (!spawn || !forward || !seat_barrel::Finite3(spawn))
        return;
    float launch[3]{forward[0], forward[1], forward[2]};
    float speed = 0.0f;
    if (velocity && seat_barrel::Finite3(velocity))
    {
        speed = std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]);
        if (speed > 1.0e-3f)
            for (int i = 0; i < 3; ++i)
                launch[i] = velocity[i] / speed;
    }
    if (!seat_barrel::Finite3(launch) || s.busy.test_and_set(std::memory_order_acquire))
        return;
    auto& p = s.pending;
    if (p.valid && p.slot >= 0)
    {
        p.valid = false;   // one round per aim call
        auto& gun = s.guns[p.slot];
        uint8_t taught = 3;
        if (p.pointKnown && p.barrelWrite)
        {
            if (!p.steady)
            {
                taught = 2;
                s.unsteady.fetch_add(1, std::memory_order_relaxed);
            }
            else if (gun.pointing.Learn(p.hull, spawn, launch, p.point))
            {
                taught = 1;
                s.pointed.fetch_add(1, std::memory_order_relaxed);
            }
            else
                taught = 4;
        }
#if HALOMCCVR_DEV_DIAGNOSTICS
        {
            auto& note = s.notes[s.noteCount % 8];
            note = {};
            note.barrel = gun.barrel;
            note.taught = taught;
            note.steady = p.steadyUpdates;
            note.modelFires = gun.model.fires();
            note.speedWu = speed;
            double r = 0.0, u = 0.0;
            if (p.pointKnown && seat_barrel::Pointing::Residual(p.hull, spawn, launch, p.point, r, u))
            {
                note.residualRightDeg = static_cast<float>(r / seat_barrel::kDegToRad);
                note.residualUpDeg = static_cast<float>(u / seat_barrel::kDegToRad);
                const float d[3]{p.point[0] - spawn[0], p.point[1] - spawn[1], p.point[2] - spawn[2]};
                note.rangeWu = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            }
            note.pushRightDeg = static_cast<float>(gun.pointing.right / seat_barrel::kDegToRad);
            note.pushUpDeg = static_cast<float>(gun.pointing.up / seat_barrel::kDegToRad);
            const float dot = p.aimDirection[0] * launch[0] + p.aimDirection[1] * launch[1] + p.aimDirection[2] * launch[2];
            note.aimCallVsRoundDeg = std::acos(std::clamp(dot, -1.0f, 1.0f)) * 57.29578f;
            const auto distance = [](const float a[3], const float b[3]) {
                const float d[3]{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
                return std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            };
            note.roundFromEyeWu = distance(spawn, p.eye);
            note.aimCallFromEyeWu = distance(p.aimOrigin, p.eye);
            note.roundFromAimCallWu = distance(spawn, p.aimOrigin);
            s.stepNote = s.noteCount % 8;
            s.stepRoundUs = Sight_NowUs();
            std::memcpy(s.stepSpawn, spawn, sizeof(s.stepSpawn));
            std::memcpy(s.stepForward, launch, sizeof(s.stepForward));
            std::memcpy(s.stepAimCall, p.aimDirection, sizeof(s.stepAimCall));
            s.awaitingStep.store(true, std::memory_order_release);
            ++s.noteCount;
        }
#endif
        if (gun.model.Learn(p.hull, p.eye, spawn, launch))
        {
            s.active = p.slot;
            s.learned.fetch_add(1, std::memory_order_relaxed);
        }
    }
    s.busy.clear(std::memory_order_release);
}

#if HALOMCCVR_DEV_DIAGNOSTICS
// Three floats from the engine's memory, guarded.
bool Halo2SeatBarrelCopy3(const float* from, float to[3]) noexcept
{
    __try
    {
        std::memcpy(to, from, 3 * sizeof(float));
        return true;
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

// Any thread that runs a collision query (dev): the first step of the round just created -- a query that starts at
// the round's spawn point and runs roughly the round's way -- says which way the round really flies.
void Halo2SeatBarrelNoteQuery(const float* start, const float* vector) noexcept
{
    auto& s = g_halo2SeatBarrel;
    if (!s.awaitingStep.load(std::memory_order_acquire) || !start || !vector)
        return;
    float a[3]{}, v[3]{};
    if (!Halo2SeatBarrelCopy3(start, a) || !Halo2SeatBarrelCopy3(vector, v))
        return;
    const float length = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!seat_barrel::Finite3(a) || !std::isfinite(length) || !(length > 0.01f) ||
        s.busy.test_and_set(std::memory_order_acquire))
        return;
    if (s.awaitingStep.load(std::memory_order_relaxed))
    {
        const float d[3]{a[0] - s.stepSpawn[0], a[1] - s.stepSpawn[1], a[2] - s.stepSpawn[2]};
        const float from = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        const auto angle = [&](const float w[3]) {
            const float dot = (v[0] * w[0] + v[1] * w[1] + v[2] * w[2]) / length;
            return std::acos(std::clamp(dot, -1.0f, 1.0f)) * 57.29578f;
        };
        const float vsForward = angle(s.stepForward), vsAimCall = angle(s.stepAimCall);
        if (Sight_NowUs() - s.stepRoundUs > 300000)
            s.awaitingStep.store(false, std::memory_order_release);   // no step came: the note keeps its -1
        else if (from <= 0.6f && (vsForward <= 25.0f || vsAimCall <= 25.0f))
        {
            auto& note = s.notes[s.stepNote];
            note.stepVsForwardDeg = vsForward;
            note.stepVsAimCallDeg = vsAimCall;
            note.stepStartWu = from;
            note.stepLengthWu = length;
            s.awaitingStep.store(false, std::memory_order_release);
        }
    }
    s.busy.clear(std::memory_order_release);
}
#endif

// Sim thread: `bent` (the aim from the seat's camera through `point`) becomes the aim for which the barrel's line
// runs through `point`, when a gun of this seat has fired and the model answers. Otherwise it is left as it is.
void Halo2SeatBarrelAim(uint32_t unit, uint32_t stance, const float eye[3], const float point[3],
                        float bent[3]) noexcept
{
    auto& s = g_halo2SeatBarrel;
    if (!g_config.seat_aim_through_barrel || s.busy.test_and_set(std::memory_order_acquire))
        return;
    // How long the aim has kept still, judged on the camera's line (what the hand asks for, before any push).
    const float turn = bent[0] * s.lastCameraAim[0] + bent[1] * s.lastCameraAim[1] + bent[2] * s.lastCameraAim[2];
    s.steady = turn >= kHalo2SeatBarrelSteadyCos ? s.steady + 1 : 0;
    std::memcpy(s.lastCameraAim, bent, sizeof(s.lastCameraAim));
    bool solved = false;
    if (s.stance == stance && s.gunner && s.active >= 0 && s.guns[s.active].model.fires())
    {
        float forward[3]{}, up[3]{}, aim[3]{};
        seat_barrel::Frame hull{};
        if (!seat_frame_rt::ReadVehicleFrame(g_halo2SeatFrame, &Halo2ObjectFromIndex, unit, forward, up) ||
            !seat_barrel::MakeFrame(forward, up, hull))
            s.noFrame.fetch_add(1, std::memory_order_relaxed);
        else if (!s.guns[s.active].model.Aim(hull, eye, point, aim))
            s.declined.fetch_add(1, std::memory_order_relaxed);
        else
        {
            s.guns[s.active].pointing.Apply(hull, aim);   // the barrel's own error at this heading
            std::memcpy(bent, aim, sizeof(aim));
            solved = true;
        }
    }
    s.lastSolved = solved;
    s.busy.clear(std::memory_order_release);
    (solved ? s.aimed : s.byEye).fetch_add(1, std::memory_order_relaxed);
}

// SEAT CONVERGENCE (seat_converge_logic.h; docs/H2-CE-SEAT-CONVERGENCE-2026-10-06.md). Halo 2 aims a seated shot as
// Halo 1 does: the fire (0x8E4940) takes the weapon's barrel marker when the weapon's owner has a gunner, else the
// unit's aim, and player_aim_projectile (0x7596A0) then traces the crosshair from the DETERMINISTIC camera of the
// player's biped (0x6D4730: first person 0x77DAA0 -- the unit camera position 0x8F9190 along the unit's aiming vector
// +0x174 -- when the selector 0x6D4C50 says so, which the mod's first-person seat view makes it say), ignoring the
// aiming unit's ultimate parent (0x8D6A50), and turns the shot at what that hits -- in a seat pinned inside the
// weapon's deviation cone about the direction it came in with (0x759A89..0x759ADE; on foot it is not pinned). The
// biped's aiming vector comes from player control's desired angles, which is what this drive writes and what the
// simulation networks. So the seat's write is the direction from that camera position through the traced point --
// or, for a vehicle gun that has fired, the direction for which its BARREL runs through that point (above).
//
// The eye is asked from the engine's own evaluator in this same update (Halo2CallFireEyeEvaluator: the very call
// the fire makes, call edge proven by the dual-aim install) and used only on its first-person path
// (seat_converge::SeatEye): the following camera orbits the aim it is built from.
void Halo2NoteSeat(bool identified, uint32_t stance, const seat_converge::Role& role, seat_converge::Why why) noexcept
{
    if (identified)
        Sight_NoteSeatPoll(GameTitle::Halo2, stance, role, why);
}

// The seat's intent ray from one observer publication (the rendered centre eye along the aiming hand's carrier, the
// same carrier choice as on foot). False: no usable controller publication.
bool Halo2SeatIntentRay(float head[3], float dir[3]) noexcept
{
    Halo2ObserverPosePublication publication{};
    Halo2CameraBasis rightCarrier{}, leftCarrier{};
    const uint32_t generation = g_generation.load(std::memory_order_acquire);
    float stockEye[3]{};
    return generation && Halo2Observer6Dof_ReadPublishedPose(publication) &&
        Halo2ObserverControllerSnapshotUsable(publication, generation) &&
        BuildStableFirstPersonCarriers(publication, generation, rightCarrier, leftCarrier,
            Halo2SupportCarrierMustDetach(publication)) &&
        Halo2SightGeometry(publication, rightCarrier, head, dir, stockEye);
}

// Sim thread. `direction`: the converged aim (world, unit) when the result is Converged; `point` (optional): the
// point it is bent through. Any other result names our reason and leaves both untouched.
seat_converge::Why Halo2BuildSeatConvergedAim(uint32_t unit, uint32_t stance, float direction[3],
                                              float* point) noexcept
{
    using seat_converge::Why;
    if (!Sight_AimEnabled())
        return Why::AimOff;
    float head[3]{}, dir[3]{};
    if (unit == UINT32_MAX || !Halo2SeatIntentRay(head, dir))
        return Why::NoHand;
    seat_converge::EyeAnswer answer{};
    auto& camera = g_halo2Dual;
    answer.bound = camera.cameraOriginal != nullptr && camera.enabled.load(std::memory_order_acquire) &&
        !camera.faulted.load(std::memory_order_acquire) &&
        camera.generation == g_generation.load(std::memory_order_acquire);
    int32_t path = -1;
    if (answer.bound)
    {
        answer.called = Halo2CallFireEyeEvaluator(unit, answer.position, path);
        answer.perspective = path;
    }
    float eye[3]{};
    const Why eyeWhy = seat_converge::SeatEye(answer, eye);
    if (eyeWhy != Why::Converged)
        return eyeWhy;
    if (Sight_Stance(GameTitle::Halo2) != stance)
        return Why::StanceSettling;
    float bent[3]{}, range = 0.0f;
    converge::ConvergeOff off = converge::ConvergeOff::None;
    if (!Sight_ConvergedDirection(GameTitle::Halo2, g_generation.load(std::memory_order_acquire), head, dir, eye,
                                  bent, 90.0f, &range, nullptr, &off))
        return off == converge::ConvergeOff::NoRange ? Why::NoRange : Why::NoTrace;
    if (!std::isfinite(range) || !(range > 0.0f))
        return Why::NoRange;
    const float aimPoint[3]{head[0] + dir[0] * range, head[1] + dir[1] * range, head[2] + dir[2] * range};
    Halo2SeatBarrelAim(unit, stance, eye, aimPoint, bent);
    for (int axis = 0; axis < 3; ++axis)
    {
        direction[axis] = bent[axis];
        if (point)
            point[axis] = aimPoint[axis];
    }
    return Why::Converged;
}

// The first writes of a seat's yaw read no limit off the record (the stick servo may still be deflecting the
// stick until the input thread sees the seat written), as the other titles' seats.
constexpr uint32_t kHalo2SeatPrimeUpdates = 2;
// A stint survives this many consecutive updates with its seat's view down (~4 s of 30 Hz control updates).
constexpr uint32_t kHalo2StintStaleUpdates = 120;

// Sim thread: the seat stint is over (left the seat, the seat stopped qualifying, a disarm or a fault).
void Halo2DriveForgetSeat(Halo2DriveState& st) noexcept
{
    st.stint.reset();
    st.stintActive = false;
    st.stintKey = 0;
    st.stintPitchFailCounted = false;
    st.stintSample = 0;
    st.stintStaleUpdates = 0;
    st.yawLatch.reset();
    st.yawTicks = 0;
    st.seatYawMs.store(0, std::memory_order_release);
    st.seatPitchMs.store(0, std::memory_order_release);
    st.stops.clear();
    st.stintAircraft = false;
}

// An air vehicle (Halo 2's own identity of the seat's vehicle): no rotational limit there, so its seats never clamp
// the reticle. Unknown identity: not an aircraft (a stop it never shows clamps nothing).
bool Halo2SeatIsAircraft(uint32_t parent) noexcept
{
    const char* name = halo2_vehicle_identity::Name(ReadHalo2VehicleIdentity(parent));
    if (!name)
        return false;
    const auto starts = [name](const char* prefix) { return std::strncmp(name, prefix, std::strlen(prefix)) == 0; };
    return starts("Banshee") || starts("Phantom") || starts("Pelican") || starts("Falcon") || starts("Longsword");
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
            // The game ran the owned user's control update (it skips this call while the director holds the
            // player: a cinematic). One reader: the head-relative motion tracker (halo2_head_relative.inl).
            st.controlMs.store(GetTickCount64(), std::memory_order_release);
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
                // SEATED. Every seat aims from this same record, written after this same call (the
                // seat's limits are applied inside the accumulator). The hand owns the seat as
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
                // THE SEAT'S FRAME AND ROLE, read as the rebase reads them (see the header): the occupied seat's live
                // flags word carries the role too (seat_converge_logic.h: bit 2 driver, bit 3 gunner -- the engine's
                // own sibling predicates 0x8FC99C / 0x8FCA5C and its aiming-unit test 0x8F8C00). A seat that does NOT
                // drive (a gunner, a passenger) is the hand's at "Drivers aim with the hand" OFF as well; a driver, or
                // a seat whose flags did not read, is taken as a driver whose look steers.
                seat_frame_rt::Live seatFrame =
                    seat_frame_rt::Read(g_halo2SeatFrame, &Halo2ObjectFromIndex, Halo2OwnedUnit());
                const seat_converge::Role seatRole = seat_converge::RoleFromSeatFlags(
                    occupancy > 0 && seatFrame.flagsKnown, seatFrame.flags);
                facts.driver = facts.look_steered = !seatRole.known || seatRole.driver;
                const seat_drive::Axes axes = g_config.vehicle_view_follow ? seat_drive::Axes{}
                    : seat_drive::Decide(facts, g_config.reach_vehicle_aim, g_config.seat_direct_drive != 0);
                const uint32_t key = seat_drive::SeatKey(static_cast<int>(seatParent), seatIndex, false);
                // The seat's account for the SIGHT ... seat line (every update that does not end in a converged
                // write says why -- our reasons only).
                const uint32_t stance = seat_converge::StanceKey(seatParent, seatIndex);
                const bool seatKnown = occupancy > 0;
                Halo2SeatBarrelNoteSeat(stance, seatKnown && seatRole.known && seatRole.gunner);
                // THE GAME'S AIM IN A SEAT IS THE UNIT'S OWN AIMING VECTOR (seat_converge::SeatAimReference). The
                // observer camera's forward -- the right reference on foot -- sits a constant ~5.1 deg of pitch above
                // the unit's aim in a seat (measured in the Ghost and as a Warthog passenger, at every heading and
                // pitch), so against it the pitch proof below never passes, the pitch stays on the stick servo and no
                // seat ever writes both axes. The unit's aiming vector is the line the
                // engine fires along and equals the record to 0.02 deg; it is read here, on the sim thread, from the
                // same unit the occupancy read above (one tick behind the record: the proof's motion term covers it).
                // A vector that does not read as one leaves the camera's forward as the reference.
                float unitAim[3]{};
                bool unitAimRead = false;
                if (occupancy > 0)
                {
                    const auto* owned = static_cast<const uint8_t*>(Halo2ObjectFromIndex(Halo2OwnedUnit()));
                    if (owned && owned[0xAA] == 0)
                    {
                        std::memcpy(unitAim, owned + kHalo2UnitAimingVectorOffset, sizeof(unitAim));
                        unitAimRead = true;
                    }
                }
                const seat_converge::AimReference seatAim =
                    seat_converge::SeatAimReference(unitAimRead, unitAim, aimValid, aimYaw, aimPitch);
                if (seatAim.valid)
                {
                    (seatAim.fromUnit ? st.seatUnitAimRefs : st.seatCameraRefs).fetch_add(1, std::memory_order_relaxed);
                    if (std::isfinite(seatAim.cameraOffDeg) &&
                        seatAim.cameraOffDeg > st.seatCameraOffMaxDeg.load(std::memory_order_relaxed))
                        st.seatCameraOffMaxDeg.store(seatAim.cameraOffDeg, std::memory_order_relaxed);
                }
                // In case (b) the game's aim goes into the record's frame for the stint check and the arm gate, and
                // the hand's below.
                float recordAimYaw = seatAim.yaw, recordAimPitch = seatAim.pitch;
                bool frameRead = true;
                if (axes.any())
                {
                    seat_frame_rt::Count(g_halo2SeatFrameCounters, seatFrame.kase);
#if HALOMCCVR_DEV_DIAGNOSTICS
                    // THE FRAME CHECK: the record through R against the game's world aim (the unit's aiming vector;
                    // the observer camera only where that did not read).
                    if (seatAim.valid)
                        seat_frame_rt::NoteFrameCheck(g_halo2SeatFrameCounters, seatFrame, nativeYaw, nativePitch,
                                                      seatAim.yaw, seatAim.pitch);
#endif
                    frameRead = seat_frame_rt::ToRecord(seatFrame, recordAimYaw, recordAimPitch, nativeYaw);
                }
                if (!frameRead)
                {
                    // No frame this update: nothing is written, the servo keeps the seat; the stint survives it as
                    // it survives a gap in the seat's view (bounded).
                    if (facts.identified && st.stintActive && st.stintKey == key &&
                        ++st.stintStaleUpdates <= kHalo2StintStaleUpdates)
                        seatKept = true;
                    st.agree = 0;
                    Halo2NoteSeat(seatKnown, stance, seatRole, seat_converge::Why::NotWritten);
                    break;
                }
                if (!axes.any())
                {
                    // A GAP in this stint's seat (the F1 menu, a pause and its fade, the theater: the observer stops
                    // publishing the seat's view) keeps the proven checks for a bounded run of updates, writing
                    // nothing -- when the seat, its policy and everything but the view are unchanged.
                    seat_drive::Seat viewed = facts;
                    viewed.view_owned = true;
                    const bool onlyTheView = !facts.view_owned && !g_config.vehicle_view_follow &&
                        seat_drive::Decide(viewed, g_config.reach_vehicle_aim, g_config.seat_direct_drive != 0).any();
                    if (onlyTheView && facts.identified && st.stintActive && st.stintKey == key &&
                        ++st.stintStaleUpdates <= kHalo2StintStaleUpdates)
                        seatKept = true;
                    st.agree = 0;
                    // The stick keeps this seat: its first-person view is not ours (the engine aims it from the
                    // chase camera), the view follows the vehicle, or the player's keys leave it to the stick.
                    Halo2NoteSeat(seatKnown, stance, seatRole,
                        !facts.identified ? seat_converge::Why::NotIdentified
                        : onlyTheView ? seat_converge::Why::ChaseCamera : seat_converge::Why::PolicyOff);
                    break;
                }
                seatKept = true;
                st.stintStaleUpdates = 0;
                if (!st.stintActive || st.stintKey != key)
                {
                    Halo2DriveForgetSeat(st);
                    st.stintActive = true;
                    st.stintKey = key;
                    st.stintAircraft = Halo2SeatIsAircraft(seatParent);
                    st.seatStints.fetch_add(1, std::memory_order_relaxed);
                }
                const bool yawWas = st.stint.yaw_ok, pitchWas = st.stint.pitch_ok;
                // The unit's aiming vector is read fresh in this very update (every one is a sample of its own; a
                // record that moved since the unit last ticked is motion the proof allows for). The camera is a
                // publication: a repeated one is no evidence.
                const bool newSample = seatAim.fromUnit || !havePublication || publication.index != st.stintSample;
                st.stintSample = havePublication ? publication.index : 0;
                st.stint.step(seatAim.valid, nativeYaw, nativePitch, recordAimYaw, recordAimPitch, now, newSample);
                if (!yawWas && st.stint.yaw_ok)
                    st.seatYawChecks.fetch_add(1, std::memory_order_relaxed);
                if (!pitchWas && st.stint.pitch_ok)
                    st.seatPitchChecks.fetch_add(1, std::memory_order_relaxed);
                if (st.stint.pitch_failed && !st.stintPitchFailCounted)
                {
                    st.stintPitchFailCounted = true;   // each failed attempt once (the check retries every 5 s)
                    st.seatPitchFails.fetch_add(1, std::memory_order_relaxed);
                }
                else if (!st.stint.pitch_failed)
                    st.stintPitchFailCounted = false;
                if (!st.armed.load(std::memory_order_acquire))
                {
                    // A seat may arm the drive on yaw agreement alone (a level that starts in a vehicle).
                    const bool agrees = seatAim.valid && std::isfinite(nativeYaw) &&
                        std::fabs(std::remainder(nativeYaw - recordAimYaw, 6.28318531f)) < kHalo2DriveAgreeRad;
                    st.agree = agrees ? st.agree + 1 : 0;
                    if (st.agree >= kHalo2DriveAgreeUpdates)
                    {
                        st.expected.store(pc, std::memory_order_release);
                        st.armed.store(true, std::memory_order_release);
                        st.arms.fetch_add(1, std::memory_order_relaxed);
                    }
                    Halo2NoteSeat(seatKnown, stance, seatRole, seat_converge::Why::NotWritten);
                    break;
                }
                if (pc != st.expected.load(std::memory_order_acquire))
                {
                    st.armed.store(false, std::memory_order_release);
                    st.agree = 0;
                    seatKept = false;
                    st.disarms.fetch_add(1, std::memory_order_relaxed);
                    Halo2NoteSeat(seatKnown, stance, seatRole, seat_converge::Why::NotWritten);
                    break;
                }
                // The hand's aim. TRACE-AND-REAIM IN THE SEAT (seat_converge_logic.h): the direction
                // from Halo 2's own deterministic camera for this biped -- the origin of its crosshair test at the
                // fire (0x6D4730 from 0x7597B8) -- through the point the hand's ray hits, as this seat's own trace
                // found it (the stance check inside: never a range measured on foot, nor in another seat). Any of
                // our own reasons falls back to the plain steer: the stock camera's line through the hand ray at the
                // reticle distance.
                float direction[3]{};
                float yaw = 0.0f, pitch = 0.0f, handWorldYaw = 0.0f, handWorldPitch = 0.0f;
                seat_converge::Why seatWhy = seat_converge::Why::NoHand;
                bool handAim = Game_Halo2SeatAimActive();
                if (handAim)
                {
                    seatWhy = Halo2BuildSeatConvergedAim(Halo2OwnedUnit(), stance, direction, nullptr);
                    if (seatWhy != seat_converge::Why::Converged)
                        handAim = BuildPublishedControllerAimDirection(direction, nullptr, false);
                }
                const bool seatConverged = handAim && seatWhy == seat_converge::Why::Converged;
                // ...in the record's frame (case b: R^-1 of it), where the latch and the pitch limits live.
                handAim = handAim && Halo2DriveYawPitch(direction, yaw, pitch);
                handWorldYaw = yaw;
                handWorldPitch = pitch;
                handAim = handAim && seat_frame_rt::ToRecord(seatFrame, yaw, pitch, nativeYaw);
                const bool writeYaw = axes.yaw && st.stint.yaw_ok && handAim;
                const bool writePitch = axes.pitch && st.stint.yaw_ok && st.stint.pitch_ok && handAim;
                // A YAW-ONLY write in case (b) keeps the record's pitch: convert the hand's heading at the pitch the
                // game will aim at (seat_frame::RecordYawForWorldYaw), not at the hand's.
                if (writeYaw && !writePitch && seatFrame.kase == seat_frame::Case::Rebased)
                {
                    float yawOnly = 0.0f;
                    if (seat_frame::RecordYawForWorldYaw(seatFrame.frame, handWorldYaw, nativeYaw, nativePitch, yawOnly))
                        yaw = yawOnly;
                }
                if (!writeYaw && !writePitch)
                {
                    st.seatWaits.fetch_add(1, std::memory_order_relaxed);
                    Halo2NoteSeat(seatKnown, stance, seatRole,
                        seatConverged ? seat_converge::Why::NotWritten
                        : seatWhy == seat_converge::Why::Converged ? seat_converge::Why::NoHand : seatWhy);
                    break;
                }
                // Yaw: the seat-limit latch reads the seat's yaw clamp (a cone about the seat marker) off last
                // update's write and holds this one at it, after a short prime. Then [0, 2pi), as the native.
                float y = aim_write::unwrap_to_branch(yaw, nativeYaw);
                bool limited = false;
                bool yawLatchRead[2]{};
                if (writeYaw)
                {
                    const bool latch[2]{st.yawTicks >= kHalo2SeatPrimeUpdates, false};
                    yawLatchRead[0] = latch[0];
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
                // The stops the reticle clamps against (seat_limit_reticle_logic.h), with the hand's world aim: the yaw
                // latch's, and the record's own eased pitch limits -- the accumulator clamps the pitch to them whoever
                // moves it, the write or the stick servo. Not in an air vehicle.
                if (st.stintAircraft)
                    st.stops.clear();
                else
                {
                    seat_limit::Stops stops{};
                    seat_limit::AddLatch(stops, st.yawLatch, yawLatchRead);
                    if (std::isfinite(low) && std::isfinite(high) && low <= high)
                        seat_limit::AddInterval(stops, 1, low, high);
                    stops.rebased = seatFrame.kase == seat_frame::Case::Rebased;
                    stops.frame = seatFrame.frame;
                    st.stops.publish(key, stops, nativeYaw, now, handWorldYaw, handWorldPitch);
                }
                st.seatWrites.fetch_add(1, std::memory_order_relaxed);
                if (limited)
                    st.seatHeld.fetch_add(1, std::memory_order_relaxed);
                // Reported as the WORLD angles the write put into effect (R e(record) in case b; an axis not written is
                // the record's own), so "last write" prints in the same frame as the camera's aim beside it.
                float worldYaw = writeYaw ? y : nativeYaw, worldPitch = writePitch ? p : nativePitch;
                seat_frame::Angles world{};
                if (seatFrame.kase == seat_frame::Case::Rebased &&
                    seat_frame::WorldFromRecord(seatFrame.frame, worldYaw, worldPitch, world))
                {
                    worldYaw = world.yaw;
                    worldPitch = world.pitch;
                }
                st.wroteYaw.store(worldYaw, std::memory_order_relaxed);
                st.wrotePitch.store(worldPitch, std::memory_order_relaxed);
                st.writes.fetch_add(1, std::memory_order_relaxed);
                // The aim now in the record is the converged one when BOTH axes carry it: the compositor draws the
                // traced reticle and the fire's backstop re-aims the engine's crosshair line. A stop the record or
                // the yaw latch held it at is natural (the reticle's own clamp shows it: seat_limit_reticle_logic.h)
                // and never takes the claim back; one axis still on the stick servo is not converged yet.
                if (seatConverged && writeYaw && writePitch)
                {
                    st.seatConvergedWrites.fetch_add(1, std::memory_order_relaxed);
                    st.convergedWrites.fetch_add(1, std::memory_order_relaxed);
                    Sight_MarkConverged(GameTitle::Halo2, Game_GetWorldScale());
                    Sight_MarkAimWritten(GameTitle::Halo2);
                    Halo2NoteSeat(seatKnown, stance, seatRole, seat_converge::Why::Converged);
                }
                else
                    Halo2NoteSeat(seatKnown, stance, seatRole,
                        seatConverged ? seat_converge::Why::NotWritten : seatWhy);
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
        st.stops.clear();
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
    st.controlMs.store(0, std::memory_order_release);
    st.slot = slot;
    st.controlReturn = call + 5;
    // The seat's frame: the vehicle-tilt rebase 0x8F0D20's gate and up getter, proven here (seat_frame_reader.h).
    const char* frameWhy = nullptr;
    bool frameBound = seat_frame_rt::Bind(g_halo2SeatFrame, seat_frame::kHalo2Reader, base, size, frameWhy);
    if (frameBound && g_halo2SeatFrame.objectsArray != base + kHalo2ObjectsDataArrayPointerRva)
    {
        // The reader resolves objects through Halo2ObjectFromIndex: it must be the array the rebase walks.
        seat_frame_rt::Unbind(g_halo2SeatFrame);
        frameBound = false;
        frameWhy = "its objects array is not the one the proven accessor reads";
    }
    g_halo2SeatFrameCounters.unknownLogged.store(0, std::memory_order_relaxed);
    if (frameBound)
        LOG("Halo 2 DRIVE: seat frame reader bound (the rebase +0x%X: its gate +0x%X and up getter +0x%X proven, "
            "objects through the proven accessor): a seated write follows the vehicle-tilt rebase (world / b)",
            seat_frame::kHalo2Reader.rebaseRva, seat_frame::kHalo2Reader.gateRva, seat_frame::kHalo2Reader.getterRva);
    else
        LOG("Halo 2 DRIVE: seat frame reader NOT bound (%s at +0x%X); seats are never written (the stick servo keeps "
            "every seat)", frameWhy ? frameWhy : "unknown", seat_frame::kHalo2Reader.gateRva);
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
    seat_frame_rt::Unbind(g_halo2SeatFrame);
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
    char frames[96];
    const bool unread = seat_frame_rt::CaseText(g_halo2SeatFrameCounters, frames, sizeof(frames));
    if (stints || seatWrites || seatWaits || yawChecks || pitchChecks || pitchFails || held || frames[0])
        LOG("Halo 2 DRIVE seats: new stints %u; checks passed yaw %u pitch %u, pitch checks failed %u (retried after 5 s); "
            "writes %u (both axes converged on the traced point %u; yaw held at a seat limit %u), waited for the hand "
            "or a check %u; %s",
            stints, yawChecks, pitchChecks, pitchFails, seatWrites, st.seatConvergedWrites.exchange(0), held,
            seatWaits, frames[0] ? frames : "frame not read");
    // The gun's barrel aimed through the point (seat_barrel_logic.h): how often, from how many fires, and why not.
    auto& barrel = g_halo2SeatBarrel;
    const uint32_t barrelAimed = barrel.aimed.exchange(0, std::memory_order_relaxed),
                   barrelByEye = barrel.byEye.exchange(0, std::memory_order_relaxed),
                   barrelLearned = barrel.learned.exchange(0, std::memory_order_relaxed),
                   barrelDeclined = barrel.declined.exchange(0, std::memory_order_relaxed),
                   barrelNoFrame = barrel.noFrame.exchange(0, std::memory_order_relaxed),
                   barrelPointed = barrel.pointed.exchange(0, std::memory_order_relaxed),
                   barrelUnsteady = barrel.unsteady.exchange(0, std::memory_order_relaxed);
    if (barrelAimed || barrelLearned || barrelDeclined || barrelNoFrame)
        LOG("Halo 2 DRIVE seats: the gun's barrel was aimed through the point on %u updates (its place learned from "
            "%u rounds in this window, its pointing corrected from %u; %u rounds came while the aim was moving and "
            "corrected nothing); the seat's camera line was kept on %u (no fire of this seat's gun yet, or: the "
            "point too near the barrel or too far off the camera's line %u, the vehicle's frame unread %u)",
            barrelAimed, barrelLearned, barrelPointed, barrelUnsteady, barrelByEye, barrelDeclined, barrelNoFrame);
#if HALOMCCVR_DEV_DIAGNOSTICS
    {
        // One line per round since the last report (the newest eight).
        Halo2SeatBarrelState::FireNote notes[8];
        uint32_t count = 0, reported = 0;
        if (!barrel.busy.test_and_set(std::memory_order_acquire))
        {
            count = barrel.noteCount;
            if (count && barrel.awaitingStep.load(std::memory_order_acquire))
                --count;   // the newest round still waits for its first step: next report
            reported = barrel.noteReported;
            std::memcpy(notes, barrel.notes, sizeof(notes));
            barrel.noteReported = count;
            barrel.busy.clear(std::memory_order_release);
        }
        static const char* const kTaught[]{"?", "pointing corrected from it", "the aim was moving: not used",
            "no barrel write before it: its place only", "residual past the limit: not used"};
        for (uint32_t i = count - reported > 8 ? count - 8 : reported; i < count; ++i)
        {
            const auto& n = notes[i % 8];
            LOG("Halo 2 DRIVE seats (dev) gun round %u: barrel %d, point at %.1f wu; the round against the point: right "
                "%+.2f up %+.2f deg (%s; steady %u updates); the push is now right %+.2f up %+.2f deg; the round "
                "starts %.2f wu from the seat's eye and %.2f wu from the aim call's origin (itself %.2f wu from the "
                "eye), placed pointing %.2f deg off the aim call's direction; ITS FIRST STEP (how it really flies; "
                "-1 = none seen) runs %.2f deg off where it was placed to point and %.2f deg off the aim call's "
                "direction, starting %.2f wu from the spawn point, %.2f wu long; %u rounds in the place model",
                i + 1, n.barrel, n.rangeWu, n.residualRightDeg, n.residualUpDeg, kTaught[n.taught < 5 ? n.taught : 0],
                n.steady, n.pushRightDeg, n.pushUpDeg, n.roundFromEyeWu, n.roundFromAimCallWu, n.aimCallFromEyeWu,
                n.aimCallVsRoundDeg, n.stepVsForwardDeg, n.stepVsAimCallDeg, n.stepStartWu, n.stepLengthWu,
                n.modelFires);
        }
    }
#endif
    // What the seat's proof was judged against (seat_converge::SeatAimReference), in every build: the unit's own
    // aiming vector, and how far the observer camera's forward sat off it.
    const uint32_t unitRefs = st.seatUnitAimRefs.exchange(0, std::memory_order_relaxed),
                   cameraRefs = st.seatCameraRefs.exchange(0, std::memory_order_relaxed);
    const float cameraOff = st.seatCameraOffMaxDeg.exchange(0.0f, std::memory_order_relaxed);
    if (unitRefs || cameraRefs)
        LOG("Halo 2 DRIVE seats: the seat's record was judged against the unit's own aiming vector on %u updates (the "
            "observer camera's forward sat up to %.2f deg off it: the camera is not a seat's aim) and against the "
            "observer camera on %u (the vector did not read)",
            unitRefs, cameraOff, cameraRefs);
    if (unread && !g_halo2SeatFrameCounters.unknownLogged.exchange(1, std::memory_order_relaxed))
        LOG("Halo 2 DRIVE seats: the seat's frame did not read (the rebase's links or the vehicle's up failed their "
            "checks); those updates write nothing and the stick servo keeps the seat (logged once)");
#if HALOMCCVR_DEV_DIAGNOSTICS
    char check[224];
    seat_frame_rt::FrameCheckText(g_halo2SeatFrameCounters, check, sizeof(check));
    if (check[0])
        LOG("Halo 2 DRIVE seats (dev): %s", check);   // frame check: max |R*e(record) - game aim| ...
#endif
}
